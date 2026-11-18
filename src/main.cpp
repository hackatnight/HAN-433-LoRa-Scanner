// H@N 433MHz LoRa Module Scanner - RSSI meter, band sweep and test transmitter.
//
// Two tabs along the bottom of the screen, RX and TX. Menus use the H@N-CYD-GBC look
// (src/han_ui.h).
//
//   RX     Two screens, switched with the header button (or a short BOOT press):
//     METER  Listens on one frequency: big dBm readout, level bar with peak hold,
//            estimated noise floor and a scrolling history graph. Tap to reset the
//            peak.
//     SWEEP  Steps across 430.00-439.95 MHz and draws a spectrum (with a max-hold
//            trace) over a scrolling waterfall - "what's in my air". Tap to clear
//            the max hold. Signals that rise above the noise floor are logged to
//            the SD card (/han433_det.csv) with frequency, peak, duration and time.
//   TX     Sends a carrier or a 1 kHz FM tone on a chosen frequency, power and length.
//
// Everything also reports over USB serial at 115200 baud. Sending "T<unix ms>" over
// serial sets the clock (tools/dev.ps1 does it); without it the log has uptime only.
//
// Board: Elecrow CrowPanel 2.4" "ESP32 Solo Miner LCD" (SKU DHM04624D).
//
// Radio wiring (two-cable hookup, its own SPI bus - VSPI):
//   SCK = IO25, MISO = IO21, MOSI = IO32, NSS = IO22
//   RST and DIO0 are left unconnected (marked NC below).
//
// Screen and touch (built in, sharing HSPI). Same setup as the H@N-CYD-GBC
// firmware for this board:
//   ILI9341V: SCLK 14, MOSI 13, CS 15, DC 2, backlight 27 (no SDO pin)
//   XPT2046:  SCLK 14, DIN 13, DOUT 4 (NOT 12), CS 33, PENIRQ 36, 600 kHz

#include <RadioLib.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <SD.h>
#include <Preferences.h>
#include "soc/spi_reg.h"
#include "soc/gpio_sig_map.h"

// ---------------------------------------------------------------- radio
#define PIN_SCK   25
#define PIN_MISO  21
#define PIN_MOSI  32
#define PIN_NSS   22
#define PIN_BOOT  0      // BOOT button: low while pressed

static const float METER_FREQ_MHZ = 433.0;
static const float METER_BW_KHZ = 125.0;   // RadioLib's default

// The grid everything is measured on: 410.00-524.95 MHz in 50 kHz steps, the range the
// Ra-01's antenna matching is built for.
static const int G0_KHZ = 410000, G_STEP = 50, GBINS = 2300;
static inline int gKHz(int g) { return G0_KHZ + g * G_STEP; }
static const float SWEEP_BW_KHZ = 62.5;   // FSK receiver bandwidth, close to the step
// Time to let the PLL lock and the RSSI settle after each retune. Too short and a
// signal shows up a bin or two late (to the right of where it is). Each sweep
// prints its duration over serial so this can be tuned on the hardware.
static const uint32_t SWEEP_DWELL_US = 200;

SPIClass radioSPI(VSPI);

// Module(cs, dio0, rst, dio1) - dio0/rst/dio1 are unconnected on this wiring.
SX1278 radio = new Module(PIN_NSS, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC,
                          radioSPI, SPISettings(2000000, MSBFIRST, SPI_MODE0));

// ---------------------------------------------------------------- display
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 panel;
  lgfx::Bus_SPI bus;
  lgfx::Light_PWM light;
  lgfx::Touch_XPT2046 touch;

public:
  LGFX() {
    {
      auto c = bus.config();
      c.spi_host = HSPI_HOST;
      c.spi_mode = 0;
      // 80/3 = 26.7 MHz, not 40: 11 x 40 MHz = 440 MHz lands in the sweep, while no
      // harmonic of 26.7 MHz falls inside 430-440 MHz (16 x = 426.7, 17 x = 453.3).
      c.freq_write = 26666667;
      c.freq_read = 16000000;
      c.spi_3wire = false;
      c.use_lock = true;
      // DMA on, as in H@N-CYD-GBC; busDmaIdle() disarms the outgoing DMA link before
      // each touch read so the touch chip still gets its command bytes.
      c.dma_channel = 1;
      c.pin_sclk = 14;
      c.pin_mosi = 13;
      c.pin_miso = 4;     // the screen has no SDO; this line is the touch chip's DOUT
      c.pin_dc = 2;
      bus.config(c);
      panel.setBus(&bus);
    }
    {
      auto c = panel.config();
      c.pin_cs = 15;
      c.pin_rst = -1;
      c.pin_busy = -1;
      c.panel_width = 240;
      c.panel_height = 320;
      c.offset_x = 0;
      c.offset_y = 0;
      c.offset_rotation = 0;
      c.readable = false;
      c.invert = false;
      c.rgb_order = false;
      c.dlen_16bit = false;
      c.bus_shared = true;   // the XPT2046 is on this bus too
      panel.config(c);
    }
    {
      auto c = light.config();
      c.pin_bl = 27;
      c.invert = false;
      c.freq = 40000;        // above hearing; lower rates whine through the speaker amp
      c.pwm_channel = 7;
      light.config(c);
      panel.setLight(&light);
    }
    {
      auto c = touch.config();
      c.x_min = 0; c.x_max = 4095;
      c.y_min = 0; c.y_max = 4095;
      c.pin_int = 36;
      c.offset_rotation = 0;
      c.bus_shared = true;
      c.spi_host = HSPI_HOST;
      c.freq = 600000;       // this panel returns noise if clocked faster
      c.pin_sclk = 14;
      c.pin_mosi = 13;
      c.pin_miso = 4;
      c.pin_cs = 33;
      touch.config(c);
      panel.setTouch(&touch);
    }
    setPanel(&panel);
  }
};

static LGFX lcd;

static const int SCR_W = 240, SCR_H = 320;
#include "han_ui.h"
static uint16_t C_GRID, C_ALERT;   // on top of the theme colors (han_ui.h)

// ---------------------------------------------------------------- shared helpers
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Map dBm in [lo, hi] onto 0 .. size-1.
static int dbmToPx(float dbm, float lo, float hi, int size) {
  float t = clampf((dbm - lo) / (hi - lo), 0, 1);
  return (int)(t * (size - 1) + 0.5f);
}

// Color by margin over the noise floor, so "something is transmitting" looks the
// same wherever the floor sits.
static uint16_t snrColor(float snr) {
  if (snr < 6)  return lcd.color565(40, 120, 150);   // noise
  if (snr < 20) return lcd.color565(60, 220, 90);    // weak signal
  if (snr < 40) return lcd.color565(255, 210, 60);   // strong
  return lcd.color565(255, 70, 90);                  // very strong / close by
}

// ---------------------------------------------------------------- tab bar
// RX | TX along the bottom edge. Full width and 34 px tall, so they still work if the
// touch mapping is off by a few pixels.
static const int TAB_Y = 286, TAB_H = 34;

static void drawTabs(int active) {   // 0 = RX, 1 = TX
  static const char *const names[2] = {"RX", "TX"};
  uiTabs(names, 2, active, TAB_Y, TAB_H);
}

// ---------------------------------------------------------------- touch calibration
// Raw XPT2046 readings (0..4095) -> screen pixels in portrait (rotation 2). The panel
// differs unit to unit, so it is calibrated on the first boot (4 crosshairs) and the
// result is saved to the SD card, and to the ESP32's flash (NVS) as a fallback.
// Hold BOOT for 2 s at any time to restart into calibration.
//
// The SD card is on VSPI pins 18/19/23/5 - the same SPI peripheral the radio uses on
// its own pins 25/21/32/22. The card is mounted once in setup() and stays mounted;
// sdBus() moves the peripheral's signals between the two pin sets through the GPIO
// matrix. While the radio has the bus, the card's SCK/MOSI pins sit idle as plain GPIO
// outputs: left attached, every radio transfer also drove the SD slot's traces, which
// radiated SPI clock harmonics into the 433 band (spurs every few MHz in the sweep).
#define PIN_SD_SCK   18
#define PIN_SD_MISO  19
#define PIN_SD_MOSI  23
#define PIN_SD_CS     5
#define PIN_TP_IRQ   36   // touch pen interrupt: low while pressed
static const char *CAL_FILE = "/han433_touch.cal";

struct TouchCal { bool valid; bool swap; float ax, bx, ay, by; };
static TouchCal cal = {false, false, 0, 0, 0, 0};
static Preferences prefs;

static bool rawToScreen(int rx, int ry, int &sx, int &sy) {
  if (!cal.valid) return false;
  float ux = cal.swap ? ry : rx, uy = cal.swap ? rx : ry;
  sx = (int)lroundf(cal.ax * ux + cal.bx);
  sy = (int)lroundf(cal.ay * uy + cal.by);
  sx = sx < 0 ? 0 : (sx > SCR_W - 1 ? SCR_W - 1 : sx);
  sy = sy < 0 ? 0 : (sy > 319 ? 319 : sy);
  return true;
}

static SPIClass sdSPI(VSPI);

static bool sdOk = false;   // card mounted (in setup)

// true: VSPI SCK/MOSI out on the card's pins, MISO in from the card. false: card pins
// idle, MISO in from the radio. The radio's own SCK/MOSI stay attached (CS high = ignored).
static void sdBus(bool on) {
  if (on) {
    pinMatrixOutAttach(PIN_SD_SCK, VSPICLK_OUT_IDX, false, false);
    pinMatrixOutAttach(PIN_SD_MOSI, VSPID_OUT_IDX, false, false);
  } else {
    pinMatrixOutDetach(PIN_SD_SCK, false, false);
    pinMatrixOutDetach(PIN_SD_MOSI, false, false);
  }
  pinMatrixInAttach(on ? PIN_SD_MISO : PIN_MISO, VSPIQ_IN_IDX, false);
}

static bool sdOpen() {
  sdSPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (!SD.begin(PIN_SD_CS, sdSPI)) { sdSPI.end(); return false; }
  return true;
}

static bool parseCal(const String &line) {
  TouchCal c;
  int swap = 0;
  if (sscanf(line.c_str(), "HAN433CAL1 %d %f %f %f %f", &swap, &c.ax, &c.bx, &c.ay, &c.by) != 5)
    return false;
  c.swap = swap != 0;
  c.valid = true;
  cal = c;
  return true;
}

static String calLine() {
  char buf[96];
  snprintf(buf, sizeof(buf), "HAN433CAL1 %d %.6f %.3f %.6f %.3f",
           cal.swap ? 1 : 0, cal.ax, cal.bx, cal.ay, cal.by);
  return String(buf);
}

// Returns where it was loaded from, or nullptr.
static const char *loadCal() {
  if (sdOk) {
    bool ok = false;
    File f = SD.open(CAL_FILE, FILE_READ);
    if (f) { ok = parseCal(f.readStringUntil('\n')); f.close(); }
    if (ok) return "SD card";
  }
  String s = prefs.getString("cal", "");
  if (s.length() && parseCal(s)) return "flash";
  return nullptr;
}

// Saves to both, so a card pulled later still leaves a working calibration.
static const char *saveCal() {
  String line = calLine();
  prefs.putString("cal", line);
  bool saved = false;
  if (sdOk) {
    File f = SD.open(CAL_FILE, FILE_WRITE);
    if (f) { f.println(line); f.close(); saved = true; }
  }
  Serial.printf("Touch calibration saved (%s): %s\n",
                saved ? "SD card + flash" : "flash only, no SD card", line.c_str());
  return saved ? "SD card" : "flash (no SD card found)";
}

// ---- ported from H@N-CYD-GBC (src/main.cpp), proven on this board -------------

// Let the bus take its next command from the FIFO rather than from DMA.
//
// Reading the touch chip means the SPI peripheral sends the chip's command bytes from
// its FIFO. That only works if the outgoing DMA link is not still armed from the last
// screen push - otherwise the peripheral sends whatever the descriptor points at, the
// chip never hears the command, and the read comes back as nothing. LovyanGFX disarms
// it only on the ESP32-S3 and friends, not on the original ESP32, so do it here.
static inline void busDmaIdle(void) {
  const uint32_t port = HSPI_HOST + 1;
  WRITE_PERI_REG(SPI_DMA_OUT_LINK_REG(port), 0);
  WRITE_PERI_REG(SPI_DMA_CONF_REG(port), 0);
}

static bool readTouchRaw(int &rx, int &ry) {
  lgfx::touch_point_t tp;
  busDmaIdle();
  if (!lcd.getTouchRaw(&tp, 1)) return false;
  rx = tp.x; ry = tp.y;
  return rx >= 0 && rx < 4096 && ry >= 0 && ry < 4096;
}

// Read the touch chip exactly the way LovyanGFX does - same SPI host, frequency and
// 57-byte burst - but return the bytes instead of the verdict, and without waiting for
// the pen interrupt first. Ported from H@N-CYD-GBC's xptReadWith(). All 00 or all FF =
// the chip is not replying; anything else = it is. The burst ends with 0x80 (power
// down, pen interrupt enabled), so this also re-arms a pen interrupt left switched off.
static bool xptRead(uint8_t *d) {
  memset(d, 0, 8);
  d[0] = 0x91; d[2] = 0xB1; d[4] = 0xD1; d[6] = 0xC1; d[56] = 0x80;
  memcpy(&d[8], d, 8); memcpy(&d[16], d, 16); memcpy(&d[32], d, 24);
  busDmaIdle();
  lgfx::spi::beginTransaction(HSPI_HOST, 600000, 0);
  digitalWrite(33, LOW);
  lgfx::spi::readBytes(HSPI_HOST, d, 57);
  digitalWrite(33, HIGH);
  lgfx::spi::endTransaction(HSPI_HOST);
  for (int i = 0; i < 8; i++)
    if (d[i] != 0x00 && d[i] != 0xFF) return true;   // something answered
  return false;
}

// Live readout on the calibration screen (and serial), so a dead touch shows itself:
// the pen interrupt, the library's verdict, and the chip's own bytes.
static void touchDiag() {
  static uint32_t last = 0;
  if (millis() - last < 200) return;
  last = millis();
  int x, y;
  bool pen = digitalRead(PIN_TP_IRQ) == LOW;
  bool got = readTouchRaw(x, y);
  uint8_t d[57];
  bool answered = xptRead(d);
  int bx = (d[5] << 8 | d[6]) >> 3, by = (d[1] << 8 | d[2]) >> 3;
  char buf[64];
  if (got) snprintf(buf, sizeof(buf), "pen %s  raw %d,%d", pen ? "DOWN" : "up", x, y);
  else snprintf(buf, sizeof(buf), "pen %s  chip %s %d,%d", pen ? "DOWN" : "up",
                answered ? "answers" : "silent", bx, by);
  static uint32_t lastBytes = 0;
  if (millis() - lastBytes > 2000 || pen) {
    lastBytes = millis();
    Serial.printf("CAL bytes %02X %02X %02X %02X %02X %02X %02X %02X\n",
                  d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
  }
  lcd.setFont(&fonts::Font2);
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(C_DIM, C_BG);
  lcd.setTextPadding(SCR_W - 70);
  lcd.drawString(buf, SCR_W / 2, 230);
  lcd.setTextPadding(0);
  static uint32_t lastPrint = 0;
  if (pen || got || millis() - lastPrint > 2000) { lastPrint = millis(); Serial.printf("CAL %s\n", buf); }
}

static void drawCross(int x, int y, uint16_t c) {
  lcd.drawFastHLine(x - 12, y, 25, c);
  lcd.drawFastVLine(x, y - 12, 25, c);
  lcd.drawCircle(x, y, 6, c);
}

// Waits for a press, averages the raw reading while held. false = aborted (BOOT).
static bool sampleRaw(float &rx, float &ry) {
  int x, y;
  while (readTouchRaw(x, y)) delay(10);
  while (!readTouchRaw(x, y)) {
    if (digitalRead(PIN_BOOT) == LOW) return false;
    touchDiag();
    delay(10);
  }
  delay(80);  // let the finger settle
  float sx = 0, sy = 0; int n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 400) {
    if (readTouchRaw(x, y)) { sx += x; sy += y; n++; }
    delay(15);
  }
  while (readTouchRaw(x, y)) delay(10);
  if (!n) return false;
  rx = sx / n; ry = sy / n;
  return true;
}

// Works out the calibration from the raw readings at four targets (TL, TR, BL, BR).
// Returns false if the taps don't make a consistent picture.
static bool calSolve(const float rx[4], const float ry[4], const float PX[4], const float PY[4]) {
  // Least-squares line through the 4 points: out = a * in + b, with corr = how well it fits.
  auto fit = [](const float *in, const float *out, float &a, float &b, float &corr) {
    float mi = 0, mo = 0;
    for (int i = 0; i < 4; i++) { mi += in[i] / 4; mo += out[i] / 4; }
    float sio = 0, sii = 0, soo = 0;
    for (int i = 0; i < 4; i++) {
      sio += (in[i] - mi) * (out[i] - mo); sii += (in[i] - mi) * (in[i] - mi); soo += (out[i] - mo) * (out[i] - mo);
    }
    if (sii < 1e-6f || soo < 1e-6f) { a = 1; b = 0; corr = 0; return; }
    a = sio / sii; b = mo - a * mi; corr = sio / sqrtf(sii * soo);
  };
  float axX, bxX, cX, axY, bxY, cY;
  fit(rx, PX, axX, bxX, cX);        // screen x from raw x
  fit(ry, PX, axY, bxY, cY);        // screen x from raw y
  bool swap = fabsf(cY) > fabsf(cX);   // which raw axis follows the screen's x
  const float *U = swap ? ry : rx, *V = swap ? rx : ry;
  float ax, bx, ay, by, c1, c2;
  fit(U, PX, ax, bx, c1);
  fit(V, PY, ay, by, c2);
  // pixels per raw unit: about 0.07 for a resistive panel (it reports 0..4095)
  const float lo = 0.02f, hi = 0.3f;
  bool ok = fabsf(c1) > 0.95f && fabsf(c2) > 0.95f &&
            fabsf(ax) > lo && fabsf(ax) < hi && fabsf(ay) > lo && fabsf(ay) < hi;
  // every point must land within 20 px of its target
  for (int i = 0; ok && i < 4; i++) {
    float ex = ax * U[i] + bx - PX[i], ey = ay * V[i] + by - PY[i];
    if (ex * ex + ey * ey > 20 * 20) ok = false;
  }
  Serial.printf("CAL solve: swap %d  x = %.4f*u + %.1f  y = %.4f*v + %.1f  fit %.3f/%.3f  %s\n",
                swap, ax, bx, ay, by, c1, c2, ok ? "OK" : "REJECTED");
  if (ok) cal = {true, swap, ax, bx, ay, by};
  return ok;
}

// 4-point calibration, as in H@N-CYD-GBC. Portrait rotation 2 is the panel's own
// orientation on this board, so screen and panel coordinates are the same.
// Returns false if skipped with BOOT or the taps don't line up.
static bool calibrateTouch() {
  const int SCR_H = 320;
  // targets in screen coords: TL, TR, BL, BR
  const int TX[4] = { 30, SCR_W - 30, 30, SCR_W - 30 };
  const int TY[4] = { 50, 50, SCR_H - 30, SCR_H - 30 };
  float rx[4], ry[4];
  Serial.println("Touch calibration started");
  lcd.fillScreen(C_BG);
  lcd.setFont(&fonts::Font2);
  lcd.setTextColor(C_TEXT, C_BG);
  lcd.setTextDatum(middle_center);
  lcd.drawString("Touch calibration", SCR_W / 2, SCR_H / 2 - 20);
  lcd.setTextColor(C_DIM, C_BG);
  lcd.drawString("Tap the center of each cross", SCR_W / 2, SCR_H / 2 + 5);
  lcd.drawString("(BOOT button = skip)", SCR_W / 2, SCR_H / 2 + 25);
  for (int i = 0; i < 4; i++) {
    drawCross(TX[i], TY[i], C_ACCENT);
    if (!sampleRaw(rx[i], ry[i])) { Serial.println("Touch calibration skipped"); return false; }
    Serial.printf("CAL point %d: raw %.0f, %.0f\n", i + 1, rx[i], ry[i]);
    drawCross(TX[i], TY[i], C_BG);
    lcd.fillCircle(TX[i], TY[i], 5, lcd.color565(60, 220, 90));
  }
  float PX[4], PY[4];
  for (int i = 0; i < 4; i++) { PX[i] = TX[i]; PY[i] = TY[i]; }
  return calSolve(rx, ry, PX, PY);
}

// Runs calibration (retrying on readings that don't line up) and saves it.
// Called from setup(), before the radio starts (see the SD note above).
static void runCalibration() {
  for (int attempt = 0; attempt < 3; attempt++) {
    if (calibrateTouch()) {
      const char *where = saveCal();
      lcd.fillScreen(C_BG);
      lcd.setFont(&fonts::Font2);
      lcd.setTextDatum(middle_center);
      lcd.setTextColor(lcd.color565(60, 220, 90), C_BG);
      lcd.drawString("Calibration saved", SCR_W / 2, 140);
      lcd.setTextColor(C_DIM, C_BG);
      lcd.drawString(String("to ") + where, SCR_W / 2, 165);
      delay(1500);
      return;
    }
    if (digitalRead(PIN_BOOT) == LOW) break;   // skipped
    lcd.fillScreen(C_BG);
    lcd.setFont(&fonts::Font2);
    lcd.setTextDatum(middle_center);
    lcd.setTextColor(C_ALERT, C_BG);
    lcd.drawString("Didn't line up - try again", SCR_W / 2, 150);
    delay(1500);
  }
  while (digitalRead(PIN_BOOT) == LOW) delay(10);   // let go of BOOT before carrying on
}

// ---------------------------------------------------------------- touch input
struct Tap { bool valid; int x, y; };
static Tap pendingTap = {false, 0, 0};
struct DragEv { bool valid; int dx, y0; };
static DragEv pendingDrag = {false, 0, 0};

// Call often (polls every 30 ms). A press that ends where it started (within DRAG_PX
// sideways) is a tap at the press point; one that moved further is a drag. Both are
// latched when the finger lifts, so a press can still turn into a drag.
static const int DRAG_PX = 15;
static void pollTouch() {
  static bool down = false;
  static int x0, y0, x1;
  static uint32_t last = 0;
  if (millis() - last < 30) return;
  last = millis();
  int rx, ry;
  bool now = readTouchRaw(rx, ry);
  if (now) {
    int sx = -1, sy = -1;
    rawToScreen(rx, ry, sx, sy);   // stays -1,-1 when uncalibrated (never a tab)
    if (!down) { x0 = x1 = sx; y0 = sy; Serial.printf("TOUCH raw %d,%d -> screen %d,%d\n", rx, ry, sx, sy); }
    else x1 = sx;
  } else if (down) {
    if (abs(x1 - x0) > DRAG_PX) { pendingDrag = {true, x1 - x0, y0}; Serial.printf("TOUCH drag %d\n", x1 - x0); }
    else pendingTap = {true, x0, y0};
  }
  down = now;
}
// For han_ui.h's uiList(): the next tap, if there is one.
static bool takeTap(int &x, int &y) {
  pollTouch();
  if (!pendingTap.valid) return false;
  pendingTap.valid = false;
  x = pendingTap.x; y = pendingTap.y;
  return true;
}

// Taps on the tab bar or the header button are navigation; the main loop takes them.
static bool tapIsOnTabs() {
  return pendingTap.valid && (pendingTap.y >= TAB_Y || uiHeaderButtonHit(pendingTap.x, pendingTap.y));
}

// True once for a tap outside the tab bar (consumes it).
static bool tapped() {
  pollTouch();
  if (pendingTap.valid && !tapIsOnTabs()) {
    pendingTap.valid = false;
    return true;
  }
  return false;
}

// BOOT button: 1 = short press (on release), 2 = held 2 s (fires once, while held).
static int bootEvent() {
  static bool down = false, longFired = false;
  static uint32_t t = 0;
  bool now = digitalRead(PIN_BOOT) == LOW;
  if (now == down || millis() - t < 40) {         // no change, or still bouncing
    if (down && !longFired && millis() - t > 2000) { longFired = true; return 2; }
    return 0;
  }
  uint32_t held = millis() - t;
  down = now;
  t = millis();
  if (now) { longFired = false; return 0; }
  return (!longFired && held < 1000) ? 1 : 0;
}

static void fatal(const char *line1, const char *line2) {
  lcd.fillRect(0, 30, SCR_W, 290, C_BG);
  lcd.setFont(&fonts::Font2);
  lcd.setTextDatum(top_center);
  lcd.setTextColor(C_ALERT, C_BG);
  lcd.drawString(line1, SCR_W / 2, 120);
  lcd.setTextColor(C_DIM, C_BG);
  lcd.drawString(line2, SCR_W / 2, 145);
  while (true) delay(1000);
}

// ================================================================ METER
namespace meter {

const float DBM_MIN = -130.0f;   // bottom of the bar and graph
const float DBM_MAX = -10.0f;    // top (a handheld right next to it reads about -20)

const int BAR_X = 10, BAR_Y = 118, BAR_W = 220, BAR_H = 22;
const int GRAPH_X = 10, GRAPH_Y = 196, GRAPH_W = 220, GRAPH_H = 84;

const uint32_t SAMPLE_MS = 5;     // RSSI read interval
const uint32_t FRAME_MS = 100;    // screen update interval
const uint32_t SERIAL_MS = 500;   // serial print interval

LGFX_Sprite barSpr(&lcd);
LGFX_Sprite graphSpr(&lcd);

float history[GRAPH_W];           // one column per frame, ring buffer
int histHead = 0;
float frameMax = DBM_MIN;         // loudest sample since the last frame
float peak = DBM_MIN;
float noiseFloor = NAN;
uint32_t lastSample, lastFrame, lastSerial;

uint16_t levelColor(float dbm) {
  return snrColor(isnan(noiseFloor) ? 0 : dbm - noiseFloor);
}

// Follows quiet readings quickly and loud ones very slowly, so a transmission
// does not drag the floor up.
void updateNoiseFloor(float dbm) {
  if (isnan(noiseFloor)) { noiseFloor = dbm; return; }
  float a = (dbm < noiseFloor) ? 0.2f : 0.002f;
  noiseFloor += a * (dbm - noiseFloor);
}

int graphY(float dbm) { return GRAPH_H - 1 - dbmToPx(dbm, DBM_MIN, DBM_MAX, GRAPH_H); }

void drawStatic() {
  lcd.fillScreen(C_BG);
  char buf[24];
  snprintf(buf, sizeof(buf), "METER %.3f", METER_FREQ_MHZ);
  uiHeader(buf, "Menu");

  lcd.setFont(&fonts::Font2);
  lcd.setTextDatum(top_center);
  lcd.setTextColor(C_DIM, C_BG);
  lcd.drawString("dBm", SCR_W / 2, 92);

  lcd.setFont(&fonts::Font0);
  for (int d = -130; d <= -10; d += 30) {
    int x = BAR_X + dbmToPx(d, DBM_MIN, DBM_MAX, BAR_W);
    lcd.drawFastVLine(x, BAR_Y + BAR_H + 1, 3, C_DIM);
    lcd.setTextDatum(d == -130 ? top_left : d == -10 ? top_right : top_center);
    lcd.drawNumber(d, x, BAR_Y + BAR_H + 5);
  }

  lcd.setTextDatum(bottom_left);
  lcd.drawString("History    (tap screen to reset peak)", GRAPH_X, GRAPH_Y - 3);
  drawTabs(0);
}

void drawFrame(float now) {
  char buf[24];

  // big readout - 7-segment digits, padded so the old value is overdrawn
  lcd.setFont(&fonts::Font7);
  lcd.setTextDatum(top_center);
  lcd.setTextColor(levelColor(now), C_BG);
  lcd.setTextPadding(SCR_W - 4);
  snprintf(buf, sizeof(buf), "%.1f", now);
  lcd.drawString(buf, SCR_W / 2, 38);

  // level bar with peak tick and dashed floor marker
  barSpr.fillScreen(C_GRID);
  barSpr.fillRect(0, 0, dbmToPx(now, DBM_MIN, DBM_MAX, BAR_W) + 1, BAR_H, levelColor(now));
  if (peak > DBM_MIN) {
    int px = dbmToPx(peak, DBM_MIN, DBM_MAX, BAR_W);
    barSpr.fillRect(px > 0 ? px - 1 : 0, 0, 2, BAR_H, C_TEXT);
  }
  int fx = dbmToPx(noiseFloor, DBM_MIN, DBM_MAX, BAR_W);
  for (int y = 0; y < BAR_H; y += 4) barSpr.drawFastVLine(fx, y, 2, C_ACCENT);
  barSpr.pushSprite(BAR_X, BAR_Y);

  // peak / floor text
  lcd.setFont(&fonts::Font2);
  lcd.setTextPadding(110);
  lcd.setTextDatum(top_left);
  lcd.setTextColor(C_TEXT, C_BG);
  snprintf(buf, sizeof(buf), "Peak  %.1f", peak);
  lcd.drawString(buf, 10, 160);
  lcd.setTextDatum(top_right);
  lcd.setTextColor(C_ACCENT, C_BG);
  snprintf(buf, sizeof(buf), "Floor  %.1f", noiseFloor);
  lcd.drawString(buf, SCR_W - 10, 160);
  lcd.setTextPadding(0);

  // history graph, oldest on the left
  graphSpr.fillScreen(C_BG);
  for (int d = -120; d <= -30; d += 30) {
    int y = graphY(d);
    for (int x = 0; x < GRAPH_W; x += 4) graphSpr.drawPixel(x, y, C_GRID);
  }
  graphSpr.drawRect(0, 0, GRAPH_W, GRAPH_H, C_GRID);
  for (int i = 0; i < GRAPH_W; i++) {
    float v = history[(histHead + i) % GRAPH_W];
    if (v <= DBM_MIN) continue;
    int y = graphY(v);
    graphSpr.drawFastVLine(i, y, GRAPH_H - y, levelColor(v));
  }
  int y = graphY(noiseFloor);
  for (int x = 0; x < GRAPH_W; x += 6) graphSpr.drawFastHLine(x, y, 3, C_ACCENT);
  graphSpr.pushSprite(GRAPH_X, GRAPH_Y);
}

void enter() {
  radio.setBandwidth(METER_BW_KHZ);
  radio.setFrequency(METER_FREQ_MHZ);
  radio.startReceive();   // continuous receive, so the channel can be read

  barSpr.setColorDepth(16);
  barSpr.createSprite(BAR_W, BAR_H);
  graphSpr.setColorDepth(16);
  graphSpr.createSprite(GRAPH_W, GRAPH_H);
  for (int i = 0; i < GRAPH_W; i++) history[i] = DBM_MIN;
  histHead = 0;
  frameMax = DBM_MIN;
  peak = DBM_MIN;
  noiseFloor = NAN;
  lastSample = lastFrame = lastSerial = millis();

  drawStatic();
  Serial.printf("METER on %.3f MHz\n", METER_FREQ_MHZ);
}

void leave() {
  barSpr.deleteSprite();
  graphSpr.deleteSprite();
}

void loop() {
  uint32_t ms = millis();

  if (ms - lastSample >= SAMPLE_MS) {
    lastSample = ms;
    // Instantaneous channel RSSI (not a packet). Frames show the loudest sample
    // since the last frame, so short bursts are not missed between redraws.
    float rssi = radio.getRSSI(false);
    if (rssi > frameMax) frameMax = rssi;
    if (rssi > peak) peak = rssi;
    updateNoiseFloor(rssi);
  }

  if (ms - lastFrame >= FRAME_MS && frameMax > DBM_MIN) {
    lastFrame = ms;
    float now = frameMax;
    frameMax = DBM_MIN;
    history[histHead] = now;
    histHead = (histHead + 1) % GRAPH_W;

    if (tapped()) peak = now;
    drawFrame(now);

    if (ms - lastSerial >= SERIAL_MS) {
      lastSerial = ms;
      Serial.printf("RSSI: %.1f dBm\n", now);
    }
  }
}

}  // namespace meter

// ================================================================ RADIO (raw FSK)
// SWEEP and SCAN run the radio in FSK mode: its RSSI is smoothed over just 2 samples
// and the receiver can be relocked onto a new frequency with one register write, so a
// reading is valid ~0.2 ms after a retune. LoRa mode's RSSI is averaged far longer.
// RadioLib's setFrequency()/startReceive() verify every register they touch (~1.4 ms
// per step); a step here is 4 writes and 1 read over raw SPI.
namespace rf {

SPISettings fastSPI(8000000, MSBFIRST, SPI_MODE0);
inline void rw(uint8_t reg, uint8_t val) {
  digitalWrite(PIN_NSS, LOW); radioSPI.transfer16((reg | 0x80) << 8 | val); digitalWrite(PIN_NSS, HIGH);
}
inline uint8_t rr(uint8_t reg) {
  digitalWrite(PIN_NSS, LOW); uint8_t v = radioSPI.transfer16(reg << 8); digitalWrite(PIN_NSS, HIGH);
  return v;
}
const uint8_t RX_CFG = 0x08;             // RegRxConfig: AGC on, AFC off, no trigger
// RegRxBw: mantissa 16, exponent 3 / 2 / 1 = 62.5 / 125 / 250 kHz
const uint8_t BW_62 = 0x03, BW_125 = 0x02, BW_250 = 0x01;

void tune(int khz) {
  uint32_t frf = (uint64_t)khz * 524288 / 32000;   // f x 2^19 / 32 MHz
  radioSPI.beginTransaction(fastSPI);
  rw(0x06, frf >> 16); rw(0x07, frf >> 8); rw(0x08, frf);
  rw(0x0D, RX_CFG | 0x20);               // RestartRxWithPllLock: relock on the new frequency
  radioSPI.endTransaction();
}
int rssi() {                             // FSK RegRssiValue = -2 x dBm
  radioSPI.beginTransaction(fastSPI);
  int v = -(rr(0x11) >> 1);
  radioSPI.endTransaction();
  return v;
}
void setBw(uint8_t v) {
  radioSPI.beginTransaction(fastSPI);
  rw(0x12, v); rw(0x0D, RX_CFG | 0x20);
  radioSPI.endTransaction();
}
void startRx(uint8_t bw) {
  radio.beginFSK(433.0, 4.8, 5.0, SWEEP_BW_KHZ);
  radio.startReceive();
  radioSPI.beginTransaction(fastSPI);
  rw(0x0E, 0x00);                        // RegRssiConfig: no offset, smoothing 2 samples
  rw(0x0D, RX_CFG);
  radioSPI.endTransaction();
  setBw(bw);
}

// Floor (25th percentile) and noise top (95th) of n readings, by counting whole dB.
void levels(const int16_t *v, int n, int &floor, int &top) {
  uint16_t cnt[136] = {0};
  for (int i = 0; i < n; i++) { int k = v[i] + 140; cnt[k < 0 ? 0 : k > 135 ? 135 : k]++; }
  floor = -140;
  for (int k = 0, c = 0; k < 136; k++) if ((c += cnt[k]) >= n / 4) { floor = k - 140; break; }
  top = 135;
  for (int c = 0; top > 0 && (c += cnt[top]) < n / 20 + 1; top--) {}
  top -= 140;
}

}  // namespace rf

// ================================================================ DETECTION LOG
// Everything works on one fixed grid: 410.00-524.95 MHz in 50 kHz steps (GBINS bins) -
// the range the Ra-01's antenna matching is built for (the SX1278 itself tunes
// 137-525 MHz, but below 410 the module hears little). SWEEP and SCAN measure points on
// the grid; detections and learned spurs are kept per grid bin.
//
// SWEEP detections: the 2-sample RSSI is noisy, so thresholds hang off each sweep's
// "noise top", its 95th percentile. A detection = a run of points at least ON_DB over
// the noise top, seen again in the very next sweep. It stays open while a run near where
// it started stays OFF_DB over the noise top - the bin never moves, so it can't wander
// from spike to spike - and ends after MISS sweeps without one. SCAN logs its own (see
// there). The logged SNR is relative to the floor.
//
// Ended detections are printed over serial ("DET ...") and kept in RAM, then appended to
// the SD card in batches between sweeps: when FLUSH_N are waiting or the oldest has
// waited FLUSH_MS. Every row has the boot number (NVS counter) and uptime in ms; once the
// PC has sent "T<unix ms>" over serial, rows also get UTC - including rows still in RAM.
namespace detlog {

const int ON_DB = 6, OFF_DB = 1;     // dB over the noise top to open / to stay open
// The ESP32's 40 MHz crystal lands at 440, 480 and 520 MHz; its skirts come and go, so
// +-400 kHz around each is shown but never detected.
bool xtal(int g) {
  int khz = gKHz(g);
  for (int h = 440000; h <= 520000; h += 40000) if (abs(khz - h) <= 400) return true;
  return false;
}
// Other spurs are learned: busy[g] is how often bin g is over the stay-open level when
// measured, averaged over ~2048 measurements, 65535 = always. A bin counts as quiet - a
// spur, or a steady local emitter - when it or a neighbor is busier than SPUR_BUSY.
// Spurs here flicker at 5-15%; a 2 s burst only lifts busy to ~2%. Learning is slow on
// purpose (a fast learner takes an early real signal for a spur), so the learned bins
// are kept on the SD card as a bit map and loaded at boot.
const int32_t SPUR_BUSY = 65535 * 5 / 100;
uint16_t busy[GBINS];
uint8_t spurMap[(GBINS + 7) / 8];    // learned spur bins, as last saved / loaded
bool spurDirty;                      // the learned bins differ from spurMap
uint32_t spurSaved;                  // millis() of the last save
const uint32_t SPUR_SAVE_MS = 60000;
const char *SPUR_FILE = "/han433_spur.bin";
const int MISS = 2;                  // sweeps a signal may drop out before it ends
const int NEAR = 3;                  // a run within 3 bins (150 kHz) continues a detection
// A transmitter close by overloads the front end and shows sidelobes up to ~1 MHz out,
// 30-40 dB down: a run SIDE_DB under the loudest point within SIDE bins is ignored.
const int SIDE = 20, SIDE_DB = 25;
const uint32_t MAX_MS = 60000;       // a carrier that never stops: one row a minute
const int NOPEN = 8, NBUF = 32, FLUSH_N = 16, NRECENT = 4;
const uint32_t FLUSH_MS = 30000;
const char *FILE_NAME = "/han433_log.csv";
enum { SRC_SWEEP, SRC_SCAN };

// at = bin it opened on (fixed), bin = loudest bin; ok = confirmed (seen 2 sweeps running)
// hits = sweeps it was seen in (SWEEP) or 50 ms windows it was heard in (SCAN)
struct Det { uint32_t t0, t1; int16_t bin, at, peak, snr; uint16_t hits; uint8_t miss; bool ok; uint8_t src; };
Det act[NOPEN], buf[NBUF], recent[NRECENT];
int nOpen, nBuf, nRecent, hot, spurs;   // last sweep: points over the open level / spurs
uint32_t total, dropped, boot, pendSince;
int64_t utcOffMs;                    // unix ms - millis(); 0 = clock not set

bool isSpur(int g) {
  return xtal(g) || busy[g] > SPUR_BUSY || (g && busy[g - 1] > SPUR_BUSY) ||
         (g < GBINS - 1 && busy[g + 1] > SPUR_BUSY);
}

// "2026-11-18T14:03:22.123Z", or "" before the clock is set.
void utc(char *s, uint32_t ms) {
  *s = 0;
  if (!utcOffMs) return;
  int64_t e = utcOffMs + ms;
  time_t sec = e / 1000;
  struct tm t;
  gmtime_r(&sec, &t);
  sprintf(s, "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ", t.tm_year + 1900, t.tm_mon + 1,
          t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec, (int)(e % 1000));
}

void setClock(uint64_t unixMs) {
  utcOffMs = (int64_t)unixMs - millis();
  char s[28];
  utc(s, millis());
  Serial.printf("TIME set %s\n", s);
}

void emit(const Det &d) {
  if (!d.hits) return;
  int khz = gKHz(d.bin);
  Serial.printf("DET %s %d.%03d MHz  %d dBm  +%d dB  %lu ms  %u hits\n", d.src ? "SCAN" : "SWEEP",
                khz / 1000, khz % 1000, d.peak, d.snr, (unsigned long)(d.t1 - d.t0), d.hits);
  total++;
  memmove(recent + 1, recent, sizeof(Det) * (NRECENT - 1));   // newest first
  recent[0] = d;
  if (nRecent < NRECENT) nRecent++;
  if (!sdOk) return;
  if (nBuf == NBUF) { dropped++; return; }
  if (!nBuf) pendSince = millis();
  buf[nBuf++] = d;
}

// Spur learning, for every measured point: v[] dBm at grid bins g[], off = stay-open level.
void learn(const int16_t *v, const uint16_t *g, int n, int off) {
  for (int k = 0; k < n; k++) {
    int i = g[k];
    busy[i] += ((v[k] >= off ? 65535 : 0) - (int32_t)busy[i]) >> 11;
    uint8_t bit = 1 << (i & 7), &m = spurMap[i >> 3];
    if ((busy[i] > SPUR_BUSY) != !!(m & bit)) { m ^= bit; spurDirty = true; }
  }
}

// After each SWEEP: n points v[] (dBm) at grid bins g[] (ascending), the sweep's floor
// and noise top, its start time.
void scan(const int16_t *v, const uint16_t *g, int n, int floor, int top, uint32_t now) {
  const int on = top + ON_DB, off = top + OFF_DB;
  const int step = n > 1 ? max(1, g[1] - g[0]) : 1;        // grid bins between points
  const int near = max(NEAR, step), side = max(1, SIDE / step);
  static bool lit[600];                                     // over off, and not a spur
  bool seen[NOPEN] = {};
  learn(v, g, n, off);
  hot = spurs = 0;
  for (int k = 0; k < n; k++) {
    bool spur = isSpur(g[k]);
    spurs += spur;
    lit[k] = v[k] >= off && !spur;
    hot += lit[k] && v[k] >= on;
  }
  for (int i = 0; i < n;) {
    if (!lit[i]) { i++; continue; }
    int r0 = i, p = i;                                      // run r0..i-1, peak point p
    for (; i < n && lit[i]; i++) if (v[i] > v[p]) p = i;
    int big = v[p];                                         // loudest within +-1 MHz
    for (int j = p < side ? 0 : p - side; j <= p + side && j < n; j++) if (v[j] > big) big = v[j];
    if (v[p] <= big - SIDE_DB) continue;                    // a strong neighbor's sidelobe
    int k = 0;
    while (k < nOpen && (act[k].at < g[r0] - near || act[k].at > g[i - 1] + near)) k++;
    if (k == nOpen) {                                       // a new signal?
      if (v[p] < on || nOpen == NOPEN) continue;
      act[nOpen++] = {now, now, (int16_t)g[p], (int16_t)g[p], -128, 0, 0, 0, false, SRC_SWEEP};
    }
    Det &d = act[k];
    if (!seen[k]) { seen[k] = true; d.t1 = now; d.ok |= ++d.hits >= 2; d.miss = 0; }
    if (v[p] > d.peak) { d.peak = v[p]; d.bin = g[p]; d.snr = v[p] - floor; }
  }
  for (int k = 0; k < nOpen;) {
    Det &d = act[k];
    if (!seen[k] && (!d.ok || ++d.miss > MISS)) {           // gone: log it if confirmed
      if (d.ok) emit(d);
      act[k] = act[--nOpen]; seen[k] = seen[nOpen];
      continue;
    }
    if (now - d.t0 >= MAX_MS) {                             // still there after a minute
      emit(d);
      d.t0 = d.t1 = now; d.hits = 0; d.peak = -128;
    }
    k++;
  }
}

// The view changed: what was open can't be followed any more. Log the confirmed ones.
void closeAll() {
  for (int k = 0; k < nOpen; k++) if (act[k].ok) emit(act[k]);
  nOpen = 0;
}

// Setup, SD card mounted: start from the spurs learned last time. A map from before the
// full-range grid (25 bytes, 430.00-439.95 MHz) is moved to its place on the grid.
void loadSpurs() {
  File f = SD.open(SPUR_FILE, FILE_READ);
  if (!f) return;
  int got = f.read(spurMap, sizeof spurMap);
  f.close();
  if (got == 25) {
    uint8_t old[25];
    memcpy(old, spurMap, 25);
    memset(spurMap, 0, sizeof spurMap);
    const int at = (430000 - G0_KHZ) / G_STEP;
    for (int i = 0; i < 200; i++)
      if (old[i >> 3] & 1 << (i & 7)) spurMap[(at + i) >> 3] |= 1 << ((at + i) & 7);
    spurDirty = true;
  } else if (got != sizeof spurMap) {
    memset(spurMap, 0, sizeof spurMap);
    return;
  }
  int n = 0;
  for (int i = 0; i < GBINS; i++)
    if (spurMap[i >> 3] & 1 << (i & 7)) { busy[i] = SPUR_BUSY * 2; n++; }
  Serial.printf("LOG %d spur bins loaded from SD card\n", n);
}

// Main loop, between sweeps: hands the bus to the card, appends the waiting rows and/or
// saves the spur map, hands the bus back.
void flush() {
  bool rows = nBuf && (nBuf >= FLUSH_N || millis() - pendSince >= FLUSH_MS);
  bool spur = sdOk && spurDirty && millis() - spurSaved >= SPUR_SAVE_MS;
  if (!rows && !spur) return;
  uint32_t t = millis();
  sdBus(true);
  if (spur) {
    File s = SD.open(SPUR_FILE, FILE_WRITE);
    bool ok = s;
    if (ok) { s.write(spurMap, sizeof spurMap); s.close(); }
    Serial.printf("LOG spur map %s\n", ok ? "saved to SD card" : "save FAILED");
    spurDirty = false;
    spurSaved = millis();
  }
  if (!rows) { sdBus(false); return; }
  File f = SD.open(FILE_NAME, FILE_APPEND);
  bool ok = f;
  if (ok) {
    if (!f.size()) f.println("boot,uptime_ms,utc,mode,freq_mhz,peak_dbm,snr_db,dur_ms,hits");
    char s[28];
    for (int k = 0; k < nBuf; k++) {
      const Det &d = buf[k];
      int khz = gKHz(d.bin);
      utc(s, d.t0);
      f.printf("%lu,%lu,%s,%s,%d.%03d,%d,%d,%lu,%u\n", (unsigned long)boot, (unsigned long)d.t0, s,
               d.src ? "SCAN" : "SWEEP", khz / 1000, khz % 1000, d.peak, d.snr,
               (unsigned long)(d.t1 - d.t0), d.hits);
    }
    f.close();
  } else dropped += nBuf;
  sdBus(false);
  Serial.printf("LOG %s %d rows in %lu ms (%lu logged, %lu dropped)\n", ok ? "wrote" : "FAILED",
                nBuf, (unsigned long)(millis() - t), (unsigned long)total, (unsigned long)dropped);
  nBuf = 0;
}

// "T<unix ms>" + newline from the PC sets the clock.
void pollSerial() {
  static char line[24];
  static uint8_t n;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') { if (n < sizeof line - 1) line[n++] = c; continue; }
    line[n] = 0;
    if (n > 10 && line[0] == 'T') setClock(strtoull(line + 1, nullptr, 10));
    n = 0;
  }
}

}  // namespace detlog

// ================================================================ SWEEP
// A window onto the grid: NPX pixels from grid bin vStart, bpp grid bins per pixel
// (zoom 1 / 2 / 3 / 5 / 12 = 10 / 20 / 30 / 50 / 115 MHz wide). The receiver widens with
// the zoom (62.5 / 125 / 250 kHz); where a pixel spans more than the receiver hears it is
// measured at several points and shows the loudest. Buttons under the axis: < > move a
// screen width, - + zoom around the center. Drag to pan finely, tap the spectrum to clear
// the max hold. The header Menu has band presets.
namespace sweep {

const int NPX = 200;
const int ZOOMS[] = {1, 2, 3, 5, 12};
const int NZOOM = 5;
int vStart = (430000 - G0_KHZ) / G_STEP, bpp = 1;   // starts on 430-440 MHz

// Spectrum: -130..-10 dBm into 90 px (3/4 px per dB), below the 32 px header.
const int X0 = 20, SPEC_Y = 52, SPEC_H = 90, AXIS_Y = SPEC_Y + SPEC_H + 3;

// Waterfall = ILI9341 hardware vertical scroll. No pixel is ever copied: each sweep
// writes one new row and moves the controller's scroll pointer (VSCRSADD). Scrolling
// works on frame-memory rows, and with rotation 2 (MY set) screen row y is memory row
// 319 - y. So, in memory rows:
//   TFA = the tabs strip at the screen bottom, VSA = the waterfall, BFA = everything above.
// Under the axis labels: [<] [-] span [+] [>] buttons.
const int BTN_Y = AXIS_Y + 12, BTN_H = 26;
const int WF_Y = BTN_Y + BTN_H + 3, WF_H = 284 - WF_Y;
const int TFA = 320 - WF_Y - WF_H, VSA = WF_H, BFA = WF_Y;
// Rotation 2 sets MX|MY|ML|MH. ML/MH only reverse the panel's refresh order, but ML also
// flips where the scroll areas sit, so drop both while sweeping: same picture, one layout.
const uint8_t MADCTL_SWEEP = 0xC8, MADCTL_ROT2 = 0xDC;   // MY|MX|BGR (+ML|MH)

int16_t px[NPX], hold[NPX];
int16_t mv[600];                         // this sweep's points: dBm ...
uint16_t mg[600];                        // ... at grid bins
uint16_t pal[64];
int floorDbm = -120, vsp = TFA, wfRow;   // wfRow = memory row being drawn this sweep
uint32_t sweepCount, shownTotal;

inline int specH(int dbm) { int v = (dbm + 130) * 3 / 4; return v < 0 ? 0 : v > SPEC_H ? SPEC_H : v; }
int npx() { return min(NPX, (GBINS - vStart + bpp - 1) / bpp); }
int subs() { return (bpp + 4) / 5; }      // points per pixel: each covers <= 250 kHz
int pointBin(int x, int s) { return min(GBINS - 1, vStart + x * bpp + s * bpp / subs() + bpp / subs() / 2); }
uint8_t bw() { return bpp == 1 ? rf::BW_62 : bpp == 2 ? rf::BW_125 : rf::BW_250; }

// ---- display, raw ILI9341 commands
void lcdCmd(uint8_t c, const uint16_t *d, int n) {
  lcd.startWrite(); lcd.writeCommand(c);
  for (int k = 0; k < n; k++) lcd.writeData16(d[k]);
  lcd.endWrite();
}
void lcdMadctl(uint8_t v) { lcd.startWrite(); lcd.writeCommand(0x36); lcd.writeData(v); lcd.endWrite(); }
void scrollTo(int p) { uint16_t d = p; lcdCmd(0x37, &d, 1); }

// Waterfall palette: black -> blue -> cyan -> yellow -> red over 0..40 dB above floor.
void buildPalette() {
  const uint8_t s[5][3] = {{0, 0, 0}, {10, 40, 160}, {40, 220, 230}, {255, 220, 40}, {255, 50, 70}};
  for (int i = 0; i < 64; i++) {
    int k = i * 4 / 64, f = i * 4 % 64 * 4;   // segment, 0..255 within it
    pal[i] = lcd.color565(s[k][0] + (s[k + 1][0] - s[k][0]) * f / 256,
                          s[k][1] + (s[k + 1][1] - s[k][1]) * f / 256,
                          s[k][2] + (s[k + 1][2] - s[k][2]) * f / 256);
  }
}

// Draw pixel x straight to the screen: spectrum column, max-hold dot, floor dash and
// this sweep's waterfall pixel. ~4 tiny writes, done while the next point settles.
void drawPx(int i) {
  int x = X0 + i, h = specH(px[i]), y0 = SPEC_Y + SPEC_H;
  lcd.drawFastVLine(x, SPEC_Y, SPEC_H - h, C_BG);
  lcd.drawFastVLine(x, y0 - h, h, snrColor(px[i] - floorDbm));
  lcd.drawPixel(x, y0 - 1 - specH(hold[i]), C_TEXT);
  if (i % 6 < 3) lcd.drawPixel(x, y0 - 1 - specH(floorDbm), C_ACCENT);
  int k = (px[i] - floorDbm) * 63 / 40;
  lcd.drawPixel(x, 319 - wfRow, pal[k < 0 ? 0 : k > 63 ? 63 : k]);
}

// MHz labels for the current view, every 50 px.
// Button row: x, width of < - + > and whether each can do anything in this view.
const int BX[4] = {4, 48, 152, 196}, BW = 40;
bool btnOn(int i) {
  int z = 0;
  while (z < NZOOM && ZOOMS[z] != bpp) z++;
  return i == 0 ? vStart > 0 : i == 1 ? z < NZOOM - 1 : i == 2 ? z > 0 : vStart + NPX * bpp < GBINS;
}
void drawButtons() {
  static const char *const lab[4] = {"<", "-", "+", ">"};
  lcd.fillRect(0, BTN_Y, SCR_W, BTN_H, C_BG);
  for (int i = 0; i < 4; i++) {
    bool on = btnOn(i);
    uiButton(BX[i], BTN_Y, BW, BTN_H, lab[i], C_PANEL, on ? C_ACCENT : C_PANEL, on ? C_TEXT : C_DIM);
  }
  char b[12];
  int khz = min(NPX * bpp, GBINS) * G_STEP;
  snprintf(b, sizeof b, "%d MHz", khz / 1000);
  lcd.setFont(&fonts::Font2);
  lcd.setTextColor(C_TEXT, C_BG);
  lcd.setTextDatum(middle_center);
  lcd.drawString(b, SCR_W / 2, BTN_Y + BTN_H / 2);
  lcd.setTextDatum(top_left);
}

void drawAxis() {
  lcd.fillRect(0, AXIS_Y - 2, SCR_W, BTN_Y - AXIS_Y + 2, C_BG);
  lcd.setFont(&fonts::Font0);
  lcd.setTextColor(C_DIM, C_BG);
  lcd.setTextDatum(top_center);
  char b[8];
  const int n = npx();
  for (int x = 0; x < n + 50; x += 50) {
    int xx = x < n ? x : n - 1, khz = gKHz(vStart + xx * bpp);
    if (x >= n && n - 1 - (x - 50) < 30) break;     // too close to the previous label
    lcd.drawFastVLine(X0 + xx, AXIS_Y - 2, 3, C_DIM);
    if (bpp == 1) snprintf(b, sizeof b, "%d.%d", khz / 1000, khz % 1000 / 100);
    else snprintf(b, sizeof b, "%d", khz / 1000);
    lcd.drawString(b, X0 + xx, AXIS_Y + 2);
  }
}

// Header: the mode and the detections logged so far; its button opens the RX menu.
void title() {
  shownTotal = detlog::total;
  char s[32];
  snprintf(s, sizeof s, sdOk ? "SWEEP  %lu logged" : "SWEEP  %lu, no SD", (unsigned long)shownTotal);
  uiHeader(s, "Menu");
}

// Moves the window (clamped to the grid) and starts its picture afresh.
void setView(int start, int zoom) {
  bpp = zoom;
  vStart = constrain(start, 0, max(0, GBINS - NPX * bpp));
  detlog::closeAll();
  for (int i = 0; i < NPX; i++) hold[i] = px[i] = -130;
  rf::setBw(bw());
  lcd.fillRect(X0, SPEC_Y, NPX, SPEC_H, C_BG);
  lcd.fillRect(X0, WF_Y, NPX, WF_H, C_BG);        // every waterfall row, wherever it scrolled
  drawAxis();
  drawButtons();
  int a = gKHz(vStart), b = gKHz(vStart + npx() * bpp);
  Serial.printf("SWEEP view %d.%02d-%d.%02d MHz, %d kHz/px\n", a / 1000, a % 1000 / 10,
                b / 1000, b % 1000 / 10, bpp * G_STEP);
}
// A band from the menu: the closest zoom that holds it, centered.
void setBand(int loKHz, int hiKHz) {
  int z = ZOOMS[NZOOM - 1];
  for (int i = 0; i < NZOOM; i++)
    if (NPX * ZOOMS[i] * G_STEP >= hiKHz - loKHz) { z = ZOOMS[i]; break; }
  bpp = z;
  vStart = ((loKHz + hiKHz) / 2 - G0_KHZ) / G_STEP - NPX * z / 2;   // setView clamps it on entry
}
// The button row: < > a screen width lower / higher, - + one zoom step around the center.
void button(int i) {
  if (!btnOn(i)) return;
  int z = 0;
  while (z < NZOOM && ZOOMS[z] != bpp) z++;
  int span = NPX * bpp, center = vStart + min(span, GBINS) / 2;
  if (i == 0) setView(vStart - span, bpp);
  else if (i == 3) setView(vStart + span, bpp);
  else {
    int nb = ZOOMS[i == 1 ? z + 1 : z - 1];
    setView(center - NPX * nb / 2, nb);
  }
}

void enter() {
  rf::startRx(bw());
  sweepCount = 0;
  lcd.fillScreen(C_BG);
  title();
  lcd.setFont(&fonts::Font0);
  lcd.setTextColor(C_DIM, C_BG);
  lcd.setTextDatum(middle_right);                // dBm labels
  for (int d = -110; d <= -30; d += 40) lcd.drawNumber(d, X0 - 2, SPEC_Y + SPEC_H - 1 - specH(d));
  drawTabs(0);                                   // SWEEP is under RX

  uint16_t def[3] = {TFA, VSA, BFA};
  lcdMadctl(MADCTL_SWEEP);
  lcdCmd(0x33, def, 3);                          // VSCRDEF
  vsp = TFA;
  scrollTo(vsp);
  setView(vStart, bpp);
}

void leave() {
  detlog::closeAll();
  scrollTo(0);
  lcd.startWrite(); lcd.writeCommand(0x13); lcd.endWrite();   // NORON: back to normal mode
  lcdMadctl(MADCTL_ROT2);
  radio.begin(METER_FREQ_MHZ);                                 // back to LoRa for the meter
}

// One sweep per call. Each pixel's drawing happens while the next point settles, so the
// screen updates continuously. Returns early on a tap, a drag or BOOT.
void loop() {
  if (pendingDrag.valid) {                       // drag: the picture follows the finger
    pendingDrag.valid = false;
    if (pendingDrag.y0 >= SPEC_Y && pendingDrag.y0 < TAB_Y && bpp < ZOOMS[NZOOM - 1])
      setView(vStart - pendingDrag.dx * bpp, bpp);
    return;
  }
  if (tapped()) {
    int x = pendingTap.x, y = pendingTap.y;
    if (y >= BTN_Y - 4 && y < BTN_Y + BTN_H + 4) {
      for (int i = 0; i < 4; i++) if (x >= BX[i] - 2 && x < BX[i] + BW + 2) button(i);
    } else if (y >= SPEC_Y && y < BTN_Y) for (int i = 0; i < NPX; i++) hold[i] = px[i];
    return;
  }
  uint32_t t0 = millis();
  // New waterfall row: advance the scroll pointer, then the row that just wrapped to
  // the top (the oldest one) is the one to overwrite. Blank it first.
  wfRow = vsp;
  vsp = TFA + (vsp - TFA + 1) % VSA;
  lcd.drawFastHLine(X0, 319 - wfRow, NPX, C_BG);
  scrollTo(vsp);
  const int nx = npx(), ns = subs();
  // A wider receiver settles faster: 200 / 150 / 120 us at 62.5 / 125 / 250 kHz.
  const uint32_t dwell = bpp == 1 ? SWEEP_DWELL_US : bpp == 2 ? 150 : 120;
  rf::tune(gKHz(pointBin(0, 0)));                // the jump back to the start: 1 ms for the PLL
  delayMicroseconds(1000);

  int n = 0;
  for (int x = 0; x < nx; x++) {
    int best = -200;
    for (int s = 0; s < ns; s++) {
      uint32_t t = micros();
      int g = pointBin(x, s);
      rf::tune(gKHz(g));
      if (x && !s) drawPx(x - 1);
      while (micros() - t < dwell) {}
      int v = rf::rssi();
      mv[n] = v; mg[n++] = g;
      if (v > best) best = v;
    }
    px[x] = best;
    if (best > hold[x]) hold[x] = best;
    if (x % 25 == 0) {
      pollTouch();
      if (pendingTap.valid || pendingDrag.valid || digitalRead(PIN_BOOT) == LOW) return;
    }
  }
  drawPx(nx - 1);
  uint32_t dt = millis() - t0;

  int top;
  rf::levels(mv, n, floorDbm, top);
  detlog::scan(mv, mg, n, floorDbm, top, t0);
  if (detlog::total != shownTotal) title();

  int peak = 0;
  for (int k = 1; k < n; k++) if (mv[k] > mv[peak]) peak = k;
  char b[40];
  int snr = mv[peak] - floorDbm, khz = gKHz(mg[peak]);
  lcd.setFont(&fonts::Font2);
  lcd.setTextDatum(top_left);
  lcd.setTextPadding(SCR_W - 16);
  lcd.setTextColor(snr >= 6 ? snrColor(snr) : C_DIM, C_BG);
  snprintf(b, sizeof b, "Peak %d.%02d MHz  %d dBm", khz / 1000, khz % 1000 / 10, mv[peak]);
  lcd.drawString(b, 8, UI_HDR_H + 3);
  lcd.setTextPadding(0);

  if (++sweepCount % 10 == 0)
    Serial.printf("SWEEP %lu ms  floor %d  top %d dBm  hot %d  spurs %d  peak %s\n",
                  (unsigned long)dt, floorDbm, top, detlog::hot, detlog::spurs, b + 5);
}

}  // namespace sweep

// ================================================================ SCAN
// A scanner over the whole grid. Scanning: full-range sweeps (250 kHz receiver, a point
// every 200 kHz, ~7 a second) with a small spectrum. A point ON_DB over the sweep's
// noise top that isn't a spur or locked out is checked up close: 62.5 kHz receiver,
// every 50 kHz bin within +-300 kHz, plus the local floor from points 0.6-1.4 MHz either
// side. If its best bin is LOCK_DB over that floor, the scanner locks on: it parks there
// and samples ~1000 times a second. Each 50 ms window with a sample over the line counts
// as heard. After HANG_MS without one it logs the signal (start, frequency, peak,
// duration) and scans on, skipping that frequency for SKIP_MS. After MAX_LOCK_MS it logs,
// locks the frequency out for LOCKOUT_MS and moves on; a tap does the same (Skip).
namespace scan {

const int LOCK_DB = 10;
const uint32_t HANG_MS = 2000, MAX_LOCK_MS = 60000, SKIP_MS = 10000, LOCKOUT_MS = 300000;
const int BPP = 12, NX = (GBINS + BPP - 1) / BPP, SUBS = 3;   // full range, 192 px
const int X0 = 24, SP_Y = 150, SP_H = 44, AX_Y = SP_Y + SP_H + 2, HIT_Y = 212;

bool locked;
int lockG, lockFloor, peak, level;
uint32_t tStart, tLast, tShown;
uint16_t hits;
int16_t mv[NX * SUBS];
uint16_t mg[NX * SUBS];
struct Skip { int16_t g; uint32_t until; } skips[8];
uint32_t shownTotal;

bool skipped(int g) {
  for (auto &s : skips) if (s.until && (int32_t)(s.until - millis()) > 0 && abs(g - s.g) <= 2) return true;
  return false;
}
void skip(int g, uint32_t ms) {
  int o = 0;                                     // the slot that frees up first
  for (int i = 1; i < 8; i++) if ((int32_t)(skips[i].until - skips[o].until) < 0) o = i;
  skips[o] = {(int16_t)g, millis() + ms};
}

void title() {
  shownTotal = detlog::total;
  char s[32];
  snprintf(s, sizeof s, sdOk ? "SCAN  %lu logged" : "SCAN  %lu, no SD", (unsigned long)shownTotal);
  uiHeader(s, "Menu");
}

void status(const char *s, uint16_t c) {
  lcd.setFont(&fonts::Font2);
  lcd.setTextDatum(top_left);
  lcd.setTextColor(c, C_BG);
  lcd.setTextPadding(SCR_W - 16);
  lcd.drawString(s, 8, UI_HDR_H + 4);
  lcd.setTextPadding(0);
}

void bigFreq(int khz, uint16_t c) {
  char b[12];
  if (khz) snprintf(b, sizeof b, "%d.%03d", khz / 1000, khz % 1000);
  else snprintf(b, sizeof b, "---.---");
  lcd.setFont(&fonts::Font7);
  lcd.setTextDatum(top_center);
  lcd.setTextColor(c, C_BG);
  lcd.setTextPadding(SCR_W - 4);
  lcd.drawString(b, SCR_W / 2, 56);
  lcd.setTextPadding(0);
  lcd.setTextDatum(top_left);
}

// Level bar: the signal against the locked frequency's floor, dBm on the right.
void bar(int dbm) {
  const int x = 8, y = 112, w = 170, h = 14;
  lcd.fillRect(x, y, w, h, C_GRID);
  if (dbm > -140) {
    int f = constrain((dbm + 130) * w / 120, 0, w);
    lcd.fillRect(x, y, f, h, snrColor(dbm - lockFloor));
  }
  char b[12];
  snprintf(b, sizeof b, dbm > -140 ? "%d dBm" : "", dbm);
  lcd.setFont(&fonts::Font2);
  lcd.setTextDatum(middle_right);
  lcd.setTextColor(C_TEXT, C_BG);
  lcd.setTextPadding(56);
  lcd.drawString(b, SCR_W - 6, y + h / 2);
  lcd.setTextPadding(0);
  lcd.setTextDatum(top_left);
}

// The last few detections (SWEEP and SCAN), newest first: time, MHz, dBm, seconds.
void drawHits() {
  lcd.fillRect(0, HIT_Y, SCR_W, TAB_Y - HIT_Y, C_BG);
  lcd.setFont(&fonts::Font2);
  lcd.setTextDatum(top_left);
  for (int i = 0; i < detlog::nRecent; i++) {
    const detlog::Det &d = detlog::recent[i];
    char t[28], b[48];
    detlog::utc(t, d.t0);
    if (*t) memmove(t, t + 11, 9);               // HH:MM:SS of the UTC stamp
    else snprintf(t, sizeof t, "+%lus", (unsigned long)(d.t0 / 1000));
    int khz = gKHz(d.bin);
    snprintf(b, sizeof b, "%.8s  %d.%03d  %d dBm  %lu.%lus", t, khz / 1000, khz % 1000, d.peak,
             (unsigned long)((d.t1 - d.t0) / 1000), (unsigned long)((d.t1 - d.t0) % 1000 / 100));
    lcd.setTextColor(i ? C_DIM : C_TEXT, C_BG);
    lcd.drawString(b, 8, HIT_Y + i * 18);
  }
}

void drawStatic() {
  lcd.fillScreen(C_BG);
  title();
  lcd.setFont(&fonts::Font0);
  lcd.setTextColor(C_DIM, C_BG);
  lcd.setTextDatum(top_center);
  for (int mhz = 410; mhz <= 520; mhz += 30) {
    int x = X0 + (mhz * 1000 - G0_KHZ) / G_STEP / BPP;
    lcd.drawFastVLine(x, AX_Y, 3, C_DIM);
    lcd.drawNumber(mhz, x, AX_Y + 4);
  }
  bigFreq(0, C_DIM);
  bar(-200);
  drawHits();
  drawTabs(0);                                   // SCAN is under RX
}

void startScan() {
  locked = false;
  rf::setBw(rf::BW_250);
  status("Scanning 410 - 525 MHz", C_DIM);
  bigFreq(0, C_DIM);
  bar(-200);
}

// Up-close check of grid bin g0: local floor and the loudest bin near it.
bool check(int g0) {
  rf::setBw(rf::BW_62);
  int16_t ref[16];
  for (int j = 0; j < 16; j++) {
    int g = constrain(g0 + (j < 8 ? -12 - j * 2 : 12 + (j - 8) * 2), 0, GBINS - 1);
    rf::tune(gKHz(g));
    delayMicroseconds(j ? 250 : 1000);
    ref[j] = rf::rssi();
  }
  int fl, tp;
  rf::levels(ref, 16, fl, tp);
  int best = -200, bestG = g0;
  for (int g = max(0, g0 - 6); g <= min(GBINS - 1, g0 + 6); g++) {
    if (detlog::isSpur(g)) continue;
    rf::tune(gKHz(g));
    delayMicroseconds(g == max(0, g0 - 6) ? 1000 : 250);
    int v = rf::rssi();
    if (v > best) { best = v; bestG = g; }
  }
  if (best < fl + LOCK_DB) { rf::setBw(rf::BW_250); return false; }
  lockG = bestG; lockFloor = fl; peak = level = best;
  rf::tune(gKHz(lockG));
  return true;
}

void endLock(bool longOut) {
  detlog::Det d = {tStart, tLast, (int16_t)lockG, (int16_t)lockG, (int16_t)peak,
                   (int16_t)(peak - lockFloor), hits, 0, true, detlog::SRC_SCAN};
  detlog::emit(d);
  skip(lockG, longOut ? LOCKOUT_MS : SKIP_MS);
  if (detlog::total != shownTotal) title();
  drawHits();
  startScan();
}

void enter() {
  rf::startRx(rf::BW_250);
  drawStatic();
  startScan();
  Serial.println("SCAN 410.00-524.95 MHz");
}

void leave() {
  if (locked) endLock(false);
  radio.begin(METER_FREQ_MHZ);                   // back to LoRa for the meter
}

void sweepOnce() {
  rf::tune(gKHz(SUBS / 2 * BPP / SUBS));
  delayMicroseconds(1000);
  int n = 0;
  for (int x = 0; x < NX; x++) {
    int best = -200;
    for (int s = 0; s < SUBS; s++) {
      int g = min(GBINS - 1, x * BPP + s * BPP / SUBS + BPP / SUBS / 2);
      uint32_t t = micros();
      rf::tune(gKHz(g));
      if (x && !s) {                             // draw the previous pixel while this settles
        int v = mv[n - 1];
        for (int k = 2; k <= SUBS; k++) v = max(v, (int)mv[n - k]);
        int h = constrain((v + 130) * SP_H / 120, 0, SP_H);
        lcd.drawFastVLine(X0 + x - 1, SP_Y, SP_H - h, C_BG);
        lcd.drawFastVLine(X0 + x - 1, SP_Y + SP_H - h, h, C_ACCENT);
      }
      while (micros() - t < SWEEP_DWELL_US) {}
      mv[n] = rf::rssi(); mg[n++] = g;
      if (mv[n - 1] > best) best = mv[n - 1];
    }
    if (x % 32 == 0) {
      pollTouch();
      if (pendingTap.valid || digitalRead(PIN_BOOT) == LOW) return;
    }
  }
  int fl, tp;
  rf::levels(mv, n, fl, tp);
  detlog::learn(mv, mg, n, tp + detlog::OFF_DB);
  int c = -1;                                    // the loudest candidate
  for (int k = 0; k < n; k++)
    if (mv[k] >= tp + detlog::ON_DB && (c < 0 || mv[k] > mv[c]) && !detlog::isSpur(mg[k]) && !skipped(mg[k]))
      c = k;
  if (c >= 0 && check(mg[c])) {
    locked = true;
    tStart = tLast = tShown = millis();
    hits = 1;
    int khz = gKHz(lockG);
    Serial.printf("SCAN lock %d.%03d MHz  %d dBm (floor %d)\n", khz / 1000, khz % 1000, peak, lockFloor);
    bigFreq(khz, C_TEXT);
    status("LOCKED   tap to skip", C_ALERT);
  }
}

// Locked: one 50 ms window of samples per call.
void listen() {
  uint32_t t0 = millis();
  int best = -200;
  while (millis() - t0 < 50) {
    int v = rf::rssi();
    if (v > best) best = v;
    delayMicroseconds(900);
  }
  uint32_t now = millis();
  if (best >= lockFloor + LOCK_DB) { tLast = now; hits++; if (best > peak) peak = best; }
  bar(best);
  char s[40];
  snprintf(s, sizeof s, "LOCKED  %lu.%lus  quiet %lu.%lus", (unsigned long)((now - tStart) / 1000),
           (unsigned long)((now - tStart) % 1000 / 100), (unsigned long)((now - tLast) / 1000),
           (unsigned long)((now - tLast) % 1000 / 100));
  status(s, now - tLast < 200 ? C_ALERT : C_TEXT);
  if (now - tLast > HANG_MS) endLock(false);
  else if (now - tStart > MAX_LOCK_MS) endLock(true);
}

void loop() {
  if (tapped()) {                                // tap while locked: skip this one
    if (locked) { Serial.println("SCAN skip"); endLock(true); }
    return;
  }
  pendingDrag.valid = false;
  if (locked) listen(); else sweepOnce();
}

}  // namespace scan

// ================================================================ TX
// Test transmitter: a plain carrier, a 1 kHz FM tone, or a 5 s chiptune. The SX1278 has
// no audio input, so the audio is made in software: the radio sends a carrier (FSK
// continuous mode, zero deviation) and the firmware keeps rewriting its frequency
// registers. The tone follows a 16-step sine (+-3 kHz, 16 writes per cycle); the
// chiptune flips between two frequencies (+-2.5 kHz) at the note's pitch - square-wave
// FM, which an FM receiver plays as an 8-bit pulse voice. Chords are arpeggios: the
// notes of the chord taken in turn every ~33 ms, the old trick for one voice.
// Settings are kept in NVS. A tap or BOOT stops a transmission early.
namespace tx {

const int POWERS[] = {2, 5, 10, 14, 17};          // dBm out of PA_BOOST
const int LENGTHS_MS[] = {500, 1000, 2000, 5000};
const int F_MIN = 430000, F_MAX = 439995;         // kHz
enum { SIG_CARRIER, SIG_TONE, SIG_SONG, NUM_SIG };
int fKHz = 433500, powIdx = 0, sig = SIG_TONE, lenIdx = 1, sel = -1;

// 3 kHz deviation in FRF steps (32 MHz / 2^19 = 61.035 Hz): 49 x sin(k x 22.5 deg)
const int8_t SIN16[16] = {0, 19, 35, 45, 49, 45, 35, 19, 0, -19, -35, -45, -49, -45, -35, -19};
const int SONG_DEV = 41;                          // +-2.5 kHz for the chiptune

// ---- the chiptune: an original 5 s tune, 150 BPM, one step = a 16th note = 100 ms.
// note = MIDI number (60 = middle C, 0 = rest), len in steps, arp: 0 none, 1 major, 2 minor
struct Step { uint8_t note, len, arp; };
const Step SONG[] = {
  {72, 2, 0}, {76, 2, 0}, {79, 2, 0}, {84, 2, 0}, {83, 1, 0}, {79, 1, 0}, {76, 2, 0}, {77, 2, 0}, {81, 2, 0},
  {79, 3, 0}, {76, 1, 0}, {74, 2, 0}, {72, 2, 0}, {74, 1, 0}, {76, 1, 0}, {79, 2, 0}, {81, 4, 2},
  {60, 4, 1}, {65, 4, 1}, {67, 4, 1},
  {72, 6, 1},
};
const int STEP_MS = 100, GAP_MS = 12;             // a short gap ends each note: staccato
// Half a period in us for C4..B4; each octave up halves it.
const uint16_t HALF_US4[12] = {1911, 1804, 1703, 1607, 1517, 1432, 1351, 1276, 1204, 1136, 1073, 1012};
uint32_t halfUs(int midi) { return HALF_US4[midi % 12] >> (midi / 12 - 5) ; }   // midi >= 60

const int ROW_X = 6, ROW_W = SCR_W - 12, ROW_Y = UI_HDR_H + 6, ROW_H = 36, ROW_P = 42;
const int TX_Y = ROW_Y + 4 * ROW_P + 4, TX_H = 50;      // TRANSMIT button
const int STATUS_Y = TX_Y + TX_H + 6;
const char *const LABELS[4] = {"Frequency", "Power", "Signal", "Length"};

void load() {
  fKHz = constrain((int)prefs.getUInt("txf", fKHz), F_MIN, F_MAX);
  powIdx = prefs.getUChar("txp", powIdx) % 5;
  sig = prefs.getUChar("txs", sig) % NUM_SIG;
  lenIdx = prefs.getUChar("txl", lenIdx) % 4;
}
void save() {
  prefs.putUInt("txf", fKHz); prefs.putUChar("txp", powIdx);
  prefs.putUChar("txs", sig); prefs.putUChar("txl", lenIdx);
}

int songMs() { int n = 0; for (auto &s : SONG) n += s.len; return n * STEP_MS; }

void value(int r, char *s, int n) {
  static const char *const SIGS[NUM_SIG] = {"Carrier", "Tone 1 kHz", "Chiptune"};
  switch (r) {
    case 0: snprintf(s, n, "%d.%03d MHz", fKHz / 1000, fKHz % 1000); break;
    case 1: snprintf(s, n, "%d dBm", POWERS[powIdx]); break;
    case 2: snprintf(s, n, "%s", SIGS[sig]); break;
    default:
      if (sig == SIG_SONG) snprintf(s, n, "%d.%d s (song)", songMs() / 1000, songMs() % 1000 / 100);
      else snprintf(s, n, "%d.%d s", LENGTHS_MS[lenIdx] / 1000, LENGTHS_MS[lenIdx] % 1000 / 100);
      break;
  }
}
void drawRow(int r) {
  char v[20];
  value(r, v, sizeof v);
  int y = ROW_Y + r * ROW_P;
  uiRow(ROW_X, y, ROW_W, ROW_H, LABELS[r], v, r == sel);
  if (r == sel) glowSet(ROW_X, y, ROW_W, ROW_H, C_HILITE);
}
void drawButton(bool on) {
  uiButton(ROW_X, TX_Y, ROW_W, TX_H, on ? "TRANSMITTING" : "TRANSMIT",
           on ? C_ALERT : C_PANEL, C_ALERT, C_TEXT);
}
void status(const char *s, uint16_t c) {
  lcd.setFont(&fonts::Font2);
  lcd.setTextDatum(top_center);
  lcd.setTextColor(c, C_BG);
  lcd.setTextPadding(SCR_W - 8);
  lcd.drawString(s, SCR_W / 2, STATUS_Y);
  lcd.setTextPadding(0);
  lcd.setTextDatum(top_left);
}
void drawAll() {
  glowOff();
  lcd.fillScreen(C_BG);
  uiHeader("TX", "Menu");
  for (int r = 0; r < 4; r++) drawRow(r);
  drawButton(false);
  status("Tap a row to change it", C_DIM);
  drawTabs(1);
}

// Frequency picker, GBC pickNumber style: step buttons, big value, OK. Blocking.
void pickFreq() {
  static const char *const lab[8] = {"-1M", "-100k", "+100k", "+1M", "-25k", "-5k", "+5k", "+25k"};
  static const int step[8] = {-1000, -100, 100, 1000, -25, -5, 5, 25};
  const int bw = (SCR_W - 5 * 8) / 4, bh = 40, by = 150, oky = SCR_H - 48;
  int v = fKHz;
  glowOff();
  lcd.fillScreen(C_BG);
  uiHeader("TX frequency", "Back");
  lcd.setFont(&fonts::Font2);
  lcd.setTextColor(C_DIM, C_BG);
  lcd.setTextDatum(middle_center);
  lcd.drawString("430.000 - 439.995 MHz", SCR_W / 2, 50);
  for (int i = 0; i < 8; i++)
    uiButton(8 + i % 4 * (bw + 8), by + i / 4 * (bh + 8), bw, bh, lab[i], C_PANEL, C_PANEL, C_TEXT);
  uiButton(8, oky, SCR_W - 16, 38, "OK", C_HILITE, C_ACCENT, C_TEXT);
  auto drawValue = [&]() {
    char b[12];
    snprintf(b, sizeof b, "%d.%03d", v / 1000, v % 1000);
    lcd.setFont(&fonts::Font2);
    lcd.setTextSize(3);
    lcd.setTextColor(C_TEXT, C_BG);
    lcd.setTextDatum(middle_center);
    lcd.setTextPadding(SCR_W - 8);
    lcd.drawString(b, SCR_W / 2, 100);
    lcd.setTextSize(1);
    lcd.setTextPadding(0);
    lcd.setTextDatum(top_left);
  };
  drawValue();
  int x, y;
  while (takeTap(x, y)) {}
  while (true) {
    uiTick();
    if (!takeTap(x, y)) { delay(10); continue; }
    if (uiHeaderButtonHit(x, y)) return;                   // Back: keep the old value
    if (y >= oky && y < oky + 38) { fKHz = v; save(); return; }
    int col = (x - 8) / (bw + 8), row = (y - by) / (bh + 8);
    if (y >= by && col >= 0 && col < 4 && row >= 0 && row < 2) {
      v = constrain(v + step[row * 4 + col], F_MIN, F_MAX);
      drawValue();
    }
  }
}

// ---- sending
uint32_t frf0;                                            // the carrier, in FRF steps
inline void setFrf(uint32_t f) {
  radioSPI.beginTransaction(rf::fastSPI);
  rf::rw(0x06, f >> 16); rf::rw(0x07, f >> 8); rf::rw(0x08, f);
  radioSPI.endTransaction();
}
// Stop on BOOT, or on a new touch (the finger that started it must lift first).
bool armed;
bool stopNow() {
  bool pen = digitalRead(PIN_TP_IRQ) == LOW;
  if (!pen) armed = true;
  return (armed && pen) || digitalRead(PIN_BOOT) == LOW;
}
// Square-wave FM at the note's pitch for ms (a rest holds the carrier). False = stopped.
bool playNote(int midi, int arp, uint32_t ms) {
  static const int8_t ARP[3][3] = {{0, 0, 0}, {0, 4, 7}, {0, 3, 7}};
  uint32_t t0 = millis(), next = micros();
  bool hi = false;
  int a = 0;
  uint32_t half = midi ? halfUs(midi) : 0, aAt = t0;
  while (millis() - t0 < ms) {
    if (arp && millis() - aAt >= 33) { aAt = millis(); a = (a + 1) % 3; half = halfUs(midi + ARP[arp][a]); }
    if (!half) { delay(1); if (stopNow()) return false; continue; }
    while ((int32_t)(micros() - next) < 0) {}
    next += half;
    hi = !hi;
    setFrf(frf0 + (hi ? SONG_DEV : -SONG_DEV));
    if (hi && stopNow()) return false;
  }
  setFrf(frf0);
  return true;
}

// Transmits signal s (SIG_*) on the current frequency and power.
void send(int s) {
  const int ms = s == SIG_SONG ? songMs() : LENGTHS_MS[lenIdx], dbm = POWERS[powIdx];
  static const char *const NAMES[NUM_SIG] = {"carrier", "tone 1 kHz", "chiptune"};
  Serial.printf("TX %d.%03d MHz  %d dBm  %s  %d ms\n", fKHz / 1000, fKHz % 1000, dbm, NAMES[s], ms);
  glowOff();
  drawButton(true);
  status("Tap or BOOT to stop", C_DIM);
  frf0 = (uint64_t)fKHz * 524288 / 32000;                 // f x 2^19 / 32 MHz
  radio.setFrequency(fKHz / 1000.0f);
  radio.setOutputPower(dbm);
  radioSPI.beginTransaction(rf::fastSPI);
  rf::rw(0x04, 0); rf::rw(0x05, 0);                       // RegFdev = 0: a plain carrier
  radioSPI.endTransaction();
  radio.transmitDirect();                                 // continuous mode, PA on

  armed = false;
  bool stopped = false;
  uint32_t t0 = millis();
  if (s == SIG_SONG) {
    for (auto &st : SONG) {
      uint32_t len = st.len * STEP_MS;
      if (!playNote(st.note, st.arp, len - GAP_MS) || !playNote(0, 0, GAP_MS)) { stopped = true; break; }
    }
  } else {
    uint32_t next = micros();
    for (int k = 0; millis() - t0 < (uint32_t)ms; k++) {
      if (s == SIG_TONE) {                                // next sine step, every 62.5 us
        while ((int32_t)(micros() - next) < 0) {}
        next += 62 + (k & 1);
        setFrf(frf0 + SIN16[k & 15]);
        if (k & 15) continue;                             // buttons once per tone cycle
      } else {
        delay(1);
      }
      if (stopNow()) { stopped = true; break; }
    }
  }
  radio.standby();
  uint32_t dt = millis() - t0;
  Serial.printf("TX done after %lu ms%s\n", (unsigned long)dt, stopped ? " (stopped)" : "");
  while (digitalRead(PIN_BOOT) == LOW) delay(10);         // don't let BOOT switch modes
  bootEvent();
  delay(60); pollTouch(); pendingTap.valid = pendingDrag.valid = false;   // the stopping tap
  drawButton(false);
  char b[40];
  snprintf(b, sizeof b, stopped ? "Stopped after %lu ms" : "Sent %lu ms", (unsigned long)dt);
  status(b, C_TEXT);
}

void enter() {
  load();
  radio.beginFSK(fKHz / 1000.0f, 4.8, 5.0, SWEEP_BW_KHZ);
  radio.standby();
  sel = -1;
  drawAll();
  Serial.printf("TX ready: %d.%03d MHz\n", fKHz / 1000, fKHz % 1000);
}

void leave() {
  glowOff();
  radio.standby();
  radio.begin(METER_FREQ_MHZ);                            // LoRa again, as METER expects
}

// The header Menu: quick actions on the current frequency and power.
void menu() {
  static const char *const L[] = {"Play chiptune", "Send 1 kHz tone", "Send carrier", "Frequency", "Power"};
  char f[16], p[12], song[12], len[12];
  snprintf(f, sizeof f, "%d.%03d", fKHz / 1000, fKHz % 1000);
  snprintf(p, sizeof p, "%d dBm", POWERS[powIdx]);
  snprintf(song, sizeof song, "%d s", songMs() / 1000);
  snprintf(len, sizeof len, "%d.%d s", LENGTHS_MS[lenIdx] / 1000, LENGTHS_MS[lenIdx] % 1000 / 100);
  const char *V[] = {song, len, len, f, p};
  int c = uiList("TX menu", L, V, 5, takeTap);
  sel = -1;
  if (c == 3) pickFreq();
  if (c == 4) { powIdx = (powIdx + 1) % 5; save(); }
  drawAll();
  if (c >= 0 && c <= 2) send(c == 0 ? SIG_SONG : c == 1 ? SIG_TONE : SIG_CARRIER);
}

void loop() {
  pendingDrag.valid = false;
  if (!tapped()) return;
  int y = pendingTap.y;
  if (y >= TX_Y && y < TX_Y + TX_H) { send(sig); return; }
  int r = (y - ROW_Y) / ROW_P;
  if (y < ROW_Y || r > 3 || (y - ROW_Y) % ROW_P >= ROW_H) return;
  int was = sel;
  sel = r;
  switch (r) {
    case 0: pickFreq(); drawAll(); return;
    case 1: powIdx = (powIdx + 1) % 5; break;
    case 2: sig = (sig + 1) % NUM_SIG; break;
    default: lenIdx = (lenIdx + 1) % 4; break;
  }
  save();
  glowOff();
  if (was >= 0 && was != r) drawRow(was);
  drawRow(r);
  if (r == 2) drawRow(3);                                 // Length follows the signal
}

}  // namespace tx

// ================================================================ main
// Tabs: RX (METER, SWEEP or SCAN, whichever was last) and TX.
enum Mode { MODE_METER, MODE_SWEEP, MODE_SCAN, MODE_TX };
static Mode mode = MODE_METER, rxMode = MODE_METER;

static void leaveMode(Mode m) {
  if (m == MODE_METER) meter::leave(); else if (m == MODE_SWEEP) sweep::leave();
  else if (m == MODE_SCAN) scan::leave(); else tx::leave();
}
static void enterMode(Mode m) {
  pendingTap.valid = pendingDrag.valid = false;   // nothing carries over from the last screen
  mode = m;
  if (m != MODE_TX) rxMode = m;
  if (m == MODE_METER) meter::enter(); else if (m == MODE_SWEEP) sweep::enter();
  else if (m == MODE_SCAN) scan::enter(); else tx::enter();
}
static void switchMode(Mode m) {
  if (m == mode) return;
  leaveMode(mode);
  enterMode(m);
}

// The RX header Menu: the three screens, then band presets (they open in SWEEP).
static void rxMenu() {
  static const char *const L[] = {"Meter", "Sweep", "Scan", "Full range", "70 cm ham", "430 - 440",
                                  "433 ISM", "PMR446", "UHF business", "FRS / GMRS"};
  static const char *const V[] = {"433.000", "", "410 - 525", "410 - 525", "420 - 450", "MHz",
                                  "433.05 - 434.79", "446.0 - 446.2", "450 - 470", "462 - 467"};
  static const int BAND[][2] = {{410000, 525000}, {420000, 450000}, {430000, 440000}, {433050, 434790},
                                {446000, 446200}, {450000, 470000}, {462000, 467725}};
  Mode was = mode;
  leaveMode(mode);                     // SWEEP's scrolling screen has to be undone first
  int c = uiList("RX menu", L, V, 10, takeTap);
  if (c < 0) enterMode(was);
  else if (c < 3) enterMode(c == 0 ? MODE_METER : c == 1 ? MODE_SWEEP : MODE_SCAN);
  else { sweep::setBand(BAND[c - 3][0], BAND[c - 3][1]); enterMode(MODE_SWEEP); }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nH@N 433MHz LoRa Module Scanner");

  pinMode(PIN_BOOT, INPUT_PULLUP);

  lcd.init();
  lcd.setRotation(2);                  // portrait, same as H@N-CYD-GBC on this board
  lcd.setBrightness(255);

  applyTheme(0);                       // H@N Night (han_ui.h)
  C_GRID  = C_PANEL;
  C_ALERT = lcd.color565(255, 70, 90);
  sweep::buildPalette();

  // Touch calibration. Runs on the first boot, or after BOOT was held 2 s (which sets
  // "recal" and restarts).
  { uint8_t d[57]; xptRead(d); }   // re-arm the touch chip's pen interrupt
  prefs.begin("han433", false);
  sdOk = sdOpen();                 // stays mounted; shares VSPI with the radio
  detlog::boot = prefs.getUInt("boot", 0) + 1;
  prefs.putUInt("boot", detlog::boot);
  Serial.printf("LOG boot %lu, SD card %s\n", (unsigned long)detlog::boot,
                sdOk ? "mounted" : "not found - detections go to serial only");
  if (sdOk) detlog::loadSpurs();
  const char *calFrom = loadCal();
  bool recal = prefs.getBool("recal", false);
  if (calFrom) Serial.printf("Touch calibration loaded from %s\n", calFrom);
  if (!calFrom || recal) {
    prefs.putBool("recal", false);
    runCalibration();
  }

  lcd.fillScreen(C_BG);
  uiHeader("H@N 433 SCANNER");

  radioSPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
  if (sdOk) sdBus(false);              // card pins idle until a log flush

  // begin() resets (skipped here - RST is NC), reads the chip version, and sets
  // default LoRa parameters.
  int st = radio.begin(METER_FREQ_MHZ);
  if (st == RADIOLIB_ERR_NONE) {
    Serial.println("Radio found - SX1278 responding.");
  } else {
    Serial.print("Radio NOT found. Error code: ");
    Serial.println(st);
    Serial.println("Check NSS and MISO first - those are the usual suspects.");
    char buf[40];
    snprintf(buf, sizeof(buf), "Radio NOT found (error %d)", st);
    fatal(buf, "Check the NSS and MISO wires");
  }

  meter::enter();
}

void loop() {
  detlog::pollSerial();
  detlog::flush();
  int boot = bootEvent();
  if (boot == 1) {                       // short press: METER -> SWEEP -> SCAN (in RX)
    if (mode != MODE_TX) switchMode(mode == MODE_METER ? MODE_SWEEP : mode == MODE_SWEEP ? MODE_SCAN : MODE_METER);
    return;
  }
  if (boot == 2) {                       // held 2 s: restart into touch calibration
    prefs.putBool("recal", true);
    lcd.fillScreen(C_BG);
    lcd.setFont(&fonts::Font2);
    lcd.setTextDatum(middle_center);
    lcd.setTextColor(C_TEXT, C_BG);
    lcd.drawString("Restarting into touch calibration", SCR_W / 2, 150);
    lcd.drawString("- let go of BOOT -", SCR_W / 2, 175);
    while (digitalRead(PIN_BOOT) == LOW) delay(10);   // BOOT low at reset = flashing mode
    delay(200);
    ESP.restart();
  }
  pollTouch();
  if (tapIsOnTabs()) {                   // navigation: header button or RX/TX tabs
    pendingTap.valid = false;
    if (pendingTap.y < TAB_Y) {          // header button: the tab's menu
      if (mode == MODE_TX) tx::menu(); else rxMenu();
    } else {
      switchMode(pendingTap.x < SCR_W / 2 ? rxMode : MODE_TX);
    }
    return;
  }
  uiTick();                              // header stars, edge shine, row glow
  if (mode == MODE_METER) meter::loop();
  else if (mode == MODE_SWEEP) sweep::loop();
  else if (mode == MODE_SCAN) scan::loop();
  else tx::loop();
}
