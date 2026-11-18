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

// Sweep: SWEEP_BINS steps of SWEEP_STEP_MHZ from SWEEP_START_MHZ.
static const float SWEEP_START_MHZ = 430.0;
static const float SWEEP_STEP_MHZ = 0.05;
static const int   SWEEP_BINS = 200;               // 430.00 - 439.95 MHz
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

// Call often. Latches one tap per press (the press edge), polled every 30 ms.
static void pollTouch() {
  static bool was = false;
  static uint32_t last = 0;
  if (millis() - last < 30) return;
  last = millis();
  int rx, ry;
  bool now = readTouchRaw(rx, ry);
  if (now && !was) {
    int sx = -1, sy = -1;
    rawToScreen(rx, ry, sx, sy);   // stays -1,-1 when uncalibrated (never a tab)
    pendingTap = {true, sx, sy};
    Serial.printf("TOUCH raw %d,%d -> screen %d,%d\n", rx, ry, sx, sy);
  }
  was = now;
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
  uiHeader(buf, "Sweep");

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

// ================================================================ DETECTION LOG
// The 2-sample RSSI is noisy: ~10% of bins sit 10+ dB over the floor (25th percentile)
// in quiet air. So thresholds hang off each sweep's "noise top" instead - its 95th
// percentile (10th-loudest bin). A detection = a run of bins at least ON_DB over the
// noise top, seen again in the very next sweep. It stays open while a run within NEAR bins
// of where it started stays OFF_DB over the noise top - the bin never moves, so it
// can't wander from spike to spike - and ends after MISS sweeps without one. The logged
// SNR is relative to the floor. Ended detections are printed over
// serial ("DET ...") and kept in RAM, then appended to the SD card in batches between
// sweeps: when FLUSH_N are waiting or the oldest has waited FLUSH_MS.
//
// Time: every row has the boot number (NVS counter) and uptime in ms. Once the PC has
// sent "T<unix ms>" over serial, rows also get UTC - including rows still in RAM.
namespace detlog {

const int ON_DB = 6, OFF_DB = 1;     // dB over the noise top to open / to stay open
// The ESP32's 40 MHz crystal x 11 = 440.000 MHz: its skirt fills 439.60-439.95 on every
// unit and comes and goes, so those bins are shown but not scanned for detections.
const int SPUR_BIN = 192;            // 439.60 MHz
// Other spurs (this board: 432.05, 436.7, 437.95, 438.35) are learned: busy[i] is how
// often bin i is over the stay-open level, averaged over ~2048 sweeps (~95 s), 65535 =
// always. A bin counts as quiet - a spur, or a steady local emitter - when it or a
// neighbor is busier than SPUR_BUSY (a spur's skirt flickers less than its center).
// Spurs here flicker at 5-15%; a 2 s burst only lifts busy to ~2%. Learning is slow on
// purpose (a fast learner takes an early real signal for a spur), so the learned bins
// are kept on the SD card as a 200-bit map and loaded at boot: only the first run on a
// card has a warm-up (a minute or two of spur rows).
const int32_t SPUR_BUSY = 65535 * 5 / 100;
uint16_t busy[SWEEP_BINS];
uint8_t spurMap[SWEEP_BINS / 8];     // learned spur bins, as last saved / loaded
bool spurDirty;                      // the learned bins differ from spurMap
uint32_t spurSaved;                  // millis() of the last save
const uint32_t SPUR_SAVE_MS = 60000;
const char *SPUR_FILE = "/han433_spur.bin";
const int MISS = 2;                  // sweeps a signal may drop out before it ends
const int NEAR = 3;                  // a run within 3 bins (150 kHz) continues a detection
// A transmitter close by overloads the front end and shows sidelobes up to ~1 MHz out,
// 30-40 dB down: a run SIDE_DB under the loudest bin within SIDE bins is ignored.
const int SIDE = 20, SIDE_DB = 25;
const uint32_t MAX_MS = 60000;       // a carrier that never stops: one row a minute
const int NOPEN = 8, NBUF = 32, FLUSH_N = 16;
const uint32_t FLUSH_MS = 30000;
const char *FILE_NAME = "/han433_det.csv";

// at = bin it opened on (fixed), bin = loudest bin; ok = confirmed (seen 2 sweeps running)
struct Det { uint32_t t0, t1; int16_t bin, at, peak, snr; uint16_t sweeps; uint8_t miss; bool ok; };
Det act[NOPEN], buf[NBUF];
int nOpen, nBuf, hot, spurs;         // last sweep: bins over the open level / learned spurs
uint32_t total, dropped, boot, pendSince;
int64_t utcOffMs;                    // unix ms - millis(); 0 = clock not set

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
  if (!d.sweeps) return;
  int khz = 430000 + d.bin * 50;
  Serial.printf("DET %d.%03d MHz  %d dBm  +%d dB  %lu ms  %u sweeps\n", khz / 1000, khz % 1000,
                d.peak, d.snr, (unsigned long)(d.t1 - d.t0), d.sweeps);
  total++;
  if (!sdOk) return;
  if (nBuf == NBUF) { dropped++; return; }
  if (!nBuf) pendSince = millis();
  buf[nBuf++] = d;
}

// After each sweep: the bins in dBm, the sweep's floor and noise top, its start time.
void scan(const int16_t *bins, int n, int floor, int top, uint32_t now) {
  const int on = top + ON_DB, off = top + OFF_DB;
  bool seen[NOPEN] = {};
  bool lit[SWEEP_BINS];                                     // over off, and not a spur
  hot = spurs = 0;
  for (int i = 0; i < n; i++) {
    busy[i] += ((bins[i] >= off ? 65535 : 0) - (int32_t)busy[i]) >> 11;
    uint8_t bit = 1 << (i & 7), &m = spurMap[i >> 3];
    if ((busy[i] > SPUR_BUSY) != !!(m & bit)) { m ^= bit; spurDirty = true; }
  }
  for (int i = 0; i < n; i++) {
    bool spur = busy[i] > SPUR_BUSY || (i && busy[i - 1] > SPUR_BUSY) ||
                (i < n - 1 && busy[i + 1] > SPUR_BUSY);
    spurs += spur;
    lit[i] = bins[i] >= off && !spur;
    hot += lit[i] && bins[i] >= on;
  }
  for (int i = 0; i < n;) {
    if (!lit[i]) { i++; continue; }
    int r0 = i, p = i;                                      // run r0..i-1, peak bin p
    for (; i < n && lit[i]; i++) if (bins[i] > bins[p]) p = i;
    int big = bins[p];                                      // loudest within +-1 MHz
    for (int j = p < SIDE ? 0 : p - SIDE; j <= p + SIDE && j < n; j++) if (bins[j] > big) big = bins[j];
    if (bins[p] <= big - SIDE_DB) continue;                 // a strong neighbor's sidelobe
    int k = 0;
    while (k < nOpen && (act[k].at < r0 - NEAR || act[k].at > i - 1 + NEAR)) k++;
    if (k == nOpen) {                                       // a new signal?
      if (bins[p] < on || nOpen == NOPEN) continue;
      act[nOpen++] = {now, now, (int16_t)p, (int16_t)p, -128, 0, 0, 0, false};
    }
    Det &d = act[k];
    if (!seen[k]) { seen[k] = true; d.t1 = now; d.ok |= ++d.sweeps >= 2; d.miss = 0; }
    if (bins[p] > d.peak) { d.peak = bins[p]; d.bin = p; d.snr = bins[p] - floor; }
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
      d.t0 = d.t1 = now; d.sweeps = 0; d.peak = -128;
    }
    k++;
  }
}

// Setup, SD card mounted: start from the spurs learned last time.
void loadSpurs() {
  File f = SD.open(SPUR_FILE, FILE_READ);
  if (!f) return;
  int got = f.read(spurMap, sizeof spurMap);
  f.close();
  if (got != sizeof spurMap) { memset(spurMap, 0, sizeof spurMap); return; }
  int n = 0;
  for (int i = 0; i < SWEEP_BINS; i++)
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
    if (!f.size()) f.println("boot,uptime_ms,utc,freq_mhz,peak_dbm,snr_db,dur_ms,sweeps");
    char s[28];
    for (int k = 0; k < nBuf; k++) {
      const Det &d = buf[k];
      int khz = 430000 + d.bin * 50;
      utc(s, d.t0);
      f.printf("%lu,%lu,%s,%d.%03d,%d,%d,%lu,%u\n", (unsigned long)boot, (unsigned long)d.t0, s,
               khz / 1000, khz % 1000, d.peak, d.snr, (unsigned long)(d.t1 - d.t0), d.sweeps);
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
namespace sweep {

// Everything is integer dBm. Spectrum: -130..-10 dBm into 90 px (3/4 px per dB).
const int X0 = 20, SPEC_Y = 52, SPEC_H = 90, AXIS_Y = SPEC_Y + SPEC_H + 3;   // below the 32 px header

// Waterfall = ILI9341 hardware vertical scroll. No pixel is ever copied: each sweep
// writes one new 200-px row and moves the controller's scroll pointer (VSCRSADD).
// Scrolling works on frame-memory rows, and with rotation 2 (MY set) screen row y is
// memory row 319 - y. So, in memory rows:
//   TFA = the tabs strip at the screen bottom, VSA = the waterfall, BFA = everything above.
const int WF_Y = 158, WF_H = 124;
const int TFA = 320 - WF_Y - WF_H, VSA = WF_H, BFA = WF_Y;
// Rotation 2 sets MX|MY|ML|MH. ML/MH only reverse the panel's refresh order, but ML also
// flips where the scroll areas sit, so drop both while sweeping: same picture, one layout.
const uint8_t MADCTL_SWEEP = 0xC8, MADCTL_ROT2 = 0xDC;   // MY|MX|BGR (+ML|MH)

int16_t bins[SWEEP_BINS], hold[SWEEP_BINS];
uint16_t pal[64];
int floorDbm = -120, vsp = TFA, wfRow;   // wfRow = memory row being drawn this sweep
uint32_t sweepCount, shownTotal;
// The sweep runs the radio in FSK mode: its RSSI is smoothed over just 2 samples and the
// receiver can be relocked onto a new frequency with one register write, so a reading is
// valid ~0.2 ms after a retune. LoRa mode's RSSI is averaged far longer, which smeared
// signals a few bins late and read them low.
const uint8_t RX_CFG = 0x08;             // RegRxConfig: AGC on, AFC off, no trigger

inline int specH(int dbm) { int v = (dbm + 130) * 3 / 4; return v < 0 ? 0 : v > SPEC_H ? SPEC_H : v; }
inline int binKHz(int i) { return 430000 + i * 50; }

// ---- radio, raw SPI. RadioLib's setFrequency()/startReceive() verify every register
// they touch (~1.4 ms per step); a sweep only needs 4 writes and 1 read.
SPISettings fastSPI(8000000, MSBFIRST, SPI_MODE0);
inline void rw(uint8_t reg, uint8_t val) {
  digitalWrite(PIN_NSS, LOW); radioSPI.transfer16((reg | 0x80) << 8 | val); digitalWrite(PIN_NSS, HIGH);
}
inline uint8_t rr(uint8_t reg) {
  digitalWrite(PIN_NSS, LOW); uint8_t v = radioSPI.transfer16(reg << 8); digitalWrite(PIN_NSS, HIGH);
  return v;
}
void retune(int i) {
  uint32_t frf = (uint64_t)binKHz(i) * 524288 / 32000;   // f * 2^19 / 32 MHz
  radioSPI.beginTransaction(fastSPI);
  rw(0x06, frf >> 16); rw(0x07, frf >> 8); rw(0x08, frf);
  rw(0x0D, RX_CFG | 0x20);                // RestartRxWithPllLock: relock on the new frequency
  radioSPI.endTransaction();
}
int rssi() {   // FSK RegRssiValue = -2 x dBm
  radioSPI.beginTransaction(fastSPI);
  int v = -(rr(0x11) >> 1);
  radioSPI.endTransaction();
  return v;
}

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

// Draw bin i straight to the screen: spectrum column, max-hold dot, floor dash and
// this sweep's waterfall pixel. ~4 tiny writes, done while the next bin settles.
void drawBin(int i) {
  int x = X0 + i, h = specH(bins[i]), y0 = SPEC_Y + SPEC_H;
  lcd.drawFastVLine(x, SPEC_Y, SPEC_H - h, C_BG);
  lcd.drawFastVLine(x, y0 - h, h, snrColor(bins[i] - floorDbm));
  lcd.drawPixel(x, y0 - 1 - specH(hold[i]), C_TEXT);
  if (i % 6 < 3) lcd.drawPixel(x, y0 - 1 - specH(floorDbm), C_ACCENT);
  int k = (bins[i] - floorDbm) * 63 / 40;
  lcd.drawPixel(x, 319 - wfRow, pal[k < 0 ? 0 : k > 63 ? 63 : k]);
}

// Header: the mode and the detections logged so far; its button goes to METER.
void title() {
  shownTotal = detlog::total;
  char s[32];
  snprintf(s, sizeof s, sdOk ? "SWEEP  %lu logged" : "SWEEP  %lu, no SD", (unsigned long)shownTotal);
  uiHeader(s, "Meter");
}

void enter() {
  radio.beginFSK(430.0, 4.8, 5.0, SWEEP_BW_KHZ);
  radio.startReceive();
  radioSPI.beginTransaction(fastSPI);
  rw(0x0E, 0x00);                         // RegRssiConfig: no offset, smoothing 2 samples
  rw(0x0D, RX_CFG);
  radioSPI.endTransaction();

  for (int i = 0; i < SWEEP_BINS; i++) hold[i] = -130;
  sweepCount = 0;

  lcd.fillScreen(C_BG);
  title();
  lcd.setFont(&fonts::Font0);
  lcd.setTextColor(C_DIM, C_BG);
  lcd.setTextDatum(top_center);
  char b[8];
  for (int i = 0; i <= SWEEP_BINS; i += 50) {   // MHz axis
    int x = X0 + (i < SWEEP_BINS ? i : SWEEP_BINS - 1);
    lcd.drawFastVLine(x, AXIS_Y - 2, 3, C_DIM);
    snprintf(b, sizeof b, "%d.%d", binKHz(i) / 1000, binKHz(i) % 1000 / 100);
    lcd.drawString(b, x, AXIS_Y + 2);
  }
  lcd.setTextDatum(middle_right);                // dBm labels
  for (int d = -110; d <= -30; d += 40) lcd.drawNumber(d, X0 - 2, SPEC_Y + SPEC_H - 1 - specH(d));
  drawTabs(0);                                   // SWEEP is under RX

  uint16_t def[3] = {TFA, VSA, BFA};
  lcdMadctl(MADCTL_SWEEP);
  lcdCmd(0x33, def, 3);                          // VSCRDEF
  vsp = TFA;
  scrollTo(vsp);
  Serial.printf("SWEEP 430.00-439.95 MHz, %d bins, 50 kHz step, dwell %lu us\n",
                SWEEP_BINS, (unsigned long)SWEEP_DWELL_US);
}

void leave() {
  scrollTo(0);
  lcd.startWrite(); lcd.writeCommand(0x13); lcd.endWrite();   // NORON: back to normal mode
  lcdMadctl(MADCTL_ROT2);
  radio.begin(METER_FREQ_MHZ);                                 // back to LoRa for the meter
}

// One sweep per call. Each bin's drawing happens while the next bin settles, so the
// screen updates continuously. Returns false early on a tab tap or BOOT.
bool loop() {
  uint32_t t0 = millis();
  // New waterfall row: advance the scroll pointer, then the row that just wrapped to
  // the top (the oldest one) is the one to overwrite. Blank it first.
  wfRow = vsp;
  vsp = TFA + (vsp - TFA + 1) % VSA;
  lcd.drawFastHLine(X0, 319 - wfRow, SWEEP_BINS, C_BG);
  scrollTo(vsp);
  retune(0);                     // 439.95 -> 430.00 is a 10 MHz jump: give the PLL 1 ms
  delayMicroseconds(1000);

  for (int i = 0; i < SWEEP_BINS; i++) {
    uint32_t t = micros();
    retune(i);
    if (i) drawBin(i - 1);
    while (micros() - t < SWEEP_DWELL_US) {}
    bins[i] = rssi();
    if (bins[i] > hold[i]) hold[i] = bins[i];
    if (i % 25 == 0) {
      pollTouch();
      if (tapIsOnTabs() || digitalRead(PIN_BOOT) == LOW) return false;
    }
  }
  drawBin(SWEEP_BINS - 1);
  uint32_t dt = millis() - t0;

  // Floor for the next sweep: 25th percentile, by counting (bins are whole dB).
  uint8_t cnt[136] = {0};
  int peak = 0;
  for (int i = 0; i < SWEEP_BINS; i++) {
    int v = bins[i] + 140; cnt[v < 0 ? 0 : v > 135 ? 135 : v]++;
    if (bins[i] > bins[peak]) peak = i;
  }
  for (int v = 0, n = 0; v < 136; v++) if ((n += cnt[v]) >= SWEEP_BINS / 4) { floorDbm = v - 140; break; }

  int top = 135;                          // noise top: 95th percentile, 10th-loudest bin
  for (int n = 0; top > 0 && (n += cnt[top]) < SWEEP_BINS / 20; top--) {}
  top -= 140;
  detlog::scan(bins, detlog::SPUR_BIN, floorDbm, top, t0);
  if (detlog::total != shownTotal) title();

  if (tapped()) for (int i = 0; i < SWEEP_BINS; i++) hold[i] = bins[i];

  char b[40];
  int snr = bins[peak] - floorDbm;
  lcd.setFont(&fonts::Font2);
  lcd.setTextDatum(top_left);
  lcd.setTextPadding(SCR_W - 16);
  lcd.setTextColor(snr >= 6 ? snrColor(snr) : C_DIM, C_BG);
  snprintf(b, sizeof b, "Peak %d.%02d MHz  %d dBm", binKHz(peak) / 1000, binKHz(peak) % 1000 / 10, bins[peak]);
  lcd.drawString(b, 8, UI_HDR_H + 3);
  lcd.setTextPadding(0);

  if (++sweepCount % 10 == 0)
    Serial.printf("SWEEP %lu ms  floor %d  top %d dBm  hot %d  spurs %d  peak %s\n",
                  (unsigned long)dt, floorDbm, top, detlog::hot, detlog::spurs, b + 5);
  return true;
}

}  // namespace sweep

// ================================================================ TX
// Test transmitter: a plain carrier, or a 1 kHz FM tone. The SX1278 has no audio input,
// so the tone is made in software: the radio sends a carrier (FSK continuous mode, zero
// deviation) and the frequency registers are rewritten along a 16-step sine, 16 times
// per tone cycle - +-3 kHz at 1 kHz, which an FM receiver plays as a tone.
// Settings are kept in NVS. A tap or BOOT stops a transmission early.
namespace tx {

const int POWERS[] = {2, 5, 10, 14, 17};          // dBm out of PA_BOOST
const int LENGTHS_MS[] = {500, 1000, 2000, 5000};
const int F_MIN = 430000, F_MAX = 439995;         // kHz
int fKHz = 433500, powIdx = 0, sig = 1, lenIdx = 1, sel = -1;   // sig: 0 carrier, 1 tone

// 3 kHz deviation in FRF steps (32 MHz / 2^19 = 61.035 Hz): 49 x sin(k x 22.5 deg)
const int8_t SIN16[16] = {0, 19, 35, 45, 49, 45, 35, 19, 0, -19, -35, -45, -49, -45, -35, -19};

const int ROW_X = 6, ROW_W = SCR_W - 12, ROW_Y = UI_HDR_H + 6, ROW_H = 36, ROW_P = 42;
const int TX_Y = ROW_Y + 4 * ROW_P + 4, TX_H = 50;      // TRANSMIT button
const int STATUS_Y = TX_Y + TX_H + 6;
const char *const LABELS[4] = {"Frequency", "Power", "Signal", "Length"};

void load() {
  fKHz = constrain((int)prefs.getUInt("txf", fKHz), F_MIN, F_MAX);
  powIdx = prefs.getUChar("txp", powIdx) % 5;
  sig = prefs.getUChar("txs", sig) & 1;
  lenIdx = prefs.getUChar("txl", lenIdx) % 4;
}
void save() {
  prefs.putUInt("txf", fKHz); prefs.putUChar("txp", powIdx);
  prefs.putUChar("txs", sig); prefs.putUChar("txl", lenIdx);
}

void value(int r, char *s, int n) {
  switch (r) {
    case 0: snprintf(s, n, "%d.%03d MHz", fKHz / 1000, fKHz % 1000); break;
    case 1: snprintf(s, n, "%d dBm", POWERS[powIdx]); break;
    case 2: snprintf(s, n, sig ? "Tone 1 kHz" : "Carrier"); break;
    default: snprintf(s, n, "%d.%d s", LENGTHS_MS[lenIdx] / 1000, LENGTHS_MS[lenIdx] % 1000 / 100); break;
  }
}
void drawRow(int r) {
  char v[16];
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
  uiHeader("TX  test transmitter");
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
  pendingTap.valid = false;
  while (true) {
    uiTick();
    pollTouch();
    if (!pendingTap.valid) { delay(10); continue; }
    pendingTap.valid = false;
    int x = pendingTap.x, y = pendingTap.y;
    if (uiHeaderButtonHit(x, y)) return;                   // Back: keep the old value
    if (y >= oky && y < oky + 38) { fKHz = v; save(); return; }
    int col = (x - 8) / (bw + 8), row = (y - by) / (bh + 8);
    if (y >= by && col >= 0 && col < 4 && row >= 0 && row < 2) {
      v = constrain(v + step[row * 4 + col], F_MIN, F_MAX);
      drawValue();
    }
  }
}

void send() {
  const int ms = LENGTHS_MS[lenIdx], dbm = POWERS[powIdx];
  Serial.printf("TX %d.%03d MHz  %d dBm  %s  %d ms\n", fKHz / 1000, fKHz % 1000, dbm,
                sig ? "tone 1 kHz" : "carrier", ms);
  glowOff();
  drawButton(true);
  status("Tap or BOOT to stop", C_DIM);
  const uint32_t frf = (uint64_t)fKHz * 524288 / 32000;  // f x 2^19 / 32 MHz
  radio.setFrequency(fKHz / 1000.0f);
  radio.setOutputPower(dbm);
  radioSPI.beginTransaction(sweep::fastSPI);
  sweep::rw(0x04, 0); sweep::rw(0x05, 0);                 // RegFdev = 0: a plain carrier
  radioSPI.endTransaction();
  radio.transmitDirect();                                 // continuous mode, PA on

  // Stop on BOOT, or on a new touch (the finger that pressed TRANSMIT must lift first).
  bool armed = false, stopped = false;
  uint32_t t0 = millis(), next = micros();
  for (int k = 0; millis() - t0 < (uint32_t)ms; k++) {
    if (sig) {                                            // next sine step, every 62.5 us
      while ((int32_t)(micros() - next) < 0) {}
      next += 62 + (k & 1);
      uint32_t f = frf + SIN16[k & 15];
      radioSPI.beginTransaction(sweep::fastSPI);
      sweep::rw(0x06, f >> 16); sweep::rw(0x07, f >> 8); sweep::rw(0x08, f);
      radioSPI.endTransaction();
      if (k & 15) continue;                               // buttons once per tone cycle
    } else {
      delay(1);
    }
    bool pen = digitalRead(PIN_TP_IRQ) == LOW;
    if (!pen) armed = true;
    if ((armed && pen) || digitalRead(PIN_BOOT) == LOW) { stopped = true; break; }
  }
  radio.standby();
  uint32_t dt = millis() - t0;
  Serial.printf("TX done after %lu ms%s\n", (unsigned long)dt, stopped ? " (stopped)" : "");
  while (digitalRead(PIN_BOOT) == LOW) delay(10);         // don't let BOOT switch modes
  bootEvent();
  pollTouch(); pendingTap.valid = false;                  // the stopping tap is used up
  drawButton(false);
  char s[40];
  snprintf(s, sizeof s, stopped ? "Stopped after %lu ms" : "Sent %lu ms", (unsigned long)dt);
  status(s, C_TEXT);
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

void loop() {
  if (!tapped()) return;
  int x = pendingTap.x, y = pendingTap.y;
  (void)x;
  if (y >= TX_Y && y < TX_Y + TX_H) { send(); return; }
  int r = (y - ROW_Y) / ROW_P;
  if (y < ROW_Y || r > 3 || (y - ROW_Y) % ROW_P >= ROW_H) return;
  int was = sel;
  sel = r;
  switch (r) {
    case 0: pickFreq(); drawAll(); return;
    case 1: powIdx = (powIdx + 1) % 5; break;
    case 2: sig ^= 1; break;
    default: lenIdx = (lenIdx + 1) % 4; break;
  }
  save();
  glowOff();
  if (was >= 0 && was != r) drawRow(was);
  drawRow(r);
}

}  // namespace tx

// ================================================================ main
// Tabs: RX (METER or SWEEP, whichever was last) and TX.
enum Mode { MODE_METER, MODE_SWEEP, MODE_TX };
static Mode mode = MODE_METER, rxMode = MODE_METER;

static void switchMode(Mode m) {
  if (m == mode) return;
  if (mode == MODE_METER) meter::leave(); else if (mode == MODE_SWEEP) sweep::leave(); else tx::leave();
  mode = m;
  if (m != MODE_TX) rxMode = m;
  if (m == MODE_METER) meter::enter(); else if (m == MODE_SWEEP) sweep::enter(); else tx::enter();
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
  if (boot == 1) {                       // short press: METER <-> SWEEP (in RX)
    if (mode != MODE_TX) switchMode(mode == MODE_METER ? MODE_SWEEP : MODE_METER);
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
    if (pendingTap.y < TAB_Y) {          // header button: METER <-> SWEEP
      if (mode != MODE_TX) switchMode(mode == MODE_METER ? MODE_SWEEP : MODE_METER);
    } else {
      switchMode(pendingTap.x < SCR_W / 2 ? rxMode : MODE_TX);
    }
    return;
  }
  uiTick();                              // header stars, edge shine, row glow
  if (mode == MODE_METER) meter::loop();
  else if (mode == MODE_SWEEP) sweep::loop();
  else tx::loop();
}
