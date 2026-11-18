// H@N 433MHz LoRa Module Scanner - RSSI meter and band sweep.
//
// Two screens, switched with the METER / SWEEP tabs along the bottom of the screen
// (the BOOT button also toggles them):
//
//   METER  Listens on one frequency: big dBm readout, level bar with peak hold,
//          estimated noise floor and a scrolling history graph. Tap to reset the
//          peak.
//   SWEEP  Steps across 430.00-439.95 MHz and draws a spectrum (with a max-hold
//          trace) over a scrolling waterfall - "what's in my air". Tap to clear
//          the max hold.
//
// Both also report over USB serial at 115200 baud.
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

static const int SCR_W = 240;
static uint16_t C_BG, C_PANEL, C_DIM, C_TEXT, C_ACCENT, C_GRID, C_ALERT;

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

static void drawHeader(const char *right) {
  lcd.fillRect(0, 0, SCR_W, 26, C_PANEL);
  lcd.setFont(&fonts::Font2);
  lcd.setTextDatum(middle_left);
  lcd.setTextColor(C_ACCENT, C_PANEL);
  lcd.drawString("H@N 433 SCANNER", 8, 13);
  lcd.setTextDatum(middle_right);
  lcd.setTextColor(C_TEXT, C_PANEL);
  lcd.drawString(right, SCR_W - 8, 13);
}

// ---------------------------------------------------------------- tab bar
// Two buttons along the bottom edge. They span the full width and are 34 px tall,
// so they still work if the touch mapping below is off by a few pixels.
static const int TAB_Y = 286, TAB_H = 34;

static void drawTabs(int active) {   // 0 = METER, 1 = SWEEP
  const char *names[2] = {"METER", "SWEEP"};
  lcd.setFont(&fonts::Font2);
  lcd.setTextDatum(middle_center);
  for (int i = 0; i < 2; i++) {
    int x = i * (SCR_W / 2);
    bool on = (i == active);
    uint16_t fill = on ? C_PANEL : C_BG;
    lcd.fillRect(x + 2, TAB_Y + 2, SCR_W / 2 - 4, TAB_H - 4, fill);
    lcd.drawRect(x + 2, TAB_Y + 2, SCR_W / 2 - 4, TAB_H - 4, on ? C_ACCENT : C_GRID);
    lcd.setTextColor(on ? C_ACCENT : C_DIM, fill);
    lcd.drawString(names[i], x + SCR_W / 4, TAB_Y + TAB_H / 2);
  }
}

// ---------------------------------------------------------------- touch calibration
// Raw XPT2046 readings (0..4095) -> screen pixels in portrait (rotation 2). The panel
// differs unit to unit, so it is calibrated on the first boot (4 crosshairs) and the
// result is saved to the SD card, and to the ESP32's flash (NVS) as a fallback.
// Hold BOOT for 2 s at any time to restart into calibration.
//
// The SD card is on VSPI pins 18/19/23/5 - the same SPI peripheral the radio uses on
// its own pins. So the card is only touched in setup(), before the radio starts.
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

static bool sdOpen() {
  sdSPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (!SD.begin(PIN_SD_CS, sdSPI)) { sdSPI.end(); return false; }
  return true;
}
static void sdClose() { SD.end(); sdSPI.end(); }

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
  if (sdOpen()) {
    bool ok = false;
    File f = SD.open(CAL_FILE, FILE_READ);
    if (f) { ok = parseCal(f.readStringUntil('\n')); f.close(); }
    sdClose();
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
  bool sdOk = false;
  if (sdOpen()) {
    File f = SD.open(CAL_FILE, FILE_WRITE);
    if (f) { f.println(line); f.close(); sdOk = true; }
    sdClose();
  }
  Serial.printf("Touch calibration saved (%s): %s\n",
                sdOk ? "SD card + flash" : "flash only, no SD card", line.c_str());
  return sdOk ? "SD card" : "flash (no SD card found)";
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

static bool tapIsOnTabs() { return pendingTap.valid && pendingTap.y >= TAB_Y; }

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
  snprintf(buf, sizeof(buf), "%.3f MHz", METER_FREQ_MHZ);
  drawHeader(buf);

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

// ================================================================ SWEEP
namespace sweep {

// Everything is integer dBm. Spectrum: -130..-10 dBm into 90 px (3/4 px per dB).
const int X0 = 20, SPEC_Y = 48, SPEC_H = 90, AXIS_Y = SPEC_Y + SPEC_H + 3;

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
uint32_t sweepCount;
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
  drawHeader("SWEEP");
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
  drawTabs(1);

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

  if (tapped()) for (int i = 0; i < SWEEP_BINS; i++) hold[i] = bins[i];

  char b[40];
  int snr = bins[peak] - floorDbm;
  lcd.setFont(&fonts::Font2);
  lcd.setTextDatum(top_left);
  lcd.setTextPadding(SCR_W - 16);
  lcd.setTextColor(snr >= 6 ? snrColor(snr) : C_DIM, C_BG);
  snprintf(b, sizeof b, "Peak %d.%02d MHz  %d dBm", binKHz(peak) / 1000, binKHz(peak) % 1000 / 10, bins[peak]);
  lcd.drawString(b, 8, 29);
  lcd.setTextPadding(0);

  if (++sweepCount % 10 == 0)
    Serial.printf("SWEEP %lu ms  floor %d dBm  peak %s\n", (unsigned long)dt, floorDbm, b + 5);
  return true;
}

}  // namespace sweep

// ================================================================ main
enum Mode { MODE_METER, MODE_SWEEP };
static Mode mode = MODE_METER;

static void switchMode(Mode m) {
  if (mode == MODE_METER) meter::leave(); else sweep::leave();
  mode = m;
  if (mode == MODE_METER) meter::enter(); else sweep::enter();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nH@N 433MHz LoRa Module Scanner");

  pinMode(PIN_BOOT, INPUT_PULLUP);

  lcd.init();
  lcd.setRotation(2);                  // portrait, same as H@N-CYD-GBC on this board
  lcd.setBrightness(255);

  C_BG     = lcd.color565(0, 0, 0);
  C_PANEL  = lcd.color565(3, 45, 99);
  C_DIM    = lcd.color565(158, 192, 235);
  C_TEXT   = lcd.color565(238, 246, 255);
  C_ACCENT = lcd.color565(124, 232, 255);
  C_GRID   = lcd.color565(20, 32, 60);
  C_ALERT  = lcd.color565(255, 70, 90);
  sweep::buildPalette();

  // Touch calibration, before the radio starts (the SD card shares its SPI peripheral).
  // Runs on the first boot, or after BOOT was held 2 s (which sets "recal" and restarts).
  { uint8_t d[57]; xptRead(d); }   // re-arm the touch chip's pen interrupt
  prefs.begin("han433", false);
  const char *calFrom = loadCal();
  bool recal = prefs.getBool("recal", false);
  if (calFrom) Serial.printf("Touch calibration loaded from %s\n", calFrom);
  if (!calFrom || recal) {
    prefs.putBool("recal", false);
    runCalibration();
  }

  lcd.fillScreen(C_BG);
  drawHeader("");

  radioSPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);

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
  int boot = bootEvent();
  if (boot == 1) {                       // short press: switch mode
    switchMode(mode == MODE_METER ? MODE_SWEEP : MODE_METER);
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
  if (tapIsOnTabs()) {
    pendingTap.valid = false;
    Mode want = pendingTap.x < SCR_W / 2 ? MODE_METER : MODE_SWEEP;
    if (want != mode) switchMode(want);
    return;
  }
  if (mode == MODE_METER) meter::loop();
  else sweep::loop();
}
