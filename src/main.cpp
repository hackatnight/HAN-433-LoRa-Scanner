// H@N 433MHz LoRa Module Scanner - single-channel RSSI meter.
//
// Listens on one frequency and shows the live channel signal level on the
// built-in 240x320 screen: a big dBm readout, a level bar with peak hold, the
// estimated noise floor, and a scrolling history graph. Tap the screen to reset
// the peak. The level is also still printed over USB serial at 115200 baud.
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

// ---------------------------------------------------------------- radio
#define PIN_SCK   25
#define PIN_MISO  21
#define PIN_MOSI  32
#define PIN_NSS   22

static const float FREQ_MHZ = 433.0;

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
      c.freq_write = 40000000;
      c.freq_read = 16000000;
      c.spi_3wire = false;
      c.use_lock = true;
      // No DMA: the meter only pushes small sprites, and plain transfers keep the
      // shared bus simple for the touch chip.
      c.dma_channel = 0;
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
static LGFX_Sprite barSpr(&lcd);
static LGFX_Sprite graphSpr(&lcd);

// ---------------------------------------------------------------- layout
static const int SCR_W = 240;
static const float DBM_MIN = -130.0f;   // bottom of the bar and graph
static const float DBM_MAX = -10.0f;    // top (a handheld right next to it reads about -20)

static const int BAR_X = 10, BAR_Y = 118, BAR_W = 220, BAR_H = 22;
static const int GRAPH_X = 10, GRAPH_Y = 196, GRAPH_W = 220, GRAPH_H = 104;

static uint16_t C_BG, C_PANEL, C_DIM, C_TEXT, C_ACCENT, C_GRID;

// ---------------------------------------------------------------- state
static const uint32_t SAMPLE_MS = 5;     // RSSI read interval
static const uint32_t FRAME_MS = 100;    // screen update interval
static const uint32_t SERIAL_MS = 500;   // serial print interval (same as bring-up)

static float history[GRAPH_W];           // one column per frame, ring buffer
static int histHead = 0;
static float frameMax = DBM_MIN;         // loudest sample since the last frame
static float peak = DBM_MIN;
static float noiseFloor = NAN;
static bool wasTouched = false;

// ---------------------------------------------------------------- helpers
static int dbmToX(float dbm, int width) {
  float t = (dbm - DBM_MIN) / (DBM_MAX - DBM_MIN);
  if (t < 0) t = 0;
  if (t > 1) t = 1;
  return (int)(t * (width - 1) + 0.5f);
}

// Color by how far the level sits above the noise floor, not by absolute dBm,
// so "something is transmitting" looks the same wherever the floor is.
static uint16_t levelColor(float dbm) {
  float snr = isnan(noiseFloor) ? 0 : dbm - noiseFloor;
  if (snr < 6)  return lcd.color565(40, 120, 150);   // noise
  if (snr < 20) return lcd.color565(60, 220, 90);    // weak signal
  if (snr < 40) return lcd.color565(255, 210, 60);   // strong
  return lcd.color565(255, 70, 90);                  // very strong / close by
}

// Noise floor: follows quiet readings quickly and loud ones very slowly, so a
// transmission does not drag it up.
static void updateNoiseFloor(float dbm) {
  if (isnan(noiseFloor)) { noiseFloor = dbm; return; }
  float a = (dbm < noiseFloor) ? 0.2f : 0.002f;
  noiseFloor += a * (dbm - noiseFloor);
}

static void drawStatic() {
  lcd.fillScreen(C_BG);

  // header
  lcd.fillRect(0, 0, SCR_W, 26, C_PANEL);
  lcd.setFont(&fonts::Font2);
  lcd.setTextDatum(middle_left);
  lcd.setTextColor(C_ACCENT, C_PANEL);
  lcd.drawString("H@N 433 SCANNER", 8, 13);
  lcd.setTextDatum(middle_right);
  lcd.setTextColor(C_TEXT, C_PANEL);
  char buf[24];
  snprintf(buf, sizeof(buf), "%.3f MHz", FREQ_MHZ);
  lcd.drawString(buf, SCR_W - 8, 13);

  // "dBm" unit under the big readout
  lcd.setTextDatum(top_center);
  lcd.setTextColor(C_DIM, C_BG);
  lcd.drawString("dBm", SCR_W / 2, 92);

  // bar scale
  lcd.setFont(&fonts::Font0);
  lcd.setTextColor(C_DIM, C_BG);
  for (int d = -130; d <= -10; d += 30) {
    int x = BAR_X + dbmToX(d, BAR_W);
    lcd.drawFastVLine(x, BAR_Y + BAR_H + 1, 3, C_DIM);
    lcd.setTextDatum(top_center);
    if (d == -130) lcd.setTextDatum(top_left);
    if (d == -10) lcd.setTextDatum(top_right);
    lcd.drawNumber(d, x, BAR_Y + BAR_H + 5);
  }

  // footer
  lcd.setTextDatum(bottom_center);
  lcd.drawString("Tap the screen to reset the peak", SCR_W / 2, 318);
}

static void drawReadout(float now) {
  char buf[16];
  lcd.setFont(&fonts::Font7);          // 7-segment digits, 48 px tall
  lcd.setTextDatum(top_center);
  lcd.setTextColor(levelColor(now), C_BG);
  lcd.setTextPadding(SCR_W - 4);       // overdraw the previous value, no flicker
  snprintf(buf, sizeof(buf), "%.1f", now);
  lcd.drawString(buf, SCR_W / 2, 38);
  lcd.setTextPadding(0);
}

static void drawBar(float now) {
  barSpr.fillScreen(C_GRID);
  int w = dbmToX(now, BAR_W) + 1;
  barSpr.fillRect(0, 0, w, BAR_H, levelColor(now));
  if (peak > DBM_MIN) {
    int px = dbmToX(peak, BAR_W);
    barSpr.fillRect(px > 0 ? px - 1 : 0, 0, 2, BAR_H, C_TEXT);
  }
  if (!isnan(noiseFloor)) {
    int fx = dbmToX(noiseFloor, BAR_W);
    for (int y = 0; y < BAR_H; y += 4) barSpr.drawFastVLine(fx, y, 2, C_ACCENT);
  }
  barSpr.pushSprite(BAR_X, BAR_Y);
}

static void drawStats() {
  char buf[24];
  lcd.setFont(&fonts::Font2);
  lcd.setTextPadding(110);

  lcd.setTextDatum(top_left);
  lcd.setTextColor(C_TEXT, C_BG);
  if (peak > DBM_MIN) snprintf(buf, sizeof(buf), "Peak  %.1f", peak);
  else snprintf(buf, sizeof(buf), "Peak  --");
  lcd.drawString(buf, 10, 160);

  lcd.setTextDatum(top_right);
  lcd.setTextColor(C_ACCENT, C_BG);
  snprintf(buf, sizeof(buf), "Floor  %.1f", isnan(noiseFloor) ? 0.0f : noiseFloor);
  lcd.drawString(buf, SCR_W - 10, 160);

  lcd.setTextPadding(0);
}

static int dbmToGraphY(float dbm) {
  return GRAPH_H - 1 - dbmToX(dbm, GRAPH_H);
}

static void drawGraph() {
  graphSpr.fillScreen(C_BG);
  for (int d = -120; d <= -30; d += 30) {
    int y = dbmToGraphY(d);
    for (int x = 0; x < GRAPH_W; x += 4) graphSpr.drawPixel(x, y, C_GRID);
  }
  graphSpr.drawRect(0, 0, GRAPH_W, GRAPH_H, C_GRID);

  // oldest column on the left, newest on the right
  for (int i = 0; i < GRAPH_W; i++) {
    float v = history[(histHead + i) % GRAPH_W];
    if (v <= DBM_MIN) continue;
    int y = dbmToGraphY(v);
    graphSpr.drawFastVLine(i, y, GRAPH_H - y, levelColor(v));
  }

  if (!isnan(noiseFloor)) {
    int y = dbmToGraphY(noiseFloor);
    for (int x = 0; x < GRAPH_W; x += 6) graphSpr.drawFastHLine(x, y, 3, C_ACCENT);
  }
  graphSpr.pushSprite(GRAPH_X, GRAPH_Y);

  // axis labels just above the graph
  lcd.setFont(&fonts::Font0);
  lcd.setTextColor(C_DIM, C_BG);
  lcd.setTextDatum(bottom_left);
  lcd.drawString("History (dBm, -130 to -10)", GRAPH_X, GRAPH_Y - 3);
}

static void fatal(const char *line1, const char *line2) {
  lcd.fillRect(0, 30, SCR_W, 290, C_BG);
  lcd.setFont(&fonts::Font2);
  lcd.setTextDatum(top_center);
  lcd.setTextColor(lcd.color565(255, 70, 90), C_BG);
  lcd.drawString(line1, SCR_W / 2, 120);
  lcd.setTextColor(C_DIM, C_BG);
  lcd.drawString(line2, SCR_W / 2, 145);
  while (true) delay(1000);
}

// ---------------------------------------------------------------- main
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nH@N 433MHz LoRa Module Scanner - RSSI meter");

  lcd.init();
  lcd.setRotation(2);                  // portrait, same as H@N-CYD-GBC on this board
  lcd.setBrightness(255);

  C_BG     = lcd.color565(0, 0, 0);
  C_PANEL  = lcd.color565(3, 45, 99);
  C_DIM    = lcd.color565(158, 192, 235);
  C_TEXT   = lcd.color565(238, 246, 255);
  C_ACCENT = lcd.color565(124, 232, 255);
  C_GRID   = lcd.color565(20, 32, 60);

  barSpr.setColorDepth(16);
  barSpr.createSprite(BAR_W, BAR_H);
  graphSpr.setColorDepth(16);
  graphSpr.createSprite(GRAPH_W, GRAPH_H);
  for (int i = 0; i < GRAPH_W; i++) history[i] = DBM_MIN;

  drawStatic();

  radioSPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);

  // begin() resets (skipped here - RST is NC), reads the chip version, and sets
  // default LoRa parameters.
  int st = radio.begin(FREQ_MHZ);
  if (st == RADIOLIB_ERR_NONE) {
    Serial.printf("Radio found - SX1278 responding. Listening on %.1f MHz.\n", FREQ_MHZ);
  } else {
    Serial.print("Radio NOT found. Error code: ");
    Serial.println(st);
    Serial.println("Check NSS and MISO first - those are the usual suspects.");
    char buf[40];
    snprintf(buf, sizeof(buf), "Radio NOT found (error %d)", st);
    fatal(buf, "Check the NSS and MISO wires");
  }

  radio.startReceive();   // continuous receive, so the channel can be read
}

void loop() {
  static uint32_t lastSample = 0, lastFrame = 0, lastSerial = 0;
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

    lgfx::touch_point_t tp;
    bool touched = lcd.getTouch(&tp, 1) > 0;
    if (touched && !wasTouched) peak = now;   // tap resets the peak to the current level
    wasTouched = touched;

    drawReadout(now);
    drawBar(now);
    drawStats();
    drawGraph();

    if (ms - lastSerial >= SERIAL_MS) {
      lastSerial = ms;
      Serial.print("RSSI: ");
      Serial.print(now, 1);
      Serial.println(" dBm");
    }
  }
}
