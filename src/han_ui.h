// H@N UI kit: the menu look of H@N-CYD-GBC in one file, so later projects can copy it.
//
//   - themes (10 colors each; H@N Night is from the H@N logo)
//   - uiHeader(): gunmetal gradient bar with a bronze bottom edge, title on the left and
//     an optional rounded button on the right
//   - uiTick(): call it from every loop. Twinkles stars in the header, runs a light
//     along its bronze edge and pulses the glow around the selected row (glowSet()).
//     Throttled to one frame per 40 ms.
//   - uiRow(), uiButton(), uiTabs(): rounded panels in the theme colors
//
// Ported from H@N-CYD-GBC src/main.cpp (ThemeDef, headerColor, hdrBegin, uiTick,
// uiHeader, runList's rows, pickNumber's buttons). The marquee and controller hints are
// left out. Before including: declare `lcd` (a LovyanGFX device), SCR_W and SCR_H.
#pragma once

#define RGB565(r, g, b) (uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))

struct ThemeDef {
  const char *name;
  uint32_t bg, panel, hilite, accent, bronze, text, dim;
  uint32_t hdrTop, hdrA, hdrB;   // header bar: top edge, gradient top, gradient bottom
};
static const ThemeDef THEMES[] = {
  { "H@N Night", 0x0E1622, 0x1D2A38, 0x34495E, 0x86A8C4, 0x9A8462, 0xE4ECF2, 0x8494A5, 0xC8D6E2, 0x6A7E92, 0x263444 },
  { "Neon",      0x0A0618, 0x1A1030, 0x3A185A, 0xFF3CC8, 0x2CE6FF, 0xF4ECFF, 0x9A88B8, 0xFFB0EE, 0x5A2A7A, 0x1E0E34 },
  { "DMG Green", 0x0F380F, 0x204A20, 0x306230, 0x9BBC0F, 0xB8C858, 0xC8E08C, 0x7A9A40, 0xD0F0A0, 0x4A7A2A, 0x1A3A14 },
  { "Amber CRT", 0x140A02, 0x261606, 0x4A2A08, 0xFFA830, 0xC07010, 0xFFD898, 0xA88050, 0xFFE0B0, 0x7A4A14, 0x2A1606 },
};
#define NUM_THEMES (int)(sizeof THEMES / sizeof THEMES[0])

struct ThemeColors { uint16_t bg, panel, hilite, accent, bronze, text, dim; };
static ThemeColors tc;
static uint8_t uiThemeIdx = 0;

static inline uint16_t hex565(uint32_t c) { return RGB565(c >> 16, (c >> 8) & 0xFF, c & 0xFF); }
// a..b at t/255, colors as 0xRRGGBB
static inline uint16_t mixHex(uint32_t a, uint32_t b, int t) {
  int r = ((a >> 16) * (255 - t) + (b >> 16) * t) / 255;
  int g = (((a >> 8) & 0xFF) * (255 - t) + ((b >> 8) & 0xFF) * t) / 255;
  int bl = ((a & 0xFF) * (255 - t) + (b & 0xFF) * t) / 255;
  return RGB565(r, g, bl);
}
static const ThemeDef &theme() { return THEMES[uiThemeIdx % NUM_THEMES]; }
static void applyTheme(int idx) {
  uiThemeIdx = idx % NUM_THEMES;
  const ThemeDef &t = theme();
  tc = { hex565(t.bg), hex565(t.panel), hex565(t.hilite), hex565(t.accent), hex565(t.bronze), hex565(t.text), hex565(t.dim) };
}

#define C_BG     tc.bg       // background
#define C_PANEL  tc.panel    // rows, buttons
#define C_HILITE tc.hilite   // selected row / active tab
#define C_ACCENT tc.accent
#define C_BRONZE tc.bronze   // header edge
#define C_TEXT   tc.text
#define C_DIM    tc.dim

static const int UI_HDR_H = 32;      // header height: content starts below this

// Header bar color at row y: bright top edge, gradient, bronze bottom edge.
static uint16_t headerColor(int y) {
  const ThemeDef &t = theme();
  return y == 0 ? hex565(t.hdrTop) : y >= 30 ? C_BRONZE : mixHex(t.hdrA, t.hdrB, y * 255 / 31);
}

// ---- animation: header stars, a shine along the bronze edge, the glowing selected row
#define HDR_STARS 22
static struct { int16_t x, y; uint8_t ph; } hdrStar[HDR_STARS];
static int hdrX0 = 0, hdrX1 = 0;     // free part of the header (no title or button)
static bool hdrOn = false;
static uint32_t uiAnimAt = 0;

static void hdrBegin(const char *title, bool rightButton) {
  lcd.setFont(&fonts::Font2);
  hdrX0 = 8 + lcd.textWidth(title) + 10;
  hdrX1 = rightButton ? SCR_W - 70 : SCR_W - 4;
  hdrOn = true;
  uint32_t r = esp_random();
  for (int i = 0; i < HDR_STARS; i++) {
    r = r * 1103515245u + 12345u;
    hdrStar[i].x = (r >> 8) % SCR_W;
    hdrStar[i].y = 3 + (r >> 20) % 24;
    hdrStar[i].ph = r >> 24;
  }
}

// selected row: pulsing outline with a light running along its top and bottom edges
static struct { bool on; int16_t x, y, w, h; uint16_t bg; } selGlow = { false };
static void glowSet(int x, int y, int w, int h, uint16_t bg) { selGlow = { true, (int16_t)x, (int16_t)y, (int16_t)w, (int16_t)h, bg }; }
static void glowOff() { selGlow.on = false; }

static void uiTick() {
  uint32_t now = millis();
  if (now - uiAnimAt < 40) return;
  uiAnimAt = now;
  const ThemeDef &t = theme();
  lcd.startWrite();
  if (hdrOn) {
    // stars twinkle in the free part of the header
    for (int i = 0; i < HDR_STARS; i++) {
      int x = hdrStar[i].x, y = hdrStar[i].y;
      if (x < hdrX0 || x >= hdrX1) continue;
      int b = (int)(128 + 127 * sinf(now * 0.004f + hdrStar[i].ph * 0.0245f));
      b = b < 45 ? 0 : (b - 45) * 255 / 210;
      // near the peak the color runs toward white
      uint32_t tint = t.text;
      if (b > 217) {
        int k = (b - 217) * 255 / 38;
        uint32_t cr = (t.text >> 16) & 255, cg = (t.text >> 8) & 255, cb = t.text & 255;
        cr += (255 - cr) * k / 255; cg += (255 - cg) * k / 255; cb += (255 - cb) * k / 255;
        tint = (cr << 16) | (cg << 8) | cb;
      }
      // every pixel of the footprint is painted every tick, so a star erases its own flare
      auto px = [&](int ax, int ay, int lvl) {
        if (ax < hdrX0 || ax >= hdrX1 || ay < 2 || ay > 28) return;
        uint16_t c = headerColor(ay);
        uint32_t bg = ((c >> 11) << 19) | (((c >> 5) & 63) << 10) | ((c & 31) << 3);
        lcd.drawPixel(ax, ay, mixHex(bg, tint, lvl));
      };
      int wide = b > 190 ? (b - 190) * 255 / 65 : 0;   // core swells to 2x2 at the peak
      int fa   = b > 217 ? (b - 217) * 255 / 38 : 0;   // four-point flare
      px(x, y, b);
      px(x + 1, y, wide); px(x, y + 1, wide); px(x + 1, y + 1, wide);
      px(x - 2, y, fa); px(x + 2, y, fa); px(x, y - 2, fa); px(x, y + 2, fa);
    }
    // a light runs along the bronze edge every few seconds
    static int shineLast = -100;
    int sx = (int)((now / 6) % (SCR_W + 600)) - 60;
    if (shineLast > -100) lcd.drawFastHLine(max(0, shineLast - 40), 30, 80, C_BRONZE), lcd.drawFastHLine(max(0, shineLast - 40), 31, 80, C_BRONZE);
    if (sx < SCR_W + 40) {
      for (int x = max(0, sx - 40); x < min(SCR_W, sx + 40); x++) {
        int k = 255 - abs(x - sx) * 255 / 40;
        uint16_t c = mixHex(t.bronze, 0xFFFFFF, k * 3 / 4);
        lcd.drawPixel(x, 30, c); lcd.drawPixel(x, 31, c);
      }
      shineLast = sx;
    } else {
      shineLast = -100;
    }
  }
  if (selGlow.on) {
    int p = (int)(128 + 127 * sinf(now * 0.005f));
    uint16_t edge = mixHex(t.accent, t.bronze, p);
    const int x = selGlow.x, y = selGlow.y, w = selGlow.w, h = selGlow.h;
    lcd.drawRoundRect(x, y, w, h, 5, edge);
    lcd.drawRoundRect(x - 1, y - 1, w + 2, h + 2, 6, mixHex(t.bg, t.accent, p / 3));   // soft halo
    // running light on the straight top and bottom edges
    int span = w - 12, lx = x + 6 + (int)((now / 5) % (span + 80)) - 40;
    for (int i = -24; i <= 24; i++) {
      int px = lx + i;
      if (px < x + 6 || px >= x + w - 6) continue;
      int k = 255 - abs(i) * 255 / 24;
      uint16_t c = k > 8 ? mixHex(t.accent, 0xFFFFFF, k * 2 / 3) : edge;
      lcd.drawPixel(px, y, c);
      lcd.drawPixel(px, y + h - 1, c);
    }
    int px = lx - 25;   // pixel the light left behind
    if (px >= x + 6 && px < x + w - 6) { lcd.drawPixel(px, y, edge); lcd.drawPixel(px, y + h - 1, edge); }
  }
  lcd.endWrite();
}

// Gunmetal bar: bright top edge, steel gradient, bronze bottom edge. right = label of a
// rounded button at the right end (tap area: y < UI_HDR_H, x > SCR_W - 70), or nullptr.
static void uiHeader(const char *title, const char *right = nullptr) {
  for (int y = 0; y < UI_HDR_H; y++) lcd.drawFastHLine(0, y, SCR_W, headerColor(y));
  hdrBegin(title, right != nullptr);
  lcd.setTextColor(C_TEXT);   // transparent background over the gradient
  lcd.setFont(&fonts::Font2);
  lcd.setTextDatum(middle_left);
  lcd.drawString(title, 8, 15);
  if (right) {
    lcd.fillRoundRect(SCR_W - 64, 4, 60, 24, 6, C_PANEL);
    lcd.setTextColor(C_TEXT, C_PANEL);
    lcd.setTextDatum(middle_center);
    lcd.drawString(right, SCR_W - 34, 16);
  }
  lcd.setTextDatum(top_left);
}
static bool uiHeaderButtonHit(int x, int y) { return y >= 0 && y < UI_HDR_H && x > SCR_W - 70; }

// One list row (GBC runList style): label left, value right in the accent color.
static void uiRow(int x, int y, int w, int h, const char *label, const char *value, bool hi) {
  uint16_t bg = hi ? C_HILITE : C_PANEL;
  lcd.fillRoundRect(x, y, w, h, 6, bg);
  lcd.drawRoundRect(x - 1, y - 1, w + 2, h + 2, 6, C_BG);   // clears an old halo
  if (hi) lcd.drawRoundRect(x, y, w, h, 6, C_ACCENT);
  lcd.setFont(&fonts::Font2);
  if (value && *value) {
    lcd.setTextColor(C_ACCENT, bg);
    lcd.setTextDatum(middle_right);
    lcd.drawString(value, x + w - 8, y + h / 2);
  }
  lcd.setTextColor(C_TEXT, bg);
  lcd.setTextDatum(middle_left);
  lcd.drawString(label, x + 8, y + h / 2);
  lcd.setTextDatum(top_left);
}

// A rounded button with centered text (pickNumber style).
static void uiButton(int x, int y, int w, int h, const char *label, uint16_t fill, uint16_t edge, uint16_t fg) {
  lcd.fillRoundRect(x, y, w, h, 6, fill);
  if (edge != fill) lcd.drawRoundRect(x, y, w, h, 6, edge);
  lcd.setFont(&fonts::Font2);
  lcd.setTextColor(fg, fill);
  lcd.setTextDatum(middle_center);
  lcd.drawString(label, x + w / 2, y + h / 2);
  lcd.setTextDatum(top_left);
}

// A full-screen list (GBC runList style, touch only, no scrolling): header with a Back
// button, one row per item, label left and value right. Blocks until a row is tapped;
// returns its index, or -1 for Back. tap() reports the next finished tap, if any.
static int uiList(const char *title, const char *const *labels, const char *const *values, int n,
                  bool (*tap)(int &x, int &y)) {
  const int y0 = UI_HDR_H + 4, pitch = min(34, (SCR_H - y0) / max(1, n)), rowH = pitch - 2;
  glowOff();
  lcd.fillScreen(C_BG);
  uiHeader(title, "Back");
  for (int i = 0; i < n; i++) uiRow(6, y0 + i * pitch, SCR_W - 12, rowH, labels[i], values ? values[i] : nullptr, false);
  int x, y;
  while (tap(x, y)) {}                            // drop a tap left over from before
  while (true) {
    uiTick();
    if (!tap(x, y)) { delay(10); continue; }
    if (uiHeaderButtonHit(x, y)) { hdrOn = false; return -1; }
    int i = (y - y0) / pitch;
    if (y >= y0 && i < n) {                       // light the row briefly, then go
      uiRow(6, y0 + i * pitch, SCR_W - 12, rowH, labels[i], values ? values[i] : nullptr, true);
      delay(120);
      hdrOn = false;
      return i;
    }
  }
}

// Tab bar across the full width at y..y+h: equal tabs, the active one highlighted.
static void uiTabs(const char *const *names, int n, int active, int y, int h) {
  lcd.fillRect(0, y, SCR_W, h, C_BG);
  int w = SCR_W / n;
  for (int i = 0; i < n; i++) {
    bool on = i == active;
    uiButton(i * w + 4, y + 3, w - 8, h - 6, names[i], on ? C_HILITE : C_PANEL,
             on ? C_ACCENT : C_PANEL, on ? C_TEXT : C_DIM);
  }
}
