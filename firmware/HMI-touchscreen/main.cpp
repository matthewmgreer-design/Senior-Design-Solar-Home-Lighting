#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_FT6206.h>

// PIN MAPPING

// TFT SPI interface
#define TFT_CS    5
#define TFT_DC    2
#define TFT_RST   4
#define TFT_CLK   18
#define TFT_MOSI  20
#define TFT_MISO  19   // connected but unused
#define TFT_BL    14

// Touchscreen FT6206
#define TOUCH_SDA 21
#define TOUCH_SCL 11

// UART between HMI ESP32 and MCU
#define UART_TX   16
#define UART_RX   17
#define UART_BAUD 115200

// Power LED
#define POWER_LED 7

// Touch calibration
// rawX: ~10 (top) to ~237 (bottom)  -> vertical
// rawY: ~320 (left) to ~1 (right)   -> horizontal, reversed
const int RAWX_MIN = 10;   // top
const int RAWX_MAX = 237;  // bottom
const int RAWY_MIN = 1;    // right
const int RAWY_MAX = 320;  // left

// Timing
const unsigned long BOOT_SCREEN_MS = 2000UL;
const unsigned long LOCAL_ECHO_WINDOW_MS = 1200UL;
const unsigned long SCREEN_SLEEP_MS = 30000UL;

// ILI9341 commands for sleep/display power save
const uint8_t ILI9341_CMD_SLPIN   = 0x10;
const uint8_t ILI9341_CMD_SLPOUT  = 0x11;
const uint8_t ILI9341_CMD_DISPOFF = 0x28;
const uint8_t ILI9341_CMD_DISPON  = 0x29;

Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);
Adafruit_FT6206  touch;
// UI Layout
static const int SCREEN_W = 320;
static const int SCREEN_H = 240;

// Main light buttons
static const int MAIN_X   = 14;
static const int MAIN_W   = 160;
static const int BTN_H    = 52;
static const int BTN_R    = 14;
static const int BTN_GAP  = 16;

// Small Auto/Man buttons
static const int AUTO_X   = 182;
static const int AUTO_W   = 52;
static const int AUTO_R   = 12;

// Right status panel
static const int PANEL_X = 240;
static const int PANEL_Y = 30;
static const int PANEL_W = 74;
static const int PANEL_H = 170;

// Power save toggle
static const int PS_LABEL_X  = 239;
static const int PS_LABEL_Y  = 194;
static const int PS_TOGGLE_X = 244;
static const int PS_TOGGLE_Y = 206;
static const int PS_TOGGLE_W = 60;
static const int PS_TOGGLE_H = 20;
static const int PS_KNOB_D   = 16;

// Battery layout inside status panel
static const int BATT_A_LABEL_Y = PANEL_Y + 34;
static const int BATT_A_ICON_Y  = PANEL_Y + 46;
static const int BATT_A_TEXT_Y  = PANEL_Y + 64;

static const int BATT_B_LABEL_Y = PANEL_Y + 106;
static const int BATT_B_ICON_Y  = PANEL_Y + 118;
static const int BATT_B_TEXT_Y  = PANEL_Y + 136;

// Data structures
struct MainButton {
  int x, y, w, h, r;
  const char* name;
};

struct ModeButton {
  int x, y, w, h, r;
  const char* name;
};

MainButton mainButtons[3] = {
  { MAIN_X,  40, MAIN_W, BTN_H, BTN_R, "Porch" },
  { MAIN_X,  40 + (BTN_H + BTN_GAP), MAIN_W, BTN_H, BTN_R, "Foyer" },
  { MAIN_X,  40 + 2 * (BTN_H + BTN_GAP), MAIN_W, BTN_H, BTN_R, "Security" }
};

ModeButton modeButtons[3] = {
  { AUTO_X,  40, AUTO_W, BTN_H, AUTO_R, "PorchAuto" },
  { AUTO_X,  40 + (BTN_H + BTN_GAP), AUTO_W, BTN_H, AUTO_R, "FoyerAuto" },
  { AUTO_X,  40 + 2 * (BTN_H + BTN_GAP), AUTO_W, BTN_H, AUTO_R, "SecurityAuto" }
};

// System state
bool statePorch    = false;
bool stateFoyer    = false;
bool stateSecurity = false;

// AUTO starts enabled
bool autoPorch     = true;
bool autoFoyer     = true;
bool autoSecurity  = true;

// Tracks whether the current ON state came from MCU auto logic
bool mcuForcedPorch    = false;
bool mcuForcedFoyer    = false;
bool mcuForcedSecurity = false;

// Track recent local touchscreen commands so an MCU echo does not get
// mistaken as an MCU-forced ON command.
bool pendingLocalState[3] = { false, false, false };
bool pendingLocalValid[3] = { false, false, false };
unsigned long pendingLocalMs[3] = { 0, 0, 0 };

// Battery display values
int packAPercent = 0;
int packBPercent = 0;

// Track last drawn values independently
int lastPackAPercent = -1;
int lastPackBPercent = -1;

// Boot/UI state
bool uiActive = false;
unsigned long bootStartMs = 0;

// Sleep state
bool screenSleeping = false;
bool powerSaveEnabled = true;
unsigned long lastInteractionMs = 0;

// Forward declarations
void drawOperatingScreen();
void wakeDisplay();
void noteInteraction();
void drawBatteryA(bool force = false);
void drawBatteryB(bool force = false);
void drawAllBatteryInfo(bool force = false);

// Color helpers
uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return tft.color565(r, g, b);
}

uint16_t panelBgColor() {
  return rgb565(12, 16, 28);
}

uint16_t gradientColorAtY(int y) {
  y = constrain(y, 0, SCREEN_H - 1);
  float t = (float)y / (SCREEN_H - 1);
  uint8_t r = (uint8_t)(8 + (10 * t));
  uint8_t g = (uint8_t)(14 + (18 * t));
  uint8_t b = (uint8_t)(36 + (34 * t));
  return rgb565(r, g, b);
}

void drawGradientBackground() {
  for (int y = 0; y < SCREEN_H; y++) {
    tft.drawFastHLine(0, y, SCREEN_W, gradientColorAtY(y));
  }
}

void drawGradientRect(int x, int y, int w, int h) {
  int x2 = min(x + w, SCREEN_W);
  int y2 = min(y + h, SCREEN_H);
  x = max(x, 0);
  y = max(y, 0);

  for (int yy = y; yy < y2; yy++) {
    tft.drawFastHLine(x, yy, x2 - x, gradientColorAtY(yy));
  }
}

// Boot art / logo
void drawPixelBlock(int x, int y, int s, uint16_t color) {
  tft.fillRect(x, y, s, s, color);
}

void drawBootLogo8Bit(int x, int y, int scale) {
  uint16_t sun      = ILI9341_YELLOW;
  uint16_t ray      = rgb565(255, 185, 40);
  uint16_t roof     = rgb565(50, 170, 255);
  uint16_t panel    = rgb565(20, 90, 190);
  uint16_t body     = rgb565(160, 220, 255);
  uint16_t battery  = ILI9341_GREEN;
  uint16_t outline  = ILI9341_WHITE;
  uint16_t accent   = ILI9341_CYAN;

  int sx = x + 6 * scale;
  int sy = y + 1 * scale;
  for (int r = 0; r < 3; r++) {
    drawPixelBlock(sx + (r + 1) * scale, sy + 0 * scale, scale, sun);
    drawPixelBlock(sx + 0 * scale, sy + (r + 1) * scale, scale, sun);
    drawPixelBlock(sx + 4 * scale, sy + (r + 1) * scale, scale, sun);
    drawPixelBlock(sx + (r + 1) * scale, sy + 4 * scale, scale, sun);
  }
  drawPixelBlock(sx + 1 * scale, sy + 1 * scale, scale, sun);
  drawPixelBlock(sx + 2 * scale, sy + 1 * scale, scale, sun);
  drawPixelBlock(sx + 3 * scale, sy + 1 * scale, scale, sun);
  drawPixelBlock(sx + 1 * scale, sy + 2 * scale, scale, sun);
  drawPixelBlock(sx + 2 * scale, sy + 2 * scale, scale, sun);
  drawPixelBlock(sx + 3 * scale, sy + 2 * scale, scale, sun);
  drawPixelBlock(sx + 1 * scale, sy + 3 * scale, scale, sun);
  drawPixelBlock(sx + 2 * scale, sy + 3 * scale, scale, sun);
  drawPixelBlock(sx + 3 * scale, sy + 3 * scale, scale, sun);

  drawPixelBlock(sx + 2 * scale, sy - 2 * scale, scale, ray);
  drawPixelBlock(sx + 2 * scale, sy - 1 * scale, scale, ray);
  drawPixelBlock(sx + 2 * scale, sy + 5 * scale, scale, ray);
  drawPixelBlock(sx + 2 * scale, sy + 6 * scale, scale, ray);
  drawPixelBlock(sx - 2 * scale, sy + 2 * scale, scale, ray);
  drawPixelBlock(sx - 1 * scale, sy + 2 * scale, scale, ray);
  drawPixelBlock(sx + 5 * scale, sy + 2 * scale, scale, ray);
  drawPixelBlock(sx + 6 * scale, sy + 2 * scale, scale, ray);
  drawPixelBlock(sx - 1 * scale, sy - 1 * scale, scale, ray);
  drawPixelBlock(sx + 5 * scale, sy - 1 * scale, scale, ray);
  drawPixelBlock(sx - 1 * scale, sy + 5 * scale, scale, ray);
  drawPixelBlock(sx + 5 * scale, sy + 5 * scale, scale, ray);

  int hx = x + 2 * scale;
  int hy = y + 10 * scale;

  for (int i = 0; i < 8; i++) {
    drawPixelBlock(hx + (2 + i) * scale, hy + 0 * scale, scale, roof);
  }
  for (int i = 0; i < 10; i++) {
    drawPixelBlock(hx + (1 + i) * scale, hy + 1 * scale, scale, roof);
  }
  for (int i = 0; i < 12; i++) {
    drawPixelBlock(hx + (0 + i) * scale, hy + 2 * scale, scale, outline);
  }

  for (int row = 0; row < 2; row++) {
    for (int col = 0; col < 6; col++) {
      drawPixelBlock(hx + (3 + col) * scale, hy + row * scale, scale, panel);
    }
  }
  for (int col = 0; col < 6; col += 2) {
    drawPixelBlock(hx + (3 + col) * scale, hy + 0 * scale, scale, accent);
    drawPixelBlock(hx + (3 + col) * scale, hy + 1 * scale, scale, accent);
  }

  for (int row = 0; row < 6; row++) {
    for (int col = 0; col < 10; col++) {
      drawPixelBlock(hx + (1 + col) * scale, hy + (3 + row) * scale, scale, body);
    }
  }

  for (int col = 0; col < 10; col++) {
    drawPixelBlock(hx + (1 + col) * scale, hy + 3 * scale, scale, outline);
    drawPixelBlock(hx + (1 + col) * scale, hy + 8 * scale, scale, outline);
  }
  for (int row = 0; row < 6; row++) {
    drawPixelBlock(hx + 1 * scale, hy + (3 + row) * scale, scale, outline);
    drawPixelBlock(hx + 10 * scale, hy + (3 + row) * scale, scale, outline);
  }

  for (int row = 0; row < 3; row++) {
    drawPixelBlock(hx + 5 * scale, hy + (6 + row) * scale, scale, rgb565(110, 70, 30));
    drawPixelBlock(hx + 6 * scale, hy + (6 + row) * scale, scale, rgb565(110, 70, 30));
  }

  drawPixelBlock(hx + 3 * scale, hy + 5 * scale, scale, ILI9341_WHITE);
  drawPixelBlock(hx + 4 * scale, hy + 5 * scale, scale, ILI9341_WHITE);
  drawPixelBlock(hx + 3 * scale, hy + 6 * scale, scale, ILI9341_WHITE);
  drawPixelBlock(hx + 4 * scale, hy + 6 * scale, scale, ILI9341_WHITE);

  int bx = hx + 13 * scale;
  int by = hy + 4 * scale;

  for (int row = 0; row < 5; row++) {
    for (int col = 0; col < 4; col++) {
      drawPixelBlock(bx + col * scale, by + row * scale, scale, battery);
    }
  }
  for (int col = 0; col < 4; col++) {
    drawPixelBlock(bx + col * scale, by + 0 * scale, scale, outline);
    drawPixelBlock(bx + col * scale, by + 4 * scale, scale, outline);
  }
  for (int row = 0; row < 5; row++) {
    drawPixelBlock(bx + 0 * scale, by + row * scale, scale, outline);
    drawPixelBlock(bx + 3 * scale, by + row * scale, scale, outline);
  }
  drawPixelBlock(bx + 1 * scale, by - 1 * scale, scale, outline);
  drawPixelBlock(bx + 2 * scale, by - 1 * scale, scale, outline);

  drawPixelBlock(bx + 1 * scale, by + 2 * scale, scale, outline);
  drawPixelBlock(bx + 2 * scale, by + 2 * scale, scale, outline);
  drawPixelBlock(bx + 1 * scale, by + 1 * scale, scale, outline);
  drawPixelBlock(bx + 1 * scale, by + 3 * scale, scale, outline);
}

void drawBootLinePair(int logoTop, int logoBottom) {
  uint16_t c1 = rgb565(40, 160, 255);
  uint16_t c2 = rgb565(0, 255, 255);

  int topLineY = logoTop - 10;
  int botLineY = logoBottom + 10;

  tft.drawFastHLine(28, topLineY - 1, 264, c1);
  tft.drawFastHLine(20, topLineY,     280, c2);
  tft.drawFastHLine(28, topLineY + 1, 264, c1);

  tft.drawFastHLine(28, botLineY - 1, 264, c1);
  tft.drawFastHLine(20, botLineY,     280, c2);
  tft.drawFastHLine(28, botLineY + 1, 264, c1);
}

void drawBootScreen() {
  drawGradientBackground();

  int scale = 4;
  int logoW = 19 * scale + 16;
  int logoH = 20 * scale;
  int logoX = (SCREEN_W - logoW) / 2;
  int logoY = 32;

  drawBootLinePair(logoY, logoY + logoH);
  drawBootLogo8Bit(logoX, logoY, scale);

  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);

  String line1 = "PhotonPhools Official";
  String line2 = "Solar System";

  int x1 = (SCREEN_W - (line1.length() * 12)) / 2;
  int x2 = (SCREEN_W - (line2.length() * 12)) / 2;

  tft.setCursor(x1, 178);
  tft.print(line1);

  tft.setCursor(x2, 204);
  tft.print(line2);
}

// Status panel
void drawStatusPanelFrame() {
  tft.fillRoundRect(PANEL_X, PANEL_Y, PANEL_W, PANEL_H, 10, panelBgColor());
  tft.drawRoundRect(PANEL_X, PANEL_Y, PANEL_W, PANEL_H, 10, ILI9341_WHITE);

  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE, panelBgColor());
  tft.setCursor(PANEL_X + 10, PANEL_Y + 10);
  tft.print("Status");
}

// Icons
void drawBatteryIcon(int x, int y, int percent) {
  int bw = 28, bh = 14;
  int capW = 3, capH = 6;

  tft.drawRect(x, y, bw, bh, ILI9341_WHITE);
  tft.drawRect(x + bw, y + (bh - capH) / 2, capW, capH, ILI9341_WHITE);

  int innerX = x + 2;
  int innerY = y + 2;
  int innerW = bw - 4;
  int innerH = bh - 4;

  tft.fillRect(innerX, innerY, innerW, innerH, panelBgColor());

  percent = constrain(percent, 0, 100);
  int fillW = (innerW * percent) / 100;

  uint16_t fillColor = (percent < 20) ? ILI9341_RED : (percent < 50) ? ILI9341_YELLOW : ILI9341_GREEN;
  if (fillW > 0) tft.fillRect(innerX, innerY, fillW, innerH, fillColor);
}

void drawBulbIcon(int x, int y, bool on) {
  int cx = x + 10;
  int cy = y + 14;
  int r = 10;

  if (on) {
    tft.fillCircle(cx, cy, r, ILI9341_YELLOW);
    tft.drawCircle(cx, cy, r, ILI9341_WHITE);
  } else {
    tft.drawCircle(cx, cy, r, ILI9341_WHITE);
  }

  int baseX = cx - 6;
  int baseY = cy + r - 1;
  int baseW = 12;
  int baseH = 10;

  if (on) {
    tft.fillRect(baseX, baseY, baseW, baseH, rgb565(160, 160, 160));
    tft.drawRect(baseX, baseY, baseW, baseH, ILI9341_WHITE);
  } else {
    tft.drawRect(baseX, baseY, baseW, baseH, ILI9341_WHITE);
    tft.drawLine(cx - 5, cy, cx + 5, cy, ILI9341_WHITE);
  }
}

// State access helpers
bool getMainStateByIndex(int i) {
  if (i == 0) return statePorch;
  if (i == 1) return stateFoyer;
  return stateSecurity;
}

void setMainStateByIndex(int i, bool s) {
  if (i == 0) statePorch = s;
  else if (i == 1) stateFoyer = s;
  else stateSecurity = s;
}

bool getAutoStateByIndex(int i) {
  if (i == 0) return autoPorch;
  if (i == 1) return autoFoyer;
  return autoSecurity;
}

void setAutoStateByIndex(int i, bool s) {
  if (i == 0) autoPorch = s;
  else if (i == 1) autoFoyer = s;
  else autoSecurity = s;
}

bool getMCUForcedStateByIndex(int i) {
  if (i == 0) return mcuForcedPorch;
  if (i == 1) return mcuForcedFoyer;
  return mcuForcedSecurity;
}

void setMCUForcedStateByIndex(int i, bool s) {
  if (i == 0) mcuForcedPorch = s;
  else if (i == 1) mcuForcedFoyer = s;
  else mcuForcedSecurity = s;
}

bool isMainButtonLocked(int i) {
  return getAutoStateByIndex(i) && getMainStateByIndex(i) && getMCUForcedStateByIndex(i);
}

bool isRecentLocalEchoMatch(int i, bool incomingState) {
  if (!pendingLocalValid[i]) return false;

  unsigned long age = millis() - pendingLocalMs[i];
  if (age > LOCAL_ECHO_WINDOW_MS) {
    pendingLocalValid[i] = false;
    return false;
  }

  if (pendingLocalState[i] == incomingState) {
    pendingLocalValid[i] = false;
    return true;
  }

  return false;
}

void markLocalCommand(int i, bool state) {
  pendingLocalState[i] = state;
  pendingLocalValid[i] = true;
  pendingLocalMs[i] = millis();
}

// Power save
void sleepDisplay() {
  if (screenSleeping) return;

  tft.writeCommand(ILI9341_CMD_DISPOFF);
  delay(20);
  tft.writeCommand(ILI9341_CMD_SLPIN);
  delay(120);
  digitalWrite(TFT_BL, LOW);

  screenSleeping = true;
}

void wakeDisplay() {
  if (!screenSleeping) return;

  digitalWrite(TFT_BL, HIGH);
  delay(10);
  tft.writeCommand(ILI9341_CMD_SLPOUT);
  delay(120);
  tft.writeCommand(ILI9341_CMD_DISPON);
  delay(20);

  screenSleeping = false;

  lastPackAPercent = -1;
  lastPackBPercent = -1;
  drawOperatingScreen();
}

void noteInteraction() {
  lastInteractionMs = millis();
}

// Drawing buttons
void drawMainButton(int i) {
  const MainButton& b = mainButtons[i];
  bool isOn = getMainStateByIndex(i);
  bool locked = isMainButtonLocked(i);

  uint16_t fill = isOn ? ILI9341_GREEN : ILI9341_RED;

  tft.fillRoundRect(b.x + 3, b.y + 3, b.w, b.h, b.r, rgb565(0, 0, 0));
  tft.fillRoundRect(b.x, b.y, b.w, b.h, b.r, fill);
  tft.drawRoundRect(b.x, b.y, b.w, b.h, b.r, ILI9341_WHITE);

  drawBulbIcon(b.x + 10, b.y + 10, isOn);

  tft.setTextColor(ILI9341_BLACK, fill);
  tft.setTextSize(2);

  int textX = b.x + 44;
  tft.setCursor(textX, b.y + 10);
  tft.print(b.name);

  tft.setCursor(textX, b.y + 30);
  tft.print(isOn ? "ON" : "OFF");

  if (locked) {
    tft.fillRoundRect(b.x + b.w - 44, b.y + 4, 36, 14, 6, rgb565(20, 20, 20));
    tft.drawRoundRect(b.x + b.w - 44, b.y + 4, 36, 14, 6, ILI9341_WHITE);
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_WHITE, rgb565(20, 20, 20));
    tft.setCursor(b.x + b.w - 39, b.y + 8);
    tft.print("AUTO");
  }
}

void drawModeButton(int i) {
  const ModeButton& b = modeButtons[i];
  bool isAuto = getAutoStateByIndex(i);

  uint16_t fill = isAuto ? ILI9341_GREEN : ILI9341_RED;

  tft.fillRoundRect(b.x + 2, b.y + 2, b.w, b.h, b.r, rgb565(0, 0, 0));
  tft.fillRoundRect(b.x, b.y, b.w, b.h, b.r, fill);
  tft.drawRoundRect(b.x, b.y, b.w, b.h, b.r, ILI9341_WHITE);

  tft.setTextColor(ILI9341_BLACK, fill);

  tft.setTextSize(1);
  String line1 = isAuto ? "AUTO" : "Man";
  int textX1 = b.x + (b.w - (line1.length() * 6)) / 2;
  tft.setCursor(textX1, b.y + 14);
  tft.print(line1);

  String line2 = "Mode";
  int textX2 = b.x + (b.w - (line2.length() * 6)) / 2;
  tft.setCursor(textX2, b.y + 31);
  tft.print(line2);
}

void drawPowerSaveToggle() {
  drawGradientRect(PS_LABEL_X - 2, PS_LABEL_Y - 2, 48, 10);
  drawGradientRect(PS_TOGGLE_X - 2, PS_TOGGLE_Y - 2, PS_TOGGLE_W + 4, PS_TOGGLE_H + 4);

  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(PS_LABEL_X, PS_LABEL_Y);
  tft.print("PWR SAVE");

  uint16_t slotColor = powerSaveEnabled ? rgb565(40, 210, 120) : rgb565(90, 90, 90);
  uint16_t knobColor = ILI9341_WHITE;
  uint16_t borderColor = ILI9341_WHITE;

  tft.fillRoundRect(PS_TOGGLE_X, PS_TOGGLE_Y, PS_TOGGLE_W, PS_TOGGLE_H, PS_TOGGLE_H / 2, slotColor);
  tft.drawRoundRect(PS_TOGGLE_X, PS_TOGGLE_Y, PS_TOGGLE_W, PS_TOGGLE_H, PS_TOGGLE_H / 2, borderColor);

  int knobX = powerSaveEnabled
    ? (PS_TOGGLE_X + PS_TOGGLE_W - PS_KNOB_D - 2)
    : (PS_TOGGLE_X + 2);

  int knobY = PS_TOGGLE_Y + (PS_TOGGLE_H - PS_KNOB_D) / 2;

  tft.fillCircle(knobX + PS_KNOB_D / 2, knobY + PS_KNOB_D / 2, PS_KNOB_D / 2, knobColor);
  tft.drawCircle(knobX + PS_KNOB_D / 2, knobY + PS_KNOB_D / 2, PS_KNOB_D / 2, rgb565(50, 50, 50));
}

bool inMainButton(const MainButton& b, int x, int y) {
  return (x >= b.x && x <= b.x + b.w &&
          y >= b.y && y <= b.y + b.h);
}

bool inModeButton(const ModeButton& b, int x, int y) {
  return (x >= b.x && x <= b.x + b.w &&
          y >= b.y && y <= b.y + b.h);
}

bool inPowerSaveToggle(int x, int y) {
  return (x >= PS_TOGGLE_X && x <= PS_TOGGLE_X + PS_TOGGLE_W &&
          y >= PS_TOGGLE_Y && y <= PS_TOGGLE_Y + PS_TOGGLE_H);
}

// Battery drawing
void drawBatteryA(bool force) {
  if (!force && packAPercent == lastPackAPercent) return;

  tft.fillRect(PANEL_X + 6, PANEL_Y + 24, PANEL_W - 12, 60, panelBgColor());

  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE, panelBgColor());
  tft.setCursor(PANEL_X + 8, BATT_A_LABEL_Y);
  tft.print("BatteryA");

  drawBatteryIcon(PANEL_X + 8, BATT_A_ICON_Y, packAPercent);

  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE, panelBgColor());
  tft.fillRect(PANEL_X + 8, BATT_A_TEXT_Y, PANEL_W - 18, 16, panelBgColor());
  tft.setCursor(PANEL_X + 8, BATT_A_TEXT_Y);
  tft.print(packAPercent);
  tft.print("%");

  lastPackAPercent = packAPercent;
}

void drawBatteryB(bool force) {
  if (!force && packBPercent == lastPackBPercent) return;

  tft.fillRect(PANEL_X + 6, PANEL_Y + 96, PANEL_W - 12, 60, panelBgColor());

  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE, panelBgColor());
  tft.setCursor(PANEL_X + 8, BATT_B_LABEL_Y);
  tft.print("BatteryB");

  drawBatteryIcon(PANEL_X + 8, BATT_B_ICON_Y, packBPercent);

  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE, panelBgColor());
  tft.fillRect(PANEL_X + 8, BATT_B_TEXT_Y, PANEL_W - 18, 16, panelBgColor());
  tft.setCursor(PANEL_X + 8, BATT_B_TEXT_Y);
  tft.print(packBPercent);
  tft.print("%");

  lastPackBPercent = packBPercent;
}

void drawAllBatteryInfo(bool force) {
  drawBatteryA(force);
  drawBatteryB(force);
}

// UART send + parsing
void sendState(const char* name, bool on) {
  String msg = String(name) + "=" + (on ? "1" : "0");
  Serial2.println(msg);
  Serial.print("Sent: ");
  Serial.println(msg);
}

bool extractBoolAssignment(const String& line, const char* key, bool& outState) {
  String s = line;
  s.replace("\r", "");
  String low = s;
  low.toLowerCase();

  String k = String(key);
  k.toLowerCase();

  int pos = low.indexOf(k);
  if (pos < 0) return false;

  int eq = low.indexOf('=', pos + k.length());
  if (eq < 0) return false;

  int i = eq + 1;
  while (i < (int)low.length() && low[i] == ' ') i++;
  if (i >= (int)low.length()) return false;

  char c = low[i];
  if (c == '0') { outState = false; return true; }
  if (c == '1') { outState = true;  return true; }
  return false;
}

bool extractEqualsPercent(const String& line, const char* key, int& outPct) {
  String s = line;
  s.replace("\r", "");
  s.trim();

  int eq = s.indexOf('=');
  if (eq < 0) return false;

  String lhs = s.substring(0, eq);
  String rhs = s.substring(eq + 1);

  lhs.trim();
  rhs.trim();
  lhs.toUpperCase();

  String target = String(key);
  target.toUpperCase();

  if (lhs != target) return false;
  if (rhs.length() == 0) return false;

  for (int i = 0; i < (int)rhs.length(); i++) {
    if (rhs[i] < '0' || rhs[i] > '9') return false;
  }

  int val = rhs.toInt();
  if (val < 0 || val > 100) return false;

  outPct = val;
  return true;
}

void refreshLightAndModeRow(int i) {
  if (!uiActive || screenSleeping) return;
  drawMainButton(i);
  drawModeButton(i);
}

void applyIncomingMainLightState(int i, bool newState) {
  bool wasLocalEcho = isRecentLocalEchoMatch(i, newState);

  setMainStateByIndex(i, newState);

  if (!newState) {
    setMCUForcedStateByIndex(i, false);
  } else {
    if (wasLocalEcho) {
      setMCUForcedStateByIndex(i, false);
    } else {
      setMCUForcedStateByIndex(i, getAutoStateByIndex(i));

      // Wake the screen if the MCU automatically turned a light on,
      // and reset the timeout so it stays on for the full sleep interval.
      if (powerSaveEnabled && screenSleeping && getAutoStateByIndex(i)) {
        wakeDisplay();
        noteInteraction();
      }
    }
  }

  refreshLightAndModeRow(i);
}

void handleIncomingUART() {
  while (Serial2.available()) {
    String line = Serial2.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    Serial.print("RX: ");
    Serial.println(line);

    int p;
    if (extractEqualsPercent(line, "PACKA", p)) {
      if (packAPercent != p) {
        packAPercent = p;
        if (uiActive && !screenSleeping) drawBatteryA(false);
      }
      continue;
    }

    if (extractEqualsPercent(line, "PACKB", p)) {
      if (packBPercent != p) {
        packBPercent = p;
        if (uiActive && !screenSleeping) drawBatteryB(false);
      }
      continue;
    }

    bool st;

    if (extractBoolAssignment(line, "PorchAuto", st)) {
      if (autoPorch != st) {
        autoPorch = st;
        refreshLightAndModeRow(0);
      }
      continue;
    }
    if (extractBoolAssignment(line, "FoyerAuto", st)) {
      if (autoFoyer != st) {
        autoFoyer = st;
        refreshLightAndModeRow(1);
      }
      continue;
    }
    if (extractBoolAssignment(line, "SecurityAuto", st)) {
      if (autoSecurity != st) {
        autoSecurity = st;
        refreshLightAndModeRow(2);
      }
      continue;
    }

    if (extractBoolAssignment(line, "Porch", st)) {
      applyIncomingMainLightState(0, st);
      continue;
    }
    if (extractBoolAssignment(line, "Foyer", st)) {
      applyIncomingMainLightState(1, st);
      continue;
    }
    if (extractBoolAssignment(line, "Security", st)) {
      applyIncomingMainLightState(2, st);
      continue;
    }
  }
}

// Touch handling
void handleTouch(int sx, int sy) {
  noteInteraction();

  if (inPowerSaveToggle(sx, sy)) {
    powerSaveEnabled = !powerSaveEnabled;
    drawPowerSaveToggle();
    return;
  }

  for (int i = 0; i < 3; i++) {
    if (inMainButton(mainButtons[i], sx, sy)) {
      if (isMainButtonLocked(i)) {
        return;
      }

      bool cur = getMainStateByIndex(i);
      bool next = !cur;

      setMainStateByIndex(i, next);
      setMCUForcedStateByIndex(i, false);
      markLocalCommand(i, next);

      drawMainButton(i);
      sendState(mainButtons[i].name, next);
      return;
    }
  }

  for (int i = 0; i < 3; i++) {
    if (inModeButton(modeButtons[i], sx, sy)) {
      bool cur = getAutoStateByIndex(i);
      bool next = !cur;

      setAutoStateByIndex(i, next);

      if (!next) {
        setMCUForcedStateByIndex(i, false);
      }

      refreshLightAndModeRow(i);
      sendState(modeButtons[i].name, next);
      return;
    }
  }
}

// Full normal UI draw
void drawOperatingScreen() {
  drawGradientBackground();

  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(10, 10);
  tft.print("PhotonPhools Solar System");

  drawStatusPanelFrame();
  drawAllBatteryInfo(true);

  drawMainButton(0);
  drawMainButton(1);
  drawMainButton(2);

  drawModeButton(0);
  drawModeButton(1);
  drawModeButton(2);

  drawPowerSaveToggle();
}

// Setup / Loop
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(POWER_LED, OUTPUT);
  digitalWrite(POWER_LED, HIGH);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);

  Wire.begin(TOUCH_SDA, TOUCH_SCL);
  touch.begin(40);

  SPI.begin(TFT_CLK, -1, TFT_MOSI, TFT_CS);
  tft.begin(4000000);
  tft.setRotation(1);

  bootStartMs = millis();
  lastInteractionMs = millis();

  drawBootScreen();
}

void loop() {
  handleIncomingUART();

  if (!uiActive && (millis() - bootStartMs >= BOOT_SCREEN_MS)) {
    uiActive = true;
    lastPackAPercent = -1;
    lastPackBPercent = -1;
    drawOperatingScreen();
    noteInteraction();
  }

  if (!uiActive) return;

  if (powerSaveEnabled && !screenSleeping && (millis() - lastInteractionMs >= SCREEN_SLEEP_MS)) {
    sleepDisplay();
  }

  if (touch.touched()) {
    TS_Point p = touch.getPoint();

    int rawX = p.x;
    int rawY = p.y;

    int sx = map(rawY, RAWY_MAX, RAWY_MIN, 0, tft.width());
    int sy = map(rawX, RAWX_MIN, RAWX_MAX, 0, tft.height());

    sx = constrain(sx, 0, tft.width() - 1);
    sy = constrain(sy, 0, tft.height() - 1);

    if (screenSleeping) {
      wakeDisplay();
      noteInteraction();
      delay(120);
      while (touch.touched()) delay(10);
      return;
    }

    handleTouch(sx, sy);

    delay(80);
    while (touch.touched()) delay(10);
  }
}