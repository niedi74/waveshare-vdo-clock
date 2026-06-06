// Waveshare ESP32-S3-Touch-LCD-2.8C — VDO Quartz-Zeit Clock
// PHASE 1: Display Bring-up (Arduino_GFX, ST7701 RGB 480x480)
//
// Display: ST7701 RGB-Parallel. CS/RST des LCD + Touch-RST laufen ueber
// einen PCA9554 I2C-Expander (@0x20). Arduino_GFX kann den Expander-CS
// nicht selbst steuern -> wir halten CS dauerhaft LOW per Expander und
// fuehren die Reset-Sequenz manuell aus, bevor gfx->begin() laeuft.
//
// HINWEIS: Die ST7701-Init-Sequenz ist board-spezifisch. Hier startet sie
// mit st7701_type1_init_operations als Naeherung. Falls das Bild fehlt
// oder Farben/Versatz falsch sind, brauchen wir die exakte Init aus dem
// offiziellen Waveshare-2.8C-Demo (Arduino).

#include <Arduino.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>

// ---- I2C / PCA9554 Expander ----
#define PIN_I2C_SDA   15
#define PIN_I2C_SCL   7
#define PCA9554_ADDR  0x20
#define PCA9554_OUTPUT 0x01
#define PCA9554_CONFIG 0x03
#define EXIO_LCD_RST  (1 << 0)
#define EXIO_TP_RST   (1 << 1)
#define EXIO_LCD_CS   (1 << 2)

// ---- LCD ST7701 Init-SPI (3-wire; CS extern via Expander) ----
#define PIN_LCD_SCK   2
#define PIN_LCD_MOSI  1
#define PIN_LCD_BL    6

// ---- RGB Sync ----
#define PIN_DE     40
#define PIN_VSYNC  39
#define PIN_HSYNC  38
#define PIN_PCLK   41

// ---- RGB Daten (5R / 6G / 5B) ----
#define PIN_R0 46
#define PIN_R1 3
#define PIN_R2 8
#define PIN_R3 18
#define PIN_R4 17
#define PIN_G0 14
#define PIN_G1 13
#define PIN_G2 12
#define PIN_G3 11
#define PIN_G4 10
#define PIN_G5 9
#define PIN_B0 5
#define PIN_B1 45
#define PIN_B2 48
#define PIN_B3 47
#define PIN_B4 21

// -------- PCA9554 Helfer --------
static void pca9554Write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA9554_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Expander init: Pin0..2 Output, CS=LOW halten, Reset-Puls Display+Touch.
static void expanderInit() {
  pca9554Write(PCA9554_CONFIG, 0xF8);   // Pin0..2 = Output
  pca9554Write(PCA9554_OUTPUT, EXIO_LCD_RST | EXIO_TP_RST);  // CS=0, RST=1
  delay(20);
  pca9554Write(PCA9554_OUTPUT, 0x00);   // alles LOW -> Reset aktiv
  delay(20);
  pca9554Write(PCA9554_OUTPUT, EXIO_LCD_RST | EXIO_TP_RST);  // Reset aus, CS bleibt 0
  delay(120);
}

// -------- Arduino_GFX Aufbau --------
// SWSPI fuer ST7701-Init: DC=-1 (9-bit SPI), CS=-1 (extern), SCK, MOSI
Arduino_DataBus *bus = new Arduino_SWSPI(
    GFX_NOT_DEFINED /* DC */, GFX_NOT_DEFINED /* CS extern */,
    PIN_LCD_SCK, PIN_LCD_MOSI, GFX_NOT_DEFINED /* MISO */);

Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    PIN_DE, PIN_VSYNC, PIN_HSYNC, PIN_PCLK,
    PIN_R0, PIN_R1, PIN_R2, PIN_R3, PIN_R4,
    PIN_G0, PIN_G1, PIN_G2, PIN_G3, PIN_G4, PIN_G5,
    PIN_B0, PIN_B1, PIN_B2, PIN_B3, PIN_B4,
    1 /* hsync_polarity */, 10 /* hsync_front_porch */, 8 /* hsync_pulse_width */, 50 /* hsync_back_porch */,
    1 /* vsync_polarity */, 10 /* vsync_front_porch */, 8 /* vsync_pulse_width */, 20 /* vsync_back_porch */,
    1 /* pclk_active_neg */, 14000000 /* prefer_speed Hz */);

// ST7701 RGB-Display, 480x480, RST=-1 (extern via Expander), IPS=true.
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
    480, 480, rgbpanel, 0 /* rotation */, true /* auto_flush */,
    bus, GFX_NOT_DEFINED /* RST extern */,
    st7701_type1_init_operations, sizeof(st7701_type1_init_operations));

static void drawTestScreen() {
  gfx->fillScreen(RGB565_BLACK);
  gfx->fillCircle(240, 240, 235, RGB565_RED);
  gfx->fillCircle(240, 240, 180, RGB565_GREEN);
  gfx->fillCircle(240, 240, 120, RGB565_BLUE);
  gfx->fillCircle(240, 240, 60, RGB565_BLACK);

  gfx->setTextColor(RGB565_WHITE);
  gfx->setTextSize(3);
  gfx->setCursor(150, 222);
  gfx->println("VDO");
  gfx->setTextSize(2);
  gfx->setCursor(150, 258);
  gfx->println("Bring-up");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Waveshare 2.8C VDO Clock — Bring-up (Arduino_GFX) ===");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
  Serial.println("PCA9554: init + reset");
  expanderInit();

  Serial.println("Display: begin...");
  if (!gfx->begin()) {
    Serial.println("Display: begin() FAILED");
  } else {
    Serial.println("Display: begin() OK");
  }

  // Backlight an
  pinMode(PIN_LCD_BL, OUTPUT);
  digitalWrite(PIN_LCD_BL, HIGH);

  drawTestScreen();
  Serial.println("Test screen drawn.");
}

void loop() {
  delay(100);
}
