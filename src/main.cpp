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

// -------- ST7701 Init-Sequenz Waveshare 2.8C/2.8D (480x480) --------
// Uebersetzt aus dem offiziellen Waveshare ESP-BSP
// (waveshareteam/Waveshare-ESP32-components, esp32_s3_touch_lcd_2_8d.c)
// ins Arduino_GFX init_operations Format.
static const uint8_t waveshare_28c_init[] = {
    BEGIN_WRITE,
    WRITE_COMMAND_8, 0x11,  // Sleep out
    END_WRITE,
    DELAY, 120,
    BEGIN_WRITE,
    WRITE_COMMAND_8, 0xFF, WRITE_BYTES, 5, 0x77, 0x01, 0x00, 0x00, 0x13,
    WRITE_C8_D8, 0xFE, 0x08,
    WRITE_COMMAND_8, 0xFF, WRITE_BYTES, 5, 0x77, 0x01, 0x00, 0x00, 0x10,
    WRITE_C8_D16, 0xC0, 0x3B, 0x00,
    WRITE_C8_D16, 0xC1, 0x10, 0x0C,
    WRITE_C8_D16, 0xC2, 0x07, 0x0A,
    WRITE_C8_D8, 0xC7, 0x00,
    WRITE_C8_D8, 0xCC, 0x10,
    WRITE_C8_D8, 0xCD, 0x08,
    WRITE_COMMAND_8, 0xB0, WRITE_BYTES, 16,
      0x05, 0x12, 0x98, 0x0E, 0x0F, 0x07, 0x07, 0x09,
      0x09, 0x23, 0x05, 0x52, 0x0F, 0x67, 0x2C, 0x11,
    WRITE_COMMAND_8, 0xB1, WRITE_BYTES, 16,
      0x0B, 0x11, 0x97, 0x0C, 0x12, 0x06, 0x06, 0x08,
      0x08, 0x22, 0x03, 0x51, 0x11, 0x66, 0x2B, 0x0F,
    WRITE_COMMAND_8, 0xFF, WRITE_BYTES, 5, 0x77, 0x01, 0x00, 0x00, 0x11,
    WRITE_C8_D8, 0xB0, 0x5D,
    WRITE_C8_D8, 0xB1, 0x3E,
    WRITE_C8_D8, 0xB2, 0x81,
    WRITE_C8_D8, 0xB3, 0x80,
    WRITE_C8_D8, 0xB5, 0x4E,
    WRITE_C8_D8, 0xB7, 0x85,
    WRITE_C8_D8, 0xB8, 0x20,
    WRITE_C8_D8, 0xC1, 0x78,
    WRITE_C8_D8, 0xC2, 0x78,
    WRITE_C8_D8, 0xD0, 0x88,
    WRITE_COMMAND_8, 0xE0, WRITE_BYTES, 3, 0x00, 0x00, 0x02,
    WRITE_COMMAND_8, 0xE1, WRITE_BYTES, 11,
      0x06, 0x30, 0x08, 0x30, 0x05, 0x30, 0x07, 0x30, 0x00, 0x33, 0x33,
    WRITE_COMMAND_8, 0xE2, WRITE_BYTES, 12,
      0x11, 0x11, 0x33, 0x33, 0xF4, 0x00, 0x00, 0x00, 0xF4, 0x00, 0x00, 0x00,
    WRITE_COMMAND_8, 0xE3, WRITE_BYTES, 4, 0x00, 0x00, 0x11, 0x11,
    WRITE_C8_D16, 0xE4, 0x44, 0x44,
    WRITE_COMMAND_8, 0xE5, WRITE_BYTES, 16,
      0x0D, 0xF5, 0x30, 0xF0, 0x0F, 0xF7, 0x30, 0xF0,
      0x09, 0xF1, 0x30, 0xF0, 0x0B, 0xF3, 0x30, 0xF0,
    WRITE_COMMAND_8, 0xE6, WRITE_BYTES, 4, 0x00, 0x00, 0x11, 0x11,
    WRITE_C8_D16, 0xE7, 0x44, 0x44,
    WRITE_COMMAND_8, 0xE8, WRITE_BYTES, 16,
      0x0C, 0xF4, 0x30, 0xF0, 0x0E, 0xF6, 0x30, 0xF0,
      0x08, 0xF0, 0x30, 0xF0, 0x0A, 0xF2, 0x30, 0xF0,
    WRITE_C8_D16, 0xE9, 0x36, 0x01,
    WRITE_COMMAND_8, 0xEB, WRITE_BYTES, 7, 0x00, 0x01, 0xE4, 0xE4, 0x44, 0x88, 0x40,
    WRITE_COMMAND_8, 0xED, WRITE_BYTES, 16,
      0xFF, 0x10, 0xAF, 0x76, 0x54, 0x2B, 0xCF, 0xFF,
      0xFF, 0xFC, 0xB2, 0x45, 0x67, 0xFA, 0x01, 0xFF,
    WRITE_COMMAND_8, 0xFF, WRITE_BYTES, 5, 0x77, 0x01, 0x00, 0x00, 0x00,
    WRITE_COMMAND_8, 0x11,  // Sleep out (page 0)
    END_WRITE,
    DELAY, 120,
    BEGIN_WRITE,
    WRITE_C8_D8, 0x3A, 0x66,  // COLMOD 18-bit
    WRITE_C8_D8, 0x36, 0x00,
    WRITE_C8_D8, 0x35, 0x00,  // TE on
    WRITE_COMMAND_8, 0x29,    // Display ON
    END_WRITE,
};

// -------- Arduino_GFX Aufbau --------
// SWSPI fuer ST7701-Init: DC=-1 (9-bit SPI), CS=-1 (extern via Expander), SCK, MOSI
Arduino_DataBus *bus = new Arduino_SWSPI(
    GFX_NOT_DEFINED /* DC */, GFX_NOT_DEFINED /* CS extern */,
    PIN_LCD_SCK, PIN_LCD_MOSI, GFX_NOT_DEFINED /* MISO */);

// RGB-Timing aus Waveshare BSP (WAVESHARE_2_8D_480_480_PANEL_60HZ_RGB_TIMING):
// pclk=21MHz, hsync(pw=8,bp=50,fp=10), vsync(pw=2,bp=18,fp=8), pclk_active_neg=0
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    PIN_DE, PIN_VSYNC, PIN_HSYNC, PIN_PCLK,
    PIN_R0, PIN_R1, PIN_R2, PIN_R3, PIN_R4,
    PIN_G0, PIN_G1, PIN_G2, PIN_G3, PIN_G4, PIN_G5,
    PIN_B0, PIN_B1, PIN_B2, PIN_B3, PIN_B4,
    0 /* hsync_polarity */, 10 /* hsync_front_porch */, 8 /* hsync_pulse_width */, 50 /* hsync_back_porch */,
    0 /* vsync_polarity */, 8 /* vsync_front_porch */, 2 /* vsync_pulse_width */, 18 /* vsync_back_porch */,
    0 /* pclk_active_neg */, 21000000 /* prefer_speed Hz */);

// ST7701 RGB-Display, 480x480, RST=-1 (extern via Expander), IPS=true.
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
    480, 480, rgbpanel, 0 /* rotation */, true /* auto_flush */,
    bus, GFX_NOT_DEFINED /* RST extern */,
    waveshare_28c_init, sizeof(waveshare_28c_init));

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
