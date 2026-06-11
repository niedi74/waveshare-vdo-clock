#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"

#define HAL_I2C_SDA 15
#define HAL_I2C_SCL 7
#define HAL_LCD_SCK 2
#define HAL_LCD_MOSI 1
#define HAL_LCD_BL 6

// Optional: Original-Tacho-Helligkeits-Drehregler per ADC (siehe FUTURE.md).
// GPIO4 = ADC1_CH3 — einziger freier ADC1-Pin bei aktivem WiFi; Touch/Display unberührt.
#ifndef FEATURE_TACHO_DIMMER
#define FEATURE_TACHO_DIMMER 0
#endif
#define HAL_TACHO_DIMMER_ADC 4

#define HAL_PCA9554_ADDR 0x20
#define HAL_PCA_OUTPUT 0x01
#define HAL_PCA_CONFIG 0x03
#define HAL_EXIO_LCD_RST (1 << 0)
#define HAL_EXIO_TP_RST  (1 << 1)
#define HAL_EXIO_LCD_CS  (1 << 2)
#define HAL_EXIO_BUZZER  (1 << 7)  // PCA9554 Pin 8 = Bit 7 (Buzzer)

#define HAL_PIN_DE     40
#define HAL_PIN_VSYNC  39
#define HAL_PIN_HSYNC  38
#define HAL_PIN_PCLK   41
#define HAL_PIN_R0 46
#define HAL_PIN_R1 3
#define HAL_PIN_R2 8
#define HAL_PIN_R3 18
#define HAL_PIN_R4 17
#define HAL_PIN_G0 14
#define HAL_PIN_G1 13
#define HAL_PIN_G2 12
#define HAL_PIN_G3 11
#define HAL_PIN_G4 10
#define HAL_PIN_G5 9
#define HAL_PIN_B0 5
#define HAL_PIN_B1 45
#define HAL_PIN_B2 48
#define HAL_PIN_B3 47
#define HAL_PIN_B4 21

static spi_device_handle_t hal_spi = nullptr;
static esp_lcd_panel_handle_t hal_panel = nullptr;
static uint16_t *hal_frame = nullptr;
static bool hal_frame_owned = false;
static bool hal_ready = false;

static bool hal_bind_framebuffer() {
  if (!hal_panel) return false;
  void *fb0 = nullptr;
  if (esp_lcd_rgb_panel_get_frame_buffer(hal_panel, 1, &fb0) != ESP_OK || !fb0) {
    return false;
  }
  if (hal_frame && hal_frame_owned) {
    heap_caps_free(hal_frame);
  }
  hal_frame = (uint16_t *)fb0;
  hal_frame_owned = false;
  Serial.printf("hal_fb: panel FB0 %p\n", hal_frame);
  return true;
}

static uint8_t hal_pca_write_once(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(HAL_PCA9554_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission();
}

static bool hal_pca_write(uint8_t reg, uint8_t val) {
  uint8_t err = 0xFF;
  for (uint8_t attempt = 0; attempt < 5; attempt++) {
    err = hal_pca_write_once(reg, val);
    if (err == 0) {
      Serial.printf("PCA9554 write reg 0x%02X = 0x%02X -> %s\n",
                    reg, val, attempt ? "OK retry" : "0");
      return true;
    }
    delay(15);
  }
  Serial.printf("PCA9554 write reg 0x%02X = 0x%02X -> FAIL (last err %u)\n",
                reg, val, err);
  return false;
}

static uint8_t hal_pca_read(uint8_t reg) {
  uint8_t val = 0;
  Wire.beginTransmission(HAL_PCA9554_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) == 0) {
    if (Wire.requestFrom((int)HAL_PCA9554_ADDR, 1) == 1) {
      val = Wire.read();
    }
  }
  return val;
}

static void hal_buzzer(bool on) {
  // KEIN Read-Modify-Write! Ein fehlschlagender PCA-Read wuerde 0 liefern
  // und CS/RST/TP_RST loeschen -> Touch (TP_RST) und Display tot. Stattdessen
  // immer den bekannten Betriebs-Zustand schreiben + Buzzer-Bit oben drauf.
  const uint8_t base = HAL_EXIO_LCD_RST | HAL_EXIO_TP_RST | HAL_EXIO_LCD_CS;
  hal_pca_write(HAL_PCA_OUTPUT, on ? (base | HAL_EXIO_BUZZER) : base);
}

// GT911: INT level waehrend TP_RST-Puls setzt I2C-Adresse (LOW=0x5D, HIGH=0x14).
void hal_touch_reset(bool intHigh, int touchIntPin) {
  pinMode(touchIntPin, OUTPUT);
  digitalWrite(touchIntPin, intHigh ? HIGH : LOW);
  delay(2);
  hal_pca_write(HAL_PCA_OUTPUT, HAL_EXIO_LCD_RST | HAL_EXIO_LCD_CS);
  delay(10);
  hal_pca_write(HAL_PCA_OUTPUT, HAL_EXIO_LCD_RST | HAL_EXIO_TP_RST | HAL_EXIO_LCD_CS);
  delay(50);
  pinMode(touchIntPin, INPUT);
  delay(10);
}

static void hal_expander_init() {
  // 0x78: Bit0-2 = Output (LCD RST/CS, TP RST), Bit7 = Output (Buzzer),
  // Bit3-6 = Input. Buzzer-Bit muss Output sein, sonst schaltet er nicht.
  hal_pca_write(HAL_PCA_CONFIG, 0x78);
  hal_pca_write(HAL_PCA_OUTPUT, HAL_EXIO_LCD_RST | HAL_EXIO_TP_RST | HAL_EXIO_LCD_CS);
  delay(80);
  hal_pca_write(HAL_PCA_OUTPUT, HAL_EXIO_LCD_CS);
  delay(120);
  hal_pca_write(HAL_PCA_OUTPUT, HAL_EXIO_LCD_RST | HAL_EXIO_TP_RST | HAL_EXIO_LCD_CS);
  delay(200);
}

static void hal_cmd(uint8_t cmd) {
  if (!hal_spi) return;
  spi_transaction_t t = {};
  t.cmd = 0;
  t.addr = cmd;
  spi_device_transmit(hal_spi, &t);
}

static void hal_data(uint8_t data) {
  if (!hal_spi) return;
  spi_transaction_t t = {};
  t.cmd = 1;
  t.addr = data;
  spi_device_transmit(hal_spi, &t);
}

static void hal_st7701_init() {
  if (hal_spi) {
    spi_bus_remove_device(hal_spi);
    hal_spi = nullptr;
  }
  spi_bus_free(SPI2_HOST);

  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = HAL_LCD_MOSI;
  buscfg.miso_io_num = -1;
  buscfg.sclk_io_num = HAL_LCD_SCK;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = 64;
  esp_err_t err = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
  Serial.printf("SPI bus init: %s\n", esp_err_to_name(err));

  spi_device_interface_config_t devcfg = {};
  devcfg.command_bits = 1;
  devcfg.address_bits = 8;
  devcfg.mode = SPI_MODE0;
  devcfg.clock_speed_hz = 40000000;
  devcfg.spics_io_num = -1;
  devcfg.queue_size = 1;
  err = spi_bus_add_device(SPI2_HOST, &devcfg, &hal_spi);
  Serial.printf("SPI device add: %s\n", esp_err_to_name(err));

  hal_pca_write(HAL_PCA_OUTPUT, HAL_EXIO_LCD_RST | HAL_EXIO_TP_RST);
  delay(10);

  hal_cmd(0xFF); hal_data(0x77); hal_data(0x01); hal_data(0x00); hal_data(0x00); hal_data(0x13);
  hal_cmd(0xEF); hal_data(0x08);
  hal_cmd(0xFF); hal_data(0x77); hal_data(0x01); hal_data(0x00); hal_data(0x00); hal_data(0x10);
  hal_cmd(0xC0); hal_data(0x3B); hal_data(0x00);
  hal_cmd(0xC1); hal_data(0x10); hal_data(0x0C);
  hal_cmd(0xC2); hal_data(0x07); hal_data(0x0A);
  hal_cmd(0xC7); hal_data(0x00);
  hal_cmd(0xCC); hal_data(0x10);
  hal_cmd(0xCD); hal_data(0x08);
  hal_cmd(0xB0);
  for (uint8_t v : {0x05,0x12,0x98,0x0E,0x0F,0x07,0x07,0x09,0x09,0x23,0x05,0x52,0x0F,0x67,0x2C,0x11}) hal_data(v);
  hal_cmd(0xB1);
  for (uint8_t v : {0x0B,0x11,0x97,0x0C,0x12,0x06,0x06,0x08,0x08,0x22,0x03,0x51,0x11,0x66,0x2B,0x0F}) hal_data(v);
  hal_cmd(0xFF); hal_data(0x77); hal_data(0x01); hal_data(0x00); hal_data(0x00); hal_data(0x11);
  hal_cmd(0xB0); hal_data(0x5D);
  hal_cmd(0xB1); hal_data(0x3E);
  hal_cmd(0xB2); hal_data(0x81);
  hal_cmd(0xB3); hal_data(0x80);
  hal_cmd(0xB5); hal_data(0x4E);
  hal_cmd(0xB7); hal_data(0x85);
  hal_cmd(0xB8); hal_data(0x20);
  hal_cmd(0xC1); hal_data(0x78);
  hal_cmd(0xC2); hal_data(0x78);
  hal_cmd(0xD0); hal_data(0x88);
  hal_cmd(0xE0); hal_data(0x00); hal_data(0x00); hal_data(0x02);
  hal_cmd(0xE1);
  for (uint8_t v : {0x06,0x30,0x08,0x30,0x05,0x30,0x07,0x30,0x00,0x33,0x33}) hal_data(v);
  hal_cmd(0xE2);
  for (uint8_t v : {0x11,0x11,0x33,0x33,0xF4,0x00,0x00,0x00,0xF4,0x00,0x00,0x00}) hal_data(v);
  hal_cmd(0xE3); hal_data(0x00); hal_data(0x00); hal_data(0x11); hal_data(0x11);
  hal_cmd(0xE4); hal_data(0x44); hal_data(0x44);
  hal_cmd(0xE5);
  for (uint8_t v : {0x0D,0xF5,0x30,0xF0,0x0F,0xF7,0x30,0xF0,0x09,0xF1,0x30,0xF0,0x0B,0xF3,0x30,0xF0}) hal_data(v);
  hal_cmd(0xE6); hal_data(0x00); hal_data(0x00); hal_data(0x11); hal_data(0x11);
  hal_cmd(0xE7); hal_data(0x44); hal_data(0x44);
  hal_cmd(0xE8);
  for (uint8_t v : {0x0C,0xF4,0x30,0xF0,0x0E,0xF6,0x30,0xF0,0x08,0xF0,0x30,0xF0,0x0A,0xF2,0x30,0xF0}) hal_data(v);
  hal_cmd(0xE9); hal_data(0x36); hal_data(0x01);
  hal_cmd(0xEB); hal_data(0x00); hal_data(0x01); hal_data(0xE4); hal_data(0xE4); hal_data(0x44); hal_data(0x88); hal_data(0x40);
  hal_cmd(0xED);
  for (uint8_t v : {0xFF,0x10,0xAF,0x76,0x54,0x2B,0xCF,0xFF,0xFF,0xFC,0xB2,0x45,0x67,0xFA,0x01,0xFF}) hal_data(v);
  hal_cmd(0xEF); hal_data(0x08); hal_data(0x08); hal_data(0x08); hal_data(0x45); hal_data(0x3F); hal_data(0x54);
  hal_cmd(0xFF); hal_data(0x77); hal_data(0x01); hal_data(0x00); hal_data(0x00); hal_data(0x00);
  hal_cmd(0x11);
  delay(120);
  hal_cmd(0x3A); hal_data(0x66);
  hal_cmd(0x36); hal_data(0x00);
  hal_cmd(0x35); hal_data(0x00);
  hal_cmd(0x29);
  hal_pca_write(HAL_PCA_OUTPUT, HAL_EXIO_LCD_RST | HAL_EXIO_TP_RST | HAL_EXIO_LCD_CS);
  delay(10);
}

static bool hal_panel_init() {
  hal_ready = false;
  if (hal_panel) {
    esp_lcd_panel_del(hal_panel);
    hal_panel = nullptr;
  }
  if (hal_frame && hal_frame_owned) {
    heap_caps_free(hal_frame);
  }
  hal_frame = nullptr;
  hal_frame_owned = false;

  esp_lcd_rgb_panel_config_t cfg = {};
  cfg.clk_src = LCD_CLK_SRC_PLL160M;
  cfg.timings.pclk_hz = 8000000;
  cfg.timings.h_res = 480;
  cfg.timings.v_res = 480;
  cfg.timings.hsync_pulse_width = 8;
  cfg.timings.hsync_back_porch = 10;
  cfg.timings.hsync_front_porch = 50;
  cfg.timings.vsync_pulse_width = 2;
  cfg.timings.vsync_back_porch = 18;
  cfg.timings.vsync_front_porch = 8;
  cfg.timings.flags.pclk_active_neg = 0;
  cfg.data_width = 16;
  cfg.bits_per_pixel = 16;
  cfg.num_fbs = 2;
  cfg.bounce_buffer_size_px = 0;
  cfg.psram_trans_align = 64;
  cfg.hsync_gpio_num = HAL_PIN_HSYNC;
  cfg.vsync_gpio_num = HAL_PIN_VSYNC;
  cfg.de_gpio_num = HAL_PIN_DE;
  cfg.pclk_gpio_num = HAL_PIN_PCLK;
  cfg.disp_gpio_num = GPIO_NUM_NC;
  cfg.data_gpio_nums[0] = HAL_PIN_B0;
  cfg.data_gpio_nums[1] = HAL_PIN_B1;
  cfg.data_gpio_nums[2] = HAL_PIN_B2;
  cfg.data_gpio_nums[3] = HAL_PIN_B3;
  cfg.data_gpio_nums[4] = HAL_PIN_B4;
  cfg.data_gpio_nums[5] = HAL_PIN_G0;
  cfg.data_gpio_nums[6] = HAL_PIN_G1;
  cfg.data_gpio_nums[7] = HAL_PIN_G2;
  cfg.data_gpio_nums[8] = HAL_PIN_G3;
  cfg.data_gpio_nums[9] = HAL_PIN_G4;
  cfg.data_gpio_nums[10] = HAL_PIN_G5;
  cfg.data_gpio_nums[11] = HAL_PIN_R0;
  cfg.data_gpio_nums[12] = HAL_PIN_R1;
  cfg.data_gpio_nums[13] = HAL_PIN_R2;
  cfg.data_gpio_nums[14] = HAL_PIN_R3;
  cfg.data_gpio_nums[15] = HAL_PIN_R4;
  cfg.flags.fb_in_psram = true;

  esp_err_t err = esp_lcd_new_rgb_panel(&cfg, &hal_panel);
  Serial.printf("RGB panel create: %s\n", esp_err_to_name(err));
  if (err != ESP_OK) return false;
  err = esp_lcd_panel_reset(hal_panel);
  Serial.printf("RGB panel reset: %s\n", esp_err_to_name(err));
  err = esp_lcd_panel_init(hal_panel);
  Serial.printf("RGB panel init: %s\n", esp_err_to_name(err));
  hal_ready = (err == ESP_OK);
  if (hal_ready) hal_bind_framebuffer();
  return hal_ready;
}

void hal_backlight(bool on) {
  pinMode(HAL_LCD_BL, OUTPUT);
  digitalWrite(HAL_LCD_BL, on ? HIGH : LOW);
}

#if FEATURE_TACHO_DIMMER
static bool hal_dimmerReady = false;

void hal_tacho_dimmer_init() {
  pinMode(HAL_TACHO_DIMMER_ADC, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(HAL_TACHO_DIMMER_ADC, ADC_11db);
  hal_dimmerReady = true;
  Serial.printf("Dimmer ADC: GPIO%d (ADC1_CH3) bereit\n", HAL_TACHO_DIMMER_ADC);
}

int hal_tacho_dimmer_read_raw() {
  if (!hal_dimmerReady) return -1;
  return analogRead(HAL_TACHO_DIMMER_ADC);
}

int hal_tacho_dimmer_read_pct() {
  int raw = hal_tacho_dimmer_read_raw();
  if (raw < 0) return -1;
  int pct = map(raw, 0, 4095, 5, 100);
  if (pct < 5) pct = 5;
  if (pct > 100) pct = 100;
  return pct;
}
#else
inline void hal_tacho_dimmer_init() {}
inline int hal_tacho_dimmer_read_raw() { return -1; }
inline int hal_tacho_dimmer_read_pct() { return -1; }
#endif

uint16_t *hal_fb() {
  if (!hal_frame) {
    if (!hal_bind_framebuffer()) {
      hal_frame = (uint16_t *)heap_caps_malloc(480 * 480 * sizeof(uint16_t),
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      hal_frame_owned = (hal_frame != nullptr);
      Serial.printf("hal_fb: malloc fallback %p\n", hal_frame);
    }
  }
  return hal_frame;
}

void hal_present() {
  if (!hal_frame || !hal_panel) return;
  esp_lcd_panel_draw_bitmap(hal_panel, 0, 0, 480, 480, hal_frame);
  if (!hal_frame_owned) {
    esp_cache_msync(hal_frame, 480 * 480 * sizeof(uint16_t), 0);
  }
}

// RGB-DMA bei Bedarf neu synchronisieren (recovert ein durch WiFi/BLE
// verschobenes/schwarzes Bild beim naechsten VSYNC, ohne Neu-Init).
void hal_restart() {
  if (hal_panel) esp_lcd_rgb_panel_restart(hal_panel);
}

// RGB VSYNC ISR laeuft aus Flash. OTA (Update.write) deaktiviert den Flash-Cache
// periodisch -> ohne Pause kann die TCP-Verbindung bei ~128 KB abbrechen.
static bool hal_otaPaused = false;

void hal_pause_for_ota(bool pause) {
  if (pause) {
    if (!hal_panel || hal_otaPaused) return;
    esp_lcd_panel_del(hal_panel);
    hal_panel = nullptr;
    hal_ready = false;
    hal_backlight(false);
    hal_otaPaused = true;
    Serial.println("HAL: RGB pausiert fuer OTA");
  } else if (hal_otaPaused) {
    hal_otaPaused = false;
    if (hal_panel_init()) {
      hal_backlight(true);
      Serial.println("HAL: RGB nach OTA wieder aktiv");
    } else {
      Serial.println("HAL: RGB resume FAIL");
    }
  }
}

void hal_fill(uint16_t color) {
  uint16_t *fb = hal_fb();
  if (!fb) return;
  const uint32_t pair = (uint32_t)color | ((uint32_t)color << 16);
  uint32_t *w = (uint32_t *)fb;
  const int n = (480 * 480) / 2;
  for (int i = 0; i < n; i++) w[i] = pair;
}

bool hal_ok() {
  return hal_ready && hal_panel != nullptr;
}

void hal_init() {
  hal_ready = false;
  Wire.begin(HAL_I2C_SDA, HAL_I2C_SCL, 100000);
  delay(20);
  hal_backlight(false);
  hal_expander_init();
  hal_st7701_init();
  if (hal_panel_init()) {
    Serial.println("HAL init: OK");
  } else {
    Serial.println("HAL init: FAIL");
  }
}
