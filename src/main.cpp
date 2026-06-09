// Waveshare ESP32-S3-Touch-LCD-2.8C - VDO Quartz-Zeit Clock
// Main app: clock, touch menu, WiFi/NTP, Spartan3-Hub BLE client, WebGUI.
// Display hardware ownership lives in hal_waveshare_28c.h.
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <NimBLEDevice.h>
#if __has_include("wifi_secret.h")
  #include "wifi_secret.h"
#else
  #define WIFI_SSID     ""
  #define WIFI_PASSWORD ""
#endif
#include "hal_waveshare_28c.h"
#include "vdo_dial_480_rgb565.h"
#include <sys/time.h>
#include <time.h>

#define FEATURE_TOUCH 1

// ---- Touch / I2C ----
#define PIN_TOUCH_INT      16
#define GT911_ADDR_PRIMARY 0x5D
#define GT911_ADDR_ALT     0x14
#define GT911_PRODUCT_ID   0x8140
#define GT911_READ_XY      0x814E

// ---- PCF85063 RTC ----
#define PCF85063_ADDR    0x51
#define PCF85063_CTRL1   0x00
#define PCF85063_SECONDS 0x04

#ifndef RGB565
#define RGB565(r, g, b) (uint16_t)((((uint16_t)(r) & 0xF8) << 8) | (((uint16_t)(g) & 0xFC) << 3) | ((uint16_t)(b) >> 3))
#endif
#ifndef RGB565_BLACK
#define RGB565_BLACK RGB565(0, 0, 0)
#endif

// -------- Spartan3-Hub BLE-Client --------
#define SPARTAN_MAC    "30:30:f9:1d:d0:fd"
#define SPARTAN_SVC    "7f510001-5a6b-4d2a-9f20-14a7f3e20000"
#define SPARTAN_STATUS "7f510002-5a6b-4d2a-9f20-14a7f3e20000"

static float g_lambda = 0, g_rpm = 0, g_adv = 0, g_map = 0;
static float g_battVolt = 0, g_speedKmh = 0;
static float g_g123Volt = 0, g_g123Temp = 0, g_g123Coil = 0;
static bool  g_lambdaValid = false, g_battValid = false;
static bool  g_speedValid = false, g_g123Valid = false;
static bool  g_bleConn = false;
static uint32_t g_bleLastRx = 0, g_bleRxCnt = 0;
static String g_bleHubName = "---";

static NimBLEClient*      bleClient    = nullptr;
static NimBLEAddress      bleTarget;
static volatile bool      bleDoConnect = false;
static uint32_t           bleNextScanAt = 0;

// ---- GT911 touch state ----
static uint8_t  gt911Addr = GT911_ADDR_PRIMARY;
static bool     gt911Found = false;
static uint16_t g_lastTouchX = 0, g_lastTouchY = 0;
static uint32_t g_lastTouchMs = 0;
static uint8_t  g_lastTouchStatus = 0;

// ---- App state ----
static uint8_t   currentPage    = 0;
static bool      touchSeen      = false;
static char      g_ipStr[20]    = "---";
static int       g_dialScalePct = 100;
static int       g_brightnessPct = 100;
static int       g_rotationDeg  = 0;
static float     g_rotSin       = 0.0f;
static float     g_rotCos       = 1.0f;
static bool      g_featureWifi  = true;
static bool      g_featureBle   = false;
static bool      g_webStarted   = false;
static bool      g_redrawPage   = false;
static uint8_t   g_wifiProfile  = 0;
static WebServer webServer(80);
static void startWebServer();   // forward declaration
static void reconnectWifiProfile();
static void cycleWifiProfile();
static void updateRotationCache();

struct WifiProfile {
  const char* ssid;
  const char* pass;
};

static const WifiProfile WIFI_PROFILES[] = {
  { WIFI_SSID, WIFI_PASSWORD },
#ifdef WIFI_SSID_2
  { WIFI_SSID_2, WIFI_PASSWORD_2 },
#endif
#ifdef WIFI_SSID_3
  { WIFI_SSID_3, WIFI_PASSWORD_3 },
#endif
};

static uint8_t wifiProfileCount() {
  return (uint8_t)(sizeof(WIFI_PROFILES) / sizeof(WIFI_PROFILES[0]));
}

static const char* currentWifiSsid() {
  uint8_t count = wifiProfileCount();
  if (count == 0) return "";
  if (g_wifiProfile >= count) g_wifiProfile = 0;
  return WIFI_PROFILES[g_wifiProfile].ssid;
}

static const char* currentWifiPassword() {
  uint8_t count = wifiProfileCount();
  if (count == 0) return "";
  if (g_wifiProfile >= count) g_wifiProfile = 0;
  return WIFI_PROFILES[g_wifiProfile].pass;
}

// Build-time fallbacks (inject_time.py provides these)
#ifndef RTC_BUILD_Y
#define RTC_BUILD_Y   2026
#define RTC_BUILD_MO  1
#define RTC_BUILD_D   1
#define RTC_BUILD_H   12
#define RTC_BUILD_MI  0
#define RTC_BUILD_S   0
#define RTC_BUILD_DOW 4
#define RTC_BUILD_ID  0
#endif

// ---- PCF85063 helpers ----
static uint8_t decToBcd(uint8_t v) { return (uint8_t)((v / 10 * 16) + (v % 10)); }
static uint8_t bcdToDec(uint8_t v) { return (uint8_t)((v / 16 * 10) + (v % 16)); }

static bool rtcRead(struct tm *now) {
  Wire.beginTransmission(PCF85063_ADDR);
  Wire.write(PCF85063_SECONDS);
  if (Wire.endTransmission(true) != 0) return false;
  if (Wire.requestFrom((int)PCF85063_ADDR, 7) != 7) return false;
  uint8_t b[7];
  for (int i = 0; i < 7; i++) b[i] = Wire.read();
  now->tm_sec  = bcdToDec(b[0] & 0x7F);
  now->tm_min  = bcdToDec(b[1] & 0x7F);
  now->tm_hour = bcdToDec(b[2] & 0x3F);
  now->tm_mday = bcdToDec(b[3] & 0x3F);
  now->tm_wday = bcdToDec(b[4] & 0x07);
  now->tm_mon  = bcdToDec(b[5] & 0x1F) - 1;
  now->tm_year = bcdToDec(b[6]) + 100;
  return true;
}

static void rtcWrite(const struct tm *t) {
  Wire.beginTransmission(PCF85063_ADDR);
  Wire.write(PCF85063_CTRL1);
  Wire.write(0x01);
  Wire.endTransmission();
  Wire.beginTransmission(PCF85063_ADDR);
  Wire.write(PCF85063_SECONDS);
  Wire.write(decToBcd(t->tm_sec));
  Wire.write(decToBcd(t->tm_min));
  Wire.write(decToBcd(t->tm_hour));
  Wire.write(decToBcd(t->tm_mday));
  Wire.write(decToBcd(t->tm_wday));
  Wire.write(decToBcd(t->tm_mon + 1));
  Wire.write(decToBcd((t->tm_year + 1900) - 2000));
  Wire.endTransmission();
}

static bool readClockTime(struct tm *now) {
  if (rtcRead(now)) return true;
  time_t t = time(nullptr);
  return localtime_r(&t, now) != nullptr;
}

static void initTimeSource() {
  struct tm rtcNow = {};
  bool haveRtc  = rtcRead(&rtcNow);
  bool rtcValid = haveRtc && (rtcNow.tm_year + 1900) >= 2024;
  Preferences prefs;
  prefs.begin("clock", false);
  uint32_t savedId = prefs.getUInt("buildid", 0);
  bool newFlash = (savedId != (uint32_t)RTC_BUILD_ID);
  if (!rtcValid || newFlash) {
    struct tm bt = {};
    bt.tm_year = RTC_BUILD_Y - 1900;
    bt.tm_mon  = RTC_BUILD_MO - 1;
    bt.tm_mday = RTC_BUILD_D;
    bt.tm_hour = RTC_BUILD_H;
    bt.tm_min  = RTC_BUILD_MI;
    bt.tm_sec  = RTC_BUILD_S;
    bt.tm_wday = RTC_BUILD_DOW;
    rtcWrite(&bt);
    prefs.putUInt("buildid", (uint32_t)RTC_BUILD_ID);
    Serial.printf("RTC set from build time: %04d-%02d-%02d %02d:%02d:%02d (reason: %s)\n",
                  RTC_BUILD_Y, RTC_BUILD_MO, RTC_BUILD_D, RTC_BUILD_H, RTC_BUILD_MI, RTC_BUILD_S,
                  !rtcValid ? "RTC invalid" : "new flash");
  } else {
    Serial.printf("RTC running: %04d-%02d-%02d %02d:%02d:%02d\n",
                  rtcNow.tm_year + 1900, rtcNow.tm_mon + 1, rtcNow.tm_mday,
                  rtcNow.tm_hour, rtcNow.tm_min, rtcNow.tm_sec);
  }
  prefs.end();
}

// Non-blocking WiFi/NTP handler. Returns true on fresh NTP sync.
static bool wifiNtpTick() {
  static bool     sntpStarted = false;
  static bool     ntpSynced   = false;
  static uint32_t lastTry     = 0;

  if (!g_featureWifi || strlen(currentWifiSsid()) == 0) return false;

  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastTry > 30000) {
      lastTry = millis();
      WiFi.begin(currentWifiSsid(), currentWifiPassword());  // kein disconnect() davor
      Serial.printf("WiFi: Reconnect-Versuch zu '%s'\n", currentWifiSsid());
    }
    if (g_ipStr[0] == '-') strcpy(g_ipStr, "...");
    return false;
  }

  static char lastIp[20] = "";
  snprintf(g_ipStr, sizeof(g_ipStr), "%s", WiFi.localIP().toString().c_str());
  if (strcmp(lastIp, g_ipStr) != 0) {
    strcpy(lastIp, g_ipStr);
    Serial.printf("WiFi verbunden, IP: %s\n", g_ipStr);
  }
  if (!g_webStarted) {
    startWebServer();
    g_webStarted = true;
  }
  if (!sntpStarted) {
    configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org", "time.nist.gov", "de.pool.ntp.org");
    sntpStarted = true;
    Serial.println("NTP: SNTP gestartet");
  }
  if (!ntpSynced) {
    time_t t = time(nullptr);
    if (t > 1700000000) {
      struct tm now;
      localtime_r(&t, &now);
      rtcWrite(&now);
      ntpSynced = true;
      Serial.printf("NTP: synchronisiert -> RTC gestellt: %04d-%02d-%02d %02d:%02d:%02d\n",
                    now.tm_year + 1900, now.tm_mon + 1, now.tm_mday,
                    now.tm_hour, now.tm_min, now.tm_sec);
      return true;
    }
  }
  return false;
}

// -------- BLE Spartan3-Hub client --------
static void parseSpartanPayload(const String& p) {
  if (!(p.startsWith("L") && p.indexOf('R') > 1)) return;
  int posR = p.indexOf('R');
  int posA = p.indexOf('A', posR + 1);
  int posM = p.indexOf('M', posA + 1);
  if (!(posR > 1 && posA > posR && posM > posA)) return;

  g_lambda = p.substring(1, posR).toFloat();  g_lambdaValid = true;
  g_rpm    = p.substring(posR + 1, posA).toFloat();
  g_adv    = p.substring(posA + 1, posM).toFloat();

  int posV   = p.indexOf('V', posM + 1);
  int posS   = p.indexOf('S', posM + 1);
  int posI   = p.indexOf('I', posM + 1);
  int mapEnd = posV > posM ? posV : (posS > posM ? posS : (posI > posM ? posI : p.length()));
  g_map = p.substring(posM + 1, mapEnd).toFloat();

  if (posV > posM) {
    int vEnd = posS > posV ? posS : (posI > posV ? posI : p.length());
    float v = p.substring(posV + 1, vEnd).toFloat();
    if (v > 0.5f) { g_battVolt = v; g_battValid = true; }
  }
  if (posS > posM) {
    int sEnd = posI > posS ? posI : p.length();
    g_speedKmh = p.substring(posS + 1, sEnd).toFloat();
    g_speedValid = true;
  }
  if (posI > posM) {
    int posT = p.indexOf('T', posI + 1);
    int posC = p.indexOf('C', posT > 0 ? posT + 1 : posI + 1);
    if (posT > posI && posC > posT) {
      g_g123Volt = p.substring(posI + 1, posT).toFloat();
      g_g123Temp = p.substring(posT + 1, posC).toFloat();
      g_g123Coil = p.substring(posC + 1).toFloat();
      g_g123Valid = true;
    }
  }
  g_bleRxCnt++;
  g_bleLastRx = millis();
}

static void bleNotifyCB(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
  String s;
  s.reserve(len + 1);
  for (size_t i = 0; i < len; i++) s += (char)data[i];
  parseSpartanPayload(s);
}

class SpartanClientCB : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient*) override {
    g_bleConn = true;
    Serial.println("BLE: mit Spartan-Hub verbunden");
  }
  void onDisconnect(NimBLEClient*, int reason) override {
    g_bleConn = false;
    g_bleHubName = "---";
    Serial.printf("BLE: getrennt (reason=%d), neuer Scan\n", reason);
    bleNextScanAt = millis() + 15000;
  }
  bool onConnParamsUpdateRequest(NimBLEClient*, const ble_gap_upd_params*) override {
    return true;
  }
};

class SpartanScanCB : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    String addr = dev->getAddress().toString().c_str();
    addr.toLowerCase();
    String name = dev->getName().c_str();
    g_bleHubName = name.length() > 0 ? name : "---";
    if (addr == SPARTAN_MAC ||
        dev->isAdvertisingService(NimBLEUUID(SPARTAN_SVC))) {
      bleTarget    = dev->getAddress();
      bleDoConnect = true;
      NimBLEDevice::getScan()->stop();
      Serial.printf("BLE: Spartan-Hub gefunden (%s / %s)\n", addr.c_str(), name.c_str());
    }
  }
  void onScanEnd(const NimBLEScanResults&, int) override {
    if (!g_bleConn && !bleDoConnect) bleNextScanAt = millis() + 15000;
  }
};

static SpartanClientCB spartanClientCB;
static SpartanScanCB   spartanScanCB;

static void bleStartScan() {
  if (g_bleConn || bleDoConnect) return;
  auto* s = NimBLEDevice::getScan();
  s->setScanCallbacks(&spartanScanCB);
  s->setActiveScan(false);
  s->setInterval(160);
  s->setWindow(30);
  s->start(3000, false);
  Serial.println("BLE: Scan nach Spartan-Hub...");
}

static void bleConnect() {
  bleDoConnect = false;
  if (!bleClient) {
    bleClient = NimBLEDevice::createClient();
    bleClient->setClientCallbacks(&spartanClientCB, false);
  }
  if (!bleClient->connect(bleTarget, true, false, false)) {
    Serial.println("BLE: Connect fehlgeschlagen");
    bleNextScanAt = millis() + 15000;
    return;
  }
  auto* svc = bleClient->getService(SPARTAN_SVC);
  if (!svc) { Serial.println("BLE: kein Service"); bleNextScanAt = millis() + 15000; return; }
  auto* status = svc->getCharacteristic(SPARTAN_STATUS);
  if (!status) { Serial.println("BLE: kein Status-Char"); bleNextScanAt = millis() + 15000; return; }
  bool ok = status->subscribe(true, bleNotifyCB, true);
  Serial.printf("BLE: Subscribe %s\n", ok ? "OK" : "FAIL");
}

static void bleTick() {
  if (bleDoConnect) { bleConnect(); return; }
  if (!g_bleConn && bleNextScanAt != 0 && millis() >= bleNextScanAt) {
    bleNextScanAt = 0;
    bleStartScan();
  }
}

// ---- I2C helpers (GT911) ----
static bool i2cRegRead16(uint8_t addr, uint16_t reg, uint8_t *data, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg >> 8);
  Wire.write(reg & 0xFF);
  if (Wire.endTransmission(true) != 0) return false;
  uint8_t got = Wire.requestFrom((int)addr, (int)len);
  if (got != len) { while (Wire.available()) Wire.read(); return false; }
  for (uint8_t i = 0; i < len; i++) data[i] = Wire.read();
  return true;
}

static bool i2cRegWrite16(uint8_t addr, uint16_t reg, const uint8_t *data, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg >> 8);
  Wire.write(reg & 0xFF);
  for (uint8_t i = 0; i < len; i++) Wire.write(data[i]);
  return Wire.endTransmission() == 0;
}

static void gt911ResetAddressMode(bool intHigh) {
  pinMode(PIN_TOUCH_INT, OUTPUT);
  digitalWrite(PIN_TOUCH_INT, intHigh ? HIGH : LOW);
  delay(20);
  digitalWrite(PIN_TOUCH_INT, intHigh ? HIGH : LOW);
  delay(5);
  pinMode(PIN_TOUCH_INT, INPUT);
  delay(80);
}

static bool gt911Probe() {
  uint8_t id[4] = {0};
  if (i2cRegRead16(GT911_ADDR_PRIMARY, GT911_PRODUCT_ID, id, sizeof(id))) {
    gt911Addr = GT911_ADDR_PRIMARY; gt911Found = true;
  } else if (i2cRegRead16(GT911_ADDR_ALT, GT911_PRODUCT_ID, id, sizeof(id))) {
    gt911Addr = GT911_ADDR_ALT; gt911Found = true;
  } else {
    gt911Found = false; return false;
  }
  Serial.printf("GT911: addr 0x%02X id %c%c%c%c\n", gt911Addr, id[0], id[1], id[2], id[3]);
  uint8_t clear = 0;
  i2cRegWrite16(gt911Addr, GT911_READ_XY, &clear, 1);
  return true;
}

static void gt911Init() {
  Serial.println("GT911: reset/probe INT low");
  gt911ResetAddressMode(false);
  if (gt911Probe()) return;
  Serial.println("GT911: reset/probe INT high");
  gt911ResetAddressMode(true);
  if (gt911Probe()) return;
  Serial.println("GT911: not found on 0x5D/0x14");
}

static bool readTouch(uint16_t *x, uint16_t *y) {
  uint8_t status = 0;
  if (!i2cRegRead16(gt911Addr, GT911_READ_XY, &status, 1)) {
    uint8_t other = (gt911Addr == GT911_ADDR_PRIMARY) ? GT911_ADDR_ALT : GT911_ADDR_PRIMARY;
    if (!i2cRegRead16(other, GT911_READ_XY, &status, 1)) return false;
    gt911Addr  = other;
    gt911Found = true;
  }
  g_lastTouchStatus = status;
  if ((status & 0x80) == 0) {
    uint8_t clear = 0;
    i2cRegWrite16(gt911Addr, GT911_READ_XY, &clear, 1);
    return false;
  }
  uint8_t clear  = 0;
  uint8_t points = status & 0x0F;
  if (points == 0 || points > 5) {
    i2cRegWrite16(gt911Addr, GT911_READ_XY, &clear, 1);
    return false;
  }
  uint8_t point[8] = {0};
  bool ok = i2cRegRead16(gt911Addr, GT911_READ_XY + 1, point, sizeof(point));
  i2cRegWrite16(gt911Addr, GT911_READ_XY, &clear, 1);
  if (!ok) return false;
  // GT911 ab 0x814F: [TrackID, X-low, X-high, Y-low, Y-high, Size-low, ...]
  // -> X = point[1]|point[2]<<8, Y = point[3]|point[4]<<8 (war um 1 Byte verschoben)
  uint16_t rawX = (uint16_t)point[1] | ((uint16_t)point[2] << 8);
  uint16_t rawY = (uint16_t)point[3] | ((uint16_t)point[4] << 8);
  if (g_rotationDeg == 0) {
    *x = rawX;
    *y = rawY;
  } else {
    float dx = (float)rawX - 239.5f;
    float dy = (float)rawY - 239.5f;
    int lx = (int)lroundf(dx * g_rotCos + dy * g_rotSin + 239.5f);
    int ly = (int)lroundf(-dx * g_rotSin + dy * g_rotCos + 239.5f);
    if (lx < 0) lx = 0; else if (lx > 479) lx = 479;
    if (ly < 0) ly = 0; else if (ly > 479) ly = 479;
    *x = (uint16_t)lx;
    *y = (uint16_t)ly;
  }
  g_lastTouchX  = *x;
  g_lastTouchY  = *y;
  g_lastTouchMs = millis();
  return true;
}

// ---- Display helpers (all writes go through HAL) ----
static bool ensureFrame()         { return hal_fb() != nullptr; }
static void presentFrame()        { hal_present(); }
static void fillFrame(uint16_t c) { hal_fill(c); }

static void setPixel(int x, int y, uint16_t color) {
  uint16_t *fb = hal_fb();
  if (!fb || (unsigned)x >= 480 || (unsigned)y >= 480) return;
  if (g_rotationDeg != 0) {
    float dx = (float)x - 239.5f;
    float dy = (float)y - 239.5f;
    x = (int)lroundf(dx * g_rotCos - dy * g_rotSin + 239.5f);
    y = (int)lroundf(dx * g_rotSin + dy * g_rotCos + 239.5f);
    if ((unsigned)x >= 480 || (unsigned)y >= 480) return;
  }
  fb[y * 480 + x] = color;
}

static void fillRectFast(int x, int y, int w, int h, uint16_t color) {
  for (int yy = y; yy < y + h; yy++)
    for (int xx = x; xx < x + w; xx++)
      setPixel(xx, yy, color);
}

static void drawCircleLine(int cx, int cy, int radius, int thickness, uint16_t color) {
  int outer = radius * radius, innerRadius = radius - thickness, inner = innerRadius * innerRadius;
  for (int y = cy - radius; y <= cy + radius; y++)
    for (int x = cx - radius; x <= cx + radius; x++) {
      int dx = x - cx, dy = y - cy, d = dx*dx + dy*dy;
      if (d <= outer && d >= inner) setPixel(x, y, color);
    }
}

static void fillCircleFast(int cx, int cy, int radius, uint16_t color) {
  int r2 = radius * radius;
  for (int y = cy - radius; y <= cy + radius; y++)
    for (int x = cx - radius; x <= cx + radius; x++) {
      int dx = x - cx, dy = y - cy;
      if (dx*dx + dy*dy <= r2) setPixel(x, y, color);
    }
}

static void drawLineFast(int x0, int y0, int x1, int y1, uint16_t color, int thickness) {
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy, radius = thickness / 2;
  while (true) {
    fillCircleFast(x0, y0, radius, color);
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

static void drawHand(float value, float maxValue, int length, int thickness, uint16_t color) {
  float angle = (value / maxValue) * 2.0f * PI - PI / 2.0f;
  int x = 240 + (int)(cosf(angle) * length);
  int y = 240 + (int)(sinf(angle) * length);
  drawLineFast(240, 240, x, y, color, thickness);
}

static uint8_t glyphColumn(char c, uint8_t col) {
  static const uint8_t blank[5] = {0,0,0,0,0};
  const uint8_t *g = blank;
  switch (c) {
    case 'A': { static const uint8_t v[5]={0x7E,0x11,0x11,0x11,0x7E}; g=v; break; }
    case 'B': { static const uint8_t v[5]={0x7F,0x49,0x49,0x49,0x36}; g=v; break; }
    case 'D': { static const uint8_t v[5]={0x7F,0x41,0x41,0x22,0x1C}; g=v; break; }
    case 'E': { static const uint8_t v[5]={0x7F,0x49,0x49,0x49,0x41}; g=v; break; }
    case 'H': { static const uint8_t v[5]={0x7F,0x08,0x08,0x08,0x7F}; g=v; break; }
    case 'I': { static const uint8_t v[5]={0x00,0x41,0x7F,0x41,0x00}; g=v; break; }
    case 'K': { static const uint8_t v[5]={0x7F,0x08,0x14,0x22,0x41}; g=v; break; }
    case 'L': { static const uint8_t v[5]={0x7F,0x40,0x40,0x40,0x40}; g=v; break; }
    case 'M': { static const uint8_t v[5]={0x7F,0x02,0x0C,0x02,0x7F}; g=v; break; }
    case 'N': { static const uint8_t v[5]={0x7F,0x04,0x08,0x10,0x7F}; g=v; break; }
    case 'O': { static const uint8_t v[5]={0x3E,0x41,0x41,0x41,0x3E}; g=v; break; }
    case 'P': { static const uint8_t v[5]={0x7F,0x09,0x09,0x09,0x06}; g=v; break; }
    case 'Q': { static const uint8_t v[5]={0x3E,0x41,0x51,0x21,0x5E}; g=v; break; }
    case 'R': { static const uint8_t v[5]={0x7F,0x09,0x19,0x29,0x46}; g=v; break; }
    case 'S': { static const uint8_t v[5]={0x46,0x49,0x49,0x49,0x31}; g=v; break; }
    case 'T': { static const uint8_t v[5]={0x01,0x01,0x7F,0x01,0x01}; g=v; break; }
    case 'U': { static const uint8_t v[5]={0x3F,0x40,0x40,0x40,0x3F}; g=v; break; }
    case 'V': { static const uint8_t v[5]={0x1F,0x20,0x40,0x20,0x1F}; g=v; break; }
    case 'W': { static const uint8_t v[5]={0x7F,0x20,0x18,0x20,0x7F}; g=v; break; }
    case 'X': { static const uint8_t v[5]={0x63,0x14,0x08,0x14,0x63}; g=v; break; }
    case 'Y': { static const uint8_t v[5]={0x07,0x08,0x70,0x08,0x07}; g=v; break; }
    case 'Z': { static const uint8_t v[5]={0x61,0x51,0x49,0x45,0x43}; g=v; break; }
    case '/': { static const uint8_t v[5]={0x20,0x10,0x08,0x04,0x02}; g=v; break; }
    case '-': { static const uint8_t v[5]={0x08,0x08,0x08,0x08,0x08}; g=v; break; }
    case '.': { static const uint8_t v[5]={0x00,0x00,0x40,0x00,0x00}; g=v; break; }
    case ':': { static const uint8_t v[5]={0x00,0x00,0x24,0x00,0x00}; g=v; break; }
    case '%': { static const uint8_t v[5]={0x62,0x64,0x08,0x13,0x23}; g=v; break; }
    case '0': { static const uint8_t v[5]={0x3E,0x51,0x49,0x45,0x3E}; g=v; break; }
    case '1': { static const uint8_t v[5]={0x00,0x42,0x7F,0x40,0x00}; g=v; break; }
    case '2': { static const uint8_t v[5]={0x62,0x51,0x49,0x49,0x46}; g=v; break; }
    case '3': { static const uint8_t v[5]={0x22,0x41,0x49,0x49,0x36}; g=v; break; }
    case '4': { static const uint8_t v[5]={0x18,0x14,0x12,0x7F,0x10}; g=v; break; }
    case '5': { static const uint8_t v[5]={0x27,0x45,0x45,0x45,0x39}; g=v; break; }
    case '6': { static const uint8_t v[5]={0x3C,0x4A,0x49,0x49,0x30}; g=v; break; }
    case '7': { static const uint8_t v[5]={0x01,0x71,0x09,0x05,0x03}; g=v; break; }
    case '8': { static const uint8_t v[5]={0x36,0x49,0x49,0x49,0x36}; g=v; break; }
    case '9': { static const uint8_t v[5]={0x06,0x49,0x49,0x29,0x1E}; g=v; break; }
    default: break;
  }
  return g[col];
}

static void drawTextSmall(int x, int y, const char *text, uint16_t color, int scale) {
  int cursor = x;
  while (*text) {
    char c = *text++;
    if (c == ' ') { cursor += 4 * scale; continue; }
    for (int col = 0; col < 5; col++) {
      uint8_t bits = glyphColumn(c, col);
      for (int row = 0; row < 7; row++)
        if (bits & (1 << row)) fillRectFast(cursor + col * scale, y + row * scale, scale, scale, color);
    }
    cursor += 6 * scale;
  }
}

static int textWidthSmall(const char *text, int scale) {
  int w = 0;
  while (*text) w += (*text++ == ' ') ? 4 * scale : 6 * scale;
  return w;
}

static void drawTextCentered(int cx, int y, const char *text, uint16_t color, int scale) {
  drawTextSmall(cx - textWidthSmall(text, scale) / 2, y, text, color, scale);
}

static void drawGlyphPixelRotated(int x, int y, int lx, int ly, int w, int h, int scale, int rot, uint16_t color) {
  int rx = lx, ry = ly;
  if (rot == 1)      { rx = h - 1 - ly; ry = lx; }
  else if (rot == 2) { rx = w - 1 - lx; ry = h - 1 - ly; }
  else if (rot == 3) { rx = ly;          ry = w - 1 - lx; }
  fillRectFast(x + rx * scale, y + ry * scale, scale, scale, color);
}

static void drawTextRotated(int x, int y, const char *text, uint16_t color, int scale, int rotation) {
  int w = textWidthSmall(text, 1), h = 7, cursor = 0;
  while (*text) {
    char c = *text++;
    if (c == ' ') { cursor += 4; continue; }
    for (int col = 0; col < 5; col++) {
      uint8_t bits = glyphColumn(c, col);
      for (int row = 0; row < 7; row++)
        if (bits & (1 << row)) drawGlyphPixelRotated(x, y, cursor + col, row, w, h, scale, rotation, color);
    }
    cursor += 6;
  }
}

static void drawTextCenteredRotated(int cx, int cy, const char *text, uint16_t color, int scale, int rotation) {
  int w = textWidthSmall(text, 1) * scale, h = 7 * scale;
  int rw = (rotation == 1 || rotation == 3) ? h : w;
  int rh = (rotation == 1 || rotation == 3) ? w : h;
  drawTextRotated(cx - rw / 2, cy - rh / 2, text, color, scale, rotation);
}

static void drawDialText(int cx, int cy, const char *text, uint16_t color, int scale, int rotation) {
  if (rotation == 0) drawTextCentered(cx, cy - (7 * scale) / 2, text, color, scale);
  else drawTextCenteredRotated(cx, cy, text, color, scale, rotation);
}

static void copyVdoDialToFrame() {
  uint16_t *fb = hal_fb();
  if (!fb) return;
  int pct = g_dialScalePct;
  if (pct < 30) pct = 30;
  if (pct > 100) pct = 100;
  if (pct == 100 && g_rotationDeg == 0) {
    for (int i = 0; i < 480 * 480; i++) fb[i] = pgm_read_word(&VDO_DIAL_480_RGB565[i]);
    return;
  }
  for (int i = 0; i < 480 * 480; i++) fb[i] = RGB565_BLACK;
  int outSize = (480 * pct) / 100;
  int offset  = (480 - outSize) / 2;
  if (g_rotationDeg == 0) {
    for (int oy = 0; oy < outSize; oy++) {
      int sy     = (oy * 480) / outSize;
      int dstRow = (offset + oy) * 480 + offset;
      int srcRow = sy * 480;
      for (int ox = 0; ox < outSize; ox++) {
        int sx = (ox * 480) / outSize;
        fb[dstRow + ox] = pgm_read_word(&VDO_DIAL_480_RGB565[srcRow + sx]);
      }
    }
    return;
  }
  for (int py = 0; py < 480; py++) {
    for (int px = 0; px < 480; px++) {
      float dx = (float)px - 239.5f;
      float dy = (float)py - 239.5f;
      int lx = (int)lroundf(dx * g_rotCos + dy * g_rotSin + 239.5f);
      int ly = (int)lroundf(-dx * g_rotSin + dy * g_rotCos + 239.5f);
      int ox = lx - offset;
      int oy = ly - offset;
      if ((unsigned)ox < (unsigned)outSize && (unsigned)oy < (unsigned)outSize) {
        int sx = (ox * 480) / outSize;
        int sy = (oy * 480) / outSize;
        fb[py * 480 + px] = pgm_read_word(&VDO_DIAL_480_RGB565[sy * 480 + sx]);
      }
    }
  }
}

static void drawVdoClock() {
  if (!ensureFrame()) return;
  copyVdoDialToFrame();
  struct tm now = {};
  readClockTime(&now);
  float seconds     = now.tm_sec;
  float minuteValue = now.tm_min + seconds / 60.0f;
  float hourValue   = (now.tm_hour % 12) + minuteValue / 60.0f;
  float s = g_dialScalePct / 100.0f;
  if (s < 0.30f) s = 0.30f;
  if (s > 1.0f)  s = 1.0f;
  #define SC(v) ((int)((v) * s + 0.5f))
  drawHand(hourValue,   12.0f, SC(118), SC(18), RGB565(24,  24,  22));
  drawHand(hourValue,   12.0f, SC(118), SC(13), RGB565(222, 222, 214));
  drawHand(minuteValue, 60.0f, SC(172), SC(15), RGB565(24,  24,  22));
  drawHand(minuteValue, 60.0f, SC(172), SC(10), RGB565(226, 226, 218));
  drawHand(seconds,     60.0f, SC(188), SC(4),  RGB565(235, 24,  20));
  fillCircleFast(240, 240, SC(26), RGB565(205, 205, 198));
  fillCircleFast(240, 240, SC(15), RGB565(166, 122, 42));
  fillCircleFast(240, 240, SC(9),  RGB565(38,  30,  18));
  fillCircleFast(240, 240, SC(5),  RGB565_BLACK);
  #undef SC
  presentFrame();
}

static void drawMenuTile(int x, int y, int w, int h, const char *label, uint16_t accent) {
  fillRectFast(x, y, w, h, RGB565(18, 18, 18));
  fillRectFast(x, y, 8, h, accent);
  drawLineFast(x, y,     x + w, y,     RGB565(70, 70, 70), 2);
  drawLineFast(x, y + h, x + w, y + h, RGB565(55, 55, 55), 2);
  drawTextSmall(x + 24, y + 22, label, RGB565(235, 235, 225), 4);
}

// Touch zones for menu: 5 equal 60px bands starting at y=108.
// Zone 0 (UHR)=108..168, Zone 1 (MOTOR)=168..228, Zone 2 (LAMBDA)=228..288,
// Zone 3 (HUB)=288..348, Zone 4 (SETUP)=348..408.
// Tiles are drawn 2px inset so the visual tile exactly fills its touch zone.
#define MENU_ZONE_Y0  108
#define MENU_ZONE_H    60

static void drawMenuOverview() {
  if (!ensureFrame()) return;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(80, 80, 75));
  drawTextCentered(240, 50, "MENU", RGB565(235, 235, 225), 7);
  // Tiles inset 2px from zone boundary so visual == touchable area
  drawMenuTile(88, MENU_ZONE_Y0 +   0 + 2, 304, MENU_ZONE_H - 4, "UHR",    RGB565(200, 40,  35));
  drawMenuTile(88, MENU_ZONE_Y0 +  60 + 2, 304, MENU_ZONE_H - 4, "MOTOR",  RGB565(40,  150, 210));
  drawMenuTile(88, MENU_ZONE_Y0 + 120 + 2, 304, MENU_ZONE_H - 4, "LAMBDA", RGB565(60,  185, 90));
  drawMenuTile(88, MENU_ZONE_Y0 + 180 + 2, 304, MENU_ZONE_H - 4, "HUB",    RGB565(190, 90,  210));
  drawMenuTile(88, MENU_ZONE_Y0 + 240 + 2, 304, MENU_ZONE_H - 4, "SETUP",  RGB565(210, 170, 45));
  char ipLine[32];
  snprintf(ipLine, sizeof(ipLine), "IP %s", g_ipStr);
  drawTextCentered(240, 420, ipLine, RGB565(150, 200, 150), 2);
  if (g_featureWifi && strlen(currentWifiSsid()) > 0) {
    drawTextCentered(240, 444, currentWifiSsid(), RGB565(120, 150, 150), 2);
  }
  presentFrame();
}

static bool bleFresh() {
  return g_bleConn && (millis() - g_bleLastRx < 3000);
}

static void drawDataRow(int y, const char* label, const char* value, uint16_t col) {
  drawTextSmall(92,  y, label, RGB565(160, 160, 160), 2);
  drawTextSmall(244, y, value, col, 2);
}

static void drawMotorPage() {
  if (!ensureFrame()) return;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(40, 110, 160));
  drawTextCentered(240, 52, "MOTOR", RGB565(60, 170, 230), 5);
  bool fresh = bleFresh();
  char buf[16];
  uint16_t cv = fresh ? RGB565(235, 235, 225) : RGB565(110, 60, 60);
  const char* na = "---";
  if (fresh) { snprintf(buf, sizeof(buf), "%d", (int)g_rpm); drawDataRow(112, "RPM", buf, cv); }
  else drawDataRow(112, "RPM", na, cv);
  if (fresh) { snprintf(buf, sizeof(buf), "%d", (int)g_adv); drawDataRow(152, "ADV", buf, cv); }
  else drawDataRow(152, "ADV", na, cv);
  if (fresh) { snprintf(buf, sizeof(buf), "%d", (int)g_map); drawDataRow(192, "MAP", buf, cv); }
  else drawDataRow(192, "MAP", na, cv);
  if (fresh && g_g123Valid) { snprintf(buf, sizeof(buf), "%.1fV", g_g123Volt); drawDataRow(238, "123V", buf, cv); }
  else drawDataRow(238, "123V", na, cv);
  if (fresh && g_g123Valid) { snprintf(buf, sizeof(buf), "%dC", (int)g_g123Temp); drawDataRow(278, "123T", buf, cv); }
  else drawDataRow(278, "123T", na, cv);
  if (fresh && g_battValid) { snprintf(buf, sizeof(buf), "%.1fV", g_battVolt); drawDataRow(318, "BATT", buf, cv); }
  else drawDataRow(318, "BATT", na, cv);
  const char* st = g_bleConn ? (fresh ? "LIVE" : "WARTE") : "KEIN HUB";
  drawTextCentered(240, 370, st, g_bleConn && fresh ? RGB565(60, 200, 90) : RGB565(200, 120, 50), 2);
  presentFrame();
}

static void drawLambdaPage() {
  if (!ensureFrame()) return;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(45, 150, 70));
  drawTextCentered(240, 58, "LAMBDA", RGB565(70, 200, 100), 5);
  bool fresh = bleFresh() && g_lambdaValid;
  char buf[16];
  if (fresh) snprintf(buf, sizeof(buf), "%.2f", g_lambda);
  else strcpy(buf, "----");
  uint16_t col = RGB565(240, 240, 230);
  if (fresh) {
    if      (g_lambda < 0.97f) col = RGB565(235, 120, 40);
    else if (g_lambda > 1.03f) col = RGB565(80,  160, 240);
    else                        col = RGB565(70,  210, 100);
  } else col = RGB565(110, 60, 60);
  drawTextCentered(240, 198, buf, col, 8);
  if (fresh && g_speedValid) {
    char sp[16]; snprintf(sp, sizeof(sp), "%d km/h", (int)g_speedKmh);
    drawTextCentered(240, 318, sp, RGB565(180, 180, 180), 3);
  }
  const char* st = g_bleConn ? (bleFresh() ? "LIVE" : "WARTE") : "KEIN HUB";
  drawTextCentered(240, 370, st, g_bleConn && bleFresh() ? RGB565(60, 200, 90) : RGB565(200, 120, 50), 2);
  presentFrame();
}

static void drawHubPage() {
  if (!ensureFrame()) return;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(150, 70, 180));
  drawTextCentered(240, 54, "HUB", RGB565(205, 120, 230), 5);
  drawTextCentered(240, 84, g_bleHubName.c_str(), RGB565(150, 150, 150), 2);
  char buf[24];
  drawDataRow(112, "BLE",   g_bleConn ? "OK" : "SCAN",
              g_bleConn ? RGB565(60, 210, 100) : RGB565(220, 130, 50));
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)g_bleRxCnt);
  drawDataRow(152, "RX",    buf, RGB565(235, 235, 225));
  snprintf(buf, sizeof(buf), "%lu MS", g_bleLastRx ? (unsigned long)(millis() - g_bleLastRx) : 0UL);
  drawDataRow(192, "AGE",   buf, RGB565(235, 235, 225));
  snprintf(buf, sizeof(buf), "%.1f V", g_battVolt);
  drawDataRow(232, "BATT",  g_battValid  ? buf : "---", RGB565(235, 235, 225));
  snprintf(buf, sizeof(buf), "%.0f KMH", g_speedKmh);
  drawDataRow(272, "SPEED", g_speedValid ? buf : "---", RGB565(235, 235, 225));
  drawDataRow(312, "IP",    g_ipStr, RGB565(150, 200, 150));
  drawTextCentered(240, 370, "TIP MENU", RGB565(180, 180, 170), 2);
  presentFrame();
}

static void drawSetupPage() {
  if (!ensureFrame()) return;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(185, 150, 45));
  drawTextCentered(240, 54, "SETUP", RGB565(230, 190, 70), 5);
  char buf[28];
  snprintf(buf, sizeof(buf), "%d %%", g_dialScalePct);
  drawDataRow(112, "UHR",   buf, RGB565(235, 235, 225));
  snprintf(buf, sizeof(buf), "%d %%", g_brightnessPct);
  drawDataRow(152, "HELL",  buf, RGB565(235, 235, 225));
  snprintf(buf, sizeof(buf), "%d DEG", g_rotationDeg);
  drawDataRow(192, "ROT",   buf, RGB565(235, 235, 225));
  if (g_featureWifi && strlen(currentWifiSsid()) > 0) {
    snprintf(buf, sizeof(buf), "%s", currentWifiSsid());
    drawDataRow(232, "WIFI", buf,
                WiFi.status() == WL_CONNECTED ? RGB565(60, 210, 100) : RGB565(220, 130, 50));
  } else {
    drawDataRow(232, "WIFI", "AUS", RGB565(220, 130, 50));
  }
  drawDataRow(272, "BLE",   g_featureBle ? (g_bleConn ? "OK" : "AN") : "AUS",
              g_featureBle && g_bleConn ? RGB565(60, 210, 100) : RGB565(220, 130, 50));
  drawDataRow(312, "HUB",   g_bleHubName.c_str(), RGB565(150, 150, 150));
  drawTextCentered(240, 370, "TIP MENU", RGB565(180, 180, 170), 2);
  presentFrame();
}

static void drawCurrentPage() {
  if      (currentPage == 0) drawVdoClock();
  else if (currentPage == 1) drawMenuOverview();
  else if (currentPage == 2) drawMotorPage();
  else if (currentPage == 3) drawLambdaPage();
  else if (currentPage == 4) drawHubPage();
  else if (currentPage == 5) drawSetupPage();
}

// -------- Preferences --------
static void loadSettings() {
  Preferences p;
  p.begin("clock", true);
  g_dialScalePct  = p.getInt("scale",     100);
  g_brightnessPct = p.getInt("bright",    100);
  g_rotationDeg   = p.getInt("rot_deg",   0);
  g_wifiProfile   = p.getUChar("wifi_prof", 0);
  if (g_wifiProfile >= wifiProfileCount()) g_wifiProfile = 0;
  g_featureWifi   = p.getBool("feat_wifi", strlen(currentWifiSsid()) > 0);
  g_featureBle    = p.getBool("feat_ble",  false);
  p.end();
  if (g_dialScalePct  < 30)  g_dialScalePct  = 30;
  if (g_dialScalePct  > 100) g_dialScalePct  = 100;
  if (g_brightnessPct < 5)   g_brightnessPct = 5;
  if (g_brightnessPct > 100) g_brightnessPct = 100;
  g_rotationDeg %= 360;
  if (g_rotationDeg < 0) g_rotationDeg += 360;
  updateRotationCache();
}

static void saveDialScale(int pct) {
  if (pct < 30)  pct = 30;
  if (pct > 100) pct = 100;
  g_dialScalePct = pct;
  Preferences p;
  p.begin("clock", false);
  p.putInt("scale", pct);
  p.end();
}

static void saveBrightness(int pct) {
  if (pct < 5)   pct = 5;
  if (pct > 100) pct = 100;
  g_brightnessPct = pct;
  Preferences p;
  p.begin("clock", false);
  p.putInt("bright", pct);
  p.end();
}

static void updateRotationCache() {
  g_rotationDeg %= 360;
  if (g_rotationDeg < 0) g_rotationDeg += 360;
  float rad = (float)g_rotationDeg * PI / 180.0f;
  g_rotSin = sinf(rad);
  g_rotCos = cosf(rad);
}

static void saveRotation(int deg) {
  g_rotationDeg = deg;
  updateRotationCache();
  Preferences p;
  p.begin("clock", false);
  p.putInt("rot_deg", g_rotationDeg);
  p.end();
}

static void saveFeatures(bool wifi, bool ble) {
  const bool wasBle = g_featureBle;
  g_featureWifi = wifi;
  g_featureBle  = ble;
  Preferences p;
  p.begin("clock", false);
  p.putBool("feat_wifi", g_featureWifi);
  p.putBool("feat_ble",  g_featureBle);
  p.end();
  if (!g_featureWifi) {
    WiFi.disconnect();
    strcpy(g_ipStr, "---");
    g_webStarted = false;
    Serial.println("WiFi: AUS");
  } else if (WiFi.status() != WL_CONNECTED && strlen(currentWifiSsid()) > 0) {
    reconnectWifiProfile();
  }
  if (!g_featureBle) {
    if (bleClient && bleClient->isConnected()) bleClient->disconnect();
    g_bleConn    = false;
    g_bleHubName = "---";
    bleDoConnect = false;
    bleNextScanAt = 0;
    Serial.println("BLE: AUS");
  } else if (!wasBle) {
    NimBLEDevice::init("VDO-Clock");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    bleNextScanAt = millis() + 5000;
    Serial.println("BLE: AN, Scan startet...");
  } else if (!g_bleConn && !bleDoConnect && bleNextScanAt == 0) {
    bleNextScanAt = millis() + 5000;
  }
}

// -------- Web-GUI --------
static void handleWebRoot() {
  struct tm now = {};
  readClockTime(&now);
  char timeStr[16];
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", now.tm_hour, now.tm_min, now.tm_sec);

  String html = F("<!DOCTYPE html><html lang='de'><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>VDO Uhr</title><style>"
    "body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:20px;text-align:center}"
    "h1{color:#e0c040;font-weight:600}.card{background:#1c1c1c;border-radius:12px;padding:18px;margin:14px auto;max-width:420px}"
    ".big{font-size:2.2em;letter-spacing:2px}input[type=range]{width:90%}"
    "button{background:#e0c040;border:0;border-radius:8px;padding:12px 20px;font-size:1em;margin:6px;cursor:pointer}"
    "a{color:#8cf}.val{font-size:1.6em;color:#e0c040}</style></head><body>");
  html += F("<h1>VDO Quartz-Zeit</h1>");
  html += "<div class='card'><div class='big'>" + String(timeStr) + "</div>";
  html += "<div>IP " + String(g_ipStr) + "</div></div>";

  // WLAN-Profil umschalten
  html += F("<div class='card'><h3>WLAN</h3>");
  html += "<div>Aktiv: <b>" + String(currentWifiSsid()) + "</b> &middot; " +
          String(WiFi.status() == WL_CONNECTED ? "verbunden" : "nicht verbunden") + "</div>";
  for (uint8_t i = 0; i < wifiProfileCount(); i++) {
    html += "<a href='/wifi?prof=" + String(i) + "'><button" +
            String(i == g_wifiProfile ? " style='background:#6c6'" : "") + ">" +
            String(WIFI_PROFILES[i].ssid) + "</button></a>";
  }
  html += F("</div>");

  html += F("<div class='card'><h3>Display-Seite</h3>"
    "<a href='/page?p=0'><button>Uhr</button></a>"
    "<a href='/page?p=1'><button>Menu</button></a>"
    "<a href='/page?p=2'><button>Motor</button></a>"
    "<a href='/page?p=3'><button>Lambda</button></a>"
    "<a href='/page?p=4'><button>Hub</button></a>"
    "<a href='/page?p=5'><button>Setup</button></a></div>");

  html += F("<div class='card'><h3>Funktionen</h3><form action='/features' method='get'>");
  html += "<p><label><input type='checkbox' name='wifi' value='1' ";
  html += g_featureWifi ? "checked" : "";
  html += F("> WLAN/Web aktiv</label></p><p><label>"
    "<input type='checkbox' name='ble' value='1' ");
  html += g_featureBle ? "checked" : "";
  html += F(" onchange='this.form.submit()'> BLE-Hub Daten aktiv</label></p>"
    "<button type='submit'>Speichern</button></form></div>");

  html += F("<div class='card'><h3>Spartan-Hub Live</h3>");
  if (!g_featureBle) {
    html += F("<div>BLE: <b>aus</b></div>");
  } else if (g_bleConn) {
    html += F("<div>BLE: <b>verbunden</b></div>");
    html += "<div>Lambda: " + String(g_lambdaValid ? String(g_lambda, 2) : String("---")) + "</div>";
    html += "<div>RPM: " + String((int)g_rpm) + " &nbsp; ADV: " + String(g_adv, 1) + "</div>";
    html += "<div>Batt: " + String(g_battValid ? String(g_battVolt, 1) + " V" : String("---")) + "</div>";
  } else {
    html += F("<div>BLE: scanning/wartet</div>");
  }
  html += "<div>RX: " + String((unsigned long)g_bleRxCnt) + "</div></div>";

  html += F("<div class='card'><h3>Zifferblatt-Gr&ouml;&szlig;e</h3>"
    "<form action='/set' method='get'>"
    "<div class='val'><span id='v'>");
  html += String(g_dialScalePct);
  html += F("</span>%</div>"
    "<input type='range' name='scale' min='30' max='100' step='1' value='");
  html += String(g_dialScalePct);
  html += F("' oninput=\"document.getElementById('v').innerText=this.value\">"
    "<br><button type='submit'>&Uuml;bernehmen</button></form>"
    "<div><a href='/set?scale=100'>100%</a> &middot; <a href='/set?scale=90'>90%</a> &middot; "
    "<a href='/set?scale=80'>80%</a> &middot; <a href='/set?scale=70'>70%</a></div></div>");
  html += F("<div class='card'><h3>Display-Rotation</h3><div class='val'>");
  html += String(g_rotationDeg);
  html += F("&deg;</div><a href='/set?rot_delta=-1'><button>- 1&deg;</button></a>"
            "<a href='/set?rot_delta=1'><button>+ 1&deg;</button></a><br>"
            "<a href='/set?rot_delta=-5'><button>- 5&deg;</button></a>"
            "<a href='/set?rot_delta=5'><button>+ 5&deg;</button></a>"
            "<div><a href='/set?rot=0'>0&deg;</a> &middot; <a href='/set?rot=90'>90&deg;</a> &middot; "
            "<a href='/set?rot=180'>180&deg;</a> &middot; <a href='/set?rot=270'>270&deg;</a></div></div>");
  html += F("<p style='color:#666'>VW T2b Cockpit &middot; ESP32-S3 2.8\"</p></body></html>");
  webServer.send(200, "text/html", html);
}

static void handleWebSet() {
  if (webServer.hasArg("scale")) {
    saveDialScale(webServer.arg("scale").toInt());
    g_redrawPage = true;
    Serial.printf("Web: Zifferblatt-Groesse = %d%%\n", g_dialScalePct);
  }
  if (webServer.hasArg("rot_delta")) {
    saveRotation(g_rotationDeg + webServer.arg("rot_delta").toInt());
    g_redrawPage = true;
    Serial.printf("Web: Rotation = %d deg\n", g_rotationDeg);
  }
  if (webServer.hasArg("rot")) {
    saveRotation(webServer.arg("rot").toInt());
    g_redrawPage = true;
    Serial.printf("Web: Rotation = %d deg\n", g_rotationDeg);
  }
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

static void handleWebFeatures() {
  const bool wifi = webServer.hasArg("wifi");
  const bool ble  = webServer.hasArg("ble");
  saveFeatures(wifi, ble);
  Serial.printf("Web: Funktionen wifi=%s ble=%s\n",
                g_featureWifi ? "on" : "off", g_featureBle ? "on" : "off");
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

static void handleWebPage() {
  if (webServer.hasArg("p")) {
    int page = webServer.arg("p").toInt();
    if (page < 0) page = 0;
    if (page > 5) page = 5;
    currentPage  = static_cast<uint8_t>(page);
    g_redrawPage = true;
    Serial.printf("Web: page=%u\n", currentPage);
  }
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

static void reconnectWifiProfile();    // fwd
static void saveWifiProfile(uint8_t);  // fwd

// WLAN-Profil per Web umschalten: /wifi?prof=N
static void handleWebWifi() {
  if (webServer.hasArg("prof")) {
    int idx = webServer.arg("prof").toInt();
    if (idx < 0) idx = 0;
    saveWifiProfile((uint8_t)idx);
    reconnectWifiProfile();
    Serial.printf("Web: WLAN-Profil -> %d (%s)\n", idx, currentWifiSsid());
  }
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

static void startWebServer() {
  webServer.on("/",        handleWebRoot);
  webServer.on("/set",     handleWebSet);
  webServer.on("/features",handleWebFeatures);
  webServer.on("/page",    handleWebPage);
  webServer.on("/wifi",    handleWebWifi);
  webServer.begin();
  Serial.println("WebGUI: gestartet auf Port 80");
}

// Fallback-Setup-AP: nur AN wenn keine STA-Verbindung besteht, damit das
// Display im verbundenen Normalbetrieb stabil bleibt (kein AP-Dauerbeacon).
static bool     g_apOn = false;
static void manageWifiAp() {
  if (!g_featureWifi) return;
  const bool conn = (WiFi.status() == WL_CONNECTED);
  if (conn && g_apOn) {
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    g_apOn = false;
    Serial.println("WiFi: STA verbunden -> Setup-AP aus");
  } else if (!conn && !g_apOn && millis() > 15000) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("VDO-Clock-Setup", "vdoclock");
    g_apOn = true;
    Serial.printf("WiFi: Setup-AP an -> http://%s  (SSID VDO-Clock-Setup / vdoclock)\n",
                  WiFi.softAPIP().toString().c_str());
    if (!g_webStarted) { startWebServer(); g_webStarted = true; }
  }
}

static void handleSetupLongPress(uint16_t y, uint32_t durMs) {
  Serial.printf("setup long-press y=%u dur=%lu\n", y, (unsigned long)durMs);

  if (y >= 340) {
    currentPage = 1;
    drawMenuOverview();
    Serial.println("setup long: menu");
    return;
  }

  if (y >= 94 && y < 134) {
    int next = (g_dialScalePct < 85) ? 90 : (g_dialScalePct < 95 ? 100 : 80);
    saveDialScale(next);
    drawSetupPage();
    Serial.printf("setup long: dial=%d%%\n", g_dialScalePct);
  } else if (y >= 134 && y < 174) {
    int next = (g_brightnessPct < 63) ? 75 : (g_brightnessPct < 88 ? 100 : 50);
    saveBrightness(next);
    drawSetupPage();
    Serial.printf("setup long: brightness=%d%%\n", g_brightnessPct);
  } else if (y >= 174 && y < 214) {
    // Tap schaltet die 4 Presets durch (Feintuning per Web-GUI)
    int next = (g_rotationDeg < 90) ? 90 : (g_rotationDeg < 180 ? 180 : (g_rotationDeg < 270 ? 270 : 0));
    saveRotation(next);
    drawSetupPage();
    Serial.printf("setup tap: rotation=%d deg\n", g_rotationDeg);
  } else if (y >= 214 && y < 254) {
    cycleWifiProfile();
    drawSetupPage();
    Serial.printf("setup long: wifi profile=%u ssid=%s\n", g_wifiProfile, currentWifiSsid());
  } else if (y >= 254 && y < 294) {
    saveFeatures(g_featureWifi, !g_featureBle);
    drawSetupPage();
    Serial.printf("setup long: ble=%s\n", g_featureBle ? "on" : "off");
  } else {
    drawSetupPage();
    Serial.println("setup long: no action");
  }
}

static void saveWifiProfile(uint8_t idx) {
  uint8_t count = wifiProfileCount();
  if (count == 0) return;
  if (idx >= count) idx = 0;
  g_wifiProfile = idx;
  g_featureWifi = true;
  Preferences p;
  p.begin("clock", false);
  p.putUChar("wifi_prof", g_wifiProfile);
  p.putBool("feat_wifi", true);
  p.end();
}

static void reconnectWifiProfile() {
  if (strlen(currentWifiSsid()) == 0) return;
  g_featureWifi = true;
  strcpy(g_ipStr, "...");
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(currentWifiSsid(), currentWifiPassword());
  Serial.printf("WiFi: Profil %u -> '%s'\n", g_wifiProfile, currentWifiSsid());
}

static void cycleWifiProfile() {
  uint8_t count = wifiProfileCount();
  if (count == 0) return;
  saveWifiProfile((uint8_t)((g_wifiProfile + 1) % count));
  reconnectWifiProfile();
}

void setup() {
  // Cold-Boot Robustness: let power rails settle before touching I2C/display.
  delay(500);

  Serial.begin(115200);
  uint32_t serialWait = millis();
  while (!Serial && millis() - serialWait < 2000) delay(10);
  Serial.println("\n=== Waveshare 2.8C VDO Clock ===");
  Serial.printf("PSRAM found: %s, size: %u bytes\n", psramFound() ? "yes" : "no", ESP.getPsramSize());

  // Backlight diagnostic blink (2x 50ms) before panel init.
  hal_backlight(true);  delay(50);
  hal_backlight(false); delay(50);
  hal_backlight(true);  delay(50);
  hal_backlight(false);

  loadSettings();

  // WiFi and BLE MUST init before the RGB panel. WiFi/BLE PHY init temporarily
  // disables the flash cache; if the RGB VSYNC ISR (in flash) fires during that
  // window the CPU faults. With no panel running yet there is no VSYNC ISR.
  if (g_featureWifi && strlen(currentWifiSsid()) > 0) {
    WiFi.persistent(false);  // keine WiFi-Flash-Schreibzugriffe -> kein Cache-Disable
    WiFi.setSleep(true);     // Modem-Sleep: weniger PSRAM-Bus-Konkurrenz
    WiFi.mode(WIFI_STA);
    WiFi.begin(currentWifiSsid(), currentWifiPassword());
    Serial.printf("WiFi: Verbindung zu '%s' im Hintergrund gestartet\n", currentWifiSsid());
  }

  if (g_featureBle) {
    NimBLEDevice::init("VDO-Clock");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    bleNextScanAt = millis() + 20000;
    Serial.println("BLE: Client initialisiert");
  }

  Serial.println("Display: HAL init...");
  hal_init();
  gt911Init();
  initTimeSource();
  hal_backlight(true);
  drawVdoClock();
  Serial.println("VDO clock drawn.");
}

void loop() {
  static uint32_t lastTouch = 0;
  static uint32_t lastDraw  = 0;
  static String   serialLine;
  static bool     touchActive = false;
  static bool     touchLongHandled = false;
  static uint32_t touchStartMs = 0;
  static uint32_t touchLastSeenMs = 0;
  static uint16_t touchStartX = 0, touchStartY = 0;
  static uint16_t touchLastX = 0, touchLastY = 0;
  uint16_t x = 0, y = 0;

#if FEATURE_TOUCH
  const uint32_t nowMs = millis();
  const bool touchFrame = readTouch(&x, &y);
  const bool touchHeld = touchActive && (nowMs - touchLastSeenMs < 200);
  const bool touchNow = touchFrame || touchHeld;
  if (touchNow) {
    touchSeen = true;
    touchLastSeenMs = nowMs;
    if (touchFrame) {
      touchLastX = x;
      touchLastY = y;
    }
    if (!touchActive) {
      touchActive = true;
      touchLongHandled = false;
      touchStartMs = nowMs;
      touchStartX = x;
      touchStartY = y;
      touchLastX = x;
      touchLastY = y;
    }
    if (currentPage == 5 && !touchLongHandled && nowMs - touchStartMs >= 600) {
      touchLongHandled = true;
      lastTouch = nowMs;
      handleSetupLongPress(touchLastY, nowMs - touchStartMs);
    }
  } else if (touchActive && nowMs - touchLastSeenMs > (currentPage == 5 ? 700UL : 180UL)) {
    const uint32_t durMs = touchLastSeenMs - touchStartMs;
    const uint16_t tapX = touchLastX ? touchLastX : touchStartX;
    const uint16_t tapY = touchLastY ? touchLastY : touchStartY;
    touchActive = false;
    if (!touchLongHandled && durMs < 600 && nowMs - lastTouch > 350) {
      lastTouch = nowMs;
      Serial.printf("touch x=%u y=%u page=%u dur=%lu\n", tapX, tapY, currentPage, (unsigned long)durMs);
      if (currentPage == 0) {
        currentPage = 1;
        drawMenuOverview();
        Serial.println("page: menu");
      } else if (currentPage == 1) {
        // Route by y-position: zones match MENU_ZONE_Y0 / MENU_ZONE_H exactly.
        const uint16_t z0 = MENU_ZONE_Y0;
        const uint16_t zh = MENU_ZONE_H;
        if      (tapY >= z0       && tapY < z0 +   zh) { currentPage = 0; drawVdoClock();   Serial.println("page: clock"); }
        else if (tapY >= z0 +  zh && tapY < z0 + 2*zh) { currentPage = 2; drawMotorPage();  Serial.println("page: motor"); }
        else if (tapY >= z0 +2*zh && tapY < z0 + 3*zh) { currentPage = 3; drawLambdaPage(); Serial.println("page: lambda"); }
        else if (tapY >= z0 +3*zh && tapY < z0 + 4*zh) { currentPage = 4; drawHubPage();    Serial.println("page: hub"); }
        else if (tapY >= z0 +4*zh && tapY < z0 + 5*zh) { currentPage = 5; drawSetupPage();  Serial.println("page: setup"); }
        else { currentPage = 2; drawMotorPage(); Serial.println("page: motor fallback"); }
      } else if (currentPage == 5) {
        // SETUP: kurzer Tap auf eine Zeile aendert die jeweilige Einstellung
        // (Zifferblatt/Helligkeit/Rotation/WLAN/BLE). Tap unten (y>=340) =
        // zurueck ins Menue. Long-Press war zu unzuverlaessig (INT-Pin), Tap
        // funktioniert sicher.
        handleSetupLongPress(tapY, durMs);
      } else {
        // Data pages: short tap advances to next page, wraps to clock.
        currentPage++;
        if (currentPage > 5) currentPage = 0;
        drawCurrentPage();
        Serial.printf("page: next %u\n", currentPage);
      }
    }
  }
#endif

  // Serial commands
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (serialLine.length() > 0) {
        String cmd = serialLine; serialLine = "";
        cmd.trim(); cmd.toLowerCase();
        if      (cmd == "ble:on")  { saveFeatures(g_featureWifi, true); }
        else if (cmd == "ble:off") { saveFeatures(g_featureWifi, false); }
        else if (cmd == "wifi:next") { cycleWifiProfile(); g_redrawPage = true; }
        else if (cmd == "wifi:off")  { saveFeatures(false, g_featureBle); g_redrawPage = true; }
        else if (cmd == "rot:+") { saveRotation(g_rotationDeg + 1); g_redrawPage = true; }
        else if (cmd == "rot:-") { saveRotation(g_rotationDeg - 1); g_redrawPage = true; }
        else if (cmd.startsWith("rot:")) { saveRotation(cmd.substring(4).toInt()); g_redrawPage = true; }
        else if (cmd == "clock")   { currentPage = 0; drawVdoClock(); }
        else { Serial.println("Commands: ble:on | ble:off | wifi:next | wifi:off | rot:+ | rot:- | rot:NN | clock"); }
      }
    } else if (serialLine.length() < 64) {
      serialLine += c;
    }
  }

  // Fallback-Setup-AP verwalten (nur AN wenn keine STA-Verbindung)
  manageWifiAp();

  // WiFi/NTP background tick; redraw clock on fresh sync
  if (wifiNtpTick() && currentPage == 0) drawVdoClock();

  // BLE client tick
  if (g_featureBle) bleTick();

  // Web server
  if (g_webStarted) webServer.handleClient();

  // Redraw after web/serial change
  if (g_redrawPage) {
    g_redrawPage = false;
    drawCurrentPage();
  }

  // Clock: update every second
  if (currentPage == 0 && millis() - lastDraw >= 1000) {
    lastDraw = millis();
    drawVdoClock();
    hal_restart();  // DMA-Resync gegen WiFi/BLE-bedingten Bildversatz/Schwarz
  }
  // Data pages: update at 2 Hz
  if (currentPage >= 2 && currentPage <= 5 && millis() - lastDraw >= 500) {
    lastDraw = millis();
    drawCurrentPage();
  }

  delay(10);
}
