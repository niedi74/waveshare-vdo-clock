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
#ifndef S24_AP_PASS            // S24-Hotspot-Passwort: nur lokal in wifi_secret.h, NIE committen
  #define S24_AP_PASS ""
#endif
#include "hal_waveshare_28c.h"
#include "qmi8658_imu.h"
#include "vdo_dial_480_rgb565.h"
#include "driver/twai.h"
#include <HTTPClient.h>
#include <Update.h>
#include <ESPmDNS.h>
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

// -------- 123TUNE+ direkt (NUS) --------
#define NUS_SVC "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_RX  "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_TX  "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

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

// Letzte Datenquelle der Cockpit-Werte (fuer Anzeige/Diagnose)
static const char* g_lastSrc = "---";

// -------- CAN cockpit client (TWAI, hoert 0x510 vom Spartan-Test-Hub) --------
#define COCKPIT_CAN_ID     0x510
#define COCKPIT_CAN_RX_PIN GPIO_NUM_44
#define COCKPIT_CAN_TX_PIN GPIO_NUM_43

static bool     g_canReady      = false;
static bool     g_canListenOnly = true;   // Display hoert nur mit (kein ACK/TX)
static uint32_t g_canRx         = 0;
static uint32_t g_canIgnored    = 0;
static uint32_t g_canLastRxMs   = 0;

// -------- HTTP-Poll-Client (zieht /api/status vom Spartan-Hub) --------
// g_hubIp darf eine IP ODER ein mDNS-Hostname sein (z.B. "spartanhub.local") -
// per Hostname findet das Display den Hub in jedem Subnetz (Handy-Hotspot!).
static String   g_hubIp           = "192.168.0.91";
static uint32_t g_httpRx          = 0;
static uint32_t g_httpLastRxMs    = 0;
static String   g_hubResolvedIp   = "";        // mDNS-Cache (bei Hostname-Ziel)
static bool     g_mdnsStarted     = false;

// -------- OTA (Firmware-Update per WLAN/Web) --------
static volatile bool   g_otaBusy    = false;
static volatile size_t g_otaRxBytes = 0;

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
static bool      g_featureBuzzer = false;  // default OFF, per Setup/Web schaltbar
static bool      g_feature123    = false;  // 123-Direkt-Fallback: default AUS (sonst Dauer-BLE -> Ruckeln)
static float     g_imuOffPitch   = 0.0f;   // IMU-Nullung (Einbaulage) - Pitch/Roll-Offset
static float     g_imuOffRoll    = 0.0f;
static bool      g_wifiAuto      = true;   // WLAN-Auto-Fallback: S24 > Heim > Hub-AP (verfuegbares Netz)
static bool      g_apOn          = false;  // Setup-AP aktiv?
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

// Drei zur Laufzeit (per WebGUI/Setup) vorwaehlbare WLAN-Profile: Heim / Hub-AP / S24.
// Jedes mit SSID, Passwort und zugehoeriger Hub-IP. Persistent im NVS. Haben Vorrang
// vor den einkompilierten WIFI_PROFILES. g_wifiProfile = aktiver Slot (0..2).
#define WPROF_COUNT 3
struct StaProfile { char ssid[33]; char pass[65]; char hubip[20]; };
static StaProfile g_wprof[WPROF_COUNT];
static const char* const WPROF_LABELS[WPROF_COUNT] = { "Heim", "Hub-AP", "S24" };

static const char* currentWifiSsid() {
  if (g_wifiProfile >= WPROF_COUNT) g_wifiProfile = 0;
  if (g_wprof[g_wifiProfile].ssid[0]) return g_wprof[g_wifiProfile].ssid;
  uint8_t count = wifiProfileCount();
  if (count == 0) return "";
  return WIFI_PROFILES[g_wifiProfile < count ? g_wifiProfile : 0].ssid;
}

static const char* currentWifiPassword() {
  if (g_wifiProfile >= WPROF_COUNT) g_wifiProfile = 0;
  if (g_wprof[g_wifiProfile].ssid[0]) return g_wprof[g_wifiProfile].pass;
  uint8_t count = wifiProfileCount();
  if (count == 0) return "";
  return WIFI_PROFILES[g_wifiProfile < count ? g_wifiProfile : 0].pass;
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
    // Bei aktivem Auto-Fallback uebernimmt wifiAutoTick() das Verbinden.
    if (!g_wifiAuto && millis() - lastTry > 30000) {
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
    Serial.printf("WiFi verbunden, IP: %s (Profil %s)\n", g_ipStr, WPROF_LABELS[g_wifiProfile]);
    Preferences pp; pp.begin("clock", false); pp.putUChar("wifi_prof", g_wifiProfile); pp.end();  // verbundenes Profil merken
  }
  if (!g_webStarted) {
    startWebServer();
    g_webStarted = true;
  }
  if (!g_mdnsStarted) {                 // mDNS-Responder: noetig um Hub per Hostname zu finden
    if (MDNS.begin("vdo-clock")) { g_mdnsStarted = true; Serial.println("mDNS: gestartet (vdo-clock.local)"); }
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

// WLAN-Auto-Fallback: probiert (ohne Scan!) die belegten Profile der Reihe nach
// in Prioritaet S24 > Heim > Hub-AP durch, je ~6 s. KEIN WiFi.scanNetworks() -
// das crasht auf diesem Arduino-Stand, wenn kein Netz in Reichweite ist
// (esp_wifi_scan_start StoreProhibited -> Boot-Loop). WiFi.begin im AP_STA ist ok.
static void wifiAutoTick() {
  if (!g_featureWifi || !g_wifiAuto) return;
  if (WiFi.status() == WL_CONNECTED) return;
  static const uint8_t order[WPROF_COUNT] = { 2, 0, 1 };   // S24 > Heim > Hub-AP
  static uint8_t  oi    = 0;
  static uint32_t tryAt = 0;
  if (millis() < tryAt) return;
  for (uint8_t k = 0; k < WPROF_COUNT; k++) {
    uint8_t slot = order[(oi + k) % WPROF_COUNT];
    if (!g_wprof[slot].ssid[0]) continue;          // leeres Profil ueberspringen
    oi = (uint8_t)((oi + k + 1) % WPROF_COUNT);
    g_wifiProfile = slot;
    if (g_wprof[slot].hubip[0]) g_hubIp = g_wprof[slot].hubip;
    strcpy(g_ipStr, "...");
    WiFi.begin(g_wprof[slot].ssid, g_wprof[slot].pass);
    tryAt = millis() + 6000;                       // diesem Profil 6 s geben
    Serial.printf("WiFi-Auto: versuche %s (%s)\n", WPROF_LABELS[slot], g_wprof[slot].ssid);
    return;
  }
  tryAt = millis() + 5000;                         // kein belegtes Profil
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
  g_lastSrc = "BLE";
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

// -------- CAN cockpit client (TWAI) --------
static bool canFresh() { return g_canLastRxMs != 0 && millis() - g_canLastRxMs < 3000; }

static bool setupCockpitCan() {
  g_canReady = false;
  twai_stop();               // falls schon RUNNING (sonst schlaegt uninstall fehl)
  twai_driver_uninstall();   // idempotent: ignoriert "not installed"
  twai_mode_t mode = g_canListenOnly ? TWAI_MODE_LISTEN_ONLY : TWAI_MODE_NORMAL;
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(COCKPIT_CAN_TX_PIN, COCKPIT_CAN_RX_PIN, mode);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  esp_err_t err = twai_driver_install(&g, &t, &f);
  if (err != ESP_OK) { Serial.printf("CAN: install failed %s\n", esp_err_to_name(err)); return false; }
  err = twai_start();
  if (err != ESP_OK) { Serial.printf("CAN: start failed %s\n", esp_err_to_name(err)); twai_driver_uninstall(); return false; }
  g_canReady = true;
  Serial.printf("CAN: cockpit RX ready id=0x%03X TX=%d RX=%d mode=%s 500k\n",
                COCKPIT_CAN_ID, (int)COCKPIT_CAN_TX_PIN, (int)COCKPIT_CAN_RX_PIN,
                g_canListenOnly ? "listen" : "normal");
  return true;
}

// 0x510-Frame (8 Byte): [lambda_x1000 BE][rpm BE][adv_x10 BE int16][map u8][flags]
static void applyCockpitCanFrame(const twai_message_t& msg) {
  const uint16_t lambdaX1000 = ((uint16_t)msg.data[0] << 8) | msg.data[1];
  const uint16_t rpm         = ((uint16_t)msg.data[2] << 8) | msg.data[3];
  const int16_t  advX10      = (int16_t)(((uint16_t)msg.data[4] << 8) | msg.data[5]);
  const uint8_t  flags       = msg.data[7];
  g_lambda      = lambdaX1000 / 1000.0f;
  g_lambdaValid = (flags & 0x01) && lambdaX1000 > 0;
  if (flags & 0x02) {            // tune fresh -> rpm/adv/map gueltig
    g_rpm = rpm;
    g_adv = advX10 / 10.0f;
    g_map = msg.data[6];
  }
  g_lastSrc     = "CAN";
  g_canRx++;
  g_canLastRxMs = millis();
}

static void cockpitCanTick() {
  if (!g_canReady) return;
  twai_status_info_t st;                          // Treiber wirklich installiert?
  if (twai_get_status_info(&st) != ESP_OK) {      // sonst assertet twai_receive auf NULL-Queue
    g_canReady = false;
    return;
  }
  twai_message_t msg;
  uint8_t drained = 0;
  while (drained < 8 && twai_receive(&msg, 0) == ESP_OK) {
    drained++;
    if (msg.extd || msg.identifier != COCKPIT_CAN_ID || msg.data_length_code != 8) { g_canIgnored++; continue; }
    applyCockpitCanFrame(msg);
  }
}

static void runCanTest() {
  Serial.printf("CAN TEST: SN65HVD230 GPIO43=TXD/D GPIO44=RXD/R mode=%s\n",
                g_canListenOnly ? "listen" : "normal");
  if (!g_canReady) setupCockpitCan();
  twai_status_info_t s = {};
  if (g_canReady && twai_get_status_info(&s) == ESP_OK) {
    Serial.printf("CAN TEST: ready=1 state=%d rx=%lu ignored=%lu msgs_to_rx=%u rx_err=%u bus_err=%u arb_lost=%u\n",
                  (int)s.state, (unsigned long)g_canRx, (unsigned long)g_canIgnored,
                  (unsigned)s.msgs_to_rx, (unsigned)s.rx_error_counter,
                  (unsigned)s.bus_error_count, (unsigned)s.arb_lost_count);
  } else {
    Serial.printf("CAN TEST: ready=%d (install/status fail)\n", g_canReady ? 1 : 0);
  }
  Serial.println("CAN TEST: done");
}

// TX-Selbsttest: kurz NORMAL, EIN Single-Shot-Frame senden, auf ACK pruefen.
// ACK => ein anderer aktiver Knoten haengt am Bus (Bus lebt). Kein ACK => allein.
static void runCanPing() {
  const bool wasListen = g_canListenOnly;
  if (g_canListenOnly) { g_canListenOnly = false; setupCockpitCan(); }
  if (!g_canReady) {
    Serial.println("CAN PING: CAN nicht bereit");
    if (wasListen) { g_canListenOnly = true; setupCockpitCan(); }
    return;
  }
  twai_status_info_t s0 = {}; twai_get_status_info(&s0);
  twai_message_t m = {};
  m.identifier = 0x7FE;
  m.ss = 1;                       // Single-Shot: kein Retransmit-Sturm auf den Bus
  m.data_length_code = 2; m.data[0] = 0xAA; m.data[1] = 0x55;
  esp_err_t err = twai_transmit(&m, pdMS_TO_TICKS(200));
  delay(200);
  twai_status_info_t s1 = {}; twai_get_status_info(&s1);
  const bool acked = (err == ESP_OK) && (s1.tx_error_counter <= s0.tx_error_counter) &&
                     (s1.msgs_to_tx == 0) && (s1.state == TWAI_STATE_RUNNING);
  Serial.printf("CAN PING: queued=%s state=%d tx_err %u->%u msgs_to_tx=%u bus_err=%u\n",
                esp_err_to_name(err), (int)s1.state,
                (unsigned)s0.tx_error_counter, (unsigned)s1.tx_error_counter,
                (unsigned)s1.msgs_to_tx, (unsigned)s1.bus_error_count);
  if (acked) Serial.println("CAN PING: ACK -> anderer aktiver Knoten am Bus (Bus lebt) -> RX-Ader R->GPIO44 pruefen.");
  else       Serial.println("CAN PING: KEIN ACK -> Touch allein / Bus still / Tap isoliert (TX-Ader/GND/Versorgung/Bus).");
  if (wasListen) { g_canListenOnly = true; setupCockpitCan(); }
}

// -------- HTTP-Poll-Client --------
static bool httpFresh() { return g_httpLastRxMs != 0 && millis() - g_httpLastRxMs < 3000; }

// Minimaler JSON-Feldparser: sucht "key": und liest die folgende Zahl.
static bool jsonNum(const String& s, const char* key, float& out) {
  String k = String("\"") + key + "\":";
  int i = s.indexOf(k);
  if (i < 0) return false;
  out = s.substring(i + k.length()).toFloat();
  return true;
}
static bool jsonTrue(const String& s, const char* key) {
  return s.indexOf(String("\"") + key + "\":true") >= 0;
}

static bool hubIsDottedIp() {
  if (g_hubIp.length() == 0) return false;
  for (size_t i = 0; i < g_hubIp.length(); i++) {
    char c = g_hubIp[i];
    if (!((c >= '0' && c <= '9') || c == '.')) return false;
  }
  return true;
}

// Ziel-Adresse des Hubs: feste IP direkt, sonst per mDNS aufloesen (gecacht).
static String hubTarget() {
  // Hub-AP-Profil: der Hub ist immer das Gateway des AP -> nutzen, egal welches
  // (zufaellige) Subnetz der Hub-AP gerade vergibt.
  if (g_wifiProfile == 1 && WiFi.status() == WL_CONNECTED) {
    IPAddress gw = WiFi.gatewayIP();
    if ((uint32_t)gw != 0) return gw.toString();
  }
  if (hubIsDottedIp()) return g_hubIp;
  if (!g_mdnsStarted) return g_hubResolvedIp;          // mDNS noch nicht bereit
  // Cache nutzen solange Daten frisch sind; sonst gedrosselt neu aufloesen.
  if (g_hubResolvedIp.length() && httpFresh()) return g_hubResolvedIp;
  static uint32_t lastTry = 0;
  if (g_hubResolvedIp.length() && millis() - lastTry < 5000) return g_hubResolvedIp;
  lastTry = millis();
  String name = g_hubIp;
  int dot = name.indexOf(".local");
  if (dot > 0) name = name.substring(0, dot);
  IPAddress ip = MDNS.queryHost(name.c_str(), 1500);
  if (ip != IPAddress(0, 0, 0, 0)) {
    g_hubResolvedIp = ip.toString();
    Serial.printf("mDNS: %s -> %s\n", g_hubIp.c_str(), g_hubResolvedIp.c_str());
  }
  return g_hubResolvedIp;
}

static void httpPollTick() {
  if (!g_featureWifi || WiFi.status() != WL_CONNECTED || g_hubIp.length() == 0) return;
  static uint32_t last = 0;
  static uint8_t  failStreak = 0;
  // Backoff: bei totem Hub nur alle 4s pollen, sonst blockiert der GET den Loop
  // dauernd (Timeout) -> Touch wird traege. Bei Erfolg wieder flott (500ms).
  const uint32_t interval = (failStreak >= 3) ? 4000 : 500;
  if (millis() - last < interval) return;
  last = millis();

  String tgt = hubTarget();
  if (tgt.length() == 0) { if (failStreak < 200) failStreak++; return; }  // Hostname (noch) nicht aufloesbar
  HTTPClient http;
  String url = "http://" + tgt + "/api/status";
  if (!http.begin(url)) { if (failStreak < 200) failStreak++; return; }
  http.setConnectTimeout(300);     // kurz halten -> Loop/Touch bleibt reaktiv
  http.setTimeout(400);
  int code = http.GET();
  if (code == 200) {
    String b = http.getString();
    float v;
    if (jsonNum(b, "rpm", v))       g_rpm = v;
    if (jsonNum(b, "advance", v))   g_adv = v;
    if (jsonNum(b, "map", v))       g_map = v;
    if (jsonNum(b, "lambda", v))  { g_lambda = v; g_lambdaValid = jsonTrue(b, "valid") && v > 0; }
    if (jsonNum(b, "tune_temp", v)) g_g123Temp = v;
    if (jsonNum(b, "volt", v))      g_g123Volt = v;
    if (jsonNum(b, "tune_amp", v))  g_g123Coil = v;
    if (jsonNum(b, "speed_kmh", v)){ g_speedKmh = v; g_speedValid = true; }
    g_g123Valid   = jsonTrue(b, "tune_connected");
    g_lastSrc     = "HTTP";
    g_httpRx++;
    if (g_httpRx == 1) Serial.printf("HTTP: erste Daten von %s\n", tgt.c_str());
    g_httpLastRxMs = millis();
    failStreak = 0;
  } else {
    if (failStreak < 200) failStreak++;
  }
  http.end();
}

// ===================== 123TUNE+ direkter BLE-Fallback =====================
// Verbindet sich direkt zur 123TUNE+ (NUS), wenn HTTP/CAN keine Daten liefern
// und der BLE-Hub-Client nicht aktiv ist. Single-Central: sobald HTTP/CAN wieder
// Daten bringen, wird das 123 wieder freigegeben (sonst Konflikt mit dem Hub).
static NimBLEClient*               tune123Client = nullptr;
static NimBLERemoteCharacteristic* tune123Rx     = nullptr;   // NUS RX (Display->123)
static NimBLEAddress               tune123Target;
static volatile bool g_tune123DoConnect = false;
static bool     g_tune123Conn       = false;
static uint32_t g_tune123LastRxMs   = 0;
static uint32_t g_tune123NextScanAt = 0;
static uint32_t g_tune123KeepAt     = 0;
static bool     g_bleInited         = false;

static bool tune123Fresh() { return g_tune123LastRxMs != 0 && millis() - g_tune123LastRxMs < 3000; }

static void bleEnsureInit() {                 // NimBLE einmal initialisieren (vor RGB-Panel!)
  if (g_bleInited) return;
  NimBLEDevice::init("VDO-Clock");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  g_bleInited = true;
}

static int hexNibble(uint8_t c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0;
}

// Datenpaket: [opcode][hi_ascii_hex][lo_ascii_hex]
static void onTune123Notify(NimBLERemoteCharacteristic*, uint8_t* d, size_t n, bool) {
  if (n < 3) return;
  int hi = hexNibble(d[1]), lo = hexNibble(d[2]);
  int raw = (hi << 4) | lo;
  switch (d[0]) {
    case 0x30: g_rpm = hi * 800.0f + lo * 50.0f; break;
    case 0x31: g_adv = hi * 3.2f + lo * 0.2f;    break;
    case 0x32: g_map = (float)raw;               break;
    case 0x33: g_g123Temp = (float)(raw - 30); g_g123Valid = true; break;
    case 0x35: g_g123Coil = raw / 8.65f;       g_g123Valid = true; break;
    case 0x41: g_g123Volt = raw / 4.54f;       g_g123Valid = true; break;
    default: break;                              // u.a. 0x42 ignorieren
  }
  g_lambdaValid = false;                         // 123 liefert kein Lambda
  g_tune123LastRxMs = millis();
  g_lastSrc = "123";
}

class Tune123ClientCB : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient*) override { Serial.println("123: verbunden"); }
  void onDisconnect(NimBLEClient*, int reason) override {
    g_tune123Conn = false; tune123Rx = nullptr;
    Serial.printf("123: getrennt (reason=%d)\n", reason);
    g_tune123NextScanAt = millis() + 3000;
  }
  bool onConnParamsUpdateRequest(NimBLEClient*, const ble_gap_upd_params*) override { return true; }
};

class Tune123ScanCB : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    String name = dev->getName().c_str(); name.toLowerCase();
    bool match = name.indexOf("123") >= 0 || name.indexOf("tune") >= 0 ||
                 name.indexOf("raytac") >= 0 || dev->isAdvertisingService(NimBLEUUID(NUS_SVC));
    if (match) {
      tune123Target = dev->getAddress();
      g_tune123DoConnect = true;
      NimBLEDevice::getScan()->stop();
      Serial.printf("123: gefunden %s (%s)\n", tune123Target.toString().c_str(), name.c_str());
    }
  }
  void onScanEnd(const NimBLEScanResults&, int) override {
    if (!g_tune123Conn && !g_tune123DoConnect) g_tune123NextScanAt = millis() + 5000;
  }
};

static Tune123ClientCB tune123ClientCB;
static Tune123ScanCB   tune123ScanCB;

static void tune123Connect() {
  g_tune123DoConnect = false;
  if (!tune123Client) {
    tune123Client = NimBLEDevice::createClient();
    tune123Client->setClientCallbacks(&tune123ClientCB, false);
  }
  if (!tune123Client->connect(tune123Target, true, false, false)) {
    Serial.println("123: Connect fehlgeschlagen");
    g_tune123NextScanAt = millis() + 4000;
    return;
  }
  auto* svc = tune123Client->getService(NUS_SVC);
  if (!svc) { Serial.println("123: kein NUS"); tune123Client->disconnect(); return; }
  auto* tx = svc->getCharacteristic(NUS_TX);
  tune123Rx = svc->getCharacteristic(NUS_RX);
  if (!tx || !tune123Rx) { Serial.println("123: NUS-Char fehlt"); tune123Client->disconnect(); return; }
  if (!tx->subscribe(true, onTune123Notify, true)) {
    Serial.println("123: subscribe FAIL"); tune123Client->disconnect(); return;
  }
  // Auth (write ohne Response): I@\r , dann PW1234!\r (Antwort wird ignoriert)
  tune123Rx->writeValue((const uint8_t*)"I@\r", 3, false);
  delay(50);
  tune123Rx->writeValue((const uint8_t*)"PW1234!\r", 8, false);
  g_tune123Conn   = true;
  g_tune123KeepAt = millis() + 1000;
  Serial.println("123: subscribe OK + Auth gesendet");
}

static void tune123ScanTick() {
  if (!g_feature123) return;                     // Fallback nur wenn bewusst aktiviert (sonst kein BLE -> fluessig)
  if (g_featureBle) return;                      // wenn Hub-BLE aktiv: Radio gehoert dem Hub-Client
  if (httpFresh() || canFresh()) {               // Hub-Quelle da -> 123 freigeben
    if (g_tune123Conn && tune123Client) { tune123Client->disconnect(); g_tune123Conn = false; }
    return;
  }
  if (g_tune123Conn) return;
  if (g_tune123DoConnect) { tune123Connect(); return; }
  if (g_tune123NextScanAt != 0 && millis() < g_tune123NextScanAt) return;
  bleEnsureInit();
  auto* s = NimBLEDevice::getScan();
  if (s->isScanning()) return;
  s->setScanCallbacks(&tune123ScanCB);
  s->setActiveScan(true);                        // Name lesen -> active scan
  s->setInterval(160); s->setWindow(60);
  g_tune123NextScanAt = millis() + 8000;
  s->start(5000, false);
  Serial.println("123: Scan...");
}

static void tune123KeepaliveTick() {             // alle 1000ms "$\r"
  if (!g_tune123Conn || !tune123Rx) return;
  if (millis() < g_tune123KeepAt) return;
  g_tune123KeepAt = millis() + 1000;
  tune123Rx->writeValue((const uint8_t*)"$\r", 2, false);
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
  if (gt911Probe()) {
    return;
  }
  Serial.println("GT911: reset/probe INT high");
  gt911ResetAddressMode(true);
  if (gt911Probe()) {
    return;
  }
  Serial.println("GT911: not found on 0x5D/0x14");
}

static bool readTouch(uint16_t *x, uint16_t *y) {
  uint8_t status = 0;
  if (!i2cRegRead16(gt911Addr, GT911_READ_XY, &status, 1)) {
    uint8_t other = (gt911Addr == GT911_ADDR_PRIMARY) ? GT911_ADDR_ALT : GT911_ADDR_PRIMARY;
    if (!i2cRegRead16(other, GT911_READ_XY, &status, 1)) {
      return false;
    }
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
  if (!ok) {
    return false;
  }
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
  if (pct > 150) pct = 150;
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
#define MENU_ZONE_Y0  100
#define MENU_ZONE_H    50    // 6 Tiles a 50px

static void drawMenuOverview() {
  if (!ensureFrame()) return;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(80, 80, 75));
  drawTextCentered(240, 50, "MENU", RGB565(235, 235, 225), 6);
  // 6 Tiles, inset 2px from zone boundary so visual == touchable area
  drawMenuTile(88, MENU_ZONE_Y0 +   0 + 2, 304, MENU_ZONE_H - 4, "UHR",    RGB565(200, 40,  35));
  drawMenuTile(88, MENU_ZONE_Y0 +  50 + 2, 304, MENU_ZONE_H - 4, "MOTOR",  RGB565(40,  150, 210));
  drawMenuTile(88, MENU_ZONE_Y0 + 100 + 2, 304, MENU_ZONE_H - 4, "LAMBDA", RGB565(60,  185, 90));
  drawMenuTile(88, MENU_ZONE_Y0 + 150 + 2, 304, MENU_ZONE_H - 4, "HUB",    RGB565(190, 90,  210));
  drawMenuTile(88, MENU_ZONE_Y0 + 200 + 2, 304, MENU_ZONE_H - 4, "IMU",    RGB565(200, 100, 50));
  drawMenuTile(88, MENU_ZONE_Y0 + 250 + 2, 304, MENU_ZONE_H - 4, "SETUP",  RGB565(210, 170, 45));
  char ipLine[32];
  snprintf(ipLine, sizeof(ipLine), "IP %s", g_ipStr);
  drawTextCentered(240, 424, ipLine, RGB565(150, 200, 150), 2);
  if (g_featureWifi && strlen(currentWifiSsid()) > 0) {
    drawTextCentered(240, 446, currentWifiSsid(), RGB565(120, 150, 150), 2);
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

// -------- Palette im 123TUNE+-Cockpit-Stil --------
// Schwarzes Zifferblatt, heller Chrom-Ring, weisse Striche/Ziffern, orange Zeiger.
#define VDO_FACE    RGB565(12, 12, 14)     // schwarzes Zifferblatt
#define VDO_CREAM   RGB565(238, 238, 236)  // weiss (Striche/Ziffern/Text)
#define VDO_CREAMD  RGB565(122, 122, 126)  // gedimmtes weiss (Nebenstriche/Label)
#define VDO_RED     RGB565(220, 44, 32)    // rote Warnung
#define VDO_BEZEL   RGB565(206, 210, 214)  // heller Chrom-Ring
#define VDO_BEZELD  RGB565(90, 94, 100)    // dunkle Bezel-Innenkante
#define VDO_NEEDLE  RGB565(224, 122, 36)   // orange Zeiger (123TUNE+)
#define VDO_BRASS   RGB565(224, 122, 36)   // orange Nabe
#define VDO_HUBDK   RGB565(70, 32, 6)      // Nabenkern (dunkel-orange)

// -------- Umschaltbare Anzeige-Stile (Themes) --------
struct GaugeTheme {
  uint16_t face, bezel, bezelDk;
  bool     chrome;       // Chrom-Ring (true) vs keiner (digital)
  bool     bandTach;     // digital: gestreiftes Tacho-Band + roter Bereich
  bool     redlineMark;  // rote Striche/Ziffern >=6000 (diskrete Stile)
  bool     redTip;       // rote Zeigerspitze am Tacho
  uint16_t tickMaj, tickMin, numCol;
  uint16_t needle, hub, hubDk;
  bool     miniAccent;   // digital: bunte Mini-Gauges (Akzent + Fuellung)
  uint16_t txt, txtDim;
  uint16_t liveCol, statusBad;
  bool     lambda3;      // 3-farbiges Lambda (digital) vs creme/weiss + rote Warnung
  uint16_t ground;       // Horizont-/Boden-Ton (IMU-Seite)
};

static const GaugeTheme THEME_DIGITAL = {
  RGB565(0,0,0), 0, 0, false, true, false, true,
  RGB565(190,190,190), RGB565(190,190,190), RGB565(235,235,225),
  RGB565(235,235,225), RGB565(200,200,190), RGB565(40,40,40),
  true, RGB565(235,235,225), RGB565(120,120,120),
  RGB565(60,200,90), RGB565(200,120,50), true, RGB565(28,24,20) };

static const GaugeTheme THEME_VDO = {
  RGB565(40,42,46), RGB565(214,218,222), RGB565(110,114,120), true, false, true, true,
  RGB565(234,230,208), RGB565(150,148,130), RGB565(234,230,208),
  RGB565(234,230,208), RGB565(166,122,42), RGB565(40,30,16),
  false, RGB565(234,230,208), RGB565(150,148,130),
  RGB565(234,230,208), RGB565(220,44,32), false, RGB565(30,32,36) };

static const GaugeTheme THEME_123 = {
  RGB565(12,12,14), RGB565(206,210,214), RGB565(90,94,100), true, false, false, false,
  RGB565(238,238,236), RGB565(122,122,126), RGB565(238,238,236),
  RGB565(224,122,36), RGB565(224,122,36), RGB565(70,32,6),
  false, RGB565(238,238,236), RGB565(122,122,126),
  RGB565(238,238,236), RGB565(220,44,32), false, RGB565(22,22,26) };

static uint8_t g_motorStyle = 2;   // 0=digital, 1=vdo, 2=123tune+
static const char* const MOTOR_STYLE_NAMES[] = { "DIGITAL", "VDO", "123TUNE+" };
static const GaugeTheme& gTheme() {
  return g_motorStyle == 0 ? THEME_DIGITAL : g_motorStyle == 1 ? THEME_VDO : THEME_123;
}

// -------- Graphical gauge primitives (cockpit dashboard) --------
// Angle convention: degrees, 0 = east, 90 = south (screen y grows downward),
// 270 = north (up). Matches drawHand's cos/sin usage.
static void plotRadial(int cx, int cy, float angleDeg, int rInner, int rOuter,
                       uint16_t color, int thickness) {
  float a  = angleDeg * (float)PI / 180.0f;
  float ca = cosf(a), sa = sinf(a);
  drawLineFast(cx + (int)(ca * rInner), cy + (int)(sa * rInner),
               cx + (int)(ca * rOuter), cy + (int)(sa * rOuter), color, thickness);
}

static void drawArcBand(int cx, int cy, int rInner, int rOuter,
                        float startDeg, float endDeg, uint16_t color) {
  if (endDeg < startDeg) { float t = startDeg; startDeg = endDeg; endDeg = t; }
  for (float d = startDeg; d <= endDeg; d += 1.5f)
    plotRadial(cx, cy, d, rInner, rOuter, color, 2);
}

static float gaugeAngle(float value, float vmin, float vmax, float startDeg, float endDeg) {
  if (vmax <= vmin) return startDeg;
  float f = (value - vmin) / (vmax - vmin);
  if (f < 0) f = 0; else if (f > 1) f = 1;
  return startDeg + f * (endDeg - startDeg);
}

// Small arc gauge: 180deg top semicircle, value low=left .. high=right.
// Stil je nach Theme: digital (bunt + Fuellung) oder monochrom (creme/weiss).
static void drawMiniGauge(int cx, int cy, int r, float value, float vmin, float vmax,
                          const char* label, const char* valStr,
                          uint16_t accent, bool valid, const GaugeTheme& t) {
  const float A0 = 180.0f, A1 = 360.0f;
  if (t.miniAccent) {                              // digital: bunt + gefuellt
    drawArcBand(cx, cy, r - 5, r, A0, A1, RGB565(60, 60, 64));
    if (valid) { float va = gaugeAngle(value, vmin, vmax, A0, A1);
                 drawArcBand(cx, cy, r - 5, r, A0, va, accent); }
    plotRadial(cx, cy, A0,            r - 8, r, RGB565(120, 120, 120), 2);
    plotRadial(cx, cy, (A0 + A1) / 2, r - 8, r, RGB565(120, 120, 120), 2);
    plotRadial(cx, cy, A1,            r - 8, r, RGB565(120, 120, 120), 2);
    float na = gaugeAngle(valid ? value : vmin, vmin, vmax, A0, A1);
    plotRadial(cx, cy, na, 0, r - 9, valid ? RGB565(235, 235, 225) : RGB565(90, 50, 50), 3);
    fillCircleFast(cx, cy, 4, RGB565(200, 200, 190));
    drawTextCentered(cx, cy - r - 16, label, accent, 2);
    drawTextCentered(cx, cy + 8, valStr, valid ? RGB565(235, 235, 225) : RGB565(110, 60, 60), 2);
    return;
  }
  for (int i = 0; i <= 8; i++) {                   // diskrete Striche
    float a = A0 + (A1 - A0) * (i / 8.0f);
    bool major = (i % 4) == 0;
    plotRadial(cx, cy, a, r - (major ? 9 : 5), r, major ? t.tickMaj : t.tickMin, 2);
  }
  float na = gaugeAngle(valid ? value : vmin, vmin, vmax, A0, A1);
  plotRadial(cx, cy, na, 0, r - 8, valid ? t.needle : t.tickMin, 3);
  fillCircleFast(cx, cy, 4, t.hub);
  fillCircleFast(cx, cy, 2, t.hubDk);
  drawTextCentered(cx, cy - r - 16, label, t.txtDim, 2);
  drawTextCentered(cx, cy + 8, valStr, valid ? t.txt : t.txtDim, 2);
}

// Main RPM tachometer: 240deg sweep, Gap unten, Rand-Zeiger (Mitte frei).
// Stil je nach Theme: digital (gestreiftes Band + roter Bereich) oder diskrete Striche.
static const uint16_t TACH_RED = RGB565(220, 44, 32);
static void drawTach(int cx, int cy, float rpm, bool valid, const GaugeTheme& t) {
  const float A0 = 150.0f, A1 = 390.0f;
  if (t.bandTach) {                                 // digital: Streifenband
    const int rIn = 152, rOut = 212;
    drawArcBand(cx, cy, rIn, rOut, A0, A1, RGB565(45, 45, 50));
    drawArcBand(cx, cy, rIn, rOut, gaugeAngle(6000, 0, 8000, A0, A1), A1, RGB565(150, 30, 28));
    for (int k = 0; k <= 8; k++) {
      float a  = gaugeAngle(k * 1000, 0, 8000, A0, A1);
      uint16_t tc = (k >= 6) ? RGB565(235, 80, 70) : RGB565(190, 190, 190);
      plotRadial(cx, cy, a, rIn, rOut, tc, (k % 2) ? 2 : 3);
      float ar = a * (float)PI / 180.0f;
      int lx = cx + (int)(cosf(ar) * (rIn - 18));
      int ly = cy + (int)(sinf(ar) * (rIn - 18));
      char n[3]; snprintf(n, sizeof(n), "%d", k);
      drawTextCentered(lx, ly - 7, n, tc, 2);
    }
    float pa = gaugeAngle(valid ? rpm : 0, 0, 8000, A0, A1);
    plotRadial(cx, cy, pa, rIn - 6, rOut + 2, t.needle, 6);
    plotRadial(cx, cy, pa, rOut - 16, rOut + 2, TACH_RED, 6);
    return;
  }
  const int rOut = 208;                             // diskrete Striche
  for (int v = 0; v <= 8000; v += 200) {
    float a = gaugeAngle(v, 0, 8000, A0, A1);
    bool major = (v % 1000) == 0;
    uint16_t tc = (t.redlineMark && v >= 6000) ? TACH_RED : (major ? t.tickMaj : t.tickMin);
    plotRadial(cx, cy, a, rOut - (major ? 24 : 12), rOut, tc, major ? 3 : 2);
  }
  for (int k = 0; k <= 8; k++) {
    float a  = gaugeAngle(k * 1000, 0, 8000, A0, A1);
    float ar = a * (float)PI / 180.0f;
    int lx = cx + (int)(cosf(ar) * (rOut - 42));
    int ly = cy + (int)(sinf(ar) * (rOut - 42));
    char n[3]; snprintf(n, sizeof(n), "%d", k);
    drawTextCentered(lx, ly - 10, n, (t.redlineMark && k >= 6) ? TACH_RED : t.numCol, 3);
  }
  float pa = gaugeAngle(valid ? rpm : 0, 0, 8000, A0, A1);
  plotRadial(cx, cy, pa, 150, rOut + 2, t.needle, 6);
  if (t.redTip) plotRadial(cx, cy, pa, rOut - 16, rOut + 2, TACH_RED, 6);
}

static void drawMotorPage() {
  if (!ensureFrame()) return;
  const GaugeTheme& t = gTheme();
  fillFrame(t.face);
  if (t.chrome) {
    drawCircleLine(240, 240, 236, 7, t.bezel);           // heller Chrom-Ring
    drawCircleLine(240, 240, 229, 2, t.bezelDk);          // dunkle Innenkante
  }
  const bool fresh = bleFresh() || canFresh() || httpFresh() || tune123Fresh();
  char buf[16];

  drawTach(240, 240, g_rpm, fresh, t);

  // RPM digital (oben, innerhalb des Rings)
  if (fresh) snprintf(buf, sizeof(buf), "%d", (int)g_rpm); else strcpy(buf, "----");
  drawTextCentered(240, 92, buf, fresh ? t.txt : t.txtDim, 5);
  drawTextCentered(240, 134, "RPM", t.txtDim, 2);

  // Vier Mini-Gauges (Akzentfarben nur im Digital-Stil genutzt)
  const bool g123 = fresh && g_g123Valid;
  if (fresh) snprintf(buf, sizeof(buf), "%d", (int)g_adv); else strcpy(buf, "--");
  drawMiniGauge(148, 196, 36, g_adv, 0, 50, "ADV", buf, RGB565(40, 150, 210), fresh, t);
  if (fresh) snprintf(buf, sizeof(buf), "%d", (int)g_map); else strcpy(buf, "--");
  drawMiniGauge(332, 196, 36, g_map, 0, 200, "KPA", buf, RGB565(60, 185, 90), fresh, t);
  if (g123) snprintf(buf, sizeof(buf), "%d", (int)g_g123Temp); else strcpy(buf, "--");
  drawMiniGauge(148, 324, 36, g_g123Temp, 0, 120, "TEMP", buf, RGB565(210, 120, 50), g123, t);
  if (g123) snprintf(buf, sizeof(buf), "%.1f", g_g123Volt); else strcpy(buf, "--");
  drawMiniGauge(332, 324, 36, g_g123Volt, 10, 15, "VOLT", buf, RGB565(210, 180, 60), g123, t);

  // Lambda zentral
  drawTextCentered(240, 196, "LAMBDA", t.txtDim, 2);
  uint16_t lcol;
  if (fresh && g_lambdaValid) {
    snprintf(buf, sizeof(buf), "%.2f", g_lambda);
    if (t.lambda3) {
      if      (g_lambda < 0.97f) lcol = RGB565(235, 120, 40);
      else if (g_lambda > 1.03f) lcol = RGB565(80, 160, 240);
      else                       lcol = RGB565(70, 210, 100);
    } else {
      lcol = (g_lambda < 0.95f || g_lambda > 1.05f) ? TACH_RED : t.txt;
    }
  } else { strcpy(buf, "----"); lcol = t.txtDim; }
  drawTextCentered(240, 216, buf, lcol, 6);

  // AMP (Zuendspulenstrom) + Tempo, dann Status, im unteren Ring-Gap
  char line[24];
  char amp[10], spd[10];
  if (g123)        snprintf(amp, sizeof(amp), "%.1fA", g_g123Coil); else strcpy(amp, "--");
  if (fresh && g_speedValid) snprintf(spd, sizeof(spd), "%dKMH", (int)g_speedKmh); else strcpy(spd, "--");
  snprintf(line, sizeof(line), "AMP %s  %s", amp, spd);
  drawTextCentered(240, 396, line, t.txtDim, 2);

  char st[16];
  if (fresh)            snprintf(st, sizeof(st), "LIVE %s", g_lastSrc);
  else if (g_bleConn)   strcpy(st, "WARTE");
  else if (g_canReady)  strcpy(st, "CAN WARTE");
  else                  strcpy(st, "KEIN HUB");
  drawTextCentered(240, 424, st, fresh ? t.liveCol : t.statusBad, 2);
  presentFrame();
}

static void drawLambdaPage() {
  if (!ensureFrame()) return;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(45, 150, 70));
  drawTextCentered(240, 58, "LAMBDA", RGB565(70, 200, 100), 5);
  bool live  = bleFresh() || canFresh() || httpFresh() || tune123Fresh();
  bool fresh = live && g_lambdaValid;
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
  char st[16];
  if (live)            snprintf(st, sizeof(st), "LIVE %s", g_lastSrc);
  else if (g_bleConn)  strcpy(st, "WARTE");
  else if (g_canReady) strcpy(st, "CAN WARTE");
  else                 strcpy(st, "KEIN HUB");
  drawTextCentered(240, 370, st, live ? RGB565(60, 200, 90) : RGB565(200, 120, 50), 2);
  presentFrame();
}

static void drawHubPage() {
  if (!ensureFrame()) return;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(150, 70, 180));
  drawTextCentered(240, 52, "HUB", RGB565(205, 120, 230), 5);
  char buf[24];
  const uint16_t gr = RGB565(60, 210, 100), og = RGB565(220, 130, 50),
                 cw = RGB565(235, 235, 225), dk = RGB565(120, 120, 120);
  const bool anyFresh = httpFresh() || canFresh() || bleFresh() || tune123Fresh();
  drawDataRow(96,  "QUELLE", anyFresh ? g_lastSrc : "---", anyFresh ? gr : og);
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)g_httpRx);
  drawDataRow(130, "HTTP",   buf, httpFresh() ? gr : dk);
  snprintf(buf, sizeof(buf), "%lu/%lu", (unsigned long)g_canRx, (unsigned long)g_canIgnored);
  drawDataRow(164, "CAN",    buf, g_canReady ? (canFresh() ? gr : cw) : dk);
  drawDataRow(198, "BLE",    g_bleConn ? "OK" : (g_bleInited ? "SCAN" : "AUS"),
                             g_bleConn ? gr : (g_bleInited ? og : dk));
  drawDataRow(232, "123",    g_tune123Conn ? "OK" : (g_feature123 ? "AN" : "AUS"),
                             g_tune123Conn ? gr : (g_feature123 ? og : dk));
  drawDataRow(266, "HUBIP",  g_hubIp.c_str(), RGB565(150, 200, 150));
  drawDataRow(300, "IP",     g_ipStr, RGB565(150, 200, 150));
  drawTextCentered(240, 360, "TIP MENU", RGB565(180, 180, 170), 2);
  presentFrame();
}

static void drawSetupPage() {
  if (!ensureFrame()) return;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(185, 150, 45));
  drawTextCentered(240, 54, "SETUP", RGB565(230, 190, 70), 5);
  char buf[28];
  snprintf(buf, sizeof(buf), "%d %%", g_dialScalePct);
  drawDataRow(110, "UHR",   buf, RGB565(235, 235, 225));
  snprintf(buf, sizeof(buf), "%d %%", g_brightnessPct);
  drawDataRow(146, "HELL",  buf, RGB565(235, 235, 225));
  snprintf(buf, sizeof(buf), "%d DEG", g_rotationDeg);
  drawDataRow(182, "ROT",   buf, RGB565(235, 235, 225));
  if (g_featureWifi && strlen(currentWifiSsid()) > 0) {
    snprintf(buf, sizeof(buf), "%s", WPROF_LABELS[g_wifiProfile]);
    drawDataRow(218, "WIFI", buf,
                WiFi.status() == WL_CONNECTED ? RGB565(60, 210, 100) : RGB565(220, 130, 50));
  } else {
    drawDataRow(218, "WIFI", "AUS", RGB565(220, 130, 50));
  }
  drawDataRow(254, "BLE",    g_featureBle ? (g_bleConn ? "OK" : "AN") : "AUS",
              g_featureBle && g_bleConn ? RGB565(60, 210, 100) : RGB565(220, 130, 50));
  drawDataRow(290, "BUZZER", g_featureBuzzer ? "AN" : "AUS",
              g_featureBuzzer ? RGB565(60, 210, 100) : RGB565(150, 150, 150));
  if (g_imuPresent) {
    qmi8658Read();
    snprintf(buf, sizeof(buf), "%+.1f DEG", g_imuPitch - g_imuOffPitch);
    drawDataRow(326, "IMU 0", buf, RGB565(235, 235, 225));   // TIP = aktuelle Lage nullen
  } else {
    drawDataRow(326, "IMU 0", "---", RGB565(150, 150, 150));
  }
  drawTextCentered(240, 372, "TIP MENU", RGB565(180, 180, 170), 2);
  presentFrame();
}

// IMU-Seite: Seitenansicht/Neigungsmesser mit Steigung (Pitch) + Roll.
static void drawImuPage() {
  if (!ensureFrame()) return;
  const GaugeTheme& t = gTheme();
  const int cx = 240, cy = 240;
  const uint16_t cream = t.tickMaj;                   // Hauptfarbe (creme/weiss)
  const uint16_t dim   = t.tickMin;                   // gedimmt
  const uint16_t mark  = t.needle;                    // Fahrzeug/Roll-Zeiger
  const uint16_t lower = t.ground;                    // Boden (untere Haelfte)

  fillFrame(t.face);
  fillCircleFast(cx, cy, 216, t.face);                // Zifferblatt
  for (int y = cy; y <= cy + 204; y++)                // untere Haelfte abgesetzt
    for (int x = cx - 204; x <= cx + 204; x++) {
      int dx = x - cx, dy = y - cy;
      if (dx * dx + dy * dy <= 204 * 204) setPixel(x, y, lower);
    }
  if (t.chrome) {
    drawCircleLine(cx, cy, 236, 7, t.bezel);          // heller Chrom-Ring
    drawCircleLine(cx, cy, 229, 2, t.bezelDk);
  } else {
    drawCircleLine(cx, cy, 223, 8, RGB565(84, 80, 72));
  }
  drawLineFast(cx - 190, cy, cx + 190, cy, dim, 3);   // Horizont-Referenz

  for (int a = -40; a <= 40; a += 10) {               // obere Neigungsskala
    float ar = (float)a * PI / 180.0f - PI / 2.0f;
    int ro = 190, ri = (a % 20 == 0) ? 164 : 176;
    drawLineFast(cx + (int)lroundf(cosf(ar) * ri), cy + (int)lroundf(sinf(ar) * ri),
                 cx + (int)lroundf(cosf(ar) * ro), cy + (int)lroundf(sinf(ar) * ro),
                 cream, (a % 20 == 0) ? 3 : 2);
    if (a != 0 && a % 20 == 0) {
      char lab[8]; snprintf(lab, sizeof(lab), "%d", abs(a));
      drawTextCentered(cx + (int)lroundf(cosf(ar) * 136.0f),
                       cy + (int)lroundf(sinf(ar) * 136.0f) - 7, lab, cream, 2);
    }
  }
  drawLineFast(cx - 17, cy - 176, cx, cy - 144, cream, 3);   // Marker oben
  drawLineFast(cx + 17, cy - 176, cx, cy - 144, cream, 3);
  drawLineFast(cx - 17, cy - 176, cx + 17, cy - 176, cream, 3);

  if (!g_imuPresent) {
    drawTextCentered(240, 240, "KEIN IMU", RGB565(200, 60, 60), 4);
    presentFrame();
    return;
  }

  const float pitch = constrain(g_imuPitch - g_imuOffPitch, -45.0f, 45.0f);
  const float roll  = constrain(g_imuRoll  - g_imuOffRoll,  -45.0f, 45.0f);
  const int pitchOffset = (int)lroundf(pitch * 1.8f);
  for (int p = -40; p <= 40; p += 10) {               // Pitch-Leiter (wandert)
    if (p == 0) continue;
    int y = cy - pitchOffset + p * 3;
    int half = (p % 20 == 0) ? 74 : 48;
    drawLineFast(cx - half, y, cx + half, y, dim, 2);
    if (p % 20 == 0) {
      char lab[8]; snprintf(lab, sizeof(lab), "%d", abs(p));
      drawTextCentered(cx - half - 30, y - 7, lab, dim, 1);
      drawTextCentered(cx + half + 30, y - 7, lab, dim, 1);
    }
  }

  float rr = -roll * PI / 180.0f;                      // Roll-Linie
  int rx = (int)lroundf(cosf(rr) * 66.0f), ry = (int)lroundf(sinf(rr) * 66.0f);
  drawLineFast(cx - rx, cy - ry, cx + rx, cy + ry, mark, 4);
  fillCircleFast(cx, cy, 5, mark);
  drawLineFast(cx, cy - 24, cx, cy + 92, mark, 3);    // Fahrzeug-Seitenkontur
  drawLineFast(cx - 45, cy + 92, cx + 45, cy + 92, mark, 3);
  drawLineFast(cx - 45, cy + 92, cx - 66, cy + 66, mark, 3);
  drawLineFast(cx + 45, cy + 92, cx + 66, cy + 66, mark, 3);

  const float gradePct = constrain(tanf(pitch * PI / 180.0f) * 100.0f, -99.0f, 99.0f);
  char gradeLine[18], degLine[18];
  snprintf(gradeLine, sizeof(gradeLine), "%+.0f%%", gradePct);
  snprintf(degLine,   sizeof(degLine),   "%+.1f DEG", pitch);
  drawTextCentered(240, 348, "STEIGUNG", dim, 2);
  drawTextCentered(240, 372, gradeLine, cream, 4);
  drawTextCentered(240, 412, degLine, dim, 2);

  static bool buzzerOn = false;                        // Shake-Buzzer beibehalten
  const bool wantBuzz = g_featureBuzzer && qmi8658ShakeDetected(1.5f);
  if (wantBuzz != buzzerOn) { buzzerOn = wantBuzz; hal_buzzer(buzzerOn); }

  presentFrame();
}

static void saveDialScale(int pct);   // fwd (Definition weiter unten)
static void saveRotation(int deg);    // fwd

// Touch-Button mit zentriertem Label (dunkler Text auf farbiger Flaeche).
static void drawAdjBtn(int x, int y, int w, int h, const char* lbl, uint16_t col) {
  fillRectFast(x, y, w, h, col);
  drawTextCentered(x + w / 2, y + h / 2 - 11, lbl, RGB565(20, 20, 20), 3);
}

// Justage-Seite: Zifferblatt-Groesse (+/-5%) und Rotation (+/-1, +/-5) per Touch.
static void drawAdjustPage() {
  if (!ensureFrame()) return;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(185, 150, 45));
  drawTextCentered(240, 44, "JUSTAGE", RGB565(230, 190, 70), 5);
  char buf[16];
  const uint16_t minus = RGB565(210, 120, 60), plus = RGB565(90, 195, 110);
  drawTextCentered(240, 92, "GROESSE", RGB565(150, 150, 150), 2);
  snprintf(buf, sizeof(buf), "%d %%", g_dialScalePct);
  drawTextCentered(240, 114, buf, RGB565(235, 235, 225), 4);
  drawAdjBtn(120, 150, 90, 46, "-", minus);
  drawAdjBtn(270, 150, 90, 46, "+", plus);
  drawTextCentered(240, 222, "ROTATION", RGB565(150, 150, 150), 2);
  snprintf(buf, sizeof(buf), "%d DEG", g_rotationDeg);
  drawTextCentered(240, 244, buf, RGB565(235, 235, 225), 4);
  drawAdjBtn(120, 280, 90, 40, "-1", minus);
  drawAdjBtn(270, 280, 90, 40, "+1", plus);
  drawAdjBtn(120, 328, 90, 40, "-5", minus);
  drawAdjBtn(270, 328, 90, 40, "+5", plus);
  drawTextCentered(240, 392, "TIP UNTEN = MENU", RGB565(180, 180, 170), 2);
  presentFrame();
}

static void handleAdjustTap(uint16_t x, uint16_t y) {
  const bool right = (x >= 240);
  if (y >= 150 && y < 196)        saveDialScale(g_dialScalePct + (right ? 5 : -5));   // GROESSE +/-5%
  else if (y >= 280 && y < 320)   saveRotation(g_rotationDeg + (right ? 1 : -1));     // ROT +/-1
  else if (y >= 328 && y < 368)   saveRotation(g_rotationDeg + (right ? 5 : -5));     // ROT +/-5
  else if (y >= 376)            { currentPage = 1; drawMenuOverview(); return; }      // unten -> Menue
  drawAdjustPage();
}

static void drawCurrentPage() {
  if      (currentPage == 0) drawVdoClock();
  else if (currentPage == 1) drawMenuOverview();
  else if (currentPage == 2) drawMotorPage();
  else if (currentPage == 3) drawLambdaPage();
  else if (currentPage == 4) drawHubPage();
  else if (currentPage == 5) drawSetupPage();
  else if (currentPage == 6) drawImuPage();
  else if (currentPage == 7) drawAdjustPage();
}

// -------- Preferences --------
static void loadSettings() {
  Preferences p;
  p.begin("clock", true);
  g_dialScalePct  = p.getInt("scale",     100);
  g_brightnessPct = p.getInt("bright",    100);
  g_rotationDeg   = p.getInt("rot_deg",   0);
  for (uint8_t i = 0; i < WPROF_COUNT; i++) {     // 3 WLAN-Profile laden
    char k[8];
    snprintf(k, sizeof(k), "wp%u_s", i); p.getString(k, g_wprof[i].ssid,  sizeof(g_wprof[i].ssid));
    snprintf(k, sizeof(k), "wp%u_p", i); p.getString(k, g_wprof[i].pass,  sizeof(g_wprof[i].pass));
    snprintf(k, sizeof(k), "wp%u_h", i); p.getString(k, g_wprof[i].hubip, sizeof(g_wprof[i].hubip));
  }
  if (!g_wprof[0].ssid[0]) {                       // Migration alte Einzel-STA -> Slot 0 (Heim)
    p.getString("sta_ssid", g_wprof[0].ssid, sizeof(g_wprof[0].ssid));
    p.getString("sta_pass", g_wprof[0].pass, sizeof(g_wprof[0].pass));
  }
  if (!g_wprof[0].hubip[0]) p.getString("hub_ip", g_wprof[0].hubip, sizeof(g_wprof[0].hubip));
  if (!g_wprof[1].ssid[0]) {                        // Default Hub-AP in Slot 1
    strncpy(g_wprof[1].ssid, "Spartan3-TestHub", sizeof(g_wprof[1].ssid) - 1);
    strncpy(g_wprof[1].pass, "lambda123",        sizeof(g_wprof[1].pass) - 1);
  }
  if (!g_wprof[1].hubip[0]) strncpy(g_wprof[1].hubip, "spartanhub.local", sizeof(g_wprof[1].hubip) - 1);  // Hub-AP: Gateway/mDNS (Subnetz egal)
  if (!g_wprof[2].ssid[0])  strncpy(g_wprof[2].ssid,  "Android-AP1",      sizeof(g_wprof[2].ssid)  - 1);  // S24: Hotspot-SSID
  if (!g_wprof[2].pass[0])  strncpy(g_wprof[2].pass,  S24_AP_PASS,        sizeof(g_wprof[2].pass)  - 1);  // S24: Passwort (Seed aus wifi_secret.h)
  if (!g_wprof[2].hubip[0]) strncpy(g_wprof[2].hubip, "spartanhub.local", sizeof(g_wprof[2].hubip) - 1);  // S24: Hub per mDNS
  g_wifiProfile   = p.getUChar("wifi_prof", 0);
  if (g_wifiProfile >= WPROF_COUNT) g_wifiProfile = 0;
  if (g_wprof[g_wifiProfile].hubip[0]) g_hubIp = g_wprof[g_wifiProfile].hubip;  // Hub-IP aus aktivem Profil
  g_featureWifi   = p.getBool("feat_wifi", strlen(currentWifiSsid()) > 0);
  g_featureBle    = p.getBool("feat_ble",  false);
  g_featureBuzzer = p.getBool("feat_buzzer", false);  // default OFF
  g_feature123    = p.getBool("feat_123", false);     // 123-Fallback default OFF
  g_imuOffPitch   = p.getFloat("imu_off_p", 0.0f);     // IMU-Nullung
  g_imuOffRoll    = p.getFloat("imu_off_r", 0.0f);
  g_wifiAuto      = p.getBool("wifi_auto", true);      // WLAN-Auto-Fallback default AN
  g_canListenOnly = p.getBool("can_listen", true);     // CAN: listen-only default (NORMAL = ACK)
  g_motorStyle    = p.getUChar("mstyle", 2);          // 0=digital,1=vdo,2=123tune+
  if (g_motorStyle > 2) g_motorStyle = 2;
  p.end();
  if (g_dialScalePct  < 30)  g_dialScalePct  = 30;
  if (g_dialScalePct  > 150) g_dialScalePct  = 150;
  if (g_brightnessPct < 5)   g_brightnessPct = 5;
  if (g_brightnessPct > 100) g_brightnessPct = 100;
  g_rotationDeg %= 360;
  if (g_rotationDeg < 0) g_rotationDeg += 360;
  updateRotationCache();
}

static void saveDialScale(int pct) {
  if (pct < 30)  pct = 30;
  if (pct > 150) pct = 150;
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

static void saveMotorStyle(int style) {
  if (style < 0) style = 0;
  if (style > 2) style = 2;
  g_motorStyle = (uint8_t)style;
  Preferences p;
  p.begin("clock", false);
  p.putUChar("mstyle", g_motorStyle);
  p.end();
}

// IMU nullen: aktuelle Lage als Referenz speichern (Einbaulage ausgleichen).
static void saveImuNull() {
  if (!g_imuPresent) { Serial.println("IMU: NULL - kein IMU"); return; }
  qmi8658Read();
  g_imuOffPitch = g_imuPitch;
  g_imuOffRoll  = g_imuRoll;
  Preferences p;
  p.begin("clock", false);
  p.putFloat("imu_off_p", g_imuOffPitch);
  p.putFloat("imu_off_r", g_imuOffRoll);
  p.end();
  Serial.printf("IMU: NULL gesetzt P=%.1f R=%.1f\n", g_imuOffPitch, g_imuOffRoll);
}

static void saveFeatures(bool wifi, bool ble, bool buzzer) {
  const bool wasBle = g_featureBle;
  g_featureWifi   = wifi;
  g_featureBle    = ble;
  g_featureBuzzer = buzzer;
  Preferences p;
  p.begin("clock", false);
  p.putBool("feat_wifi",   g_featureWifi);
  p.putBool("feat_ble",    g_featureBle);
  p.putBool("feat_buzzer", g_featureBuzzer);
  p.end();
  if (!g_featureBuzzer) hal_buzzer(false);  // sofort aus wenn deaktiviert
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
    bleEnsureInit();
    bleNextScanAt = millis() + 5000;
    Serial.println("BLE: AN, Scan startet...");
  } else if (!g_bleConn && !bleDoConnect && bleNextScanAt == 0) {
    bleNextScanAt = millis() + 5000;
  }
}

// 123-Direkt-Fallback an/aus. AUS gibt das BLE-Radio frei (wieder fluessig).
static void saveFeature123(bool on) {
  g_feature123 = on;
  Preferences p;
  p.begin("clock", false);
  p.putBool("feat_123", on);
  p.end();
  if (on) {
    Serial.println("123: Fallback AN (BLE startet nur bei Hub-Ausfall)");
    return;
  }
  if (tune123Client && tune123Client->isConnected()) tune123Client->disconnect();
  g_tune123Conn = false;
  g_tune123DoConnect = false;
  g_tune123NextScanAt = 0;
  if (!g_featureBle && g_bleInited) {     // Radio nur freigeben, wenn Hub-BLE es nicht braucht
    NimBLEDevice::deinit(true);
    g_bleInited   = false;
    tune123Client = nullptr;
    tune123Rx     = nullptr;
    Serial.println("123: AUS - BLE deinitialisiert (fluessig)");
  } else {
    Serial.println("123: Fallback AUS");
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
    "body{font-family:sans-serif;background:#111;color:#eee;margin:0;padding:16px;text-align:center}"
    "h1{color:#e0c040;font-weight:600;margin:6px}.card{background:#1c1c1c;border-radius:12px;padding:18px;margin:14px auto;max-width:420px}"
    ".big{font-size:2.2em;letter-spacing:2px}input[type=range]{width:90%}"
    "button{background:#e0c040;border:0;border-radius:8px;padding:12px 20px;font-size:1em;margin:6px;cursor:pointer}"
    "a{color:#8cf}.val{font-size:1.6em;color:#e0c040}"
    ".tabs{display:flex;flex-wrap:wrap;justify-content:center;gap:4px;max-width:440px;margin:10px auto}"
    ".tabbtn{background:#333;color:#eee;padding:10px 16px}"
    ".tab{display:none}.tab.on{display:block}</style></head><body>");
  html += F("<h1>VDO Quartz-Zeit</h1>");
  html += "<div class='card'><div class='big'>" + String(timeStr) + "</div>";
  html += "<div>IP " + String(g_ipStr) + "</div></div>";

  html += F("<div class='tabs'>"
    "<button class='tabbtn' id='b-live' onclick=\"sh('live',this)\">Live</button>"
    "<button class='tabbtn' onclick=\"sh('wlan',this)\">WLAN</button>"
    "<button class='tabbtn' onclick=\"sh('anz',this)\">Anzeige</button>"
    "<button class='tabbtn' onclick=\"sh('imu',this)\">IMU</button>"
    "<button class='tabbtn' onclick=\"sh('sys',this)\">System</button></div>");

  // ===== Tab: Live =====
  html += F("<div class='tab on' id='t-live'>");
  html += F("<div class='card'><h3>Spartan-Hub Live</h3>");
  bool anyFresh = bleFresh() || canFresh() || httpFresh();
  html += "<div>Quelle: <b>" + String(anyFresh ? g_lastSrc : "---") + "</b> &middot; " +
          String(anyFresh ? "LIVE" : "keine Daten") + "</div>";
  html += "<div>Lambda: " + String(g_lambdaValid ? String(g_lambda, 2) : String("---")) +
          " &nbsp; RPM: " + String((int)g_rpm) + " &nbsp; ADV: " + String(g_adv, 1) + "</div>";
  html += "<div>MAP: " + String((int)g_map) + " &nbsp; TEMP: " + String((int)g_g123Temp) +
          " &nbsp; VOLT: " + String(g_g123Volt, 1) + " &nbsp; AMP: " + String(g_g123Coil, 1) + "</div>";
  html += "<div style='color:#888'>HTTP rx " + String((unsigned long)g_httpRx) +
          " &middot; CAN rx " + String((unsigned long)g_canRx) +
          " &middot; BLE rx " + String((unsigned long)g_bleRxCnt) + "</div></div>";
  html += F("<div class='card'><h3>Display-Seite</h3>"
    "<a href='/page?p=0'><button>Uhr</button></a>"
    "<a href='/page?p=1'><button>Menu</button></a>"
    "<a href='/page?p=2'><button>Motor</button></a>"
    "<a href='/page?p=3'><button>Lambda</button></a>"
    "<a href='/page?p=4'><button>Hub</button></a>"
    "<a href='/page?p=6'><button>IMU</button></a>"
    "<a href='/page?p=5'><button>Setup</button></a></div>");
  html += F("</div>");

  // ===== Tab: WLAN =====
  html += F("<div class='tab' id='t-wlan'>");
  html += F("<div class='card'><h3>WLAN-Profile (Vorauswahl)</h3>");
  html += "<div>Aktiv: <b>" + String(currentWifiSsid()) + "</b> &middot; " +
          String(WiFi.status() == WL_CONNECTED ? "verbunden" : "nicht verbunden") +
          " &middot; Hub " + String(g_hubIp) + "</div>";
  for (uint8_t i = 0; i < WPROF_COUNT; i++) {
    html += F("<div style='border-top:1px solid #333;margin-top:10px;padding-top:8px;text-align:left'>");
    html += "<b style='color:#e0c040'>" + String(WPROF_LABELS[i]) +
            (i == g_wifiProfile ? " &mdash; aktiv" : "") + "</b>";
    html += "<form action='/wifi' method='post'><input type='hidden' name='slot' value='" + String(i) + "'>";
    html += "<input name='ssid' placeholder='SSID' value='" + String(g_wprof[i].ssid) +
            "' autocomplete='off' style='width:88%;padding:8px;margin:3px;border:0;border-radius:6px'><br>";
    html += F("<input name='pass' type='password' placeholder='Passwort (leer = unver&auml;ndert)' "
              "autocomplete='off' style='width:88%;padding:8px;margin:3px;border:0;border-radius:6px'><br>");
    html += "<input name='hubip' placeholder='Hub-IP' value='" + String(g_wprof[i].hubip) +
            "' style='width:50%;padding:8px;margin:3px;border:0;border-radius:6px'>";
    html += F("<button type='submit'>Speichern+Verbinden</button></form>");
    html += "<a href='/wifi?sel=" + String(i) + "'><button" +
            String(i == g_wifiProfile ? " style='background:#6c6'" : "") + ">" +
            String(WPROF_LABELS[i]) + " aktivieren</button></a>";
    html += F("</div>");
  }
  html += F("<div style='color:#888;margin-top:8px'>Hub-IP darf ein Hostname sein "
            "(z.B. <b>spartanhub.local</b>) &ndash; findet den Hub per mDNS in jedem Subnetz "
            "(ideal f&uuml;r den S24-Hotspot mit wechselnden IP-Bereichen).</div>");
  html += F("</div></div>");

  // ===== Tab: Anzeige =====
  html += F("<div class='tab' id='t-anz'>");
  html += F("<div class='card'><h3>Motor-Anzeige Stil</h3>");
  for (uint8_t i = 0; i < 3; i++) {
    html += "<a href='/set?style=" + String(i) + "'><button" +
            String(i == g_motorStyle ? " style='background:#6c6'" : "") + ">" +
            String(MOTOR_STYLE_NAMES[i]) + "</button></a>";
  }
  html += F("</div>");
  html += F("<div class='card'><h3>Zifferblatt-Gr&ouml;&szlig;e</h3>"
    "<form action='/set' method='get'>"
    "<div class='val'><span id='v'>");
  html += String(g_dialScalePct);
  html += F("</span>%</div>"
    "<input type='range' name='scale' min='30' max='150' step='1' value='");
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
  html += F("</div>");

  // ===== Tab: IMU =====
  html += F("<div class='tab' id='t-imu'>");
  html += F("<div class='card'><h3>IMU Steigung</h3>");
  if (g_imuPresent) {
    html += "<div class='val'>" + String(g_imuPitch - g_imuOffPitch, 1) + "&deg;</div>";
    html += F("<a href='/set?imunull=1'><button>IMU 0 setzen (Einbaulage)</button></a>"
              "<div style='color:#888'>Display waagerecht/eingebaut, dann nullen.</div>");
  } else {
    html += F("<div>kein IMU</div>");
  }
  html += F("</div></div>");

  // ===== Tab: System =====
  html += F("<div class='tab' id='t-sys'>");
  html += F("<div class='card'><h3>Funktionen</h3><form action='/features' method='get'>");
  html += "<p><label><input type='checkbox' name='wifi' value='1' ";
  html += g_featureWifi ? "checked" : "";
  html += F("> WLAN/Web aktiv</label></p><p><label>"
    "<input type='checkbox' name='ble' value='1' ");
  html += g_featureBle ? "checked" : "";
  html += F(" onchange='this.form.submit()'> BLE-Hub Daten aktiv</label></p><p><label>"
    "<input type='checkbox' name='buzzer' value='1' ");
  html += g_featureBuzzer ? "checked" : "";
  html += F(" onchange='this.form.submit()'> Buzzer (Shake-Alarm) aktiv</label></p><p><label>"
    "<input type='checkbox' name='f123' value='1' ");
  html += g_feature123 ? "checked" : "";
  html += F(" onchange='this.form.submit()'> 123TUNE+ direkt (Fallback bei Hub-Ausfall)</label></p><p><label>"
    "<input type='checkbox' name='wauto' value='1' ");
  html += g_wifiAuto ? "checked" : "";
  html += F(" onchange='this.form.submit()'> WLAN-Auto (S24 &gt; Heim &gt; Hub-AP)</label></p>"
    "<div style='color:#888'>WLAN-Auto verbindet automatisch das verf&uuml;gbare Netz nach Priorit&auml;t.</div>"
    "<button type='submit'>Speichern</button></form></div>");
  html += F("<div class='card'><h3>Firmware-Update (OTA)</h3>"
    "<form method='POST' action='/update' enctype='multipart/form-data'>"
    "<input type='file' name='firmware' accept='.bin' style='width:88%;margin:6px'><br>"
    "<button type='submit'>Firmware flashen</button></form>"
    "<div style='color:#888'>.bin aus .pio/build/waveshare_s3_28c/firmware.bin</div></div>");
  html += F("</div>");

  html += F("<p style='color:#666'>VW T2b Cockpit &middot; ESP32-S3 2.8\"</p>"
    "<script>function sh(t,b){var x=document.querySelectorAll('.tab');"
    "for(var i=0;i<x.length;i++)x[i].className='tab';"
    "document.getElementById('t-'+t).className='tab on';"
    "var y=document.querySelectorAll('.tabbtn');"
    "for(var i=0;i<y.length;i++)y[i].style.background='';b.style.background='#6c6';}"
    "window.addEventListener('load',function(){var b=document.getElementById('b-live');if(b)b.style.background='#6c6';});"
    "</script></body></html>");
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
  if (webServer.hasArg("hubip")) {
    g_hubIp = webServer.arg("hubip");
    g_hubIp.trim();
    if (g_wifiProfile < WPROF_COUNT)               // Hub-IP ans aktive Profil binden
      snprintf(g_wprof[g_wifiProfile].hubip, sizeof(g_wprof[g_wifiProfile].hubip), "%s", g_hubIp.c_str());
    Preferences p;
    p.begin("clock", false);
    p.putString("hub_ip", g_hubIp);
    char k[8]; snprintf(k, sizeof(k), "wp%u_h", g_wifiProfile); p.putString(k, g_hubIp);
    p.end();
    Serial.printf("Web: Hub-IP = %s (Profil %u)\n", g_hubIp.c_str(), g_wifiProfile);
  }
  if (webServer.hasArg("style")) {
    saveMotorStyle(webServer.arg("style").toInt());
    g_redrawPage = true;
    Serial.printf("Web: Motor-Stil = %u (%s)\n", g_motorStyle, MOTOR_STYLE_NAMES[g_motorStyle]);
  }
  if (webServer.hasArg("imunull")) {
    saveImuNull();
    g_redrawPage = true;
  }
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

static void handleWebFeatures() {
  const bool wifi   = webServer.hasArg("wifi");
  const bool ble    = webServer.hasArg("ble");
  const bool buzzer = webServer.hasArg("buzzer");
  const bool f123   = webServer.hasArg("f123");
  const bool wauto  = webServer.hasArg("wauto");
  saveFeatures(wifi, ble, buzzer);
  if (f123 != g_feature123) saveFeature123(f123);
  if (wauto != g_wifiAuto) {
    g_wifiAuto = wauto;
    Preferences p; p.begin("clock", false); p.putBool("wifi_auto", g_wifiAuto); p.end();
  }
  Serial.printf("Web: Funktionen wifi=%s ble=%s buzzer=%s 123=%s\n",
                g_featureWifi ? "on" : "off", g_featureBle ? "on" : "off",
                g_featureBuzzer ? "on" : "off", g_feature123 ? "on" : "off");
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

static void handleWebPage() {
  if (webServer.hasArg("p")) {
    int page = webServer.arg("p").toInt();
    if (page < 0) page = 0;
    if (page > 7) page = 7;
    currentPage  = static_cast<uint8_t>(page);
    g_redrawPage = true;
    Serial.printf("Web: page=%u\n", currentPage);
  }
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

static void reconnectWifiProfile();                         // fwd
static void selectWprof(uint8_t idx);                       // fwd
static void saveWprof(uint8_t, const char*, const char*, const char*);  // fwd

// WLAN per Web: Profil auswaehlen (/wifi?sel=N) oder Profil bearbeiten
// (/wifi POST slot,ssid,pass,hubip -> speichert + verbindet).
static void handleWebWifi() {
  if (webServer.hasArg("sel")) {
    selectWprof((uint8_t)webServer.arg("sel").toInt());
    Serial.printf("Web: WLAN-Profil aktiv -> %u (%s)\n", g_wifiProfile, currentWifiSsid());
  } else if (webServer.hasArg("slot")) {
    uint8_t idx = (uint8_t)webServer.arg("slot").toInt();
    if (idx < WPROF_COUNT) {
      // leeres Passwort = bestehendes behalten (wird im Formular nie zurueckgegeben)
      const char* pw = webServer.arg("pass").length() ? webServer.arg("pass").c_str()
                                                       : g_wprof[idx].pass;
      saveWprof(idx, webServer.arg("ssid").c_str(), pw,
                webServer.hasArg("hubip") ? webServer.arg("hubip").c_str() : g_wprof[idx].hubip);
      selectWprof(idx);
      Serial.printf("Web: WLAN-Profil %u gesetzt -> '%s'\n", idx, g_wprof[idx].ssid);
    }
  }
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

// OTA: Multipart-Upload der .bin, schreibt in die freie App-Partition.
// Panel wird pausiert (Flash-Cache-Disable + RGB-ISR vertragen sich nicht).
static void handleOtaUpload() {
  HTTPUpload& up = webServer.upload();
  if (up.status == UPLOAD_FILE_START) {
    g_otaBusy = true; g_otaRxBytes = 0;
    hal_pause_for_ota(true);
    Serial.printf("OTA: Start %s (heap %u, frei %u)\n",
                  up.filename.c_str(), ESP.getFreeHeap(), ESP.getFreeSketchSpace());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial); g_otaBusy = false; hal_pause_for_ota(false);
    }
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (g_otaBusy) {
      if (Update.write(up.buf, up.currentSize) != up.currentSize) Update.printError(Serial);
      else g_otaRxBytes += up.currentSize;
      yield();
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (g_otaBusy) {
      if (Update.end(true)) Serial.printf("OTA: Erfolg, %u Bytes\n", (unsigned)g_otaRxBytes);
      else                  Update.printError(Serial);
    }
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    Update.abort(); g_otaBusy = false; hal_pause_for_ota(false);
    Serial.println("OTA: abgebrochen");
  }
}

static void handleOtaDone() {
  const bool ok = !Update.hasError() && g_otaRxBytes > 0;
  webServer.sendHeader("Connection", "close");
  webServer.send(200, "text/html",
    ok ? F("<meta charset='utf-8'><body style='font-family:sans-serif;background:#111;color:#eee;text-align:center;padding:40px'>"
           "<h1>OTA OK &ndash; Neustart&hellip;</h1></body>")
       : F("<meta charset='utf-8'><body style='font-family:sans-serif;background:#111;color:#eee;text-align:center;padding:40px'>"
           "<h1>OTA FEHLER</h1></body>"));
  delay(400);
  if (ok) { Serial.println("OTA: Neustart"); ESP.restart(); }
  else    { g_otaBusy = false; hal_pause_for_ota(false); }
}

static void startWebServer() {
  webServer.on("/",        handleWebRoot);
  webServer.on("/set",     handleWebSet);
  webServer.on("/features",handleWebFeatures);
  webServer.on("/page",    handleWebPage);
  webServer.on("/wifi",    handleWebWifi);
  webServer.on("/update",  HTTP_POST, handleOtaDone, handleOtaUpload);
  webServer.begin();
  Serial.println("WebGUI: gestartet auf Port 80");
}

// Fallback-Setup-AP: nur AN wenn keine STA-Verbindung besteht, damit das
// Display im verbundenen Normalbetrieb stabil bleibt (kein AP-Dauerbeacon).
static void manageWifiAp() {
  if (!g_featureWifi) return;
  if (g_wifiAuto) return;            // bei Auto-Fallback besitzt wifiAutoTick das WLAN (kein stoerender AP)
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

  // Zonen passend zu drawSetupPage (Zeilen-Mitte = Zonenstart + 18, Hoehe 36)
  if (y >= 345) {                       // unten -> zurueck ins Menue
    currentPage = 1;
    drawMenuOverview();
    Serial.println("setup tap: menu");
    return;
  }

  if (y >= 92 && y < 128) {             // UHR (Zeile 110) -> Justage-Seite (Groesse/Rotation fein)
    currentPage = 7;
    drawAdjustPage();
    Serial.println("setup tap: -> Justage");
  } else if (y >= 128 && y < 164) {     // HELL (Zeile 146)
    int next = (g_brightnessPct < 63) ? 75 : (g_brightnessPct < 88 ? 100 : 50);
    saveBrightness(next);
    drawSetupPage();
    Serial.printf("setup tap: brightness=%d%%\n", g_brightnessPct);
  } else if (y >= 164 && y < 200) {     // ROT (Zeile 182) -> Justage-Seite
    currentPage = 7;
    drawAdjustPage();
    Serial.println("setup tap: -> Justage");
  } else if (y >= 200 && y < 236) {     // WIFI (Zeile 218)
    cycleWifiProfile();
    drawSetupPage();
    Serial.printf("setup tap: wifi profile=%u ssid=%s\n", g_wifiProfile, currentWifiSsid());
  } else if (y >= 236 && y < 272) {     // BLE (Zeile 254)
    saveFeatures(g_featureWifi, !g_featureBle, g_featureBuzzer);
    drawSetupPage();
    Serial.printf("setup tap: ble=%s\n", g_featureBle ? "on" : "off");
  } else if (y >= 272 && y < 308) {     // BUZZER (Zeile 290)
    saveFeatures(g_featureWifi, g_featureBle, !g_featureBuzzer);
    drawSetupPage();
    Serial.printf("setup tap: buzzer=%s\n", g_featureBuzzer ? "on" : "off");
  } else if (y >= 308 && y < 345) {     // IMU 0 (Zeile 326) -> Einbaulage nullen
    saveImuNull();
    drawSetupPage();
    Serial.println("setup tap: IMU NULL");
  } else {
    drawSetupPage();
    Serial.println("setup tap: no action");
  }
}

// Ein WLAN-Profil (Slot) speichern. Persistent im NVS.
static void saveWprof(uint8_t idx, const char* ssid, const char* pass, const char* hubip) {
  if (idx >= WPROF_COUNT) return;
  snprintf(g_wprof[idx].ssid,  sizeof(g_wprof[idx].ssid),  "%s", ssid  ? ssid  : "");
  snprintf(g_wprof[idx].pass,  sizeof(g_wprof[idx].pass),  "%s", pass  ? pass  : "");
  snprintf(g_wprof[idx].hubip, sizeof(g_wprof[idx].hubip), "%s", hubip ? hubip : "");
  Preferences p;
  p.begin("clock", false);
  char k[8];
  snprintf(k, sizeof(k), "wp%u_s", idx); p.putString(k, g_wprof[idx].ssid);
  snprintf(k, sizeof(k), "wp%u_p", idx); p.putString(k, g_wprof[idx].pass);
  snprintf(k, sizeof(k), "wp%u_h", idx); p.putString(k, g_wprof[idx].hubip);
  p.end();
}

static void reconnectWifiProfile() {
  if (strlen(currentWifiSsid()) == 0) return;
  g_featureWifi = true;
  strcpy(g_ipStr, "...");
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(currentWifiSsid(), currentWifiPassword());
  Serial.printf("WiFi: Profil %u (%s) -> '%s'\n", g_wifiProfile, WPROF_LABELS[g_wifiProfile], currentWifiSsid());
}

// Profil aktiv schalten: Index + zugehoerige Hub-IP setzen, dann verbinden.
static void selectWprof(uint8_t idx) {
  if (idx >= WPROF_COUNT) idx = 0;
  g_wifiProfile = idx;
  g_featureWifi = true;
  Preferences p;
  p.begin("clock", false);
  p.putUChar("wifi_prof", g_wifiProfile);
  p.putBool("feat_wifi", true);
  if (g_wprof[idx].hubip[0]) { g_hubIp = g_wprof[idx].hubip; p.putString("hub_ip", g_hubIp); }
  p.end();
  reconnectWifiProfile();
}

static void cycleWifiProfile() {                 // zum naechsten belegten Profil
  for (uint8_t step = 1; step <= WPROF_COUNT; step++) {
    uint8_t i = (g_wifiProfile + step) % WPROF_COUNT;
    if (g_wprof[i].ssid[0]) { selectWprof(i); return; }
  }
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

  // NimBLE NICHT pauschal starten - Dauer-BLE neben WiFi/RGB-Panel ruckelt.
  // Nur initialisieren, wenn der Hub-BLE-Client aktiv ist. Der 123-Fallback
  // initialisiert BLE bei Bedarf selbst (g_feature123).
  if (g_featureBle) {
    bleEnsureInit();
    bleNextScanAt = millis() + 20000;
    Serial.println("BLE: Hub-Client aktiv");
  }

  Serial.println("Display: HAL init...");
  hal_init();
  gt911Init();

  // QMI8658 IMU erkennen + initialisieren (I2C laeuft via hal_init/Wire)
  g_imuPresent = qmi8658Detect();
  if (g_imuPresent) {
    qmi8658Init();
    Serial.println("IMU: QMI8658 found at 0x6B, initialized");
  } else {
    Serial.println("IMU: QMI8658 NOT found");
  }

  // CAN-Cockpit-Empfaenger (TWAI, listen-only) starten: hoert 0x510 vom Test-Hub
  setupCockpitCan();

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
    if (touchFrame) {
      // WICHTIG: nur bei echtem Touch-Frame aktualisieren! Sonst haelt der
      // touchHeld-Zeitfenster-Mechanismus sich selbst am Leben (touchHeld ->
      // touchNow -> touchLastSeenMs=nowMs -> Fenster nie abgelaufen) und der
      // Release-Zweig (Tap-Erkennung) feuert NIE.
      touchLastSeenMs = nowMs;
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
    if (!touchLongHandled && nowMs - touchStartMs >= 600) {
      if (currentPage == 5) {
        touchLongHandled = true;
        lastTouch = nowMs;
        handleSetupLongPress(touchLastY, nowMs - touchStartMs);
      } else if (currentPage == 2) {
        // MOTOR: langer Druck in die Mitte -> Anzeige-Stil wechseln (DIGITAL/VDO/123TUNE+)
        const int dx = (int)touchLastX - 240, dy = (int)touchLastY - 240;
        if (dx * dx + dy * dy <= 95 * 95) {
          touchLongHandled = true;
          lastTouch = nowMs;
          saveMotorStyle((g_motorStyle + 1) % 3);
          drawMotorPage();
          Serial.printf("motor long-press -> Stil %u (%s)\n", g_motorStyle, MOTOR_STYLE_NAMES[g_motorStyle]);
        }
      }
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
        else if (tapY >= z0 +4*zh && tapY < z0 + 5*zh) { currentPage = 6; drawImuPage();    Serial.println("page: imu"); }
        else if (tapY >= z0 +5*zh && tapY < z0 + 6*zh) { currentPage = 5; drawSetupPage();  Serial.println("page: setup"); }
        else { currentPage = 0; drawVdoClock(); Serial.println("page: clock fallback"); }
      } else if (currentPage == 5) {
        // SETUP: kurzer Tap auf eine Zeile aendert die jeweilige Einstellung
        // (Zifferblatt/Helligkeit/Rotation/WLAN/BLE/Buzzer). Tap unten = Menue.
        handleSetupLongPress(tapY, durMs);
      } else if (currentPage == 7) {
        handleAdjustTap(tapX, tapY);    // Justage: Groesse/Rotation +/-
      } else {
        // Data pages (Motor/Lambda/Hub/IMU): Tap -> naechste, dann zurueck zur Uhr.
        if      (currentPage == 4) currentPage = 6;  // Hub -> IMU
        else if (currentPage == 6) currentPage = 0;  // IMU -> Uhr
        else currentPage++;                          // Motor->Lambda->Hub
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
        if      (cmd == "ble:on")  { saveFeatures(g_featureWifi, true, g_featureBuzzer); }
        else if (cmd == "ble:off") { saveFeatures(g_featureWifi, false, g_featureBuzzer); }
        else if (cmd == "buzzer:on")  { saveFeatures(g_featureWifi, g_featureBle, true); }
        else if (cmd == "buzzer:off") { saveFeatures(g_featureWifi, g_featureBle, false); }
        else if (cmd == "123:on")     { saveFeature123(true); }
        else if (cmd == "123:off")    { saveFeature123(false); }
        else if (cmd == "imu:null")   { saveImuNull(); if (currentPage == 5 || currentPage == 6) drawCurrentPage(); }
        else if (cmd == "ap:on") {     // Setup-AP sicher hochfahren: WLAN an, Auto AUS (kein STA-Connect bei leerem Profil)
          g_featureWifi = true; g_wifiAuto = false;
          Preferences pa; pa.begin("clock", false); pa.putBool("feat_wifi", true); pa.putBool("wifi_auto", false); pa.end();
          Serial.println("Setup-AP an (Auto aus) - VDO-Clock-Setup / vdoclock -> http://192.168.4.1");
        }
        else if (cmd == "wauto:on" || cmd == "wauto:off") {
          g_wifiAuto = (cmd == "wauto:on");
          Preferences pw; pw.begin("clock", false); pw.putBool("wifi_auto", g_wifiAuto); pw.end();
          Serial.printf("WLAN-Auto = %s\n", g_wifiAuto ? "AN (S24>Heim>Hub)" : "AUS");
        }
        else if (cmd == "wifi:next") { cycleWifiProfile(); g_redrawPage = true; }
        else if (cmd == "wifi:off")  { saveFeatures(false, g_featureBle, g_featureBuzzer); g_redrawPage = true; }
        else if (cmd == "rot:+") { saveRotation(g_rotationDeg + 1); g_redrawPage = true; }
        else if (cmd == "rot:-") { saveRotation(g_rotationDeg - 1); g_redrawPage = true; }
        else if (cmd.startsWith("rot:")) { saveRotation(cmd.substring(4).toInt()); g_redrawPage = true; }
        else if (cmd == "clock")   { currentPage = 0; drawVdoClock(); }
        else if (cmd == "can:normal" || cmd == "can:listen") {
          g_canListenOnly = (cmd == "can:listen");
          Preferences pc; pc.begin("clock", false); pc.putBool("can_listen", g_canListenOnly); pc.end();
          setupCockpitCan();
          Serial.printf("CAN-Mode = %s\n", g_canListenOnly ? "listen-only" : "NORMAL (ACK)");
        }
        else if (cmd == "can:test"){ runCanTest(); }
        else if (cmd == "can:ping"){ runCanPing(); }
        else if (cmd == "can:rx")  { Serial.printf("CAN: ready=%d rx=%lu ignored=%lu age=%lums src=%s\n",
                                       g_canReady ? 1 : 0, (unsigned long)g_canRx, (unsigned long)g_canIgnored,
                                       g_canLastRxMs ? (unsigned long)(millis() - g_canLastRxMs) : 0UL, g_lastSrc); }
        else if (cmd == "motor")   { currentPage = 2; drawMotorPage(); }
        else if (cmd.startsWith("style:")) { saveMotorStyle(cmd.substring(6).toInt());
                                             Serial.printf("Motor-Stil = %u (%s)\n", g_motorStyle, MOTOR_STYLE_NAMES[g_motorStyle]);
                                             if (currentPage == 2) drawMotorPage(); }
        else { Serial.println("Commands: ble:on|off | 123:on|off | buzzer:on|off | wifi:next|off | wauto:on|off | rot:+|-|NN | clock | motor | style:0|1|2 | imu:null | can:test | can:rx | can:normal|listen"); }
      }
    } else if (serialLine.length() < 64) {
      serialLine += c;
    }
  }

  // WLAN-Auto-Fallback (S24 > Heim > Hub-AP) - verbindet zum verfuegbaren Netz
  wifiAutoTick();

  // Fallback-Setup-AP verwalten (nur AN wenn keine STA-Verbindung)
  manageWifiAp();

  // WiFi/NTP background tick; redraw clock on fresh sync
  if (wifiNtpTick() && currentPage == 0) drawVdoClock();

  // BLE client tick
  if (g_featureBle) bleTick();

  // CAN cockpit tick (0x510)
  cockpitCanTick();

  // HTTP-Poll vom Hub (/api/status)
  httpPollTick();

  // 123TUNE+ direkter Fallback (nur wenn HTTP/CAN tot + Hub-BLE aus)
  tune123ScanTick();
  tune123KeepaliveTick();

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
  // IMU page: read sensor + redraw at ~10 Hz
  if (currentPage == 6 && millis() - lastDraw >= 100) {
    lastDraw = millis();
    qmi8658Read();
    drawCurrentPage();
  }

  // RGB-DMA periodisch resyncen - auf ALLEN Seiten (WiFi/BLE koennen das Bild
  // schwarz/verschoben machen; bisher resynct nur die Uhr-Seite -> Setup/Daten
  // blieben bei WLAN-Aktivitaet schwarz).
  static uint32_t lastResync = 0;
  if (currentPage != 0 && millis() - lastResync >= 1000) {
    lastResync = millis();
    hal_restart();
  }

  // Periodische Status-Zeile fuer Testfahrt-Log (alle 5s)
  static uint32_t lastStat = 0;
  if (millis() - lastStat >= 5000) {
    lastStat = millis();
    Serial.printf("STAT up=%lus ip=%s wifi=%d prof=%s httpRx=%lu canRx=%lu src=%s age=%lums heap=%u\n",
                  (unsigned long)(millis() / 1000), g_ipStr, (int)WiFi.status(),
                  WPROF_LABELS[g_wifiProfile], (unsigned long)g_httpRx, (unsigned long)g_canRx,
                  g_lastSrc, g_httpLastRxMs ? (unsigned long)(millis() - g_httpLastRxMs) : 0UL,
                  (unsigned)ESP.getFreeHeap());
  }

  delay(10);
}
