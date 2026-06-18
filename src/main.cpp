// Waveshare ESP32-S3-Touch-LCD-2.8C - VDO Quartz-Zeit Clock
// Main app: clock, touch menu, WiFi/NTP, Spartan3-Hub data client, WebGUI.
// Data path priority: ESP-NOW (Bus) > WiFi hub HTTP > BLE hub > direct 123.
// Display hardware ownership lives in hal_waveshare_28c.h.
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Update.h>
#include <NimBLEDevice.h>
#include "esp_task_wdt.h"
#include "esp_ota_ops.h"
#include "esp_sntp.h"
#if __has_include("wifi_secret.h")
  #include "wifi_secret.h"
#else
  #define WIFI_SSID     ""
  #define WIFI_PASSWORD ""
#endif
#include "hal_waveshare_28c.h"
#include "qmi8658_imu.h"
#include "vdo_dial_480_rgb565.h"
#include <sys/time.h>
#include <time.h>
#include <cstring>
#include <cstdarg>

#ifndef ENABLE_ESP_NOW_CLIENT
#define ENABLE_ESP_NOW_CLIENT 0
#endif
#if ENABLE_ESP_NOW_CLIENT
#include <esp_now.h>
#include <esp_wifi.h>
#include "spartan_cockpit_frame.h"
#ifndef ESP_NOW_WIFI_CHANNEL
#define ESP_NOW_WIFI_CHANNEL 6
#endif
#endif

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

// Serial debug: DLOG=verbose app logs; BLE_LOG_SERIAL=throttled BLE connection logs (default on).
#ifndef FEATURE_VERBOSE_SERIAL
#define FEATURE_VERBOSE_SERIAL 0
#endif
#ifndef DEBUG_SERIAL
#define DEBUG_SERIAL 0
#endif

// -------- Spartan3-Hub / 123TUNE+ BLE-Client --------
#define SPARTAN_MAC     "30:30:f9:1d:d0:fd"
#define SPARTAN_NAME    "Spartan3-Hub"
#define SPARTAN_SVC     "7f510001-5a6b-4d2a-9f20-14a7f3e20000"
#define SPARTAN_STATUS  "7f510002-5a6b-4d2a-9f20-14a7f3e20000"
#define NUS_SVC         "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_RX          "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_TX          "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
#define DEFAULT_123_MAC "ef:a8:b2:de:e0:9e"
#define DEFAULT_HUB_MAC SPARTAN_MAC
#define BLE_SCAN_MAX    32
#define BLE_DISCOVERY_SCAN_MS 5000
#define BLE_OP_SCAN_123_MS    10000  // M5Dial: Scan 10s fuer 123/BM6
#define BLE_BG_SCAN_INTERVAL_MS 12000
#define BLE_RECONNECT_123_MS    1500
#define BLE_CONNECT_TIMEOUT_MS      3000
#define BLE_CONNECT_TIMEOUT_HUB_MS  8000
// M5 Dial 123: conn itvl=40ms lat=0 to=4000ms (NimBLE: 1.25ms / 10ms units)
#define BLE_123_CONN_ITVL       32     // 40ms
#define BLE_123_CONN_LATENCY    0
#define BLE_123_CONN_TIMEOUT    400    // 4000ms
#define BLE_123_KICK_DELAY_MS   900    // ~1s after NUS subscribe before $+CR
#define BLE_123_KICK_CR_MS      40     // gap between $ and CR
#define BLE_123_KICK_IDLE       0
#define BLE_123_KICK_WAIT       1      // waiting pre-kick delay
#define BLE_123_KICK_CR         2      // $ sent, waiting for CR
#define BLE_TICK_MS           300
#define BLE_SCAN_WEB_WAIT_MS  800
#define BLE_STATUS_LOG_MS     8000   // periodischer Offline-Status (Serial + API)
#define BLE_LIVE_LOG_MS       4000   // rpm/adv wenn LIVE-Daten ankommen
#define DEVICE_HOSTNAME       "vdo-touch-28"
// Throttled BLE logs on USB serial @115200. Set 0 in platformio.ini to silence serial.
#ifndef BLE_LOG_SERIAL
#define BLE_LOG_SERIAL 1
#endif

enum BleLogKind : uint8_t { BLE_LOG_EVENT = 0, BLE_LOG_STATUS = 1, BLE_LOG_LIVE = 2 };

static char  g_bleState[40]   = "init";
static char  g_bleLogLine[128] = "---";
static uint32_t g_bleStatusLogAt = 0;
static uint32_t g_bleLiveLogAt   = 0;

static void bleLogSetState(const char* state) {
  strncpy(g_bleState, state, sizeof(g_bleState) - 1);
  g_bleState[sizeof(g_bleState) - 1] = 0;
}

static void bleLog(BleLogKind kind, const char* fmt, ...) {
  char buf[128];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  strncpy(g_bleLogLine, buf, sizeof(g_bleLogLine) - 1);
  g_bleLogLine[sizeof(g_bleLogLine) - 1] = 0;

  const uint32_t now = millis();
  bool serial = false;
  switch (kind) {
    case BLE_LOG_EVENT:
      serial = true;
      break;
    case BLE_LOG_STATUS:
      if (now - g_bleStatusLogAt >= BLE_STATUS_LOG_MS) {
        g_bleStatusLogAt = now;
        serial = true;
      }
      break;
    case BLE_LOG_LIVE:
      if (now - g_bleLiveLogAt >= BLE_LIVE_LOG_MS) {
        g_bleLiveLogAt = now;
        serial = true;
      }
      break;
  }
#if DEBUG_SERIAL || BLE_LOG_SERIAL
  if (serial) Serial.println(buf);
#endif
}

#define BLELOG(fmt, ...)   bleLog(BLE_LOG_EVENT, fmt, ##__VA_ARGS__)
#define BLELOGN(msg)       bleLog(BLE_LOG_EVENT, "%s", msg)
#define BLELOG_STATUS(...) bleLog(BLE_LOG_STATUS, __VA_ARGS__)
#define BLELOG_LIVE(...)   bleLog(BLE_LOG_LIVE, __VA_ARGS__)

enum BleConnMode : uint8_t { BLE_MODE_DIRECT_123 = 0, BLE_MODE_SPARTAN_HUB = 1 };

struct BleScanEntry {
  char mac[18];
  char name[24];
  int8_t rssi;
  bool spartan;
  bool nus;
};

static float g_lambda = 0, g_rpm = 0, g_adv = 0, g_map = 0;
static float g_battVolt = 0, g_speedKmh = 0, g_auxBattVolt = 0;
static float g_g123Volt = 0, g_g123Temp = 0, g_g123Coil = 0;
static bool  g_lambdaValid = false, g_battValid = false, g_auxBattValid = false;
static bool  g_speedValid = false, g_g123Valid = false;
static bool  g_bleConn = false;
static uint32_t g_bleLastRx = 0, g_bleRxCnt = 0;
static String g_bleHubName = "---";

static NimBLEClient*      bleClient    = nullptr;
static NimBLERemoteCharacteristic* g_nusRx = nullptr;
static NimBLEAddress      bleTarget;
static volatile bool      bleDoConnect = false;
static uint32_t           bleNextScanAt = 0;
static uint32_t           g_ble123PingAt = 0;
static uint32_t           g_ble123KickAt = 0;
static uint8_t            g_ble123KickStage = BLE_123_KICK_IDLE;
static BleConnMode        g_bleConnMode = BLE_MODE_SPARTAN_HUB;
static char               g_bleTargetMac[18] = DEFAULT_123_MAC;  // 123 direkt
static uint8_t            g_ble123AddrType   = BLE_ADDR_RANDOM;
static char               g_bleHubMac[18]    = "";               // Hub (leer bis Scan/Connect)
static bool               g_ble123ScanNext   = false;            // 123: Scan nach Direct-Fail
static bool               g_bleHubScanNext   = true;             // Hub: Scan vor Direct (Hub advert. selten)
static uint8_t            g_ble123AddrFlip   = 0;               // Direct: RANDOM/PUBLIC wechseln
static BleScanEntry       g_bleScanList[BLE_SCAN_MAX];
static uint8_t            g_bleScanCount = 0;
static volatile bool      g_bleDiscoveryScan = false;
static bool               g_bleScanWifiSleepWasOn = false;
static bool               g_bleOpScanActive = false;
static volatile bool      g_bleSetupBusy = false;
static uint8_t            g_blePickIndex = 0;
static bool               g_bleStackReady = false;

// -------- Spartan Hub data path (WiFi HTTP default; BLE optional fallback) --------
enum DataPath : uint8_t { DATA_PATH_WIFI_HUB = 0, DATA_PATH_BLE = 1 };
#define HUB_WIFI_DEFAULT_HOST "192.168.0.87"
#define HUB_BUS_HOST          "192.168.4.1"
#define HUB_BUS_CLIENT_IP     "192.168.4.3"   // M5 Dial uses .2 on Spartan3-Setup
#define HUB_WIFI_POLL_MS      300
#define HUB_WIFI_TIMEOUT_MS   1200
#define DATA_FRESH_MS         3000
static constexpr uint8_t kSettingsVersion = 1;  // v1: Bus-Profil Standard beim Boot
static DataPath  g_dataPath       = DATA_PATH_WIFI_HUB;
static char      g_hubHost[48]    = HUB_WIFI_DEFAULT_HOST;
static bool      g_hubWifiOk      = false;
static uint32_t  g_hubLastOkMs    = 0;
static uint32_t  g_hubPollCnt     = 0;
static uint32_t  g_hubErrCnt      = 0;
static char      g_hubState[32]   = "init";
static char      g_hubSourceTag[16] = "---";
static uint32_t  g_liveLastRx     = 0;
static uint32_t  g_liveRxCnt      = 0;

#if ENABLE_ESP_NOW_CLIENT
static bool      g_espNowReady    = false;
static uint8_t   g_espNowActiveChannel = 0;
static uint32_t  g_espNowRx       = 0;
static uint16_t  g_espNowSeq      = 0;
static uint32_t  g_espNowLastRxMs = 0;
static volatile bool g_espNowPending = false;
static SpartanCockpitFrame g_espNowPendingFrame;
static portMUX_TYPE g_espNowMux = portMUX_INITIALIZER_UNLOCKED;
#endif

// ---- GT911 touch state ----
static uint8_t  gt911Addr = GT911_ADDR_PRIMARY;
static bool     gt911Found = false;
static uint8_t  g_touchI2cFailStreak = 0;
#define TOUCH_I2C_RECOVER_AFTER 40
static uint16_t g_lastTouchX = 0, g_lastTouchY = 0;
static uint32_t g_lastTouchMs = 0;
static uint8_t  g_lastTouchStatus = 0;

// ---- App state ----
static uint8_t   currentPage    = 0;
static bool      touchSeen      = false;
static char      g_ipStr[20]    = "---";
static const int DIAL_SCALE_DEFAULT = 115;  // sweet spot: fills display, chrome cropped
static const int DIAL_SCALE_MIN     = 30;
static const int DIAL_SCALE_MAX     = 120;
static const int DIAL_CENTER        = 240;
// Rotation pivot must match setPixel/readTouch (not DIAL_CENTER) so dial cache and hands align.
static constexpr float ROT_PIVOT    = 239.5f;
static const int DIAL_CENTER_OFF_X_DEFAULT = 4;   // bitmap optical center @115% (hands stay at 240,240)
static const int DIAL_CENTER_OFF_Y_DEFAULT = -9;
static const uint16_t DIAL_FACE_BG  = RGB565(46, 43, 40);
static int       g_dialScalePct = DIAL_SCALE_DEFAULT;
static int       g_dialCenterOffsetX = DIAL_CENTER_OFF_X_DEFAULT;
static int       g_dialCenterOffsetY = DIAL_CENTER_OFF_Y_DEFAULT;
static int       g_brightnessPct = 100;
static bool      g_nightMode = false;
static int       g_rotationDeg  = 0;
static const int RPM_SCALE_MIN_VALUE = 4;        // VDO tach: 4 at ~7 o'clock (x100 RPM)
static const int RPM_SCALE_MAX_DEFAULT = 6000;   // Bulli default: scale ends at 60 (x100 RPM)
static const int RPM_SCALE_MAX_MIN     = 4000;
static const int RPM_SCALE_MAX_MAX     = 8000;
static const int RPM_REDLINE_DEFAULT = 4200;
static const int RPM_REDLINE_MIN     = 2000;
static const int RPM_REDLINE_MAX     = 8000;
static int       g_rpmRedline        = RPM_REDLINE_DEFAULT;
static int       g_rpmScaleMax       = RPM_SCALE_MAX_DEFAULT;
static const uint8_t MOTOR_STYLE_VDO       = 0;
static const uint8_t MOTOR_STYLE_MERCEDES  = 1;
static const int     MB_SCALE_MAX_VALUE    = 70;   // Mercedes tach 0–70 (×100 RPM)
static const float   MB_SCALE_MID_VALUE    = 35.0f;
static const int     MB_CLOCK_CX           = 240;
static const int     MB_CLOCK_CY           = 318;
static const int     MB_CLOCK_R            = 54;
static uint8_t       g_motorStyle          = MOTOR_STYLE_VDO;
static const uint8_t PAGE_MAX            = 9;   // 0=UHR .. 9=SETUP2 (5=SETUP1, 6=IMU)
static float     g_rotSin       = 0.0f;
static float     g_rotCos       = 1.0f;
// Throttled debug logging (115200 baud Serial)
static uint32_t  g_logTimeLastMs  = 0;
static uint32_t  g_logDrawLastMs  = 0;
static int       g_logLastRotDeg  = -1;
#if DEBUG_SERIAL
#define DLOG(...) Serial.printf(__VA_ARGS__)
#define DLOGN(msg) Serial.println(msg)
#else
#define DLOG(...) ((void)0)
#define DLOGN(msg) ((void)0)
#endif
#define LOG_TIME_MS  1000
#define LOG_DRAW_MS  1000
#define LOG_TOUCH_MS 80
#define TOUCH_RELEASE_MS   40   // ms nach letztem Touch-Frame bis Tap
#define TOUCH_COOLDOWN_MS  50   // ms zwischen Taps
#define DIAL_REBUILD_ROWS  24   // inkrementeller Dial-Cache pro Loop-Tick
static bool      g_featureWifi  = true;
static bool      g_featureBle   = false;
static bool      g_featureBuzzer = false;  // default OFF, per Setup/Web schaltbar
static bool      g_featureEspNow = true;
static uint8_t   g_espNowChannelPref = 0;  // 0=auto (WiFi), sonst 1..14
static bool      g_wifiApOnly   = false;   // Standalone AP (VDO-Clock-Setup), kein Home-STA
static bool      g_apOn         = false;
static bool      g_webStarted   = false;
static bool      g_redrawPage   = false;
static volatile bool g_otaBusy  = false;
static volatile size_t g_otaRxBytes = 0;
static const char* g_otaBootLabel = "normal";
static uint8_t   g_wifiProfile  = 0;
static WebServer webServer(80);
static void startWebServer();   // forward declaration
static void bleAbortDiscoveryScan();
static void reconnectWifiProfile();
static void applyBusProfile(bool reconnect);
static void syncHubHostForWifi();
static void disconnectBleForModeChange();
static void saveBleConnMode(BleConnMode mode);
static void saveDataPath(DataPath path);
static void saveHubHost(const char* host);
static void saveFeatures(bool wifi, bool ble, bool buzzer);
static void saveWifiProfile(uint8_t idx);
static void cycleWifiProfile();
static void enterApOnlyMode();
#if ENABLE_ESP_NOW_CLIENT
static void teardownEspNowClient();
static void saveEspNowFeature(bool enabled);
static const char* espNowChannelLabel();
#endif
static void stopApOnlyMode();
static void cycleWifiNetworkMode();
static void cycleSourceMode();
static void applySourceMode(const char* mode);
static void saveNightMode(bool enabled);
static void updateRotationCache();
static void logicalToDisplay(int lx, int ly, int *dx, int *dy);
static bool readTouch(uint16_t *x, uint16_t *y);
static uint16_t *g_dialCache = nullptr;
static bool      g_dialRebuilding = false;
static bool dialCacheMatches(int pct, int rot, int offX, int offY);

static void webOtaAbortUpload();

static void otaNoteBootState() {
  const esp_partition_t* part = esp_ota_get_running_partition();
  if (!part) return;
  esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
  if (esp_ota_get_state_partition(part, &state) != ESP_OK) return;

  if (state == ESP_OTA_IMG_PENDING_VERIFY) {
    g_otaBootLabel = "pending";
    Serial.println("OTA: neue Firmware wartet auf Self-Test");
  } else if (state == ESP_OTA_IMG_INVALID || state == ESP_OTA_IMG_ABORTED) {
    g_otaBootLabel = "rollback";
    Serial.println("OTA: Rollback-Boot erkannt");
  }
}

static void otaValidatePendingBoot() {
  const esp_partition_t* part = esp_ota_get_running_partition();
  if (!part) return;
  esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
  if (esp_ota_get_state_partition(part, &state) != ESP_OK) return;
  if (state != ESP_OTA_IMG_PENDING_VERIFY) return;
  if (!hal_ok()) {
    Serial.println("OTA: Display/HAL fehlgeschlagen -> Rollback");
    esp_ota_mark_app_invalid_rollback_and_reboot();
  }
}

static void otaConfirmBootIfPending() {
  const esp_partition_t* part = esp_ota_get_running_partition();
  if (!part) return;
  esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
  if (esp_ota_get_state_partition(part, &state) != ESP_OK) return;
  if (state != ESP_OTA_IMG_PENDING_VERIFY) return;
  if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
    g_otaBootLabel = "valid";
    Serial.println("OTA: Firmware bestaetigt (Rollback aus)");
  }
}

static bool webOtaRejectBusy() {
  if (!g_otaBusy) return false;
  webServer.sendHeader("Connection", "close");
  webServer.send(503, "text/plain", "OTA busy");
  return true;
}

static void webServerPoll(uint8_t n) {
  if (!g_webStarted) return;
  for (uint8_t i = 0; i < n; i++) webServer.handleClient();
}

#if FEATURE_TOUCH
static bool readTouch(uint16_t *x, uint16_t *y);
static void processTouchInput(bool *touchBusyOut);
static void drawCurrentPage();
static void gt911Init();
#endif

static void webOtaBeginUpload(const char* filename) {
  g_otaBusy = true;
  g_otaRxBytes = 0;
  hal_pause_for_ota(true);
  WiFi.setSleep(WIFI_PS_NONE);
  if (g_featureBle) {
    bleAbortDiscoveryScan();
    NimBLEDevice::getScan()->stop();
    if (bleClient && bleClient->isConnected()) bleClient->disconnect();
  }
  webServer.client().setNoDelay(true);
  webServer.client().setTimeout(300);
  Serial.printf("OTA: Start %s (heap %u, sketch free %u)\n",
                filename ? filename : "?", ESP.getFreeHeap(), ESP.getFreeSketchSpace());
  if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
    Update.printError(Serial);
    g_otaBusy = false;
    hal_pause_for_ota(false);
  }
}

static void webOtaWriteChunk(uint8_t* data, size_t len) {
  if (!g_otaBusy || !data || len == 0) return;
  g_otaRxBytes += len;
  if (Update.write(data, len) != len) {
    Update.printError(Serial);
    Update.abort();
    g_otaBusy = false;
    hal_pause_for_ota(false);
    return;
  }
  static uint32_t lastLog = 0;
  const size_t done = Update.progress();
  if (millis() - lastLog >= 2000 || (done % 131072) < len) {
    Serial.printf("OTA: %u bytes\n", (unsigned)done);
    lastLog = millis();
  }
  yield();
  delay(1);
  esp_task_wdt_reset();
}

static void webOtaFinishUpload(size_t totalSize) {
  if (!g_otaBusy) return;
  if (Update.end(true)) {
    Serial.printf("OTA: Erfolg, %u Bytes\n", (unsigned)totalSize);
  } else {
    Update.printError(Serial);
    g_otaBusy = false;
    hal_pause_for_ota(false);
  }
}

static void webOtaAbortUpload() {
  Update.abort();
  g_otaBusy = false;
  hal_pause_for_ota(false);
  Serial.println("OTA: abgebrochen");
}

struct WifiProfile {
  const char* ssid;
  const char* pass;
};

#ifndef HUB_SETUP_WIFI_SSID
#define HUB_SETUP_WIFI_SSID "Spartan3-Setup"
#endif
#ifndef HUB_SETUP_WIFI_PASS
#define HUB_SETUP_WIFI_PASS "lambda123"
#endif

static const WifiProfile WIFI_PROFILES[] = {
  { WIFI_SSID, WIFI_PASSWORD },
#ifdef WIFI_SSID_2
  { WIFI_SSID_2, WIFI_PASSWORD_2 },
#endif
#ifdef WIFI_SSID_3
  { WIFI_SSID_3, WIFI_PASSWORD_3 },
#endif
  { HUB_SETUP_WIFI_SSID, HUB_SETUP_WIFI_PASS },
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

static uint8_t hubWifiProfileIndex() {
  const uint8_t count = wifiProfileCount();
  for (uint8_t i = 0; i < count; i++) {
    if (strcmp(WIFI_PROFILES[i].ssid, HUB_SETUP_WIFI_SSID) == 0) return i;
  }
  return 0xFF;
}

static uint8_t preferredHomeWifiProfileIndex() {
  const uint8_t count = wifiProfileCount();
#ifdef WIFI_SSID_2
  for (uint8_t i = 0; i < count; i++) {
    if (strcmp(WIFI_PROFILES[i].ssid, WIFI_SSID_2) == 0) return i;
  }
#endif
  for (uint8_t i = 0; i < count; i++) {
    if (WIFI_PROFILES[i].ssid[0] != 0 && strcmp(WIFI_PROFILES[i].ssid, HUB_SETUP_WIFI_SSID) != 0) return i;
  }
  return 0;
}

static bool isBusWifiSsid(const char* ssid) {
  return ssid && strcmp(ssid, HUB_SETUP_WIFI_SSID) == 0;
}

static bool isOnBusWifi() {
  if (isBusWifiSsid(currentWifiSsid())) return true;
  return WiFi.status() == WL_CONNECTED && WiFi.SSID() == HUB_SETUP_WIFI_SSID;
}

static void syncHubHostForWifi() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (WiFi.SSID() == HUB_SETUP_WIFI_SSID) {
    if (strcmp(g_hubHost, HUB_BUS_HOST) != 0) saveHubHost(HUB_BUS_HOST);
  } else if (strcmp(WiFi.SSID().c_str(), WIFI_SSID) == 0
#ifdef WIFI_SSID_2
             || strcmp(WiFi.SSID().c_str(), WIFI_SSID_2) == 0
#endif
            ) {
    if (strcmp(g_hubHost, HUB_WIFI_DEFAULT_HOST) != 0) saveHubHost(HUB_WIFI_DEFAULT_HOST);
  } else {
    IPAddress gw = WiFi.gatewayIP();
    if (gw[0] != 0) {
      char buf[20];
      snprintf(buf, sizeof(buf), "%s", gw.toString().c_str());
      if (strcmp(g_hubHost, buf) != 0) saveHubHost(buf);
    }
  }
}

static void applyWifiIpConfig() {
  if (isBusWifiSsid(currentWifiSsid())) {
    IPAddress ip, gw, mask, dns;
    ip.fromString(HUB_BUS_CLIENT_IP);
    gw.fromString(HUB_BUS_HOST);
    mask.fromString("255.255.255.0");
    dns.fromString(HUB_BUS_HOST);
    WiFi.config(ip, gw, mask, dns);
    Serial.printf("WiFi: Bus static %s (hub %s)\n", HUB_BUS_CLIENT_IP, HUB_BUS_HOST);
  } else {
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
  }
}

static void applyBusProfile(bool reconnect) {
  const uint8_t idx = hubWifiProfileIndex();
  if (idx == 0xFF) {
    Serial.println("Bus: Spartan3-Setup Profil fehlt");
    return;
  }
  stopApOnlyMode();
  saveWifiProfile(idx);
  saveDataPath(DATA_PATH_BLE);
  saveHubHost(HUB_BUS_HOST);
  if (g_bleConnMode != BLE_MODE_DIRECT_123) {
    saveBleConnMode(BLE_MODE_DIRECT_123);
    disconnectBleForModeChange();
  }
  saveFeatures(true, true, g_featureBuzzer);
#if ENABLE_ESP_NOW_CLIENT
  saveEspNowFeature(true);
  g_espNowChannelPref = 0;
  Preferences p;
  p.begin("clock", false);
  p.putUChar("espnow_ch", g_espNowChannelPref);
  p.end();
  teardownEspNowClient();
#endif
  Serial.printf("Bus: Spartan3-Setup + hub %s + client %s + BLE 123 direkt\n",
                HUB_BUS_HOST, HUB_BUS_CLIENT_IP);
#if ENABLE_ESP_NOW_CLIENT
  Serial.printf("Bus: ESP-NOW recv %s (HTTP nur Fallback/Status)\n", espNowChannelLabel());
#endif
  if (reconnect) reconnectWifiProfile();
  g_redrawPage = true;
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

// ---- Timezone / NTP ----
struct TimezoneEntry {
  const char* label;
  const char* posix;
};

static const TimezoneEntry TIMEZONES[] = {
  { "Europe/Berlin (CET/CEST)", "CET-1CEST,M3.5.0/2,M10.5.0/3" },
  { "UTC",                      "UTC0" },
  { "Europe/London (GMT/BST)",  "GMT0BST,M3.5.0/1,M10.5.0" },
  { "America/New_York (EST/EDT)","EST5EDT,M3.2.0,M11.1.0" },
  { "America/Los_Angeles (PST/PDT)", "PST8PDT,M3.2.0,M11.1.0" },
  { "Asia/Tokyo (JST)",         "JST-9" },
};
static const uint8_t TIMEZONE_COUNT =
    (uint8_t)(sizeof(TIMEZONES) / sizeof(TIMEZONES[0]));
static const uint8_t TIMEZONE_DEFAULT = 0;  // Europe/Berlin

static uint8_t  g_timezoneIdx      = TIMEZONE_DEFAULT;
static bool     g_sntpStarted      = false;
static bool     g_ntpSynced        = false;
static bool     g_hubTimeSynced    = false;
static bool     g_ntpResyncRequested = false;
static uint32_t g_lastRtcSyncMs    = 0;
static uint32_t g_lastHubEpochSyncMs = 0;
static constexpr uint32_t NTP_RESYNC_INTERVAL_MS = 15UL * 60UL * 1000UL;  // 15 min (ESP default: 1 h)
static constexpr uint32_t HUB_TIME_RESYNC_MS    = 60UL * 1000UL;
static uint8_t timezoneCount() { return TIMEZONE_COUNT; }

static const char* timezoneLabel(uint8_t idx) {
  if (idx >= TIMEZONE_COUNT) idx = TIMEZONE_DEFAULT;
  return TIMEZONES[idx].label;
}

static const char* timezonePosix(uint8_t idx) {
  if (idx >= TIMEZONE_COUNT) idx = TIMEZONE_DEFAULT;
  return TIMEZONES[idx].posix;
}

static const char* currentTimezonePosix() {
  return timezonePosix(g_timezoneIdx);
}

static void applyTimezone() {
  const char* tz = currentTimezonePosix();
  setenv("TZ", tz, 1);
  tzset();
  configTzTime(tz, "pool.ntp.org", "time.nist.gov", "de.pool.ntp.org");
  // IMMED: avoid adjtime slow-correction leaving ~1 min visible offset after sync.
  esp_sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
  esp_sntp_set_sync_interval(NTP_RESYNC_INTERVAL_MS);
}

static void requestNtpResync() {
  g_ntpResyncRequested = true;
  g_ntpSynced = false;
  if (g_sntpStarted && esp_sntp_enabled()) sntp_restart();
}

static void saveTimezone(uint8_t idx) {
  if (idx >= TIMEZONE_COUNT) idx = TIMEZONE_DEFAULT;
  if (idx == g_timezoneIdx) return;
  g_timezoneIdx = idx;
  Preferences p;
  p.begin("clock", false);
  p.putUChar("tz_idx", g_timezoneIdx);
  p.end();
  applyTimezone();
  requestNtpResync();
  Serial.printf("TZ: %s (%s)\n", timezoneLabel(g_timezoneIdx), currentTimezonePosix());
}

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

static const char* clockSourceLabel();

static bool readClockTime(struct tm *now) {
  // Priority: Hub NTP (via /api/status) > local SNTP > PCF85063 RTC.
  const char *src = "none";
  time_t t = time(nullptr);
  if (t > 1700000000) {
    localtime_r(&t, now);
    src = clockSourceLabel();
  } else if (rtcRead(now)) {
    src = "RTC";
  } else {
    memset(now, 0, sizeof(*now));
    return false;
  }
  const uint32_t logMs = millis();
#if FEATURE_VERBOSE_SERIAL
  if (!g_logTimeLastMs || logMs - g_logTimeLastMs >= LOG_TIME_MS) {
    g_logTimeLastMs = logMs;
    Serial.printf("[TIME] %02d:%02d:%02d src=%s epoch=%ld rot=%d\n",
                  now->tm_hour, now->tm_min, now->tm_sec, src, (long)t, g_rotationDeg);
  }
#endif
  return true;
}

static bool tmWithinSeconds(const struct tm *a, const struct tm *b, int maxDelta) {
  struct tm ac = *a, bc = *b;
  ac.tm_isdst = bc.tm_isdst = -1;
  const time_t ta = mktime(&ac);
  const time_t tb = mktime(&bc);
  if (ta == (time_t)-1 || tb == (time_t)-1) return false;
  return labs((long)(ta - tb)) <= maxDelta;
}

static bool syncRtcFromNtp(const struct tm *ntpLocal, bool force) {
  struct tm rtcNow = {};
  if (!force && rtcRead(&rtcNow) && tmWithinSeconds(&rtcNow, ntpLocal, 2)) return false;
  rtcWrite(ntpLocal);
  g_lastRtcSyncMs = millis();
  g_ntpSynced = true;
  g_ntpResyncRequested = false;
  Serial.printf("NTP -> RTC: %04d-%02d-%02d %02d:%02d:%02d\n",
                ntpLocal->tm_year + 1900, ntpLocal->tm_mon + 1, ntpLocal->tm_mday,
                ntpLocal->tm_hour, ntpLocal->tm_min, ntpLocal->tm_sec);
  return true;
}

static const char* clockSourceLabel() {
  if (g_hubTimeSynced && g_hubWifiOk) return "HUB";
  if (g_ntpSynced) return "SNTP";
  time_t t = time(nullptr);
  if (t > 1700000000) return "SYS";
  return "RTC";
}

static void initTimeSource() {
  struct tm rtcNow = {};
  bool haveRtc  = rtcRead(&rtcNow);
  bool rtcValid = haveRtc && (rtcNow.tm_year + 1900) >= 2024;
  Preferences prefs;
  prefs.begin("clock", false);
  uint32_t savedId = prefs.getUInt("buildid", 0);
  bool newFlash = (savedId != (uint32_t)RTC_BUILD_ID);
  if (!rtcValid) {
    struct tm bt = {};
    bt.tm_year = RTC_BUILD_Y - 1900;
    bt.tm_mon  = RTC_BUILD_MO - 1;
    bt.tm_mday = RTC_BUILD_D;
    bt.tm_hour = RTC_BUILD_H;
    bt.tm_min  = RTC_BUILD_MI;
    bt.tm_sec  = RTC_BUILD_S;
    bt.tm_wday = RTC_BUILD_DOW;
    rtcWrite(&bt);
    Serial.printf("RTC set from build time: %04d-%02d-%02d %02d:%02d:%02d (RTC invalid)\n",
                  RTC_BUILD_Y, RTC_BUILD_MO, RTC_BUILD_D, RTC_BUILD_H, RTC_BUILD_MI, RTC_BUILD_S);
  } else if (newFlash) {
    Serial.printf("New firmware (build id %u): RTC kept %04d-%02d-%02d %02d:%02d:%02d, await NTP\n",
                  (unsigned)RTC_BUILD_ID,
                  rtcNow.tm_year + 1900, rtcNow.tm_mon + 1, rtcNow.tm_mday,
                  rtcNow.tm_hour, rtcNow.tm_min, rtcNow.tm_sec);
  } else {
    Serial.printf("RTC running: %04d-%02d-%02d %02d:%02d:%02d\n",
                  rtcNow.tm_year + 1900, rtcNow.tm_mon + 1, rtcNow.tm_mday,
                  rtcNow.tm_hour, rtcNow.tm_min, rtcNow.tm_sec);
  }
  prefs.putUInt("buildid", (uint32_t)RTC_BUILD_ID);
  prefs.end();
}

// Non-blocking WiFi/NTP handler. Returns true on fresh NTP sync.
static bool wifiNtpTick() {
  static uint32_t lastTry = 0;
  static uint32_t failSince = 0;

  if (!g_featureWifi || strlen(currentWifiSsid()) == 0) return false;

  if (WiFi.status() != WL_CONNECTED) {
    if (failSince == 0) failSince = millis();
    if (millis() - lastTry > 30000) {
      lastTry = millis();
      applyWifiIpConfig();
      WiFi.begin(currentWifiSsid(), currentWifiPassword());  // kein disconnect() davor
      Serial.printf("WiFi: Reconnect-Versuch zu '%s'\n", currentWifiSsid());
    }
    if (g_ipStr[0] == '-') strcpy(g_ipStr, "...");
    return false;
  }

  failSince = 0;
  syncHubHostForWifi();

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
  if (!g_sntpStarted) {
    applyTimezone();
    g_sntpStarted = true;
    Serial.printf("NTP: SNTP gestartet (TZ %s)\n", timezoneLabel(g_timezoneIdx));
  }
  if (g_hubTimeSynced && g_hubWifiOk &&
      (millis() - g_hubLastOkMs) < (HUB_TIME_RESYNC_MS + 5000)) {
    return false;
  }
  time_t t = time(nullptr);
  if (t > 1700000000) {
    struct tm now;
    localtime_r(&t, &now);
    const bool wasSynced = g_ntpSynced;
    const bool resyncDue = g_lastRtcSyncMs != 0
        && (millis() - g_lastRtcSyncMs) > NTP_RESYNC_INTERVAL_MS;
    const bool resyncReq = g_ntpResyncRequested;
    if ((resyncDue || resyncReq) && esp_sntp_enabled()) esp_sntp_restart();
    const bool rtcUpdated = syncRtcFromNtp(&now, resyncDue || resyncReq);
    if (!g_ntpSynced) g_ntpSynced = true;
    if (rtcUpdated || !wasSynced || resyncDue || resyncReq) {
      if (!wasSynced) Serial.println("NTP: synchronisiert");
      return true;
    }
  }
  return false;
}

// -------- Spartan Hub WiFi client (HTTP /api/status poll) --------
static const char* dataPathLabel() {
  return g_dataPath == DATA_PATH_WIFI_HUB ? "WiFi" : "BLE";
}

static bool dataFresh() {
  return g_liveLastRx != 0 && (millis() - g_liveLastRx) < DATA_FRESH_MS;
}

static void touchLiveRx();

#if ENABLE_ESP_NOW_CLIENT
static bool espNowClientActive() {
  return g_featureEspNow;
}

static uint8_t espNowEffectiveChannel() {
  if (WiFi.status() == WL_CONNECTED) {
    const uint8_t ch = WiFi.channel();
    if (ch > 0 && ch <= 14 && g_espNowChannelPref == 0) return ch;
  }
  if (g_espNowChannelPref >= 1 && g_espNowChannelPref <= 14) {
    return g_espNowChannelPref;
  }
  return ESP_NOW_WIFI_CHANNEL;
}

static void teardownEspNowClient() {
  if (!g_espNowReady) return;
  esp_now_deinit();
  g_espNowReady = false;
  g_espNowActiveChannel = 0;
  g_espNowLastRxMs = 0;
}

static const char* espNowChannelLabel() {
  static char buf[12];
  if (!g_featureEspNow) return "AUS";
  if (g_espNowChannelPref >= 1 && g_espNowChannelPref <= 14) {
    snprintf(buf, sizeof(buf), "%u", g_espNowChannelPref);
    return buf;
  }
  if (WiFi.status() == WL_CONNECTED) {
    snprintf(buf, sizeof(buf), "A%u", WiFi.channel());
    return buf;
  }
  return "AUTO";
}

static void cycleEspNowChannelPref() {
  if (g_espNowChannelPref == 0) g_espNowChannelPref = 6;
  else if (g_espNowChannelPref == 6) g_espNowChannelPref = 11;
  else g_espNowChannelPref = 0;
  Preferences p;
  p.begin("clock", false);
  p.putUChar("espnow_ch", g_espNowChannelPref);
  p.end();
  teardownEspNowClient();
  Serial.printf("ESP-NOW: Kanal -> %s\n", espNowChannelLabel());
}

static void saveEspNowFeature(bool enabled) {
  g_featureEspNow = enabled;
  Preferences p;
  p.begin("clock", false);
  p.putBool("feat_espnow", enabled);
  p.end();
  if (!enabled) teardownEspNowClient();
  else Serial.println("ESP-NOW: AN");
}

static bool espNowDataFresh() {
  return g_espNowLastRxMs != 0 && (millis() - g_espNowLastRxMs) < DATA_FRESH_MS;
}

static bool espNowFallbackAllowed() {
  return !g_featureEspNow;
}

static void applyEspNowFrame(const SpartanCockpitFrame& frame) {
  g_espNowSeq = frame.seq;
  const bool lambdaValid = (frame.flags & kSpartanFlagLambdaValid) != 0;
  if (lambdaValid && frame.lambda_x1000 > 0) {
    g_lambda = frame.lambda_x1000 / 1000.0f;
    g_lambdaValid = true;
  }
  if ((frame.flags & kSpartanFlagTuneFresh) != 0) {
    g_rpm = frame.rpm;
    g_adv = frame.advance_x10 / 10.0f;
    g_map = frame.map;
  }
  g_espNowLastRxMs = millis();
  strncpy(g_hubSourceTag, "espnow", sizeof(g_hubSourceTag));
  g_hubSourceTag[sizeof(g_hubSourceTag) - 1] = 0;
  touchLiveRx();
}

#if defined(ESP_IDF_VERSION) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  (void)info;
#else
static void IRAM_ATTR onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
  (void)mac;
#endif
  if (len != (int)kSpartanCockpitFrameSize) return;
  SpartanCockpitFrame frame;
  memcpy(&frame, data, sizeof(frame));
  if (!spartanCockpitFrameValid(frame)) return;
  portENTER_CRITICAL_ISR(&g_espNowMux);
  g_espNowPendingFrame = frame;
  g_espNowPending = true;
  if (g_espNowRx < UINT32_MAX) g_espNowRx++;
  portEXIT_CRITICAL_ISR(&g_espNowMux);
}

static void espNowClientProcessPending() {
  if (!g_espNowPending) return;
  SpartanCockpitFrame frame;
  portENTER_CRITICAL(&g_espNowMux);
  frame = g_espNowPendingFrame;
  g_espNowPending = false;
  portEXIT_CRITICAL(&g_espNowMux);
  applyEspNowFrame(frame);
}

static void setupEspNowClient() {
  if (!espNowClientActive()) return;
  if (WiFi.getMode() == WIFI_OFF) {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
  }
  const uint8_t channel = espNowEffectiveChannel();
  if (g_espNowReady && g_espNowActiveChannel == channel) return;
  if (g_espNowReady) teardownEspNowClient();

  if (WiFi.status() != WL_CONNECTED) {
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  }
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW: init failed");
    return;
  }
  esp_now_register_recv_cb(onEspNowRecv);
  g_espNowReady = true;
  g_espNowActiveChannel = channel;
  Serial.printf("ESP-NOW: recv ready on channel %u\n", channel);
}

static void espNowClientTick() {
  espNowClientProcessPending();
  if (!espNowClientActive()) {
    teardownEspNowClient();
    return;
  }
  const uint8_t channel = espNowEffectiveChannel();
  if (g_espNowReady && g_espNowActiveChannel != channel) {
    teardownEspNowClient();
  }
  setupEspNowClient();
}
#endif

static bool hubLinkOk() {
  if (g_dataPath == DATA_PATH_WIFI_HUB) {
#if ENABLE_ESP_NOW_CLIENT
    if (espNowDataFresh()) return true;
#endif
    if (!g_featureWifi || WiFi.status() != WL_CONNECTED) return false;
    if (g_hubWifiOk) return true;
    // Grace after last OK poll — avoids KEIN HUB flicker between 300 ms polls
    return g_hubLastOkMs != 0 && (millis() - g_hubLastOkMs) < 5000;
  }
  return g_featureBle && g_bleConn;
}

static void touchLiveRx() {
  g_liveLastRx = millis();
  g_liveRxCnt++;
}

static void clearLiveValues() {
  g_lambda = g_rpm = g_adv = g_map = 0;
  g_battVolt = g_speedKmh = g_auxBattVolt = 0;
  g_g123Volt = g_g123Temp = g_g123Coil = 0;
  g_lambdaValid = g_battValid = g_auxBattValid = g_speedValid = g_g123Valid = false;
  g_liveLastRx = 0;
  g_liveRxCnt = 0;
  g_bleRxCnt = 0;
  g_bleLastRx = 0;
  g_hubWifiOk = false;
  g_hubLastOkMs = 0;
  strncpy(g_hubSourceTag, "---", sizeof(g_hubSourceTag));
  g_hubSourceTag[sizeof(g_hubSourceTag) - 1] = 0;
}

static int jsonKeyPos(const String& json, const char* key) {
  const String needle = String("\"") + key + "\":";
  return json.indexOf(needle);
}

static bool jsonExtractFloat(const String& json, const char* key, float* out) {
  const int i = jsonKeyPos(json, key);
  if (i < 0) return false;
  int p = i + (int)strlen(key) + 3;
  while (p < (int)json.length() && (json[p] == ' ' || json[p] == '\t')) p++;
  *out = json.substring(p).toFloat();
  return true;
}

static bool jsonExtractBool(const String& json, const char* key, bool* out) {
  const int i = jsonKeyPos(json, key);
  if (i < 0) return false;
  int p = i + (int)strlen(key) + 3;
  while (p < (int)json.length() && json[p] == ' ') p++;
  if (json.startsWith("true", p)) { *out = true; return true; }
  if (json.startsWith("false", p)) { *out = false; return true; }
  return false;
}

static bool jsonExtractULong(const String& json, const char* key, unsigned long* out) {
  const int i = jsonKeyPos(json, key);
  if (i < 0) return false;
  int p = i + (int)strlen(key) + 3;
  while (p < (int)json.length() && (json[p] == ' ' || json[p] == '\t')) p++;
  char* end = nullptr;
  const unsigned long v = strtoul(json.substring(p).c_str(), &end, 10);
  if (end == nullptr || v == 0) return false;
  *out = v;
  return true;
}

static bool jsonExtractString(const String& json, const char* key, char* out, size_t outLen) {
  const int i = jsonKeyPos(json, key);
  if (i < 0) return false;
  int q1 = json.indexOf('"', i + (int)strlen(key) + 3);
  if (q1 < 0) return false;
  int q2 = json.indexOf('"', q1 + 1);
  if (q2 < 0) return false;
  String s = json.substring(q1 + 1, q2);
  strncpy(out, s.c_str(), outLen - 1);
  out[outLen - 1] = 0;
  return true;
}

static bool maybeSyncClockFromHubJson(const String& json) {
  bool hubNtp = false;
  if (!jsonExtractBool(json, "ntp_synced", &hubNtp) || !hubNtp) return false;
  unsigned long epoch = 0;
  if (!jsonExtractULong(json, "time_epoch", &epoch) || epoch < 1700000000UL) return false;

  const uint32_t nowMs = millis();
  const time_t cur = time(nullptr);
  const long drift = (cur > 1700000000) ? labs((long)epoch - (long)cur) : 999;
  if (g_hubTimeSynced && drift < 2 &&
      (nowMs - g_lastHubEpochSyncMs) < HUB_TIME_RESYNC_MS) {
    return true;
  }

  struct timeval tv = {};
  tv.tv_sec = (time_t)epoch;
  settimeofday(&tv, nullptr);
  struct tm local = {};
  const time_t epochT = (time_t)epoch;
  localtime_r(&epochT, &local);
  syncRtcFromNtp(&local, drift >= 2 || !g_hubTimeSynced);
  g_hubTimeSynced = true;
  g_ntpSynced = true;
  g_lastHubEpochSyncMs = nowMs;
  Serial.printf("Hub -> clock: epoch %lu (%02d:%02d:%02d TZ %s)\n",
                epoch, local.tm_hour, local.tm_min, local.tm_sec,
                timezoneLabel(g_timezoneIdx));
  return true;
}

static bool parseHubStatusJson(const String& json) {
  bool valid = false;
  const bool hasValid = jsonExtractBool(json, "valid", &valid);

  maybeSyncClockFromHubJson(json);

#if ENABLE_ESP_NOW_CLIENT
  if (!espNowFallbackAllowed()) {
    float bm6v = 0, auxv = 0, volt = 0, speed = 0;
    if (jsonExtractFloat(json, "bm6_voltage", &bm6v) && bm6v > 0.5f) {
      g_battVolt = bm6v;
      g_battValid = true;
    } else if (jsonExtractFloat(json, "volt", &volt) && volt > 0.5f) {
      g_battVolt = volt;
      g_battValid = true;
    }
    if (jsonExtractFloat(json, "bm6_aux_voltage", &auxv) && auxv > 0.5f) {
      g_auxBattVolt = auxv;
      g_auxBattValid = true;
    }
    if (jsonExtractFloat(json, "speed_kmh", &speed) && speed >= 0.0f) {
      g_speedKmh = speed;
      g_speedValid = true;
    }
    return true;
  }
#endif

  float lambda = 0, rpm = 0, adv = 0, map = 0;
  jsonExtractFloat(json, "lambda", &lambda);
  jsonExtractFloat(json, "rpm", &rpm);
  jsonExtractFloat(json, "advance", &adv);
  jsonExtractFloat(json, "map", &map);

  g_lambda = lambda;
  g_lambdaValid = lambda > 0.001f && (!hasValid || valid);
  g_rpm = rpm;
  g_adv = adv;
  g_map = map;

  float bm6v = 0, auxv = 0, volt = 0, speed = 0;
  if (jsonExtractFloat(json, "bm6_voltage", &bm6v) && bm6v > 0.5f) {
    g_battVolt = bm6v;
    g_battValid = true;
  } else if (jsonExtractFloat(json, "volt", &volt) && volt > 0.5f) {
    g_battVolt = volt;
    g_battValid = true;
  } else {
    g_battValid = false;
  }
  if (jsonExtractFloat(json, "bm6_aux_voltage", &auxv) && auxv > 0.5f) {
    g_auxBattVolt = auxv;
    g_auxBattValid = true;
  } else {
    g_auxBattValid = false;
  }
  if (jsonExtractFloat(json, "speed_kmh", &speed) && speed >= 0.0f) {
    g_speedKmh = speed;
    g_speedValid = true;
  } else {
    g_speedValid = false;
  }
  jsonExtractString(json, "source", g_hubSourceTag, sizeof(g_hubSourceTag));
  touchLiveRx();
  return true;
}

static bool pollHubAtHost(const char* host, uint32_t now, bool persistHost) {
  HTTPClient http;
  const String url = String("http://") + host + "/api/status";
  http.setTimeout(HUB_WIFI_TIMEOUT_MS);
  if (!http.begin(url)) return false;
  http.addHeader("X-Device", "vdo-clock-28");
  const int code = http.GET();
  bool ok = false;
  if (code == HTTP_CODE_OK) {
    const String body = http.getString();
    if (body.length() > 20 && parseHubStatusJson(body)) {
      if (persistHost && strcmp(g_hubHost, host) != 0) saveHubHost(host);
      g_hubWifiOk = true;
      g_hubLastOkMs = now;
      strncpy(g_hubState, "ok", sizeof(g_hubState));
      g_hubState[sizeof(g_hubState) - 1] = 0;
      ok = true;
    }
  }
  http.end();
  return ok;
}

static void hubWifiPollTick() {
  if (g_dataPath != DATA_PATH_WIFI_HUB || !g_featureWifi) return;
  static uint32_t nextPollMs = 0;
  const uint32_t now = millis();
  if (now < nextPollMs) return;
  nextPollMs = now + HUB_WIFI_POLL_MS;

  if (WiFi.status() != WL_CONNECTED) {
    g_hubWifiOk = false;
    g_hubTimeSynced = false;
    strncpy(g_hubState, "no_wifi", sizeof(g_hubState));
    g_hubState[sizeof(g_hubState) - 1] = 0;
    return;
  }
  syncHubHostForWifi();
  if (g_hubHost[0] == '\0') {
    g_hubWifiOk = false;
    strncpy(g_hubState, "no_host", sizeof(g_hubState));
    g_hubState[sizeof(g_hubState) - 1] = 0;
    return;
  }

  g_hubPollCnt++;
  if (pollHubAtHost(g_hubHost, now, false)) return;

  g_hubWifiOk = false;
  g_hubErrCnt++;
  strncpy(g_hubState, "poll_fail", sizeof(g_hubState));
  g_hubState[sizeof(g_hubState) - 1] = 0;
}

// -------- BLE Spartan3-Hub / 123TUNE+ client --------
static const char* bleConnModeLabel() {
  return g_bleConnMode == BLE_MODE_SPARTAN_HUB ? "HUB" : "123 dir";
}

static bool macEquals(const String& a, const char* b) {
  String x = a; x.toLowerCase();
  String y = String(b); y.toLowerCase();
  return x == y;
}

static void clearBleLiveValues() {
  clearLiveValues();
}

static void bleIoYield();  // GT911 poll during BLE blocking waits

static void bleWaitScanStopped(NimBLEScan* s, uint32_t timeoutMs) {
  const uint32_t until = millis() + timeoutMs;
  while (s->isScanning() && millis() < until) {
    bleIoYield();
    delay(10);
  }
}

static void bleWifiSleepRestore() {
  if (g_bleScanWifiSleepWasOn) {
    WiFi.setSleep(true);
    g_bleScanWifiSleepWasOn = false;
  }
  g_bleOpScanActive = false;
}

static void bleDiscoveryScanEnded() {
  bleWifiSleepRestore();
}

static void bleOpScanBegin() {
  if (!g_bleOpScanActive) {
    g_bleScanWifiSleepWasOn = (WiFi.getSleep() != WIFI_PS_NONE);
    WiFi.setSleep(WIFI_PS_NONE);
    g_bleOpScanActive = true;
  }
}

static void bleAbortDiscoveryScan() {
  if (!g_bleDiscoveryScan) return;
  if (g_bleStackReady) {
    auto* s = NimBLEDevice::getScan();
    s->stop();
    bleWaitScanStopped(s, 500);
  }
  g_bleDiscoveryScan = false;
  bleDiscoveryScanEnded();
}

// Touch hat Vorrang: laufenden BLE-Scan abbrechen (Radio/CPU entlasten).
static void blePauseForTouch() {
  auto* s = NimBLEDevice::getScan();
  if (!s || !s->isScanning()) return;
  s->stop();
  bleWaitScanStopped(s, 150);
  bleWifiSleepRestore();
  if (!g_bleConn && !bleDoConnect) {
    bleNextScanAt = millis() + 4000;
  }
}

static void disconnectBleForModeChange() {
  bleDoConnect = false;
  g_bleConn = false;
  g_bleHubName = "---";
  g_nusRx = nullptr;
  g_ble123PingAt = 0;
  g_ble123KickAt = 0;
  g_ble123KickStage = BLE_123_KICK_IDLE;
  g_ble123ScanNext = false;
  g_ble123AddrFlip = 0;
  g_bleHubScanNext = true;
  clearBleLiveValues();
  bleAbortDiscoveryScan();
  if (g_bleStackReady) {
    NimBLEDevice::getScan()->stop();
    if (bleClient && bleClient->isConnected()) bleClient->disconnect();
  }
  bleNextScanAt = millis() + 2000;
}

static void saveBleConnMode(BleConnMode mode) {
  g_bleConnMode = mode;
  Preferences p;
  p.begin("clock", false);
  p.putUChar("ble_mode", static_cast<uint8_t>(g_bleConnMode));
  p.end();
}

static void saveBleMac123(const char* mac, uint8_t addrType = 0xFF) {
  strncpy(g_bleTargetMac, mac, sizeof(g_bleTargetMac) - 1);
  g_bleTargetMac[sizeof(g_bleTargetMac) - 1] = 0;
  if (addrType != 0xFF) g_ble123AddrType = addrType;
  Preferences p;
  p.begin("clock", false);
  p.putString("ble_mac_123", g_bleTargetMac);
  if (addrType != 0xFF) p.putUChar("ble_addr_123", g_ble123AddrType);
  p.end();
}

static void saveBleMacHub(const char* mac) {
  strncpy(g_bleHubMac, mac, sizeof(g_bleHubMac) - 1);
  g_bleHubMac[sizeof(g_bleHubMac) - 1] = 0;
  Preferences p;
  p.begin("clock", false);
  p.putString("ble_mac_hub", g_bleHubMac);
  p.end();
}

static const char* bleSavedMacForMode() {
  if (g_bleConnMode == BLE_MODE_SPARTAN_HUB) {
    return g_bleHubMac[0] != '\0' ? g_bleHubMac : DEFAULT_HUB_MAC;
  }
  return g_bleTargetMac[0] != '\0' ? g_bleTargetMac : DEFAULT_123_MAC;
}

static const char* bleTargetDisplay() {
  return bleSavedMacForMode();
}

static void cycleBleConnMode() {
  BleConnMode next = g_bleConnMode == BLE_MODE_SPARTAN_HUB ?
                     BLE_MODE_DIRECT_123 : BLE_MODE_SPARTAN_HUB;
  if (next == g_bleConnMode) return;
  saveBleConnMode(next);
  disconnectBleForModeChange();
  DLOG("BLE: Quelle -> %s\n", bleConnModeLabel());
}

static void saveDataPath(DataPath path);  // fwd
static void saveFeatures(bool wifi, bool ble, bool buzzer);  // fwd

static const char* sourceModeLabel() {
#if ENABLE_ESP_NOW_CLIENT
  if (espNowDataFresh()) return "HUB ESP-NOW";
#endif
  if (g_dataPath == DATA_PATH_WIFI_HUB) return g_featureEspNow ? "HTTP Diagnose" : "HTTP Live";
  if (g_bleConnMode == BLE_MODE_SPARTAN_HUB) return g_featureEspNow ? "BLE Diagnose" : "BLE Live";
  return "123 dir";
}

static void cycleSourceMode() {
  if (g_dataPath == DATA_PATH_WIFI_HUB) {
    saveDataPath(DATA_PATH_BLE);
    if (!g_featureBle) {
      saveFeatures(g_featureWifi, true, g_featureBuzzer);
    } else if (g_bleConnMode != BLE_MODE_SPARTAN_HUB) {
      saveBleConnMode(BLE_MODE_SPARTAN_HUB);
      disconnectBleForModeChange();
    }
  } else if (g_bleConnMode == BLE_MODE_SPARTAN_HUB) {
    if (!g_featureBle) saveFeatures(g_featureWifi, true, g_featureBuzzer);
    saveBleConnMode(BLE_MODE_DIRECT_123);
    disconnectBleForModeChange();
  } else {
    saveDataPath(DATA_PATH_WIFI_HUB);
  }
  g_redrawPage = true;
  DLOG("Source -> %s\n", sourceModeLabel());
}

static int bleHexNib(uint8_t c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0;
}

static void decode123Frame(const uint8_t* d, size_t n) {
  if (n < 3) return;
  int hi  = bleHexNib(d[1]);
  int lo  = bleHexNib(d[2]);
  int raw = (hi << 4) | lo;
  switch (d[0]) {
    case 0x30: g_rpm = hi * 800.0f + lo * 50.0f; break;
    case 0x31: g_adv = hi * 3.2f   + lo * 0.2f;  break;
    case 0x32: g_map = (float)raw;               break;
    case 0x33:
      g_g123Temp = (float)(raw - 30);
      g_g123Valid = true;
      break;
    case 0x41:
      g_battVolt = raw / 4.54f;
      g_battValid = g_battVolt > 0.5f;
      break;
    default: break;
  }
  g_bleRxCnt++;
  g_bleLastRx = millis();
  if (g_dataPath == DATA_PATH_BLE) touchLiveRx();
  if (g_bleConnMode == BLE_MODE_DIRECT_123) {
    BLELOG_LIVE("BLE: LIVE rpm=%.0f adv=%.1f map=%.0f rx=%lu",
                g_rpm, g_adv, g_map, (unsigned long)g_bleRxCnt);
  }
}

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
  int posW   = p.indexOf('W', posM + 1);
  int posS   = p.indexOf('S', posM + 1);
  int posI   = p.indexOf('I', posM + 1);
  int mapEnd = posV > posM ? posV : (posW > posM ? posW : (posS > posM ? posS : (posI > posM ? posI : p.length())));
  g_map = p.substring(posM + 1, mapEnd).toFloat();

  if (posV > posM) {
    int vEnd = posW > posV ? posW : (posS > posV ? posS : (posI > posV ? posI : p.length()));
    float v = p.substring(posV + 1, vEnd).toFloat();
    if (v > 0.5f) { g_battVolt = v; g_battValid = true; }
  }
  if (posW > posM) {
    int wEnd = posS > posW ? posS : (posI > posW ? posI : p.length());
    float w = p.substring(posW + 1, wEnd).toFloat();
    if (w > 0.5f) { g_auxBattVolt = w; g_auxBattValid = true; }
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
  if (g_dataPath == DATA_PATH_BLE) touchLiveRx();
}

static void bleNotifyHubCB(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
  String s;
  s.reserve(len + 1);
  for (size_t i = 0; i < len; i++) s += (char)data[i];
  parseSpartanPayload(s);
}

static void bleNotify123CB(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
  decode123Frame(data, len);
}

class SpartanClientCB : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient*) override {
    bleNextScanAt = 0;
    bleLogSetState("link_up");
    BLELOG("BLE: link up (%s)", bleConnModeLabel());
    // g_bleConn erst nach NUS/Hub-Subscribe (sonst bricht Setup ab)
    if (g_bleConnMode == BLE_MODE_SPARTAN_HUB) {
      g_bleConn = true;
      bleLogSetState("connected");
    }
  }
  void onDisconnect(NimBLEClient*, int reason) override {
    g_bleConn = false;
    g_bleHubName = "---";
    g_nusRx = nullptr;
    g_ble123PingAt = 0;
    g_ble123KickAt = 0;
    g_ble123KickStage = BLE_123_KICK_IDLE;
    uint32_t pause = (g_bleConnMode == BLE_MODE_DIRECT_123) ?
                     BLE_RECONNECT_123_MS : BLE_BG_SCAN_INTERVAL_MS;
    // 520/0x208 = Link-Timeout — schnell Direct+Scan abwechseln (M5 bleibt dran)
    if (g_bleConnMode == BLE_MODE_DIRECT_123 &&
        (reason == 520 || reason == 0x208 || reason == 0x213)) {
      pause = 400;
      g_ble123ScanNext = true;
    } else if (g_bleConnMode == BLE_MODE_SPARTAN_HUB) {
      g_bleHubScanNext = true;
    }
    if (g_bleSetupBusy) pause = 800;
    bleLogSetState("disconnected");
    BLELOG("BLE: getrennt reason=%d, reconnect in %lums (%s)",
           reason, (unsigned long)pause, bleConnModeLabel());
    bleNextScanAt = millis() + pause;
  }
  bool onConnParamsUpdateRequest(NimBLEClient*, const ble_gap_upd_params*) override {
    return true;
  }
};

static void bleScanListAdd(const NimBLEAdvertisedDevice* dev) {
  if (g_bleScanCount >= BLE_SCAN_MAX) return;
  String addr = dev->getAddress().toString().c_str();
  addr.toLowerCase();
  String name = dev->getName().c_str();
  name.toLowerCase();
  const bool is123Mac = macEquals(addr, DEFAULT_123_MAC) || macEquals(addr, g_bleTargetMac);
  const bool is123Name = name.indexOf("123") >= 0 || name.indexOf("bm6") >= 0 ||
                         name.indexOf("micro") >= 0 || name.indexOf("tune") >= 0 ||
                         name.indexOf("squirt") >= 0 || name.indexOf("ign") >= 0;
  const bool hasNus = dev->isAdvertisingService(NimBLEUUID(NUS_SVC)) || is123Mac || is123Name;
  const bool hasSpartan = dev->isAdvertisingService(NimBLEUUID(SPARTAN_SVC)) ||
                          name.indexOf("spartan") >= 0 || macEquals(addr, SPARTAN_MAC);
  for (uint8_t i = 0; i < g_bleScanCount; i++) {
    if (macEquals(addr, g_bleScanList[i].mac)) {
      g_bleScanList[i].rssi = dev->getRSSI();
      if (name.length() > 0) {
        strncpy(g_bleScanList[i].name, name.c_str(), sizeof(g_bleScanList[i].name) - 1);
        g_bleScanList[i].name[sizeof(g_bleScanList[i].name) - 1] = 0;
      }
      g_bleScanList[i].spartan = g_bleScanList[i].spartan || hasSpartan;
      g_bleScanList[i].nus = g_bleScanList[i].nus || hasNus;
      return;
    }
  }
  BleScanEntry& e = g_bleScanList[g_bleScanCount++];
  strncpy(e.mac, addr.c_str(), sizeof(e.mac) - 1);
  e.mac[sizeof(e.mac) - 1] = 0;
  strncpy(e.name, name.length() > 0 ? name.c_str() : "---", sizeof(e.name) - 1);
  e.name[sizeof(e.name) - 1] = 0;
  e.rssi = dev->getRSSI();
  e.spartan = hasSpartan;
  e.nus = hasNus;
}

static bool bleScanMatchesTarget(const NimBLEAdvertisedDevice* dev, String& addr, String& name) {
  addr = dev->getAddress().toString().c_str();
  addr.toLowerCase();
  name = dev->getName().c_str();
  String lname = name;
  lname.toLowerCase();
  if (g_bleConnMode == BLE_MODE_SPARTAN_HUB) {
    if (macEquals(addr, bleSavedMacForMode())) return true;
    return lname.indexOf("spartan") >= 0 ||
           macEquals(addr, SPARTAN_MAC) ||
           dev->isAdvertisingService(NimBLEUUID(SPARTAN_SVC));
  }
  // 123 direkt: nur konfigurierte Ziel-MAC (nicht jedes BM6 im Scan)
  return macEquals(addr, bleSavedMacForMode());
}

class SpartanScanCB : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    String addr, name;
    if (g_bleDiscoveryScan) {
      bleScanListAdd(dev);
      return;
    }
    if (!bleScanMatchesTarget(dev, addr, name)) return;
    g_bleHubName = name.length() > 0 ? name :
                   (g_bleConnMode == BLE_MODE_SPARTAN_HUB ? SPARTAN_NAME : "123");
    if (g_bleConnMode == BLE_MODE_SPARTAN_HUB) saveBleMacHub(addr.c_str());
    else saveBleMac123(addr.c_str(), dev->getAddress().getType());
    bleTarget    = dev->getAddress();
    g_ble123ScanNext = false;
    g_ble123AddrFlip = 0;
    bleDoConnect = true;
    NimBLEDevice::getScan()->stop();
    bleLogSetState("found");
    BLELOG("BLE: %s gefunden %s / %s",
           g_bleConnMode == BLE_MODE_SPARTAN_HUB ? "Hub" : "123",
           addr.c_str(), name.c_str());
  }
  void onScanEnd(const NimBLEScanResults& results, int) override {
    if (g_bleDiscoveryScan) {
      g_bleDiscoveryScan = false;
      bleDiscoveryScanEnded();
      bleLogSetState("discovery_done");
      BLELOG("BLE: Discovery %u Geraete", g_bleScanCount);
      return;
    }
    bleWifiSleepRestore();
    BLELOG("BLE: Scan Ende r=%d (%s)", results.getCount(), bleConnModeLabel());
    if (!g_bleConn && !bleDoConnect) {
      const uint32_t pause = (g_bleConnMode == BLE_MODE_DIRECT_123) ?
                             BLE_RECONNECT_123_MS : BLE_BG_SCAN_INTERVAL_MS;
      bleLogSetState("wait_reconnect");
      if (g_bleConnMode == BLE_MODE_DIRECT_123) {
        BLELOG("BLE: Scan ohne Treffer @ %s, Direct in %lums",
               bleSavedMacForMode(), (unsigned long)pause);
      } else {
        BLELOG("BLE: Reconnect geplant in %lums", (unsigned long)pause);
      }
      bleNextScanAt = millis() + pause;
    }
  }
};

static SpartanClientCB spartanClientCB;
static SpartanScanCB   spartanScanCB;

static bool bleTryConnectSavedMac() {
  if (g_bleConn || bleDoConnect || g_bleDiscoveryScan) return false;
  const char* macStr = bleSavedMacForMode();
  if (!macStr[0]) return false;
  if (g_bleConnMode == BLE_MODE_DIRECT_123) {
    // BM6/123: Typ aus Scan/NVS; bei Fehlschlag RANDOM<->PUBLIC wechseln
    uint8_t addrType = g_ble123AddrType;
    if (g_ble123AddrFlip & 1) {
      addrType = (addrType == BLE_ADDR_RANDOM) ? BLE_ADDR_PUBLIC : BLE_ADDR_RANDOM;
    }
    bleTarget = NimBLEAddress(std::string(macStr), addrType);
    bleLogSetState("connect_attempt");
    BLELOG("BLE: Direktconnect %s type=%u (%s)", macStr, addrType, bleConnModeLabel());
  } else {
    bleTarget = NimBLEAddress(std::string(macStr), BLE_ADDR_PUBLIC);
    bleLogSetState("connect_attempt");
    BLELOG("BLE: Direktconnect %s (%s)", macStr, bleConnModeLabel());
  }
  bleDoConnect = true;
  return true;
}

static void bleStartScan() {
  if (g_bleConn || bleDoConnect || g_bleDiscoveryScan) return;
  auto* s = NimBLEDevice::getScan();
  s->stop();
  bleWaitScanStopped(s, 300);
  bleOpScanBegin();
  s->setScanCallbacks(&spartanScanCB);
  s->setMaxResults(0xFF);
  s->setActiveScan(true);
  s->setInterval(200);
  s->setWindow(80);
  const uint32_t scanMs = (g_bleConnMode == BLE_MODE_DIRECT_123) ?
                          BLE_OP_SCAN_123_MS : BLE_DISCOVERY_SCAN_MS;
  bleLogSetState("scanning");
  BLELOG("BLE: Scan %lus (%s @ %s)",
         (unsigned long)(scanMs / 1000), bleConnModeLabel(), bleSavedMacForMode());
  s->start(scanMs, false, true);
}

static bool bleStartDiscoveryScan() {
  if (!g_featureBle || g_bleDiscoveryScan || g_bleSetupBusy || bleDoConnect) return false;
  bleDoConnect = false;
  g_bleScanCount = 0;
  auto* s = NimBLEDevice::getScan();
  if (bleClient && bleClient->isConnected()) {
    g_bleConn = false;
    bleClient->disconnect();
    uint32_t until = millis() + 1000;
    while (bleClient->isConnected() && millis() < until) {
      if (g_webStarted) webServer.handleClient();
      delay(10);
      yield();
    }
  }
  s->stop();
  bleWaitScanStopped(s, 500);
  s->clearResults();
  s->setScanCallbacks(&spartanScanCB, true);
  s->setActiveScan(true);
  s->setInterval(200);
  s->setWindow(80);
  s->setMaxResults(0);
  g_bleScanWifiSleepWasOn = (WiFi.getSleep() != WIFI_PS_NONE);
  WiFi.setSleep(WIFI_PS_NONE);
  g_bleDiscoveryScan = true;
  if (!s->start(BLE_DISCOVERY_SCAN_MS, false, true)) {
    g_bleDiscoveryScan = false;
    bleDiscoveryScanEnded();
    DLOGN("BLE: Discovery-Scan Start FAIL");
    return false;
  }
  DLOG("BLE: Discovery-Scan %ums...\n", BLE_DISCOVERY_SCAN_MS);
  return true;
}

static void ble123KickTick();
static void ble123PingTick();
static bool bleLinkUp();

static void bleIoYield() {
#if FEATURE_TOUCH
  bool tb = false;
  processTouchInput(&tb);
  if (g_redrawPage) {
    g_redrawPage = false;
    drawCurrentPage();
  }
#endif
  if (g_webStarted) webServerPoll(1);
  yield();
}

static void blePauseMs(uint32_t ms) {
  const uint32_t end = millis() + ms;
  while (millis() < end) {
    bleIoYield();
    delay(2);
  }
}

static void ble123KickTick() {
  if (g_ble123KickStage == BLE_123_KICK_IDLE) return;
  if (!g_nusRx || !bleLinkUp()) {
    g_ble123KickStage = BLE_123_KICK_IDLE;
    return;
  }
  const uint32_t now = millis();
  if (g_ble123KickStage == BLE_123_KICK_WAIT) {
    if (now < g_ble123KickAt) return;
    if (!g_nusRx->canWrite()) {
      BLELOG("BLE: 123 kick $ skip (no write)");
      g_ble123KickStage = BLE_123_KICK_IDLE;
      return;
    }
    const uint8_t dollar[] = {'$'};
    g_nusRx->writeValue(dollar, 1, false);
    BLELOG("BLE: 123 kick $ nach %lums (M5Dial)", (unsigned long)BLE_123_KICK_DELAY_MS);
    g_ble123KickAt = now + BLE_123_KICK_CR_MS;
    g_ble123KickStage = BLE_123_KICK_CR;
    return;
  }
  if (g_ble123KickStage == BLE_123_KICK_CR) {
    if (now < g_ble123KickAt) return;
    const uint8_t cr[] = {'\r'};
    g_nusRx->writeValue(cr, 1, false);
    BLELOGN("BLE: 123 kick CR (M5Dial)");
    g_ble123KickStage = BLE_123_KICK_IDLE;
    g_ble123PingAt = now;
  }
}

static void ble123PingTick() {
  if (!g_bleConn || g_bleConnMode != BLE_MODE_DIRECT_123 || !g_nusRx) return;
  if (g_ble123KickStage != BLE_123_KICK_IDLE) return;
  if (millis() - g_ble123PingAt < 2500) return;
  g_ble123PingAt = millis();
  if (g_nusRx->canWrite()) {
    const uint8_t dollar[] = {'$'};
    g_nusRx->writeValue(dollar, 1, false);
  }
}

static bool bleLinkUp() {
  return bleClient && bleClient->isConnected();
}

static void bleConnectBegin() {
  g_bleSetupBusy = true;
  bleOpScanBegin();
  auto* s = NimBLEDevice::getScan();
  s->stop();
  bleWaitScanStopped(s, 300);
}

static void bleConnectEnd() {
  g_bleSetupBusy = false;
  bleWifiSleepRestore();
#if FEATURE_TOUCH
  g_touchI2cFailStreak = 0;
#endif
}

static void bleConnect() {
  bleDoConnect = false;
  g_nusRx = nullptr;
  g_bleConn = false;
  if (!bleClient) {
    bleClient = NimBLEDevice::createClient();
    bleClient->setClientCallbacks(&spartanClientCB, false);
  }
  if (g_bleConnMode == BLE_MODE_DIRECT_123) {
    bleClient->setConnectionParams(BLE_123_CONN_ITVL, BLE_123_CONN_ITVL,
                                   BLE_123_CONN_LATENCY, BLE_123_CONN_TIMEOUT);
  }
  bleClient->setConnectTimeout(g_bleConnMode == BLE_MODE_SPARTAN_HUB ?
                               BLE_CONNECT_TIMEOUT_HUB_MS : BLE_CONNECT_TIMEOUT_MS);
  bleClient->setConnectRetries(1);
  bleConnectBegin();
  bleLogSetState("connecting");
  BLELOG("BLE: Connect %s type=%u (%s)...",
         bleTarget.toString().c_str(), bleTarget.getType(), bleConnModeLabel());

  bool ok = false;
  for (uint8_t attempt = 0; attempt < 2 && !ok; attempt++) {
    if (attempt > 0) {
      BLELOG("BLE: Connect retry %u", (unsigned)(attempt + 1));
      if (bleClient->isConnected()) bleClient->disconnect();
      blePauseMs(80);
    }
    if (!bleClient->connect(bleTarget, false, false, true)) {
      BLELOG("BLE: connect FAIL err=%d (%s)", bleClient->getLastError(), bleConnModeLabel());
      continue;
    }
    blePauseMs(120);
    if (!bleLinkUp()) {
      BLELOG("BLE: link down nach connect err=%d", bleClient->getLastError());
      continue;
    }
    if (!bleClient->discoverAttributes()) {
      BLELOG("BLE: discover FAIL err=%d", bleClient->getLastError());
      if (!bleLinkUp()) continue;
    }
    blePauseMs(50);
    if (!bleLinkUp()) {
      BLELOG("BLE: link down nach discover err=%d", bleClient->getLastError());
      continue;
    }

    if (g_bleConnMode == BLE_MODE_SPARTAN_HUB) {
      auto* svc = bleClient->getService(SPARTAN_SVC);
      if (!svc || !bleLinkUp()) continue;
      auto* status = svc->getCharacteristic(SPARTAN_STATUS);
      if (!status || !bleLinkUp()) continue;
      ok = status->subscribe(true, bleNotifyHubCB, true);
      BLELOG("BLE: Hub-Subscribe %s", ok ? "OK" : "FAIL");
      if (ok) {
        g_bleConn = true;
        bleLogSetState("connected");
      }
      break;
    }

    auto* svc = bleClient->getService(NUS_SVC);
    if (!svc || !bleLinkUp()) {
      BLELOG("BLE: kein NUS-Svc err=%d link=%d", bleClient->getLastError(), bleLinkUp());
      continue;
    }
    auto* tx = svc->getCharacteristic(NUS_TX);
    if (!tx || !bleLinkUp()) {
      BLELOG("BLE: kein NUS-TX err=%d", bleClient->getLastError());
      continue;
    }
    tx->subscribe(false, nullptr, false);
    blePauseMs(40);
    ok = tx->subscribe(true, bleNotify123CB, true);
    BLELOG("BLE: 123-Subscribe %s err=%d", ok ? "OK" : "FAIL", bleClient->getLastError());
    if (!ok || !bleLinkUp()) { ok = false; continue; }
    g_nusRx = svc->getCharacteristic(NUS_RX);
    g_ble123KickStage = BLE_123_KICK_WAIT;
    g_ble123KickAt = millis() + BLE_123_KICK_DELAY_MS;
    BLELOG("BLE: 123 kick geplant in %lums (M5Dial)", (unsigned long)BLE_123_KICK_DELAY_MS);
    g_bleConn = true;
    g_ble123ScanNext = false;
    g_ble123AddrFlip = 0;
    bleLogSetState("nus_ok");
    BLELOG("BLE: verbunden (%s)", bleConnModeLabel());
    bleLogSetState("connected");
    break;
  }

  bleConnectEnd();
  if (!ok) {
    bleLogSetState("connect_fail");
    BLELOG("BLE: Connect fehlgeschlagen %s type=%u err=%d",
           bleSavedMacForMode(), bleTarget.getType(), bleClient ? bleClient->getLastError() : -1);
    if (bleClient && bleClient->isConnected()) bleClient->disconnect();
    const uint32_t pause = (g_bleConnMode == BLE_MODE_DIRECT_123) ?
                           BLE_RECONNECT_123_MS : 2500;
    if (g_bleConnMode == BLE_MODE_DIRECT_123) {
      g_ble123ScanNext = true;
      g_ble123AddrFlip++;
    } else {
      g_bleHubScanNext = true;
    }
    BLELOG("BLE: Reconnect geplant in %lums%s", (unsigned long)pause,
           (g_bleConnMode == BLE_MODE_DIRECT_123 && g_ble123ScanNext) ? " (Scan)" :
           (g_bleConnMode == BLE_MODE_SPARTAN_HUB && g_bleHubScanNext) ? " (Scan)" : "");
    bleNextScanAt = millis() + pause;
  }
}

static const char* blePhaseLabel() {
  if (g_bleDiscoveryScan) return "discovery";
  if (g_bleSetupBusy || bleDoConnect) return "connecting";
  if (g_bleOpScanActive) return "scanning";
  if (bleNextScanAt != 0 && millis() < bleNextScanAt) return "wait_reconnect";
  if (g_bleConn) return "connected";
  return "idle";
}

static void bleLogPeriodicStatus() {
  if (!g_featureBle || g_bleConn) return;
  const uint32_t waitMs = (bleNextScanAt != 0 && millis() < bleNextScanAt) ?
                          (bleNextScanAt - millis()) : 0;
  BLELOG_STATUS("BLE: offline mode=%s mac=%s rx=%lu phase=%s wait=%lus",
                bleConnModeLabel(), bleSavedMacForMode(),
                (unsigned long)g_bleRxCnt, blePhaseLabel(),
                (unsigned long)(waitMs / 1000));
}

static void bleTick() {
  bleLogPeriodicStatus();
  if (g_bleSetupBusy) return;
  if (g_bleConn && g_bleConnMode == BLE_MODE_DIRECT_123) {
    ble123KickTick();
    ble123PingTick();
    return;
  }
  if (bleDoConnect) { bleConnect(); return; }
  if (g_bleConn || g_bleDiscoveryScan) return;
  if (bleNextScanAt == 0 || millis() < bleNextScanAt) return;
  bleNextScanAt = 0;
  // 123 direkt: Direct und Scan abwechseln (BM6 advertisiert selten)
  if (g_bleConnMode == BLE_MODE_DIRECT_123) {
    if (g_ble123ScanNext) {
      g_ble123ScanNext = false;
      bleStartScan();
    } else if (!bleTryConnectSavedMac()) {
      bleStartScan();
    }
    return;
  }
  // Hub: Scan und Direct abwechseln — Direct allein scheitert wenn Hub nicht advertisiert
  // (NimBLE stoppt Advertising nach erstem Client, z.B. M5 Dial).
  if (g_bleHubScanNext) {
    g_bleHubScanNext = false;
    bleStartScan();
  } else if (!bleTryConnectSavedMac()) {
    g_bleHubScanNext = true;
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
  hal_touch_reset(intHigh, PIN_TOUCH_INT);
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

static void gt911TouchRecoverIfNeeded() {
  if (g_touchI2cFailStreak < TOUCH_I2C_RECOVER_AFTER) return;
  if (g_bleSetupBusy) return;
  g_touchI2cFailStreak = 0;
  Serial.println("GT911: I2C recover re-init");
  gt911Init();
}

static bool readTouch(uint16_t *x, uint16_t *y) {
  if (!gt911Found) {
    if (g_touchI2cFailStreak < 255) g_touchI2cFailStreak++;
    gt911TouchRecoverIfNeeded();
    return false;
  }
  uint8_t status = 0;
  if (!i2cRegRead16(gt911Addr, GT911_READ_XY, &status, 1)) {
    uint8_t other = (gt911Addr == GT911_ADDR_PRIMARY) ? GT911_ADDR_ALT : GT911_ADDR_PRIMARY;
    if (!i2cRegRead16(other, GT911_READ_XY, &status, 1)) {
      if (g_touchI2cFailStreak < 255) g_touchI2cFailStreak++;
      gt911TouchRecoverIfNeeded();
      return false;
    }
    gt911Addr  = other;
    gt911Found = true;
  }
  g_touchI2cFailStreak = 0;
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
    if (g_touchI2cFailStreak < 255) g_touchI2cFailStreak++;
    gt911TouchRecoverIfNeeded();
    return false;
  }
  // GT911 ab 0x814F: [TrackID, X-low, X-high, Y-low, Y-high, Size-low, ...]
  // -> X = point[1]|point[2]<<8, Y = point[3]|point[4]<<8 (war um 1 Byte verschoben)
  uint16_t rawX = (uint16_t)point[1] | ((uint16_t)point[2] << 8);
  uint16_t rawY = (uint16_t)point[3] | ((uint16_t)point[4] << 8);
  if (g_rotationDeg == 0) {
    *x = rawX;
    *y = rawY;
  } else if (g_rotationDeg == 90) {
    int lx = (int)rawY;
    int ly = 479 - (int)rawX;
    if (lx < 0) lx = 0; else if (lx > 479) lx = 479;
    if (ly < 0) ly = 0; else if (ly > 479) ly = 479;
    *x = (uint16_t)lx;
    *y = (uint16_t)ly;
  } else if (g_rotationDeg == 180) {
    *x = (uint16_t)(479 - rawX);
    *y = (uint16_t)(479 - rawY);
  } else if (g_rotationDeg == 270) {
    int lx = 479 - (int)rawY;
    int ly = (int)rawX;
    if (lx < 0) lx = 0; else if (lx > 479) lx = 479;
    if (ly < 0) ly = 0; else if (ly > 479) ly = 479;
    *x = (uint16_t)lx;
    *y = (uint16_t)ly;
  } else {
    float dx = (float)rawX - ROT_PIVOT;
    float dy = (float)rawY - ROT_PIVOT;
    int lx = (int)lroundf(dx * g_rotCos + dy * g_rotSin + ROT_PIVOT);
    int ly = (int)lroundf(-dx * g_rotSin + dy * g_rotCos + ROT_PIVOT);
    if (lx < 0) lx = 0; else if (lx > 479) lx = 479;
    if (ly < 0) ly = 0; else if (ly > 479) ly = 479;
    *x = (uint16_t)lx;
    *y = (uint16_t)ly;
  }
  g_lastTouchX  = *x;
  g_lastTouchY  = *y;
  g_lastTouchMs = millis();
  static uint32_t s_touchLogMs = 0;
#if FEATURE_VERBOSE_SERIAL
  if (g_lastTouchMs - s_touchLogMs >= LOG_TOUCH_MS) {
    s_touchLogMs = g_lastTouchMs;
    Serial.printf("[TOUCH] raw=%u,%u log=%u,%u rot=%d pts=%u st=0x%02X\n",
                  rawX, rawY, *x, *y, g_rotationDeg, (unsigned)points, status);
  }
#endif
  return true;
}

// ---- Display helpers (all writes go through HAL) ----
static bool ensureFrame()         { return hal_fb() != nullptr; }
static void presentFrame()        { hal_present(); }
static void fillFrame(uint16_t c) { hal_fill(c); }

static void setPixelFb(int x, int y, uint16_t color) {
  uint16_t *fb = hal_fb();
  if (!fb || (unsigned)x >= 480 || (unsigned)y >= 480) return;
  fb[y * 480 + x] = color;
}

static void logicalToDisplay(int lx, int ly, int *dx, int *dy);

static void setPixel(int x, int y, uint16_t color) {
  if (g_rotationDeg != 0) {
    logicalToDisplay(x, y, &x, &y);
    if ((unsigned)x >= 480 || (unsigned)y >= 480) return;
  }
  setPixelFb(x, y, color);
}

static void fillRectFast(int x, int y, int w, int h, uint16_t color) {
  if (g_rotationDeg == 0) {
    for (int yy = y; yy < y + h; yy++)
      for (int xx = x; xx < x + w; xx++)
        setPixelFb(xx, yy, color);
    return;
  }
  if (g_rotationDeg == 90 || g_rotationDeg == 180 || g_rotationDeg == 270) {
    for (int ly = y; ly < y + h; ly++) {
      for (int lx = x; lx < x + w; lx++) {
        int dx, dy;
        logicalToDisplay(lx, ly, &dx, &dy);
        setPixelFb(dx, dy, color);
      }
    }
    return;
  }
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

// VDO tach arc: 4 @ ~7 o'clock, 40 @ 12 o'clock, 80 @ ~5 o'clock (270° CW sweep).
static uint16_t vdoFaceColor() { return g_nightMode ? RGB565(33, 37, 33) : DIAL_FACE_BG; }
static uint16_t vdoMarkColor() { return g_nightMode ? RGB565(202, 222, 202) : RGB565(226, 211, 181); }
static uint16_t vdoDimMarkColor() { return g_nightMode ? RGB565(112, 126, 108) : RGB565(154, 143, 124); }
static uint16_t vdoNeedleColor() { return g_nightMode ? RGB565(232, 185, 55) : RGB565(232, 168, 0); }
static uint16_t vdoWarnColor() { return g_nightMode ? RGB565(166, 74, 28) : RGB565(184, 66, 26); }
static uint16_t vdoDarkRimColor() { return g_nightMode ? RGB565(11, 12, 11) : RGB565(16, 15, 14); }
static uint16_t vdoHubColor() { return g_nightMode ? RGB565(35, 37, 34) : RGB565(40, 38, 35); }

static float degToRad(float deg) {
  return deg * PI / 180.0f;
}

static float gaugeValueAngle(float value, float vMin, float vMax, float degStart, float degEnd) {
  if (value < vMin) value = vMin;
  if (value > vMax) value = vMax;
  return degToRad(degStart + (value - vMin) / (vMax - vMin) * (degEnd - degStart));
}

static float rpmScaleMaxValue() {
  return (float)g_rpmScaleMax / 100.0f;
}

static float rpmDialFrac(float scaleVal) {
  const float maxValue = rpmScaleMaxValue();
  if (scaleVal <= (float)RPM_SCALE_MIN_VALUE) return 7.0f / 12.0f;
  if (scaleVal >= maxValue) return 5.0f / 12.0f;
  float d;
  if (scaleVal <= 40.0f)
    d = (scaleVal - (float)RPM_SCALE_MIN_VALUE) / (40.0f - (float)RPM_SCALE_MIN_VALUE) * (5.0f / 12.0f);
  else
    d = 5.0f / 12.0f + (scaleVal - 40.0f) / (maxValue - 40.0f) * (5.0f / 12.0f);
  float frac = 7.0f / 12.0f + d;
  if (frac >= 1.0f) frac -= 1.0f;
  return frac;
}

static float rpmScaleAngle(float scaleVal) {
  return rpmDialFrac(scaleVal) * 2.0f * PI - PI / 2.0f;
}

static float rpmScaleAtDialFrac(float frac) {
  const float maxValue = rpmScaleMaxValue();
  float d = frac - 7.0f / 12.0f;
  if (d < 0.0f) d += 1.0f;
  const float arcLen = 10.0f / 12.0f;
  if (d > arcLen + 0.02f) return -1.0f;
  if (d <= 5.0f / 12.0f)
    return (float)RPM_SCALE_MIN_VALUE + d / (5.0f / 12.0f) * (40.0f - (float)RPM_SCALE_MIN_VALUE);
  return 40.0f + (d - 5.0f / 12.0f) / (5.0f / 12.0f) * (maxValue - 40.0f);
}

static void drawArcRing(int cx, int cy, int radius, int thickness,
                        float vStart, float vEnd, uint16_t color) {
  if (vEnd <= vStart) return;
  const int outer = radius * radius;
  const int innerR = radius - thickness;
  const int inner = innerR > 0 ? innerR * innerR : 0;
  for (int y = cy - radius; y <= cy + radius; y++) {
    for (int x = cx - radius; x <= cx + radius; x++) {
      const int dx = x - cx, dy = y - cy;
      const int d = dx * dx + dy * dy;
      if (d > outer || d < inner) continue;
      float ang = atan2f((float)dy, (float)dx) + PI / 2.0f;
      if (ang < 0.0f) ang += 2.0f * PI;
      const float val = rpmScaleAtDialFrac(ang / (2.0f * PI));
      if (val < 0.0f) continue;
      if (val >= vStart - 0.05f && val <= vEnd + 0.05f) setPixel(x, y, color);
    }
  }
}

static void drawArcRingLinear(int cx, int cy, int radius, int thickness,
                              float vStart, float vEnd, float vMax, uint16_t color) {
  if (vEnd <= vStart || vMax <= 0.0f) return;
  const int outer = radius * radius;
  const int innerR = radius - thickness;
  const int inner = innerR > 0 ? innerR * innerR : 0;
  for (int y = cy - radius; y <= cy + radius; y++) {
    for (int x = cx - radius; x <= cx + radius; x++) {
      const int dx = x - cx, dy = y - cy;
      const int d = dx * dx + dy * dy;
      if (d > outer || d < inner) continue;
      float ang = atan2f((float)dy, (float)dx) + PI / 2.0f;
      if (ang < 0.0f) ang += 2.0f * PI;
      const float val = ang * vMax / (2.0f * PI);
      if (val >= vStart - 0.02f && val <= vEnd + 0.02f) setPixel(x, y, color);
    }
  }
}

static float rpmScaleValue(float rpm) {
  float v = rpm / 100.0f;
  if (v < (float)RPM_SCALE_MIN_VALUE) v = (float)RPM_SCALE_MIN_VALUE;
  if (v > rpmScaleMaxValue()) v = rpmScaleMaxValue();
  return v;
}

// Mercedes tach: 0 @ ~7 o'clock, 35 @ 12 o'clock, 70 @ ~5 o'clock.
static float mbDialFrac(float scaleVal) {
  if (scaleVal <= 0.0f) return 7.0f / 12.0f;
  if (scaleVal >= (float)MB_SCALE_MAX_VALUE) return 5.0f / 12.0f;
  float d;
  if (scaleVal <= MB_SCALE_MID_VALUE)
    d = scaleVal / MB_SCALE_MID_VALUE * (5.0f / 12.0f);
  else
    d = 5.0f / 12.0f
        + (scaleVal - MB_SCALE_MID_VALUE) / ((float)MB_SCALE_MAX_VALUE - MB_SCALE_MID_VALUE)
          * (5.0f / 12.0f);
  float frac = 7.0f / 12.0f + d;
  if (frac >= 1.0f) frac -= 1.0f;
  return frac;
}

static float mbScaleAngle(float scaleVal) {
  return mbDialFrac(scaleVal) * 2.0f * PI - PI / 2.0f;
}

static float mbScaleAtDialFrac(float frac) {
  float d = frac - 7.0f / 12.0f;
  if (d < 0.0f) d += 1.0f;
  const float arcLen = 10.0f / 12.0f;
  if (d > arcLen + 0.02f) return -1.0f;
  if (d <= 5.0f / 12.0f)
    return d / (5.0f / 12.0f) * MB_SCALE_MID_VALUE;
  return MB_SCALE_MID_VALUE
         + (d - 5.0f / 12.0f) / (5.0f / 12.0f) * ((float)MB_SCALE_MAX_VALUE - MB_SCALE_MID_VALUE);
}

static void drawMbArcRing(int cx, int cy, int radius, int thickness,
                          float vStart, float vEnd, uint16_t color) {
  if (vEnd <= vStart) return;
  const int outer = radius * radius;
  const int innerR = radius - thickness;
  const int inner = innerR > 0 ? innerR * innerR : 0;
  for (int y = cy - radius; y <= cy + radius; y++) {
    for (int x = cx - radius; x <= cx + radius; x++) {
      const int dx = x - cx, dy = y - cy;
      const int d = dx * dx + dy * dy;
      if (d > outer || d < inner) continue;
      float ang = atan2f((float)dy, (float)dx) + PI / 2.0f;
      if (ang < 0.0f) ang += 2.0f * PI;
      const float val = mbScaleAtDialFrac(ang / (2.0f * PI));
      if (val < 0.0f) continue;
      if (val >= vStart - 0.05f && val <= vEnd + 0.05f) setPixel(x, y, color);
    }
  }
}

static float mbScaleValue(float rpm) {
  float v = rpm / 100.0f;
  if (v < 0.0f) v = 0.0f;
  if (v > (float)MB_SCALE_MAX_VALUE) v = (float)MB_SCALE_MAX_VALUE;
  return v;
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

static void drawTextSmall(int x, int y, const char *text, uint16_t color, int scale);
static int textWidthSmall(const char *text, int scale);

static void drawGaugeArcRing(int cx, int cy, int radius, int thickness,
                             float degStart, float degEnd, uint16_t color) {
  if (degEnd <= degStart) return;
  const int outer = radius * radius;
  const int innerR = radius - thickness;
  const int inner = innerR > 0 ? innerR * innerR : 0;
  for (int y = cy - radius; y <= cy + radius; y++) {
    for (int x = cx - radius; x <= cx + radius; x++) {
      const int dx = x - cx, dy = y - cy;
      const int d = dx * dx + dy * dy;
      if (d > outer || d < inner) continue;
      float deg = atan2f((float)dy, (float)dx) * 180.0f / PI;
      if (deg < 0.0f) deg += 360.0f;
      if (deg >= degStart - 0.5f && deg <= degEnd + 0.5f) setPixel(x, y, color);
    }
  }
}

static void drawGaugeTick(int cx, int cy, int rOuter, int rInner,
                          float value, float vMin, float vMax,
                          float degStart, float degEnd, uint16_t color, int thick) {
  const float angle = gaugeValueAngle(value, vMin, vMax, degStart, degEnd);
  const int x0 = cx + (int)lroundf(cosf(angle) * (float)rInner);
  const int y0 = cy + (int)lroundf(sinf(angle) * (float)rInner);
  const int x1 = cx + (int)lroundf(cosf(angle) * (float)rOuter);
  const int y1 = cy + (int)lroundf(sinf(angle) * (float)rOuter);
  drawLineFast(x0, y0, x1, y1, color, thick);
}

static void drawGaugeLabel(int cx, int cy, int radius, float value, float vMin, float vMax,
                           float degStart, float degEnd, const char* label,
                           uint16_t color, int scale) {
  const float angle = gaugeValueAngle(value, vMin, vMax, degStart, degEnd);
  const int lx = cx + (int)lroundf(cosf(angle) * (float)radius) - textWidthSmall(label, scale) / 2;
  const int ly = cy + (int)lroundf(sinf(angle) * (float)radius) - (7 * scale) / 2;
  drawTextSmall(lx, ly, label, color, scale);
}

static void drawGaugeNeedle(int cx, int cy, float value, float vMin, float vMax,
                            float degStart, float degEnd, int length, int thickness,
                            uint16_t color) {
  const float angle = gaugeValueAngle(value, vMin, vMax, degStart, degEnd);
  const int x1 = cx + (int)lroundf(cosf(angle) * (float)length);
  const int y1 = cy + (int)lroundf(sinf(angle) * (float)length);
  drawLineFast(cx, cy, x1, y1, color, thickness);
}

static void drawMbTick(int cx, int cy, int rOuter, int rInner, float scaleVal, uint16_t color, int thick) {
  const float angle = mbScaleAngle(scaleVal);
  const int x0 = cx + (int)lroundf(cosf(angle) * (float)rInner);
  const int y0 = cy + (int)lroundf(sinf(angle) * (float)rInner);
  const int x1 = cx + (int)lroundf(cosf(angle) * (float)rOuter);
  const int y1 = cy + (int)lroundf(sinf(angle) * (float)rOuter);
  drawLineFast(x0, y0, x1, y1, color, thick);
}

static uint16_t faceColorAtFb(int px, int py) {
  if ((unsigned)px >= 480 || (unsigned)py >= 480) return vdoFaceColor();
  int pct = g_dialScalePct;
  if (pct < DIAL_SCALE_MIN) pct = DIAL_SCALE_MIN;
  if (pct > DIAL_SCALE_MAX) pct = DIAL_SCALE_MAX;
  if (pct == 100 && g_rotationDeg == 0 && g_dialCenterOffsetX == 0 && g_dialCenterOffsetY == 0) {
    return pgm_read_word(&VDO_DIAL_480_RGB565[py * 480 + px]);
  }
  if (g_dialCache && dialCacheMatches(pct, g_rotationDeg, g_dialCenterOffsetX, g_dialCenterOffsetY)
      && !g_dialRebuilding) {
    return g_dialCache[py * 480 + px];
  }
  return vdoFaceColor();
}

static bool clockFaceSourceReady() {
  int pct = g_dialScalePct;
  if (pct < DIAL_SCALE_MIN) pct = DIAL_SCALE_MIN;
  if (pct > DIAL_SCALE_MAX) pct = DIAL_SCALE_MAX;
  if (pct == 100 && g_rotationDeg == 0 && g_dialCenterOffsetX == 0 && g_dialCenterOffsetY == 0) {
    return true;
  }
  return g_dialCache && dialCacheMatches(pct, g_rotationDeg, g_dialCenterOffsetX, g_dialCenterOffsetY)
      && !g_dialRebuilding;
}

static void paintHand(float value, float maxValue, int length, int thickness, uint16_t color,
                      bool restoreFromFace) {
  float angle = (value / maxValue) * 2.0f * PI - PI / 2.0f;
  const int lx1 = 240 + (int)lroundf(cosf(angle) * (float)length);
  const int ly1 = 240 + (int)lroundf(sinf(angle) * (float)length);
  const int r = thickness / 2;
  int x0 = 240, y0 = 240, x1 = lx1, y1 = ly1;
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  while (true) {
    for (int oy = -r; oy <= r; oy++) {
      for (int ox = -r; ox <= r; ox++) {
        if (ox * ox + oy * oy > r * r) continue;
        int lx = x0 + ox, ly = y0 + oy;
        int px = lx, py = ly;
        if (g_rotationDeg != 0) logicalToDisplay(lx, ly, &px, &py);
        const uint16_t c = restoreFromFace ? faceColorAtFb(px, py) : color;
        setPixelFb(px, py, c);
      }
    }
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

static void drawHand(float value, float maxValue, int length, int thickness, uint16_t color) {
  paintHand(value, maxValue, length, thickness, color, false);
}

static void drawHandAt(int cx, int cy, float value, float maxValue, int length, int thickness, uint16_t color) {
  const float angle = (value / maxValue) * 2.0f * PI - PI / 2.0f;
  const int x1 = cx + (int)lroundf(cosf(angle) * (float)length);
  const int y1 = cy + (int)lroundf(sinf(angle) * (float)length);
  drawLineFast(cx, cy, x1, y1, color, thickness);
}

static void drawHandAngle(float angle, int length, int thickness, uint16_t color) {
  const int lx1 = 240 + (int)lroundf(cosf(angle) * (float)length);
  const int ly1 = 240 + (int)lroundf(sinf(angle) * (float)length);
  drawLineFast(240, 240, lx1, ly1, color, thickness);
}

static void drawVdoNeedle(int cx, int cy, float angle, int length, uint16_t color) {
  const float ca = cosf(angle);
  const float sa = sinf(angle);
  const float px = -sa;
  const float py = ca;
  const int tipX = cx + (int)lroundf(ca * (float)length);
  const int tipY = cy + (int)lroundf(sa * (float)length);
  const int tailX = cx - (int)lroundf(ca * 22.0f);
  const int tailY = cy - (int)lroundf(sa * 22.0f);
  const int baseLx = cx + (int)lroundf(px * 7.0f);
  const int baseLy = cy + (int)lroundf(py * 7.0f);
  const int baseRx = cx - (int)lroundf(px * 7.0f);
  const int baseRy = cy - (int)lroundf(py * 7.0f);
  drawLineFast(baseLx, baseLy, tipX, tipY, color, 4);
  drawLineFast(baseRx, baseRy, tipX, tipY, color, 4);
  drawLineFast(tailX, tailY, cx, cy, color, 4);
  drawLineFast(cx + (int)lroundf(px * 2.0f), cy + (int)lroundf(py * 2.0f),
               tipX - (int)lroundf(ca * 22.0f), tipY - (int)lroundf(sa * 22.0f),
               RGB565(250, 200, 45), 1);
}

static void restoreHand(float value, float maxValue, int length, int thickness) {
  paintHand(value, maxValue, length, thickness, 0, true);
}

static void restoreCircleFromFace(int cx, int cy, int radius) {
  int r2 = radius * radius;
  for (int y = cy - radius; y <= cy + radius; y++) {
    for (int x = cx - radius; x <= cx + radius; x++) {
      int dx = x - cx, dy = y - cy;
      if (dx * dx + dy * dy > r2) continue;
      int px = x, py = y;
      if (g_rotationDeg != 0) logicalToDisplay(x, y, &px, &py);
      setPixelFb(px, py, faceColorAtFb(px, py));
    }
  }
}

static uint8_t glyphColumn(char c, uint8_t col) {
  static const uint8_t blank[5] = {0,0,0,0,0};
  const uint8_t *g = blank;
  switch (c) {
    case 'A': { static const uint8_t v[5]={0x7E,0x11,0x11,0x11,0x7E}; g=v; break; }
    case 'B': { static const uint8_t v[5]={0x7F,0x49,0x49,0x49,0x36}; g=v; break; }
    case 'D': { static const uint8_t v[5]={0x7F,0x41,0x41,0x22,0x1C}; g=v; break; }
    case 'E': { static const uint8_t v[5]={0x7F,0x49,0x49,0x49,0x41}; g=v; break; }
    case 'G': { static const uint8_t v[5]={0x3E,0x41,0x49,0x49,0x7A}; g=v; break; }
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
    case 'x': { static const uint8_t v[5]={0x63,0x14,0x08,0x14,0x63}; g=v; break; }
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

static void drawRpmTick(int cx, int cy, int rOuter, int rInner, float scaleVal, uint16_t color) {
  const float angle = rpmScaleAngle(scaleVal);
  const int x0 = cx + (int)lroundf(cosf(angle) * (float)rInner);
  const int y0 = cy + (int)lroundf(sinf(angle) * (float)rInner);
  const int x1 = cx + (int)lroundf(cosf(angle) * (float)rOuter);
  const int y1 = cy + (int)lroundf(sinf(angle) * (float)rOuter);
  drawLineFast(x0, y0, x1, y1, color, 3);
}

static void drawRpmTickLinear(int cx, int cy, int rOuter, int rInner, float scaleVal,
                              float vMax, uint16_t color) {
  const float angle = (scaleVal / vMax) * 2.0f * PI - PI / 2.0f;
  const int x0 = cx + (int)lroundf(cosf(angle) * (float)rInner);
  const int y0 = cy + (int)lroundf(sinf(angle) * (float)rInner);
  const int x1 = cx + (int)lroundf(cosf(angle) * (float)rOuter);
  const int y1 = cy + (int)lroundf(sinf(angle) * (float)rOuter);
  drawLineFast(x0, y0, x1, y1, color, 3);
}

static void drawRpmScaleLabel(int cx, int cy, int radius, float scaleVal, const char* label) {
  const float angle = rpmScaleAngle(scaleVal);
  const int lx = cx + (int)lroundf(cosf(angle) * (float)radius) - textWidthSmall(label, 2) / 2;
  const int ly = cy + (int)lroundf(sinf(angle) * (float)radius) - 7;
  drawTextSmall(lx, ly, label, RGB565(235, 235, 225), 2);
}

static void drawRpmScaleLabelLinear(int cx, int cy, int radius, float scaleVal, float vMax,
                                    const char* label) {
  const float angle = (scaleVal / vMax) * 2.0f * PI - PI / 2.0f;
  const int lx = cx + (int)lroundf(cosf(angle) * (float)radius) - textWidthSmall(label, 2) / 2;
  const int ly = cy + (int)lroundf(sinf(angle) * (float)radius) - 7;
  drawTextSmall(lx, ly, label, RGB565(235, 235, 225), 2);
}

static void drawMbScaleLabel(int cx, int cy, int radius, float scaleVal, const char* label) {
  const float angle = mbScaleAngle(scaleVal);
  const int lx = cx + (int)lroundf(cosf(angle) * (float)radius) - textWidthSmall(label, 2) / 2;
  const int ly = cy + (int)lroundf(sinf(angle) * (float)radius) - 7;
  drawTextSmall(lx, ly, label, RGB565(235, 235, 225), 2);
}

static void drawMercedesSubClock() {
  const int cx = MB_CLOCK_CX, cy = MB_CLOCK_CY, r = MB_CLOCK_R;
  const uint16_t white = vdoMarkColor();
  const uint16_t yellow = vdoNeedleColor();
  for (int h = 0; h < 12; h++) {
    const float angle = ((float)h / 12.0f) * 2.0f * PI - PI / 2.0f;
    const int thick = (h % 3 == 0) ? 3 : 2;
    const int rOut = (h % 3 == 0) ? r - 4 : r - 10;
    const int x0 = cx + (int)lroundf(cosf(angle) * (float)(r - 18));
    const int y0 = cy + (int)lroundf(sinf(angle) * (float)(r - 18));
    const int x1 = cx + (int)lroundf(cosf(angle) * (float)rOut);
    const int y1 = cy + (int)lroundf(sinf(angle) * (float)rOut);
    drawLineFast(x0, y0, x1, y1, white, thick);
  }
  drawTextCentered(cx, cy - 30, "12", white, 2);
  drawTextCentered(cx + 30, cy, "3", white, 2);
  drawTextCentered(cx, cy + 30, "6", white, 2);
  drawTextCentered(cx - 30, cy, "9", white, 2);
  struct tm now = {};
  if (!readClockTime(&now)) return;
  const float minuteValue = now.tm_min + now.tm_sec / 60.0f;
  const float hourValue   = (now.tm_hour % 12) + minuteValue / 60.0f;
  drawHandAt(cx, cy, hourValue,   12.0f, 20, 4, yellow);
  drawHandAt(cx, cy, minuteValue, 60.0f, 30, 3, yellow);
  fillCircleFast(cx, cy, 7, vdoHubColor());
  fillCircleFast(cx, cy, 4, white);
}

static void drawVdoSubClockAt(int cx, int cy, int r) {
  const uint16_t cream = vdoMarkColor();
  const uint16_t dim = vdoDimMarkColor();
  const uint16_t yellow = vdoNeedleColor();
  drawCircleLine(cx, cy, r + 8, 2, vdoDarkRimColor());
  drawCircleLine(cx, cy, r, 1, dim);
  for (int h = 0; h < 12; h++) {
    const float angle = ((float)h / 12.0f) * 2.0f * PI - PI / 2.0f;
    const bool major = (h % 3) == 0;
    const int rOut = r - 4;
    const int rIn = r - (major ? 18 : 12);
    drawLineFast(cx + (int)lroundf(cosf(angle) * (float)rIn),
                 cy + (int)lroundf(sinf(angle) * (float)rIn),
                 cx + (int)lroundf(cosf(angle) * (float)rOut),
                 cy + (int)lroundf(sinf(angle) * (float)rOut),
                 cream, major ? 3 : 2);
  }
  drawTextCentered(cx, cy - 26, "12", cream, 2);
  drawTextCentered(cx + 26, cy - 2, "3", cream, 2);
  drawTextCentered(cx, cy + 22, "6", cream, 2);
  drawTextCentered(cx - 26, cy - 2, "9", cream, 2);
  struct tm now = {};
  if (readClockTime(&now)) {
    const float minuteValue = now.tm_min + now.tm_sec / 60.0f;
    const float hourValue = (now.tm_hour % 12) + minuteValue / 60.0f;
    drawHandAt(cx, cy, hourValue, 12.0f, r - 25, 4, yellow);
    drawHandAt(cx, cy, minuteValue, 60.0f, r - 14, 3, yellow);
  }
  fillCircleFast(cx, cy, 7, vdoHubColor());
  fillCircleFast(cx, cy, 3, cream);
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

// Pre-scaled dial in PSRAM: bilinear rebuild on setting change, fast memcpy each frame.
static int       g_dialCacheScale = -1;
static int       g_dialCacheRot   = -1;
static int       g_dialCacheOffX  = -999;
static int       g_dialCacheOffY  = -999;
static int       g_dialRebuildRow = 0;
static int       g_dialRebuildPct = -1;
static int       g_dialRebuildRot = -1;
static int       g_dialRebuildOffX = 0;
static int       g_dialRebuildOffY = 0;

static inline uint16_t lerpRgb565(uint16_t a, uint16_t b, uint8_t t) {
  int ar = (a >> 11) & 0x1F, ag = (a >> 5) & 0x3F, ab = a & 0x1F;
  int br = (b >> 11) & 0x1F, bg = (b >> 5) & 0x3F, bb = b & 0x1F;
  int r = ar + ((br - ar) * t >> 8);
  int g = ag + ((bg - ag) * t >> 8);
  int bc = ab + ((bb - ab) * t >> 8);
  return (uint16_t)((r << 11) | (g << 5) | bc);
}

static inline uint16_t bilinearRgb565(uint16_t c00, uint16_t c10, uint16_t c01, uint16_t c11,
                                      uint8_t fx, uint8_t fy) {
  return lerpRgb565(lerpRgb565(c00, c10, fx), lerpRgb565(c01, c11, fx), fy);
}

static void invalidateDialCache() {
  g_dialCacheScale = -1;
  g_dialCacheRot   = -1;
  g_dialRebuilding = false;
  g_dialRebuildRow = 0;
}

static bool dialCacheMatches(int pct, int rot, int offX, int offY) {
  return g_dialCache && g_dialCacheScale == pct && g_dialCacheRot == rot
      && g_dialCacheOffX == offX && g_dialCacheOffY == offY;
}

static bool dialCacheAlloc() {
  if (g_dialCache) return true;
  g_dialCache = (uint16_t*)ps_malloc(480 * 480 * sizeof(uint16_t));
  if (!g_dialCache) {
    Serial.println("dial cache: PSRAM alloc failed");
    return false;
  }
  return true;
}

static void dialCacheBeginRebuild(int pct, int rot, int offX, int offY) {
  if (!dialCacheAlloc()) return;
  g_dialRebuildPct = pct;
  g_dialRebuildRot = rot;
  g_dialRebuildOffX = offX;
  g_dialRebuildOffY = offY;
  g_dialRebuildRow = 0;
  g_dialRebuilding = true;
  g_dialCacheScale = -1;
  for (int i = 0; i < 480 * 480; i++) g_dialCache[i] = vdoFaceColor();
}

static void dialCacheRebuildRows(int pct, int rot, int offX, int offY, int rowStart, int rowEnd) {
  const float pivot = ROT_PIVOT;
  const float invScale = 100.0f / (float)pct;
  float rotSin = 0.0f, rotCos = 1.0f;
  if (rot != 0) {
    const float rad = (float)rot * PI / 180.0f;
    rotSin = sinf(rad);
    rotCos = cosf(rad);
  }
  for (int py = rowStart; py < rowEnd; py++) {
    for (int px = 0; px < 480; px++) {
      float dx = (float)px - pivot;
      float dy = (float)py - pivot;
      float lx = dx;
      float ly = dy;
      if (rot != 0) {
        lx = dx * rotCos + dy * rotSin;
        ly = -dx * rotSin + dy * rotCos;
      }
      float sx = pivot + lx * invScale + (float)offX;
      float sy = pivot + ly * invScale + (float)offY;
      if (sx < 0.0f || sy < 0.0f || sx > 479.0f || sy > 479.0f) continue;

      int x0 = (int)sx;
      int y0 = (int)sy;
      int x1 = x0 + 1;
      int y1 = y0 + 1;
      if (x1 >= 480) x1 = 479;
      if (y1 >= 480) y1 = 479;
      uint8_t fx = (uint8_t)((sx - (float)x0) * 255.0f);
      uint8_t fy = (uint8_t)((sy - (float)y0) * 255.0f);

      uint16_t c00 = pgm_read_word(&VDO_DIAL_480_RGB565[y0 * 480 + x0]);
      uint16_t c10 = pgm_read_word(&VDO_DIAL_480_RGB565[y0 * 480 + x1]);
      uint16_t c01 = pgm_read_word(&VDO_DIAL_480_RGB565[y1 * 480 + x0]);
      uint16_t c11 = pgm_read_word(&VDO_DIAL_480_RGB565[y1 * 480 + x1]);
      g_dialCache[py * 480 + px] = bilinearRgb565(c00, c10, c01, c11, fx, fy);
    }
  }
}

// Inkrementeller Rebuild: blockiert Touch/Display nicht mehr fuer mehrere Sekunden.
static void tickDialCacheRebuild() {
  int pct = g_dialScalePct;
  if (pct < DIAL_SCALE_MIN) pct = DIAL_SCALE_MIN;
  if (pct > DIAL_SCALE_MAX) pct = DIAL_SCALE_MAX;
  if (pct == 100 && g_rotationDeg == 0 && g_dialCenterOffsetX == 0 && g_dialCenterOffsetY == 0) {
    g_dialRebuilding = false;
    return;
  }
  if (dialCacheMatches(pct, g_rotationDeg, g_dialCenterOffsetX, g_dialCenterOffsetY)) {
    g_dialRebuilding = false;
    return;
  }
  if (!g_dialRebuilding
      || g_dialRebuildPct != pct || g_dialRebuildRot != g_rotationDeg
      || g_dialRebuildOffX != g_dialCenterOffsetX || g_dialRebuildOffY != g_dialCenterOffsetY) {
    dialCacheBeginRebuild(pct, g_rotationDeg, g_dialCenterOffsetX, g_dialCenterOffsetY);
  }
  if (!g_dialRebuilding || !g_dialCache) return;

  const int rowEnd = g_dialRebuildRow + DIAL_REBUILD_ROWS;
  const int clampEnd = rowEnd > 480 ? 480 : rowEnd;
  dialCacheRebuildRows(g_dialRebuildPct, g_dialRebuildRot, g_dialRebuildOffX, g_dialRebuildOffY,
                       g_dialRebuildRow, clampEnd);
  g_dialRebuildRow = clampEnd;
  if (g_dialRebuildRow >= 480) {
    g_dialCacheScale = g_dialRebuildPct;
    g_dialCacheRot   = g_dialRebuildRot;
    g_dialCacheOffX  = g_dialRebuildOffX;
    g_dialCacheOffY  = g_dialRebuildOffY;
    g_dialRebuilding = false;
  }
}

static void finishDialCacheRebuildBlocking() {
  while (true) {
    tickDialCacheRebuild();
    int pct = g_dialScalePct;
    if (pct < DIAL_SCALE_MIN) pct = DIAL_SCALE_MIN;
    if (pct > DIAL_SCALE_MAX) pct = DIAL_SCALE_MAX;
    if (pct == 100 && g_rotationDeg == 0 && g_dialCenterOffsetX == 0 && g_dialCenterOffsetY == 0) break;
    if (dialCacheMatches(pct, g_rotationDeg, g_dialCenterOffsetX, g_dialCenterOffsetY)) break;
    bleIoYield();
  }
}

static bool      g_clockHandsValid = false;
static float     g_clockLastSecVal = -1.0f;
static float     g_clockLastMinVal = -1.0f;
static float     g_clockLastHourVal = -1.0f;
static uint8_t   g_clockFacePage = 255;

static void invalidateClockHands() {
  g_clockHandsValid = false;
  g_clockLastSecVal = g_clockLastMinVal = g_clockLastHourVal = -1.0f;
}

static void requestDialCacheRebuild() {
  invalidateDialCache();
  invalidateClockHands();
}

static void copyVdoDialToFrame() {
  uint16_t *fb = hal_fb();
  if (!fb) return;
  int pct = g_dialScalePct;
  if (pct < DIAL_SCALE_MIN) pct = DIAL_SCALE_MIN;
  if (pct > DIAL_SCALE_MAX) pct = DIAL_SCALE_MAX;
  if (pct == 100 && g_rotationDeg == 0 && g_dialCenterOffsetX == 0 && g_dialCenterOffsetY == 0) {
    for (int i = 0; i < 480 * 480; i++) fb[i] = pgm_read_word(&VDO_DIAL_480_RGB565[i]);
    return;
  }

  // Fertiger oder teilweise aufgebauter Cache: memcpy statt teurem NN-Fallback pro Frame.
  if (g_dialCache && (dialCacheMatches(pct, g_rotationDeg, g_dialCenterOffsetX, g_dialCenterOffsetY)
      || g_dialRebuilding)) {
    memcpy(fb, g_dialCache, 480 * 480 * sizeof(uint16_t));
    return;
  }

  // Letzter Ausweg ohne Cache (PSRAM fehlgeschlagen): Hintergrund, Zeiger bleiben sichtbar.
  for (int i = 0; i < 480 * 480; i++) fb[i] = vdoFaceColor();
}

static void logicalToDisplay(int lx, int ly, int *dx, int *dy) {
  switch (g_rotationDeg) {
    case 0:
      *dx = lx;
      *dy = ly;
      return;
    case 90:
      *dx = 479 - ly;
      *dy = lx;
      return;
    case 180:
      *dx = 479 - lx;
      *dy = 479 - ly;
      return;
    case 270:
      *dx = ly;
      *dy = 479 - lx;
      return;
    default:
      break;
  }
  float fx = (float)lx - ROT_PIVOT;
  float fy = (float)ly - ROT_PIVOT;
  *dx = (int)lroundf(fx * g_rotCos - fy * g_rotSin + ROT_PIVOT);
  *dy = (int)lroundf(fx * g_rotSin + fy * g_rotCos + ROT_PIVOT);
}

static struct tm g_lastClockDrawTm = {};
static bool      g_lastClockDrawTmValid = false;

static void drawVdoClock() {
  if (!ensureFrame()) return;
  if (g_clockFacePage != 0) {
    invalidateClockHands();
    g_clockFacePage = 0;
  }
  struct tm now = {};
  if (!readClockTime(&now)) return;
  g_lastClockDrawTm = now;
  g_lastClockDrawTmValid = true;

  float seconds     = now.tm_sec;
  float minuteValue = now.tm_min + seconds / 60.0f;
  float hourValue   = (now.tm_hour % 12) + minuteValue / 60.0f;
  float s = g_dialScalePct / 100.0f;
  if (s < 0.30f) s = 0.30f;
  if (s > 1.20f) s = 1.20f;
#if FEATURE_VERBOSE_SERIAL
  const uint32_t logMs = millis();
  if (!g_logDrawLastMs || logMs - g_logDrawLastMs >= LOG_DRAW_MS) {
    g_logDrawLastMs = logMs;
    float secAngle = (seconds / 60.0f) * 2.0f * PI - PI / 2.0f;
    int tipLx = 240 + (int)lroundf(cosf(secAngle) * 188.0f * s);
    int tipLy = 240 + (int)lroundf(sinf(secAngle) * 188.0f * s);
    int tipDx = tipLx, tipDy = tipLy;
    logicalToDisplay(tipLx, tipLy, &tipDx, &tipDy);
    Serial.printf("[DRAW] clock %02d:%02d:%02d h=%.2f m=%.2f rot=%d scale=%d%% secTip=%d,%d->%d,%d\n",
                  now.tm_hour, now.tm_min, now.tm_sec, hourValue, minuteValue,
                  g_rotationDeg, g_dialScalePct, tipLx, tipLy, tipDx, tipDy);
  }
#endif
  #define SC(v) ((int)((v) * s + 0.5f))

  const bool incremental = g_clockHandsValid && clockFaceSourceReady();
  if (!incremental) {
    copyVdoDialToFrame();
  } else {
    restoreHand(g_clockLastHourVal, 12.0f, SC(118), SC(18));
    restoreHand(g_clockLastHourVal, 12.0f, SC(118), SC(13));
    restoreHand(g_clockLastMinVal, 60.0f, SC(172), SC(15));
    restoreHand(g_clockLastMinVal, 60.0f, SC(172), SC(10));
    restoreHand(g_clockLastSecVal, 60.0f, SC(188), SC(4));
    restoreCircleFromFace(240, 240, SC(26));
    webServerPoll(2);
  }

  drawHand(hourValue,   12.0f, SC(118), SC(18), RGB565(24,  24,  22));
  drawHand(hourValue,   12.0f, SC(118), SC(13), RGB565(222, 222, 214));
  drawHand(minuteValue, 60.0f, SC(172), SC(15), RGB565(24,  24,  22));
  drawHand(minuteValue, 60.0f, SC(172), SC(10), RGB565(226, 226, 218));
  webServerPoll(2);
  drawHand(seconds,     60.0f, SC(188), SC(4),  RGB565(235, 24,  20));
  fillCircleFast(240, 240, SC(26), RGB565(205, 205, 198));
  fillCircleFast(240, 240, SC(15), RGB565(166, 122, 42));
  fillCircleFast(240, 240, SC(9),  RGB565(38,  30,  18));
  fillCircleFast(240, 240, SC(5),  RGB565_BLACK);
  #undef SC

  g_clockLastSecVal = seconds;
  g_clockLastMinVal = minuteValue;
  g_clockLastHourVal = hourValue;
  g_clockHandsValid = clockFaceSourceReady();
  presentFrame();
}

// 2-column menu grid on 480 round display: gaps between tiles, inset for round clip.
#define MENU_ROWS        5
#define MENU_COLS        2
#define MENU_COL0_X      84
#define MENU_COL1_X      252
#define MENU_COL_W       152
#define MENU_GRID_Y0     82
#define MENU_ROW_H       52    // zone per row (tile + vertical gap)
#define MENU_TILE_H      42
#define MENU_LABEL_SCALE 2
#define EDGE_TAP_W       48    // linker/rechter Rand: Seite vor/zurueck

struct MenuEntry { const char *label; uint16_t accent; uint8_t page; };

static const MenuEntry MENU_ENTRIES[MENU_ROWS * MENU_COLS] = {
  {"UHR",    RGB565(200, 40,  35), 0},
  {"MOTOR",  RGB565(40,  150, 210), 2},
  {"LAMBDA", RGB565(60,  185, 90),  3},
  {"HUB",    RGB565(190, 90,  210), 4},
  {"IMU",    RGB565(200, 100, 50),  6},
  {"OEL",    RGB565(230, 135, 40),  7},
  {"SETUP",  RGB565(210, 170, 45),  5},
  {"DZM UHR",RGB565(225, 185, 75),  8},
  {"SETUP 2",RGB565(185, 145, 50),  9},
  {"",       RGB565(80, 80, 80),    0},
};

static void menuTileRect(int index, int *x, int *y, int *w, int *h) {
  const int row = index / MENU_COLS;
  const int col = index % MENU_COLS;
  *x = (col == 0) ? MENU_COL0_X : MENU_COL1_X;
  *y = MENU_GRID_Y0 + row * MENU_ROW_H;
  *w = MENU_COL_W;
  *h = MENU_TILE_H;
}

static int menuIndexFromTap(uint16_t tapX, uint16_t tapY) {
  if (tapY < MENU_GRID_Y0) return -1;
  const int row = (int)(tapY - MENU_GRID_Y0) / MENU_ROW_H;
  if (row >= MENU_ROWS) return -1;
  int col = -1;
  if      (tapX >= MENU_COL0_X && tapX < MENU_COL0_X + MENU_COL_W) col = 0;
  else if (tapX >= MENU_COL1_X && tapX < MENU_COL1_X + MENU_COL_W) col = 1;
  if (col < 0) return -1;
  const int idx = row * MENU_COLS + col;
  if (MENU_ENTRIES[idx].label[0] == 0) return -1;
  return idx;
}

static void drawMenuTile(int x, int y, int w, int h, const char *label, uint16_t accent) {
  fillRectFast(x, y, w, h, RGB565(18, 18, 18));
  fillRectFast(x, y, 5, h, accent);
  drawLineFast(x, y,     x + w, y,     RGB565(70, 70, 70), 1);
  drawLineFast(x, y + h, x + w, y + h, RGB565(55, 55, 55), 1);
  const int scale = MENU_LABEL_SCALE;
  const int ty = y + (h - 7 * scale) / 2;
  drawTextCentered(x + w / 2, ty, label, RGB565(235, 235, 225), scale);
}

static void drawMenuOverview() {
  if (!ensureFrame()) return;
  g_clockFacePage = currentPage;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(80, 80, 75));
  drawTextCentered(240, 52, "MENU", RGB565(235, 235, 225), 3);
  for (int i = 0; i < MENU_ROWS * MENU_COLS; i++) {
    int x, y, w, h;
    menuTileRect(i, &x, &y, &w, &h);
    if (MENU_ENTRIES[i].label[0] == 0) continue;
    drawMenuTile(x, y, w, h, MENU_ENTRIES[i].label, MENU_ENTRIES[i].accent);
    if (i == 2 || i == 4) webServerPoll(2);
  }
  char ipLine[32];
  snprintf(ipLine, sizeof(ipLine), "IP %s", g_ipStr);
  drawTextCentered(240, 412, ipLine, RGB565(150, 200, 150), 2);
  if (g_featureWifi && strlen(currentWifiSsid()) > 0) {
    drawTextCentered(240, 434, currentWifiSsid(), RGB565(120, 150, 150), 2);
  }
  presentFrame();
}

static bool bleFresh() { return dataFresh(); }

static const char* liveStatusText() {
  if (dataFresh()) return "LIVE";
  if (hubLinkOk()) return "WARTE";
  if (g_dataPath == DATA_PATH_WIFI_HUB) {
    if (!g_featureWifi || WiFi.status() != WL_CONNECTED) return "WiFi...";
    return "Hub...";
  }
  return g_bleConnMode == BLE_MODE_SPARTAN_HUB ? "KEIN HUB" : "KEIN 123";
}

static void drawDataRow(int y, const char* label, const char* value, uint16_t col) {
  drawTextSmall(92,  y, label, RGB565(160, 160, 160), 2);
  drawTextSmall(244, y, value, col, 2);
}

static void drawMotorStatusFooter(int yLambda, int yStatus) {
  char lbuf[12];
  uint16_t lcol = RGB565(110, 60, 60);
  const bool fresh = dataFresh();
  if (fresh && g_lambdaValid) {
    snprintf(lbuf, sizeof(lbuf), "L %.2f", g_lambda);
    if (g_lambda < 0.97f) lcol = RGB565(235, 120, 40);
    else if (g_lambda > 1.03f) lcol = RGB565(80, 160, 240);
    else lcol = RGB565(70, 210, 100);
  } else {
    strcpy(lbuf, "L ---");
  }
  drawTextCentered(240, yLambda, lbuf, lcol, 2);
  const char* st = liveStatusText();
  drawTextCentered(240, yStatus, st, fresh ? RGB565(60, 200, 90) : RGB565(200, 120, 50), 2);
}

static void drawMbMajorTicks(int cx, int cy, int ringR, uint16_t color) {
  for (int v = 0; v <= MB_SCALE_MAX_VALUE; v += 5) {
    const int thick = (v % 10 == 0) ? 3 : 2;
    const int rIn = ringR - ((v % 10 == 0) ? 18 : 12);
    drawMbTick(cx, cy, ringR + 2, rIn, (float)v, color, thick);
  }
}

static void drawMotorPageVdo() {
  const int ringR = 198, ringT = 14, labelR = ringR - ringT - 28;
  const uint16_t ringBase  = RGB565(40, 110, 160);
  const uint16_t redZone   = RGB565(100, 18, 18);
  const uint16_t needleCol = RGB565(255, 130, 40);
  const int maxScale = g_rpmScaleMax / 100;
  drawTextCentered(240, 52, "MOTOR", RGB565(60, 170, 230), 5);
  drawCircleLine(240, 240, ringR, ringT, ringBase);
  const float redlineScale = (float)g_rpmRedline / 100.0f;
  drawArcRing(240, 240, ringR, ringT, redlineScale, rpmScaleMaxValue(), redZone);
  drawRpmTick(240, 240, ringR + 4, ringR - ringT - 8, 4.0f, RGB565(235, 235, 225));
  for (int v = 10; v <= maxScale; v += 10) {
    drawRpmTick(240, 240, ringR + 4, ringR - ringT - 8, (float)v, RGB565(235, 235, 225));
    char lab[4];
    snprintf(lab, sizeof(lab), "%d", v);
    drawRpmScaleLabel(240, 240, labelR, (float)v, lab);
  }
  const bool fresh = dataFresh();
  const float rpmVal = fresh ? rpmScaleValue(g_rpm) : (float)RPM_SCALE_MIN_VALUE;
  drawHandAngle(rpmScaleAngle(rpmVal), 168, 6, needleCol);
  fillCircleFast(240, 240, 8, RGB565(30, 30, 28));
  fillCircleFast(240, 240, 4, needleCol);
  char buf[16];
  uint16_t cv = fresh ? RGB565(235, 235, 225) : RGB565(110, 60, 60);
  const char* na = "---";
  if (fresh) { snprintf(buf, sizeof(buf), "%d", (int)g_rpm); drawTextCentered(240, 268, buf, cv, 3); }
  else drawTextCentered(240, 268, na, cv, 3);
  if (fresh) { snprintf(buf, sizeof(buf), "%d", (int)g_adv); drawDataRow(300, "ADV", buf, cv); }
  else drawDataRow(300, "ADV", na, cv);
  if (fresh) { snprintf(buf, sizeof(buf), "%d", (int)g_map); drawDataRow(328, "MAP", buf, cv); }
  else drawDataRow(328, "MAP", na, cv);
  drawMotorStatusFooter(395, 420);
}

static void drawMotorPageMercedes() {
  const int ringR = 212, ringT = 10, labelR = 158;
  const uint16_t cream = vdoMarkColor();
  const uint16_t redZone = vdoWarnColor();
  const uint16_t yellow = vdoNeedleColor();
  fillCircleFast(240, 240, 214, vdoFaceColor());
  drawCircleLine(240, 240, 222, 9, RGB565(18, 18, 17));
  drawCircleLine(240, 240, 211, 2, g_nightMode ? RGB565(58, 76, 54) : RGB565(65, 63, 58));
  const float redlineScale = (float)g_rpmRedline / 100.0f;
  drawArcRing(240, 240, ringR - 18, 24, redlineScale, rpmScaleMaxValue(), redZone);
  drawRpmTick(240, 240, ringR, ringR - 24, 4.0f, cream);
  const int maxScale = g_rpmScaleMax / 100;
  for (int v = 10; v <= maxScale; v += 10) {
    drawRpmTick(240, 240, ringR, ringR - ((v % 20 == 0) ? 30 : 22), (float)v, cream);
    char lab[4];
    snprintf(lab, sizeof(lab), "%d", v);
    drawRpmScaleLabel(240, 240, labelR, (float)v, lab);
  }
  drawTextCentered(240, 124, "VDO", cream, 2);
  drawTextCentered(240, 148, "UPM", cream, 2);
  drawTextCentered(240, 172, "X100", cream, 2);
  drawCircleLine(MB_CLOCK_CX, MB_CLOCK_CY, MB_CLOCK_R + 12, 2, RGB565(12, 12, 12));
  drawMercedesSubClock();
  const bool fresh = dataFresh();
  const float rpmVal = fresh ? rpmScaleValue(g_rpm) : (float)RPM_SCALE_MIN_VALUE;
  drawHandAngle(rpmScaleAngle(rpmVal), 182, 5, yellow);
  fillCircleFast(240, 240, 33, RGB565(42, 41, 39));
  drawCircleLine(240, 240, 33, 2, g_nightMode ? RGB565(70, 100, 62) : RGB565(85, 83, 76));
  drawTextCentered(240, 438, "MADE IN GERMANY", cream, 2);
}

static const char* motorStyleLabel() {
  return "VDO";
}

static void drawMotorPage() {
  if (!ensureFrame()) return;
  g_clockFacePage = currentPage;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(40, 110, 160));
  drawTextCentered(240, 58, "MOTOR", RGB565(60, 170, 230), 3);
  const bool fresh = dataFresh();
  char buf[16];
  const char* na = "---";
  uint16_t cv = fresh ? RGB565(235, 235, 225) : RGB565(110, 60, 60);
  if (fresh) snprintf(buf, sizeof(buf), "%d", (int)g_rpm);
  drawTextCentered(240, 148, fresh ? buf : na, cv, 5);
  drawTextCentered(240, 188, "RPM", RGB565(160, 160, 160), 2);
  if (fresh) { snprintf(buf, sizeof(buf), "%d", (int)g_adv); drawDataRow(248, "ADV", buf, cv); }
  else drawDataRow(248, "ADV", na, cv);
  if (fresh) { snprintf(buf, sizeof(buf), "%d", (int)g_map); drawDataRow(276, "MAP", buf, cv); }
  else drawDataRow(276, "MAP", na, cv);
  if (fresh && g_battValid) {
    snprintf(buf, sizeof(buf), "%.1f V", g_battVolt);
    drawDataRow(304, "BAT", buf, cv);
  } else drawDataRow(304, "BAT", na, cv);
  drawMotorStatusFooter(360, 400);
  presentFrame();
}

static void drawCombiSmallGauge(int cx, int cy, const char* title, const char* unit,
                                float value, bool valid, float vMin, float vMax,
                                const int* labels, size_t labelCount) {
  const uint16_t cream = vdoMarkColor();
  const uint16_t dim = vdoDimMarkColor();
  const uint16_t yellow = vdoNeedleColor();
  const float a0 = 210.0f, a1 = 330.0f;
  drawCircleLine(cx, cy, 58, 2, vdoDarkRimColor());
  drawGaugeArcRing(cx, cy, 51, 2, a0, a1, dim);
  for (int i = (int)vMin; i <= (int)vMax; i++) {
    const bool major = (i == (int)vMin || i == (int)vMax || (i % 5) == 0);
    drawGaugeTick(cx, cy, 51, major ? 39 : 44, (float)i, vMin, vMax, a0, a1, cream, major ? 3 : 2);
  }
  char buf[8];
  for (size_t i = 0; i < labelCount; i++) {
    snprintf(buf, sizeof(buf), "%d", labels[i]);
    drawGaugeLabel(cx, cy, 31, (float)labels[i], vMin, vMax, a0, a1, buf, cream, 1);
  }
  drawGaugeNeedle(cx, cy, valid ? value : vMin, vMin, vMax, a0, a1, 38, 3, yellow);
  fillCircleFast(cx, cy, 8, vdoHubColor());
  drawCircleLine(cx, cy, 8, 1, dim);
  drawTextCentered(cx, cy + 36, title, cream, 1);
  drawTextCentered(cx, cy + 48, unit, dim, 1);
}

static void drawCombiInstrument() {
  const uint16_t cream = vdoMarkColor();
  const uint16_t dim = vdoDimMarkColor();
  const uint16_t yellow = vdoNeedleColor();
  const uint16_t redZone = vdoWarnColor();
  fillCircleFast(240, 240, 216, vdoFaceColor());
  drawCircleLine(240, 240, 222, 8, vdoDarkRimColor());
  drawCircleLine(240, 240, 211, 1, g_nightMode ? RGB565(60, 64, 58) : RGB565(73, 68, 60));

  const int mainCx = 240, mainCy = 214;
  const float mainMin = 50.0f, mainMax = 150.0f;
  const float mainA0 = 205.0f, mainA1 = 335.0f;
  drawGaugeArcRing(mainCx, mainCy, 184, 2, mainA0, mainA1, dim);
  drawGaugeArcRing(mainCx, mainCy, 181, 17,
                   205.0f + (120.0f - mainMin) / (mainMax - mainMin) * (mainA1 - mainA0),
                   mainA1, redZone);
  for (int t = 50; t <= 150; t += 10) {
    const bool major = (t % 25) == 0 || t == 50 || t == 150;
    drawGaugeTick(mainCx, mainCy, 187, major ? 151 : 164,
                  (float)t, mainMin, mainMax, mainA0, mainA1, cream, major ? 4 : 2);
  }
  const int mainLabels[] = {50, 100, 150};
  for (size_t i = 0; i < sizeof(mainLabels) / sizeof(mainLabels[0]); i++) {
    char lab[8];
    snprintf(lab, sizeof(lab), "%d", mainLabels[i]);
    const float angle = gaugeValueAngle((float)mainLabels[i], mainMin, mainMax, mainA0, mainA1);
    const int scale = mainLabels[i] == 100 ? 2 : 3;
    const int lx = mainCx + (int)lroundf(cosf(angle) * 132.0f) - textWidthSmall(lab, scale) / 2;
    const int ly = mainCy + (int)lroundf(sinf(angle) * 132.0f) - (7 * scale) / 2;
    drawTextSmall(lx, ly, lab, cream, scale);
  }
  drawTextCentered(240, 78, "VDO", cream, 2);
  drawTextCentered(240, 108, "OEL-TEMP.", cream, 2);
  drawTextCentered(240, 132, "C", dim, 2);

  const bool tempValid = dataFresh() && g_g123Valid && g_g123Temp > -40.0f && g_g123Temp < 180.0f;
  const float oilTemp = tempValid ? g_g123Temp : mainMin;
  drawVdoNeedle(mainCx, mainCy, gaugeValueAngle(oilTemp, mainMin, mainMax, mainA0, mainA1), 144, yellow);
  fillCircleFast(mainCx, mainCy, 24, vdoHubColor());
  drawCircleLine(mainCx, mainCy, 24, 2, dim);

  const int pressureLabels[] = {0, 5, 10};
  drawCombiSmallGauge(104, 326, "OEL-DRUCK", "BAR", 0.0f, false, 0.0f, 10.0f,
                      pressureLabels, sizeof(pressureLabels) / sizeof(pressureLabels[0]));
  const int voltLabels[] = {10, 12, 16};
  const float volt = g_battValid ? g_battVolt : 10.0f;
  drawCombiSmallGauge(376, 326, "VOLT", "V", volt, g_battValid, 10.0f, 16.0f,
                      voltLabels, sizeof(voltLabels) / sizeof(voltLabels[0]));

  drawVdoSubClockAt(240, 344, 43);
  if (!tempValid) drawTextCentered(240, 152, "---", dim, 2);
  drawTextCentered(240, 438, "MADE IN GERMANY", cream, 2);
}

static void drawCombiPage() {
  if (!ensureFrame()) return;
  g_clockFacePage = currentPage;
  fillFrame(RGB565_BLACK);
  drawCombiInstrument();
  presentFrame();
}

static void drawTachClockCombiPage() {
  if (!ensureFrame()) return;
  g_clockFacePage = currentPage;
  fillFrame(RGB565_BLACK);
  const uint16_t cream = vdoMarkColor();
  const uint16_t dim = vdoDimMarkColor();
  const uint16_t yellow = vdoNeedleColor();
  const uint16_t warn = vdoWarnColor();
  fillCircleFast(240, 240, 216, vdoFaceColor());
  drawCircleLine(240, 240, 223, 8, vdoDarkRimColor());
  drawCircleLine(240, 240, 211, 1, g_nightMode ? RGB565(60, 64, 58) : RGB565(73, 68, 60));

  const int cx = 240, cy = 240;
  const int ringR = 214;
  const int maxScale = g_rpmScaleMax / 100;
  const float redlineScale = constrain((float)g_rpmRedline / 100.0f,
                                       (float)RPM_SCALE_MIN_VALUE, rpmScaleMaxValue());
  drawArcRing(cx, cy, ringR - 21, 18, redlineScale, rpmScaleMaxValue(), warn);
  drawArcRing(cx, cy, ringR - 7, 1, (float)RPM_SCALE_MIN_VALUE, rpmScaleMaxValue(), dim);
  drawRpmTick(cx, cy, ringR, ringR - 34, 4.0f, cream);
  for (int v = 10; v <= maxScale; v += 5) {
    const bool major = (v % 10) == 0;
    drawRpmTick(cx, cy, ringR, ringR - (major ? 38 : 25), (float)v, cream);
  }
  for (int v = 10; v <= maxScale; v += 10) {
    char lab[6];
    snprintf(lab, sizeof(lab), "%d", v);
    const float angle = rpmScaleAngle((float)v);
    const int scale = 3;
    const int lx = cx + (int)lroundf(cosf(angle) * 166.0f) - textWidthSmall(lab, scale) / 2;
    const int ly = cy + (int)lroundf(sinf(angle) * 166.0f) - (7 * scale) / 2;
    drawTextSmall(lx, ly, lab, cream, scale);
  }

  drawTextCentered(240, 98, "VDO", cream, 1);
  drawTextCentered(240, 114, "UPM", cream, 1);
  drawTextCentered(240, 130, "x100", cream, 1);
  char lambdaBuf[12];
  if (dataFresh() && g_lambdaValid) snprintf(lambdaBuf, sizeof(lambdaBuf), "L %.2f", g_lambda);
  else snprintf(lambdaBuf, sizeof(lambdaBuf), "L --");
  drawTextCentered(240, 154, lambdaBuf, g_lambdaValid ? RGB565(120, 220, 150) : dim, 2);

  const bool fresh = dataFresh();
  const float rpmVal = fresh ? rpmScaleValue(g_rpm) : (float)RPM_SCALE_MIN_VALUE;
  drawVdoNeedle(cx, cy, rpmScaleAngle(rpmVal), 184, yellow);
  fillCircleFast(cx, cy, 19, vdoHubColor());
  drawCircleLine(cx, cy, 19, 2, dim);
  fillCircleFast(cx, cy, 8, yellow);

  drawVdoSubClockAt(240, 342, 43);
  drawTextCentered(240, 438, "MADE IN GERMANY", cream, 2);
  presentFrame();
}

static void drawLambdaPage() {
  if (!ensureFrame()) return;
  g_clockFacePage = currentPage;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(45, 150, 70));
  drawTextCentered(240, 58, "LAMBDA", RGB565(70, 200, 100), 5);
  const bool fresh = dataFresh() && g_lambdaValid;
  const bool linked = fresh || hubLinkOk();
  char buf[16];
  {
    char mbuf[12];
    if (linked) snprintf(mbuf, sizeof(mbuf), "MAP %d", (int)g_map);
    else strcpy(mbuf, "MAP ---");
    drawTextCentered(240, 88, mbuf, linked ? RGB565(180, 180, 180) : RGB565(110, 60, 60), 2);
  }
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
  const char* st = liveStatusText();
  drawTextCentered(240, 370, st, dataFresh() ? RGB565(60, 200, 90) : RGB565(200, 120, 50), 2);
  presentFrame();
}

static void drawHubPage() {
  if (!ensureFrame()) return;
  g_clockFacePage = currentPage;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(150, 70, 180));
  drawTextCentered(240, 54, "HUB", RGB565(205, 120, 230), 5);
  drawTextCentered(240, 84, dataPathLabel(), RGB565(150, 150, 150), 2);
  char buf[24];
  drawDataRow(112, "HOST",  g_hubHost, RGB565(235, 235, 225));
  drawDataRow(148, "LINK",  hubLinkOk() ? "OK" : "OFF",
              hubLinkOk() ? RGB565(60, 210, 100) : RGB565(220, 130, 50));
  drawDataRow(188, "SRC",   g_hubSourceTag, RGB565(235, 235, 225));
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)g_liveRxCnt);
  drawDataRow(228, "RX",    buf, RGB565(235, 235, 225));
  snprintf(buf, sizeof(buf), "%lu MS", g_liveLastRx ? (unsigned long)(millis() - g_liveLastRx) : 0UL);
  drawDataRow(268, "AGE",   buf, RGB565(235, 235, 225));
  snprintf(buf, sizeof(buf), "%.1f V", g_battVolt);
  drawDataRow(308, "BATT",  g_battValid  ? buf : "---", RGB565(235, 235, 225));
  snprintf(buf, sizeof(buf), "%.1f V", g_auxBattVolt);
  drawDataRow(348, "AUX",   g_auxBattValid ? buf : "---", RGB565(235, 235, 225));
  drawDataRow(388, "IP",    g_ipStr, RGB565(150, 200, 150));
  drawTextCentered(240, 410, liveStatusText(), dataFresh() ? RGB565(60, 200, 90) : RGB565(200, 120, 50), 2);
  presentFrame();
}

static void drawSetupPage() {
  if (!ensureFrame()) return;
  g_clockFacePage = currentPage;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(185, 150, 45));
  drawTextCentered(240, 54, "SETUP 1", RGB565(230, 190, 70), 4);
  drawTextCentered(240, 82, "ANZEIGE / WLAN", RGB565(170, 160, 135), 2);
  char buf[28];
  snprintf(buf, sizeof(buf), "%d %%", g_dialScalePct);
  drawDataRow(124, "UHR",   buf, RGB565(235, 235, 225));
  snprintf(buf, sizeof(buf), "%d %%", g_brightnessPct);
  drawDataRow(172, "HELL",  buf, RGB565(235, 235, 225));
  snprintf(buf, sizeof(buf), "%d DEG", g_rotationDeg);
  drawDataRow(220, "ROT",   buf, RGB565(235, 235, 225));
  char wbuf[20];
  if (g_wifiApOnly || (g_apOn && WiFi.status() != WL_CONNECTED)) {
    drawDataRow(268, "WIFI", "AP Setup", RGB565(235, 180, 60));
  } else if (!g_featureWifi) {
    drawDataRow(268, "WIFI", "AUS", RGB565(220, 130, 50));
  } else if (WiFi.status() == WL_CONNECTED && strlen(currentWifiSsid()) > 0) {
    if (isOnBusWifi()) {
      drawDataRow(268, "WIFI", "BUS OK", RGB565(60, 210, 100));
    } else {
      snprintf(wbuf, sizeof(wbuf), "%.16s", currentWifiSsid());
      drawDataRow(268, "WIFI", wbuf, RGB565(60, 210, 100));
    }
  } else {
    drawDataRow(268, "WIFI", "Home...", RGB565(220, 130, 50));
  }
  drawDataRow(316, "NACHT", g_nightMode ? "GRUEN" : "TAG",
              g_nightMode ? RGB565(80, 220, 120) : RGB565(235, 235, 225));
  drawDataRow(364, "WEITER", "SETUP 2", RGB565(230, 190, 70));
  drawTextCentered(240, 432, "TIP MENU", RGB565(180, 180, 170), 2);
  drawTextCentered(240, 456, "RAND = SEITE", RGB565(120, 120, 115), 1);
  presentFrame();
}

static void drawSetup2Page() {
  if (!ensureFrame()) return;
  g_clockFacePage = currentPage;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(185, 150, 45));
  drawTextCentered(240, 54, "SETUP 2", RGB565(230, 190, 70), 4);
  drawTextCentered(240, 82, "FUNK / QUELLE", RGB565(170, 160, 135), 2);
  drawDataRow(112, "BLE",    g_featureBle ? (g_bleConn ? "OK" : "AN") : "AUS",
              g_featureBle && g_bleConn ? RGB565(60, 210, 100) : RGB565(220, 130, 50));
  drawDataRow(154, "BLE SRC", g_bleConnMode == BLE_MODE_SPARTAN_HUB ? "SPARTAN" : "123 DIREKT",
              g_bleConnMode == BLE_MODE_SPARTAN_HUB ? RGB565(120, 200, 120) : RGB565(235, 150, 60));
  {
    char ebuf[20];
    if (!g_featureEspNow) snprintf(ebuf, sizeof(ebuf), "AUS");
    else if (espNowDataFresh()) snprintf(ebuf, sizeof(ebuf), "OK rx%lu", (unsigned long)g_espNowRx);
    else snprintf(ebuf, sizeof(ebuf), "AN %s", espNowChannelLabel());
    drawDataRow(196, "ESPNOW", ebuf,
                g_featureEspNow && espNowDataFresh() ? RGB565(60, 210, 100) :
                (g_featureEspNow ? RGB565(220, 180, 60) : RGB565(150, 150, 150)));
  }
  drawDataRow(238, "ESPN CH", espNowChannelLabel(), RGB565(120, 180, 240));
  drawDataRow(280, "BUZZER", g_featureBuzzer ? "AN" : "AUS",
              g_featureBuzzer ? RGB565(60, 210, 100) : RGB565(150, 150, 150));
  drawDataRow(322, "QUELLE", sourceModeLabel(),
              g_dataPath == DATA_PATH_WIFI_HUB ? RGB565(80, 160, 240) :
              (g_bleConnMode == BLE_MODE_SPARTAN_HUB ? RGB565(120, 200, 120) : RGB565(235, 150, 60)));
  drawDataRow(364, "IMU NULL", g_imuTrimmed ? "SET" : "TAP",
              g_imuTrimmed ? RGB565(60, 210, 100) : RGB565(220, 130, 50));
  drawTextCentered(240, 432, "TIP MENU", RGB565(180, 180, 170), 2);
  drawTextCentered(240, 456, "RAND = SEITE", RGB565(120, 120, 115), 1);
  presentFrame();
}

static void drawImuPage() {
  if (!ensureFrame()) return;
  g_clockFacePage = currentPage;
  fillFrame(RGB565_BLACK);
  const int cx = 240, cy = 240;
  const uint16_t cream = vdoMarkColor();
  const uint16_t dim = vdoDimMarkColor();
  const uint16_t lower = g_nightMode ? RGB565(23, 30, 27) : RGB565(43, 36, 31);
  fillCircleFast(cx, cy, 216, vdoFaceColor());
  for (int y = cy; y <= cy + 204; y++) {
    for (int x = cx - 204; x <= cx + 204; x++) {
      const int dx = x - cx, dy = y - cy;
      if (dx * dx + dy * dy <= 204 * 204) setPixel(x, y, lower);
    }
  }
  drawCircleLine(cx, cy, 223, 8, vdoDarkRimColor());
  drawCircleLine(cx, cy, 211, 2, g_nightMode ? RGB565(58, 82, 52) : RGB565(84, 80, 72));
  drawLineFast(cx - 190, cy, cx + 190, cy, dim, 3);

  for (int a = -40; a <= 40; a += 10) {
    const float ar = (float)a * PI / 180.0f - PI / 2.0f;
    const int ro = 190;
    const int ri = (a % 20 == 0) ? 164 : 176;
    drawLineFast(cx + (int)lroundf(cosf(ar) * ri), cy + (int)lroundf(sinf(ar) * ri),
                 cx + (int)lroundf(cosf(ar) * ro), cy + (int)lroundf(sinf(ar) * ro),
                 cream, (a % 20 == 0) ? 3 : 2);
    if (a != 0 && a % 20 == 0) {
      char lab[8];
      snprintf(lab, sizeof(lab), "%d", abs(a));
      const int tx = cx + (int)lroundf(cosf(ar) * 136.0f);
      const int ty = cy + (int)lroundf(sinf(ar) * 136.0f);
      drawTextCentered(tx, ty - 7, lab, cream, 2);
    }
  }
  drawLineFast(cx - 17, cy - 176, cx, cy - 144, cream, 3);
  drawLineFast(cx + 17, cy - 176, cx, cy - 144, cream, 3);
  drawLineFast(cx - 17, cy - 176, cx + 17, cy - 176, cream, 3);

  if (g_imuPresent) {
    const float pitch = constrain(g_imuPitch, -45.0f, 45.0f);
    const float roll = constrain(g_imuRoll, -45.0f, 45.0f);
    const int pitchOffset = (int)lroundf(pitch * 1.8f);
    for (int p = -40; p <= 40; p += 10) {
      if (p == 0) continue;
      const int y = cy - pitchOffset + p * 3;
      const int half = (p % 20 == 0) ? 74 : 48;
      drawLineFast(cx - half, y, cx + half, y, dim, 2);
      if (p % 20 == 0) {
        char lab[8];
        snprintf(lab, sizeof(lab), "%d", abs(p));
        drawTextCentered(cx - half - 34, y - 7, lab, dim, 1);
        drawTextCentered(cx + half + 34, y - 7, lab, dim, 1);
      }
    }

    const float rr = -roll * PI / 180.0f;
    const int rx = (int)lroundf(cosf(rr) * 66.0f);
    const int ry = (int)lroundf(sinf(rr) * 66.0f);
    drawLineFast(cx - rx, cy - ry, cx + rx, cy + ry, cream, 4);
    fillCircleFast(cx, cy, 5, cream);
    drawLineFast(cx, cy - 24, cx, cy + 92, cream, 3);
    drawLineFast(cx - 45, cy + 92, cx + 45, cy + 92, cream, 3);
    drawLineFast(cx - 45, cy + 92, cx - 66, cy + 66, cream, 3);
    drawLineFast(cx + 45, cy + 92, cx + 66, cy + 66, cream, 3);

    const float gradePct = constrain(tanf(pitch * PI / 180.0f) * 100.0f, -99.0f, 99.0f);
    char gradeLine[18];
    char degLine[18];
    snprintf(gradeLine, sizeof(gradeLine), "%+.0f%%", gradePct);
    snprintf(degLine, sizeof(degLine), "%+.1f DEG", pitch);
    drawTextCentered(240, 360, "STEIGUNG", dim, 2);
    drawTextCentered(240, 390, gradeLine, cream, 4);
    drawTextCentered(240, 420, degLine, dim, 2);
    const int barY = 448;
    const int barX0 = 78;
    const int barX1 = 402;
    const int barCx = 240;
    drawLineFast(barX0, barY, barX1, barY, dim, 4);
    drawLineFast(barCx, barY - 15, barCx, barY + 15, cream, 3);
    for (int g = -20; g <= 20; g += 10) {
      const int tx = barCx + (int)lroundf((float)g / 25.0f * 150.0f);
      drawLineFast(tx, barY - 9, tx, barY + 9, dim, 2);
      if (g != 0) {
        char gl[8];
        snprintf(gl, sizeof(gl), "%d", abs(g));
        drawTextCentered(tx, barY - 32, gl, dim, 1);
      }
    }
    const int markerX = constrain(barCx + (int)lroundf(gradePct / 25.0f * 150.0f), barX0, barX1);
    const uint16_t markerCol = gradePct >= 0.0f ? RGB565(235, 175, 45) : RGB565(120, 180, 240);
    drawLineFast(markerX - 12, barY - 18, markerX, barY - 2, markerCol, 4);
    drawLineFast(markerX + 12, barY - 18, markerX, barY - 2, markerCol, 4);
    drawLineFast(markerX, barY - 2, markerX, barY + 18, markerCol, 4);
    drawTextCentered(105, 466, "AB", dim, 1);
    drawTextCentered(375, 466, "AUF", dim, 1);
    if (g_imuTrimmed) drawTextCentered(240, 96, "NULL", dim, 1);

    static bool buzzerOn = false;
    const bool shake = qmi8658ShakeDetected(1.5f);
    const bool wantBuzz = g_featureBuzzer && shake;
    if (shake) drawTextCentered(240, 112, "SHAKE", RGB565(255, 70, 50), 2);
    if (wantBuzz != buzzerOn) {
      buzzerOn = wantBuzz;
      hal_buzzer(buzzerOn);
    }
  } else {
    drawTextCentered(240, 210, "KEIN IMU", RGB565(200, 60, 60), 4);
  }
  presentFrame();
}

static void drawCurrentPage() {
  if (currentPage != 0) g_clockFacePage = currentPage;
  if      (currentPage == 0) drawVdoClock();
  else if (currentPage == 1) drawMenuOverview();
  else if (currentPage == 2) drawMotorPage();
  else if (currentPage == 3) drawLambdaPage();
  else if (currentPage == 4) drawHubPage();
  else if (currentPage == 5) drawSetupPage();
  else if (currentPage == 6) drawImuPage();
  else if (currentPage == 7) drawCombiPage();
  else if (currentPage == 8) drawTachClockCombiPage();
  else if (currentPage == 9) drawSetup2Page();
  else { currentPage = 0; drawVdoClock(); }
}

static void requestPage(uint8_t page) {
  if (page > PAGE_MAX) page = PAGE_MAX;
  currentPage = page;
  g_redrawPage = true;
}

// -------- Preferences --------
static void loadSettings() {
  Preferences p;
  p.begin("clock", true);
  g_dialScalePct       = p.getInt("scale",     DIAL_SCALE_DEFAULT);
  g_dialCenterOffsetX  = p.getInt("dial_off_x", DIAL_CENTER_OFF_X_DEFAULT);
  g_dialCenterOffsetY  = p.getInt("dial_off_y", DIAL_CENTER_OFF_Y_DEFAULT);
  g_brightnessPct = p.getInt("bright",    100);
  g_nightMode     = p.getBool("night",    false);
  g_rotationDeg   = p.getInt("rot_deg",   0);
  g_wifiProfile   = p.getUChar("wifi_prof", 0);
  if (g_wifiProfile >= wifiProfileCount()) g_wifiProfile = 0;
  g_featureWifi   = p.getBool("feat_wifi", strlen(currentWifiSsid()) > 0);
  g_featureBle    = p.getBool("feat_ble",  false);
  g_featureBuzzer = p.getBool("feat_buzzer", false);  // default OFF
  g_featureEspNow = p.getBool("feat_espnow", true);
  g_espNowChannelPref = p.getUChar("espnow_ch", 0);
  g_bleConnMode   = p.getUChar("ble_mode", BLE_MODE_SPARTAN_HUB) == BLE_MODE_DIRECT_123 ?
                    BLE_MODE_DIRECT_123 : BLE_MODE_SPARTAN_HUB;
  String mac123 = p.getString("ble_mac_123", "");
  if (mac123.length() == 0) mac123 = p.getString("ble_mac", DEFAULT_123_MAC);
  strncpy(g_bleTargetMac, mac123.c_str(), sizeof(g_bleTargetMac) - 1);
  if (mac123.equalsIgnoreCase("3c:ab:72:7f:d0:bc")) {
    mac123 = DEFAULT_123_MAC;
    Preferences pw;
    pw.begin("clock", false);
    pw.putString("ble_mac_123", mac123);
    pw.end();
  }
  g_bleTargetMac[sizeof(g_bleTargetMac) - 1] = 0;
  g_ble123AddrType = p.getUChar("ble_addr_123", BLE_ADDR_RANDOM);
  String macHub = p.getString("ble_mac_hub", "");
  if (macHub.length() == 0) macHub = DEFAULT_HUB_MAC;
  // Alte Defaults hatten BM6-MAC als Hub — auf Spartan3-Hub umstellen
  if (macHub.equalsIgnoreCase(DEFAULT_123_MAC)) {
    macHub = SPARTAN_MAC;
    Preferences pw;
    pw.begin("clock", false);
    pw.putString("ble_mac_hub", SPARTAN_MAC);
    pw.end();
  }
  strncpy(g_bleHubMac, macHub.c_str(), sizeof(g_bleHubMac) - 1);
  g_bleHubMac[sizeof(g_bleHubMac) - 1] = 0;
  qmi8658SetTrim(p.getFloat("imu_off_p", 0.0f), p.getFloat("imu_off_r", 0.0f),
                 p.getBool("imu_trim", false));
  g_timezoneIdx = p.getUChar("tz_idx", TIMEZONE_DEFAULT);
  if (g_timezoneIdx >= TIMEZONE_COUNT) g_timezoneIdx = TIMEZONE_DEFAULT;
  g_rpmRedline = p.getInt("rpm_red", RPM_REDLINE_DEFAULT);
  g_rpmScaleMax = p.getInt("rpm_max", RPM_SCALE_MAX_DEFAULT);
  g_motorStyle   = p.getUChar("motor_style", MOTOR_STYLE_VDO);
  if (g_motorStyle > MOTOR_STYLE_MERCEDES) g_motorStyle = MOTOR_STYLE_VDO;
  g_motorStyle = MOTOR_STYLE_VDO;
  g_dataPath = p.getUChar("data_path", DATA_PATH_WIFI_HUB) == DATA_PATH_BLE ?
               DATA_PATH_BLE : DATA_PATH_WIFI_HUB;
  g_wifiApOnly = p.getBool("wifi_ap_only", false);
  String hubHost = p.getString("hub_host", HUB_WIFI_DEFAULT_HOST);
  hubHost.trim();
  if (hubHost.length() == 0) hubHost = HUB_WIFI_DEFAULT_HOST;
  strncpy(g_hubHost, hubHost.c_str(), sizeof(g_hubHost) - 1);
  g_hubHost[sizeof(g_hubHost) - 1] = 0;
  if (isBusWifiSsid(currentWifiSsid()) && strcmp(g_hubHost, HUB_BUS_HOST) != 0) {
    strncpy(g_hubHost, HUB_BUS_HOST, sizeof(g_hubHost) - 1);
    g_hubHost[sizeof(g_hubHost) - 1] = 0;
  }
  p.end();
  applyTimezone();
  if (g_dialScalePct  < DIAL_SCALE_MIN) g_dialScalePct  = DIAL_SCALE_MIN;
  if (g_dialScalePct  > DIAL_SCALE_MAX) g_dialScalePct  = DIAL_SCALE_MAX;
  if (g_dialCenterOffsetX < -20) g_dialCenterOffsetX = -20;
  if (g_dialCenterOffsetX >  20) g_dialCenterOffsetX =  20;
  if (g_dialCenterOffsetY < -20) g_dialCenterOffsetY = -20;
  if (g_dialCenterOffsetY >  20) g_dialCenterOffsetY =  20;
  if (g_brightnessPct < 5)   g_brightnessPct = 5;
  if (g_brightnessPct > 100) g_brightnessPct = 100;
  if (g_rpmScaleMax < RPM_SCALE_MAX_MIN) g_rpmScaleMax = RPM_SCALE_MAX_MIN;
  if (g_rpmScaleMax > RPM_SCALE_MAX_MAX) g_rpmScaleMax = RPM_SCALE_MAX_MAX;
  g_rpmScaleMax = (g_rpmScaleMax / 100) * 100;
  if (g_rpmRedline < RPM_REDLINE_MIN) g_rpmRedline = RPM_REDLINE_MIN;
  if (g_rpmRedline > RPM_REDLINE_MAX) g_rpmRedline = RPM_REDLINE_MAX;
  if (g_rpmRedline > g_rpmScaleMax) g_rpmRedline = g_rpmScaleMax;
  g_rotationDeg %= 360;
  if (g_rotationDeg < 0) g_rotationDeg += 360;
  updateRotationCache();
}

static void saveDialScale(int pct) {
  if (pct < DIAL_SCALE_MIN) pct = DIAL_SCALE_MIN;
  if (pct > DIAL_SCALE_MAX) pct = DIAL_SCALE_MAX;
  g_dialScalePct = pct;
  requestDialCacheRebuild();
  Preferences p;
  p.begin("clock", false);
  p.putInt("scale", pct);
  p.end();
}

static void saveDialOffset(int offX, int offY) {
  if (offX < -20) offX = -20;
  if (offX >  20) offX =  20;
  if (offY < -20) offY = -20;
  if (offY >  20) offY =  20;
  g_dialCenterOffsetX = offX;
  g_dialCenterOffsetY = offY;
  requestDialCacheRebuild();
  Preferences p;
  p.begin("clock", false);
  p.putInt("dial_off_x", offX);
  p.putInt("dial_off_y", offY);
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

static void saveRpmRedline(int rpm) {
  if (rpm < RPM_REDLINE_MIN) rpm = RPM_REDLINE_MIN;
  if (rpm > RPM_REDLINE_MAX) rpm = RPM_REDLINE_MAX;
  if (rpm > g_rpmScaleMax) rpm = g_rpmScaleMax;
  rpm = (rpm / 100) * 100;
  g_rpmRedline = rpm;
  Preferences p;
  p.begin("clock", false);
  p.putInt("rpm_red", rpm);
  p.end();
}

static void saveNightMode(bool enabled) {
  g_nightMode = enabled;
  requestDialCacheRebuild();
  Preferences p;
  p.begin("clock", false);
  p.putBool("night", enabled);
  p.end();
}

static void saveRpmScaleMax(int rpm) {
  if (rpm < RPM_SCALE_MAX_MIN) rpm = RPM_SCALE_MAX_MIN;
  if (rpm > RPM_SCALE_MAX_MAX) rpm = RPM_SCALE_MAX_MAX;
  rpm = (rpm / 100) * 100;
  g_rpmScaleMax = rpm;
  if (g_rpmRedline > g_rpmScaleMax) g_rpmRedline = g_rpmScaleMax;
  Preferences p;
  p.begin("clock", false);
  p.putInt("rpm_max", g_rpmScaleMax);
  p.putInt("rpm_red", g_rpmRedline);
  p.end();
}

static void saveMotorStyle(uint8_t style) {
  (void)style;
  g_motorStyle = MOTOR_STYLE_VDO;
  Preferences p;
  p.begin("clock", false);
  p.putUChar("motor_style", g_motorStyle);
  p.end();
}

static void saveDataPath(DataPath path) {
  if (path != DATA_PATH_BLE) path = DATA_PATH_WIFI_HUB;
  if (path == g_dataPath) return;
  g_dataPath = path;
  clearLiveValues();
  Preferences p;
  p.begin("clock", false);
  p.putUChar("data_path", g_dataPath);
  p.end();
  DLOG("Hub: Datenweg -> %s\n", dataPathLabel());
}

static void saveHubHost(const char* host) {
  if (!host) return;
  String h = String(host);
  h.trim();
  if (h.length() == 0) h = HUB_WIFI_DEFAULT_HOST;
  if (h.length() >= sizeof(g_hubHost)) h = h.substring(0, sizeof(g_hubHost) - 1);
  if (h.equals(g_hubHost)) return;
  strncpy(g_hubHost, h.c_str(), sizeof(g_hubHost) - 1);
  g_hubHost[sizeof(g_hubHost) - 1] = 0;
  g_hubWifiOk = false;
  Preferences p;
  p.begin("clock", false);
  p.putString("hub_host", g_hubHost);
  p.end();
  DLOG("Hub: Host -> %s\n", g_hubHost);
}

static void updateRotationCache() {
  g_rotationDeg %= 360;
  if (g_rotationDeg < 0) g_rotationDeg += 360;
  float rad = (float)g_rotationDeg * PI / 180.0f;
  g_rotSin = sinf(rad);
  g_rotCos = cosf(rad);
#if FEATURE_VERBOSE_SERIAL
  if (g_rotationDeg != g_logLastRotDeg) {
    Serial.printf("[ROT] deg=%d sin=%.4f cos=%.4f\n", g_rotationDeg, g_rotSin, g_rotCos);
    g_logLastRotDeg = g_rotationDeg;
  }
#endif
}

static void saveRotation(int deg) {
  g_rotationDeg = deg;
  updateRotationCache();
  requestDialCacheRebuild();
  Preferences p;
  p.begin("clock", false);
  p.putInt("rot_deg", g_rotationDeg);
  p.end();
}

static void saveImuTrim() {
  Preferences p;
  p.begin("clock", false);
  p.putFloat("imu_off_p", g_imuOffPitch);
  p.putFloat("imu_off_r", g_imuOffRoll);
  p.putBool("imu_trim", g_imuTrimmed);
  p.end();
}

static bool calibrateImuZero() {
  if (!g_imuPresent) return false;
  if (!qmi8658Zero()) return false;
  saveImuTrim();
  Serial.printf("IMU: NULL gesetzt P=%.1f R=%.1f\n", g_imuOffPitch, g_imuOffRoll);
  return true;
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
    g_wifiApOnly = false;
    WiFi.disconnect();
    if (g_apOn) {
      WiFi.softAPdisconnect(true);
      g_apOn = false;
    }
    strcpy(g_ipStr, "---");
    g_webStarted = false;
    Preferences pw;
    pw.begin("clock", false);
    pw.putBool("wifi_ap_only", false);
    pw.end();
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
static String jsonEscape(const String& value) {
  String out;
  out.reserve(value.length() + 4);
  for (size_t i = 0; i < value.length(); i++) {
    const char c = value[i];
    if (c == '\\' || c == '"') {
      out += '\\';
      out += c;
    } else if (c == '\n') {
      out += F("\\n");
    } else if (c == '\r') {
      out += F("\\r");
    } else if (c == '\t') {
      out += F("\\t");
    } else {
      out += c;
    }
  }
  return out;
}

static void jsonAppendFloat(String& json, float v, int decimals) {
  if (!isfinite(v)) json += F("0");
  else json += String(v, decimals);
}

static const char* webPageLabel(uint8_t page) {
  switch (page) {
    case 0: return "UHR";
    case 1: return "MENU";
    case 2: return "MOTOR";
    case 3: return "LAMBDA";
    case 4: return "HUB";
    case 5: return "SETUP 1";
    case 6: return "IMU";
    case 7: return "OEL";
    case 8: return "DZM UHR";
    case 9: return "SETUP 2";
    default: return "???";
  }
}

static const char* webPageAccent(uint8_t page) {
  switch (page) {
    case 2: return "#2896d2";
    case 3: return "#3cb95a";
    case 4: return "#be5ad2";
    case 5: return "#d2bc2d";
    case 6: return "#c86432";
    case 7: return "#e68728";
    case 8: return "#d8c172";
    case 9: return "#b99132";
    default: return "#505050";
  }
}

static void webAppendVdoHands(String& svg, const struct tm* now) {
  const float secVal  = (float)now->tm_sec;
  const float minVal  = (float)now->tm_min + secVal / 60.0f;
  const float hourVal = (float)(now->tm_hour % 12) + minVal / 60.0f;
  char hDeg[12], mDeg[12], sDeg[12];
  snprintf(hDeg, sizeof(hDeg), "%.2f", hourVal * 30.0f);
  snprintf(mDeg, sizeof(mDeg), "%.2f", minVal * 6.0f);
  snprintf(sDeg, sizeof(sDeg), "%.2f", secVal * 6.0f);

  svg += F("<g class='preview-hands' transform='translate(120 120)'>"
           "<g class='hand-h' transform='rotate(");
  svg += hDeg;
  svg += F(")'><rect x='-5' y='-30' width='10' height='30' rx='3' fill='#181816'/>"
           "<rect x='-4' y='-30' width='8' height='30' rx='3' fill='#deded6'/></g>"
           "<g class='hand-m' transform='rotate(");
  svg += mDeg;
  svg += F(")'><rect x='-4' y='-43' width='8' height='43' rx='2' fill='#181816'/>"
           "<rect x='-3' y='-43' width='6' height='43' rx='2' fill='#e2e2da'/></g>"
           "<g class='hand-s' transform='rotate(");
  svg += sDeg;
  svg += F(")'><line x1='0' y1='10' x2='0' y2='-47' stroke='#eb1814' stroke-width='2.5' stroke-linecap='round'/></g>"
           "<circle r='13' fill='#cdcbc2'/><circle r='8' fill='#a67a2a'/><circle r='5' fill='#261e12'/></g>");
}

static void webAppendClockSvg(String& svg, const struct tm* now) {
  svg += F("<svg class='preview-svg' viewBox='0 0 240 240' aria-label='VDO Uhr'>"
           "<defs><radialGradient id='vdoFace' cx='50%' cy='42%' r='58%'>"
           "<stop offset='0%' stop-color='#141210'/><stop offset='55%' stop-color='#070707'/><stop offset='100%' stop-color='#020202'/></radialGradient></defs>"
           "<circle cx='120' cy='120' r='112' fill='#0a0a0a' stroke='#3a3a3a' stroke-width='3'/>"
           "<circle cx='120' cy='120' r='104' fill='url(#vdoFace)' stroke='#d8d0bc' stroke-width='2'/>"
           "<g stroke='#8f8878' stroke-linecap='round'>");
  for (int i = 0; i < 60; i++) {
    const bool major = (i % 5) == 0;
    const float rad = (float)i * PI / 30.0f;
    const float r0  = major ? 86.0f : 91.0f;
    const float r1  = 98.0f;
    char tick[96];
    snprintf(tick, sizeof(tick),
             "<line x1='%.1f' y1='%.1f' x2='%.1f' y2='%.1f' stroke-width='%d'/>",
             120.0f + sinf(rad) * r0, 120.0f - cosf(rad) * r0,
             120.0f + sinf(rad) * r1, 120.0f - cosf(rad) * r1,
             major ? 2 : 1);
    svg += tick;
  }
  svg += F("</g><g fill='#ddd8cc' font-family='Georgia,Times New Roman,serif' font-weight='700' text-anchor='middle'>");
  for (int n = 1; n <= 12; n++) {
    const float rad = (float)n * PI / 6.0f;
    char num[64];
    snprintf(num, sizeof(num),
             "<text x='%.1f' y='%.1f' font-size='15'>%d</text>",
             120.0f + sinf(rad) * 72.0f, 120.0f - cosf(rad) * 72.0f + 5.0f, n);
    svg += num;
  }
  svg += F("</g><text x='120' y='58' text-anchor='middle' fill='#c8c0aa' font-family='Georgia,serif' font-size='13' letter-spacing='2'>VDO</text>"
           "<text x='120' y='152' text-anchor='middle' fill='#8a8474' font-family='system-ui,sans-serif' font-size='8' letter-spacing='1.5'>QUARTZ-ZEIT</text>");
  webAppendVdoHands(svg, now);
  svg += F("</svg>");
}

static void webAppendPageMockSvg(String& svg, uint8_t page) {
  svg += F("<svg class='preview-svg' viewBox='0 0 240 240' aria-label='Display Seite'>"
           "<circle cx='120' cy='120' r='112' fill='#050505' stroke='#333' stroke-width='3'/>"
           "<circle cx='120' cy='120' r='104' fill='#080808' stroke='");
  svg += webPageAccent(page);
  svg += F("' stroke-width='3'/><text x='120' y='58' text-anchor='middle' fill='");
  svg += webPageAccent(page);
  svg += F("' font-family='system-ui,sans-serif' font-size='18' font-weight='700'>");
  svg += webPageLabel(page);
  svg += F("</text>");

  if (page == 1) {
    const char* items[] = {"UHR", "MOTOR", "LAMBDA", "HUB", "IMU", "OEL", "SETUP 1", "DZM UHR", "SETUP 2"};
    const char* cols[] = {"#c82823", "#2896d2", "#3cb95a", "#be5ad2", "#c86432", "#e68728", "#d2bc2d", "#d8c172", "#b99132"};
    for (int i = 0; i < 9; i++) {
      const int row = i / 2;
      const int col = i % 2;
      const int tx = 42 + col * 78;
      const int ty = 68 + row * 29;
      char rowBuf[160];
      snprintf(rowBuf, sizeof(rowBuf),
               "<rect x='%d' y='%d' width='72' height='26' rx='3' fill='#121212' stroke='#333'/>"
               "<rect x='%d' y='%d' width='4' height='26' fill='%s'/>"
               "<text x='%d' y='%d' fill='#ddd' font-family='system-ui,sans-serif' font-size='8'>%s</text>",
               tx, ty, tx, ty, cols[i], tx + 10, ty + 17, items[i]);
      svg += rowBuf;
    }
  } else if (page == 2) {
    const bool fresh = dataFresh();
    char line[32];
    svg += F("<text x='52' y='108' fill='#888' font-size='9' font-family='system-ui,sans-serif'>RPM</text>"
             "<text x='188' y='108' text-anchor='end' fill='#eee' font-size='10' class='pv-rpm' font-family='system-ui,sans-serif'>");
    snprintf(line, sizeof(line), "%d", fresh ? (int)g_rpm : 0);
    svg += line;
    svg += F("</text><text x='52' y='128' fill='#888' font-size='9' font-family='system-ui,sans-serif'>ADV</text>"
             "<text x='188' y='128' text-anchor='end' fill='#eee' font-size='10' class='pv-adv' font-family='system-ui,sans-serif'>");
    snprintf(line, sizeof(line), "%.0f", fresh ? g_adv : 0.0f);
    svg += line;
    svg += F("</text><text x='52' y='148' fill='#888' font-size='9' font-family='system-ui,sans-serif'>MAP</text>"
             "<text x='188' y='148' text-anchor='end' fill='#eee' font-size='10' class='pv-map' font-family='system-ui,sans-serif'>");
    snprintf(line, sizeof(line), "%d", fresh ? (int)g_map : 0);
    svg += line;
    svg += F("</text><text x='52' y='168' fill='#888' font-size='9' font-family='system-ui,sans-serif'>BATT</text>"
             "<text x='188' y='168' text-anchor='end' fill='#eee' font-size='10' class='pv-batt' font-family='system-ui,sans-serif'>");
    if (fresh && g_battValid) snprintf(line, sizeof(line), "%.1fV", g_battVolt);
    else strcpy(line, "---");
    svg += line;
    svg += F("</text><text x='120' y='192' text-anchor='middle' fill='#888' font-size='9' class='pv-motor-st' font-family='system-ui,sans-serif'>");
    svg += fresh ? F("LIVE") : F("---");
    svg += F("</text>");
  } else if (page == 3) {
    char line[24];
    if (g_lambdaValid) snprintf(line, sizeof(line), "%.2f", g_lambda);
    else strcpy(line, "----");
    svg += F("<text x='120' y='132' text-anchor='middle' fill='#46d06a' font-size='28' font-family='system-ui,sans-serif' font-weight='700' class='pv-lambda'>");
    svg += line;
    svg += F("</text>");
  } else if (page == 4) {
    svg += F("<text x='120' y='108' text-anchor='middle' fill='#");
    svg += hubLinkOk() ? F("54d273") : F("e7a944");
    svg += F("' font-size='14' font-family='system-ui,sans-serif' class='pv-hub-st'>");
    svg += hubLinkOk() ? F("HUB OK") : F("KEIN HUB");
    svg += F("</text><text x='120' y='128' text-anchor='middle' fill='#aaa' font-size='9' font-family='system-ui,sans-serif'>");
    svg += dataPathLabel();
    svg += F("</text><text x='120' y='148' text-anchor='middle' fill='#aaa' font-size='10' font-family='system-ui,sans-serif'>RX <tspan class='pv-rx'>");
    svg += String((unsigned long)g_liveRxCnt);
    svg += F("</tspan></text>");
  } else if (page == 5) {
    svg += F("<text x='72' y='104' fill='#888' font-size='9' font-family='system-ui,sans-serif'>UHR</text>"
             "<text x='168' y='104' text-anchor='end' fill='#eee' font-size='10' font-family='system-ui,sans-serif'>");
    svg += String(g_dialScalePct);
    svg += F("%</text><text x='72' y='124' fill='#888' font-size='9' font-family='system-ui,sans-serif'>HELL</text>"
             "<text x='168' y='124' text-anchor='end' fill='#eee' font-size='10' font-family='system-ui,sans-serif'>");
    svg += String(g_brightnessPct);
    svg += F("%</text><text x='72' y='144' fill='#888' font-size='9' font-family='system-ui,sans-serif'>ROT</text>"
             "<text x='168' y='144' text-anchor='end' fill='#eee' font-size='10' font-family='system-ui,sans-serif'>");
    svg += String(g_rotationDeg);
    svg += F("</text><text x='72' y='164' fill='#888' font-size='9' font-family='system-ui,sans-serif'>WLAN</text>"
             "<text x='168' y='164' text-anchor='end' fill='#eee' font-size='10' font-family='system-ui,sans-serif'>");
    svg += g_featureWifi ? F("an") : F("aus");
    svg += F("</text><text x='72' y='184' fill='#888' font-size='9' font-family='system-ui,sans-serif'>NACHT</text>"
             "<text x='168' y='184' text-anchor='end' fill='#eee' font-size='10' font-family='system-ui,sans-serif'>");
    svg += g_nightMode ? F("gruen") : F("tag");
    svg += F("</text>");
  } else if (page == 9) {
    svg += F("<text x='72' y='108' fill='#888' font-size='9' font-family='system-ui,sans-serif'>BLE</text>"
             "<text x='168' y='118' text-anchor='end' fill='#eee' font-size='10' font-family='system-ui,sans-serif'>");
    svg += g_featureBle ? F("an") : F("aus");
    svg += F("</text><text x='72' y='138' fill='#888' font-size='9' font-family='system-ui,sans-serif'>ESPNOW</text>"
             "<text x='168' y='138' text-anchor='end' fill='#eee' font-size='10' font-family='system-ui,sans-serif'>");
    svg += g_featureEspNow ? F("an") : F("aus");
    svg += F("</text><text x='72' y='158' fill='#888' font-size='9' font-family='system-ui,sans-serif'>CH</text>"
             "<text x='168' y='158' text-anchor='end' fill='#eee' font-size='10' font-family='system-ui,sans-serif'>");
    svg += espNowChannelLabel();
    svg += F("</text><text x='72' y='178' fill='#888' font-size='9' font-family='system-ui,sans-serif'>QUELLE</text>"
             "<text x='168' y='178' text-anchor='end' fill='#eee' font-size='10' font-family='system-ui,sans-serif'>");
    svg += sourceModeLabel();
    svg += F("</text>");
  } else if (page == 6) {
    if (g_imuPresent) {
      char line[24];
      if (g_imuTrimmed) {
        svg += F("<text x='120' y='98' text-anchor='middle' fill='#54d273' font-size='8' font-family='system-ui,sans-serif'>NULL</text>");
      }
      snprintf(line, sizeof(line), "P %.1f", g_imuPitch);
      svg += F("<text x='120' y='118' text-anchor='middle' fill='#eee' font-size='11' class='pv-imu-p' font-family='system-ui,sans-serif'>");
      svg += line;
      snprintf(line, sizeof(line), "R %.1f", g_imuRoll);
      svg += F("</text><text x='120' y='142' text-anchor='middle' fill='#eee' font-size='11' class='pv-imu-r' font-family='system-ui,sans-serif'>");
      svg += line;
      svg += F("</text>");
    } else {
      svg += F("<text x='120' y='132' text-anchor='middle' fill='#c84040' font-size='12' font-family='system-ui,sans-serif'>KEIN IMU</text>");
    }
  } else if (page == 7) {
    const bool fresh = dataFresh();
    char line[24];
    svg += F("<circle cx='120' cy='128' r='72' fill='#151515' stroke='#d8cbb0' stroke-width='2'/>"
             "<path d='M62 128 A58 58 0 0 1 178 128' fill='none' stroke='#cfc2a8' stroke-width='4'/>"
             "<path d='M164 104 A58 58 0 0 1 178 128' fill='none' stroke='#d94b25' stroke-width='7'/>"
             "<line x1='120' y1='128' x2='74' y2='140' stroke='#f0b000' stroke-width='5' stroke-linecap='round'/>"
             "<circle cx='120' cy='128' r='10' fill='#202020' stroke='#555'/>"
             "<circle cx='120' cy='166' r='24' fill='#111' stroke='#666'/>");
    snprintf(line, sizeof(line), "%d RPM", fresh ? (int)g_rpm : 0);
    svg += F("<text x='120' y='92' text-anchor='middle' fill='#eee' font-size='11' class='pv-rpm' font-family='system-ui,sans-serif'>");
    svg += fresh ? line : "--- RPM";
    svg += F("</text>");
  }
  svg += F("</svg>");
}

static String webDisplayPreviewInner(const struct tm* now) {
  String s;
  s.reserve(4096);
  s += F("<div class='preview-content' style='--scale:");
  s += String(g_dialScalePct);
  s += F("'>");
  if (currentPage == 0) webAppendClockSvg(s, now);
  else webAppendPageMockSvg(s, currentPage);
  s += F("</div>");
  return s;
}

static String webDisplayPreviewShell(const struct tm* now) {
  String s;
  s.reserve(4300);
  s += F("<div class='preview-shell'><div class='preview-bezel'>");
  s += webDisplayPreviewInner(now);
  s += F("</div><div class='preview-meta' id='previewMeta'>");
  s += webPageLabel(currentPage);
  s += " &middot; ";
  s += String(g_dialScalePct);
  s += F("% &middot; ");
  s += String(g_rotationDeg);
  s += F("&deg;</div></div>");
  return s;
}

static void handleWebPreview() {
  if (webOtaRejectBusy()) return;
  struct tm now = {};
  readClockTime(&now);
  webServer.send(200, "text/html", webDisplayPreviewShell(&now));
}

static void handleWebRoot() {
  if (g_otaBusy) {
    webServer.send(503, "text/html",
                    F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
                      "<title>OTA</title></head><body><h1>OTA Upload laeuft...</h1><p>Bitte warten.</p></body></html>"));
    return;
  }
  struct tm now = {};
  readClockTime(&now);
  char timeStr[16];
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", now.tm_hour, now.tm_min, now.tm_sec);

  String html;
  html.reserve(32768);
  html += F("<!DOCTYPE html><html lang='de'><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>VDO Cockpit</title><style>"
    ":root{color-scheme:dark;--bg:#070707;--panel:#151515;--line:#2b2b2b;--gold:#e0c040;--muted:#9a9a9a;--ok:#54d273;--warn:#e7a944}"
    "*{box-sizing:border-box}body{font-family:system-ui,-apple-system,Segoe UI,sans-serif;background:radial-gradient(circle at top,#202020,#070707 60%);color:#eee;margin:0;padding:16px}"
    "main{max-width:760px;margin:0 auto}h1{margin:10px 0 4px;color:var(--gold);font-weight:700}.sub{color:var(--muted);margin-bottom:16px}"
    ".tabs{display:flex;gap:7px;overflow:auto;margin-bottom:12px}.tabs label{background:#202020;border:1px solid var(--line);border-radius:999px;padding:10px 13px;white-space:nowrap;cursor:pointer}"
    "input.tab{display:none}.page{display:none}.card{background:rgba(21,21,21,.95);border:1px solid var(--line);border-radius:18px;padding:17px;margin:12px 0;box-shadow:0 10px 30px #0005}"
    "#t0:checked~.tabs label[for=t0],#t1:checked~.tabs label[for=t1],#t2:checked~.tabs label[for=t2],#t3:checked~.tabs label[for=t3],#t4:checked~.tabs label[for=t4],#t5:checked~.tabs label[for=t5]{background:var(--gold);color:#161100;border-color:var(--gold)}"
    "#t0:checked~#p0,#t1:checked~#p1,#t2:checked~#p2,#t3:checked~#p3,#t4:checked~#p4,#t5:checked~#p5{display:block}"
    ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(145px,1fr));gap:10px}.metric{background:#0c0c0c;border:1px solid #262626;border-radius:14px;padding:14px;text-align:left}"
    ".metric b{display:block;font-size:1.7rem;color:#fff}.metric span{color:var(--muted);font-size:.9rem}.ok{color:var(--ok)}.warn{color:var(--warn)}"
    "button{background:var(--gold);border:0;border-radius:10px;padding:11px 15px;margin:5px 4px;color:#171200;font-weight:700;cursor:pointer}a{color:#9bd1ff;text-decoration:none}"
    "input[type=range]{width:100%}select{width:100%;max-width:100%;padding:11px 12px;background:#0d0d0d;border:1px solid #333;border-radius:10px;color:#eee;font-size:1rem}"
    ".row{display:flex;gap:10px;align-items:center;justify-content:space-between;flex-wrap:wrap}.pill{border:1px solid #333;border-radius:999px;padding:6px 10px;color:#ccc}"
    ".preview-shell{text-align:center;margin:8px auto 16px}.preview-bezel{width:min(78vw,300px);aspect-ratio:1;margin:0 auto;padding:12px;border-radius:50%;background:linear-gradient(145deg,#3a3a3a,#151515);box-shadow:0 16px 36px #0009,inset 0 2px 8px #ffffff14;display:flex;align-items:center;justify-content:center}.preview-content{width:100%;height:100%;transform:scale(calc(var(--scale,100)/100));transform-origin:center}.preview-svg{width:100%;height:auto;display:block;border-radius:50%}.preview-meta{margin-top:10px;color:var(--muted);font-size:.95rem;letter-spacing:.04em}.preview-shell--sm .preview-bezel{width:min(42vw,160px);padding:8px}"
    "table{width:100%;border-collapse:collapse}td,th{padding:9px;border-bottom:1px solid #292929;text-align:left}.file{width:100%;padding:12px;background:#0d0d0d;border:1px solid #333;border-radius:10px;color:#eee}"
    ".ota-progress{margin-top:12px}.ota-track{height:10px;background:#222;border-radius:999px;overflow:hidden;border:1px solid #333}.ota-bar{height:100%;width:0;background:linear-gradient(90deg,#b8921f,var(--gold));transition:width .25s}</style></head><body><main>");
  html += F("<h1>VDO Cockpit</h1><div class='sub'>ESP32-S3 WebGUI &middot; IP ");
  html += String(g_ipStr);
  html += F(" &middot; <span id='headerTime'>");
  html += String(timeStr);
  html += F("</span></div><input class='tab' id='t0' name='tab' type='radio' checked><input class='tab' id='t1' name='tab' type='radio'><input class='tab' id='t2' name='tab' type='radio'><input class='tab' id='t3' name='tab' type='radio'><input class='tab' id='t4' name='tab' type='radio'><input class='tab' id='t5' name='tab' type='radio'>"
            "<nav class='tabs'><label for='t0'>Dashboard</label><label for='t1'>WLAN</label><label for='t2'>Display</label><label for='t3'>Live</label><label for='t4'>Setup</label><label for='t5'>Setup 2</label></nav>");

  html += F("<section class='page' id='p0'><div class='card row'><div><b>Aktive Display-Seite</b><br><span id='dashPageLabel' style='font-size:1.4rem;color:var(--gold)'>");
  html += webPageLabel(currentPage);
  html += F("</span></div><div class='pill' id='dashPage'>Seite ");
  html += String(currentPage);
  html += F("</div></div><div class='card preview-shell preview-shell--sm' id='dashPreviewHost'><div class='preview-bezel'>");
  html += webDisplayPreviewInner(&now);
  html += F("</div></div><div class='card'><div class='grid'>");
  html += "<div class='metric'><span>RPM</span><b id='dashRpm'>" + String((int)g_rpm) + "</b></div>";
  html += "<div class='metric'><span>ADV</span><b id='dashAdv'>" + String(g_adv, 1) + "&deg;</b></div>";
  html += "<div class='metric'><span>Lambda</span><b id='dashLambda'>" + String(g_lambdaValid ? String(g_lambda, 2) : String("---")) + "</b></div>";
  html += "<div class='metric'><span>Volt</span><b id='dashVolt'>" + String(g_battValid ? String(g_battVolt, 1) : String("---")) + "</b></div>";
  html += F("</div></div><div class='card row'><div><b>Hub Link</b><br><span id='dashBle' class='");
  html += hubLinkOk() ? "ok" : "warn";
  html += "'>" + String(g_dataPath == DATA_PATH_WIFI_HUB ?
       (g_hubWifiOk ? "WiFi OK" : "WiFi wartet") :
       (g_featureBle ? (g_bleConn ? "BLE OK" : "BLE scan") : "aus")) + "</span></div>";
  html += "<div class='pill'>" + String(dataPathLabel()) + " RX <span id='dashRx'>" +
          String((unsigned long)g_liveRxCnt) + "</span></div></div></section>";

  html += F("<section class='page' id='p1'><div class='card'><h2>WLAN</h2><div>Aktiv: <b>");
  html += String(currentWifiSsid());
  html += F("</b> &middot; ");
  html += WiFi.status() == WL_CONNECTED ? F("<span class='ok'>verbunden</span>") : F("<span class='warn'>nicht verbunden</span>");
  html += F("</div><div>");
  for (uint8_t i = 0; i < wifiProfileCount(); i++) {
    html += "<a href='/wifi?prof=" + String(i) + "'><button";
    if (i == g_wifiProfile) html += F(" style='background:#54d273'");
    html += ">" + String(WIFI_PROFILES[i].ssid) + "</button></a>";
  }
  html += F("</div><button onclick='scanWifi()'>&#128269; Scan</button><table><thead><tr><th>SSID</th><th>RSSI</th><th>Status</th><th></th></tr></thead><tbody id='scanRows'><tr><td colspan='4'>Noch kein Scan</td></tr></tbody></table><p class='sub'>Scan-Zeile antippen oder Profil oben w&auml;hlen. Spartan3-Setup = Hub-AP @ 192.168.4.1</p></div>"
            "<div class='card'><h3>Bus (unterwegs)</h3><p class='sub'>Fahrtprofil: BLE direkt zum 123, ESP-NOW Lambda vom Hub, HTTP nur Status/Zeit. Display @ 192.168.4.3 (M5 @ .2).</p><div class='row'><a href='/wifi?mode=bus'><button");
  if (isOnBusWifi() && g_dataPath == DATA_PATH_BLE && g_bleConnMode == BLE_MODE_DIRECT_123 && g_featureEspNow) html += F(" style='background:#54d273'");
  html += F(">BUS / Spartan3-Setup</button></a></div></div>"
            "<div class='card'><h3>Netzwerk-Modus</h3><p class='sub'>Home = Heimrouter/LAN &middot; AP = VDO-Clock-Setup / vdoclock @ 192.168.4.1 &middot; Hub = Spartan3-Setup / lambda123 @ 192.168.4.1</p><div class='row'>");
  html += F("<a href='/wifi?mode=home'><button");
  if (g_featureWifi && !g_wifiApOnly) html += F(" style='background:#54d273'");
  html += F(">Home WiFi</button></a><a href='/wifi?mode=ap'><button");
  if (g_wifiApOnly) html += F(" style='background:#54d273'");
  html += F(">AP Setup</button></a><a href='/wifi?mode=off'><button");
  if (!g_featureWifi && !g_wifiApOnly) html += F(" style='background:#54d273'");
  html += F(">Aus</button></a></div></div></section>");

  html += F("<section class='page' id='p2'><div class='card'><h2>Display</h2><div id='displayPreviewHost'>");
  html += webDisplayPreviewShell(&now);
  html += F("</div><div class='grid'>"
            "<a href='/page?p=0'><button>Uhr</button></a><a href='/page?p=1'><button>Menu</button></a><a href='/page?p=2'><button>Motor</button></a><a href='/page?p=8'><button>DZM Uhr</button></a><a href='/page?p=7'><button>Oel</button></a><a href='/page?p=3'><button>Lambda</button></a><a href='/page?p=4'><button>Hub</button></a><a href='/page?p=6'><button>IMU</button></a><a href='/page?p=5'><button>Setup 1</button></a><a href='/page?p=9'><button>Setup 2</button></a></div></div>"
            "<div class='card'><h3>Zifferblatt-Groesse</h3><form action='/set' method='get'><div class='row'><b><span id='scaleVal'>");
  html += String(g_dialScalePct);
  html += F("</span>%</b><button type='submit'>Uebernehmen</button></div><input type='range' name='scale' min='30' max='120' step='1' value='");
  html += String(g_dialScalePct);
  html += F("' oninput=\"scaleVal.innerText=this.value\"></form><div class='row'><span><a href='/set?scale_delta=-5'><button>-5%</button></a><a href='/set?scale_delta=-1'><button>-1%</button></a><a href='/set?scale_delta=1'><button>+1%</button></a><a href='/set?scale_delta=5'><button>+5%</button></a></span></div></div><div class='card'><h3>Rotation</h3><div class='row'><b>");
  html += String(g_rotationDeg);
  html += F("&deg;</b><span><a href='/set?rot_delta=-5'><button>-5&deg;</button></a><a href='/set?rot_delta=-1'><button>-1&deg;</button></a><a href='/set?rot_delta=1'><button>+1&deg;</button></a><a href='/set?rot_delta=5'><button>+5&deg;</button></a></span></div><div class='row'><span><a href='/set?rot=0'><button>0&deg;</button></a><a href='/set?rot=90'><button>90&deg;</button></a><a href='/set?rot=180'><button>180&deg;</button></a><a href='/set?rot=270'><button>270&deg;</button></a></span></div></div><div class='card'><h3>Zifferblatt-Position</h3><p class='sub'>Nur Bitmap, Zeiger bleiben in der Mitte.</p><div class='row'><b>X ");
  html += String(g_dialCenterOffsetX);
  html += F(" &middot; Y ");
  html += String(g_dialCenterOffsetY);
  html += F("</b></div><div class='row'><span><a href='/set?dial_x_delta=-1'><button>X -1</button></a><a href='/set?dial_x_delta=1'><button>X +1</button></a></span></div><div class='row'><span><a href='/set?dial_y_delta=-1'><button>Y -1</button></a><a href='/set?dial_y_delta=1'><button>Y +1</button></a><a href='/set?dial_reset=1'><button>Zentrum</button></a></span></div></div>"
            "<div class='card'><h3>Drehzahlmesser</h3><p class='sub'>VDO Einzeldrehzahlmesser. Skalenende 6000 passt fuer den Bulli.</p>"
            "<form action='/set' method='get'><div class='row'><label>Rot ab "
            "<input type='number' name='rpm_redline' min='2000' max='8000' step='100' value='");
  html += String(g_rpmRedline);
  html += F("'></label></div><div class='row'><label>Skala bis "
            "<input type='number' name='rpm_max' min='4000' max='8000' step='100' value='");
  html += String(g_rpmScaleMax);
  html += F("'></label><button type='submit'>Uebernehmen</button></div></form>"
            "<p class='sub'>Aktiv: <b>VDO</b> &middot; Anzeige in x100 RPM.</p></div>"
            "<div class='card'><h3>Beleuchtung</h3><p class='sub'>Nachtmodus: matte gruene Instrumentenbeleuchtung wie LED im T2B-Cockpit.</p><div class='row'><a href='/set?night=0'><button");
  if (!g_nightMode) html += F(" style='background:#54d273'");
  html += F(">Tag</button></a><a href='/set?night=1'><button");
  if (g_nightMode) html += F(" style='background:#54d273'");
  html += F(">Nacht gruen</button></a></div></div></section>");

  html += F("<section class='page' id='p3'><div class='card'><h2>Live</h2><pre id='liveBox'>Lade /api/status ...</pre></div></section>");

  html += F("<section class='page' id='p4'><div class='card'><h2>Setup: Daten & Funk</h2><div class='grid'>"
            "<div class='metric'><span>ESP-NOW</span><b id='setupEspnowOn'>-</b></div>"
            "<div class='metric'><span>Kanal</span><b id='setupEspnowCh'>-</b></div>"
            "<div class='metric'><span>RX / Seq</span><b id='setupEspnowRx'>-</b></div>"
            "<div class='metric'><span>Fresh</span><b id='setupEspnowFresh'>-</b></div>"
            "</div><p class='sub'>Bus-Betrieb: Hub, M5 und 2.8C auf denselben ESP-NOW-Kanal stellen. Meist: Bus (Kanal 6).</p>"
            "<form action='/features' method='get'>"
            "<p><label><input type='checkbox' name='wifi' value='1' ");
  html += g_featureWifi ? F("checked") : F("");
  html += F("> WLAN/Web aktiv</label></p><p><label><input type='checkbox' name='ble' value='1' ");
  html += g_featureBle ? F("checked") : F("");
  html += F("> BLE Daten aktiv</label></p><p><label><input type='checkbox' name='buzzer' value='1' ");
  html += g_featureBuzzer ? F("checked") : F("");
  html += F("> Buzzer aktiv</label></p><p><label><input type='checkbox' name='espnow' value='1' ");
  html += g_featureEspNow ? F("checked") : F("");
  html += F("> ESP-NOW Lambda</label></p><p>ESP-NOW Kanal: <select name='espnow_ch'><option value='0'");
  if (g_espNowChannelPref == 0) html += F(" selected");
  html += F(">Automatisch (folgt WLAN)</option><option value='6'");
  if (g_espNowChannelPref == 6) html += F(" selected");
  html += F(">Bus (Kanal 6 / Spartan3-Setup)</option><option value='11'");
  if (g_espNowChannelPref == 11) html += F(" selected");
  html += F(">Handy-Test (Kanal 11)</option></select></p><p class='sub'>Automatisch folgt WLAN. Bus fixiert Kanal 6. Handy-Test fixiert Kanal 11.</p><p>Hub IP: <input name='hub_host' value='");
  html += String(g_hubHost);
  html += F("' style='width:100%;max-width:220px;padding:8px;background:#0d0d0d;border:1px solid #333;border-radius:8px;color:#eee'></p>"
            "<p>Fallback wenn ESP-NOW nicht frisch ist: <select name='data_path'><option value='wifi'");
  if (g_dataPath == DATA_PATH_WIFI_HUB) html += F(" selected");
  html += F(">Hub HTTP /api/status</option><option value='ble'");
  if (g_dataPath == DATA_PATH_BLE) html += F(" selected");
  html += F(">Hub BLE</option></select></p><p class='sub'>Live bevorzugt ESP-NOW. HTTP/BLE sind nur Rueckfall und Diagnose.</p><p>Datenquelle (Kurz): "
            "<a href='/source?m=hub_wifi'><button");
  if (g_dataPath == DATA_PATH_WIFI_HUB) html += F(" style='background:#54d273'");
  html += F(">HTTP Fallback</button></a><a href='/source?m=hub_ble'><button");
  if (g_dataPath == DATA_PATH_BLE && g_bleConnMode == BLE_MODE_SPARTAN_HUB) html += F(" style='background:#54d273'");
  html += F(">BLE Fallback</button></a><a href='/source?m=123'><button");
  if (g_dataPath == DATA_PATH_BLE && g_bleConnMode == BLE_MODE_DIRECT_123) html += F(" style='background:#54d273'");
  html += F(">123 direkt</button></a></p><button type='submit'>Speichern</button></form></div></section>"
            "<section class='page' id='p5'><div class='card'><h2>Setup 2: System</h2><p class='sub'>Zeit, BLE-Ziele, IMU, OTA und Neustart.</p>"
            "<div class='row'><span><b>IMU Nullpunkt</b><br><span class='sub'>Display in Einbaulage ruhig halten, dann setzen.</span></span>"
            "<a href='/imu/zero'><button>IMU NULL SET</button></a></div></div>"
            "<div class='card'><h2>Zeitzone</h2><p class='sub'>Uhrzeit per NTP (WLAN). Aktuell: <b id='tzLabel'>");
  html += String(timezoneLabel(g_timezoneIdx));
  html += F("</b> &middot; ");
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", now.tm_hour, now.tm_min, now.tm_sec);
  html += String(timeStr);
  html += F("</p><form action='/tz' method='get'><select name='idx'>");
  for (uint8_t i = 0; i < TIMEZONE_COUNT; i++) {
    html += F("<option value='");
    html += String(i);
    html += "'";
    if (i == g_timezoneIdx) html += F(" selected");
    html += '>';
    html += String(TIMEZONES[i].label);
    html += F("</option>");
  }
  html += F("</select><p><button type='submit'>Speichern &amp; NTP sync</button></p></form></div>"
            "<div class='card'><h2>BLE Quelle</h2><div class='row'><span>Aktiv: <b id='bleModeLabel'>");
  html += String(bleConnModeLabel());
  html += F("</b></span><span><a href='/ble/mode?m=hub'><button");
  if (g_bleConnMode == BLE_MODE_SPARTAN_HUB) html += F(" style='background:#54d273'");
  html += F(">Spartan Hub</button></a><a href='/ble/mode?m=direct'><button");
  if (g_bleConnMode == BLE_MODE_DIRECT_123) html += F(" style='background:#54d273'");
  html += F(">123 direkt</button></a></span></div>"
            "<p class='sub'>Gleiche MAC fuer Hub und 123 moeglich (z.B. BM6). "
            "<b>Hub</b> = Spartan3-Hub-Protokoll (Motor/Lambda auf Hub-Seite). "
            "<b>123 direkt</b> = NUS/123TUNE+ (Motor-Seite). Modus waehlen, nicht nur MAC.</p>"
            "<form action='/ble/mac' method='get'><p>Hub-MAC: <input name='hub_mac' value='");
  html += String(g_bleHubMac[0] ? g_bleHubMac : DEFAULT_HUB_MAC);
  html += F("' style='width:11em;background:#111;border:1px solid #333;color:#eee;padding:6px;border-radius:8px'> "
            "123-MAC: <input name='direct_mac' value='");
  html += String(g_bleTargetMac);
  html += F("' style='width:11em;background:#111;border:1px solid #333;color:#eee;padding:6px;border-radius:8px'> "
            "<button type='submit'>MAC speichern</button></p></form>"
            "<p class='sub'>Aktiv: <span id='bleModeLabel2'>");
  html += String(bleConnModeLabel());
  html += F("</span> &middot; Ziel <span id='bleMacLabel'>");
  html += String(bleTargetDisplay());
  html += F("</span> (Hub: <span id='bleHubMacLabel'>");
  html += String(g_bleHubMac[0] ? g_bleHubMac : DEFAULT_HUB_MAC);
  html += F("</span>, 123: <span id='bleDirectMacLabel'>");
  html += String(g_bleTargetMac);
  html += F("</span>)</p><button onclick='scanBle()'>&#128269; BLE Scan (3s)</button><table><thead><tr><th>Name</th><th>MAC</th><th>RSSI</th><th>Typ</th><th></th></tr></thead><tbody id='bleScanRows'><tr><td colspan='5'>Noch kein Scan</td></tr></tbody></table></div>"
            "<div class='card'><h2>OTA Firmware</h2><p class='sub'>Kaputte Updates rollen automatisch zur vorherigen Firmware zurueck. PC-Backup: backups/esp32s3_vdo_backup_2026-06-10.bin (esptool).</p>"
            "<form id='otaForm' method='POST' action='/update' enctype='multipart/form-data'><input class='file' type='file' name='update' accept='.bin,application/octet-stream' required><button type='submit' id='otaBtn'>Firmware hochladen</button>"
            "<div class='ota-progress' id='otaProgress' hidden><div class='ota-track'><div class='ota-bar' id='otaBar'></div></div><p id='otaStatus' class='sub'>Upload laeuft...</p></div></form></div>"
            "<div class='card'><h2>Restart</h2><a href='/restart'><button>ESP32 neu starten</button></a></div></section>");

  html += F("<script>"
            "const pageNames=['UHR','MENU','MOTOR','LAMBDA','HUB','SETUP 1','IMU','OEL','DZM UHR','SETUP 2'];"
            "function num(v){const n=Number(v);return Number.isFinite(n)?n:0;}"
            "let lastPreviewPage=-2,espH=0,espM=0,espS=0,espSync=0,liveTimer=null,handTimer=null;"
            "function pvSet(sel,v){document.querySelectorAll(sel).forEach(e=>e.textContent=v);}"
            "function setHand(sel,deg){document.querySelectorAll(sel).forEach(g=>g.setAttribute('transform','rotate('+deg.toFixed(2)+')'));}"
            "function updateHands(){if(lastPreviewPage!==0)return;const t=Math.floor((Date.now()-espSync)/1000);"
            "const totalSec=espS+t,sec=totalSec%60,totalMin=espM+Math.floor(totalSec/60),min=totalMin%60,h=(espH+Math.floor(totalMin/60))%12;"
            "setHand('.hand-s',sec*6);setHand('.hand-m',min*6+sec*0.1);setHand('.hand-h',h*30+min*0.5);}"
            "function applyPreviewMeta(d){const meta=document.getElementById('previewMeta');"
            "if(meta)meta.textContent=(pageNames[d.page]||'?')+' · '+(d.scale||100)+'% · '+(d.rotation||0)+'°';"
            "const sv=document.getElementById('scaleVal');if(sv)sv.textContent=String(d.scale||100);"
            "document.querySelectorAll('.preview-content').forEach(el=>{el.style.setProperty('--scale',d.scale||100);});}"
            "function syncPreviewValues(d){applyPreviewMeta(d);espH=d.hour||0;espM=d.min||0;espS=d.sec||0;espSync=Date.now();"
            "if(d.page===0){updateHands();return;}if(d.page===2){const ok=d.data_fresh;"
            "pvSet('.pv-rpm',ok?String(Math.round(d.rpm||0)):'0');pvSet('.pv-adv',ok?String(Math.round(d.adv||0)):'0');"
            "pvSet('.pv-map',ok?String(Math.round(d.map||0)):'0');pvSet('.pv-batt',d.volt_valid?num(d.volt).toFixed(1)+'V':'---');"
            "pvSet('.pv-motor-st',ok?'LIVE':((d.hub_wifi_ok||d.ble_connected)?'WARTE':'---'));}"
            "if(d.page===3)pvSet('.pv-lambda',d.lambda_valid?num(d.lambda).toFixed(2):'----');"
            "if(d.page===4){pvSet('.pv-hub-st',(d.hub_wifi_ok||d.ble_connected)?'HUB OK':'KEIN HUB');pvSet('.pv-rx',String(d.live_rx||0));}"
            "if(d.page===6&&d.imu_present){pvSet('.pv-imu-p','P '+num(d.imu_pitch).toFixed(1));pvSet('.pv-imu-r','R '+num(d.imu_roll).toFixed(1));}}"
            "function syncPreviewContent(html){const m=html.match(/<div class='preview-content'[\\s\\S]*?<\\/div>/);if(!m)return;"
            "const inner=m[0].replace(/--rot:[^;'\"]+;?/g,'');"
            "const dash=document.getElementById('dashPreviewHost');if(dash){const b=dash.querySelector('.preview-bezel');if(b)b.innerHTML=inner;}}"
            "async function refreshPreview(){try{const r=await fetch('/api/preview');let html=await r.text();html=html.replace(/--rot:[^;'\"]+;?/g,'');"
            "const host=document.getElementById('displayPreviewHost');if(host)host.innerHTML=html;syncPreviewContent(html);}catch(e){}}"
            "function syncDashboard(d){if(!d)return;const set=(id,v)=>{const e=document.getElementById(id);if(e)e.textContent=v;};"
            "set('dashRpm',Math.round(num(d.rpm)));set('dashAdv',num(d.adv).toFixed(1)+'°');"
            "set('dashLambda',d.lambda_valid?num(d.lambda).toFixed(2):'---');set('dashVolt',d.volt_valid?num(d.volt).toFixed(1):'---');"
            "const ht=document.getElementById('headerTime');if(ht&&d.hour!=null)ht.textContent=String(d.hour).padStart(2,'0')+':'+String(d.min).padStart(2,'0')+':'+String(d.sec).padStart(2,'0');"
            "set('dashPage',String(d.page));set('dashPageLabel',pageNames[d.page]||'?');set('dashRx',String(d.live_rx||0));"
            "set('setupEspnowOn',d.esp_now_enabled?(d.esp_now_ready?'AN':'AN / wartet'):'AUS');"
            "set('setupEspnowCh',d.esp_now_channel_label||String(d.esp_now_channel||'-'));"
            "set('setupEspnowRx',String(d.esp_now_rx||0)+' / '+String(d.esp_now_seq||0));"
            "set('setupEspnowFresh',d.esp_now_fresh?'ja':'nein');"
            "const ble=document.getElementById('dashBle');if(ble){"
            "const ok=(d.data_path==='wifi'?d.hub_wifi_ok:(d.ble_enabled&&d.ble_connected));"
            "ble.textContent=d.data_path==='wifi'?(d.hub_wifi_ok?'WiFi OK':'WiFi wartet'):(d.ble_enabled?(d.ble_connected?'BLE OK':'BLE scan'):'aus');"
            "ble.className=ok?'ok':'warn';}"
            "syncBleSetup(d);"
            "if(d.page!==lastPreviewPage){lastPreviewPage=d.page;refreshPreview().then(()=>syncPreviewValues(d));}"
            "else syncPreviewValues(d);}"
            "async function fetchWithTimeout(url,ms,opts){const c=new AbortController();const t=setTimeout(()=>c.abort(),ms);try{return await fetch(url,{...(opts||{}),signal:c.signal});}finally{clearTimeout(t);}}"
            "function wifiScanRow(n){if(!n.ssid)return'';const href='/wifi?ssid='+encodeURIComponent(n.ssid);return`<tr class='wifi-scan-row' onclick=\"location.href='${href}'\" style='cursor:pointer'><td>${n.ssid}</td><td>${n.rssi}</td><td>${n.connected?'verbunden':''}</td><td><button type='button' onclick=\"event.stopPropagation();location.href='${href}'\">Verbinden</button></td></tr>`;}"
            "async function scanWifi(){const rows=document.getElementById('scanRows');rows.innerHTML='<tr><td colspan=4>Scan laeuft...</td></tr>';try{const r=await fetchWithTimeout('/scan',12000);const d=await r.json();rows.innerHTML=(Array.isArray(d.networks)?d.networks.map(wifiScanRow).join(''):'')||'<tr><td colspan=4>Keine Netze</td></tr>';}catch(e){rows.innerHTML='<tr><td colspan=4>Scan fehlgeschlagen</td></tr>';}}"
            "function renderBleScanRows(d){const rows=document.getElementById('bleScanRows');const typ=n=>[n.spartan?'Hub':'',n.nus?'NUS':''].filter(Boolean).join(',')||'–';const list=Array.isArray(d.devices)?d.devices:[];const hdr=list.length?`<tr><td colspan=5 class=sub>${list.length} Geraet(e) gefunden</td></tr>`:'';rows.innerHTML=hdr+list.map(n=>`<tr><td>${n.name||'---'}</td><td>${n.mac}</td><td>${n.rssi}</td><td>${typ(n)}</td><td><a href='/ble/select?mac=${encodeURIComponent(n.mac)}&mode=hub'><button>Hub</button></a> <a href='/ble/select?mac=${encodeURIComponent(n.mac)}&mode=direct'><button>123</button></a></td></tr>`).join('')||'<tr><td colspan=5>Keine Geraete</td></tr>';}"
            "async function scanBle(){const rows=document.getElementById('bleScanRows');rows.innerHTML='<tr><td colspan=5>Scan startet...</td></tr>';try{const r=await fetchWithTimeout('/ble/scan',15000,{method:'POST'});if(!r.ok){rows.innerHTML='<tr><td colspan=5>Scan fehlgeschlagen (HTTP '+r.status+')</td></tr>';return;}const d=await r.json();if(d.error==='ble_off'){rows.innerHTML='<tr><td colspan=5>BLE aus – Setup: BLE aktivieren</td></tr>';return;}if(!d.started){rows.innerHTML='<tr><td colspan=5>BLE-Scan konnte nicht gestartet werden</td></tr>';return;}rows.innerHTML='<tr><td colspan=5>Scan laeuft (3s)...</td></tr>';const deadline=Date.now()+15000;while(Date.now()<deadline){await new Promise(res=>setTimeout(res,400));const pr=await fetchWithTimeout('/ble/scan/result',12000);if(!pr.ok)continue;const pd=await pr.json();if(pd.error==='ble_off'){rows.innerHTML='<tr><td colspan=5>BLE aus – Setup: BLE aktivieren</td></tr>';return;}if(pd.scanning)continue;renderBleScanRows(pd);return;}rows.innerHTML='<tr><td colspan=5>Timeout – Scan nicht fertig</td></tr>';}catch(e){const msg=(e&&e.name==='AbortError')?'Timeout – ESP antwortet nicht':'Scan fehlgeschlagen';rows.innerHTML='<tr><td colspan=5>'+msg+'</td></tr>';}}"
            "function syncBleSetup(d){const m=document.getElementById('bleModeLabel');if(m)m.textContent=d.ble_mode_label||'?';const m2=document.getElementById('bleModeLabel2');if(m2)m2.textContent=d.ble_mode_label||'?';const mac=document.getElementById('bleMacLabel');if(mac)mac.textContent=d.ble_target_mac||'---';const hub=document.getElementById('bleHubMacLabel');if(hub)hub.textContent=d.ble_mac_hub||'---';const dir=document.getElementById('bleDirectMacLabel');if(dir)dir.textContent=d.ble_mac_123||'---';const tz=document.getElementById('tzLabel');if(tz&&d.tz_label)tz.textContent=d.tz_label;}"
            "async function live(){const box=document.getElementById('liveBox');try{const r=await fetchWithTimeout('/api/status',8000);if(!r.ok){if(box)box.textContent='HTTP '+r.status+' /api/status';return;}const d=await r.json();if(box)box.textContent=JSON.stringify(d,null,2);try{syncDashboard(d);}catch(e){}}catch(e){if(box)box.textContent='Live-Status nicht erreichbar';}}"
            "const otaForm=document.getElementById('otaForm');"
            "function otaShow(pct,msg){const bar=document.getElementById('otaBar');const st=document.getElementById('otaStatus');const box=document.getElementById('otaProgress');if(box)box.hidden=false;if(bar)bar.style.width=Math.max(0,Math.min(100,pct))+'%';if(st)st.textContent=msg||'Upload laeuft...';}"
            "if(otaForm){otaForm.addEventListener('submit',ev=>{ev.preventDefault();const fd=new FormData(otaForm);const file=fd.get('update');if(!file||!file.size)return;"
            "if(liveTimer)clearInterval(liveTimer);if(handTimer)clearInterval(handTimer);"
            "const btn=document.getElementById('otaBtn');if(btn)btn.disabled=true;otaShow(0,'Upload laeuft...');"
            "let poll=setInterval(async()=>{try{const r=await fetch('/api/ota/progress');const d=await r.json();"
            "if(d.active&&d.total>0){const n=Math.min(99,Math.round(100*(d.written||d.rx)/d.total));otaShow(n,'Flash '+n+'%');}}catch(e){}},500);"
            "const xhr=new XMLHttpRequest();xhr.open('POST','/update');"
            "xhr.upload.onprogress=e=>{if(e.lengthComputable){const n=Math.round(100*e.loaded/e.total);otaShow(Math.min(92,n),'Upload laeuft... '+n+'%');}else otaShow(0,'Upload laeuft...');};"
            "xhr.onload=()=>{clearInterval(poll);if(xhr.status===200&&xhr.responseText.trim()==='OK')otaShow(100,'Erfolg, Neustart...');"
            "else{otaShow(0,'Fehler ('+xhr.status+'): '+(xhr.responseText||'FAIL'));if(btn)btn.disabled=false;}};"
            "xhr.onerror=()=>{clearInterval(poll);otaShow(0,'Netzwerkfehler');if(btn)btn.disabled=false;};xhr.send(fd);});}"
            "live();liveTimer=setInterval(live,1000);handTimer=setInterval(updateHands,250);</script><p class='sub'>VW T2b Cockpit &middot; ESP32-S3 2.8C</p></main></body></html>");
  webServer.send(200, "text/html", html);
}

static String bleBuildScanJson() {
  uint8_t order[BLE_SCAN_MAX];
  const uint8_t n = g_bleScanCount;
  for (uint8_t i = 0; i < n; i++) order[i] = i;
  for (uint8_t i = 0; i + 1 < n; i++) {
    for (uint8_t j = i + 1; j < n; j++) {
      if (g_bleScanList[order[j]].rssi > g_bleScanList[order[i]].rssi) {
        const uint8_t t = order[i];
        order[i] = order[j];
        order[j] = t;
      }
    }
  }
  String json = F("{\"count\":");
  json += String(n);
  json += F(",\"devices\":[");
  for (uint8_t i = 0; i < n; i++) {
    if (i > 0) json += ',';
    const BleScanEntry& e = g_bleScanList[order[i]];
    json += F("{\"name\":\"");
    json += jsonEscape(String(e.name));
    json += F("\",\"mac\":\"");
    json += jsonEscape(String(e.mac));
    json += F("\",\"rssi\":");
    json += String(e.rssi);
    json += F(",\"spartan\":");
    json += e.spartan ? F("true") : F("false");
    json += F(",\"nus\":");
    json += e.nus ? F("true") : F("false");
    json += '}';
  }
  json += F("]}");
  return json;
}

static String bleBuildScanResultJson(bool scanning) {
  if (scanning) {
    return F("{\"scanning\":true,\"count\":0,\"devices\":[]}");
  }
  String json = F("{\"scanning\":false,");
  json += bleBuildScanJson().substring(1);
  return json;
}

static void handleWebBleScanStart() {
  if (webOtaRejectBusy()) return;
  if (!g_featureBle) {
    webServer.send(409, "application/json", F("{\"started\":false,\"error\":\"ble_off\"}"));
    return;
  }
  auto* scanDev = NimBLEDevice::getScan();
  if (g_bleDiscoveryScan || scanDev->isScanning()) {
    webServer.sendHeader("Connection", "close");
    webServer.send(200, "application/json", F("{\"started\":true,\"scanning\":true}"));
    return;
  }
  bleDoConnect = false;
  bleAbortDiscoveryScan();
  const bool started = bleStartDiscoveryScan();
  bleNextScanAt = millis() + BLE_DISCOVERY_SCAN_MS + 3000;
  if (!started) {
    webServer.send(503, "application/json", F("{\"started\":false,\"error\":\"scan_failed\"}"));
    return;
  }
  webServer.sendHeader("Connection", "close");
  webServer.send(200, "application/json", F("{\"started\":true,\"scanning\":true}"));
}

static void handleWebBleScanResult() {
  if (webOtaRejectBusy()) return;
  if (!g_featureBle) {
    webServer.send(409, "application/json", F("{\"scanning\":false,\"devices\":[],\"error\":\"ble_off\"}"));
    return;
  }
  auto* scanDev = NimBLEDevice::getScan();
  const bool scanning = g_bleDiscoveryScan || scanDev->isScanning();
  const String json = bleBuildScanResultJson(scanning);
  webServer.sendHeader("Connection", "close");
  webServer.send(200, "application/json", json);
}

static void handleWebBleSelect() {
  if (webOtaRejectBusy()) return;
  if (!webServer.hasArg("mac")) {
    webServer.send(400, "text/plain", "mac required");
    return;
  }
  String mac = webServer.arg("mac");
  mac.toLowerCase();
  BleConnMode mode = BLE_MODE_DIRECT_123;
  if (webServer.hasArg("mode")) {
    const String m = webServer.arg("mode");
    if (m == "hub" || m == "gateway") mode = BLE_MODE_SPARTAN_HUB;
  } else {
    for (uint8_t i = 0; i < g_bleScanCount; i++) {
      if (macEquals(mac, g_bleScanList[i].mac)) {
        mode = g_bleScanList[i].spartan ? BLE_MODE_SPARTAN_HUB : BLE_MODE_DIRECT_123;
        break;
      }
    }
  }
  if (mode == BLE_MODE_SPARTAN_HUB) saveBleMacHub(mac.c_str());
  else saveBleMac123(mac.c_str());
  saveBleConnMode(mode);
  disconnectBleForModeChange();
  bleNextScanAt = millis();
  DLOG("Web: BLE Ziel -> %s (%s)\n", bleTargetDisplay(), bleConnModeLabel());
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

static bool bleMacLooksValid(const String& mac) {
  if (mac.length() < 17) return false;
  for (size_t i = 0; i < mac.length(); i++) {
    const char c = mac[i];
    if (c == ':') continue;
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
  }
  return true;
}

static void handleWebBleMac() {
  if (webOtaRejectBusy()) return;
  bool changed = false;
  if (webServer.hasArg("hub_mac")) {
    String mac = webServer.arg("hub_mac");
    mac.trim();
    mac.toLowerCase();
    if (bleMacLooksValid(mac)) {
      saveBleMacHub(mac.c_str());
      changed = true;
    }
  }
  if (webServer.hasArg("direct_mac")) {
    String mac = webServer.arg("direct_mac");
    mac.trim();
    mac.toLowerCase();
    if (bleMacLooksValid(mac)) {
      saveBleMac123(mac.c_str());
      changed = true;
    }
  }
  if (changed) {
    disconnectBleForModeChange();
    bleNextScanAt = millis();
    DLOG("Web: BLE MAC hub=%s direct=%s\n", g_bleHubMac, g_bleTargetMac);
  }
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

static void handleWebBleMode() {
  if (webOtaRejectBusy()) return;
  BleConnMode next = BLE_MODE_SPARTAN_HUB;
  if (webServer.hasArg("m")) {
    const String m = webServer.arg("m");
    if (m == "direct" || m == "123") next = BLE_MODE_DIRECT_123;
  }
  if (next != g_bleConnMode) {
    saveBleConnMode(next);
    disconnectBleForModeChange();
    bleNextScanAt = millis();
    DLOG("Web: BLE Quelle -> %s\n", bleConnModeLabel());
  }
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

static void handleWebScan() {
  if (webOtaRejectBusy()) return;
  int count = WiFi.scanNetworks();
  String json = F("{\"networks\":[");
  const String connectedSsid = WiFi.SSID();
  for (int i = 0; i < count; i++) {
    if (i > 0) json += ',';
    const String ssid = WiFi.SSID(i);
    json += F("{\"ssid\":\"");
    json += jsonEscape(ssid);
    json += F("\",\"rssi\":");
    json += String(WiFi.RSSI(i));
    json += F(",\"connected\":");
    json += (WiFi.status() == WL_CONNECTED && ssid == connectedSsid) ? F("true") : F("false");
    json += '}';
  }
  json += F("]}");
  WiFi.scanDelete();
  webServer.send(200, "application/json", json);
}

static void handleWebStatus() {
  if (webOtaRejectBusy()) return;
  webServer.client().setNoDelay(true);
  struct tm now = {};
  readClockTime(&now);
  String json;
  json.reserve(768);
  json += F("{\"time\":");
  json += String((unsigned long)time(nullptr));
  json += F(",\"hour\":");
  json += String(now.tm_hour);
  json += F(",\"min\":");
  json += String(now.tm_min);
  json += F(",\"sec\":");
  json += String(now.tm_sec);
  json += F(",\"time_source\":\"");
  json += clockSourceLabel();
  json += F("\",\"tz_idx\":");
  json += String(g_timezoneIdx);
  json += F(",\"tz_label\":\"");
  json += jsonEscape(String(timezoneLabel(g_timezoneIdx)));
  json += F("\",\"ip\":\"");
  json += jsonEscape(String(g_ipStr));
  json += F("\",\"rpm\":");
  json += String((int)g_rpm);
  json += F(",\"adv\":");
  jsonAppendFloat(json, g_adv, 1);
  json += F(",\"map\":");
  json += String((int)g_map);
  json += F(",\"lambda\":");
  jsonAppendFloat(json, g_lambda, 3);
  json += F(",\"lambda_valid\":");
  json += g_lambdaValid ? F("true") : F("false");
  json += F(",\"volt\":");
  jsonAppendFloat(json, g_battVolt, 2);
  json += F(",\"volt_valid\":");
  json += g_battValid ? F("true") : F("false");
  json += F(",\"aux_volt\":");
  jsonAppendFloat(json, g_auxBattVolt, 2);
  json += F(",\"aux_volt_valid\":");
  json += g_auxBattValid ? F("true") : F("false");
  json += F(",\"ble_enabled\":");
  json += g_featureBle ? F("true") : F("false");
  json += F(",\"data_path\":\"");
  json += g_dataPath == DATA_PATH_WIFI_HUB ? F("wifi") : F("ble");
  json += F("\",\"data_path_label\":\"");
  json += jsonEscape(String(dataPathLabel()));
  json += F("\",\"source_mode\":\"");
  if (g_dataPath == DATA_PATH_WIFI_HUB) json += F("hub_wifi");
  else if (g_bleConnMode == BLE_MODE_SPARTAN_HUB) json += F("hub_ble");
  else json += F("direct_123");
  json += F("\",\"source_mode_label\":\"");
  json += jsonEscape(String(sourceModeLabel()));
  json += F("\",\"wifi_network\":\"");
  if (g_wifiApOnly) json += F("ap");
  else if (g_featureWifi) json += F("home");
  else json += F("off");
  json += F("\",\"wifi_ap_on\":");
  json += g_apOn ? F("true") : F("false");
  json += F(",\"hub_host\":\"");
  json += jsonEscape(String(g_hubHost));
  json += F("\",\"hub_wifi_ok\":");
  json += hubLinkOk() ? F("true") : F("false");
  json += F(",\"hub_state\":\"");
  json += jsonEscape(String(g_hubState));
  json += F("\",\"hub_source\":\"");
  json += jsonEscape(String(g_hubSourceTag));
  json += F("\",\"data_fresh\":");
  json += dataFresh() ? F("true") : F("false");
  json += F(",\"live_rx\":");
  json += String((unsigned long)g_liveRxCnt);
#if ENABLE_ESP_NOW_CLIENT
  json += F(",\"esp_now_enabled\":");
  json += g_featureEspNow ? F("true") : F("false");
  json += F(",\"esp_now_ready\":");
  json += g_espNowReady ? F("true") : F("false");
  json += F(",\"esp_now_channel\":");
  json += String((unsigned)(g_espNowReady ? g_espNowActiveChannel : espNowEffectiveChannel()));
  json += F(",\"esp_now_channel_label\":\"");
  json += jsonEscape(String(espNowChannelLabel()));
  json += F("\",\"esp_now_channel_pref\":");
  json += String((unsigned)g_espNowChannelPref);
  json += F(",\"esp_now_rx\":");
  json += String((unsigned long)g_espNowRx);
  json += F(",\"esp_now_seq\":");
  json += String((unsigned)g_espNowSeq);
  json += F(",\"esp_now_fresh\":");
  json += espNowDataFresh() ? F("true") : F("false");
#endif
  json += F(",\"ble_connected\":");
  json += g_bleConn ? F("true") : F("false");
  json += F(",\"ble_rx\":");
  json += String((unsigned long)g_bleRxCnt);
  json += F(",\"ble_mode\":\"");
  json += g_bleConnMode == BLE_MODE_SPARTAN_HUB ? F("hub") : F("direct");
  json += F("\",\"ble_mode_label\":\"");
  json += jsonEscape(String(bleConnModeLabel()));
  json += F("\",\"ble_target_mac\":\"");
  json += jsonEscape(String(bleTargetDisplay()));
  json += F("\",\"ble_mac_123\":\"");
  json += jsonEscape(String(g_bleTargetMac));
  json += F("\",\"ble_mac_hub\":\"");
  json += jsonEscape(String(g_bleHubMac));
  json += F("\",\"ble_target_name\":\"");
  json += jsonEscape(g_bleHubName);
  json += F("\",\"ble_state\":\"");
  json += jsonEscape(String(g_bleState));
  json += F("\",\"ble_log\":\"");
  json += jsonEscape(String(g_bleLogLine));
  json += F("\",\"page\":");
  json += String(currentPage);
  json += F(",\"page_label\":\"");
  json += jsonEscape(String(webPageLabel(currentPage)));
  json += F("\",\"scale\":");
  json += String(g_dialScalePct);
  json += F(",\"rotation\":");
  json += String(g_rotationDeg);
  json += F(",\"dial_off_x\":");
  json += String(g_dialCenterOffsetX);
  json += F(",\"dial_off_y\":");
  json += String(g_dialCenterOffsetY);
  json += F(",\"rpm_redline\":");
  json += String(g_rpmRedline);
  json += F(",\"rpm_scale_max\":");
  json += String(g_rpmScaleMax);
  json += F(",\"motor_style\":\"");
  json += motorStyleLabel();
  json += F("\",\"motor_style_idx\":");
  json += String((unsigned)g_motorStyle);
  json += F(",\"imu_present\":");
  json += g_imuPresent ? F("true") : F("false");
  json += F(",\"imu_pitch\":");
  jsonAppendFloat(json, g_imuPitch, 1);
  json += F(",\"imu_roll\":");
  jsonAppendFloat(json, g_imuRoll, 1);
  json += F(",\"imu_trimmed\":");
  json += g_imuTrimmed ? F("true") : F("false");
  json += F(",\"ota_boot\":\"");
  json += g_otaBootLabel;
  json += F("\"}");
  webServer.sendHeader("Connection", "close");
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(200, "application/json", json);
}

static void handleWebOtaProgress() {
  String json;
  json.reserve(96);
  json += F("{\"active\":");
  json += g_otaBusy ? F("true") : F("false");
  json += F(",\"rx\":");
  json += String((unsigned long)g_otaRxBytes);
  json += F(",\"written\":");
  json += String((unsigned)Update.progress());
  json += F(",\"total\":");
  json += String((unsigned)Update.size());
  json += F(",\"boot\":\"");
  json += g_otaBootLabel;
  json += F("\"}");
  webServer.send(200, "application/json", json);
}

static void handleWebRestart() {
  webServer.send(200, "text/html", F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>Restart</title></head><body><h1>ESP32 startet neu...</h1></body></html>"));
  delay(500);
  ESP.restart();
}

static void handleWebImuZero() {
  if (calibrateImuZero()) g_redrawPage = true;
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

static void handleWebSet() {
  if (webServer.hasArg("scale")) {
    saveDialScale(webServer.arg("scale").toInt());
    g_redrawPage = true;
    DLOG("Web: Zifferblatt-Groesse = %d%%\n", g_dialScalePct);
  }
  if (webServer.hasArg("scale_delta")) {
    saveDialScale(g_dialScalePct + webServer.arg("scale_delta").toInt());
    g_redrawPage = true;
    DLOG("Web: Zifferblatt-Groesse = %d%%\n", g_dialScalePct);
  }
  if (webServer.hasArg("rot_delta")) {
    saveRotation(g_rotationDeg + webServer.arg("rot_delta").toInt());
    g_redrawPage = true;
    DLOG("Web: Rotation = %d deg\n", g_rotationDeg);
  }
  if (webServer.hasArg("rot")) {
    saveRotation(webServer.arg("rot").toInt());
    g_redrawPage = true;
    DLOG("Web: Rotation = %d deg\n", g_rotationDeg);
  }
  if (webServer.hasArg("dial_x_delta")) {
    saveDialOffset(g_dialCenterOffsetX + webServer.arg("dial_x_delta").toInt(), g_dialCenterOffsetY);
    g_redrawPage = true;
    DLOG("Web: Zifferblatt-Offset X=%d Y=%d\n", g_dialCenterOffsetX, g_dialCenterOffsetY);
  }
  if (webServer.hasArg("dial_y_delta")) {
    saveDialOffset(g_dialCenterOffsetX, g_dialCenterOffsetY + webServer.arg("dial_y_delta").toInt());
    g_redrawPage = true;
    DLOG("Web: Zifferblatt-Offset X=%d Y=%d\n", g_dialCenterOffsetX, g_dialCenterOffsetY);
  }
  if (webServer.hasArg("dial_reset")) {
    saveDialOffset(DIAL_CENTER_OFF_X_DEFAULT, DIAL_CENTER_OFF_Y_DEFAULT);
    g_redrawPage = true;
    DLOG("Web: Zifferblatt-Offset reset X=%d Y=%d\n", g_dialCenterOffsetX, g_dialCenterOffsetY);
  }
  if (webServer.hasArg("rpm_max")) {
    saveRpmScaleMax(webServer.arg("rpm_max").toInt());
    g_redrawPage = true;
    DLOG("Web: RPM Skala max = %d\n", g_rpmScaleMax);
  }
  if (webServer.hasArg("rpm_redline")) {
    saveRpmRedline(webServer.arg("rpm_redline").toInt());
    g_redrawPage = true;
    DLOG("Web: RPM Redline = %d\n", g_rpmRedline);
  }
  if (webServer.hasArg("night")) {
    saveNightMode(webServer.arg("night").toInt() != 0);
    g_redrawPage = true;
    DLOG("Web: Nachtmodus = %s\n", g_nightMode ? "on" : "off");
  }
  if (webServer.hasArg("motor_style")) {
    saveMotorStyle((uint8_t)webServer.arg("motor_style").toInt());
    g_redrawPage = true;
    DLOG("Web: Motor style = %s\n", motorStyleLabel());
  }
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

static void handleWebFeatures() {
  const bool wifi   = webServer.hasArg("wifi");
  const bool ble    = webServer.hasArg("ble");
  const bool buzzer = webServer.hasArg("buzzer");
  saveFeatures(wifi, ble, buzzer);
#if ENABLE_ESP_NOW_CLIENT
  saveEspNowFeature(webServer.hasArg("espnow"));
  if (webServer.hasArg("espnow_ch")) {
    int ch = webServer.arg("espnow_ch").toInt();
    if (ch < 0) ch = 0;
    if (ch > 14) ch = 14;
    g_espNowChannelPref = (uint8_t)ch;
    Preferences p;
    p.begin("clock", false);
    p.putUChar("espnow_ch", g_espNowChannelPref);
    p.end();
    teardownEspNowClient();
  }
#endif
  if (webServer.hasArg("hub_host")) saveHubHost(webServer.arg("hub_host").c_str());
  if (webServer.hasArg("data_path")) {
    const String m = webServer.arg("data_path");
    saveDataPath(m == "ble" ? DATA_PATH_BLE : DATA_PATH_WIFI_HUB);
  }
  DLOG("Web: Funktionen wifi=%s ble=%s buzzer=%s hub=%s path=%s\n",
                g_featureWifi ? "on" : "off", g_featureBle ? "on" : "off",
                g_featureBuzzer ? "on" : "off", g_hubHost, dataPathLabel());
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

static void handleWebTimezone() {
  if (webOtaRejectBusy()) return;
  if (webServer.hasArg("idx")) {
    int idx = webServer.arg("idx").toInt();
    if (idx < 0) idx = 0;
    if (idx >= (int)TIMEZONE_COUNT) idx = TIMEZONE_COUNT - 1;
    if ((uint8_t)idx != g_timezoneIdx) {
      saveTimezone((uint8_t)idx);
    } else {
      applyTimezone();
      requestNtpResync();
      DLOG("Web: TZ resync %s\n", timezoneLabel(g_timezoneIdx));
    }
    g_redrawPage = true;
  }
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

static void handleWebPage() {
  if (webServer.hasArg("p")) {
    int page = webServer.arg("p").toInt();
    if (page < 0) page = 0;
    if (page > PAGE_MAX) page = PAGE_MAX;
    currentPage  = static_cast<uint8_t>(page);
    g_redrawPage = true;
    DLOG("Web: page=%u\n", currentPage);
  }
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

static void reconnectWifiProfile();    // fwd
static void saveWifiProfile(uint8_t);  // fwd

// WLAN-Profil per Web umschalten: /wifi?prof=N  oder Modus /wifi?mode=home|ap|off
static void handleWebWifi() {
  if (webServer.hasArg("mode")) {
    const String m = webServer.arg("mode");
    if (m == "bus") {
      applyBusProfile(true);
      DLOG("Web: WiFi -> BUS (Spartan3-Setup)\n");
    } else if (m == "ap") {
      enterApOnlyMode();
      DLOG("Web: WiFi -> AP Setup\n");
    } else if (m == "off") {
      stopApOnlyMode();
      saveFeatures(false, g_featureBle, g_featureBuzzer);
      DLOG("Web: WiFi -> AUS\n");
    } else {
      stopApOnlyMode();
      saveFeatures(true, g_featureBle, g_featureBuzzer);
      reconnectWifiProfile();
      DLOG("Web: WiFi -> Home (%s)\n", currentWifiSsid());
    }
  }
  if (webServer.hasArg("prof")) {
    int idx = webServer.arg("prof").toInt();
    if (idx < 0) idx = 0;
    saveWifiProfile((uint8_t)idx);
    stopApOnlyMode();
    saveFeatures(true, g_featureBle, g_featureBuzzer);
    reconnectWifiProfile();
    DLOG("Web: WLAN-Profil -> %d (%s)\n", idx, currentWifiSsid());
  }
  if (webServer.hasArg("ssid")) {
    const String ssid = webServer.arg("ssid");
    const uint8_t count = wifiProfileCount();
    for (uint8_t i = 0; i < count; i++) {
      if (ssid == WIFI_PROFILES[i].ssid) {
        saveWifiProfile(i);
        stopApOnlyMode();
        saveFeatures(true, g_featureBle, g_featureBuzzer);
        reconnectWifiProfile();
        DLOG("Web: WLAN via Scan -> %u (%s)\n", i, currentWifiSsid());
        break;
      }
    }
  }
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

static void handleWebSource() {
  if (webServer.hasArg("m")) {
    applySourceMode(webServer.arg("m").c_str());
    DLOG("Web: Source -> %s\n", sourceModeLabel());
  }
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

static void startWebServer() {
  webServer.on("/",        handleWebRoot);
  webServer.on("/set",     handleWebSet);
  webServer.on("/features",handleWebFeatures);
  webServer.on("/tz",      handleWebTimezone);
  webServer.on("/page",    handleWebPage);
  webServer.on("/wifi",    handleWebWifi);
  webServer.on("/source",  handleWebSource);
  webServer.on("/scan",    HTTP_GET, handleWebScan);
  webServer.on("/ble/scan", HTTP_POST, handleWebBleScanStart);
  webServer.on("/ble/scan", HTTP_GET, handleWebBleScanStart);
  webServer.on("/ble/scan/result", HTTP_GET, handleWebBleScanResult);
  webServer.on("/ble/select", HTTP_GET, handleWebBleSelect);
  webServer.on("/ble/mac", HTTP_GET, handleWebBleMac);
  webServer.on("/ble/mode", HTTP_GET, handleWebBleMode);
  webServer.on("/api/status", HTTP_GET, handleWebStatus);
  webServer.on("/api/ota/progress", HTTP_GET, handleWebOtaProgress);
  webServer.on("/api/preview", HTTP_GET, handleWebPreview);
  webServer.on("/restart", HTTP_GET, handleWebRestart);
  webServer.on("/imu/zero", HTTP_GET, handleWebImuZero);
  webServer.on("/imu_zero", HTTP_GET, handleWebImuZero);
  webServer.on("/update", HTTP_POST, []() {
    const bool ok = !Update.hasError() && Update.remaining() == 0;
    webServer.sendHeader("Connection", "close");
    webServer.send(ok ? 200 : 500, "text/plain", ok ? "OK" : "FAIL");
    if (ok) {
      delay(500);
      ESP.restart();
    } else {
      g_otaBusy = false;
      hal_pause_for_ota(false);
    }
  }, []() {
    HTTPUpload& upload = webServer.upload();
    if (upload.status == UPLOAD_FILE_START) {
      webOtaBeginUpload(upload.filename.c_str());
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      webOtaWriteChunk(upload.buf, upload.currentSize);
    } else if (upload.status == UPLOAD_FILE_END) {
      webOtaFinishUpload(upload.totalSize);
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
      webOtaAbortUpload();
    }
  });
  webServer.begin();
  Serial.println("WebGUI: gestartet auf Port 80");
}

// Fallback-Setup-AP: nur AN wenn keine STA-Verbindung besteht, damit das
// Display im verbundenen Normalbetrieb stabil bleibt (kein AP-Dauerbeacon).
static void saveWifiApOnly(bool on) {
  if (on == g_wifiApOnly) return;
  g_wifiApOnly = on;
  Preferences p;
  p.begin("clock", false);
  p.putBool("wifi_ap_only", g_wifiApOnly);
  p.end();
}

static void enterApOnlyMode() {
  g_featureWifi = true;
  saveWifiApOnly(true);
  WiFi.disconnect(true);
  delay(50);
  WiFi.mode(WIFI_AP);
  WiFi.softAP("VDO-Clock-Setup", "vdoclock");
  g_apOn = true;
  snprintf(g_ipStr, sizeof(g_ipStr), "%s", WiFi.softAPIP().toString().c_str());
  if (!g_webStarted) { startWebServer(); g_webStarted = true; }
  Serial.printf("WiFi: AP-only -> http://%s (VDO-Clock-Setup / vdoclock)\n", g_ipStr);
}

static void stopApOnlyMode() {
  if (!g_wifiApOnly && !g_apOn) return;
  saveWifiApOnly(false);
  if (g_apOn) {
    WiFi.softAPdisconnect(true);
    g_apOn = false;
  }
}

static void cycleWifiNetworkMode() {
  if (!g_featureWifi && !g_wifiApOnly) {
    saveWifiApOnly(false);
    saveFeatures(true, g_featureBle, g_featureBuzzer);
    reconnectWifiProfile();
    return;
  }
  if (g_wifiApOnly) {
    stopApOnlyMode();
    saveFeatures(false, g_featureBle, g_featureBuzzer);
    g_redrawPage = true;
    return;
  }
  if (isOnBusWifi()) {
    saveWifiProfile(0);
    saveDataPath(DATA_PATH_WIFI_HUB);
    saveHubHost(HUB_WIFI_DEFAULT_HOST);
    saveFeatures(true, g_featureBle, g_featureBuzzer);
    reconnectWifiProfile();
    g_redrawPage = true;
    return;
  }
  const uint8_t count = wifiProfileCount();
  if (count > 1 && g_wifiProfile + 1 < count) {
    saveWifiProfile((uint8_t)(g_wifiProfile + 1));
    reconnectWifiProfile();
  } else {
    enterApOnlyMode();
  }
  g_redrawPage = true;
}

static void applySourceMode(const char* mode) {
  if (!mode) return;
  const String m = String(mode);
  if (m == "hub_wifi" || m == "wifi") {
    saveDataPath(DATA_PATH_WIFI_HUB);
  } else if (m == "hub_ble" || m == "hub") {
    saveDataPath(DATA_PATH_BLE);
    if (!g_featureBle) saveFeatures(g_featureWifi, true, g_featureBuzzer);
    if (g_bleConnMode != BLE_MODE_SPARTAN_HUB) {
      saveBleConnMode(BLE_MODE_SPARTAN_HUB);
      disconnectBleForModeChange();
    }
  } else if (m == "123" || m == "direct") {
    saveDataPath(DATA_PATH_BLE);
    if (!g_featureBle) saveFeatures(g_featureWifi, true, g_featureBuzzer);
    if (g_bleConnMode != BLE_MODE_DIRECT_123) {
      saveBleConnMode(BLE_MODE_DIRECT_123);
      disconnectBleForModeChange();
    }
  }
  g_redrawPage = true;
}

static void manageWifiAp() {
  if (g_wifiApOnly) {
    if (!g_apOn) enterApOnlyMode();
    return;
  }
  if (!g_featureWifi) {
    if (g_apOn) {
      WiFi.softAPdisconnect(true);
      g_apOn = false;
    }
    return;
  }
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
#if FEATURE_VERBOSE_SERIAL
  Serial.printf("[TOUCH] setup y=%u dur=%lums rot=%d\n", y, (unsigned long)durMs, g_rotationDeg);
#endif

  if (currentPage == 9) {
    if (y >= 420) {
      requestPage(1);
      DLOGN("setup2 tap: menu");
    } else if (y >= 96 && y < 133) {
      saveFeatures(g_featureWifi, !g_featureBle, g_featureBuzzer);
      g_redrawPage = true;
      DLOG("setup2 tap: ble=%s\n", g_featureBle ? "on" : "off");
    } else if (y >= 133 && y < 175) {
      applySourceMode(g_bleConnMode == BLE_MODE_DIRECT_123 ? "hub_ble" : "123");
      DLOG("setup2 tap: ble source=%s\n", bleConnModeLabel());
    } else if (y >= 175 && y < 217) {
      saveEspNowFeature(!g_featureEspNow);
      g_redrawPage = true;
      DLOG("setup2 tap: espnow=%s\n", g_featureEspNow ? "on" : "off");
    } else if (y >= 217 && y < 259) {
      cycleEspNowChannelPref();
      g_redrawPage = true;
      DLOG("setup2 tap: espnow_ch=%s\n", espNowChannelLabel());
    } else if (y >= 259 && y < 301) {
      saveFeatures(g_featureWifi, g_featureBle, !g_featureBuzzer);
      g_redrawPage = true;
      DLOG("setup2 tap: buzzer=%s\n", g_featureBuzzer ? "on" : "off");
    } else if (y >= 301 && y < 343) {
      cycleSourceMode();
      DLOG("setup2 tap: source=%s\n", sourceModeLabel());
    } else if (y >= 343 && y < 397) {
      if (calibrateImuZero()) g_redrawPage = true;
      DLOGN("setup2 tap: imu_null");
    } else {
      g_redrawPage = true;
      DLOGN("setup2 tap: no action");
    }
    return;
  }

  // Zonen passend zu drawSetupPage.
  if (y >= 452) {
    requestPage(1);
    DLOGN("setup tap: menu");
    return;
  }

  if (y >= 100 && y < 148) {
    int next = (g_dialScalePct < 115) ? 115 : (g_dialScalePct < 120 ? 120 : 115);
    saveDialScale(next);
    g_redrawPage = true;
    DLOG("setup tap: dial=%d%%\n", g_dialScalePct);
  } else if (y >= 148 && y < 196) {
    int next = (g_brightnessPct < 63) ? 75 : (g_brightnessPct < 88 ? 100 : 50);
    saveBrightness(next);
    g_redrawPage = true;
    DLOG("setup tap: brightness=%d%%\n", g_brightnessPct);
  } else if (y >= 196 && y < 244) {
    int next = (g_rotationDeg < 90) ? 90 : (g_rotationDeg < 180 ? 180 : (g_rotationDeg < 270 ? 270 : 0));
    saveRotation(next);
    g_redrawPage = true;
    DLOG("[ROT] setup tap -> %d deg\n", g_rotationDeg);
  } else if (y >= 244 && y < 292) {
    cycleWifiNetworkMode();
    DLOG("setup tap: wifi network\n");
  } else if (y >= 292 && y < 340) {
    saveNightMode(!g_nightMode);
    g_redrawPage = true;
    DLOG("setup tap: night=%s\n", g_nightMode ? "on" : "off");
  } else if (y >= 340 && y < 404) {
    requestPage(9);
    DLOGN("setup tap: setup2");
  } else {
    g_redrawPage = true;
    DLOGN("setup tap: no action");
  }
}

#if FEATURE_TOUCH
static void pageNext() {
  if      (currentPage == 4) requestPage(6);
  else if (currentPage == 6) requestPage(7);
  else if (currentPage == 7) requestPage(8);
  else if (currentPage == 8) requestPage(0);
  else if (currentPage == 5) requestPage(9);
  else if (currentPage == 9) requestPage(1);
  else if (currentPage == 0) requestPage(1);
  else requestPage(currentPage + 1);
}

static void pagePrev() {
  if      (currentPage == 0) requestPage(8);
  else if (currentPage == 9) requestPage(5);
  else if (currentPage == 8) requestPage(7);
  else if (currentPage == 7) requestPage(6);
  else if (currentPage == 6) requestPage(4);
  else if (currentPage == 1) requestPage(0);
  else requestPage(currentPage - 1);
}

static void handleTouchTap(uint16_t tapX, uint16_t tapY) {
  if (tapX < EDGE_TAP_W) {
    pagePrev();
    return;
  }
  if (tapX >= 480 - EDGE_TAP_W) {
    pageNext();
    return;
  }
  if (currentPage == 0) {
    requestPage(1);
  } else if (currentPage == 1) {
    const int idx = menuIndexFromTap(tapX, tapY);
    if (idx >= 0) requestPage(MENU_ENTRIES[idx].page);
    else requestPage(0);
  } else if (currentPage == 5 || currentPage == 9) {
    handleSetupLongPress(tapY, 0);
  } else {
    pageNext();
  }
}

static void processTouchInput(bool *touchBusyOut) {
  static uint32_t lastTouch = 0;
  static bool     touchActive = false;
  static bool     touchLongHandled = false;
  static uint32_t touchStartMs = 0;
  static uint32_t touchLastSeenMs = 0;
  static uint16_t touchStartX = 0, touchStartY = 0;
  static uint16_t touchLastX = 0, touchLastY = 0;
  static bool     touchHadPos = false;

  uint16_t x = 0, y = 0;
  const uint32_t nowMs = millis();
  const bool touchFrame = readTouch(&x, &y);
  const bool touchHeld = touchActive && (nowMs - touchLastSeenMs < 200);
  const bool touchNow = touchFrame || touchHeld;
  if (touchBusyOut) *touchBusyOut = touchNow || touchActive;

  if (touchNow) {
    touchSeen = true;
    if (touchFrame) {
      touchLastSeenMs = nowMs;
      touchLastX = x;
      touchLastY = y;
      touchHadPos = true;
    }
    if (!touchActive) {
      touchActive = true;
      touchLongHandled = false;
      touchHadPos = touchFrame;
      touchStartMs = nowMs;
      touchStartX = x;
      touchStartY = y;
      touchLastX = x;
      touchLastY = y;
      blePauseForTouch();
    }
    if ((currentPage == 5 || currentPage == 9) && !touchLongHandled && nowMs - touchStartMs >= 600) {
      touchLongHandled = true;
      lastTouch = nowMs;
      handleSetupLongPress(touchLastY, nowMs - touchStartMs);
    }
  } else if (touchActive && nowMs - touchLastSeenMs > ((currentPage == 5 || currentPage == 9) ? 700UL : TOUCH_RELEASE_MS)) {
    const uint32_t durMs = touchLastSeenMs - touchStartMs;
    const uint16_t tapX = touchHadPos ? touchLastX : touchStartX;
    const uint16_t tapY = touchHadPos ? touchLastY : touchStartY;
    touchActive = false;
    touchHadPos = false;
    if (!touchLongHandled && durMs < 600 && nowMs - lastTouch > TOUCH_COOLDOWN_MS) {
      lastTouch = nowMs;
      handleTouchTap(tapX, tapY);
    }
  }
}
#endif

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
  stopApOnlyMode();
  strcpy(g_ipStr, "...");
  WiFi.setHostname(DEVICE_HOSTNAME);
  WiFi.mode(WIFI_STA);
  applyWifiIpConfig();
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
  Serial.printf("Hostname: %s\n", DEVICE_HOSTNAME);
  otaNoteBootState();
  Serial.printf("PSRAM found: %s, size: %u bytes\n", psramFound() ? "yes" : "no", ESP.getPsramSize());

  // Backlight diagnostic blink (2x 50ms) before panel init.
  // Später: ADC-Dimming vom Original-Tacho-Drehregler — siehe FUTURE.md
  hal_backlight(true);  delay(50);
  hal_backlight(false); delay(50);
  hal_backlight(true);  delay(50);
  hal_backlight(false);

  loadSettings();

  if (g_dataPath == DATA_PATH_BLE && g_bleConnMode == BLE_MODE_SPARTAN_HUB) {
    g_dataPath = DATA_PATH_WIFI_HUB;
    g_featureBle = false;
    Preferences pBoot;
    pBoot.begin("clock", false);
    pBoot.putUChar("data_path", g_dataPath);
    pBoot.putBool("feat_ble", false);
    pBoot.end();
    Serial.println("Boot: WiFi hub (was BLE hub)");
  }

  Preferences pRecover;
  pRecover.begin("clock", false);
  const bool homeRecovered = pRecover.getBool("home_rec", false);
  pRecover.end();
  const uint8_t preferredHomeProfile = preferredHomeWifiProfileIndex();
  if (!homeRecovered && wifiProfileCount() > 1 && g_wifiProfile != preferredHomeProfile) {
    saveWifiProfile(preferredHomeProfile);
    saveDataPath(DATA_PATH_WIFI_HUB);
    saveHubHost(HUB_WIFI_DEFAULT_HOST);
    g_featureBle = false;
    Preferences pBoot;
    pBoot.begin("clock", false);
    pBoot.putBool("feat_ble", false);
    pBoot.putBool("home_rec", true);
    pBoot.end();
    Serial.printf("Boot: Home-WLAN bevorzugt '%s'\n", currentWifiSsid());
  }

  // WiFi and BLE MUST init before the RGB panel. WiFi/BLE PHY init temporarily
  // disables the flash cache; if the RGB VSYNC ISR (in flash) fires during that
  // window the CPU faults. With no panel running yet there is no VSYNC ISR.
  if (g_wifiApOnly) {
    WiFi.persistent(false);
    enterApOnlyMode();
  } else if (g_featureWifi && strlen(currentWifiSsid()) > 0) {
    WiFi.persistent(false);  // keine WiFi-Flash-Schreibzugriffe -> kein Cache-Disable
    WiFi.setSleep(true);     // Modem-Sleep: weniger PSRAM-Bus-Konkurrenz
    WiFi.mode(WIFI_STA);
    applyWifiIpConfig();
    WiFi.begin(currentWifiSsid(), currentWifiPassword());
    Serial.printf("WiFi: Verbindung zu '%s' im Hintergrund gestartet\n", currentWifiSsid());
  }

  if (g_featureBle && g_dataPath == DATA_PATH_BLE) {
    NimBLEDevice::init("VDO-Clock");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    g_bleStackReady = true;
    const uint32_t bootDelay = (g_bleConnMode == BLE_MODE_DIRECT_123) ? 1500u : 3000u;
    bleNextScanAt = millis() + bootDelay;
    bleLogSetState("boot_wait");
    BLELOG("BLE: init mode=%s mac=%s start in %lums",
           bleConnModeLabel(), bleSavedMacForMode(), (unsigned long)bootDelay);
    Serial.println("BLE: Client initialisiert");
  } else if (g_dataPath == DATA_PATH_WIFI_HUB) {
    Serial.printf("Hub: WiFi poll -> http://%s/api/status every %ums\n",
                  g_hubHost, (unsigned)HUB_WIFI_POLL_MS);
#if ENABLE_ESP_NOW_CLIENT
    Serial.printf("Hub: ESP-NOW preferred on Bus (ch %d), HTTP fallback\n", ESP_NOW_WIFI_CHANNEL);
#endif
  }

  Serial.println("Display: HAL init...");
  hal_init();
#if FEATURE_TACHO_DIMMER
  hal_tacho_dimmer_init();
#endif
  otaValidatePendingBoot();
  gt911Init();

  // QMI8658 IMU erkennen + initialisieren (I2C laeuft via hal_init/Wire)
  g_imuPresent = qmi8658Detect();
  if (g_imuPresent) {
    qmi8658Init();
    Serial.println("IMU: QMI8658 found at 0x6B, initialized");
  } else {
    Serial.println("IMU: QMI8658 NOT found");
  }

  initTimeSource();
  hal_backlight(true);
  finishDialCacheRebuildBlocking();
  Serial.printf("Dial cache: %s (%d%%, rot=%d)\n",
                g_dialCache ? "ready" : "fallback", g_dialScalePct, g_rotationDeg);
  drawVdoClock();
  otaConfirmBootIfPending();
  Serial.println("VDO clock drawn.");
}

void loop() {
  static uint32_t lastDraw  = 0;
  static String   serialLine;
  bool touchBusy = false;

  if (g_otaBusy) {
    if (g_webStarted) {
      for (uint8_t n = 0; n < 32; n++) webServer.handleClient();
    }
    yield();
    delay(2);
    return;
  }

#if FEATURE_TOUCH
  processTouchInput(&touchBusy);
#else
  (void)touchBusy;
#endif

#if ENABLE_ESP_NOW_CLIENT
  espNowClientTick();
#endif

  // Page switch from touch/web/serial: redraw before BLE/NTP/cache so touch feels snappy
  if (g_redrawPage) {
    g_redrawPage = false;
    drawCurrentPage();
    webServerPoll(4);
  }

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
#if ENABLE_ESP_NOW_CLIENT
        else if (cmd == "espnow:on")  { saveEspNowFeature(true); g_redrawPage = true; }
        else if (cmd == "espnow:off") { saveEspNowFeature(false); g_redrawPage = true; }
        else if (cmd == "espnow:ch")  { cycleEspNowChannelPref(); g_redrawPage = true; }
#endif
        else if (cmd == "wifi:next") { cycleWifiProfile(); g_redrawPage = true; }
        else if (cmd == "bus" || cmd == "bus:on") { applyBusProfile(true); g_redrawPage = true; }
        else if (cmd == "wifi:off")  { saveFeatures(false, g_featureBle, g_featureBuzzer); g_redrawPage = true; }
        else if (cmd == "rot:+") { saveRotation(g_rotationDeg + 1); g_redrawPage = true; }
        else if (cmd == "rot:-") { saveRotation(g_rotationDeg - 1); g_redrawPage = true; }
        else if (cmd.startsWith("rot:")) { saveRotation(cmd.substring(4).toInt()); g_redrawPage = true; }
        else if (cmd == "clock")   { currentPage = 0; drawVdoClock(); }
        else if (cmd == "ble:hub") { saveBleConnMode(BLE_MODE_SPARTAN_HUB); disconnectBleForModeChange(); g_redrawPage = true; }
        else if (cmd == "ble:123" || cmd == "ble:direct") {
          saveBleConnMode(BLE_MODE_DIRECT_123); disconnectBleForModeChange(); g_redrawPage = true;
        }
        else if (cmd == "ble:scan") { bleStartDiscoveryScan(); }
        else if (cmd == "imu:zero") { if (calibrateImuZero()) g_redrawPage = true; }
        else if (cmd == "status") {
          Serial.printf("STATUS ip=%s ssid=%s path=%s source=%s fresh=%d rx=%lu rpm=%.0f adv=%.1f map=%.0f lambda=%.3f lambda_valid=%d\n",
                        g_ipStr,
                        WiFi.SSID().c_str(),
                        dataPathLabel(),
                        g_hubSourceTag,
                        dataFresh() ? 1 : 0,
                        static_cast<unsigned long>(g_liveRxCnt),
                        g_rpm,
                        g_adv,
                        g_map,
                        g_lambda,
                        g_lambdaValid ? 1 : 0);
#if ENABLE_ESP_NOW_CLIENT
          Serial.printf("STATUS espnow enabled=%d ready=%d ch=%u pref=%u rx=%lu seq=%u fresh=%d\n",
                        g_featureEspNow ? 1 : 0,
                        g_espNowReady ? 1 : 0,
                        g_espNowActiveChannel,
                        g_espNowChannelPref,
                        static_cast<unsigned long>(g_espNowRx),
                        g_espNowSeq,
                        espNowDataFresh() ? 1 : 0);
#endif
        }
        else { Serial.println("Commands: status | ble:on|off|hub|123|scan | buzzer:on|off | wifi:next|off | rot:+|-|NN | imu:zero | clock"); }
      }
    } else if (serialLine.length() < 64) {
      serialLine += c;
    }
  }

#if FEATURE_TACHO_DIMMER
  {
    static uint32_t dimmerLogMs = 0;
    const uint32_t now = millis();
    if (now - dimmerLogMs >= 2000) {
      dimmerLogMs = now;
      const int raw = hal_tacho_dimmer_read_raw();
      const int pct = hal_tacho_dimmer_read_pct();
      Serial.printf("[DIMMER] raw=%d pct=%d\n", raw, pct);
    }
  }
#endif

  // Fallback-Setup-AP verwalten (nur AN wenn keine STA-Verbindung)
  manageWifiAp();

  // Web server — weniger Polls, Touch hat Vorrang
  if (g_webStarted) {
    const uint8_t webPolls = (touchBusy || g_bleDiscoveryScan) ? 2 : 4;
    webServerPoll(webPolls);
  }

  if (!touchBusy) tickDialCacheRebuild();

  // WiFi/NTP background tick; redraw clock on fresh sync
  if (!touchBusy && wifiNtpTick() && currentPage == 0) drawVdoClock();

  // Hub data via WiFi HTTP (default path; no BLE slot conflict with M5 Dial)
  if (!touchBusy && !g_otaBusy) {
    hubWifiPollTick();
  }

  // 123 kick state machine: non-blocking, also while touch active
  if (g_featureBle && g_ble123KickStage != BLE_123_KICK_IDLE) {
    ble123KickTick();
  }

  // BLE: only when explicitly selected (fallback); avoids fighting M5 for hub slot
  if (g_featureBle && g_dataPath == DATA_PATH_BLE && (!touchBusy || bleDoConnect)) {
    static uint32_t bleTickMs = 0;
    if (millis() - bleTickMs >= BLE_TICK_MS) {
      bleTickMs = millis();
      bleTick();
    }
  }

  // Clock: redraw when wall-clock time changes
  if (!touchBusy && currentPage == 0 && millis() - lastDraw >= 500) {
    struct tm now = {};
    if (readClockTime(&now)) {
      if (!g_lastClockDrawTmValid
          || now.tm_hour != g_lastClockDrawTm.tm_hour
          || now.tm_min != g_lastClockDrawTm.tm_min
          || now.tm_sec != g_lastClockDrawTm.tm_sec) {
        lastDraw = millis();
        drawVdoClock();
      }
    }
  }
  // Data pages: update at 1 Hz
  if (!touchBusy && currentPage >= 2 && currentPage <= PAGE_MAX
      && currentPage != 6 && currentPage != 7
      && millis() - lastDraw >= 1000) {
    lastDraw = millis();
    drawCurrentPage();
  }
  // IMU page: sensor read is cheap, but the full dial redraw clears the frame.
  // Redraw only on visible motion, otherwise keep a slow heartbeat update.
  if (!touchBusy && currentPage == 6) {
    static uint32_t lastImuReadMs = 0;
    static uint32_t lastImuDrawMs = 0;
    static float lastImuDrawPitch = 999.0f;
    static float lastImuDrawRoll = 999.0f;
    const uint32_t nowMs = millis();
    if (nowMs - lastImuReadMs >= 250) {
      lastImuReadMs = nowMs;
      qmi8658Read();
      const bool firstDraw = lastImuDrawPitch > 900.0f;
      const bool movedEnough = fabsf(g_imuPitch - lastImuDrawPitch) >= 0.5f
                            || fabsf(g_imuRoll - lastImuDrawRoll) >= 0.5f;
      const bool heartbeat = nowMs - lastImuDrawMs >= 1500;
      if (firstDraw || movedEnough || heartbeat) {
        lastDraw = nowMs;
        lastImuDrawMs = nowMs;
        lastImuDrawPitch = g_imuPitch;
        lastImuDrawRoll = g_imuRoll;
        drawCurrentPage();
      }
    }
  }

  delay(touchBusy ? 0 : 1);
}
