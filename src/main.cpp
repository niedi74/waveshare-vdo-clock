// Waveshare ESP32-S3-Touch-LCD-2.8C - VDO Quartz-Zeit Clock
// Main app: clock, touch menu, WiFi/NTP, Spartan3-Hub BLE client, WebGUI.
// Display hardware ownership lives in hal_waveshare_28c.h.
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
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
#define BLE_RECONNECT_123_MS    3000
#define BLE_TICK_MS           300
#define BLE_SCAN_WEB_WAIT_MS  800
#define BLELOG(...) Serial.printf(__VA_ARGS__)
#define BLELOGN(msg) Serial.println(msg)

enum BleConnMode : uint8_t { BLE_MODE_DIRECT_123 = 0, BLE_MODE_SPARTAN_HUB = 1 };

struct BleScanEntry {
  char mac[18];
  char name[24];
  int8_t rssi;
  bool spartan;
  bool nus;
};

static float g_lambda = 0, g_rpm = 0, g_adv = 0, g_map = 0;
static float g_battVolt = 0, g_speedKmh = 0;
static float g_g123Volt = 0, g_g123Temp = 0, g_g123Coil = 0;
static bool  g_lambdaValid = false, g_battValid = false;
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
static BleConnMode        g_bleConnMode = BLE_MODE_SPARTAN_HUB;
static char               g_bleTargetMac[18] = DEFAULT_123_MAC;  // 123 direkt
static char               g_bleHubMac[18]    = "";               // Hub (leer bis Scan/Connect)
static BleScanEntry       g_bleScanList[BLE_SCAN_MAX];
static uint8_t            g_bleScanCount = 0;
static volatile bool      g_bleDiscoveryScan = false;
static bool               g_bleScanWifiSleepWasOn = false;
static bool               g_bleOpScanActive = false;
static volatile bool      g_bleSetupBusy = false;
static uint8_t            g_blePickIndex = 0;

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
static const int DIAL_SCALE_DEFAULT = 115;  // sweet spot: fills display, chrome cropped
static const int DIAL_SCALE_MIN     = 30;
static const int DIAL_SCALE_MAX     = 120;
static const int DIAL_CENTER        = 240;
// Rotation pivot must match setPixel/readTouch (not DIAL_CENTER) so dial cache and hands align.
static constexpr float ROT_PIVOT    = 239.5f;
static const int DIAL_CENTER_OFF_X_DEFAULT = 4;   // bitmap optical center @115% (hands stay at 240,240)
static const int DIAL_CENTER_OFF_Y_DEFAULT = -9;
static const uint16_t DIAL_FACE_BG  = RGB565(52, 47, 45);
static int       g_dialScalePct = DIAL_SCALE_DEFAULT;
static int       g_dialCenterOffsetX = DIAL_CENTER_OFF_X_DEFAULT;
static int       g_dialCenterOffsetY = DIAL_CENTER_OFF_Y_DEFAULT;
static int       g_brightnessPct = 100;
static int       g_rotationDeg  = 0;
static float     g_rotSin       = 0.0f;
static float     g_rotCos       = 1.0f;
// Throttled debug logging (115200 baud Serial)
static uint32_t  g_logTimeLastMs  = 0;
static uint32_t  g_logDrawLastMs  = 0;
static int       g_logLastRotDeg  = -1;
// Verbose Serial in hot paths blocks the main loop (USB-CDC @115200).
#ifndef FEATURE_VERBOSE_SERIAL
#define FEATURE_VERBOSE_SERIAL 0
#endif
#ifndef DEBUG_SERIAL
#define DEBUG_SERIAL 0
#endif
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
static void cycleWifiProfile();
static void updateRotationCache();
static void logicalToDisplay(int lx, int ly, int *dx, int *dy);
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

// ---- Timezone / NTP ----
struct TimezoneEntry {
  const char* label;
  const char* posix;
};

static const TimezoneEntry TIMEZONES[] = {
  { "Europe/Berlin (CET/CEST)", "CET-1CEST,M3.5.0,M10.5.0/3" },
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
static bool     g_ntpResyncRequested = false;
static uint32_t g_lastRtcSyncMs    = 0;
static constexpr uint32_t NTP_RESYNC_INTERVAL_MS = 15UL * 60UL * 1000UL;  // 15 min (ESP default: 1 h)

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

static bool readClockTime(struct tm *now) {
  // Single time source for WebGUI and drawVdoClock: SNTP (TZ via configTzTime) first.
  // PCF85063 is fallback only before NTP; never prefer stale RTC when SNTP is valid.
  const char *src = "none";
  time_t t = time(nullptr);
  if (t > 1700000000) {
    localtime_r(&t, now);
    src = "SNTP";
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
  if (!g_sntpStarted) {
    applyTimezone();
    g_sntpStarted = true;
    Serial.printf("NTP: SNTP gestartet (TZ %s)\n", timezoneLabel(g_timezoneIdx));
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
  g_lambda = g_rpm = g_adv = g_map = 0;
  g_battVolt = g_speedKmh = 0;
  g_g123Volt = g_g123Temp = g_g123Coil = 0;
  g_lambdaValid = g_battValid = g_speedValid = g_g123Valid = false;
  g_bleRxCnt = 0;
  g_bleLastRx = 0;
}

static void bleWaitScanStopped(NimBLEScan* s, uint32_t timeoutMs) {
  const uint32_t until = millis() + timeoutMs;
  while (s->isScanning() && millis() < until) {
    if (g_webStarted) webServer.handleClient();
    delay(10);
    yield();
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
  auto* s = NimBLEDevice::getScan();
  s->stop();
  bleWaitScanStopped(s, 500);
  g_bleDiscoveryScan = false;
  bleDiscoveryScanEnded();
}

static void disconnectBleForModeChange() {
  bleDoConnect = false;
  g_bleConn = false;
  g_bleHubName = "---";
  g_nusRx = nullptr;
  g_ble123PingAt = 0;
  clearBleLiveValues();
  bleAbortDiscoveryScan();
  NimBLEDevice::getScan()->stop();
  if (bleClient && bleClient->isConnected()) bleClient->disconnect();
  bleNextScanAt = millis() + 2000;
}

static void saveBleConnMode(BleConnMode mode) {
  g_bleConnMode = mode;
  Preferences p;
  p.begin("clock", false);
  p.putUChar("ble_mode", static_cast<uint8_t>(g_bleConnMode));
  p.end();
}

static void saveBleMac123(const char* mac) {
  strncpy(g_bleTargetMac, mac, sizeof(g_bleTargetMac) - 1);
  g_bleTargetMac[sizeof(g_bleTargetMac) - 1] = 0;
  Preferences p;
  p.begin("clock", false);
  p.putString("ble_mac_123", g_bleTargetMac);
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
    BLELOG("BLE: link up (%s)\n", bleConnModeLabel());
    // g_bleConn erst nach NUS/Hub-Subscribe (sonst bricht Setup ab)
    if (g_bleConnMode == BLE_MODE_SPARTAN_HUB) g_bleConn = true;
  }
  void onDisconnect(NimBLEClient*, int reason) override {
    g_bleConn = false;
    g_bleHubName = "---";
    g_nusRx = nullptr;
    g_ble123PingAt = 0;
    BLELOG("BLE: getrennt (reason=%d)\n", reason);
    uint32_t pause = (g_bleConnMode == BLE_MODE_DIRECT_123) ?
                     BLE_RECONNECT_123_MS : BLE_BG_SCAN_INTERVAL_MS;
    if (g_bleSetupBusy) pause = 2000;
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
    else saveBleMac123(addr.c_str());
    bleTarget    = dev->getAddress();
    bleDoConnect = true;
    NimBLEDevice::getScan()->stop();
    BLELOG("BLE: %s gefunden (%s / %s)\n",
           g_bleConnMode == BLE_MODE_SPARTAN_HUB ? "Hub" : "123",
           addr.c_str(), name.c_str());
  }
  void onScanEnd(const NimBLEScanResults& results, int) override {
    if (g_bleDiscoveryScan) {
      g_bleDiscoveryScan = false;
      bleDiscoveryScanEnded();
      BLELOG("BLE: Discovery %u Geraete\n", g_bleScanCount);
      return;
    }
    bleWifiSleepRestore();
    BLELOG("BLE: Scan Ende r=%d (%s)\n", results.getCount(), bleConnModeLabel());
    if (!g_bleConn && !bleDoConnect) {
      const uint32_t pause = (g_bleConnMode == BLE_MODE_DIRECT_123) ?
                             BLE_RECONNECT_123_MS : BLE_BG_SCAN_INTERVAL_MS;
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
    // BM6/123: Adresstyp aus MAC (ef:.. = static random), wie M5Dial aus Scan
    bleTarget = NimBLEAddress(std::string(macStr), BLE_ADDR_RANDOM);
  } else {
    bleTarget = NimBLEAddress(std::string(macStr), BLE_ADDR_PUBLIC);
  }
  bleDoConnect = true;
  BLELOG("BLE: Direktconnect %s (%s)\n", macStr, bleConnModeLabel());
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
  BLELOG("BLE: Scan %lus (%s @ %s)...\n",
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

static void ble123PingTick();

static void blePauseMs(uint32_t ms) {
  const uint32_t end = millis() + ms;
  while (millis() < end) {
    if (g_webStarted) webServerPoll(1);
    delay(5);
    yield();
  }
}

static void ble123KickLive(NimBLERemoteCharacteristic* rx) {
  if (!rx || !rx->canWrite()) return;
  const uint8_t dollar[] = {'$'};
  const uint8_t cr[] = {'\r'};
  rx->writeValue(dollar, 1, false);
  blePauseMs(80);
  rx->writeValue(cr, 1, false);
  BLELOGN("BLE: 123 kick $ + CR (M5Dial)");
}

static void ble123PingTick() {
  if (!g_bleConn || g_bleConnMode != BLE_MODE_DIRECT_123 || !g_nusRx) return;
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
  bleWaitScanStopped(s, 800);
}

static void bleConnectEnd() {
  g_bleSetupBusy = false;
  bleWifiSleepRestore();
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
    bleClient->setConnectionParams(32, 32, 0, 400);
  }
  bleConnectBegin();
  BLELOG("BLE: Connect %s (%s)...\n", bleTarget.toString().c_str(), bleConnModeLabel());

  bool ok = false;
  for (uint8_t attempt = 0; attempt < 2 && !ok; attempt++) {
    if (attempt > 0) {
      BLELOG("BLE: Connect retry %u\n", (unsigned)(attempt + 1));
      if (bleClient->isConnected()) bleClient->disconnect();
      blePauseMs(300);
    }
    if (!bleClient->connect(bleTarget, false, false, false)) continue;
    blePauseMs(500);
    if (!bleLinkUp()) continue;
    bleClient->discoverAttributes();
    blePauseMs(200);
    if (!bleLinkUp()) continue;

    if (g_bleConnMode == BLE_MODE_SPARTAN_HUB) {
      auto* svc = bleClient->getService(SPARTAN_SVC);
      if (!svc || !bleLinkUp()) continue;
      auto* status = svc->getCharacteristic(SPARTAN_STATUS);
      if (!status || !bleLinkUp()) continue;
      ok = status->subscribe(true, bleNotifyHubCB, true);
      BLELOG("BLE: Hub-Subscribe %s\n", ok ? "OK" : "FAIL");
      if (ok) g_bleConn = true;
      break;
    }

    auto* svc = bleClient->getService(NUS_SVC);
    if (!svc || !bleLinkUp()) continue;
    auto* tx = svc->getCharacteristic(NUS_TX);
    if (!tx || !bleLinkUp()) continue;
    tx->subscribe(false, nullptr, false);
    blePauseMs(100);
    ok = tx->subscribe(true, bleNotify123CB, true);
    BLELOG("BLE: 123-Subscribe %s\n", ok ? "OK" : "FAIL");
    if (!ok || !bleLinkUp()) { ok = false; continue; }
    g_nusRx = svc->getCharacteristic(NUS_RX);
    ble123KickLive(g_nusRx);
    g_ble123PingAt = millis();
    g_bleConn = true;
    BLELOG("BLE: verbunden (%s)\n", bleConnModeLabel());
    break;
  }

  bleConnectEnd();
  if (!ok) {
    BLELOG("BLE: Connect fehlgeschlagen (%s)\n", bleSavedMacForMode());
    if (bleClient && bleClient->isConnected()) bleClient->disconnect();
    bleNextScanAt = millis() + 5000;
  }
}

static void bleTick() {
  if (g_bleSetupBusy) return;
  if (g_bleConn && g_bleConnMode == BLE_MODE_DIRECT_123) {
    ble123PingTick();
    return;
  }
  if (bleDoConnect) { bleConnect(); return; }
  if (g_bleConn || g_bleDiscoveryScan) return;
  if (bleNextScanAt == 0 || millis() < bleNextScanAt) return;
  bleNextScanAt = 0;
  if (g_bleConnMode == BLE_MODE_DIRECT_123) {
    bleStartScan();
    return;
  }
  static uint8_t s_hubDirectTry = 0;
  if ((++s_hubDirectTry % 3) == 0 && bleTryConnectSavedMac()) return;
  bleStartScan();
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

static bool readTouch(uint16_t *x, uint16_t *y) {
  if (!gt911Found) return false;
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

static void setPixel(int x, int y, uint16_t color) {
  if (g_rotationDeg != 0) {
    float dx = (float)x - ROT_PIVOT;
    float dy = (float)y - ROT_PIVOT;
    x = (int)lroundf(dx * g_rotCos - dy * g_rotSin + ROT_PIVOT);
    y = (int)lroundf(dx * g_rotSin + dy * g_rotCos + ROT_PIVOT);
    if ((unsigned)x >= 480 || (unsigned)y >= 480) return;
  }
  setPixelFb(x, y, color);
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

static uint16_t faceColorAtFb(int px, int py) {
  if ((unsigned)px >= 480 || (unsigned)py >= 480) return DIAL_FACE_BG;
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
  return DIAL_FACE_BG;
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
  for (int i = 0; i < 480 * 480; i++) g_dialCache[i] = DIAL_FACE_BG;
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
    if (g_webStarted) webServer.handleClient();
    yield();
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
  for (int i = 0; i < 480 * 480; i++) fb[i] = DIAL_FACE_BG;
}

static void logicalToDisplay(int lx, int ly, int *dx, int *dy) {
  if (g_rotationDeg == 0) {
    *dx = lx;
    *dy = ly;
    return;
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
  g_clockFacePage = currentPage;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(80, 80, 75));
  drawTextCentered(240, 50, "MENU", RGB565(235, 235, 225), 6);
  // 6 Tiles, inset 2px from zone boundary so visual == touchable area
  drawMenuTile(88, MENU_ZONE_Y0 +   0 + 2, 304, MENU_ZONE_H - 4, "UHR",    RGB565(200, 40,  35));
  drawMenuTile(88, MENU_ZONE_Y0 +  50 + 2, 304, MENU_ZONE_H - 4, "MOTOR",  RGB565(40,  150, 210));
  webServerPoll(2);
  drawMenuTile(88, MENU_ZONE_Y0 + 100 + 2, 304, MENU_ZONE_H - 4, "LAMBDA", RGB565(60,  185, 90));
  drawMenuTile(88, MENU_ZONE_Y0 + 150 + 2, 304, MENU_ZONE_H - 4, "HUB",    RGB565(190, 90,  210));
  webServerPoll(2);
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

static void drawMotorPage() {
  if (!ensureFrame()) return;
  g_clockFacePage = currentPage;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(40, 110, 160));
  drawTextCentered(240, 52, "MOTOR", RGB565(60, 170, 230), 5);
  bool fresh = bleFresh();
  char buf[16];
  {
    char lbuf[12];
    uint16_t lcol = RGB565(110, 60, 60);
    if (fresh && g_lambdaValid) {
      snprintf(lbuf, sizeof(lbuf), "L %.2f", g_lambda);
      if (g_lambda < 0.97f) lcol = RGB565(235, 120, 40);
      else if (g_lambda > 1.03f) lcol = RGB565(80, 160, 240);
      else lcol = RGB565(70, 210, 100);
    } else {
      strcpy(lbuf, "L ---");
    }
    drawTextCentered(240, 82, lbuf, lcol, 2);
  }
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
  const char* st = g_bleConn ? (fresh ? "LIVE" : "WARTE") :
                   (g_bleConnMode == BLE_MODE_SPARTAN_HUB ? "KEIN HUB" : "KEIN 123");
  drawTextCentered(240, 370, st, g_bleConn && fresh ? RGB565(60, 200, 90) : RGB565(200, 120, 50), 2);
  presentFrame();
}

static void drawLambdaPage() {
  if (!ensureFrame()) return;
  g_clockFacePage = currentPage;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(45, 150, 70));
  drawTextCentered(240, 58, "LAMBDA", RGB565(70, 200, 100), 5);
  bool bleOk = bleFresh();
  bool fresh = bleOk && g_lambdaValid;
  char buf[16];
  {
    char mbuf[12];
    if (bleOk) snprintf(mbuf, sizeof(mbuf), "MAP %d", (int)g_map);
    else strcpy(mbuf, "MAP ---");
    drawTextCentered(240, 88, mbuf, bleOk ? RGB565(180, 180, 180) : RGB565(110, 60, 60), 2);
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
  const char* st = g_bleConn ? (bleFresh() ? "LIVE" : "WARTE") :
                   (g_bleConnMode == BLE_MODE_SPARTAN_HUB ? "KEIN HUB" : "KEIN 123");
  drawTextCentered(240, 370, st, g_bleConn && bleFresh() ? RGB565(60, 200, 90) : RGB565(200, 120, 50), 2);
  presentFrame();
}

static void drawHubPage() {
  if (!ensureFrame()) return;
  g_clockFacePage = currentPage;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(150, 70, 180));
  drawTextCentered(240, 54, "BLE", RGB565(205, 120, 230), 5);
  drawTextCentered(240, 84, bleConnModeLabel(), RGB565(150, 150, 150), 2);
  char buf[24];
  drawDataRow(112, "ZIEL",  bleTargetDisplay(), RGB565(235, 235, 225));
  drawDataRow(148, "BLE",   g_bleConn ? "OK" : "SCAN",
              g_bleConn ? RGB565(60, 210, 100) : RGB565(220, 130, 50));
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)g_bleRxCnt);
  drawDataRow(188, "RX",    buf, RGB565(235, 235, 225));
  snprintf(buf, sizeof(buf), "%lu MS", g_bleLastRx ? (unsigned long)(millis() - g_bleLastRx) : 0UL);
  drawDataRow(228, "AGE",   buf, RGB565(235, 235, 225));
  snprintf(buf, sizeof(buf), "%.1f V", g_battVolt);
  drawDataRow(268, "BATT",  g_battValid  ? buf : "---", RGB565(235, 235, 225));
  snprintf(buf, sizeof(buf), "%.0f KMH", g_speedKmh);
  drawDataRow(308, "SPEED", g_speedValid ? buf : "---", RGB565(235, 235, 225));
  drawDataRow(348, "IP",    g_ipStr, RGB565(150, 200, 150));
  drawTextCentered(240, 370, "TIP MENU", RGB565(180, 180, 170), 2);
  presentFrame();
}

static void drawSetupPage() {
  if (!ensureFrame()) return;
  g_clockFacePage = currentPage;
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
    snprintf(buf, sizeof(buf), "%s", currentWifiSsid());
    drawDataRow(218, "WIFI", buf,
                WiFi.status() == WL_CONNECTED ? RGB565(60, 210, 100) : RGB565(220, 130, 50));
  } else {
    drawDataRow(218, "WIFI", "AUS", RGB565(220, 130, 50));
  }
  drawDataRow(254, "BLE",    g_featureBle ? (g_bleConn ? "OK" : "AN") : "AUS",
              g_featureBle && g_bleConn ? RGB565(60, 210, 100) : RGB565(220, 130, 50));
  drawDataRow(290, "BUZZER", g_featureBuzzer ? "AN" : "AUS",
              g_featureBuzzer ? RGB565(60, 210, 100) : RGB565(150, 150, 150));
  drawDataRow(326, "QUELLE", bleConnModeLabel(),
              g_bleConnMode == BLE_MODE_SPARTAN_HUB ? RGB565(80, 160, 240) : RGB565(235, 150, 60));
  drawDataRow(362, "IMU NULL", g_imuTrimmed ? "SET" : "TAP",
              g_imuTrimmed ? RGB565(60, 210, 100) : RGB565(220, 130, 50));
  drawTextCentered(240, 396, "TIP MENU", RGB565(180, 180, 170), 2);
  presentFrame();
}

static void drawImuPage() {
  if (!ensureFrame()) return;
  g_clockFacePage = currentPage;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(200, 100, 50));
  drawTextCentered(240, 52, "IMU", RGB565(220, 130, 60), 5);
  if (g_imuTrimmed) {
    drawTextCentered(240, 78, "NULL", RGB565(60, 200, 90), 2);
  }

  char buf[16];
  if (g_imuPresent) {
    snprintf(buf, sizeof(buf), "%.1f", g_imuPitch);
    drawDataRow(112, "PITCH", buf, RGB565(235, 235, 225));
    snprintf(buf, sizeof(buf), "%.1f", g_imuRoll);
    drawDataRow(152, "ROLL", buf, RGB565(235, 235, 225));
    snprintf(buf, sizeof(buf), "%.2fg", g_imuGForce);
    drawDataRow(192, "G-FORCE", buf, RGB565(235, 235, 225));

    static bool buzzerOn = false;
    const bool wantBuzz = g_featureBuzzer && qmi8658ShakeDetected(1.5f);
    if (qmi8658ShakeDetected(1.5f)) {
      drawTextCentered(240, 250, "SHAKE!", RGB565(255, 50, 50), 4);
    } else {
      drawTextCentered(240, 250, "OK", RGB565(60, 200, 90), 4);
    }
    if (wantBuzz != buzzerOn) {   // nur bei Zustandswechsel schalten
      buzzerOn = wantBuzz;
      hal_buzzer(buzzerOn);
    }
  } else {
    drawTextCentered(240, 200, "KEIN IMU", RGB565(200, 60, 60), 4);
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
}

static void requestPage(uint8_t page) {
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
  g_rotationDeg   = p.getInt("rot_deg",   0);
  g_wifiProfile   = p.getUChar("wifi_prof", 0);
  if (g_wifiProfile >= wifiProfileCount()) g_wifiProfile = 0;
  g_featureWifi   = p.getBool("feat_wifi", strlen(currentWifiSsid()) > 0);
  g_featureBle    = p.getBool("feat_ble",  false);
  g_featureBuzzer = p.getBool("feat_buzzer", false);  // default OFF
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
    case 5: return "SETUP";
    case 6: return "IMU";
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
    const char* items[] = {"UHR", "MOTOR", "LAMBDA", "HUB", "IMU", "SETUP"};
    const char* cols[] = {"#c82823", "#2896d2", "#3cb95a", "#be5ad2", "#c86432", "#d2bc2d"};
    for (int i = 0; i < 6; i++) {
      char row[128];
      snprintf(row, sizeof(row),
               "<rect x='44' y='%d' width='152' height='18' rx='3' fill='#121212' stroke='#333'/>"
               "<rect x='44' y='%d' width='6' height='18' fill='%s'/>"
               "<text x='58' y='%d' fill='#ddd' font-family='system-ui,sans-serif' font-size='9'>%s</text>",
               78 + i * 22, 78 + i * 22, cols[i], 91 + i * 22, items[i]);
      svg += row;
    }
  } else if (page == 2) {
    const bool fresh = bleFresh();
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
    svg += fresh ? (g_bleConn ? F("LIVE") : F("WARTE")) : F("---");
    svg += F("</text>");
  } else if (page == 3) {
    char line[24];
    if (g_lambdaValid) snprintf(line, sizeof(line), "%.2f", g_lambda);
    else strcpy(line, "----");
    svg += F("<text x='120' y='132' text-anchor='middle' fill='#46d06a' font-size='28' font-family='system-ui,sans-serif' font-weight='700' class='pv-lambda'>");
    svg += line;
    svg += F("</text>");
  } else if (page == 4) {
    svg += F("<text x='120' y='118' text-anchor='middle' fill='#");
    svg += g_bleConn ? F("54d273") : F("e7a944");
    svg += F("' font-size='14' font-family='system-ui,sans-serif' class='pv-hub-st'>");
    svg += g_bleConn ? F("HUB OK") : F("KEIN HUB");
    svg += F("</text><text x='120' y='142' text-anchor='middle' fill='#aaa' font-size='10' font-family='system-ui,sans-serif'>RX <tspan class='pv-rx'>");
    svg += String((unsigned long)g_bleRxCnt);
    svg += F("</tspan></text>");
  } else if (page == 5) {
    svg += F("<text x='72' y='118' fill='#888' font-size='9' font-family='system-ui,sans-serif'>WLAN</text>"
             "<text x='168' y='118' text-anchor='end' fill='#eee' font-size='10' font-family='system-ui,sans-serif'>");
    svg += g_featureWifi ? F("an") : F("aus");
    svg += F("</text><text x='72' y='142' fill='#888' font-size='9' font-family='system-ui,sans-serif'>BLE</text>"
             "<text x='168' y='142' text-anchor='end' fill='#eee' font-size='10' font-family='system-ui,sans-serif'>");
    svg += g_featureBle ? F("an") : F("aus");
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
    "#t0:checked~.tabs label[for=t0],#t1:checked~.tabs label[for=t1],#t2:checked~.tabs label[for=t2],#t3:checked~.tabs label[for=t3],#t4:checked~.tabs label[for=t4]{background:var(--gold);color:#161100;border-color:var(--gold)}"
    "#t0:checked~#p0,#t1:checked~#p1,#t2:checked~#p2,#t3:checked~#p3,#t4:checked~#p4{display:block}"
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
  html += F("</span></div><input class='tab' id='t0' name='tab' type='radio' checked><input class='tab' id='t1' name='tab' type='radio'><input class='tab' id='t2' name='tab' type='radio'><input class='tab' id='t3' name='tab' type='radio'><input class='tab' id='t4' name='tab' type='radio'>"
            "<nav class='tabs'><label for='t0'>Dashboard</label><label for='t1'>WLAN</label><label for='t2'>Display</label><label for='t3'>Live</label><label for='t4'>Setup</label></nav>");

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
  html += F("</div></div><div class='card row'><div><b>BLE Hub</b><br><span id='dashBle' class='");
  html += g_bleConn ? "ok" : "warn";
  html += "'>" + String(g_featureBle ? (g_bleConn ? "verbunden" : "scan/wartet") : "aus") + "</span></div>";
  html += "<div class='pill'>RX <span id='dashRx'>" + String((unsigned long)g_bleRxCnt) + "</span></div></div></section>";

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
  html += F("</div><button onclick='scanWifi()'>&#128269; Scan</button><table><thead><tr><th>SSID</th><th>RSSI</th><th>Status</th></tr></thead><tbody id='scanRows'><tr><td colspan='3'>Noch kein Scan</td></tr></tbody></table></div></section>");

  html += F("<section class='page' id='p2'><div class='card'><h2>Display</h2><div id='displayPreviewHost'>");
  html += webDisplayPreviewShell(&now);
  html += F("</div><div class='grid'>"
            "<a href='/page?p=0'><button>Uhr</button></a><a href='/page?p=1'><button>Menu</button></a><a href='/page?p=2'><button>Motor</button></a><a href='/page?p=3'><button>Lambda</button></a><a href='/page?p=4'><button>Hub</button></a><a href='/page?p=6'><button>IMU</button></a><a href='/page?p=5'><button>Setup</button></a></div></div>"
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
  html += F("</b></div><div class='row'><span><a href='/set?dial_x_delta=-1'><button>X -1</button></a><a href='/set?dial_x_delta=1'><button>X +1</button></a></span></div><div class='row'><span><a href='/set?dial_y_delta=-1'><button>Y -1</button></a><a href='/set?dial_y_delta=1'><button>Y +1</button></a><a href='/set?dial_reset=1'><button>Zentrum</button></a></span></div></div></section>");

  html += F("<section class='page' id='p3'><div class='card'><h2>Live</h2><pre id='liveBox'>Lade /api/status ...</pre></div></section>");

  html += F("<section class='page' id='p4'><div class='card'><h2>Setup</h2><form action='/features' method='get'>"
            "<p><label><input type='checkbox' name='wifi' value='1' ");
  html += g_featureWifi ? F("checked") : F("");
  html += F("> WLAN/Web aktiv</label></p><p><label><input type='checkbox' name='ble' value='1' ");
  html += g_featureBle ? F("checked") : F("");
  html += F("> BLE Daten aktiv</label></p><p><label><input type='checkbox' name='buzzer' value='1' ");
  html += g_featureBuzzer ? F("checked") : F("");
  html += F("> Buzzer aktiv</label></p><button type='submit'>Speichern</button></form></div>"
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
            "const pageNames=['UHR','MENU','MOTOR','LAMBDA','HUB','SETUP','IMU'];"
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
            "if(d.page===0){updateHands();return;}if(d.page===2){const ok=d.ble_connected&&d.ble_enabled;"
            "pvSet('.pv-rpm',ok?String(Math.round(d.rpm||0)):'0');pvSet('.pv-adv',ok?String(Math.round(d.adv||0)):'0');"
            "pvSet('.pv-map',ok?String(Math.round(d.map||0)):'0');pvSet('.pv-batt',d.volt_valid?num(d.volt).toFixed(1)+'V':'---');"
            "pvSet('.pv-motor-st',ok?(d.ble_connected?'LIVE':'WARTE'):'---');}"
            "if(d.page===3)pvSet('.pv-lambda',d.lambda_valid?num(d.lambda).toFixed(2):'----');"
            "if(d.page===4){pvSet('.pv-hub-st',d.ble_connected?'HUB OK':'KEIN HUB');pvSet('.pv-rx',String(d.ble_rx||0));}"
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
            "set('dashPage',String(d.page));set('dashPageLabel',pageNames[d.page]||'?');set('dashRx',String(d.ble_rx||0));"
            "const ble=document.getElementById('dashBle');if(ble){ble.textContent=d.ble_enabled?(d.ble_connected?'verbunden':'scan/wartet'):'aus';"
            "ble.className=d.ble_enabled&&d.ble_connected?'ok':'warn';}"
            "syncBleSetup(d);"
            "if(d.page!==lastPreviewPage){lastPreviewPage=d.page;refreshPreview().then(()=>syncPreviewValues(d));}"
            "else syncPreviewValues(d);}"
            "async function fetchWithTimeout(url,ms,opts){const c=new AbortController();const t=setTimeout(()=>c.abort(),ms);try{return await fetch(url,{...(opts||{}),signal:c.signal});}finally{clearTimeout(t);}}"
            "async function scanWifi(){const rows=document.getElementById('scanRows');rows.innerHTML='<tr><td colspan=3>Scan laeuft...</td></tr>';try{const r=await fetchWithTimeout('/scan',12000);const d=await r.json();rows.innerHTML=d.networks.map(n=>`<tr><td>${n.ssid}</td><td>${n.rssi}</td><td>${n.connected?'verbunden':''}</td></tr>`).join('')||'<tr><td colspan=3>Keine Netze</td></tr>';}catch(e){rows.innerHTML='<tr><td colspan=3>Scan fehlgeschlagen</td></tr>';}}"
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
  json.reserve(640);
  json += F("{\"time\":");
  json += String((unsigned long)time(nullptr));
  json += F(",\"hour\":");
  json += String(now.tm_hour);
  json += F(",\"min\":");
  json += String(now.tm_min);
  json += F(",\"sec\":");
  json += String(now.tm_sec);
  json += F(",\"tz_idx\":");
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
  json += F(",\"ble_enabled\":");
  json += g_featureBle ? F("true") : F("false");
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
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

static void handleWebFeatures() {
  const bool wifi   = webServer.hasArg("wifi");
  const bool ble    = webServer.hasArg("ble");
  const bool buzzer = webServer.hasArg("buzzer");
  saveFeatures(wifi, ble, buzzer);
  DLOG("Web: Funktionen wifi=%s ble=%s buzzer=%s\n",
                g_featureWifi ? "on" : "off", g_featureBle ? "on" : "off",
                g_featureBuzzer ? "on" : "off");
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
    if (page > 6) page = 6;
    currentPage  = static_cast<uint8_t>(page);
    g_redrawPage = true;
    DLOG("Web: page=%u\n", currentPage);
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
    DLOG("Web: WLAN-Profil -> %d (%s)\n", idx, currentWifiSsid());
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
#if FEATURE_VERBOSE_SERIAL
  Serial.printf("[TOUCH] setup y=%u dur=%lums rot=%d\n", y, (unsigned long)durMs, g_rotationDeg);
#endif

  // Zonen passend zu drawSetupPage (Zeilen-Mitte = Zonenstart + 18, Hoehe 36)
  if (y >= 380) {
    requestPage(1);
    DLOGN("setup tap: menu");
    return;
  }

  if (y >= 92 && y < 128) {
    int next = (g_dialScalePct < 115) ? 115 : (g_dialScalePct < 120 ? 120 : 115);
    saveDialScale(next);
    g_redrawPage = true;
    DLOG("setup tap: dial=%d%%\n", g_dialScalePct);
  } else if (y >= 128 && y < 164) {
    int next = (g_brightnessPct < 63) ? 75 : (g_brightnessPct < 88 ? 100 : 50);
    saveBrightness(next);
    g_redrawPage = true;
    DLOG("setup tap: brightness=%d%%\n", g_brightnessPct);
  } else if (y >= 164 && y < 200) {
    int next = (g_rotationDeg < 90) ? 90 : (g_rotationDeg < 180 ? 180 : (g_rotationDeg < 270 ? 270 : 0));
    saveRotation(next);
    g_redrawPage = true;
    DLOG("[ROT] setup tap -> %d deg\n", g_rotationDeg);
  } else if (y >= 200 && y < 236) {
    cycleWifiProfile();
    g_redrawPage = true;
    DLOG("setup tap: wifi profile=%u ssid=%s\n", g_wifiProfile, currentWifiSsid());
  } else if (y >= 236 && y < 272) {
    saveFeatures(g_featureWifi, !g_featureBle, g_featureBuzzer);
    g_redrawPage = true;
    DLOG("setup tap: ble=%s\n", g_featureBle ? "on" : "off");
  } else if (y >= 272 && y < 308) {
    saveFeatures(g_featureWifi, g_featureBle, !g_featureBuzzer);
    g_redrawPage = true;
    DLOG("setup tap: buzzer=%s\n", g_featureBuzzer ? "on" : "off");
  } else if (y >= 308 && y < 344) {
    cycleBleConnMode();
    g_redrawPage = true;
    DLOG("setup tap: ble_source=%s\n", bleConnModeLabel());
  } else if (y >= 344 && y < 380) {
    if (calibrateImuZero()) g_redrawPage = true;
    DLOGN("setup tap: imu_null");
  } else {
    g_redrawPage = true;
    DLOGN("setup tap: no action");
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
  otaNoteBootState();
  Serial.printf("PSRAM found: %s, size: %u bytes\n", psramFound() ? "yes" : "no", ESP.getPsramSize());

  // Backlight diagnostic blink (2x 50ms) before panel init.
  // Später: ADC-Dimming vom Original-Tacho-Drehregler — siehe FUTURE.md
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
  static uint32_t lastTouch = 0;
  static uint32_t lastDraw  = 0;
  static String   serialLine;
  static bool     touchActive = false;
  static bool     touchLongHandled = false;
  static uint32_t touchStartMs = 0;
  static uint32_t touchLastSeenMs = 0;
  static uint16_t touchStartX = 0, touchStartY = 0;
  static uint16_t touchLastX = 0, touchLastY = 0;
  static bool     touchHadPos = false;
  uint16_t x = 0, y = 0;
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
  const uint32_t nowMs = millis();
  const bool touchFrame = readTouch(&x, &y);
  const bool touchHeld = touchActive && (nowMs - touchLastSeenMs < 200);
  const bool touchNow = touchFrame || touchHeld;
  touchBusy = touchNow || touchActive;
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
    }
    if (currentPage == 5 && !touchLongHandled && nowMs - touchStartMs >= 600) {
      touchLongHandled = true;
      lastTouch = nowMs;
      handleSetupLongPress(touchLastY, nowMs - touchStartMs);
    }
  } else if (touchActive && nowMs - touchLastSeenMs > (currentPage == 5 ? 700UL : TOUCH_RELEASE_MS)) {
    const uint32_t durMs = touchLastSeenMs - touchStartMs;
    const uint16_t tapX = touchHadPos ? touchLastX : touchStartX;
    const uint16_t tapY = touchHadPos ? touchLastY : touchStartY;
    touchActive = false;
    touchHadPos = false;
    if (!touchLongHandled && durMs < 600 && nowMs - lastTouch > TOUCH_COOLDOWN_MS) {
      lastTouch = nowMs;
      if (currentPage == 0) {
        requestPage(1);
#if FEATURE_VERBOSE_SERIAL
        Serial.printf("[TOUCH] tap log=%u,%u page=0 dur=%lums rot=%d\n",
                      tapX, tapY, (unsigned long)durMs, g_rotationDeg);
        Serial.println("[TOUCH] hit=clock->menu page=1");
#endif
      } else if (currentPage == 1) {
        // Route by y-position: zones match MENU_ZONE_Y0 / MENU_ZONE_H exactly.
        const uint16_t z0 = MENU_ZONE_Y0;
        const uint16_t zh = MENU_ZONE_H;
        const char *hit = "fallback";
        if      (tapY >= z0       && tapY < z0 +   zh) { requestPage(0); hit = "UHR"; }
        else if (tapY >= z0 +  zh && tapY < z0 + 2*zh) { requestPage(2); hit = "MOTOR"; }
        else if (tapY >= z0 +2*zh && tapY < z0 + 3*zh) { requestPage(3); hit = "LAMBDA"; }
        else if (tapY >= z0 +3*zh && tapY < z0 + 4*zh) { requestPage(4); hit = "HUB"; }
        else if (tapY >= z0 +4*zh && tapY < z0 + 5*zh) { requestPage(6); hit = "IMU"; }
        else if (tapY >= z0 +5*zh && tapY < z0 + 6*zh) { requestPage(5); hit = "SETUP"; }
        else { requestPage(0); }
#if FEATURE_VERBOSE_SERIAL
        Serial.printf("[TOUCH] tap log=%u,%u page=1 dur=%lums rot=%d\n",
                      tapX, tapY, (unsigned long)durMs, g_rotationDeg);
        Serial.printf("[TOUCH] hit=menu:%s page=%u y=%u zone=%u..%u\n",
                      hit, currentPage, tapY, z0, z0 + 6 * zh);
#endif
      } else if (currentPage == 5) {
        handleSetupLongPress(tapY, durMs);
      } else {
        const uint8_t prev = currentPage;
        if      (currentPage == 4) requestPage(6);
        else if (currentPage == 6) requestPage(0);
        else requestPage(currentPage + 1);
#if FEATURE_VERBOSE_SERIAL
        Serial.printf("[TOUCH] hit=page_next %u->%u\n", prev, currentPage);
#endif
      }
    }
  }
#else
  (void)touchBusy;
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
        else if (cmd == "wifi:next") { cycleWifiProfile(); g_redrawPage = true; }
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
        else { Serial.println("Commands: ble:on|off|hub|123|scan | buzzer:on|off | wifi:next|off | rot:+|-|NN | imu:zero | clock"); }
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

  // BLE: nicht waehrend Touch — blockiert sonst GT911/I2C und Display
  if (g_featureBle && !touchBusy) {
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
  if (!touchBusy && currentPage >= 2 && currentPage <= 5 && millis() - lastDraw >= 1000) {
    lastDraw = millis();
    drawCurrentPage();
  }
  // IMU page: ~5 Hz
  if (!touchBusy && currentPage == 6 && millis() - lastDraw >= 200) {
    lastDraw = millis();
    qmi8658Read();
    drawCurrentPage();
  }

  webServerPoll(touchBusy ? 2 : 4);
  delay(touchBusy ? 0 : 1);
}
