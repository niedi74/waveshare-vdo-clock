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
#include "esp_wps.h"               // WLAN per Tastendruck (WPS PBC) - wie im M5 Dial
#include "SD_MMC.h"                // Micro-SD am 2.8C (SDMMC 1-bit: CLK=2/CMD=1/D0=42, D3=EXIO4)
#include <sys/time.h>
#include <time.h>
#include "esp_system.h"             // esp_reset_reason() fuers SD-Bootlog

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

// -------- Board-Akku (MX1.25-LiPo, separat vom Motor-g_battVolt oben) --------
// GPIO4 = BAT_ADC, Teiler R5=200k(BAT)/R9=100k(GND) laut Schaltplan -> Vadc=Vbat/3.
#define BOARD_BAT_ADC_PIN 4
static float g_boardBattVolt = 0.0f;
static bool  g_boardBattPresent = false;   // Spannung plausibel (>2.5V) -> Akku dran+Strom
static void boardBattRead() {
  uint32_t mv = analogReadMilliVolts(BOARD_BAT_ADC_PIN);   // kalibriert (eFuse), genauer als raw*3.3/4095
  g_boardBattVolt = (mv / 1000.0f) * 3.0f;                 // Teiler-Kehrwert (R5+R9)/R9 = 3
  g_boardBattPresent = g_boardBattVolt > 2.5f;
}
// Grobe LiPo-Prozent-Schaetzung aus der Spannung (KEINE echte Ladestandsmessung -
// der Chip hat kein Coulomb-Counting; unter Last/beim Laden weicht das ab).
static int boardBattPct() {
  if (!g_boardBattPresent) return -1;
  float v = constrain(g_boardBattVolt, 3.0f, 4.2f);
  return (int)((v - 3.0f) / (4.2f - 3.0f) * 100.0f);
}
static float g_tripKm = 0.0f;              // Teilstrecke vom Hub (trip_km, Reset via POST /odo)
static bool  g_tripValid = false;
static bool  g_bleConn = false;
static uint32_t g_bleLastRx = 0, g_bleRxCnt = 0;

static NimBLEClient*      bleClient    = nullptr;
static NimBLEAddress      bleTarget;
static volatile bool      bleDoConnect = false;
static uint32_t           bleNextScanAt = 0;

// Letzte Datenquelle der Cockpit-Werte (fuer Anzeige/Diagnose)
static const char* g_lastSrc = "---";

// -------- CAN cockpit client (TWAI, hoert 0x510 vom Spartan-Test-Hub) --------
#define COCKPIT_CAN_RX_PIN GPIO_NUM_44
#define COCKPIT_CAN_TX_PIN GPIO_NUM_43

static bool     g_canReady      = false;
static bool     g_canListenOnly = true;   // Display hoert nur mit (kein ACK/TX)
static uint16_t g_canId         = 0x510;  // Cockpit-Frame-ID (Dev-Tab, NVS can_id)
static uint16_t g_canKbps       = 500;    // Bitrate kbit/s: 125/250/500/1000 (NVS can_kbps)
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
// true solange ein Finger auf dem Touch ist -> blockierenden HTTP-Poll aussetzen,
// damit kein Tap mit einem GET-Timeout kollidiert (Touch bleibt reaktiv).
static volatile bool g_uiTouchActive = false;
// true waehrend eines WiFi-Scans -> BLE-Ticks (Hub/123) aussetzen: BLE und WiFi
// teilen sich das 2.4GHz-Radio, ein laufender BLE-Scan laesst den WiFi-Scan haengen.
static volatile bool g_wlanScanBusy = false;

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
static bool      g_featureCan    = false;  // CAN-Cockpit-RX: default AUS (kein Transceiver/Bus verbunden)
static bool      g_autoCockpit    = true;  // Drehzahl > Schwelle -> automatisch Motor-Seite (von Uhr/Menue)
static int       g_autoCockpitRpm = 600;   // Schwelle, per WebGUI variabel einstellbar
static bool      g_testHub        = false; // Dev: Daten vom Test-Hub (feste IP) statt Profil-Hub; Quelle heisst dann "TEST"
static char      g_testHubIp[40]  = "192.168.0.87";  // Test-Hub im Heimnetz, per Dev-Tab einstellbar
// ---- Alarme (Grenzwerte per Dev-Tab, NVS-persistent). Bei Verletzung blinkt auf
// ---- JEDER Seite ein roter Ring (Overlay in presentFrame) + optional Buzzer-Piep.
static bool      g_alertsOn  = true;       // Alarme global an/aus
static float     g_alLamMin  = 0.90f;      // Lambda-Soll-Band (auch gruenes Band im Trend)
static float     g_alLamMax  = 1.10f;
static int       g_alRpmMax  = 4500;       // Drehzahl-Alarm (T2b Typ4)
static int       g_alTempMax = 90;         // 123TUNE-Temp-Alarm (Grad C)
static float     g_alVoltMin = 11.5f;      // Unterspannung bei laufendem Motor
static uint8_t   g_alertMask = 0;          // Bits: 1=Lambda 2=Drehzahl 4=Temp 8=Volt
static char      g_alertText[32] = "";     // muss "DREHZAHL LAMBDA TEMP VOLT" (26) fassen
// ---- Tacho-Konfiguration (Dev-Tab): roter Bereich + Skalenende
static int       g_rpmRedline = 6000;      // Tacho: rot ab hier
static int       g_rpmScaleMax = 8000;     // Tacho: Skalenende
#define FW_BUILD __DATE__ " " __TIME__     // Firmware-Stand (Compile-Zeit) fuer die WebGUI
#ifndef GIT_REV
  #define GIT_REV "unknown"                // wird von scripts/inject_time.py injiziert
#endif
#ifndef GIT_REV_URL
  #define GIT_REV_URL GIT_REV              // Hash ohne "+"-Dirty-Marker (fuer den Link)
#endif
#define GITHUB_URL "https://github.com/niedi74/waveshare-vdo-clock/commit/" GIT_REV_URL
static float     g_imuOffPitch   = 0.0f;   // IMU-Nullung (Einbaulage) - Pitch/Roll-Offset
static float     g_imuOffRoll    = 0.0f;
static bool      g_wifiAuto      = true;   // WLAN-Auto-Fallback: S24 > Heim > Hub-AP (verfuegbares Netz)
static bool      g_apOn          = false;  // Setup-AP aktiv?
static bool      g_webStarted   = false;
static bool      g_redrawPage   = false;
static uint8_t   g_wifiProfile  = 0;
static WebServer webServer(80);
static void startWebServer();   // forward declaration
static bool        g_sdMounted = false;   // Micro-SD: Mount-Status (Test)
static uint32_t    g_sdSizeMB  = 0;
static const char* g_sdType    = "-";
static int         g_sdWifiLoaded = -2;   // -2=keine SD/Datei, -1=Vorlage angelegt, >=0=Profile geladen
static const char* g_sdOtaResult   = "-"; // SD-OTA-Recovery: "-", "OK", "FEHLER"
static void sdLog(const char* msg);       // fwd - Ereignis-Log auf SD (siehe Definition weiter unten)
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

// ---- WLAN-Diagnose: letzter STA-Disconnect-Grund (Klartext) + RSSI ----
// Portiert aus deep-search b25c270 (nur Diagnose-Teil). Hilft beim Z00-Verbinden:
// zeigt WARUM es nicht geht (falsches PW / Netz nicht da / keine IP) statt nur "weg".
static volatile uint8_t g_wifiStaReason = 0;
static void onWifiStaEvent(arduino_event_id_t ev, arduino_event_info_t info) {
  if      (ev == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) g_wifiStaReason = info.wifi_sta_disconnected.reason;
  else if (ev == ARDUINO_EVENT_WIFI_STA_GOT_IP)       g_wifiStaReason = 0;
}
static const char* wifiReasonText(uint8_t r) {
  switch (r) {
    case 0:   return "-";
    case 201: return "Netz n.gef.";   // NO_AP_FOUND: SSID nicht in Reichweite/falsch
    case 202: return "falsches PW";   // AUTH_FAIL
    case 15:  return "falsches PW";   // 4WAY_HANDSHAKE_TIMEOUT
    case 2:   return "Auth weg";
    case 8:   return "keine IP";
    case 200: return "Beacon weg";
    case 205: return "Connect-Fail";
    default:  return "Abbruch";
  }
}

// Non-blocking WiFi/NTP handler. Returns true on fresh NTP sync.
static bool wifiNtpTick() {
  static bool staEvtReg = false;
  if (!staEvtReg) { WiFi.onEvent(onWifiStaEvent); staEvtReg = true; }
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
    char logMsg[64];
    snprintf(logMsg, sizeof(logMsg), "WIFI %s ip=%s", WPROF_LABELS[g_wifiProfile], g_ipStr);
    sdLog(logMsg);
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

// ---- WLAN per WPS (PBC = Tastendruck am Router), wie im M5 Dial ----
static void saveWprof(uint8_t, const char*, const char*, const char*);   // fwd
static uint32_t g_manualWifiUntil = 0;   // selectWprof: Schonfrist gegen Auto-Fallback
enum WpsState : uint8_t { WPS_IDLE = 0, WPS_RUN = 1, WPS_OK = 2, WPS_FAIL = 3 };
static volatile WpsState g_wpsState       = WPS_IDLE;
static volatile bool     g_wpsActive      = false;  // WPS laeuft -> normalen Reconnect aussetzen
static volatile bool     g_wpsSavePending = false;  // nach Erfolg Zugangsdaten ins Heim-Profil sichern

// WLAN-Auto-Fallback: probiert (ohne Scan!) Hub-AP > Heim durch, je ~6 s. KEIN
// WiFi.scanNetworks() - das crasht auf diesem Arduino-Stand, wenn kein Netz in
// Reichweite ist (esp_wifi_scan_start StoreProhibited -> Boot-Loop). WiFi.begin
// im AP_STA ist ok. Hub-AP zuerst = im Auto immer die primaere Datenquelle; S24
// (Handy-Hotspot) ist NICHT in der Auto-Kette (nur manuell), damit das Display
// bei einem kurzen Hub-AP-Abriss nicht am Hotspot ohne Hub-Daten haengenbleibt.
static void wifiAutoTick() {
  // Waehrend WPS laeuft: keinen normalen Reconnect (sonst ueberschreibt WiFi.begin
  // die per WPS empfangene Verbindung). Bei Erfolg Router-Zugangsdaten ins Heim-Profil.
  if (g_wpsActive) {
    if (g_wpsSavePending && WiFi.status() == WL_CONNECTED) {
      String s = WiFi.SSID(), pw = WiFi.psk();
      if (s.length() > 0) {
        saveWprof(0, s.c_str(), pw.c_str(), g_wprof[0].hubip);   // Slot 0 = Heim
        g_wifiProfile = 0;
        g_wpsState = WPS_OK;
        Serial.printf("WPS: ok -> Heim '%s' gespeichert\n", s.c_str());
      }
      g_wpsSavePending = false;
      g_wpsActive = false;
    }
    return;
  }
  if (!g_featureWifi || !g_wifiAuto) return;
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() < g_manualWifiUntil) return;        // manuell gewaehltem Profil Zeit lassen -
                                                   // sonst schiesst Auto jede S24-Wahl sofort ab
  // Nur Hub-AP > Heim automatisch. S24 NICHT auto (sonst landet das Display bei
  // einem Hub-AP-Abriss am Handy-Hotspot, wo es keine Hub-Daten gibt). S24 bleibt
  // manuell waehlbar (kurzer Tap auf WIFI). -> im Auto haelt es immer den Hub-AP.
  static const uint8_t AUTO_N = 2;
  static const uint8_t order[AUTO_N] = { 1, 0 };   // Hub-AP > Heim (Hub ist im Auto die primaere Verbindung)
  static uint8_t  oi    = 0;
  static uint32_t tryAt = 0;
  if (millis() < tryAt) return;
  for (uint8_t k = 0; k < AUTO_N; k++) {
    uint8_t slot = order[(oi + k) % AUTO_N];
    if (!g_wprof[slot].ssid[0]) continue;          // leeres Profil ueberspringen
    oi = (uint8_t)((oi + k + 1) % AUTO_N);
    g_wifiProfile = slot;
    if (g_wprof[slot].hubip[0]) g_hubIp = g_wprof[slot].hubip;
    strcpy(g_ipStr, "...");
    WiFi.begin(g_wprof[slot].ssid, g_wprof[slot].pass);
    tryAt = millis() + 9000;                       // diesem Profil 9 s geben (schwaches Signal:
                                                   // Assoc+DHCP brauchen >6 s -> "keine IP"-Abbrueche)
    Serial.printf("WiFi-Auto: versuche %s (%s)\n", WPROF_LABELS[slot], g_wprof[slot].ssid);
    return;
  }
  tryAt = millis() + 5000;                         // kein belegtes Profil
}

static void onWifiWpsEvent(arduino_event_id_t ev, arduino_event_info_t) {
  if (ev == ARDUINO_EVENT_WPS_ER_SUCCESS) {
    esp_wifi_wps_disable();
    g_wpsSavePending = true;
    g_wpsState = WPS_RUN;
    WiFi.begin();                                  // mit den per WPS empfangenen Daten verbinden
    Serial.println("WPS: Erfolg, verbinde...");
  } else if (ev == ARDUINO_EVENT_WPS_ER_FAILED || ev == ARDUINO_EVENT_WPS_ER_TIMEOUT) {
    esp_wifi_wps_disable();
    g_wpsActive = false;
    g_wpsSavePending = false;
    g_wpsState = WPS_FAIL;
    Serial.println("WPS: fehlgeschlagen/Timeout");
  }
}

// WPS (Push-Button) starten: Router-WPS-Taste + diesen Button = verbinden ohne Tippen.
static void startWps() {
  static bool evtRegistered = false;
  if (!evtRegistered) { WiFi.onEvent(onWifiWpsEvent); evtRegistered = true; }
  g_featureWifi = true;
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, true);
  esp_wps_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.wps_type = WPS_TYPE_PBC;
  strcpy(cfg.factory_info.manufacturer, "ESPRESSIF");
  strcpy(cfg.factory_info.model_number, "ESP32S3");
  strcpy(cfg.factory_info.model_name,   "VDO-CLOCK");
  strcpy(cfg.factory_info.device_name,  "VDO-CLOCK");
  if (esp_wifi_wps_enable(&cfg) == ESP_OK && esp_wifi_wps_start(0) == ESP_OK) {
    g_wpsActive = true; g_wpsSavePending = false; g_wpsState = WPS_RUN;
    Serial.println("WPS: gestartet (PBC) - jetzt WPS-Taste am Router druecken");
  } else {
    g_wpsActive = false; g_wpsState = WPS_FAIL;
    Serial.println("WPS: Start fehlgeschlagen");
  }
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
    bleClient->setConnectTimeout(3000);   // Default waere bis 30 s - blockiert Loop+Touch
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
  if (g_wlanScanBusy) return;                // waehrend WiFi-Scan Radio nicht fuer BLE nutzen
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
  if      (g_canKbps == 125)  { twai_timing_config_t x = TWAI_TIMING_CONFIG_125KBITS(); t = x; }
  else if (g_canKbps == 250)  { twai_timing_config_t x = TWAI_TIMING_CONFIG_250KBITS(); t = x; }
  else if (g_canKbps == 1000) { twai_timing_config_t x = TWAI_TIMING_CONFIG_1MBITS();   t = x; }
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  esp_err_t err = twai_driver_install(&g, &t, &f);
  if (err != ESP_OK) { Serial.printf("CAN: install failed %s\n", esp_err_to_name(err)); return false; }
  err = twai_start();
  if (err != ESP_OK) { Serial.printf("CAN: start failed %s\n", esp_err_to_name(err)); twai_driver_uninstall(); return false; }
  g_canReady = true;
  Serial.printf("CAN: cockpit RX ready id=0x%03X TX=%d RX=%d mode=%s %uk\n",
                g_canId, (int)COCKPIT_CAN_TX_PIN, (int)COCKPIT_CAN_RX_PIN,
                g_canListenOnly ? "listen" : "normal", g_canKbps);
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
    if (msg.extd || msg.rtr || msg.identifier != g_canId || msg.data_length_code != 8) { g_canIgnored++; continue; }
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

// IMU-Werte periodisch aufs CAN senden, damit der Hub sie mitloggen kann. Nur im
// NORMAL-Modus moeglich: TWAI_MODE_LISTEN_ONLY verbietet jede eigene Uebertragung
// (ESP-IDF-Doku). ID = Cockpit-ID+1 - wandert automatisch mit, falls g_canId je
// umgestellt wird. Fire-and-forget (Timeout 0): blockiert den Loop nie.
static void imuCanTxTick() {
  static uint32_t at = 0;
  if (!g_featureCan || !g_canReady || g_canListenOnly || !g_imuPresent) return;
  if (millis() < at) return;
  at = millis() + 200;                      // 5 Hz
  qmi8658Read();
  int16_t  pitchX10 = (int16_t)lroundf((g_imuPitch - g_imuOffPitch) * 10.0f);
  int16_t  rollX10  = (int16_t)lroundf((g_imuRoll  - g_imuOffRoll)  * 10.0f);
  uint16_t gX100    = (uint16_t)constrain((int)lroundf(g_imuGForce * 100.0f), 0, 65535);
  twai_message_t m = {};
  m.identifier = (g_canId < 0x7FF) ? (uint32_t)(g_canId + 1) : g_canId;
  m.data_length_code = 6;
  m.data[0] = (uint8_t)(pitchX10 >> 8); m.data[1] = (uint8_t)pitchX10;
  m.data[2] = (uint8_t)(rollX10  >> 8); m.data[3] = (uint8_t)rollX10;
  m.data[4] = (uint8_t)(gX100    >> 8); m.data[5] = (uint8_t)gX100;
  twai_transmit(&m, 0);
}

// -------- HTTP-Poll-Client --------
static bool httpFresh() { return g_httpLastRxMs != 0 && millis() - g_httpLastRxMs < 3000; }

// Minimaler JSON-Feldparser auf char*-Basis: sucht "key": und liest die folgende
// Zahl. Bewusst OHNE String/substring -> keine Heap-Allokation im 2-Hz-Poll (frueher
// ~30 kleine malloc/free pro Poll = Fragmentierung ueber Stunden). Semantik identisch:
// Muster "key": , dann strtof (ueberspringt Whitespace, stoppt am Komma/'}').
static bool jsonNum(const char* s, const char* key, float& out) {
  char pat[24];
  int n = snprintf(pat, sizeof(pat), "\"%s\":", key);
  if (n <= 0 || n >= (int)sizeof(pat)) return false;
  const char* p = strstr(s, pat);
  if (!p) return false;
  out = strtof(p + n, nullptr);
  return true;
}
static bool jsonTrue(const char* s, const char* key) {
  char pat[28];
  int n = snprintf(pat, sizeof(pat), "\"%s\":true", key);
  if (n <= 0 || n >= (int)sizeof(pat)) return false;
  return strstr(s, pat) != nullptr;
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
  // Dev/Test-Modus: feste Test-Hub-IP uebersteuert ALLES (auch das Hub-AP-Gateway).
  // Bewusst explizit: Quelle heisst dann "TEST" und die WebGUI zeigt ein Banner.
  if (g_testHub && g_testHubIp[0]) return String(g_testHubIp);
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
  IPAddress ip = MDNS.queryHost(name.c_str(), 400);   // kurz halten -> blockiert den Loop/Touch nicht 1,5 s
  if (ip != IPAddress(0, 0, 0, 0)) {
    g_hubResolvedIp = ip.toString();
    Serial.printf("mDNS: %s -> %s\n", g_hubIp.c_str(), g_hubResolvedIp.c_str());
  }
  return g_hubResolvedIp;
}

static void httpPollTick() {
  if (!g_featureWifi || WiFi.status() != WL_CONNECTED || g_hubIp.length() == 0) return;
  // Solange ein Finger auf dem Touch ist NICHT pollen: der GET blockiert den Loop
  // (bes. bei schwachem WLAN) und wuerde genau den Tap verschlucken. Daten pausieren
  // nur fuer die Dauer der Beruehrung -> unmerklich, Touch bleibt reaktiv.
  if (g_uiTouchActive) return;
  static uint32_t last = 0;
  static uint8_t  failStreak = 0;
  // Backoff: bei totem Hub nur alle 4s pollen, sonst blockiert der GET den Loop
  // dauernd (Timeout) -> Touch wird traege. Bei Erfolg wieder flott (500ms).
  const uint32_t interval = (failStreak >= 3) ? 4000 : 500;
  if (millis() - last < interval) return;
  last = millis();

  String tgt = hubTarget();
  if (tgt.length() == 0) { if (failStreak < 200) failStreak++; return; }  // Hostname (noch) nicht aufloesbar
  // Frische Verbindung pro Poll (KEIN Keep-Alive): der Hub-WebServer schliesst die
  // Verbindung nach jedem Request -> ein wiederverwendeter Socket liefe ins Leere und
  // erzeugte periodische Fehl-Polls (failStreak-Backoff -> 4-5 s alte Cockpit-Daten).
  // Fresh-Connect ist robust; der Touch wird nicht ueber kurze Timeouts, sondern ueber
  // die Poll-Pause bei Fingerkontakt (g_uiTouchActive) geschuetzt.
  HTTPClient http;
  http.setReuse(false);
  String url = "http://" + tgt + "/api/status";
  if (!http.begin(url)) { if (failStreak < 200) failStreak++; return; }
  http.addHeader("X-Device", "vdo-clock " GIT_REV);   // Hub-Diagnose zeigt uns namentlich
  // Timeouts adaptiv: bei schwachem Signal (< -70 dBm) braucht der TCP-Aufbau
  // laenger als 200ms -> alle Polls scheiterten (httpRx=0 trotz Verbindung).
  // Bei gutem Signal bleiben die kurzen Timeouts (Touch/Loop reaktiv; Taps
  // schuetzt ohnehin die Poll-Pause bei Fingerkontakt).
  const int rssi = WiFi.RSSI();
  if (rssi < -70) { http.setConnectTimeout(600); http.setTimeout(800); }
  else            { http.setConnectTimeout(200); http.setTimeout(350); }
  int code = http.GET();
  if (code == 200) {
    String b = http.getString();
    const char* bc = b.c_str();
    float v;
    if (jsonNum(bc, "rpm", v))       g_rpm = v;
    if (jsonNum(bc, "advance", v))   g_adv = v;
    if (jsonNum(bc, "map", v))       g_map = v;
    if (jsonNum(bc, "lambda", v))  { g_lambda = v; g_lambdaValid = jsonTrue(bc, "valid") && v > 0; }
    if (jsonNum(bc, "tune_temp", v)) g_g123Temp = v;
    if (jsonNum(bc, "volt", v))      g_g123Volt = v;
    if (jsonNum(bc, "tune_amp", v))  g_g123Coil = v;
    if (jsonNum(bc, "speed_kmh", v)){ g_speedKmh = v; g_speedValid = true; }
    if (jsonNum(bc, "trip_km", v)) { g_tripKm = v; g_tripValid = true; }
    g_g123Valid   = jsonTrue(bc, "tune_connected");
    // Quelle kennzeichnen: Dev-Schalter ODER Test-Hub-AP erkannt am Subnetz.
    // Beschluss 3.7.: Live-Hub-AP = 192.168.4.x, Test-Hub-AP = 192.168.5.x.
    g_lastSrc     = (g_testHub || tgt.startsWith("192.168.5.")) ? "TEST" : "HTTP";
    g_httpRx++;
    if (g_httpRx == 1) Serial.printf("HTTP: erste Daten von %s\n", tgt.c_str());
    g_httpLastRxMs = millis();
    failStreak = 0;
  } else {
    if (failStreak < 200) failStreak++;
  }
  http.end();
}

// Teilstrecke am Hub nullen (gleicher Endpunkt wie das ↺ in der Hub-WebGUI).
// Blockiert kurz (<1 s) - wird nur durch bewussten langen Druck ausgeloest.
static bool tripResetOnHub() {
  String tgt = hubTarget();
  if (!tgt.length()) return false;
  HTTPClient http;
  http.setReuse(false);
  http.setConnectTimeout(600);
  http.setTimeout(800);
  if (!http.begin("http://" + tgt + "/odo")) return false;
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  http.addHeader("X-Device", "vdo-clock " GIT_REV);
  int code = http.POST("trip=reset");
  http.end();
  const bool ok = (code >= 200 && code < 400);   // Hub antwortet 303 (Redirect nach POST)
  Serial.printf("TRIP-Reset -> %s: HTTP %d\n", tgt.c_str(), code);
  sdLog(ok ? "TRIP RESET" : "TRIP RESET FEHLER");
  if (ok) {
    g_tripKm = 0.0f;                      // sofort anzeigen, naechster Poll bestaetigt
    if (g_featureBuzzer) { hal_buzzer(true); delay(60); hal_buzzer(false); }
  }
  return ok;
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
    tune123Client->setConnectTimeout(3000);   // Default waere bis 30 s - blockiert Loop+Touch
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
  if (g_wlanScanBusy) return;                    // waehrend WiFi-Scan kein BLE-Scan (Radio-Konflikt)
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
static void drawCircleLine(int cx, int cy, int radius, int thickness, uint16_t color);  // fwd
static void drawTextCentered(int cx, int y, const char *text, uint16_t color, int scale);  // fwd
// Zentraler Present: bei aktivem Alarm blinkt ein roter Ring + Grund als Overlay
// auf JEDER Seite (alle Seiten zeichnen voll neu -> Overlay verschwindet sauber).
static void presentFrame() {
  if (g_alertMask && (millis() / 500) % 2 == 0) {
    drawCircleLine(240, 240, 236, 10, RGB565(220, 44, 32));
    if (g_alertText[0]) drawTextCentered(240, 20, g_alertText, RGB565(235, 60, 45), 2);
  }
  hal_present();
}
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

// Echte Kleinbuchstaben (klassischer 5x7-Font, bit0 = oberste Zeile) - noetig,
// damit die Tastatur-Ebenen klein/GROSS unterscheidbar sind (SSIDs case-sensitiv!).
static const uint8_t GLYPH_LOWER[26][5] = {
  {0x20,0x54,0x54,0x54,0x78},{0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20}, // a b c
  {0x38,0x44,0x44,0x48,0x7F},{0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02}, // d e f
  {0x0C,0x52,0x52,0x52,0x3E},{0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00}, // g h i
  {0x20,0x40,0x44,0x3D,0x00},{0x7F,0x10,0x28,0x44,0x00},{0x00,0x41,0x7F,0x40,0x00}, // j k l
  {0x7C,0x04,0x18,0x04,0x78},{0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38}, // m n o
  {0x7C,0x14,0x14,0x14,0x08},{0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08}, // p q r
  {0x48,0x54,0x54,0x54,0x20},{0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C}, // s t u
  {0x1C,0x20,0x40,0x20,0x1C},{0x3C,0x40,0x30,0x40,0x3C},{0x44,0x28,0x10,0x28,0x44}, // v w x
  {0x0C,0x50,0x50,0x50,0x3C},{0x44,0x64,0x54,0x4C,0x44},                            // y z
};

static uint8_t glyphColumn(char c, uint8_t col) {
  static const uint8_t blank[5] = {0,0,0,0,0};
  const uint8_t *g = blank;
  if (c >= 'a' && c <= 'z') return GLYPH_LOWER[c - 'a'][col];
  switch (c) {
    case 'A': { static const uint8_t v[5]={0x7E,0x11,0x11,0x11,0x7E}; g=v; break; }
    case 'B': { static const uint8_t v[5]={0x7F,0x49,0x49,0x49,0x36}; g=v; break; }
    case 'C': { static const uint8_t v[5]={0x3E,0x41,0x41,0x41,0x22}; g=v; break; }
    case 'D': { static const uint8_t v[5]={0x7F,0x41,0x41,0x22,0x1C}; g=v; break; }
    case 'E': { static const uint8_t v[5]={0x7F,0x49,0x49,0x49,0x41}; g=v; break; }
    case 'F': { static const uint8_t v[5]={0x7F,0x09,0x09,0x09,0x01}; g=v; break; }
    case 'G': { static const uint8_t v[5]={0x3E,0x41,0x49,0x49,0x3A}; g=v; break; }
    case 'J': { static const uint8_t v[5]={0x20,0x40,0x41,0x3F,0x01}; g=v; break; }
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
    case '(': { static const uint8_t v[5]={0x00,0x1C,0x22,0x41,0x00}; g=v; break; }
    case ')': { static const uint8_t v[5]={0x00,0x41,0x22,0x1C,0x00}; g=v; break; }
    case ',': { static const uint8_t v[5]={0x00,0x50,0x30,0x00,0x00}; g=v; break; }
    case ';': { static const uint8_t v[5]={0x00,0x56,0x36,0x00,0x00}; g=v; break; }
    case '+': { static const uint8_t v[5]={0x08,0x08,0x3E,0x08,0x08}; g=v; break; }
    case '!': { static const uint8_t v[5]={0x00,0x00,0x5F,0x00,0x00}; g=v; break; }
    case '?': { static const uint8_t v[5]={0x02,0x01,0x51,0x09,0x06}; g=v; break; }
    case '=': { static const uint8_t v[5]={0x14,0x14,0x14,0x14,0x14}; g=v; break; }
    case '<': { static const uint8_t v[5]={0x08,0x14,0x22,0x41,0x00}; g=v; break; }
    case '>': { static const uint8_t v[5]={0x00,0x41,0x22,0x14,0x08}; g=v; break; }
    case '*': { static const uint8_t v[5]={0x08,0x2A,0x1C,0x2A,0x08}; g=v; break; }
    case '#': { static const uint8_t v[5]={0x14,0x7F,0x14,0x7F,0x14}; g=v; break; }
    case '&': { static const uint8_t v[5]={0x36,0x49,0x55,0x22,0x50}; g=v; break; }
    case '@': { static const uint8_t v[5]={0x32,0x49,0x79,0x41,0x3E}; g=v; break; }
    case '_': { static const uint8_t v[5]={0x40,0x40,0x40,0x40,0x40}; g=v; break; }
    case 0x27: { static const uint8_t v[5]={0x00,0x05,0x03,0x00,0x00}; g=v; break; }  // '
    case '"': { static const uint8_t v[5]={0x00,0x07,0x00,0x07,0x00}; g=v; break; }
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
    // WICHTIG: bei pct>100 ist outSize>480 und offset NEGATIV -> ohne Clipping
    // schreibt fb[dstRow+ox] VOR und HINTER den Framebuffer (Heap-Korruption!
    // 0x1082-Pixel im lwIP-Heap -> StoreProhibited-Bootloop bei GROESSE=115/rot=0).
    // Ziel-Fenster auf 0..479 begrenzen, Quelle skaliert sauber mit.
    int o0 = (offset < 0) ? -offset : 0;                       // erster sichtbarer out-Index
    int o1 = (offset + outSize > 480) ? 480 - offset : outSize; // hinter letztem sichtbaren
    for (int oy = o0; oy < o1; oy++) {
      int sy     = (oy * 480) / outSize;
      int dstRow = (offset + oy) * 480 + offset;
      int srcRow = sy * 480;
      for (int ox = o0; ox < o1; ox++) {
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

// ===== Alarme: Grenzwerte pruefen (alle 500ms), Overlay via presentFrame =====
static void alertTick() {
  static uint32_t at = 0;
  if (millis() < at) return;
  at = millis() + 500;
  uint8_t m = 0;
  const bool fresh = bleFresh() || canFresh() || httpFresh() || tune123Fresh();
  if (g_alertsOn && fresh) {
    const bool running = g_rpm >= 250.0f;          // Motor dreht (kein Alarm im Stand/aus; Test-Hub sendet 300)
    if (running && g_lambdaValid && (g_lambda < g_alLamMin || g_lambda > g_alLamMax)) m |= 1;
    if (g_rpm > (float)g_alRpmMax)                                                    m |= 2;
    if (g_g123Valid && g_g123Temp > (float)g_alTempMax)                               m |= 4;
    if (running && g_g123Valid && g_g123Volt > 1.0f && g_g123Volt < g_alVoltMin)      m |= 8;
  }
  if (m != g_alertMask) {
    g_alertMask = m;
    g_alertText[0] = 0;
    if (m & 2) strlcat(g_alertText, "DREHZAHL ", sizeof(g_alertText));
    if (m & 1) strlcat(g_alertText, "LAMBDA ",   sizeof(g_alertText));
    if (m & 4) strlcat(g_alertText, "TEMP ",     sizeof(g_alertText));
    if (m & 8) strlcat(g_alertText, "VOLT ",     sizeof(g_alertText));
    size_t n = strlen(g_alertText);                 // trailing space weg
    if (n && g_alertText[n-1] == ' ') g_alertText[n-1] = 0;
    Serial.printf("ALARM: %s\n", m ? g_alertText : "aus");
    char logMsg[48];
    snprintf(logMsg, sizeof(logMsg), "ALARM %s", m ? g_alertText : "aus");
    sdLog(logMsg);
    g_redrawPage = true;                            // Ring sofort an/aus zeichnen
  }
}

// Buzzer-Piep bei Alarm: 120ms an, alle 2s, nicht blockierend. Nur wenn Buzzer-Feature an.
static void alertBuzzTick() {
  static bool on = false; static uint32_t nextAt = 0, offAt = 0;
  if (!g_alertMask || !g_featureBuzzer) { if (on) { hal_buzzer(false); on = false; } return; }
  const uint32_t now = millis();
  if (!on && now >= nextAt) { hal_buzzer(true); on = true; offAt = now + 120; nextAt = now + 2000; }
  else if (on && now >= offAt) { hal_buzzer(false); on = false; }
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
  // ZWEI Original-Referenzen im T2b (Cockpit-Fotos 3.7.): der TACHO daneben ist
  // ausgeblichen-GRAU mit weissen Zahlen, die Quartz-Zeit-UHR fast schwarz mit Creme.
  // Kombi-Flaeche = Tacho-Grau (84,86,90); die dunkle Uhr-Scheibe zeichnet der
  // VDO+UHR-Stil separat (RGB 16,16,16, von der Uhr-Seite gesampelt).
  RGB565(84,86,90), RGB565(214,218,222), RGB565(110,114,120), true, false, true, true,
  RGB565(228,228,222), RGB565(150,148,138), RGB565(228,228,222),
  RGB565(232,230,214), RGB565(208,168,40), RGB565(40,30,16),
  false, RGB565(228,228,222), RGB565(150,148,138),
  RGB565(228,228,222), RGB565(220,44,32), false, RGB565(30,32,36) };

static const GaugeTheme THEME_123 = {
  RGB565(12,12,14), RGB565(206,210,214), RGB565(90,94,100), true, false, false, false,
  RGB565(238,238,236), RGB565(122,122,126), RGB565(238,238,236),
  RGB565(224,122,36), RGB565(224,122,36), RGB565(70,32,6),
  false, RGB565(238,238,236), RGB565(122,122,126),
  RGB565(238,238,236), RGB565(220,44,32), false, RGB565(22,22,26) };

static uint8_t g_motorStyle = 2;   // 0=digital, 1=vdo, 2=123tune+
static const char* const MOTOR_STYLE_NAMES[] = { "DIGITAL", "VDO", "123TUNE+", "VDO+UHR" };
#define MOTOR_STYLE_COUNT 4
static uint32_t g_tripArmUntil = 0;   // Trip-Reset: 1. Langdruck scharf (5 s), 2. Langdruck nullt
static const GaugeTheme& gTheme() {
  return g_motorStyle == 0 ? THEME_DIGITAL : g_motorStyle == 2 ? THEME_123 : THEME_VDO;  // 1+3 = VDO-Look
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
static void drawTach(int cx, int cy, float rpm, bool valid, const GaugeTheme& t, float sc = 1.0f) {
  const float A0 = 150.0f, A1 = 390.0f;
  const float RMAX = (float)g_rpmScaleMax;          // Skalenende (Dev-Tab)
  const int   RED  = g_rpmRedline;                  // roter Bereich ab (Dev-Tab)
  const int   KMAX = g_rpmScaleMax / 1000;
  if (t.bandTach) {                                 // digital: Streifenband
    const int rIn = (int)lroundf(152 * sc), rOut = (int)lroundf(212 * sc);
    drawArcBand(cx, cy, rIn, rOut, A0, A1, RGB565(45, 45, 50));
    drawArcBand(cx, cy, rIn, rOut, gaugeAngle((float)RED, 0, RMAX, A0, A1), A1, RGB565(150, 30, 28));
    for (int k = 0; k <= KMAX; k++) {
      float a  = gaugeAngle(k * 1000, 0, RMAX, A0, A1);
      uint16_t tc = (k * 1000 >= RED) ? RGB565(235, 80, 70) : RGB565(190, 190, 190);
      plotRadial(cx, cy, a, rIn, rOut, tc, (k % 2) ? 2 : 3);
      float ar = a * (float)PI / 180.0f;
      int lx = cx + (int)(cosf(ar) * (rIn - (int)lroundf(18 * sc)));
      int ly = cy + (int)(sinf(ar) * (rIn - (int)lroundf(18 * sc)));
      char n[3]; snprintf(n, sizeof(n), "%d", k);
      drawTextCentered(lx, ly - 7, n, tc, 2);
    }
    float pa = gaugeAngle(valid ? rpm : 0, 0, RMAX, A0, A1);
    plotRadial(cx, cy, pa, rIn - 6, rOut + 2, t.needle, 6);
    plotRadial(cx, cy, pa, rOut - (int)lroundf(16 * sc), rOut + 2, TACH_RED, 6);
    return;
  }
  const int rOut = (int)lroundf(208 * sc);          // diskrete Striche
  for (int v = 0; v <= g_rpmScaleMax; v += 200) {
    float a = gaugeAngle(v, 0, RMAX, A0, A1);
    bool major = (v % 1000) == 0;
    uint16_t tc = (t.redlineMark && v >= RED) ? TACH_RED : (major ? t.tickMaj : t.tickMin);
    plotRadial(cx, cy, a, rOut - (int)lroundf((major ? 24 : 12) * sc), rOut, tc, major ? 3 : 2);
  }
  for (int k = 0; k <= KMAX; k++) {
    float a  = gaugeAngle(k * 1000, 0, RMAX, A0, A1);
    float ar = a * (float)PI / 180.0f;
    int lx = cx + (int)(cosf(ar) * (rOut - (int)lroundf(42 * sc)));
    int ly = cy + (int)(sinf(ar) * (rOut - (int)lroundf(42 * sc)));
    char n[3]; snprintf(n, sizeof(n), "%d", k);
    // Zahlen bleiben creme (Original-VDO: roter Bereich nur als Striche markiert)
    drawTextCentered(lx, ly - 10, n, t.numCol, 3);
  }
  float pa = gaugeAngle(valid ? rpm : 0, 0, RMAX, A0, A1);
  plotRadial(cx, cy, pa, (int)lroundf(150 * sc), rOut + 2, t.needle, 6);
  if (t.redTip) plotRadial(cx, cy, pa, rOut - (int)lroundf(16 * sc), rOut + 2, TACH_RED, 6);
}

// Skalierung der MOTOR-Kombi mit GROESSE (g_dialScalePct), um die Mitte 240,240 - wie die Uhr.
#define M_S    (g_dialScalePct / 100.0f)
#define M_X(x) (240 + (int)lroundf(((x) - 240) * M_S))
#define M_Y(y) (240 + (int)lroundf(((y) - 240) * M_S))
#define M_R(r) ((int)lroundf((r) * M_S))
#define M_F(f) (max(1, min(8, (int)lroundf((f) * M_S))))

static void drawMotorPage() {
  if (!ensureFrame()) return;
  const GaugeTheme& t = gTheme();
  fillFrame(t.face);
  if (t.chrome) {
    drawCircleLine(240, 240, M_R(236), 7, t.bezel);        // heller Chrom-Ring
    drawCircleLine(240, 240, M_R(229), 2, t.bezelDk);       // dunkle Innenkante
  }
  const bool fresh = bleFresh() || canFresh() || httpFresh() || tune123Fresh();
  char buf[16];

  drawTach(240, 240, g_rpm, fresh, t, M_S);

  if (g_motorStyle == 3) {
    // ===== VDO+UHR: Tacho aussen (Tacho-Grau wie der Speedo daneben), in der
    // ===== Mitte die fast schwarze Uhr-Scheibe (wie die echte Quartz-Zeit) =====
    if (fresh) snprintf(buf, sizeof(buf), "%d", (int)g_rpm); else strcpy(buf, "----");
    drawTextCentered(240, M_Y(104), buf, fresh ? t.txt : t.txtDim, M_F(3));
    // Dunkle Uhr-Scheibe (zweifarbig wie das echte Cockpit: grauer Tacho + schwarze Uhr)
    fillCircleFast(240, 240, M_R(106), RGB565(16, 16, 16));
    drawCircleLine(240, 240, M_R(106), 2, RGB565(40, 42, 44));   // dezente Kante
    // Uhr-Ticks: 12 Striche (3/6/9/12 kraeftiger), Creme wie die Original-Uhr
    for (int k = 0; k < 12; k++) {
      float a = k * 30.0f - 90.0f;
      bool major = (k % 3) == 0;
      plotRadial(240, 240, a, M_R(major ? 88 : 93), M_R(100),
                 major ? RGB565(232,232,224) : RGB565(170,168,156), major ? 3 : 2);
    }
    struct tm cn = {};
    if (readClockTime(&cn)) {
      // Zweilagige VDO-Zeiger wie auf der Uhr-Seite (dunkle Kontur + Creme-Fuellung)
      float mh = cn.tm_min + cn.tm_sec / 60.0f;
      float hh = (cn.tm_hour % 12) + mh / 60.0f;
      drawHand(hh, 12.0f, M_R(52), 10, RGB565(24, 24, 22));
      drawHand(hh, 12.0f, M_R(52), 6,  RGB565(232, 232, 224));
      drawHand(mh, 60.0f, M_R(80), 8,  RGB565(24, 24, 22));
      drawHand(mh, 60.0f, M_R(80), 4,  RGB565(232, 232, 224));
      drawHand((float)cn.tm_sec, 60.0f, M_R(86), 2, TACH_RED);
      fillCircleFast(240, 240, M_R(11), RGB565(205, 205, 198));   // Nabe wie Original:
      fillCircleFast(240, 240, M_R(7),  RGB565(208, 168, 40));    // silber/gold/dunkel
      fillCircleFast(240, 240, M_R(3),  RGB565(38, 30, 18));
    }
    // Lambda kompakt unter der Uhr + Temp/Volt-Zeile
    uint16_t lc2 = t.txtDim;
    if (fresh && g_lambdaValid) {
      snprintf(buf, sizeof(buf), "%.2f", g_lambda);
      lc2 = (g_lambda < g_alLamMin || g_lambda > g_alLamMax) ? TACH_RED : t.txt;
    } else strcpy(buf, "----");
    drawTextCentered(240, M_Y(352), buf, lc2, M_F(4));
    drawTextCentered(240, M_Y(336), "LAMBDA", t.txtDim, M_F(1));
    const bool g123c = fresh && g_g123Valid;
    char tv[32];
    if (millis() < g_tripArmUntil) {
      strcpy(tv, "TRIP NULLEN? UNTEN HALTEN");
      drawTextCentered(240, M_Y(396), tv, TACH_RED, M_F(2));
    } else {
      char tr[12] = "";
      if (g_tripValid) snprintf(tr, sizeof(tr), "  T%.1f", g_tripKm);
      if (g123c) snprintf(tv, sizeof(tv), "%dC  %.1fV%s", (int)g_g123Temp, g_g123Volt, tr);
      else       snprintf(tv, sizeof(tv), "--C  --V%s", tr);
      drawTextCentered(240, M_Y(396), tv, t.txtDim, M_F(2));
    }
    char st3[24];
    const char* src3 = fresh ? g_lastSrc : (g_bleConn ? "WARTE" : (g_canReady ? "CAN WARTE" : "KEIN HUB"));
    snprintf(st3, sizeof(st3), "%s%s", fresh ? "LIVE " : "", src3);
    drawTextCentered(240, M_Y(424), st3, fresh ? t.liveCol : t.statusBad, M_F(2));
    presentFrame();
    return;
  }

  // RPM digital (oben, innerhalb des Rings)
  if (fresh) snprintf(buf, sizeof(buf), "%d", (int)g_rpm); else strcpy(buf, "----");
  drawTextCentered(240, M_Y(92), buf, fresh ? t.txt : t.txtDim, M_F(5));
  drawTextCentered(240, M_Y(134), "RPM", t.txtDim, M_F(2));

  // Vier Mini-Gauges (Akzentfarben nur im Digital-Stil genutzt)
  const bool g123 = fresh && g_g123Valid;
  if (fresh) snprintf(buf, sizeof(buf), "%d", (int)g_adv); else strcpy(buf, "--");
  drawMiniGauge(M_X(148), M_Y(196), M_R(36), g_adv, 0, 50, "ADV", buf, RGB565(40, 150, 210), fresh, t);
  if (fresh) snprintf(buf, sizeof(buf), "%d", (int)g_map); else strcpy(buf, "--");
  drawMiniGauge(M_X(332), M_Y(196), M_R(36), g_map, 0, 200, "KPA", buf, RGB565(60, 185, 90), fresh, t);
  if (g123) snprintf(buf, sizeof(buf), "%d", (int)g_g123Temp); else strcpy(buf, "--");
  drawMiniGauge(M_X(148), M_Y(324), M_R(36), g_g123Temp, 0, 120, "TEMP", buf, RGB565(210, 120, 50), g123, t);
  if (g123) snprintf(buf, sizeof(buf), "%.1f", g_g123Volt); else strcpy(buf, "--");
  drawMiniGauge(M_X(332), M_Y(324), M_R(36), g_g123Volt, 10, 15, "VOLT", buf, RGB565(210, 180, 60), g123, t);

  // Lambda zentral
  drawTextCentered(240, M_Y(196), "LAMBDA", t.txtDim, M_F(2));
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
  drawTextCentered(240, M_Y(216), buf, lcol, M_F(6));

  // AMP (Zuendspulenstrom) + Tempo + Trip, dann Status, im unteren Ring-Gap
  char line[32];
  if (millis() < g_tripArmUntil) {
    strcpy(line, "TRIP NULLEN? UNTEN HALTEN");
    drawTextCentered(240, M_Y(396), line, TACH_RED, M_F(2));
  } else {
    char amp[10], spd[10], tr[12] = "";
    if (g123)        snprintf(amp, sizeof(amp), "%.1fA", g_g123Coil); else strcpy(amp, "--");
    if (fresh && g_speedValid) snprintf(spd, sizeof(spd), "%dKMH", (int)g_speedKmh); else strcpy(spd, "--");
    if (g_tripValid) snprintf(tr, sizeof(tr), "  T%.1f", g_tripKm);
    snprintf(line, sizeof(line), "AMP %s  %s%s", amp, spd, tr);
    drawTextCentered(240, M_Y(396), line, t.txtDim, M_F(2));
  }

  // Statuszeile unten: Uhrzeit + Datenquelle (Uhr fehlt sonst im Kombi)
  char st[24];
  const char* src = fresh ? g_lastSrc : (g_bleConn ? "WARTE" : (g_canReady ? "CAN WARTE" : "KEIN HUB"));
  struct tm mnow = {};
  if (readClockTime(&mnow))
    snprintf(st, sizeof(st), "%02d:%02d  %s%s", mnow.tm_hour, mnow.tm_min, fresh ? "LIVE " : "", src);
  else
    snprintf(st, sizeof(st), "%s%s", fresh ? "LIVE " : "", src);
  drawTextCentered(240, M_Y(424), st, fresh ? t.liveCol : t.statusBad, M_F(2));
  presentFrame();
}
#undef M_S
#undef M_X
#undef M_Y
#undef M_R
#undef M_F

// ===== Lambda-Verlauf (Trend ueber Zeit): Ringpuffer, alle 500ms ein Sample =====
#define TR_N 120                         // 120 * 500ms = 60 s Fenster
static uint8_t  g_trLam[TR_N];           // λ*100 (0 = ungueltig/Luecke)
static uint16_t g_trRpm[TR_N];           // 1/min (Kontextlinie)
static uint16_t g_trHead  = 0;           // naechster Schreibindex
static uint16_t g_trCount = 0;
static uint8_t  g_lambdaStyle = 0;       // 0 = Gauge, 1 = Verlauf

static void trendSample() {              // jeden Loop aufrufen; sampelt selbst getaktet
  static uint32_t at = 0;
  if (millis() < at) return;
  at = millis() + 500;
  bool fresh = httpFresh() || canFresh() || bleFresh() || tune123Fresh();
  int lx = (fresh && g_lambdaValid) ? (int)lroundf(g_lambda * 100.0f) : 0;
  g_trLam[g_trHead] = (uint8_t)constrain(lx, 0, 255);
  g_trRpm[g_trHead] = (uint16_t)constrain((int)lroundf(g_rpm), 0, 9999);
  g_trHead = (g_trHead + 1) % TR_N;
  if (g_trCount < TR_N) g_trCount++;
}

#define TR_PX0 72
#define TR_PX1 420
#define TR_PY0 150
#define TR_PY1 350
static inline int trLamY(float lam) {    // λ 0.75..1.25 -> y unten..oben
  float t = (lam - 0.75f) / 0.5f; if (t < 0) t = 0; if (t > 1) t = 1;
  return TR_PY1 - (int)lroundf(t * (TR_PY1 - TR_PY0));
}
static inline int trRpmY(float rpm) {    // 0..6000 1/min auf dieselbe Hoehe
  float t = rpm / 6000.0f; if (t < 0) t = 0; if (t > 1) t = 1;
  return TR_PY1 - (int)lroundf(t * (TR_PY1 - TR_PY0));
}

static void drawTrendPage() {
  if (!ensureFrame()) return;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(185, 150, 45));
  bool live  = httpFresh() || canFresh() || bleFresh() || tune123Fresh();
  bool fresh = live && g_lambdaValid;
  // Kopf: aktueller Lambda-Wert (farbcodiert) + Drehzahl
  char hb[12];
  if (fresh) snprintf(hb, sizeof(hb), "%.2f", g_lambda); else strcpy(hb, "--");
  uint16_t lc = RGB565(70, 210, 100);
  if (fresh) { if (g_lambda < 0.97f) lc = RGB565(235, 120, 40); else if (g_lambda > 1.03f) lc = RGB565(80, 160, 240); }
  else lc = RGB565(120, 90, 60);
  drawTextCentered(240, 48, "LAMBDA  60s", RGB565(200, 200, 205), 2);
  drawTextCentered(168, 92, hb, lc, 5);
  char rb[12]; snprintf(rb, sizeof(rb), "%d", (int)lroundf(g_rpm));
  drawTextCentered(338, 86, rb, RGB565(120, 170, 230), 3);
  drawTextCentered(338, 116, "1/min", RGB565(110, 110, 120), 1);
  // Soll-Band (Dev-Tab: g_alLamMin..g_alLamMax) + Stoechiometrie 1.0
  int yb0 = trLamY(g_alLamMax), yb1 = trLamY(g_alLamMin);
  fillRectFast(TR_PX0, yb0, TR_PX1 - TR_PX0, yb1 - yb0, RGB565(18, 58, 34));
  // Gitter + Achsbeschriftung
  const uint16_t grid = RGB565(42, 42, 50), dim = RGB565(120, 120, 130);
  int yg08 = trLamY(0.8f), yg10 = trLamY(1.0f), yg12 = trLamY(1.2f);
  drawLineFast(TR_PX0, yg08, TR_PX1, yg08, grid, 1);
  drawLineFast(TR_PX0, yg12, TR_PX1, yg12, grid, 1);
  drawLineFast(TR_PX0, yg10, TR_PX1, yg10, RGB565(90, 90, 100), 1);   // λ=1.0
  drawTextSmall(44, yg12 - 4, "1.2", dim, 1);
  drawTextSmall(44, yg10 - 4, "1.0", dim, 1);
  drawTextSmall(44, yg08 - 4, "0.8", dim, 1);
  drawLineFast(TR_PX0, TR_PY1, TR_PX1, TR_PY1, RGB565(60, 60, 70), 1);
  // Verlauf zeichnen (Drehzahl blass, dann Lambda kraeftig)
  int n = g_trCount;
  if (n >= 2) {
    int px = -1, pyR = 0, pyL = 0; bool pv = false;
    for (int i = 0; i < n; i++) {
      int idx = (g_trHead - n + i + 2 * TR_N) % TR_N;
      int x = TR_PX0 + (int)lroundf((float)i / (n - 1) * (TR_PX1 - TR_PX0));
      int yR = trRpmY(g_trRpm[idx]);
      if (px >= 0) drawLineFast(px, pyR, x, yR, RGB565(55, 95, 165), 1);
      pyR = yR;
      uint8_t lv = g_trLam[idx];
      if (lv > 0) {
        int yL = trLamY(lv / 100.0f);
        if (pv && px >= 0) drawLineFast(px, pyL, x, yL, RGB565(230, 190, 70), 2);
        pyL = yL; pv = true;
      } else pv = false;
      px = x;
    }
  }
  drawTextSmall(TR_PX0, 358, "-60s", dim, 1);
  drawTextSmall(388, 358, "jetzt", dim, 1);
  drawTextCentered(240, 392, live ? "lang Mitte: Anzeige" : "kein Hub", live ? RGB565(150, 150, 160) : RGB565(200, 120, 50), 1);
  presentFrame();
}

static void drawLambdaPage() {
  if (g_lambdaStyle == 1) { drawTrendPage(); return; }
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
  char wbuf[20];
  bool wok = (WiFi.status() == WL_CONNECTED);
  if (wok) snprintf(wbuf, sizeof(wbuf), "%ddBm", (int)WiFi.RSSI());
  else     snprintf(wbuf, sizeof(wbuf), "%s", wifiReasonText(g_wifiStaReason));
  drawDataRow(330, "WLAN",   wbuf, wok ? gr : og);
  char sdb[20];
  if (g_sdMounted) snprintf(sdb, sizeof(sdb), "%s %luG", g_sdType, (unsigned long)((g_sdSizeMB + 512) / 1024));
  else             strcpy(sdb, "---");
  drawDataRow(362, "SD", sdb, g_sdMounted ? gr : dk);
  presentFrame();
}

// Setup-Zeilen: EINE Tabelle fuer Zeichnung UND Touch-Zonen (handleSetupLongPress
// liest dieselben Y-Werte) - eine verschobene Zeile kann den Touch nicht mehr brechen.
// 0=UHR 1=HELL 2=ROT 3=WIFI 4=BLE 5=CAN 6=BUZZER 7=IMU0
static const int SETUP_ROW_Y[8] = { 104, 136, 168, 200, 232, 264, 296, 328 };
static void drawSetupPage() {
  if (!ensureFrame()) return;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(185, 150, 45));
  drawTextCentered(240, 50, "SETUP", RGB565(230, 190, 70), 5);
  char buf[28];
  snprintf(buf, sizeof(buf), "%d %%", g_dialScalePct);
  drawDataRow(SETUP_ROW_Y[0], "UHR",   buf, RGB565(235, 235, 225));
  snprintf(buf, sizeof(buf), "%d %%", g_brightnessPct);
  drawDataRow(SETUP_ROW_Y[1], "HELL",  buf, RGB565(235, 235, 225));
  snprintf(buf, sizeof(buf), "%d DEG", g_rotationDeg);
  drawDataRow(SETUP_ROW_Y[2], "ROT",   buf, RGB565(235, 235, 225));
  if (g_featureWifi && strlen(currentWifiSsid()) > 0) {
    snprintf(buf, sizeof(buf), "%s", WPROF_LABELS[g_wifiProfile]);
    drawDataRow(SETUP_ROW_Y[3], "WIFI", buf,
                WiFi.status() == WL_CONNECTED ? RGB565(60, 210, 100) : RGB565(220, 130, 50));
  } else {
    drawDataRow(SETUP_ROW_Y[3], "WIFI", "AUS", RGB565(220, 130, 50));
  }
  drawDataRow(SETUP_ROW_Y[4], "BLE",    g_featureBle ? (g_bleConn ? "OK" : "AN") : "AUS",
              g_featureBle && g_bleConn ? RGB565(60, 210, 100) : RGB565(220, 130, 50));
  // CAN: TIP zyklisch AUS -> AN (listen) -> ACK (normal) -> AUS
  drawDataRow(SETUP_ROW_Y[5], "CAN",
              !g_featureCan ? "AUS" : canFresh() ? (g_canListenOnly ? "OK" : "OK·ACK")
                                                 : (g_canListenOnly ? "AN" : "ACK"),
              !g_featureCan ? RGB565(150, 150, 150)
                            : canFresh() ? RGB565(60, 210, 100) : RGB565(220, 130, 50));
  drawDataRow(SETUP_ROW_Y[6], "BUZZER", g_featureBuzzer ? "AN" : "AUS",
              g_featureBuzzer ? RGB565(60, 210, 100) : RGB565(150, 150, 150));
  if (g_imuPresent) {
    qmi8658Read();
    snprintf(buf, sizeof(buf), "%+.1f DEG", g_imuPitch - g_imuOffPitch);
    drawDataRow(SETUP_ROW_Y[7], "IMU 0", buf, RGB565(235, 235, 225));   // TIP = aktuelle Lage nullen
  } else {
    drawDataRow(SETUP_ROW_Y[7], "IMU 0", "---", RGB565(150, 150, 150));
  }
  drawTextCentered(240, 368, "TIP MENU", RGB565(180, 180, 170), 2);
  presentFrame();
}

// CAN-Seite (Page 13): Status + Schalter, aus dem Setup ueber die CAN-Zeile.
// AKTIV/MODUS per Tap schaltbar; ID + Bitrate zeigen (aendern: WebGUI Dev-Tab).
static const int CAN_ROW_Y[6] = { 120, 158, 196, 234, 272, 310 };
static void drawCanPage() {
  if (!ensureFrame()) return;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 216, 3, RGB565(80, 170, 220));
  drawTextCentered(240, 56, "CAN", RGB565(120, 200, 240), 5);
  char buf[28];
  drawDataRow(CAN_ROW_Y[0], "AKTIV",   g_featureCan ? "AN" : "AUS",
              g_featureCan ? RGB565(60, 210, 100) : RGB565(150, 150, 150));
  drawDataRow(CAN_ROW_Y[1], "MODUS",   g_canListenOnly ? "LISTEN" : "ACK",
              RGB565(235, 235, 225));
  snprintf(buf, sizeof(buf), "0x%03X", g_canId);
  drawDataRow(CAN_ROW_Y[2], "ID",      buf, RGB565(235, 235, 225));
  snprintf(buf, sizeof(buf), "%uk", g_canKbps);
  drawDataRow(CAN_ROW_Y[3], "BITRATE", buf, RGB565(235, 235, 225));
  snprintf(buf, sizeof(buf), "%lu/%lu", (unsigned long)g_canRx, (unsigned long)g_canIgnored);
  drawDataRow(CAN_ROW_Y[4], "RX/IGN",  buf,
              canFresh() ? RGB565(60, 210, 100) : RGB565(150, 150, 150));
  twai_status_info_t cs = {};
  if (g_canReady && twai_get_status_info(&cs) == ESP_OK)
    snprintf(buf, sizeof(buf), "%u/%u", (unsigned)cs.rx_error_counter, (unsigned)cs.bus_error_count);
  else strcpy(buf, "---");
  drawDataRow(CAN_ROW_Y[5], "FEHLER",  buf, RGB565(235, 235, 225));
  drawTextCentered(240, 352, canFresh() ? "LIVE" : (g_featureCan ? "KEINE FRAMES" : "AUS"),
                   canFresh() ? RGB565(60, 200, 90) : RGB565(200, 120, 50), 2);
  drawTextCentered(240, 384, "TIP AKTIV/MODUS SCHALTET", RGB565(120, 120, 120), 1);
  drawTextCentered(240, 414, "TIP UNTEN ZURUECK", RGB565(180, 180, 170), 2);
  presentFrame();
}

static void handleCanTap(uint16_t, uint16_t y) {
  if (y >= 398) { currentPage = 5; drawSetupPage(); return; }   // unten -> Setup
  Preferences p;
  if ((int)y >= CAN_ROW_Y[0] - 18 && (int)y < CAN_ROW_Y[0] + 18) {        // AKTIV an/aus
    g_featureCan = !g_featureCan;
    if (g_featureCan) setupCockpitCan();
    else { twai_stop(); twai_driver_uninstall(); g_canReady = false; }
    p.begin("clock", false); p.putBool("feat_can", g_featureCan); p.end();
    sdLog(g_featureCan ? "CAN an" : "CAN aus");
    Serial.printf("CAN-Seite: aktiv=%s\n", g_featureCan ? "an" : "aus");
  } else if ((int)y >= CAN_ROW_Y[1] - 18 && (int)y < CAN_ROW_Y[1] + 18) { // MODUS listen/ACK
    g_canListenOnly = !g_canListenOnly;
    p.begin("clock", false); p.putBool("can_listen", g_canListenOnly); p.end();
    if (g_featureCan) setupCockpitCan();
    sdLog(g_canListenOnly ? "CAN modus listen" : "CAN modus ACK");
    Serial.printf("CAN-Seite: modus=%s\n", g_canListenOnly ? "listen" : "ACK");
  }
  drawCanPage();
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

  fillFrame(t.face);                                  // fuellt schon alles in t.face
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

// ===== On-Screen-Tastatur (Page 10): SSID/Passwort des aktiven Profils tippen =====
static void saveWprof(uint8_t, const char*, const char*, const char*);  // fwd
static void selectWprof(uint8_t);                                       // fwd

static char    g_kbText[65];
static uint8_t g_kbSlot  = 0;
static uint8_t g_kbField = 0;   // 0=SSID, 1=Passwort
static uint8_t g_kbLayer = 0;   // 0=abc 1=ABC 2=123/Sym

static const char* const KB_ROWS[3][4] = {
  { "1234567890", "qwertzuiop", "asdfghjkl", "yxcvbnm" },
  { "1234567890", "QWERTZUIOP", "ASDFGHJKL", "YXCVBNM" },
  { "1234567890", "@#$%&*-_=+", "!?/:;,.",   "()[]<>|" },
};
#define KB_KW 35
#define KB_KH 42
static inline int kbRowY(int r) { return 142 + r * 44; }   // Reihen 142/186/230/274 - passt in den runden Kreis

static void drawKeyboardPage();

static void openKeyboard(uint8_t slot, uint8_t field) {
  g_kbSlot  = (slot < WPROF_COUNT) ? slot : 0;
  g_kbField = field ? 1 : 0;
  g_kbLayer = 0;
  const char* src = g_kbField == 0 ? g_wprof[g_kbSlot].ssid : g_wprof[g_kbSlot].pass;
  strlcpy(g_kbText, src, sizeof(g_kbText));
  currentPage = 10;
  drawKeyboardPage();
}

// Steuerreihe-Geometrie (6 Tasten, passt mit Rand in den Kreis)
#define KB_CTL_Y  320
#define KB_CTL_H  42
#define KB_CTL_W  62
#define KB_CTL_X0 54

static void drawKeyboardPage() {
  if (!ensureFrame()) return;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 214, 3, RGB565(185, 150, 45));
  char t[40];
  if (g_kbField == 0) snprintf(t, sizeof(t), "SSID %s", WPROF_LABELS[g_kbSlot]);
  else snprintf(t, sizeof(t), "PW %.12s", g_wprof[g_kbSlot].ssid[0] ? g_wprof[g_kbSlot].ssid : "(?)");
  drawTextCentered(240, 54, t, RGB565(230, 190, 70), 2);
  // Eingabe-Box (zeigt bei langem Text nur das Ende)
  fillRectFast(92, 90, 296, 28, RGB565(28, 28, 30));
  const char* shown = g_kbText[0] ? g_kbText : "____";
  int sl = (int)strlen(shown);
  if (sl > 22) shown += (sl - 22);
  drawTextSmall(98, 96, shown, RGB565(120, 220, 140), 2);
  // Tastenfeld
  for (int r = 0; r < 4; r++) {
    const char* row = KB_ROWS[g_kbLayer][r];
    int n = (int)strlen(row);
    int x0 = 240 - (n * KB_KW) / 2;
    int y  = kbRowY(r);
    for (int i = 0; i < n; i++) {
      int kx = x0 + i * KB_KW;
      fillRectFast(kx + 2, y, KB_KW - 4, KB_KH - 4, RGB565(55, 55, 62));
      char c[2] = { row[i], 0 };
      drawTextCentered(kx + KB_KW / 2, y + 11, c, RGB565(235, 235, 225), 2);
    }
  }
  // Steuerreihe
  const char* ctl[6];
  ctl[0] = (g_kbLayer == 0) ? "ABC" : (g_kbLayer == 1) ? "123" : "abc";
  ctl[1] = "SPC"; ctl[2] = "DEL"; ctl[3] = "CLR"; ctl[4] = "OK"; ctl[5] = "ESC";
  for (int i = 0; i < 6; i++) {
    int bx = KB_CTL_X0 + i * KB_CTL_W;
    uint16_t bg = (i == 4) ? RGB565(40, 110, 60) : (i == 5) ? RGB565(120, 50, 45) : RGB565(60, 60, 66);
    fillRectFast(bx + 2, KB_CTL_Y, KB_CTL_W - 4, KB_CTL_H, bg);
    drawTextCentered(bx + KB_CTL_W / 2, KB_CTL_Y + 13, ctl[i], RGB565(235, 235, 225), 2);
  }
  presentFrame();
}

static void handleKbTap(uint16_t x, uint16_t y) {
  if (y >= KB_CTL_Y - 2) {              // Steuerreihe
    int idx = ((int)x - KB_CTL_X0) / KB_CTL_W;
    if (idx < 0) idx = 0; if (idx > 5) idx = 5;
    int l = (int)strlen(g_kbText);
    switch (idx) {
      case 0: g_kbLayer = (g_kbLayer + 1) % 3; break;
      case 1: if (l < 64) { g_kbText[l] = ' '; g_kbText[l + 1] = 0; } break;
      case 2: if (l > 0) g_kbText[l - 1] = 0; break;
      case 3: g_kbText[0] = 0; break;
      case 4:                           // OK
        if (g_kbField == 0) {           // SSID gespeichert -> weiter zum Passwort
          saveWprof(g_kbSlot, g_kbText, g_wprof[g_kbSlot].pass, g_wprof[g_kbSlot].hubip);
          openKeyboard(g_kbSlot, 1);
        } else {                        // Passwort gespeichert -> aktiv + verbinden
          saveWprof(g_kbSlot, g_wprof[g_kbSlot].ssid, g_kbText, g_wprof[g_kbSlot].hubip);
          selectWprof(g_kbSlot);
          currentPage = 5; drawSetupPage();
        }
        return;
      case 5: currentPage = 5; drawSetupPage(); return;   // ESC
    }
    drawKeyboardPage();
    return;
  }
  for (int r = 0; r < 4; r++) {         // Tastenfeld
    int yy = kbRowY(r);
    if ((int)y < yy || (int)y >= yy + KB_KH) continue;
    const char* row = KB_ROWS[g_kbLayer][r];
    int n = (int)strlen(row);
    int x0 = 240 - (n * KB_KW) / 2;
    if ((int)x < x0 || (int)x >= x0 + n * KB_KW) return;
    int i = ((int)x - x0) / KB_KW;
    if (i >= 0 && i < n) {
      int l = (int)strlen(g_kbText);
      if (l < 64) { g_kbText[l] = row[i]; g_kbText[l + 1] = 0; }
    }
    drawKeyboardPage();
    return;
  }
}

// ===== WLAN-Seite (Page 11): Buttons WPS / SSID / Passwort / Profil / Zurueck =====
#define WLAN_BTN_X 108
#define WLAN_BTN_W 264
#define WLAN_BTN_H 48
#define WLAN_BTN_N 5
static const int WLAN_BTN_Y[WLAN_BTN_N] = { 146, 200, 254, 308, 362 };

static void drawWlanPage() {
  if (!ensureFrame()) return;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 214, 3, RGB565(185, 150, 45));
  drawTextCentered(240, 48, "WLAN", RGB565(230, 190, 70), 3);
  char l[56];
  snprintf(l, sizeof(l), "%s: %s", WPROF_LABELS[g_wifiProfile], currentWifiSsid()[0] ? currentWifiSsid() : "(leer)");
  drawTextCentered(240, 90, l, RGB565(200, 200, 205), 2);
  bool conn = (WiFi.status() == WL_CONNECTED);
  if (conn) snprintf(l, sizeof(l), "verbunden  %s", g_ipStr); else snprintf(l, sizeof(l), "nicht verbunden");
  drawTextCentered(240, 114, l, conn ? RGB565(120, 220, 140) : RGB565(225, 150, 120), 2);
  if      (g_wpsState == WPS_RUN)  drawTextCentered(240, 136, "WPS laeuft - Router-Taste druecken", RGB565(230, 200, 80), 1);
  else if (g_wpsState == WPS_OK)   drawTextCentered(240, 136, "WPS ok", RGB565(120, 220, 140), 1);
  else if (g_wpsState == WPS_FAIL) drawTextCentered(240, 136, "WPS fehlgeschlagen", RGB565(225, 120, 120), 1);
  const char* lbl[WLAN_BTN_N]  = { "Netze scannen", "SSID tippen", "Passwort tippen",
                                   "Profil wechseln", "Zurueck" };
  const uint16_t bg[WLAN_BTN_N] = { RGB565(40,110,60), RGB565(50,95,140), RGB565(55,80,120),
                                    RGB565(70,70,78), RGB565(95,60,55) };
  for (int i = 0; i < WLAN_BTN_N; i++) {
    fillRectFast(WLAN_BTN_X, WLAN_BTN_Y[i], WLAN_BTN_W, WLAN_BTN_H, bg[i]);
    drawTextCentered(240, WLAN_BTN_Y[i] + 16, lbl[i], RGB565(240, 240, 235), 2);
  }
  presentFrame();
}

// ===== WLAN-Scan (Page 12): einmaliger Scan, SSID antippen -> Profil + PW-Tastatur =====
// Bewusst EINMALIG auf Knopfdruck (blockiert ~2-3 s), kein Dauerscan. Der fruehere
// Scan-Crash (StoreProhibited) war sehr wahrscheinlich die FB-Overflow-Heap-Korruption
// (ba1909b) - falls doch nicht, faellt nur dieser Button aus, nicht der Boot.
#define SCAN_MAX 7
static int16_t g_scanN = -1;

static void drawScanPage() {
  if (!ensureFrame()) return;
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 214, 3, RGB565(185, 150, 45));
  drawTextCentered(240, 48, "NETZE", RGB565(230, 190, 70), 3);
  char l[48];
  snprintf(l, sizeof(l), "Tippen = SSID in Profil %s", WPROF_LABELS[g_wifiProfile]);
  drawTextCentered(240, 84, l, RGB565(150, 150, 160), 1);
  if (g_scanN <= 0) {
    drawTextCentered(240, 220, g_scanN == 0 ? "keine Netze gefunden" : "Scan-Fehler",
                     RGB565(225, 150, 120), 2);
  }
  int n = (g_scanN > SCAN_MAX) ? SCAN_MAX : g_scanN;
  for (int i = 0; i < n; i++) {
    int y = 108 + i * 38;
    fillRectFast(78, y, 324, 34, RGB565(38, 40, 46));
    snprintf(l, sizeof(l), "%.18s", WiFi.SSID(i).c_str());
    drawTextSmall(88, y + 10, l, RGB565(235, 235, 225), 2);
    snprintf(l, sizeof(l), "%d", (int)WiFi.RSSI(i));
    drawTextSmall(352, y + 10, l, RGB565(140, 170, 140), 2);
  }
  fillRectFast(WLAN_BTN_X, 396, WLAN_BTN_W, 40, RGB565(95, 60, 55));
  drawTextCentered(240, 408, "Zurueck", RGB565(240, 240, 235), 2);
  presentFrame();
}

static void runWlanScan() {
  fillFrame(RGB565_BLACK);
  drawCircleLine(240, 240, 214, 3, RGB565(185, 150, 45));
  drawTextCentered(240, 220, "Scanne...", RGB565(230, 190, 70), 3);
  presentFrame();
  g_wlanScanBusy = true;                    // Hub-/123-BLE-Ticks aussetzen (Radio-Konflikt)
  // KERN-FIX: laufenden NimBLE-Scan stoppen. BLE + WiFi teilen sich das 2.4GHz-Radio;
  // ein aktiver BLE-Scan (123-Fallback bei fehlenden Hub-Daten) laesst esp_wifi_scan_start
  // haengen/scheitern ("wlan scan failed"). Genau der Zustand des Bus-Displays.
  if (g_bleInited) { NimBLEDevice::getScan()->stop(); delay(150); }
  WiFi.scanDelete();
  WiFi.setAutoReconnect(false);
  WiFi.disconnect();                        // laufenden Connect abbrechen (sonst "sta is connecting")
  delay(300);
  // Kurze Kanalzeit (200ms) -> ~3s statt >5s, sonst schlaegt der Task-Watchdog an.
  g_scanN = WiFi.scanNetworks(false, false, false, 200);
  if (g_scanN < 0) { delay(400); g_scanN = WiFi.scanNetworks(false, false, false, 200); }
  WiFi.setAutoReconnect(true);
  g_wlanScanBusy = false;
  reconnectWifiProfile();                   // gleich wieder verbinden (nicht auf Auto-Tick warten)
  Serial.printf("WLAN-Scan: %d Netze\n", (int)g_scanN);
  currentPage = 12;
  drawScanPage();
}

static void handleScanTap(uint16_t x, uint16_t y) {
  if ((int)y >= 396) {                     // Zurueck (Reconnect uebernimmt Auto/Tick)
    WiFi.scanDelete(); g_scanN = -1;
    reconnectWifiProfile();
    currentPage = 11; drawWlanPage(); return;
  }
  int n = (g_scanN > SCAN_MAX) ? SCAN_MAX : g_scanN;
  for (int i = 0; i < n; i++) {
    int ry = 108 + i * 38;
    if ((int)y < ry || (int)y >= ry + 34) continue;
    String ss = WiFi.SSID(i);
    if (!ss.length()) return;
    // Ziel-Slot: kennt ein Profil die SSID schon, DAS nehmen - sonst ueberschreibt
    // ein Tap auf "Heim-Netz" im Bus versehentlich das aktive Hub-AP-Profil.
    uint8_t slot = g_wifiProfile;
    for (uint8_t p = 0; p < WPROF_COUNT; p++)
      if (!strcmp(g_wprof[p].ssid, ss.c_str())) { slot = p; break; }
    WiFi.scanDelete(); g_scanN = -1;
    if (!strcmp(g_wprof[slot].ssid, ss.c_str()) && g_wprof[slot].pass[0]) {
      Serial.printf("Scan: '%s' = Profil %s (PW gespeichert) -> verbinden\n", ss.c_str(), WPROF_LABELS[slot]);
      selectWprof(slot);                   // Passwort vorhanden -> direkt verbinden
      currentPage = 11; drawWlanPage();
      return;
    }
    saveWprof(slot, ss.c_str(), g_wprof[slot].pass, g_wprof[slot].hubip);
    Serial.printf("Scan: '%s' -> Profil %s, PW tippen\n", ss.c_str(), WPROF_LABELS[slot]);
    openKeyboard(slot, 1);                 // Passwort tippen (OK verbindet)
    return;
  }
}

static void handleWlanTap(uint16_t x, uint16_t y) {
  for (int i = 0; i < WLAN_BTN_N; i++) {
    if ((int)y < WLAN_BTN_Y[i] || (int)y >= WLAN_BTN_Y[i] + WLAN_BTN_H) continue;
    if      (i == 0) { runWlanScan(); }                                  // Netze scannen -> Page 12
    else if (i == 1) { openKeyboard(g_wifiProfile, 0); }                 // SSID (-> danach PW)
    else if (i == 2) { openKeyboard(g_wifiProfile, 1); }                 // nur Passwort
    else if (i == 3) { cycleWifiProfile();  drawWlanPage(); }            // naechstes Profil
    else             { currentPage = 5;     drawSetupPage(); }           // zurueck -> Setup
    return;
  }
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
  else if (currentPage == 10) drawKeyboardPage();
  else if (currentPage == 11) drawWlanPage();
  else if (currentPage == 12) drawScanPage();
  else if (currentPage == 13) drawCanPage();
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
    // ECHTER Bus-Hub-AP = "SPARTAN3-HUB" (per Hub-API bestaetigt; auf Z00 als .91).
    // Der Test-Hub im Haus heisst "Spartan3-TestHub" - per wifi:set/wifi.txt eintragen.
    strncpy(g_wprof[1].ssid, "SPARTAN3-HUB", sizeof(g_wprof[1].ssid) - 1);
    strncpy(g_wprof[1].pass, "lambda123",    sizeof(g_wprof[1].pass) - 1);
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
  g_featureCan    = p.getBool("feat_can", false);     // CAN default OFF (kein Bus/Transceiver noetig)
  g_autoCockpit    = p.getBool("auto_cock", true);    // Auto-Cockpit default AN
  g_autoCockpitRpm = p.getInt("acock_rpm", 600);      // Schwelle variabel
  if (g_autoCockpitRpm < 100 || g_autoCockpitRpm > 5000) g_autoCockpitRpm = 600;
  g_testHub        = p.getBool("thub", false);         // Test-Hub-Modus (Dev) default AUS
  { char tip[40] = ""; p.getString("thub_ip", tip, sizeof(tip));
    if (tip[0]) snprintf(g_testHubIp, sizeof(g_testHubIp), "%s", tip); }
  g_imuOffPitch   = p.getFloat("imu_off_p", 0.0f);     // IMU-Nullung
  g_imuOffRoll    = p.getFloat("imu_off_r", 0.0f);
  g_wifiAuto      = p.getBool("wifi_auto", true);      // WLAN-Auto-Fallback default AN
  g_canListenOnly = p.getBool("can_listen", true);     // CAN: listen-only default (NORMAL = ACK)
  g_canId         = p.getUShort("can_id", 0x510);      // CAN: Frame-ID (Dev-Tab)
  g_canKbps       = p.getUShort("can_kbps", 500);      // CAN: Bitrate (Dev-Tab)
  if (g_canKbps != 125 && g_canKbps != 250 && g_canKbps != 1000) g_canKbps = 500;
  g_motorStyle    = p.getUChar("mstyle", 2);          // 0=digital,1=vdo,2=123tune+,3=vdo+uhr
  if (g_motorStyle > MOTOR_STYLE_COUNT - 1) g_motorStyle = 2;
  g_alertsOn   = p.getBool("alerts", true);            // Alarme (Dev-Tab)
  g_alLamMin   = p.getFloat("al_lmin", 0.90f);
  g_alLamMax   = p.getFloat("al_lmax", 1.10f);
  g_alRpmMax   = p.getInt("al_rpm", 4500);
  g_alTempMax  = p.getInt("al_temp", 90);
  g_alVoltMin  = p.getFloat("al_volt", 11.5f);
  g_rpmRedline = p.getInt("rl_rpm", 6000);             // Tacho-Konfig (Dev-Tab)
  g_rpmScaleMax = p.getInt("sc_rpm", 8000);
  if (g_rpmScaleMax < 3000 || g_rpmScaleMax > 12000) g_rpmScaleMax = 8000;
  if (g_rpmRedline < 1000 || g_rpmRedline > g_rpmScaleMax) g_rpmRedline = g_rpmScaleMax * 3 / 4;
  g_lambdaStyle   = p.getUChar("lstyle", 0);          // 0=Gauge, 1=Verlauf
  if (g_lambdaStyle > 1) g_lambdaStyle = 0;
  p.end();
  // Einmalige Migration: falsch getippte/veraltete Hub-AP-Namen (Tastatur konnte nur
  // eine Schreibung darstellen, SSIDs sind case-sensitiv) auf den echten AP-Namen
  // ziehen. "Spartan3-TestHub" bleibt bewusst unangetastet (Schreibtisch/Test).
  if (!strcmp(g_wprof[1].ssid, "Spartan3-hup") || !strcmp(g_wprof[1].ssid, "SPARTAN3-HUP") ||
      !strcmp(g_wprof[1].ssid, "Spartan3-Hub")) {
    strncpy(g_wprof[1].ssid, "SPARTAN3-HUB", sizeof(g_wprof[1].ssid) - 1);
    strncpy(g_wprof[1].pass, "lambda123",    sizeof(g_wprof[1].pass) - 1);
    Preferences pm; pm.begin("clock", false);
    pm.putString("wp1_s", g_wprof[1].ssid);
    pm.putString("wp1_p", g_wprof[1].pass);
    pm.end();
    Serial.println("Migration: Hub-AP-Profil -> SPARTAN3-HUB");
  }
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
  if (style > MOTOR_STYLE_COUNT - 1) style = MOTOR_STYLE_COUNT - 1;
  g_motorStyle = (uint8_t)style;
  Preferences p;
  p.begin("clock", false);
  p.putUChar("mstyle", g_motorStyle);
  p.end();
}

static void saveLambdaStyle(int s) {       // 0=Gauge, 1=Verlauf
  g_lambdaStyle = (uint8_t)(s & 1);
  Preferences p;
  p.begin("clock", false);
  p.putUChar("lstyle", g_lambdaStyle);
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
// --- SD/WLAN-Helfer fuer die WebGUI ---
static String htmlEscape(const String& in) {
  String o; o.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if      (c == '&') o += "&amp;";
    else if (c == '<') o += "&lt;";
    else if (c == '>') o += "&gt;";
    else o += c;
  }
  return o;
}
static String sdReadWifiTxt() {
  if (!g_sdMounted || !SD_MMC.exists("/wifi.txt")) return String("");
  File f = SD_MMC.open("/wifi.txt", FILE_READ);
  if (!f) return String("");
  String s = f.readString();
  f.close();
  return s;
}
// ===== SD-Systemlog: Ereignisse (Boot/WLAN/Alarm/OTA) fuer spaetere Auswertung =====
// Eine Datei pro Tag (/log/YYYYMMDD.txt), Zeile "HH:MM:SS  Ereignis". Bewusst nur
// Ereignisse (Kanten), keine Telemetrie - Lambda/RPM jede Sekunde waere Datenflut
// und witzlos (dafuer gibt es /live). Fehlschlag ist still: das Log darf den
// eigentlichen Betrieb nie stoeren, auch nicht durch g_sdMounted=false-Flapping.
static void sdLog(const char* msg) {
  if (!g_sdMounted) return;
  struct tm now = {};
  readClockTime(&now);
  char path[24];
  snprintf(path, sizeof(path), "/log/%04d%02d%02d.txt",
           now.tm_year + 1900, now.tm_mon + 1, now.tm_mday);
  File f = SD_MMC.open(path, FILE_APPEND);
  if (!f) return;
  f.printf("%02d:%02d:%02d  %s\n", now.tm_hour, now.tm_min, now.tm_sec, msg);
  f.close();
}

// Aktuelle Profile -> /wifi.txt spiegeln. Ohne das ueberschreibt eine alte SD-Datei
// beim naechsten Boot jede per Console/Tastatur/WebGUI gemachte Aenderung.
static bool g_sdApplyingWifi = false;
static void sdSyncWifiTxt() {
  if (!g_sdMounted || g_sdApplyingWifi) return;
  // Erst Temp-Datei, dann Rename: Stromausfall mitten im Schreiben (Zuendung aus)
  // darf keine halbe wifi.txt hinterlassen - die wuerde beim Boot in NVS importiert.
  File w = SD_MMC.open("/wifi.tmp", FILE_WRITE);
  if (!w) {                       // Karte gezogen/Kontaktproblem: nicht weiter dagegen anrennen,
    g_sdMounted = false;          // jeder SDMMC-Zugriff auf tote Karte blockiert den Loop spuerbar
    Serial.println("SD: Schreiben fehlgeschlagen -> als nicht gemountet markiert");
    return;
  }
  w.println("# WLAN-Profile:  <Slot>=<SSID>|<Passwort>   (0=Heim 1=Hub-AP 2=S24)");
  for (int i = 0; i < WPROF_COUNT; i++)
    if (g_wprof[i].ssid[0]) w.printf("%d=%s|%s\n", i, g_wprof[i].ssid, g_wprof[i].pass);
  w.close();
  SD_MMC.remove("/wifi.txt");
  SD_MMC.rename("/wifi.tmp", "/wifi.txt");
}
// Nur druckbares ASCII zulassen - Muell-Bytes (kaputte Datei) nie in Profile uebernehmen.
static bool wifiTxtClean(const String& s) {
  for (size_t i = 0; i < s.length(); i++)
    if ((uint8_t)s[i] < 32 || (uint8_t)s[i] > 126) return false;
  return true;
}
// Lebensader Bus: das Live-Hub-Profil muss IMMER waehlbar bleiben. Egal was
// Scan/Tastatur/SD-Datei angerichtet haben - fehlt SPARTAN3-HUB in allen
// Profilen, wird Slot 1 wiederhergestellt (nach dem SD-Import aufrufen!).
static void ensureHubProfile() {
  for (uint8_t p = 0; p < WPROF_COUNT; p++)
    if (!strcmp(g_wprof[p].ssid, "SPARTAN3-HUB")) return;
  Serial.println("WLAN: Live-Hub-Profil fehlte -> Slot 1 = SPARTAN3-HUB wiederhergestellt");
  saveWprof(1, "SPARTAN3-HUB", "lambda123", g_wprof[1].hubip);
}
// /wifi.txt parsen -> g_wprof + NVS. Anzahl geladener Profile (-2 = keine Datei).
static int sdApplyWifiTxt() {
  if (!g_sdMounted || !SD_MMC.exists("/wifi.txt")) return -2;
  File f = SD_MMC.open("/wifi.txt", FILE_READ);
  if (!f) return -2;
  g_sdApplyingWifi = true;
  int n = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0 || line[0] == '#') continue;
    int eq = line.indexOf('='), bar = line.indexOf('|');
    if (eq < 1 || bar <= eq) continue;
    int slot = line.substring(0, eq).toInt();
    if (slot < 0 || slot >= WPROF_COUNT) continue;
    String ssid = line.substring(eq + 1, bar);
    String pass = line.substring(bar + 1);
    if (!wifiTxtClean(ssid) || !wifiTxtClean(pass)) {
      Serial.printf("wifi.txt: Profil %d hat Muell-Zeichen -> Zeile ignoriert\n", slot);
      continue;
    }
    saveWprof((uint8_t)slot, ssid.c_str(), pass.c_str(), g_wprof[slot].hubip);
    n++;
    Serial.printf("wifi.txt: Profil %d = '%s' (pw-len %u)\n", slot, ssid.c_str(), (unsigned)pass.length());
  }
  f.close();
  g_sdApplyingWifi = false;
  return n;
}

static void handleWebRoot() {
  struct tm now = {};
  readClockTime(&now);
  char timeStr[16];
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", now.tm_hour, now.tm_min, now.tm_sec);

  String html;
  html.reserve(16384);     // ~14 KB HTML: ohne reserve ~150 reallocs -> Heap-Fragmentierung
  html += F("<!DOCTYPE html><html lang='de'><head><meta charset='utf-8'>"
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
    "<button class='tabbtn' onclick=\"sh('scr',this);drawScreen()\">Vorschau</button>"
    "<button class='tabbtn' onclick=\"sh('wlan',this)\">WLAN</button>"
    "<button class='tabbtn' onclick=\"sh('anz',this)\">Anzeige</button>"
    "<button class='tabbtn' onclick=\"sh('imu',this)\">IMU</button>"
    "<button class='tabbtn' onclick=\"sh('sys',this)\">System</button>"
    "<button class='tabbtn' onclick=\"sh('sd',this)\">SD</button>"
    "<button class='tabbtn' onclick=\"sh('dev',this)\">Dev</button></div>");

  // ===== Tab: Live =====
  html += F("<div class='tab on' id='t-live'>");
  if (g_testHub)   // Test-Modus deutlich markieren (Daten kommen NICHT vom echten Hub!)
    html += "<div style='background:#a45500;color:#fff;border-radius:10px;padding:10px;margin:10px auto;"
            "max-width:420px;font-weight:600'>&#9888; TEST-MODUS &ndash; Daten vom Test-Hub " +
            String(g_testHubIp) + " (Dev-Tab)</div>";
  html += F("<div class='card'><h3>Spartan-Hub Live</h3>");
  bool anyFresh = bleFresh() || canFresh() || httpFresh() || tune123Fresh();
  html += "<div>Quelle: <b id='lv_src'>" + String(anyFresh ? g_lastSrc : "---") + "</b> &middot; " +
          "<span id='lv_fresh'>" + String(anyFresh ? "LIVE" : "keine Daten") + "</span></div>";
  html += "<div>Lambda: <span id='lv_lam'>" + String(g_lambdaValid ? String(g_lambda, 2) : String("---")) +
          "</span> &nbsp; RPM: <span id='lv_rpm'>" + String((int)g_rpm) +
          "</span> &nbsp; ADV: <span id='lv_adv'>" + String(g_adv, 1) + "</span></div>";
  html += "<div>MAP: <span id='lv_map'>" + String((int)g_map) + "</span> &nbsp; TEMP: <span id='lv_temp'>" +
          String((int)g_g123Temp) + "</span> &nbsp; VOLT: <span id='lv_volt'>" + String(g_g123Volt, 1) +
          "</span> &nbsp; AMP: <span id='lv_amp'>" + String(g_g123Coil, 1) + "</span></div>";
  html += "<div style='color:#888'>HTTP rx <span id='lv_http'>" + String((unsigned long)g_httpRx) +
          "</span> &middot; CAN rx <span id='lv_can'>" + String((unsigned long)g_canRx) +
          "</span> &middot; BLE rx <span id='lv_ble'>" + String((unsigned long)g_bleRxCnt) + "</span></div>";
  html += "<div id='lv_alarm' style='color:#f55;font-weight:700;margin-top:6px'>" +
          String(g_alertMask ? (String("&#9888; ALARM: ") + g_alertText) : String("")) + "</div></div>";
  html += F("<div class='card'><h3>Display-Seite</h3>"
    "<a href='/page?p=0'><button>Uhr</button></a>"
    "<a href='/page?p=1'><button>Menu</button></a>"
    "<a href='/page?p=2'><button>Motor</button></a>"
    "<a href='/page?p=3'><button>Lambda</button></a>"
    "<a href='/page?p=4'><button>Hub</button></a>"
    "<a href='/page?p=6'><button>IMU</button></a>"
    "<a href='/page?p=5'><button>Setup</button></a></div>");
  html += F("</div>");

  // ===== Tab: Vorschau (Live-Screenshot des Displays) =====
  html += F("<div class='tab' id='t-scr'><div class='card'><h3>Display-Vorschau</h3>"
    "<canvas id='scr' width='160' height='160' style='width:288px;height:288px;border-radius:50%;"
    "background:#000;image-rendering:pixelated'></canvas>"
    "<div style='margin-top:10px'><label><input type='checkbox' id='scrAuto' checked> Auto (1,5 s)</label> "
    "<button onclick='drawScreen()'>Aktualisieren</button></div>"
    "<div id='scrInfo' style='color:#888'>&nbsp;</div>"
    "<div style='color:#666;font-size:.85em'>Spiegelt den echten Screen (inkl. Rotation). "
    "Auto-Refresh nur solange dieser Tab offen ist.</div></div></div>");

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
  for (uint8_t i = 0; i < MOTOR_STYLE_COUNT; i++) {
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
  html += F("<div class='card'><h3>Funktionen</h3><form action='/features' method='get'>"
            "<input type='hidden' name='fsave' value='1'>");
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
  html += F("<div class='card' style='color:#888'>Firmware-Stand: <b style='color:#e0c040'>");
  html += F(FW_BUILD);
  html += F("</b><br>Git: <a href='" GITHUB_URL "' target='_blank'>" GIT_REV "</a>"
    " &middot; <a href='/version'>/version</a></div>");
  boardBattRead();
  html += F("<div class='card' style='color:#888'>Board-Akku (MX1.25): ");
  if (g_boardBattPresent) {
    char bb[48];
    snprintf(bb, sizeof(bb), "<b style='color:#e0c040'>%.2f V</b> (~%d %%)", g_boardBattVolt, boardBattPct());
    html += bb;
  } else {
    html += F("<b>kein Akku / nur USB</b>");
  }
  html += F("</div>");
  html += F("</div>");

  // ===== Tab: SD =====
  html += F("<div class='tab' id='t-sd'><div class='card'><h3>SD-Karte</h3>");
  if (g_sdMounted) {
    html += "<div>Status: <b style='color:#6c6'>gemountet</b> &middot; " + String(g_sdType) +
            " &middot; " + String((g_sdSizeMB + 512) / 1024) + " GB</div>";
    html += "<div style='color:#888'>wifi.txt: " +
            String(g_sdWifiLoaded == -1 ? "Vorlage angelegt" :
                   g_sdWifiLoaded >= 0 ? String(g_sdWifiLoaded) + " Profil(e) geladen" : "keine Datei") +
            "</div></div>";
    html += F("<div class='card'><h3>WLAN per wifi.txt</h3>"
      "<form action='/sdwifi' method='post'>"
      "<textarea name='txt' spellcheck='false' style='width:92%;height:130px;background:#111;color:#eee;"
      "border:0;border-radius:6px;padding:8px;font-family:monospace;font-size:1em'>");
    html += htmlEscape(sdReadWifiTxt());
    html += F("</textarea><br><button type='submit'>Auf SD speichern + &uuml;bernehmen</button></form>"
      "<div style='color:#888;text-align:left'>Je Zeile <b>Slot=SSID|Passwort</b> "
      "(0=Heim 1=Hub-AP 2=S24), '#' = Kommentar. Wird auf die Karte geschrieben und sofort "
      "in die Profile geladen (aktiv beim n&auml;chsten Reconnect/Neustart).</div></div>");
    html += F("<div class='card'><h3>OTA-Recovery von SD</h3>"
      "<div style='color:#888;text-align:left'>Firmware als <b>/update.bin</b> auf die Karte legen "
      "und Display neu starten &rarr; flasht lokal <b>ohne WLAN</b>, benennt danach in "
      "<b>update.done</b> (Erfolg) bzw. <b>update.bad</b> um.</div>");
    html += "<div>Letztes Ergebnis: <b style='color:#e0c040'>" + String(g_sdOtaResult) + "</b>";
    if (SD_MMC.exists("/update.done")) html += F(" &middot; update.done da");
    if (SD_MMC.exists("/update.bad"))  html += F(" &middot; update.bad da");
    html += F("</div></div>");
    html += F("<div class='card'><h3>System-Log</h3>"
      "<div style='color:#888;text-align:left'>Ereignisse (Boot/WLAN/Alarm/OTA) je Tag als "
      "<b>/log/JJJJMMTT.txt</b>. <a href='/log'>Heutiges Log ansehen</a></div></div>");
  } else {
    html += F("<div>Status: <b style='color:#c66'>nicht gemountet</b> &ndash; keine Karte erkannt.</div></div>");
  }
  html += F("</div>");

  // ===== Tab: Dev =====
  html += F("<div class='tab' id='t-dev'><div class='card'><h3>Dev / Spielereien</h3>"
    "<form action='/set' method='get'><input type='hidden' name='devsave' value='1'>"
    "<p><label><input type='checkbox' name='acock' value='1' ");
  html += g_autoCockpit ? "checked" : "";
  html += F("> Auto-Cockpit aktiv</label></p>"
    "<p>Drehzahl-Schwelle: <input type='number' name='acockrpm' min='100' max='5000' value='");
  html += String(g_autoCockpitRpm);
  html += F("' style='width:90px;padding:6px;border:0;border-radius:6px'> 1/min</p>"
    "<hr style='border-color:#333'>"
    "<p><label><input type='checkbox' name='thub' value='1' ");
  html += g_testHub ? "checked" : "";
  html += F("> <b>Test-Hub verwenden</b>");
  if (g_testHub) html += F(" <span style='background:#a45500;color:#fff;border-radius:6px;"
                           "padding:2px 8px'>AKTIV</span>");
  html += F("</label></p>"
    "<p>Test-Hub IP: <input name='thubip' value='");
  html += String(g_testHubIp);
  html += F("' style='width:160px;padding:6px;border:0;border-radius:6px'></p>"
    "<hr style='border-color:#333'>"
    "<p><label><input type='checkbox' name='alerts' value='1' ");
  html += g_alertsOn ? "checked" : "";
  html += F("> <b>Alarme</b> (roter Ring blinkt");
  if (g_alertMask) html += " &ndash; <span style='color:#f55'>GERADE AKTIV: " + String(g_alertText) + "</span>";
  html += F(")</label></p>"
    "<p>&lambda;-Soll-Band <input name='allmin' type='number' step='0.01' min='0.5' max='1.5' value='");
  html += String(g_alLamMin, 2);
  html += F("' style='width:70px;padding:6px;border:0;border-radius:6px'> bis "
    "<input name='allmax' type='number' step='0.01' min='0.5' max='1.5' value='");
  html += String(g_alLamMax, 2);
  html += F("' style='width:70px;padding:6px;border:0;border-radius:6px'></p>"
    "<p>Drehzahl &gt; <input name='alrpm' type='number' min='1000' max='9000' step='100' value='");
  html += String(g_alRpmMax);
  html += F("' style='width:80px;padding:6px;border:0;border-radius:6px'> 1/min &nbsp; "
    "Temp &gt; <input name='altemp' type='number' min='40' max='150' value='");
  html += String(g_alTempMax);
  html += F("' style='width:60px;padding:6px;border:0;border-radius:6px'> &deg;C</p>"
    "<p>Spannung &lt; <input name='alvolt' type='number' step='0.1' min='8' max='14' value='");
  html += String(g_alVoltMin, 1);
  html += F("' style='width:70px;padding:6px;border:0;border-radius:6px'> V (Motor l&auml;uft)</p>"
    "<hr style='border-color:#333'>"
    "<p>Tacho: rot ab <input name='rlrpm' type='number' min='1000' max='12000' step='500' value='");
  html += String(g_rpmRedline);
  html += F("' style='width:80px;padding:6px;border:0;border-radius:6px'> &nbsp; Skala bis "
    "<input name='scrpm' type='number' min='3000' max='12000' step='1000' value='");
  html += String(g_rpmScaleMax);
  html += F("' style='width:80px;padding:6px;border:0;border-radius:6px'> 1/min</p>"
    "<hr style='border-color:#333'>"
    "<p><label><input type='checkbox' name='canon' value='1' ");
  html += g_featureCan ? "checked" : "";
  html += F("> <b>CAN-Cockpit</b>");
  if (g_canReady) {
    char cs[64];
    snprintf(cs, sizeof(cs), " <span style='color:#6c6'>rx=%lu ign=%lu</span>",
             (unsigned long)g_canRx, (unsigned long)g_canIgnored);
    html += cs;
  }
  html += F("</label></p>"
    "<p>Modus: <select name='canmode' style='padding:6px;border:0;border-radius:6px'>"
    "<option value='1'");
  html += g_canListenOnly ? " selected" : "";
  html += F(">listen-only (Bus mit Spartan-ACK)</option><option value='0'");
  html += g_canListenOnly ? "" : " selected";
  html += F(">NORMAL/ACK (2-Knoten-Pr&uuml;fstand)</option></select></p>"
    "<p>ID 0x<input name='canid' value='");
  { char idb[8]; snprintf(idb, sizeof(idb), "%03X", g_canId); html += idb; }
  html += F("' style='width:60px;padding:6px;border:0;border-radius:6px'> &nbsp; Bitrate "
    "<select name='cankbps' style='padding:6px;border:0;border-radius:6px'>");
  {
    const uint16_t rates[4] = { 125, 250, 500, 1000 };
    for (int i = 0; i < 4; i++) {
      html += "<option value='" + String(rates[i]) + "'" +
              (g_canKbps == rates[i] ? " selected" : "") + ">" + String(rates[i]) + "k</option>";
    }
  }
  html += F("</select></p>"
    "<button type='submit'>Speichern</button></form>"
    "<div style='color:#888;text-align:left'>Auto-Cockpit: ab dieser Drehzahl (Hub/CAN/123) springt "
    "das Display automatisch aufs Motor-Cockpit; Motor aus &rarr; zur&uuml;ck zur Uhr. "
    "<br><b>Test-Hub:</b> zieht die Cockpit-Daten von der festen IP statt vom Profil-Hub "
    "(&uuml;bersteuert auch das Hub-AP-Gateway). Quelle hei&szlig;t dann &quot;TEST&quot; "
    "&ndash; auf dem Display und im Live-Tab. F&uuml;r Tests im Heimnetz (z.B. Lambda-Sweep); "
    "im Bus wieder ausschalten!"
    "<br><b>Alarme:</b> bei Verletzung blinkt auf jeder Seite ein roter Ring + Grund oben; "
    "Buzzer piept alle 2s (wenn Buzzer-Feature an). &lambda;-Band gilt nur bei laufendem Motor "
    "und f&auml;rbt auch das gr&uuml;ne Band im Lambda-Verlauf."
    "<br><b>Tacho:</b> roter Bereich + Skalenende wirken auf alle Kombi-Stile.</div></div></div>");

  html += F("<p style='color:#666'>VW T2b Cockpit &middot; ESP32-S3 2.8\"</p>"
    "<script>function sh(t,b){var x=document.querySelectorAll('.tab');"
    "for(var i=0;i<x.length;i++)x[i].className='tab';"
    "document.getElementById('t-'+t).className='tab on';"
    "var y=document.querySelectorAll('.tabbtn');"
    "for(var i=0;i<y.length;i++)y[i].style.background='';b.style.background='#6c6';}"
    "window.addEventListener('load',function(){var b=document.getElementById('b-live');if(b)b.style.background='#6c6';});"
    "function drawScreen(){var c=document.getElementById('scr');if(!c)return;"
    "fetch('/screen?t='+Date.now()).then(function(r){return r.arrayBuffer();}).then(function(ab){"
    "var dv=new DataView(ab),n=160*160,ctx=c.getContext('2d'),img=ctx.createImageData(160,160);"
    "for(var i=0;i<n;i++){var v=dv.getUint16(i*2,true),r=(v>>11)&31,g=(v>>5)&63,b=v&31,o=i*4;"
    "img.data[o]=(r<<3)|(r>>2);img.data[o+1]=(g<<2)|(g>>4);img.data[o+2]=(b<<3)|(b>>2);img.data[o+3]=255;}"
    "ctx.putImageData(img,0,0);var el=document.getElementById('scrInfo');if(el)el.textContent=new Date().toLocaleTimeString();"
    "}).catch(function(e){var el=document.getElementById('scrInfo');if(el)el.textContent='Fehler';});}"
    "setInterval(function(){var a=document.getElementById('scrAuto'),t=document.getElementById('t-scr');"
    "if(a&&a.checked&&t&&t.className.indexOf('on')>=0)drawScreen();},1500);"
    "function updLive(){fetch('/live?t='+Date.now()).then(function(r){return r.json();}).then(function(d){"
    "function S(id,v){var e=document.getElementById(id);if(e)e.textContent=v;}"
    "S('lv_src',d.src);S('lv_fresh',d.fresh?'LIVE':'keine Daten');"
    "S('lv_lam',d.lambda==null?'---':d.lambda.toFixed(2));S('lv_rpm',d.rpm);S('lv_adv',d.adv.toFixed(1));"
    "S('lv_map',d.map);S('lv_temp',d.temp);S('lv_volt',d.volt.toFixed(1));S('lv_amp',d.amp.toFixed(1));"
    "S('lv_http',d.httpRx);S('lv_can',d.canRx);S('lv_ble',d.bleRx);"
    "S('lv_alarm',d.alarm?('\\u26a0 ALARM: '+d.alarm):'');}).catch(function(){});}"
    "setInterval(function(){var t=document.getElementById('t-live');"
    "if(t&&t.className.indexOf('on')>=0)updLive();},1000);"
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
  if (webServer.hasArg("lstyle")) {             // Lambda-Stil (0=Gauge, 1=Verlauf/Kurve) - wie Motor-Stil fernsetzbar
    saveLambdaStyle(webServer.arg("lstyle").toInt() ? 1 : 0);
    g_redrawPage = true;
    Serial.printf("Web: Lambda-Stil = %u (%s)\n", g_lambdaStyle, g_lambdaStyle ? "Verlauf" : "Gauge");
  }
  if (webServer.hasArg("imunull")) {
    saveImuNull();
    g_redrawPage = true;
  }
  if (webServer.hasArg("devsave")) {                 // Dev-Tab: Auto-Cockpit + Test-Hub
    bool ac = webServer.hasArg("acock");
    bool th = webServer.hasArg("thub");
    Preferences p; p.begin("clock", false);
    if (ac != g_autoCockpit) { g_autoCockpit = ac; p.putBool("auto_cock", g_autoCockpit); }
    if (webServer.hasArg("acockrpm")) {
      int v = webServer.arg("acockrpm").toInt();
      if (v >= 100 && v <= 5000) { g_autoCockpitRpm = v; p.putInt("acock_rpm", g_autoCockpitRpm); }
    }
    if (th != g_testHub) {
      g_testHub = th; p.putBool("thub", g_testHub);
      if (!th && g_lastSrc && strcmp(g_lastSrc, "TEST") == 0) g_lastSrc = "HTTP";  // Label sofort zuruecksetzen
    }
    if (webServer.hasArg("thubip")) {
      String v = webServer.arg("thubip"); v.trim();
      if (v.length() > 0 && v.length() < sizeof(g_testHubIp)) {
        snprintf(g_testHubIp, sizeof(g_testHubIp), "%s", v.c_str());
        p.putString("thub_ip", g_testHubIp);
      }
    }
    { // Alarme + Tacho-Konfig
      bool al = webServer.hasArg("alerts");
      if (al != g_alertsOn) { g_alertsOn = al; p.putBool("alerts", al);
                              if (!al) { g_alertMask = 0; g_alertText[0] = 0; } }
      if (webServer.hasArg("allmin")) { float v = webServer.arg("allmin").toFloat();
        if (v >= 0.5f && v <= 1.5f) { g_alLamMin = v; p.putFloat("al_lmin", v); } }
      if (webServer.hasArg("allmax")) { float v = webServer.arg("allmax").toFloat();
        if (v >= 0.5f && v <= 1.5f && v > g_alLamMin) { g_alLamMax = v; p.putFloat("al_lmax", v); } }
      if (webServer.hasArg("alrpm"))  { int v = webServer.arg("alrpm").toInt();
        if (v >= 1000 && v <= 9000) { g_alRpmMax = v; p.putInt("al_rpm", v); } }
      if (webServer.hasArg("altemp")) { int v = webServer.arg("altemp").toInt();
        if (v >= 40 && v <= 150) { g_alTempMax = v; p.putInt("al_temp", v); } }
      if (webServer.hasArg("alvolt")) { float v = webServer.arg("alvolt").toFloat();
        if (v >= 8.0f && v <= 14.0f) { g_alVoltMin = v; p.putFloat("al_volt", v); } }
      if (webServer.hasArg("scrpm"))  { int v = webServer.arg("scrpm").toInt();
        if (v >= 3000 && v <= 12000) { g_rpmScaleMax = v; p.putInt("sc_rpm", v); } }
      if (webServer.hasArg("rlrpm"))  { int v = webServer.arg("rlrpm").toInt();
        if (v >= 1000 && v <= g_rpmScaleMax) { g_rpmRedline = v; p.putInt("rl_rpm", v); } }
      g_redrawPage = true;                       // Tacho/Band sofort neu zeichnen
    }
    { // CAN-Konfig: an/aus, Modus, ID, Bitrate - Aenderung re-initialisiert den Treiber
      bool canChanged = false;
      bool con = webServer.hasArg("canon");
      if (con != g_featureCan) { g_featureCan = con; p.putBool("feat_can", con); canChanged = true; }
      if (webServer.hasArg("canmode")) {
        bool listen = webServer.arg("canmode").toInt() == 1;
        if (listen != g_canListenOnly) { g_canListenOnly = listen; p.putBool("can_listen", listen); canChanged = true; }
      }
      if (webServer.hasArg("canid")) {
        uint32_t id = (uint32_t)strtoul(webServer.arg("canid").c_str(), nullptr, 16);
        if (id >= 1 && id <= 0x7FF && (uint16_t)id != g_canId) {
          g_canId = (uint16_t)id; p.putUShort("can_id", g_canId); canChanged = true;
        }
      }
      if (webServer.hasArg("cankbps")) {
        int k = webServer.arg("cankbps").toInt();
        if ((k == 125 || k == 250 || k == 500 || k == 1000) && (uint16_t)k != g_canKbps) {
          g_canKbps = (uint16_t)k; p.putUShort("can_kbps", g_canKbps); canChanged = true;
        }
      }
      if (canChanged) {
        if (g_featureCan) setupCockpitCan();
        else { twai_stop(); twai_driver_uninstall(); g_canReady = false; }
        Serial.printf("Web: CAN %s id=0x%03X %uk %s\n", g_featureCan ? "an" : "aus",
                      g_canId, g_canKbps, g_canListenOnly ? "listen" : "normal");
      }
    }
    p.end();
    Serial.printf("Web/Dev: Auto-Cockpit %s (Schwelle %d), Test-Hub %s (%s), Alarme %s "
                  "(lam %.2f-%.2f rpm>%d temp>%d volt<%.1f), Tacho rot %d / max %d\n",
                  g_autoCockpit ? "an" : "aus", g_autoCockpitRpm,
                  g_testHub ? "AN" : "aus", g_testHubIp, g_alertsOn ? "an" : "aus",
                  g_alLamMin, g_alLamMax, g_alRpmMax, g_alTempMax, g_alVoltMin,
                  g_rpmRedline, g_rpmScaleMax);
  }
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

static void handleWebFeatures() {
  if (!webServer.hasArg("fsave")) {        // Schutz: nackter /features-Aufruf (Crawler/Tippfehler)
    webServer.sendHeader("Location", "/"); // wuerde sonst ALLE Features abschalten
    webServer.send(303);
    return;
  }
  const bool wifi   = webServer.hasArg("wifi");
  const bool ble    = webServer.hasArg("ble");
  const bool buzzer = webServer.hasArg("buzzer");
  const bool f123   = webServer.hasArg("f123");
  const bool wauto  = webServer.hasArg("wauto");
  saveFeatures(wifi, ble, buzzer);
  if (f123 != g_feature123) saveFeature123(f123);
  // CAN wohnt im Dev-Tab (devsave) - hier NICHT anfassen, sonst schaltet
  // jedes Speichern des Funktionen-Formulars CAN mangels Checkbox wieder aus.
  if (wauto != g_wifiAuto) {
    g_wifiAuto = wauto;
    Preferences p; p.begin("clock", false); p.putBool("wifi_auto", g_wifiAuto); p.end();
  }
  Serial.printf("Web: Funktionen wifi=%s ble=%s buzzer=%s 123=%s can=%s\n",
                g_featureWifi ? "on" : "off", g_featureBle ? "on" : "off",
                g_featureBuzzer ? "on" : "off", g_feature123 ? "on" : "off", g_featureCan ? "on" : "off");
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

static void handleWebPage() {
  if (webServer.hasArg("p")) {
    int page = webServer.arg("p").toInt();
    // Nur echte Seiten: 8/9 existieren nicht (Display friert ein), 10 (Tastatur)
    // braucht openKeyboard-Kontext, 12 (Scan) einen vorherigen Scan-Lauf.
    if ((page >= 0 && page <= 7) || page == 11 || page == 13) {
      currentPage  = static_cast<uint8_t>(page);
      g_redrawPage = true;
      Serial.printf("Web: page=%u\n", currentPage);
    }
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
      // leeres Passwort = bestehendes behalten (wird im Formular nie zurueckgegeben).
      // Benannte Strings: arg() liefert Temporaries, deren c_str() nach der Anweisung
      // in freigegebenen Heap zeigt - das hat korrupte Passwoerter in NVS+SD geschrieben.
      String ssid  = webServer.arg("ssid");
      String pass  = webServer.arg("pass").length() ? webServer.arg("pass") : String(g_wprof[idx].pass);
      String hubip = webServer.hasArg("hubip") ? webServer.arg("hubip") : String(g_wprof[idx].hubip);
      saveWprof(idx, ssid.c_str(), pass.c_str(), hubip.c_str());
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
      char logMsg[48];
      if (Update.end(true)) {
        Serial.printf("OTA: Erfolg, %u Bytes\n", (unsigned)g_otaRxBytes);
        snprintf(logMsg, sizeof(logMsg), "OTA-Web OK %u Bytes", (unsigned)g_otaRxBytes);
      } else {
        Update.printError(Serial);
        snprintf(logMsg, sizeof(logMsg), "OTA-Web FEHLER");
      }
      sdLog(logMsg);
    }
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    Update.abort(); g_otaBusy = false; hal_pause_for_ota(false);
    Serial.println("OTA: abgebrochen");
    sdLog("OTA-Web abgebrochen");
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

static void handleWebSdWifi() {
  if (g_sdMounted && webServer.hasArg("txt")) {
    File w = SD_MMC.open("/wifi.txt", FILE_WRITE);
    if (w) { w.print(webServer.arg("txt")); w.close(); }
    g_sdWifiLoaded = sdApplyWifiTxt();                  // sofort in die Profile uebernehmen
    Serial.printf("Web: /wifi.txt gespeichert -> %d Profil(e)\n", g_sdWifiLoaded);
  }
  webServer.sendHeader("Location", "/");
  webServer.send(303, "text/plain", "ok");
}

// Display-Vorschau: Framebuffer (480x480 RGB565) auf 160x160 heruntersampeln (nearest
// neighbor) und als Roh-Binaerdaten (LE uint16) senden. Der Browser dekodiert RGB565 auf
// ein Canvas. Bewusst klein (~50KB): das Senden blockiert den Loop kurz -> Schreibtisch-Tool,
// Auto-Refresh laeuft nur solange der Vorschau-Tab offen ist.
static void handleWebScreen() {
  uint16_t* fb = hal_fb();
  if (!fb) { webServer.send(503, "text/plain", "no fb"); return; }
  const int OUT = 160, STEP = 3;                 // 480 / 160 = 3
  const size_t len = (size_t)OUT * OUT * 2;
  uint16_t* buf = (uint16_t*)ps_malloc(len);
  if (!buf) { webServer.send(503, "text/plain", "no mem"); return; }
  for (int oy = 0; oy < OUT; oy++) {
    const uint16_t* srow = fb + (size_t)(oy * STEP) * 480;
    uint16_t* drow = buf + (size_t)oy * OUT;
    for (int ox = 0; ox < OUT; ox++) drow[ox] = srow[ox * STEP];
  }
  webServer.setContentLength(len);
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(200, "application/octet-stream", "");
  webServer.sendContent((const char*)buf, len);
  free(buf);
}

// Live-Cockpitwerte als JSON fuer den Live-Tab (pollt im Sekundentakt statt Reload).
// Klein (~200 B) -> blockiert den Loop kaum, laeuft nur solange der Live-Tab offen ist.
static void handleWebLive() {
  const bool anyFresh = bleFresh() || canFresh() || httpFresh() || tune123Fresh();
  String j = "{";
  j += "\"src\":\"" + String(anyFresh ? g_lastSrc : "---") + "\",";
  j += "\"fresh\":" + String(anyFresh ? "true" : "false") + ",";
  j += "\"lambda\":" + (g_lambdaValid ? String(g_lambda, 2) : String("null")) + ",";
  j += "\"rpm\":" + String((int)g_rpm) + ",";
  j += "\"adv\":" + String(g_adv, 1) + ",";
  j += "\"map\":" + String((int)g_map) + ",";
  j += "\"temp\":" + String((int)g_g123Temp) + ",";
  j += "\"volt\":" + String(g_g123Volt, 1) + ",";
  j += "\"amp\":" + String(g_g123Coil, 1) + ",";
  j += "\"httpRx\":" + String((unsigned long)g_httpRx) + ",";
  j += "\"canRx\":" + String((unsigned long)g_canRx) + ",";
  j += "\"bleRx\":" + String((unsigned long)g_bleRxCnt) + ",";
  j += "\"alarm\":\"" + String(g_alertMask ? g_alertText : "") + "\"}";
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(200, "application/json", j);
}

static void startWebServer() {
  webServer.on("/",        handleWebRoot);
  webServer.on("/screen",  handleWebScreen);
  webServer.on("/live",    handleWebLive);
  webServer.on("/set",     handleWebSet);
  webServer.on("/features",handleWebFeatures);
  webServer.on("/page",    handleWebPage);
  webServer.on("/wifi",    handleWebWifi);
  webServer.on("/update",  HTTP_POST, handleOtaDone, handleOtaUpload);
  webServer.on("/sd", []() {
    char b[96];
    snprintf(b, sizeof(b), "mounted=%d type=%s sizeMB=%lu wifitxt=%d", g_sdMounted ? 1 : 0, g_sdType, (unsigned long)g_sdSizeMB, g_sdWifiLoaded);
    webServer.send(200, "text/plain", b);
  });
  webServer.on("/sdwifi", HTTP_POST, handleWebSdWifi);
  webServer.on("/log", []() {              // ?d=JJJJMMTT waehlt einen Tag, sonst heute
    if (!g_sdMounted) { webServer.send(503, "text/plain", "SD nicht gemountet"); return; }
    struct tm now = {};
    char path[24];
    if (webServer.hasArg("d") && webServer.arg("d").length() == 8) {
      snprintf(path, sizeof(path), "/log/%s.txt", webServer.arg("d").c_str());
    } else {
      readClockTime(&now);
      snprintf(path, sizeof(path), "/log/%04d%02d%02d.txt", now.tm_year + 1900, now.tm_mon + 1, now.tm_mday);
    }
    File f = SD_MMC.open(path, FILE_READ);
    if (!f) { webServer.send(404, "text/plain", String(path) + " nicht gefunden"); return; }
    webServer.streamFile(f, "text/plain");
    f.close();
  });
  webServer.on("/imu", []() {              // IMU-Werte als JSON - fuer Hub-Polling/Logging
    char j[160];
    if (!g_imuPresent) {
      snprintf(j, sizeof(j), "{\"present\":false}");
    } else {
      qmi8658Read();
      snprintf(j, sizeof(j),
        "{\"present\":true,\"pitch\":%.2f,\"roll\":%.2f,\"gforce\":%.3f,\"shake\":%s}",
        g_imuPitch - g_imuOffPitch, g_imuRoll - g_imuOffRoll, g_imuGForce,
        qmi8658ShakeDetected(1.5f) ? "true" : "false");
    }
    webServer.send(200, "application/json", j);
  });
  webServer.on("/version", []() {          // Maschinenlesbarer Stand: Build/Git/Features (JSON)
    String j = F("{\"fw\":\"" FW_BUILD "\",\"git\":\"" GIT_REV "\",\"github\":\"" GITHUB_URL "\"");
    j += ",\"up_s\":" + String(millis() / 1000);
    j += ",\"ip\":\"" + String(g_ipStr) + "\",\"profil\":\"" + String(WPROF_LABELS[g_wifiProfile]) + "\"";
    j += ",\"features\":{\"wifi\":" + String(g_featureWifi ? 1 : 0) +
         ",\"wifi_auto\":" + String(g_wifiAuto ? 1 : 0) +
         ",\"ble\":" + String(g_featureBle ? 1 : 0) +
         ",\"buzzer\":" + String(g_featureBuzzer ? 1 : 0) +
         ",\"123\":" + String(g_feature123 ? 1 : 0) +
         ",\"can\":" + String(g_featureCan ? 1 : 0) +
         ",\"can_listen\":" + String(g_canListenOnly ? 1 : 0) +
         ",\"can_id\":\"0x" + String(g_canId, HEX) + "\",\"can_kbps\":" + String(g_canKbps) +
         ",\"auto_cockpit\":" + String(g_autoCockpit ? 1 : 0) +
         ",\"auto_cockpit_rpm\":" + String(g_autoCockpitRpm) +
         ",\"test_hub\":" + String(g_testHub ? 1 : 0) +
         ",\"test_hub_ip\":\"" + String(g_testHubIp) + "\"}";
    j += ",\"alarme\":{\"an\":" + String(g_alertsOn ? 1 : 0) +
         ",\"aktiv\":\"" + String(g_alertMask ? g_alertText : "") + "\"" +
         ",\"lam_min\":" + String(g_alLamMin, 2) + ",\"lam_max\":" + String(g_alLamMax, 2) +
         ",\"rpm_max\":" + String(g_alRpmMax) + ",\"temp_max\":" + String(g_alTempMax) +
         ",\"volt_min\":" + String(g_alVoltMin, 1) +
         ",\"tacho_rot\":" + String(g_rpmRedline) + ",\"tacho_max\":" + String(g_rpmScaleMax) + "}";
    j += ",\"anzeige\":{\"motor_stil\":" + String(g_motorStyle) +
         ",\"lambda_stil\":" + String(g_lambdaStyle) +
         ",\"groesse_pct\":" + String(g_dialScalePct) +
         ",\"rotation\":" + String(g_rotationDeg) + "}";
    j += ",\"sd\":{\"mounted\":" + String(g_sdMounted ? 1 : 0) +
         ",\"type\":\"" + String(g_sdType) + "\",\"mb\":" + String((unsigned long)g_sdSizeMB) +
         ",\"wifitxt\":" + String(g_sdWifiLoaded) + ",\"ota\":\"" + String(g_sdOtaResult) + "\"}";
    j += ",\"daten\":{\"quelle\":\"" + String(g_lastSrc) + "\",\"httpRx\":" + String((unsigned long)g_httpRx) +
         ",\"canRx\":" + String((unsigned long)g_canRx) + "}";
    j += ",\"heap\":" + String(ESP.getFreeHeap());
    boardBattRead();
    j += ",\"board_batt\":{\"present\":" + String(g_boardBattPresent ? 1 : 0) +
         ",\"volt\":" + String(g_boardBattVolt, 2) + ",\"pct\":" + String(boardBattPct()) + "}}";
    webServer.send(200, "application/json", j);
  });
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

static void handleSetupLongPress(uint16_t y, uint32_t durMs, bool isLong) {
  Serial.printf("setup tap y=%u dur=%lu long=%d\n", y, (unsigned long)durMs, isLong);

  if (y >= 346) {                       // unten -> zurueck ins Menue
    currentPage = 1;
    drawMenuOverview();
    Serial.println("setup tap: menu");
    return;
  }
  // Zeile aus derselben Tabelle bestimmen, mit der drawSetupPage zeichnet
  int row = -1;
  for (int i = 0; i < 8; i++)
    if ((int)y >= SETUP_ROW_Y[i] - 16 && (int)y < SETUP_ROW_Y[i] + 16) { row = i; break; }

  switch (row) {
    case 0:                             // UHR -> Justage-Seite (Groesse/Rotation fein)
    case 2:                             // ROT -> Justage-Seite
      currentPage = 7;
      drawAdjustPage();
      Serial.println("setup tap: -> Justage");
      break;
    case 1: {                           // HELL
      int next = (g_brightnessPct < 63) ? 75 : (g_brightnessPct < 88 ? 100 : 50);
      saveBrightness(next);
      drawSetupPage();
      Serial.printf("setup tap: brightness=%d%%\n", g_brightnessPct);
      break;
    }
    case 3:                             // WIFI -> WLAN-Seite (Scan / Tippen / Profil)
      currentPage = 11;
      drawWlanPage();
      Serial.println("setup tap: -> WLAN-Seite");
      break;
    case 4:                             // BLE an/aus
      saveFeatures(g_featureWifi, !g_featureBle, g_featureBuzzer);
      drawSetupPage();
      Serial.printf("setup tap: ble=%s\n", g_featureBle ? "on" : "off");
      break;
    case 5:                             // CAN -> CAN-Seite (Status + Schalter)
      currentPage = 13;
      drawCanPage();
      Serial.println("setup tap: -> CAN-Seite");
      break;
    case 6:                             // BUZZER an/aus
      saveFeatures(g_featureWifi, g_featureBle, !g_featureBuzzer);
      drawSetupPage();
      Serial.printf("setup tap: buzzer=%s\n", g_featureBuzzer ? "on" : "off");
      break;
    case 7:                             // IMU 0 -> Einbaulage nullen
      saveImuNull();
      drawSetupPage();
      Serial.println("setup tap: IMU NULL");
      break;
    default:
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
  sdSyncWifiTxt();          // SD-wifi.txt mitziehen, sonst gewinnt die alte Datei beim Boot
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
  g_manualWifiUntil = millis() + 20000;   // 20 s Schonfrist, bevor Auto-Fallback eingreift
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

// ===== Micro-SD: WLAN-Profile aus /wifi.txt laden (oder Vorlage anlegen) =====
// Format je Zeile:  <Slot>=<SSID>|<Passwort>   (0=Heim 1=Hub-AP 2=S24), '#' = Kommentar.
static void loadWifiFromSd() {
  if (!g_sdMounted) return;
  if (!SD_MMC.exists("/wifi.txt")) {                 // noch keine Datei -> Vorlage mit aktuellen Profilen
    sdSyncWifiTxt();                                 // gleicher crashsicherer Pfad (tmp+rename)
    g_sdWifiLoaded = -1;
    Serial.println("SD: /wifi.txt Vorlage angelegt (aktuelle Profile)");
    return;
  }
  g_sdWifiLoaded = sdApplyWifiTxt();                 // vorhandene Datei anwenden
  Serial.printf("SD: %d WLAN-Profil(e) aus /wifi.txt geladen\n", g_sdWifiLoaded);
}

// ===== Micro-SD mounten + wifi.txt verarbeiten =====
static void setupSdCard() {
  hal_free_lcd_spi();                                   // Pins 1/2 vom LCD-SPI freigeben
  hal_sd_d3_high();                                     // EXIO4 (SD_D3) Output + HIGH
  if (!SD_MMC.setPins(2, 1, 42, -1, -1, -1)) { Serial.println("SD: setPins FAIL"); return; }
  if (!SD_MMC.begin("/sdcard", true)) {                 // true = 1-bit-Modus
    Serial.println("SD: begin FAIL (Karte drin? Kontakt?)"); return;
  }
  uint8_t ct = SD_MMC.cardType();
  if (ct == CARD_NONE) { Serial.println("SD: keine Karte"); return; }
  g_sdType   = (ct == CARD_MMC) ? "MMC" : (ct == CARD_SD) ? "SDSC" : (ct == CARD_SDHC) ? "SDHC" : "?";
  g_sdSizeMB = (uint32_t)(SD_MMC.cardSize() / (1024ULL * 1024ULL));
  g_sdMounted = true;
  Serial.printf("SD: OK %s %luMB (frei %lluMB)\n", g_sdType, (unsigned long)g_sdSizeMB,
                (unsigned long long)((SD_MMC.totalBytes() - SD_MMC.usedBytes()) / (1024ULL * 1024ULL)));
  if (!SD_MMC.exists("/log")) SD_MMC.mkdir("/log");
  loadWifiFromSd();
  char bootMsg[64];
  snprintf(bootMsg, sizeof(bootMsg), "BOOT fw=%s git=%s reset=%d", FW_BUILD, GIT_REV, (int)esp_reset_reason());
  sdLog(bootMsg);
}

// ===== OTA-Recovery von SD: /update.bin beim Boot flashen (ohne WLAN/COM13) =====
// Robust: nur wenn Datei plausibel gross ist; Datei wird IMMER umbenannt (kein Boot-Loop).
static void sdCheckFirmwareUpdate() {
  if (!g_sdMounted) return;
  // Erst /update.bin -> /update.run umbenennen, DANN flashen: schlaegt schon das
  // Rename fehl (FAT krank), wird gar nicht geflasht - kein Endlos-Flash-Loop.
  // Ein liegengebliebenes /update.run (Zuendung aus mitten im Flash) wird beim
  // naechsten Boot erneut geflasht - die App-Partition war dann eh halbfertig.
  if (SD_MMC.exists("/update.bin")) {
    SD_MMC.remove("/update.run");
    if (!SD_MMC.rename("/update.bin", "/update.run")) {
      Serial.println("SD-OTA: rename update.bin->update.run FEHLER -> abgebrochen");
      g_sdOtaResult = "FEHLER";
      return;
    }
  }
  if (!SD_MMC.exists("/update.run")) return;
  File f = SD_MMC.open("/update.run", FILE_READ);
  if (!f) return;
  size_t sz = f.size();
  if (sz < 100000) { f.close(); SD_MMC.remove("/update.bad"); SD_MMC.rename("/update.run", "/update.bad");
                     Serial.println("SD-OTA: update.bin zu klein -> .bad"); g_sdOtaResult = "FEHLER"; return; }
  Serial.printf("SD-OTA: update %u Bytes -> flashe...\n", (unsigned)sz);
  hal_pause_for_ota(true);                              // RGB-Panel anhalten (Flash-Cache)
  bool ok = false;
  if (Update.begin(sz)) {
    size_t w = Update.writeStream(f);
    ok = (w == sz) && Update.end(true);
  }
  f.close();
  SD_MMC.remove(ok ? "/update.done" : "/update.bad");   // alt weg
  if (!SD_MMC.rename("/update.run", ok ? "/update.done" : "/update.bad"))
    SD_MMC.remove("/update.run");                       // Rename kaputt -> loeschen statt Flash-Loop
  g_sdOtaResult = ok ? "OK" : "FEHLER";
  Serial.printf("SD-OTA: %s\n", ok ? "OK -> Neustart" : "FEHLER");
  sdLog(ok ? "OTA-SD OK" : "OTA-SD FEHLER");
  if (ok) { delay(300); ESP.restart(); }
  hal_pause_for_ota(false);                              // bei Fehler Panel zurueck
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
  // Nur wenn bewusst aktiviert - sonst zeigt der Fallback-Text ueberall "CAN WARTE",
  // obwohl gar kein Transceiver/Bus verbunden ist.
  if (g_featureCan) setupCockpitCan();

  setupSdCard();                 // Micro-SD mounten - gibt LCD-SPI frei, liest wifi.txt
  ensureHubProfile();            // NACH SD-Import: Live-Hub-Profil muss immer waehlbar sein
  sdCheckFirmwareUpdate();       // Recovery: /update.bin von SD flashen (falls vorhanden)

  initTimeSource();
  hal_backlight(true);
  drawVdoClock();
  Serial.println("VDO clock drawn.");
}

// Auto-Cockpit: Motor laeuft (Drehzahl > AUTO_COCKPIT_RPM) -> Motor-Seite; Motor aus -> Uhr.
// Nur von Uhr/Menue automatisch REIN, und nur ZURUECK wenn wir selbst rein sind (stoert dich
// nicht, wenn du z.B. auf Lambda/Setup navigiert hast). Quelle egal (Hub/CAN/123).
static void autoCockpitTick() {
  if (!g_autoCockpit) return;
  static bool     autoEntered = false;   // wir sind automatisch auf der Motor-Seite gelandet
  static bool     jumped      = false;   // pro MOTORSTART nur EIN Auto-Sprung
  static uint32_t idleSince   = 0;
  bool fresh = httpFresh() || canFresh() || bleFresh() || tune123Fresh();
  float rpm  = fresh ? g_rpm : 0.0f;
  if (rpm > g_autoCockpitRpm) {
    idleSince = 0;
    // Nur beim Motorstart einmal springen. Danach frei navigieren: wer waehrend
    // der Fahrt zur Uhr/Setup wechselt, BLEIBT dort (kein Zurueck-Reissen mehr).
    if (!jumped) {
      jumped = true;
      if (currentPage == 0 || currentPage == 1) {
        currentPage = 2; drawMotorPage(); autoEntered = true;
        Serial.printf("Auto-Cockpit: Motor an (%d) -> Motor-Seite\n", (int)rpm);
      }
    }
    if (currentPage != 2) autoEntered = false;          // Nutzer ist selbst weg-navigiert
  } else {
    if (idleSince == 0) { idleSince = millis(); return; }
    if (millis() - idleSince > 5000) {                  // Motor 5 s aus
      if (autoEntered && currentPage == 2) {            // nur zurueck, wenn WIR rein sind
        currentPage = 0; drawVdoClock();
        Serial.println("Auto-Cockpit: Motor aus -> Uhr");
      }
      autoEntered = false;
      jumped = false;                                   // naechster Motorstart darf wieder springen
    }
  }
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
        handleSetupLongPress(touchLastY, nowMs - touchStartMs, true);
      } else if (currentPage == 2) {
        // MOTOR: langer Druck in die Mitte -> Anzeige-Stil wechseln (DIGITAL/VDO/123TUNE+)
        const int dx = (int)touchLastX - 240, dy = (int)touchLastY - 240;
        if (dx * dx + dy * dy <= 95 * 95) {
          touchLongHandled = true;
          lastTouch = nowMs;
          saveMotorStyle((g_motorStyle + 1) % MOTOR_STYLE_COUNT);
          drawMotorPage();
          Serial.printf("motor long-press -> Stil %u (%s)\n", g_motorStyle, MOTOR_STYLE_NAMES[g_motorStyle]);
        } else if (g_tripValid && touchLastY >= 370 && nowMs - touchStartMs >= 1000) {
          // Trip-Reset zweistufig (alle Stile): 1. Langdruck unten scharf machen,
          // 2. Langdruck innerhalb 5 s bestaetigt -> Teilstrecke am Hub nullen
          touchLongHandled = true;
          lastTouch = nowMs;
          if (millis() < g_tripArmUntil) {
            g_tripArmUntil = 0;
            tripResetOnHub();
          } else {
            g_tripArmUntil = millis() + 5000;
            Serial.println("Trip-Reset scharf: unten nochmal 1 s halten");
          }
          drawMotorPage();
        }
      } else if (currentPage == 3) {
        // LAMBDA: langer Druck in die Mitte -> Gauge <-> Verlauf umschalten
        const int dx = (int)touchLastX - 240, dy = (int)touchLastY - 240;
        if (dx * dx + dy * dy <= 95 * 95) {
          touchLongHandled = true;
          lastTouch = nowMs;
          saveLambdaStyle(g_lambdaStyle ^ 1);
          drawLambdaPage();
          Serial.printf("lambda long-press -> Stil %u (%s)\n", g_lambdaStyle, g_lambdaStyle ? "Verlauf" : "Gauge");
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
        handleSetupLongPress(tapY, durMs, false);
      } else if (currentPage == 7) {
        handleAdjustTap(tapX, tapY);    // Justage: Groesse/Rotation +/-
      } else if (currentPage == 10) {
        handleKbTap(tapX, tapY);        // On-Screen-Tastatur
      } else if (currentPage == 11) {
        handleWlanTap(tapX, tapY);      // WLAN-Seite (Scan/Tippen/Profil)
      } else if (currentPage == 12) {
        handleScanTap(tapX, tapY);      // Scan-Ergebnis (SSID antippen)
      } else if (currentPage == 13) {
        handleCanTap(tapX, tapY);       // CAN-Seite (aktiv/Modus schalten)
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
  g_uiTouchActive = touchActive;   // HTTP-Poll pausiert, solange beruehrt (kein verschluckter Tap)
#endif

  // Serial commands
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (serialLine.length() > 0) {
        String cmd = serialLine; serialLine = "";
        cmd.trim();
        String rawCmd = cmd;            // Original-Schreibweise behalten (SSID/Passwort case-sensitiv!)
        cmd.toLowerCase();
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
        else if (cmd == "scan")      { runWlanScan(); }   // Test: einmaliger WLAN-Scan + Ergebnisseite
        else if (cmd == "thub:show") { Serial.printf("Test-Hub: %s ip=%s\n", g_testHub ? "AN" : "aus", g_testHubIp); }
        else if (cmd == "thub:on" || cmd == "thub:off") {
          g_testHub = (cmd == "thub:on");
          Preferences pt; pt.begin("clock", false); pt.putBool("thub", g_testHub); pt.end();
          if (!g_testHub && g_lastSrc && !strcmp(g_lastSrc, "TEST")) g_lastSrc = "HTTP";
          Serial.printf("Test-Hub = %s (ip=%s)\n", g_testHub ? "AN" : "aus", g_testHubIp);
        }
        else if (cmd.startsWith("thub:ip ")) {
          String v = rawCmd.substring(8); v.trim();
          if (v.length() && v.length() < sizeof(g_testHubIp)) {
            snprintf(g_testHubIp, sizeof(g_testHubIp), "%s", v.c_str());
            Preferences pt; pt.begin("clock", false); pt.putString("thub_ip", g_testHubIp); pt.end();
            Serial.printf("Test-Hub-IP = %s\n", g_testHubIp);
          }
        }
        else if (cmd == "wifi:show") {           // alle Profile dumpen (Debug)
          for (int i = 0; i < WPROF_COUNT; i++)
            Serial.printf("Profil %d (%s): ssid='%s' pass='%s' hubip='%s'\n",
                          i, WPROF_LABELS[i], g_wprof[i].ssid, g_wprof[i].pass, g_wprof[i].hubip);
        }
        else if (cmd.startsWith("wifi:set ")) {  // wifi:set <slot> <SSID>|<Passwort> (case-sensitiv!)
          String rest = rawCmd.substring(9);
          int sp = rest.indexOf(' ');
          if (sp > 0) {
            int slot = rest.substring(0, sp).toInt();
            String body = rest.substring(sp + 1);
            int bar = body.indexOf('|');
            if (slot >= 0 && slot < WPROF_COUNT && bar >= 0) {
              String ss = body.substring(0, bar), pw = body.substring(bar + 1);
              saveWprof((uint8_t)slot, ss.c_str(), pw.c_str(), g_wprof[slot].hubip);
              selectWprof((uint8_t)slot);
              Serial.printf("wifi:set Profil %d ssid='%s' pass-len=%u -> aktiv+verbinde\n",
                            slot, ss.c_str(), (unsigned)pw.length());
            } else Serial.println("wifi:set: nutze 'wifi:set 0 SSID|Passwort'");
          }
        }
        else if (cmd == "wifi:off")  { saveFeatures(false, g_featureBle, g_featureBuzzer); g_redrawPage = true; }
        else if (cmd == "rot:+") { saveRotation(g_rotationDeg + 1); g_redrawPage = true; }
        else if (cmd == "rot:-") { saveRotation(g_rotationDeg - 1); g_redrawPage = true; }
        else if (cmd.startsWith("rot:")) { saveRotation(cmd.substring(4).toInt()); g_redrawPage = true; }
        else if (cmd == "clock")   { currentPage = 0; drawVdoClock(); }
        else if (cmd == "reboot")  { Serial.println("Reboot..."); delay(50); ESP.restart(); }
        else if (cmd == "can:on" || cmd == "can:off") {
          g_featureCan = (cmd == "can:on");
          Preferences pc; pc.begin("clock", false); pc.putBool("feat_can", g_featureCan); pc.end();
          if (g_featureCan) setupCockpitCan();
          else { twai_stop(); twai_driver_uninstall(); g_canReady = false; }
          Serial.printf("CAN-Feature = %s\n", g_featureCan ? "an" : "aus");
          sdLog(g_featureCan ? "CAN an" : "CAN aus");
        }
        else if (cmd == "can:normal" || cmd == "can:listen") {
          g_canListenOnly = (cmd == "can:listen");
          Preferences pc; pc.begin("clock", false); pc.putBool("can_listen", g_canListenOnly); pc.end();
          setupCockpitCan();
          Serial.printf("CAN-Mode = %s\n", g_canListenOnly ? "listen-only" : "NORMAL (ACK)");
        }
        else if (cmd == "trip:reset") { Serial.println(tripResetOnHub() ? "Trip genullt" : "Trip-Reset FEHLER"); }
        else if (cmd == "batt") { boardBattRead();
          Serial.printf("Board-Akku: %s %.2fV (~%d%%)\n",
                        g_boardBattPresent ? "vorhanden" : "kein Akku/nur USB",
                        g_boardBattVolt, boardBattPct()); }
        else if (cmd == "can:test"){ runCanTest(); }
        else if (cmd == "can:ping"){ runCanPing(); }
        else if (cmd == "can:rx")  { Serial.printf("CAN: ready=%d rx=%lu ignored=%lu age=%lums src=%s\n",
                                       g_canReady ? 1 : 0, (unsigned long)g_canRx, (unsigned long)g_canIgnored,
                                       g_canLastRxMs ? (unsigned long)(millis() - g_canLastRxMs) : 0UL, g_lastSrc); }
        else if (cmd == "motor")   { currentPage = 2; drawMotorPage(); }
        else if (cmd.startsWith("style:")) { saveMotorStyle(cmd.substring(6).toInt());
                                             Serial.printf("Motor-Stil = %u (%s)\n", g_motorStyle, MOTOR_STYLE_NAMES[g_motorStyle]);
                                             if (currentPage == 2) drawMotorPage(); }
        else if (cmd.startsWith("lambda:")) { saveLambdaStyle(cmd.substring(7).toInt() ? 1 : 0);
                                             Serial.printf("Lambda-Stil = %u (%s)\n", g_lambdaStyle, g_lambdaStyle ? "Verlauf" : "Gauge");
                                             if (currentPage == 3) drawLambdaPage(); }
        else { Serial.println("Commands: ble:on|off | 123:on|off | buzzer:on|off | wifi:next|off | wauto:on|off | rot:+|-|NN | clock | motor | style:0..3 | trip:reset | batt | imu:null | can:on|off | can:test | can:rx | can:normal|listen"); }
      }
    } else if (serialLine.length() < 64) {
      serialLine += c;
    }
  }

  // WLAN-Auto-Fallback (Hub-AP > Heim) - verbindet zum verfuegbaren Netz
  wifiAutoTick();

  // WLAN-Seite live halten (WPS-Status / IP aktualisieren)
  { static uint32_t wlanRedrawAt = 0;
    if (currentPage == 11 && millis() > wlanRedrawAt) { wlanRedrawAt = millis() + 1200; drawWlanPage(); } }

  // CAN-Seite live halten (RX-Zaehler / Fehler / LIVE-Status)
  { static uint32_t canRedrawAt = 0;
    if (currentPage == 13 && millis() > canRedrawAt) { canRedrawAt = millis() + 1000; drawCanPage(); } }

  // Auto-Cockpit: bei laufendem Motor automatisch aufs Motor-Display (von Uhr/Menue)
  autoCockpitTick();

  // Alarme pruefen (roter Ring via presentFrame) + Buzzer-Piep
  alertTick();
  alertBuzzTick();

  // Lambda-Verlauf: immer sampeln (Historie da, egal welche Seite) + Trend-Seite scrollen
  trendSample();
  { static uint32_t trRedrawAt = 0;
    if (currentPage == 3 && g_lambdaStyle == 1 && millis() > trRedrawAt) { trRedrawAt = millis() + 500; drawLambdaPage(); } }

  // Fallback-Setup-AP verwalten (nur AN wenn keine STA-Verbindung)
  manageWifiAp();

  // WiFi/NTP background tick; redraw clock on fresh sync
  if (wifiNtpTick() && currentPage == 0) drawVdoClock();

  // BLE client tick
  if (g_featureBle) bleTick();

  // CAN cockpit tick (0x510)
  cockpitCanTick();
  imuCanTxTick();     // IMU-Werte auf ID+1 senden (nur im NORMAL/ACK-Modus)

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
  // Shake-Buzzer der IMU-Seite darf beim Wegnavigieren nicht gelatcht anbleiben
  // (drawImuPage schaltet ihn nur aus, solange die Seite selbst noch zeichnet)
  static uint8_t buzzGuardPage = 0;
  if (buzzGuardPage == 6 && currentPage != 6 && !g_alertMask) hal_buzzer(false);
  buzzGuardPage = currentPage;

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
    Serial.printf("STAT up=%lus ip=%s wifi=%d prof=%s rssi=%d wr=%s httpRx=%lu canRx=%lu src=%s age=%lums heap=%u\n",
                  (unsigned long)(millis() / 1000), g_ipStr, (int)WiFi.status(),
                  WPROF_LABELS[g_wifiProfile],
                  (WiFi.status() == WL_CONNECTED) ? (int)WiFi.RSSI() : 0, wifiReasonText(g_wifiStaReason),
                  (unsigned long)g_httpRx, (unsigned long)g_canRx,
                  g_lastSrc, g_httpLastRxMs ? (unsigned long)(millis() - g_httpLastRxMs) : 0UL,
                  (unsigned)ESP.getFreeHeap());
  }

  delay(10);
}
