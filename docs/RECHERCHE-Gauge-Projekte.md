# Recherche: vergleichbare Gauge-Projekte, Libraries & Quellen

> Tiefen-Recherche (Web + Foren + GitHub) zu DIY-/Open-Source-Projekten und
> Libraries rund um digitale Auto-Gauges/Cockpits — zugeschnitten auf dieses
> Projekt: runder **Waveshare ESP32-S3-Touch-LCD-2.8C** (480×480, ST7701 RGB),
> der eine **VDO-Quartz-Uhr (VW T2b)** nachbildet und im Bus Live-Daten
> (ESP-NOW vom Spartan3-Hub, 123TUNE+ BLE, WiFi/HTTP) zeigt.
> Stack: PlatformIO/Arduino, LovyanGFX, NimBLE.
>
> Methodik: 5 Such-Winkel, 20 Quellen gefetcht, 86 Aussagen extrahiert,
> 25 davon gegengeprüft (3-Stimmen-Verifikation) → 22 bestätigt, 3 widerlegt.

---

## TL;DR — was du davon brauchst

1. **Genau dein Board ist gelöst:** Es gibt eine **getestete Pin-/Init-Config
   für den ESP32-S3-Touch-LCD-2.8C** (ST7701, 480×480, GT911) in der
   Home-Assistant/ESPHome-Community — als Referenz für Panel-Bring-up.
2. **123TUNE+ ist reverse-engineered:** `iafilius/123Tune-plus-Simulator`
   dokumentiert das BLE-Protokoll **inkl. ESP32-Arduino-Port** — der wichtigste
   Fund für deinen Datenpfad.
3. **Zeiger/Optik:** LVGL `lv_meter` bringt ein fertiges Analoguhr-Beispiel mit
   Bild-Zeigern; alternativ Sprite-Pivot-Rotation (TFT_eSPI/LovyanGFX) nach
   Muster mehrerer MIT-Uhr-Repos. **upir** liefert dazu die besten Tutorials.
4. **Vorbilder:** Gauge.S (MIT), ElektorLabs obd2-dashboard (GPL-3), CarCluster
   (GPL-3), Alfa-Romeo Dual-CAN (CC BY-NC). Kommerziell/closed: Revv Gauge,
   Zada Tech.

---

## 1) Vergleichbare Open-Source-Projekte

| Projekt | Hardware / Display | Datenquelle | Lizenz | Relevanz |
| --- | --- | --- | --- | --- |
| **handmade0octopus/gauge.s-sorek.uk** (Gauge.S) | ESP32-Board (v4.X), eigenes Display | OBD2/CAN, 7× Analog, G-Sensor, GPS, SD-Log | **MIT**, aktiv (v2.32, 2026-03) | ⭐ Nächstes Vorbild: ESP32 + Display + konfig. Live-Daten. Fertiggerät ~£259. |
| **ElektorLabs/obd2-dashboard** | **ESP32-S3** (JC3248W535-Board) | OBD-II via **ELM327** (UART) | GPL-3.0 | Gleiche MCU-Familie, sauberes Dashboard-Vorbild |
| **VaAndCob/ESP32-Bluetooth-OBD2-Gauge** | ESP32 + 2.8" 320×240 TFT (CYD-Klasse) | OBD2 via **ELM327 Bluetooth** | offen (Repo) | DIY-Gauge mit TFT_eSPI, einfacher Einstieg |
| **ClaudeMarais/AlfaRomeoGiulia_DashboardInfo_ESP32-S3** | **XIAO ESP32-S3**, Dual-CAN (intern + MCP2515) | Liest RPM/Boost (OBD2-CAN), sendet ans Kombiinstrument | **CC BY-NC 4.0** (nicht kommerziell!) | Starkes Dual-CAN-Muster, aber Lizenz beachten |
| **r00li/CarCluster** | ESP32-Devboard + MCP2515 | Treibt **echte OEM-Cluster** (VW MQB/PQ, BMW, Mercedes) per CAN | GPL-3.0 | Nur CAN-/VAG-Referenz — rendert *keine* Gauges aufs Display |
| **UsefulElectronics/esp32s3-gc9a01-lvgl** | ESP32-S3 + rundes GC9A01 (SPI) | SNTP-Zeit + MQTT/OpenWeather | offen | LVGL-Smartwatch-Demo = direkt vergleichbares Clock-/Datenanzeige-Szenario auf Rundpanel |
| **openxc-retro-gauge** (retro-gauge) | Arduino Pro Mini + Schrittmotor + 7-Segment | OpenXC seriell, Bridge per **Android-App** | CC-BY 4.0 | Mechanisch + Android-abhängig → nur Ideengeber, nicht übertragbar |

Weitere genannte Praktiker-Projekte: `Janos/esp32-obd2-meter` (GitLab),
„Car Heads-Up Display (Arduino+ELM327)" und „Wifi Enabled OLED ESP32 Car Gauges"
(beide Instructables).

> **Hinweis Instructables „Vehicle Digital Gauge Display":** Seite ist
> JS-gerendert, Schritt-Inhalt nicht maschinell auslesbar. Vom Genre her
> ESP32 + Display + OBD2/ELM327. Bei Bedarf manuell aufrufen.

---

## 2) Datenpfad: 123TUNE+, BLE, CAN/OBD2

### ⭐ 123TUNE+ BLE — `iafilius/123Tune-plus-Simulator`
**Der relevanteste Klassiker-/Zündungs-Fund.** Reverse-engineerter BLE-Simulator
des 123Tune+-Verteilers, damit man **eigene Client-Software statt der
Closed-Source-App** schreiben kann.
- Enthält `btgatt-server.c` (BlueZ-GATT-Server) **und** einen ESP32-Arduino-Port
  (`ESP32-Arduino/ESP32_123Tune_plus_server/...ino`) mit `InitBLE()`,
  FreeRTOS-Tasks (`BLE_RX_Responder_to_TX`), `TuneServer_init()`.
- Dokumentiert das BLE-Protokoll (Stand ~2017/18).
- **Caveat:** ESP32-Port ist vom Autor als WIP/„just for fun" markiert —
  Scaffolding, keine Produktionsimplementierung. Der eigentliche Wert: das
  **plattformunabhängige Protokoll-Wissen** (Characteristics, Frame-Aufbau),
  das du mit NimBLE auf deinem ESP32-S3 nachbauen kannst.

### CAN/OBD2-Anbindung (falls je gewünscht)
- **ELM327** (UART/Bluetooth) → einfachster Weg, siehe ElektorLabs & VaAndCob.
- **Direkt-CAN** via **SN65HVD230** (3,3 V Transceiver, ESP32-interner CAN/TWAI)
  oder **MCP2515** (SPI) → siehe Alfa-Romeo-Projekt & CarCluster.

---

## 3) Grafik-/Gauge-UI-Libraries für runde Displays

### LovyanGFX (dein aktueller Stack) — passt
- Unterstützt **nativ den Hardware-RGB-Parallel-Interface** der ST7701-480×480-
  Panels auf ESP32-S2/S3 (`Panel_RGB.cpp`, `Bus_RGB.hpp`, fertige User-Configs).
- Schnelle **Sprite-Rotation/-Skalierung** des Off-Screen-Buffers → ideal für
  rotierende analoge Zeiger.
- **Aber:** keine fertigen Gauge-/Skalen-Widgets — die VDO-Optik musst du selbst
  per Sprite-Pivot zeichnen (Muster siehe unten).

### LVGL — fertige Gauge-Bausteine
- **`lv_meter`** (LVGL **v8**): Skalen, Minor/Major-Ticks, Labels, **Zeiger als
  Linie *oder* Bild** (`lv_meter_add_needle_img` mit `pivot_x/pivot_y` —
  exakt für VDO-Zeiger um den Mittelpunkt).
- Dokumentiertes Beispiel **„A clock from a meter"** = fertiges Analoguhr-
  Zifferblatt (61 Minuten-Ticks, Stunden-Skala, Bild-Zeiger, animierte Rotation).
- ⚠️ **Versions-Falle:** `lv_meter` gibt es **nur in LVGL v8**; in **v9** ersetzt
  durch `lv_scale`. Das alte v7-„Gauge"-Widget ist **veraltet** (eine
  entsprechende Behauptung wurde in der Recherche als überholt **widerlegt**).

### Zeiger-Muster (Sprite-Pivot-Rotation) — MIT-Repos
- **BrianPea573/GC9A01-ESP32-Analogue-Clock-Analog** (MIT): separate Header
  `hourHand.h/minuteHand.h/secondHand.h`, `setPivot()` + `pushRotated(angle)`.
- **somebox/esp32-GC9A01-round**: analoge **und** digitale Zifferblätter via
  TFT_eSPI/TFT_eSprite (⚠️ **keine LICENSE-Datei** → Status ungeklärt).
- ⚠️ Beide zielen auf **GC9A01 240×240 SPI via TFT_eSPI**, nicht ST7701 480×480
  RGB/LovyanGFX → **Muster/Konzept übertragbar, kein Drop-in-Code**.

### upir (`youtube.com/@upir_upir` · `github.com/upiir`) — beste Tutorials
Spezialist für „pixels, displays and Arduino", **mit kostenlosem Code pro Video**.
Relevante Repos: `gforce_meter_gauge_esp32_nextion`, `nextion_turbo_boost_gauge`,
`arduino_car_vfd_gauges`, `arduino_tpms_tire_pressure`, `hoonicorn_gear_indicator`.
Sehr saubere **antialiased Zeiger/Skalen**.
⚠️ Einige Projekte nutzen **Nextion** (eigenes Display + Firmware) → nicht 1:1 auf
ST7701/LovyanGFX; seine Arduino_GFX-/TFT_eSPI-Inhalte (Nadel-Rendering,
Zifferblatt-Aufbau, flüssige Animation) sind das Übertragbare.

---

## 4) Genau dein Board: ST7701 480×480 Inbetriebnahme

- **ESPHome/Home-Assistant-Community:** getestete Working-Config für den
  **exakten** ESP32-S3-Touch-LCD-2.8C (ST7701S RGB, GT911) inkl. vollständigem
  Pin-Mapping — beste board-spezifische Referenz.
- **LovyanGFX (Arduino-Forum):** vollständige `Bus_RGB`-Config für ST7701/ESP32-S3:
  Daten blue(d0–d4)/green(d5–d10)/red(d11–d15)=5/6/5, Sync HSYNC/VSYNC/DE/PCLK,
  plus SPI-Seitenkanal (CS/SCLK/MOSI) für die ST7701-Register-Init.
- **Arduino_GFX (PlatformIO):** `aquaElectronics/esp32-4848s040-st7701` (**MIT**)
  — komplette ST7701-Init-Sequenz (Gamma/VCOM/VGH, Color-Format 0x3A),
  explizite RGB-Timings, Backlight. Für GUITION ESP32-4848S040, HW-äquivalent.
- ⚠️ **Widerlegt:** LovyanGFX-Issue #542 (angeblich invertierte Farben auf
  ST7701, „getestete exakte HW-Kombi") ist **stale/ohne bestätigten Fix** →
  keine verlässliche Referenz.

---

## 5) Foren & Communities

- **Home Assistant Community** — Working-Config für genau dein Board (s. o.).
- **Arduino-Forum** — ST7701/LovyanGFX-Bring-up-Threads.
- **PlatformIO Community** — Arduino_GFX-Setup für ESP32-S3-RGB-ST7701.
- **MSEXTRA / Megasquirt-Forum** — „MS, JimStim, ESP32, Arduino and CANBUS":
  externes Gauge-Display liest CAN über SN65HVD230 an ESP32-S3.
- **thesamba.com** (classic VW) — VW-Bus-Elektrik/Instrumente.
- **LovyanGFX GitHub Discussions** — RGB-Panel-Themen.
- ⚠️ miataturbo.net-Thread („DIY ESP32 CAN gauge") war als Quelle **unzuverlässig**
  (keine verwertbaren Claims).

---

## 6) Kommerzielle Vergleichsprodukte (closed, nur Feature-Referenz)

- **Revv Gauge** (revvgauge.com): rundes Touch-Gauge 52/60 mm, freies Layout,
  CAN/OBD2 + 6 Analog + EGT, ~550 AUD, **kein** Open Source. (= das ursprünglich
  als „Refgauk" gesuchte Produkt.) Namenswarnung: `revgauge.com` (ein „v") ist
  ein unverwandtes Airline-Tool.
- **Zada Tech** (zada-tech.com): OLED-Einzel-/Multi-Gauges (Boost, RPM, EGT,
  Lambda), Digital-Dashes; CAN/OBD-II/Direktsensoren; closed, kommerziell.

---

## 7) Reinfälle / Vorsicht

- ❌ **RELabUU/revv** — heißt „revv", ist aber ein Requirements-Engineering-
  Webtool der Uni Utrecht. Kein Auto-Gauge.
- ⚠️ **Lizenzen heterogen:** MIT (Gauge.S, BrianPea573, aquaElectronics,
  LovyanGFX) = frei inkl. kommerziell · GPL-3.0 (CarCluster, ElektorLabs) =
  Copyleft · **CC BY-NC** (Alfa-Romeo) = **nicht kommerziell** · somebox-Repo
  **ohne** LICENSE = ungeklärt.

---

## 8) Offene Fragen (für vertiefende Recherche)

1. Gibt es ein konkret veröffentlichtes **ESP-NOW-Cockpit für VW-Bus/air-cooled**
   (thesamba/Hackaday), das ESP-NOW-Empfang mit analoger VDO-Optik kombiniert?
   (Bisher nur ESP32-Gauges + 123Tune+-BLE gefunden, kein bestätigtes
   ESP-NOW-zu-Rundpanel-Cockpit.)
2. Liefert LovyanGFX (oder Begleit-Lib) fertige Zeiger-/Skalen-Widgets, oder
   bleibt die VDO-Optik manuelle Sprite-Rotation (während LVGL `lv_meter` es
   mitbringt)?
3. Board-spezifische Init-/Farb-Eigenheiten des ESP32-S3-Touch-LCD-2.8C vs.
   der dokumentierten GUITION/Sunton-Boards?
4. Existiert ein vollständiger **123TUNE+-BLE-Client für NimBLE/ESP32** (über
   den Simulator/GATT-Server hinaus), der Live-Werte parst?

---

## Quellen

**Projekte/Repos**
- https://github.com/handmade0octopus/gauge.s-sorek.uk (MIT) · Shop: https://shop.sorek.uk
- https://github.com/ElektorLabs/obd2-dashboard (GPL-3.0)
- https://github.com/VaAndCob/ESP32-Bluetooth-OBD2-Gauge
- https://github.com/ClaudeMarais/AlfaRomeoGiulia_DashboardInfo_ESP32-S3 (CC BY-NC 4.0)
- https://github.com/r00li/CarCluster (GPL-3.0)
- https://github.com/UsefulElectronics/esp32s3-gc9a01-lvgl
- https://github.com/iafilius/123Tune-plus-Simulator
- https://openxcplatform.com/projects/retro-gauge.html · https://github.com/openxc-retro-gauge

**Libraries / UI**
- https://github.com/lovyan03/LovyanGFX
- https://docs.lvgl.io/8/widgets/extra/meter.html (lv_meter, v8)
- https://github.com/BrianPea573/GC9A01-ESP32-Analogue-Clock-Analog (MIT)
- https://github.com/somebox/esp32-GC9A01-round (Lizenz ungeklärt)
- https://github.com/upiir · https://youtube.com/@upir_upir

**Board-Bring-up (ST7701 480×480)**
- https://community.home-assistant.io/t/waveshare-esp32-s3-touch-lcd-2-8c-working-config/836834
- https://forum.arduino.cc/t/help-waveshare-esp32-s3-lcd-3-16-st7701-rgb565-using-lovyangfx/1427901
- https://community.platformio.org/t/working-platformio-arduinogfx-setup-for-guition-esp32-4848s040-esp32-s3-rgb-st7701/53607
- https://github.com/aquaElectronics/esp32-4848s040-st7701 (MIT)

**Foren/Communities**
- https://www.msextra.com/forums/viewtopic.php?t=81485 (Megasquirt)
- https://www.thesamba.com/vw/forum/viewtopic.php?t=424027 (classic VW)
- https://github.com/lovyan03/LovyanGFX/discussions/537

**Kommerziell (closed, Feature-Referenz)**
- https://revvgauge.com/ · https://zada-tech.com/

> ⚠️ Nicht relevant: https://github.com/RELabUU/revv (RE-Tool, kein Gauge) ·
> https://github.com/lovyan03/LovyanGFX/issues/542 (stale, widerlegt) ·
> revgauge.com (Airline-Software)
