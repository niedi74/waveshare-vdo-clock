# Recherche: vergleichbare Gauge-/Display-Projekte („Refgauk")

> Ausgangspunkt: Sprach-Eingabe „Deep Search, Refgauk dot com" — gesucht war
> ein vorgestelltes Cockpit-/Gauge-Setup, dessen fertiges Produkt zu teuer ist.
> Unten alle geprüften Kandidaten. **Wichtigster Treffer für dieses Projekt:
> Gauge.S (open source, ESP32).**

---

## ⭐ Gauge.S — handmade0octopus (Maciej „sorek" Loboda)

**Das relevanteste Projekt.** Open-Source All-Car-Datenlogger **und**
ESP32-Entwicklungsboard, das gleichzeitig als konfigurierbares Display/Gauge dient.

- **Repo:** https://github.com/handmade0octopus/gauge.s-sorek.uk — **MIT-Lizenz**
- **Wiki/Doku:** https://github.com/handmade0octopus/gauge.s-sorek.uk/wiki
- **Shop:** https://shop.sorek.uk — fertige Geräte ab **~£259 GBP**
  (z. B. E39/E46-Vent-Mount; auch BMW-E36-**Direktersatz-Cluster** PnP)

| Punkt | Detail |
| --- | --- |
| Prozessor | **ESP32** (v4.X, 4 Connectoren), reines ESP32-Dev-Board für Auto-Anwendungen |
| Eingänge | bis zu **7 analoge** 0–5 V Inputs, **2 MOSFET-Ausgänge** |
| Sensorik | eingebauter **Beschleunigungssensor / G-Meter**, optionales **GPS** |
| Daten | Engine-/Car-/Sensor-Daten, Live-Feed **25–33× schneller** als Standard-OBDII |
| Logging | parallel auf **SD-Karte**, per **WiFi** vom Handy/Tablet auslesbar |
| Fahrzeuge | BMW E36/E39/E46/E38/Z3 sowie OBD2-Fahrzeuge ab ~2007 |
| Konfiguration | **`config.json` / Definitionen** in `/definitions` an Auto/ECU anpassen |
| Firmware | `update.bin` auf SD-Karte legen → Update; kostenlose künftige Updates |
| Stack | JS + HTML + Python (Tooling), Firmware als `.bin`; aktive **Discord**-Community |

**Warum relevant:** Genau das Muster „ESP32 + Display + frei konfigurierbares
Layout + Live-Fahrzeugdaten + SD-Log", das dieses Projekt verfolgt — und der
**Code ist offen (MIT)**. Hier bekommt man also tatsächlich „Infos von denen":
Definitions-/Config-Format, ECU-Parsing, Firmware-Aufbau lassen sich als Vorlage
nutzen, ohne das ~£259-Gerät kaufen zu müssen.

---

## Retro Gauge — OpenXC (ältere Inspiration, mechanisch)

Open-Source-„mechanisches" Gauge, das Fahrzeugdaten über die OpenXC-Plattform
seriell empfängt und mit Schrittmotor + 7-Segment + RGB-LEDs anzeigt.

- Projektseite: https://openxcplatform.com/projects/retro-gauge.html
- Hardware (Eagle-Schaltpläne/PCB): https://github.com/openxc-retro-gauge/retro-gauge-hardware — **CC-BY 4.0**
- Hardware: **Arduino Pro Mini**, VID29-07 Schrittmotor, 2-stellige 7-Segment-Anzeige, 2 RGB-LEDs, Mini-USB
- Datenquelle: **OpenXC** (seriell über USB)

**Einordnung:** Konzeptionell älter und mechanisch (Zeiger per Schrittmotor),
nicht display-basiert. Eher Ideengeber als Code-Vorlage für dieses Projekt.

---

## Revv Gauge — kommerziell, geschlossen (revvgauge.com)

Anpassbares **Touchscreen-Rundinstrument** (52/60 mm), freies Layout, Live-Daten.

- Web: https://revvgauge.com/ · Produktliste: https://revvgauge.com/collections/all
- CAN-Bus (OBD2 + Haltech), 6 Analog-Inputs, K-Typ-Thermoelement (EGT), 2 prog. Ausgänge, 12/24 V
- Konfiguration nur über **App**; ca. **550 AUD**, aktuell Warteliste/„sold out"
- **Kein** öffentliches GitHub / **nicht** open source — nichts zum Nachbauen

> Namens-Warnung: `revgauge.com` (ein „v") ist ein unverwandtes
> Airline-Software-Produkt — nicht verwechseln.

---

## ❌ RELabUU/revv — Reinfall (kein Auto-Gauge)

https://github.com/RELabUU/revv heißt zwar „revv", ist aber ein
**Requirements-Engineering-Validierungstool** der Uni Utrecht (PHP/HTML-Webtool
für User-Story-Analyse). Hat **nichts** mit Fahrzeug-Gauges zu tun.

---

## Fazit für dieses Projekt

- **Bestes Vorbild + offene Quelle: Gauge.S.** Gleiche Idee (ESP32 + Display +
  konfigurierbare Live-Daten + SD-Log), MIT-lizenziert → Config-/Definitions-
  Format und Firmware-Struktur als Referenz nutzbar. Fertiggerät teuer (~£259),
  der DIY-Weg hier (Waveshare-Board + eigene Firmware) bleibt die günstige Alt.
- **Unterschied dieses Projekts:** rundes **480×480**-Display, Datenpfad über
  **ESP-NOW vom Spartan3-Hub** bzw. **123TUNE+ BLE** statt klassisch CAN/OBD2,
  plus eigene **VDO-Optik**.
- **Abschaubar:** JSON-Definitions pro Fahrzeug/ECU, schnelles Live-Refresh,
  SD-Logging + WiFi-Auslesen, Beschleunigungssensor/G-Meter, Schwellwert-Alarme.

## Quellen

- https://github.com/handmade0octopus/gauge.s-sorek.uk (Repo, MIT)
- https://github.com/handmade0octopus/gauge.s-sorek.uk/wiki (Doku)
- https://shop.sorek.uk/products/gauge-s-for-e39-e46-middle-vent-mount-beta (~£259)
- https://openxcplatform.com/projects/retro-gauge.html
- https://github.com/openxc-retro-gauge/retro-gauge-hardware (CC-BY 4.0)
- https://revvgauge.com/ (kommerziell, closed)
- https://github.com/RELabUU/revv (Reinfall — RE-Tool, kein Gauge)
