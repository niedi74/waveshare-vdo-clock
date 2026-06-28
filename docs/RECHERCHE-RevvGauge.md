# Recherche: „Refgauk" = **Revv Gauge** (revvgauge.com)

> Notiz zur Sprach-Eingabe „Deep Search, Refgauk dot com". Die Domain
> `refgauk.com` existiert nicht — gemeint ist mit hoher Wahrscheinlichkeit
> **Revv Gauge** (`revvgauge.com`). „Revv Gauge" gesprochen ≈ „Reff-Gauk".
> Das Produkt deckt konzeptionell genau das ab, was dieses Projekt macht:
> ein frei gestaltbares, rundes Touch-Display für Live-Fahrzeugdaten.

## Was ist Revv Gauge?

Kommerzielles, anpassbares **Touchscreen-Rundinstrument** für Fahrzeuge. Statt
fester Templates gestaltet man das Layout komplett frei und platziert analoge
Zeiger, Werte, Labels und Bilder beliebig auf dem Display — sehr ähnlich zur
Idee „eigenes VDO-Zifferblatt + Live-Daten" in diesem Projekt.

### Eckdaten

| Punkt | Detail |
| --- | --- |
| Bauform | Rundes Gauge, **52 mm** und **60 mm**, flächenbündige Montage |
| Bedienung | **Voll-Touchscreen**, keine Tasten |
| Display | Hell, auf Tageslicht-Lesbarkeit ausgelegt, Auto-Helligkeit |
| Eingänge | **6 analoge** Inputs (NTC-Temperatur oder linear 0–5 V) |
| Sonder-Eingang | **K-Typ-Thermoelement** (EGT / Abgastemperatur) |
| Bus | **CAN-Bus**: OBD2 + Haltech-Protokoll |
| Ausgänge | 2 programmierbare Outputs (Summer/Lampe/Relais, Alarme/Warnungen) |
| Bordnetz | 12 V **und** 24 V |
| Konfiguration | per **App**; Firmware-Updates ebenfalls über die App |
| Preis | ca. **550 AUD** (~330–365 USD), aktuell Warteliste / „sold out" |

## Gibt es GitHub / offene Quellen?

**Nein.** Bei der Recherche kein öffentliches GitHub-Repo, keine Firmware-Quellen,
kein Hinweis auf Open Source oder DIY gefunden. Revv Gauge ist ein
geschlossenes Kaufprodukt — Konfiguration nur über die hauseigene App. Es gibt
also nichts „von denen" zum Nachbauen oder als Code-Vorlage.

## Einordnung für dieses Projekt

- **Konzept-Überschneidung hoch:** rundes Touch-Display, frei gestaltbares
  Layout, Live-Daten. Bestätigt, dass die Projektrichtung sinnvoll/marktnah ist.
- **Unterschied:** Revv Gauge holt Daten klassisch über CAN/OBD2/Analog-Sensoren.
  Dieses Projekt nutzt **ESP-NOW vom Spartan3-Hub** bzw. **123TUNE+ BLE** und
  einen größeren **480×480-Schirm** (ESP32-S3-Touch-LCD-2.8C) — also eher ein
  voll konfigurierbares Cockpit-Display als ein 52/60-mm-Single-Gauge.
- **Preis:** Das fertige Produkt liegt preislich höher; der DIY-Weg hier
  (Waveshare-Board + eigene Firmware) ist die deutlich günstigere Alternative
  und bietet mehr Freiheit (eigene VDO-Optik, ESP-NOW-Bus-Integration).

### Ideen, die man sich abschauen könnte

- Freies Layout-Editing direkt am Gerät bzw. per Begleit-App.
- Programmierbare **Schwellwert-Warnungen** (optisch/akustisch) aus den Live-Daten.
- **Auto-Helligkeit** (passt gut zum geplanten Tacho-Dimmer-Feature in `FUTURE.md`).
- Optionaler **K-Typ-/EGT-Eingang** als spätere Sensor-Erweiterung.

## Quellen

- https://revvgauge.com/ — Produkt-Startseite
- https://revvgauge.com/collections/all — Produktliste (52 mm + Loom, 550 AUD)
- https://www.instagram.com/revvgauge/ — Demos/Setups (@revvgauge)

> Hinweis Namensgleichheit: `revgauge.com` (ein „v") ist ein **anderes**,
> unverwandtes Airline-Software-Produkt — nicht verwechseln.
