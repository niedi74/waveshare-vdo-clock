# VDO Quartz-Zeit - Waveshare ESP32-S3 Round LCD

Digitale Nachbildung der VDO Quartz-Zeit Analoguhr aus dem VW T2b auf dem runden
Waveshare ESP32-S3-Touch-LCD-2.8C. Das Display kann als Uhr laufen und im Bus
Live-Daten vom Spartan3-Hub anzeigen.

![VDO Uhr Design](docs/design/vdo_clock_preview.png)

## Hardware

| Komponente | Detail |
| --- | --- |
| Board | Waveshare ESP32-S3-Touch-LCD-2.8C, rund |
| Display | 2.8" IPS rund, 480x480, ST7701 RGB |
| Touch | GT911 kapazitiv, I2C |
| Besonderheit | CS/RST des LCD laufen ueber TCA9554 I2C-GPIO-Expander |
| Prozessor | ESP32-S3 mit PSRAM |
| USB | native USB-CDC, lokal COM13 |

Nicht mit der eckigen `ESP32-S3-Touch-LCD-2.8` verwechseln. Dieses Projekt ist
die runde C-Variante mit 480x480 RGB-Panel.

## Software-Stack

- LovyanGFX fuer ST7701 RGB + Touch/Panel-Bring-up
- ESP-NOW als primaerer Live-Datenpfad vom Spartan3-Hub im Bus
- WiFi HTTP fuer `/api/status`, Diagnose und Hub-Zeit
- NimBLE-Arduino fuer optionale BLE-Fallbacks und direct 123TUNE+ Debug
- PlatformIO / Arduino-Framework

## Funktion

1. Uhr-Modus: analoge VDO-Uhr, Hub-NTP / lokales NTP oder RTC.
2. Bus-Modus: ESP-NOW Empfang vom `Spartan3-Hub` auf Kanal 6, HTTP-Fallback auf `192.168.4.1`.
3. Fallback-Modi: BLE Hub oder direct 123TUNE+ BLE fuer Standalone-/Debug-Betrieb.

## WLAN-Profile

Siehe [docs/WLAN-MATRIX.md](docs/WLAN-MATRIX.md) fuer Home / Phone / Bus auf M5 und Waveshare.

## Zeit-Synchronisation

Prioritaet:

1. Hub `/api/status` mit `ntp_synced:true` und `time_epoch`
2. Lokales NTP ueber Home/Phone-WLAN mit Internet
3. RTC oder Build-Zeit

Der Hub ist Time-Master: Phone-Hotspot -> Hub-NTP -> Waveshare uebernimmt Hub-Zeit.

## Live-Daten vom Spartan-Hub

Primaer wird der gemeinsame ESP-NOW Binary Frame aus `include/spartan_cockpit_frame.h`
genutzt. Der HTTP-Fallback liest `GET /api/status`.

BLE bleibt als Fallback kompatibel mit dem alten Textpayload:

```text
L<lambda>R<rpm>A<adv>M<map>V<bm6_volt>S<speed>I<123_volt>T<123_temp>C<coil>
```

Service `7f510001-5a6b-4d2a-9f20-14a7f3e20000`, Notify `...0002`.

## Build

```powershell
pio run -e waveshare_s3_28c
pio run -e waveshare_s3_28c -t upload --upload-port COM13
pio device monitor --port COM13 --baud 115200
```

## Status

Bring-up: Display-Treiber, Touch, WebGUI, WiFi-Profile und ESP-NOW Client sind vorhanden.
Pin-Belegung siehe `docs/PINOUT.md`.

## Verwandte Projekte

- [spartan3v2-can-adapter](https://github.com/niedi74/spartan3v2-can-adapter) - Motorraum-Hub
- m5stack-123 - M5 Dial Cockpit-Display
