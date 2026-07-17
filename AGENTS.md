# AGENTS.md — Briefing für Agenten in diesem Repo

## Was das hier ist

Firmware für ein rundes 2,8"-Touch-Display (Waveshare ESP32-S3-Touch-LCD-2.8C,
480×480, ST7701) im Cockpit eines **VW T2b Bus**. Es repliziert die originale
VDO-"Quartz-Zeit"-Uhr und zeigt live Motordaten (Lambda, Drehzahl, Zündung, MAP,
Tempo) vom **Spartan3-Hub** — einem zweiten ESP32 im Motorraum. Der Fahrer
**vertraut diesen Anzeigen beim Vergaser-Einstellen**: falsche/stale Werte können
zu echten Motorschäden führen. Korrektheit der Datenpfade geht vor allem anderen.

## Build & Flash

- PlatformIO (pioarduino, Arduino-Core 3.x): `pio run -e waveshare_s3_28c`
- Flash seriell: `pio run -e waveshare_s3_28c -t upload --upload-port COMxx`
  — **COM-Port wechselt gern nach Reboot** (natives USB-CDC), vorher
  `pio device list` prüfen. Auto-Reset nach Flash ist unzuverlässig.
- Flash OTA: `curl -F "firmware=@.pio/build/waveshare_s3_28c/firmware.bin" http://<ip>/update`
- `scripts/inject_time.py` stellt die RTC bei jedem Flash auf PC-Zeit und
  injiziert `GIT_REV` (+ "+"-Suffix bei dirty Tree). **Für Releases immer aus
  sauberem committeten Stand bauen.**

## Geräte-Landschaft (Heimnetz Z00-Station)

| Gerät | IP | Rolle |
|---|---|---|
| Test-Display | 192.168.0.96 (COM25, wechselt) | zum Entwickeln/Flashen |
| **Live-Display** | 192.168.0.76 | im Bus verbaut — nur verifizierte Stände! |
| Live-Hub (Bus) | 192.168.0.91, AP `SPARTAN3-HUB` | Produktiv |
| Test-Hub (Haus) | 192.168.0.87, AP `Spartan3-TestHub` | Lambda-Tests, oft aus |
| Emu-123 | 192.168.0.80 | 123TUNE+-Emulator, API sieht aus wie ein Hub! |

Displays verbinden nach Reboot bevorzugt zum **Hub-AP** (WLAN-Auto) — wenn eins
"verschwindet", hängt es meist dort und ist vom PC-LAN aus nicht erreichbar.

## Datenpfade (Priorität)

HTTP-Poll `/api/status` (primär, 2 Hz) → CAN 0x510 (TWAI, GPIO43/44,
**nicht kreuzen**, `can:normal` für ACK) → BLE-Hub (opt.) → 123TUNE+-direkt (opt.).

Wichtige Invarianten — **nicht aufweichen**:
- Lambda ist nur bei `status_code==3` (OK) gültig; bei ERR/WAIT/HEAT zeigt das
  Display den Statustext, nie einen Zahlenwert.
- Wenn der Hub simuliert (`lambda_test_active`), zeigt die Quelle **"SIM"** —
  über ALLE Pfade gelatcht (CAN/BLE tragen kein Sim-Bit; HTTP-Wissen hält das Label).
- Jeder angezeigte Wert braucht einen fresh-Guard; stale Daten zeigen "--".
- `millis()`-Deadlines nur wrap-sicher vergleichen: `(int32_t)(millis() - at) >= 0`
  (Dauerplus-Betrieb, 49,7-Tage-Overflow).

## Arbeitsweise / Konventionen

- Einzige Quelldatei ist `src/main.cpp` (~4900 Zeilen) + `src/hal_waveshare_28c.h`.
  Kommentare auf Deutsch, erklären das WARUM (oft mit Datum/Vorfall).
- Commit-Messages auf Deutsch, ausführlich (Anlass → Fix → Verifikation),
  Abschluss: `Co-Authored-By: Claude <Modell> <noreply@anthropic.com>`.
- **Nach jedem Commit**: Clean-Build und `firmware.bin` + `firmware-<hash>.bin`
  ins GitHub-Release `fw-live-latest` hochladen (`gh release upload ... --clobber`),
  Release-Notes aktualisieren. Branch UND `main` pushen (`git push origin HEAD:main`).
- Verifikation am Gerät: `/screen` liefert 160×160-RGB565-Rohdump des Framebuffers
  (Python/PIL zum Konvertieren); `/version`, `/live`, `/log` für Zustand.
  Serial-Kommandos siehe Hilfetext in main.cpp (`else { Serial.println("Commands:...`).
- WLAN-Zugangsdaten: `src/wifi_secret.h` und SD-`wifi.txt` sind gitignored —
  **niemals committen**. SD-`wifi.txt` gewinnt beim Boot über NVS.

## Bekannte Fallstricke

- `HAL_EXIO_SD_CS` ist laut Schaltplan evtl. um 1 Bit verschoben — SD läuft aber
  seit Monaten stabil: **nicht ohne Hardware-Messung ändern** (docs + Memory).
- Framebuffer-Zeichnen: `setPixel` rotiert bei `g_rotationDeg != 0`; wer direkt
  in den FB schreibt (wie `copyVdoDialToFrame`), muss Grenzen selbst prüfen
  (FB-Overflow verursachte früher lwIP-Heap-Korruption + Bootloop).
- GT911-Touch, RTC (PCF85063), IMU (QMI8658), TCA9554-Expander teilen sich
  den I2C-Bus (SDA=15, SCL=7). Externer I2C-Stecker ist für ADS1115-Dimmer
  reserviert (Backlog). UART-Stecker gehört dem CAN-Transceiver.
- Die beiden 4-Pin-Stecker (UART/I2C) auf der Platine sehen identisch aus —
  CAN-Adapter am I2C-Stecker macht das Display schwarz (LCD-Init via Expander
  wird gestört). Beschriftung prüfen.
