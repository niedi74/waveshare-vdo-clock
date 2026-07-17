# Waveshare VDO-Clock — Agenten-Briefing

## Was das ist

Cockpit-Display für einen VW T2b (Vergaser-Boxer): rundes 2,8"-Touch-Display
(Waveshare ESP32-S3-Touch-LCD-2.8C, 480×480, ST7701) repliziert die originale
VDO-"Quartz-Zeit"-Uhr und zeigt live Motordaten vom Spartan3-Hub
(Schwester-Projekt `spartan3v2-can-adapter`):
- **HTTP-Poll** `/api/status` vom Hub (primär, 2 Hz)
- **CAN 0x510** (TWAI, GPIO43/44, 500k — Adern **nicht kreuzen**, `can:normal` für ACK)
- **BLE-Hub** und **123TUNE+-direkt** als optionale Fallbacks
- 6 Kombi-Stile (DIGITAL/VDO/123TUNE+/VDO+UHR/DIGIFIZ/OPEL GSI), Lambda-Verlauf,
  Nachtmodus (grüne Birne), Alarme mit Buzzer, WebGUI, OTA, SD-Log, IMU

Eine Quelldatei: `src/main.cpp` (~4900 Zeilen) + `src/hal_waveshare_28c.h`.
Kommentare auf Deutsch, sie erklären das WARUM (oft mit Datum/Vorfall).

## Geräte im Netz (nur lesen, nichts verstellen ohne Auftrag!)

- **Live-Display (Fahrzeug): 192.168.0.76 — PRODUKTIV**, nur verifizierte Stände
- Test-Display (Werkbank): 192.168.0.96, seriell meist COM25 (wandert!)
- Live-Hub (Fahrzeug): 192.168.0.91, AP `SPARTAN3-HUB` — PRODUKTIV
- Test-Hub (Werkbank): 192.168.0.87, AP `Spartan3-TestHub`
- Emu-123: 192.168.0.80 — 123TUNE+-Emulator, API sieht aus wie ein Hub!

Displays verbinden nach Reboot bevorzugt zum Hub-AP (WLAN-Auto) — wenn eins
"verschwindet", hängt es meist dort und ist vom PC-LAN aus nicht erreichbar.

## Eiserne Regeln

1. **Sicherheitskontrakt** (nach realem Vorfall 14.7.: Fahrer verstellte den
   Vergaser nach Simulationswerten): Anzeigen müssen echte von simulierten
   Daten unterscheiden — HTTP: `lambda_test_active`/`source!="CAN"`,
   CAN-Frame 0x510 Byte 7: `flags & 0x10` (Sim-Bit). Quelle zeigt dann "SIM".
   Details: Hub-Doku `lambda-status-logik.md`.
2. **Lambda nur bei `status_code==3` (OK) als Zahl zeigen** — bei ERR/WAIT/HEAT
   den Statustext (Sonden-Aufwärmphase liefert Fantasiewerte). CAN: Bits 2-3
   im flags-Byte.
3. **Jeder angezeigte Wert braucht einen fresh-Guard** — stale Daten zeigen "--",
   nie den letzten Wert als aktuell.
4. **`millis()`-Deadlines nur wrap-sicher** vergleichen:
   `(int32_t)(millis() - at) >= 0` (Dauerplus, 49,7-Tage-Overflow).
5. **Keine Zugangsdaten committen**: `src/wifi_secret.h` und SD-`wifi.txt` sind
   gitignored. SD-`wifi.txt` gewinnt beim Boot über NVS.
6. **Live-Display (.76) nicht ungefragt flashen/OTA-en** — erst am Test-Display
   (.96) verifizieren, idealerweise mit `/screen`-Screenshot.

## Bauen, Flashen, Verifizieren

- Build: `pio run -e waveshare_s3_28c` (PlatformIO/pioarduino, Arduino-Core 3.x)
- Flash seriell: `-t upload --upload-port COMxx` — Port vorher mit
  `pio device list` prüfen (natives USB-CDC, wandert nach Reboot;
  Auto-Reset nach Flash unzuverlässig → `reboot`-Serial-Kommando)
- Flash OTA: `curl -F "firmware=@.pio/build/waveshare_s3_28c/firmware.bin" http://<ip>/update`
- `scripts/inject_time.py` stellt die RTC auf PC-Zeit und injiziert `GIT_REV`
  ("+" = dirty Tree). **Releases immer aus sauberem committeten Stand bauen.**
- Verifikation: `/version` (Stand), `/live` (Werte), `/log` (SD-Ereignisse),
  `/screen` (160×160-RGB565-Rohdump des Framebuffers, per Python/PIL wandeln).
  Serial-Kommandos: Hilfetext in main.cpp (`Serial.println("Commands:...`).

## Release-Workflow (nach JEDEM Commit)

1. Clean-Build aus committetem Stand
2. `gh release upload fw-live-latest firmware.bin --clobber` + Kopie als
   `firmware-<hash>.bin` (Rollback-Historie, alte Assets nie löschen)
3. Release-Notes aktualisieren
4. Branch UND `main` pushen (`git push && git push origin HEAD:main`)

Commit-Messages auf Deutsch (Anlass → Fix → Verifikation), Abschluss:
`Co-Authored-By: Claude <Modell> <noreply@anthropic.com>`

## Bekannte Fallstricke

- `HAL_EXIO_SD_CS` ist laut Schaltplan evtl. um 1 Bit verschoben — SD läuft
  aber seit Monaten stabil: **nicht ohne Hardware-Messung ändern**.
- `setPixel` rotiert bei `g_rotationDeg != 0`; wer direkt in den Framebuffer
  schreibt, muss Grenzen selbst prüfen (FB-Overflow verursachte früher
  lwIP-Heap-Korruption + Guru-Bootloop).
- GT911/RTC/IMU/TCA9554 teilen den I2C-Bus (SDA=15, SCL=7). Externer
  I2C-4-Pin-Stecker ist für den ADS1115-Dimmer reserviert (Backlog);
  der UART-Stecker gehört dem CAN-Transceiver. Die beiden Stecker sehen
  identisch aus — CAN am I2C-Stecker macht das Display schwarz.
- BLE neben WLAN+RGB-Panel kostet Loop-Zeit: 123-direkt ist deshalb
  default AUS; BLE-Timing nicht ohne Gerätetest "optimieren".
