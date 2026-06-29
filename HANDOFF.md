# Handoff — Waveshare VDO Clock

**Branch:** `claude/cranky-proskuriakova-cafad7` (= Stamm/Trunk)
**Stand:** 2026-06-29 · Tip-Commit siehe `git log -1`
**Board:** Waveshare ESP32-S3-Touch-LCD-2.8C (rund 480×480, ST7701 RGB, GT911 Touch) · **Port:** COM13

> Diese Datei ist die Übergabe an die Laptop-/Remote-Session, weil lokale Memory-Notizen
> nicht über Git wandern. Nach `git pull` ist hier der komplette aktuelle Stand.

---

## 1. Was JETZT läuft (geflasht & verifiziert)

- ✅ **Hub-AP = primäre Datenverbindung.** Boot geht direkt auf den Hub-AP (`192.168.4.2`),
  `src=HTTP`, frische Daten (`age ≈ 260 ms`). Kein Hängenbleiben am Handy-Hotspot mehr.
- ✅ **S24 (Handy-Hotspot) ist NICHT in der Auto-Kette** — nur manuell wählbar. Reißt der
  Hub-AP kurz ab (Hub-Reboot mit Motor), reconnectet das Display auf den Hub-AP statt auf S24.
- ✅ **Flackern weg:** der VSYNC-Doppelpuffer wurde **wieder revertiert** (s. u.).
- 🟡 **WLAN-Seite (Page 11) + WPS** (Commit `9245db0`): Setup → WIFI öffnet eine Seite mit
  großen Buttons **WPS verbinden / Passwort tippen / Profil wechseln / Zurück**. WPS (PBC,
  wie M5 Dial) speichert die Router-Daten ins Heim-Profil. **Gebaut, aber NOCH NICHT
  geflasht/getestet** (COM13 war ab). On-Screen-Tastatur (Page 10) bleibt als Fallback.

### Letzte relevante Commits
```
7c7f68c Revert HAL to single malloc framebuffer - double-buffer caused flicker
43abdc1 WiFi: Hub-AP primary (S24 out of auto) + compact keyboard + short/long WIFI tap
f705b0c WiFi auto priority (Zwischenschritt)
07008eb HAL: VSYNC double-buffer  <-- REVERTIERT durch 7c7f68c, NICHT naiv neu einbauen
981b437 Add on-screen WiFi keyboard (page 10)
e2718f0 HTTP poll: backoff + shorter timeouts
```

---

## 2. Bauen / Flashen / Debuggen

```bash
pio run -e waveshare_s3_28c                                   # Build
pio run -e waveshare_s3_28c -t upload --upload-port COM13     # Flash (USB)
```
- ⚠️ **USB-Auto-Reset nach dem Flash greift auf diesem Board NICHT zuverlässig** → die neue
  Firmware läuft erst nach einem Neustart. Dafür gibt es das Serial-Kommando **`reboot`**
  (sendet `ESP.restart()`). Nach dem Flash also `reboot` über COM13 schicken.
- Serial lesen NICHT mit `pio device monitor` (resettet), sondern RTS/DTR=0. STAT-Zeile
  alle 5 s: `STAT up=.. ip=.. wifi=.. prof=.. httpRx=.. canRx=.. src=.. age=.. heap=..`.
- Weitere Serial-Cmds: `can:test`/`can:rx`, `imu:null`, `wifi:next`, `ap:on`, `123:on`.

### ⚠️ Nicht in Git (Passwörter)
`src/wifi_secret.h` ist gitignored. Auf dem Laptop neu anlegen (aus `wifi_secret.example.h`):
```c
#define WIFI_SSID     ""           // optional
#define WIFI_PASSWORD ""
#define S24_AP_PASS   "Frankfurt1" // sonst baut es nicht (S24-Profil-Seed)
```

---

## 3. Architektur-Kurzfassung

- **Datenpfad-Priorität:** HTTP-Poll (`/api/status`) → CAN `0x510` → BLE-Hub (opt., default aus)
  → 123TUNE+ direkt (opt., default aus). **ESP-NOW ist gestrichen** (bewusst, nicht portieren).
- **WLAN-Profile** `g_wprof[3]`: 0=Heim (`Z00-Station`), 1=Hub-AP (`Spartan3-TestHub`/`lambda123`,
  Gateway=Hub), 2=S24 (`Android-AP1`/`Frankfurt1`, mDNS `spartanhub.local`).
- **Auto-Fallback** `wifiAutoTick()`: OHNE Scan (Scan crasht: `esp_wifi_scan_start`
  StoreProhibited), probiert **Hub-AP > Heim** je ~6 s. S24 nur manuell.
- **Setup → Zeile WIFI:** kurzer Tap = nächstes Profil aktivieren · langer Druck (≥0,6 s) =
  Tastatur (Page 10). `handleSetupLongPress(y, durMs, isLong)`.
- **HAL** (`hal_waveshare_28c.h`): Einzel-PSRAM-Framebuffer + `draw_bitmap`-Copy. `hal_restart()`
  (= `esp_lcd_rgb_panel_restart`) 1×/s auf allen Nicht-Uhr-Seiten gegen WiFi-Schwarz.
  `hal_pause_for_ota()` hält das RGB-Panel beim OTA-Schreiben an.

---

## 4. Offene Punkte / TODO

1. **WLAN-Seite + WPS flashen & testen** (`9245db0`) — WPS am echten Router (Z00-Station)
   prüfen: WLAN-Seite → „WPS verbinden", Router-WPS-Taste drücken → sollte verbinden und
   ins Heim-Profil speichern. Tastatur (Page 10) ist nur noch Fallback; ggf. noch kleiner.
2. **Lambda-Verlauf-Graf-Seite** (Backlog): neue Display-Seite mit **Linien-Graph über Zeit**
   für **Lambda vs Drehzahl / Unterdruck(MAP) / Geschwindigkeit** (+ später Gaspedal). Werte
   fließen schon (`g_lambda/g_rpm/g_map`/`speed_kmh`) → Ring-Puffer + Diagramm, keine neue
   Datenquelle. Speed aus Rad-Drehzahl/Reed (GPIO27 am Hub), kein GPS nötig.
3. **CAN physisch** (`0x510`, listen-only, RX=GPIO44/TX=GPIO43): braucht Spartan-ACK am Bus +
   saubere Kontakte (CANH/CANL/GND/3V3) + 60 Ω Gesamt-Terminierung. Decoder/Code sind fertig.

---

## 5. Weitermachen auf dem Laptop
```bash
git fetch origin
git checkout claude/cranky-proskuriakova-cafad7
git pull
# src/wifi_secret.h neu anlegen (s. oben), dann pio run ...
```
