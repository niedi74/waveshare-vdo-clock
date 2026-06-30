# Handover an den deep-search-Chat (`claude/deep-search-refgauk-research-3fqh32`)

**Von:** Trunk-Chat auf `claude/cranky-proskuriakova-cafad7`
**Stand:** 2026-06-30

Kurz und ehrlich, damit wir nicht doppelt/gegeneinander arbeiten. Danke für die
ESP-NOW-Deaktivierung und die WLAN-Diagnose — den **Diagnose-Teil** haben wir
übernommen. Der Rest unten bitte **nicht weiterverfolgen**.

---

## 1. Welcher Branch ist die Firmware-Quelle

**`claude/cranky-proskuriakova-cafad7` ist der Firmware-Trunk.** Alle Firmware
(`src/main.cpp` + `src/hal_waveshare_28c.h`) wird von dort gebaut und geflasht.

Die `main.cpp` im deep-search-Branch (~6000 Zeilen, ESP-NOW-zentriert, andere
Struktur: `WIFI_PROFILES`, `requestPage`, SETUP-2) **wird nicht gemergt** — die
Branches sind ~22.700 Zeilen auseinander. Ein Merge der Firmware ist nicht geplant.

➡️ **Bitte keine weiteren Firmware-Features in der deep-search-`main.cpp` bauen.**
Wenn dir ein Feature wichtig ist: kurze Notiz/Snippet, wir portieren es gezielt in
den Trunk (so lief es mit Tastatur, WPS, WLAN-Diagnose).

## 2. Was KEINEN Sinn macht / bitte NICHT weiterverfolgen

- **ESP-NOW** — endgültig gestrichen. Ihr habt es deaktiviert (gut). Bitte **nicht
  reaktivieren** und keine ESP-NOW-Features/Frames/`spartan_cockpit_frame.h`-Pflege
  mehr. Datenweg ist **WiFi-HTTP (primär) → CAN 0x510 → BLE-Hub → 123-direkt**.
- **VSYNC-Doppelpuffer** (`f0b73cb`, `536bf2d`) — **flackert auf der echten Hardware**.
  Auf dem Trunk getestet und **wieder revertiert** (unsere Seiten machen nicht jeden
  Frame ein Voll-Redraw → Buffer-Swap zeigt abwechselnd alte/neue Stände). Der
  einzelne PSRAM-Framebuffer + `draw_bitmap`-Copy ist die gewählte Lösung. Bitte
  nicht weiter daran bauen.
- **Fester WLAN-/ESP-NOW-Kanal (ch 6)** — ohne ESP-NOW gegenstandslos. Außerdem:
  ESP32 = **ein Funkmodul**, der AP folgt zwangsweise dem STA-Kanal. Kein Hardcode.
- **Paralleles WiFi-/Reconnect-Rewrite** (`setAutoReconnect(true)`, 5s-Intervall,
  WIFI_PROFILES-Slots) — der Trunk hat eine eigene, funktionierende Lösung:
  **Scan-freies Profil-Auto-Fallback (Hub-AP > Heim)** + **WPS** + On-Screen-Tastatur.
  Das beißt sich mit eurem Ansatz; bitte nicht parallel weiterentwickeln.
- **On-Screen-Tastatur / WPS** — auf dem Trunk bereits an unsere Struktur (`g_wprof`,
  `selectWprof`/`saveWprof`, WLAN-Seite Page 11) angepasst. Keine zweite Variante nötig.

## 3. Wo der deep-search-Chat WERT schafft (gerne weiter!)

Eure Stärke ist **Recherche & Doku** — das ist kanonisch und über alle Repos nützlich:
- Refgauk/Revv-Gauge-Recherche, Gauge-Projekte/Libraries-Report.
- **123TUNE+-BLE-Protokoll**, Hub-Handshake, Cockpit-Ecosystem-Matrix.
- **`include/tune123_decode.h`** als gemeinsamer Decoder-Header (single source of truth).
- Datenpfad-/NVS-Doku (ohne ESP-NOW).

➡️ Bitte **Docs/Recherche/Protokoll-Header** weiter pflegen — Firmware-Code nicht.

## 4. Kanonischer Firmware-Stand

Liegt in **`HANDOFF.md`** auf dem Trunk. Kurzfassung:
- Hub-AP primäre WiFi-Verbindung (S24 nicht in Auto-Kette), WPS + WLAN-Seite (Page 11).
- LAMBDA-Seite hat 2. Stil **λ-Verlauf** (langer Druck Mitte schaltet).
- Einzel-Framebuffer (kein Doppelpuffer). OTA über `vdo-clock.local/update`.
- Geplant: **CAN 0x510 als primäre Quelle**, sobald die Verkabelung steht.

Fragen/Wünsche fürs Firmware-Verhalten → an den Trunk-Chat, wir setzen's dort um.
