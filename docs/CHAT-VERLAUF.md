# Chat-Verlauf — Recherche & Umsetzung (VDO-Cockpit)

> Zusammenfassung der Arbeitssitzung auf Branch
> `claude/deep-search-refgauk-research-3fqh32`. Chronologisch, mit Ergebnissen,
> Entscheidungen und Commits.

---

## 1. „Refgauk" identifizieren
Suche nach „Deep Search, Refgauk dot com". `refgauk.com` existiert nicht;
phonetisch = **Revv Gauge** (`revvgauge.com`): Touchscreen-Rundinstrument
(52/60 mm, CAN/OBD2 + Analog + EGT, ~550 AUD), **closed source**. `revgauge.com`
(ein „v") ist ein Airline-Tool. → `docs/RECHERCHE-RevvGauge.md`.

## 2. Vergleichbare Gauge-Projekte
- **Gauge.S** (`handmade0octopus/gauge.s-sorek.uk`, MIT) → bestes offenes Vorbild (~£259).
- **OpenXC Retro Gauge** (CC-BY): Arduino + Schrittmotor, Android-App → Ideengeber.
- **Revv Gauge**: closed.
- ❌ `RELabUU/revv`: Reinfall (RE-Tool der Uni Utrecht).
→ `docs/RECHERCHE-Gauge-Projekte.md`.

## 3. Retro-Gauge Android?
Gauge selbst **Arduino**; OpenXC liefert Daten per **Android-App** (BT→USB-Serial).
Wenig übertragbar.

## 4. Deep-Research (Web/Foren)
5 Such-Winkel, 20 Quellen, 22 verifizierte Aussagen:
- **123TUNE+ reverse-engineered:** `iafilius/123Tune-plus-Simulator`.
- **Board gelöst:** ESPHome-Config für ESP32-S3-Touch-LCD-2.8C (ST7701 480×480).
- **UI:** LovyanGFX, LVGL `lv_meter`, TFT_eSPI-Sprite-Rotation, **upir**
  (`@upir_upir`, `github.com/upiir`).
- Vorbilder: Gauge.S (MIT), ElektorLabs obd2-dashboard (GPL-3), CarCluster (GPL-3),
  Alfa-Romeo Dual-CAN (CC BY-NC). Zusätzlich: Zada Tech (kommerziell).

## 5. 123TUNE+ auswerten + Ökosystem
Eigenes Repo hat bereits einen **verifizierten Decoder** (genauer als iafilius).
Reale Hardware spricht **NUS** (`6e400001/2/3`), nicht iafilius-Ctrl-RX/TX.
- Frame `{Opcode, HiHex, LoHex}` (ASCII-Hex-Nibbles).
- `0x30` RPM `hi*800+lo*50` · `0x31` Adv `hi*3.2+lo*0.2` · `0x32` MAP `raw` ·
  `0x33` Temp `raw−30` · `0x35` Coil `raw/8.65` · `0x41` Volt `raw/4.54`.
- Handshake: TX abonnieren → `'$'`(0x24)+`'\r'`(0x0D) → Keepalive `'$'`/1650 ms.

Ökosystem (github.com/niedi74): `waveshare-vdo-clock`, `m5stack-123`,
`spartan3v2-can-adapter` (Hub), `norbi-espnow`, `VDO-Tempomat-`, `HA`.
→ `docs/123TUNE-BLE-Protokoll.md`, `docs/123TUNE-HUB-HANDSHAKE.md`,
`docs/COCKPIT-ECOSYSTEM.md`.

## 6. 0x35-Coil-Lücke geschlossen (Code)
- `decode123Frame`: `case 0x35`; `applyEspNowFrame`: Coil aus ESP-NOW-Frame.
- Motor-Seite **COIL**-Zeile + Web-Dashboard + `/api/status` (`coil`/`coil_valid`).

## 7. Gemeinsamer Decoder `include/tune123_decode.h`
Decoder + NUS-UUIDs + Handshake-Konstanten + Skalierungen aus dem Inline-Code
extrahiert (Muster wie `spartan_cockpit_frame.h`); `main.cpp` nutzt den Header,
`bleHexNib` entfernt. Bit-genau identisch. TODO: in m5stack-123 + Hub kopieren.

## 8. „2× WLAN gleichzeitig?" — nein
Ein Radio → nur **eine** STA. Lösung: 1× STA (S24/Home) + ESP-NOW auf gleichem
Kanal (`espnow_ch=0`/Auto). Hub muss im S24 sein. „Handy-Fahrt-Modus" geplant.

## 9. Z00-Station: keine IP
Ursache = **Passwort-Falle** im WebGUI: leeres Passwortfeld wird wegen
`if (pass.length()>0)` nicht gespeichert → altes/falsches Passwort bleibt → keine
Auth → keine IP. (Nicht die Bus-Static-IP — die wird für Nicht-Bus auf DHCP gesetzt.)

## 10. On-Device WLAN-Eingabe + WPS (umgesetzt)
**Page 10:** Langdruck WLAN-Zeile → Passwort-Tastatur (abc/ABC/123, SPC/DEL/OK/ESC);
OK speichert ins Profil + reconnect. **WPS-Taste:** PBC via `esp_wps` + WiFi-Event,
speichert SSID/PSK ins Home-Profil. Kurz-Tap = nur Netzmodus.
⚠️ In der Cloud nicht kompilierbar (kein PlatformIO) → lokal
`pio run -e waveshare_s3_28c`, Layout ggf. am Gerät justieren.

---

## Commits (Branch `claude/deep-search-refgauk-research-3fqh32`)
1. Recherche Revv Gauge
2. Recherche Gauge.S/OpenXC/Reinfall
3. Vollständiger Deep-Research-Report
4. 123TUNE+ Protokoll/Handshake/Ecosystem (+ Index)
5. `feat(123)`: Coil 0x35 decodieren + anzeigen
6. `refactor(123)`: `tune123_decode.h` extrahieren
7. `feat(wifi)`: Passwort-Tastatur (Page 10) + WPS

## Offene Punkte
- `tune123_decode.h` in `m5stack-123` / `spartan3v2-can-adapter` übernehmen.
- „Handy-Fahrt-Modus" (1×STA + ESP-NOW Auto) optional.
- WLAN-Tastatur + WPS lokal bauen, am Gerät testen/justieren.
