# Cockpit-Ökosystem (niedi74) — Reuse-Matrix & Datenfluss

> Überblick, welches Repo welche Logik hält und was zwischen den Projekten
> geteilt wird/werden sollte. Bezug: VW-T2b-Cockpit mit Spartan-Hub als Gateway
> und mehreren Displays.

## Repos & Rollen

| Repo | Rolle | 123-BLE | ESP-NOW | CAN | Anzeige |
| --- | --- | --- | --- | --- | --- |
| **waveshare-vdo-clock** *(dieses)* | VDO-Rund-Display | Client (direct/Hub) | **RX** | – | ST7701 480×480 |
| **m5stack-123** | M5Dial 123-Ignition-Board | Client (Original-RE) | **RX** | – | M5Dial |
| **spartan3v2-can-adapter** | **Spartan-Hub / Gateway** | Client | **TX (Broadcast)** | Spartan λ (ID 0x400, 500 kbit/s) | I2C-LCD + Web |
| **norbi-espnow** | ESP-NOW-Monorepo: Doku + `tools/bus-debug-dashboard` | – | Doku/Debug | – | – |
| **VDO-Tempomat-** | Legacy-Tempomat an CAN | – | – | ja | – |
| **HA** | Home-Assistant-Configs | – | – | – | – |
| OpenELEC.tv, vdr4arch | alte Forks — irrelevant | – | – | – | – |

## Datenfluss (Bus-Setup)

```
Spartan-λ ──CAN(0x400)──┐
123TUNE+ ──NUS-BLE──────┤  Spartan-Hub (spartan3v2-can-adapter)
BM6 ──BLE───────────────┤   • decodiert 123 (0x30/31/32/33/35/41)
Reed-Speed ──GPIO27─────┘   • packt SpartanCockpitFrame (17B, v2, CRC8)
                            │
        ┌───────ESP-NOW Broadcast (Ch.6)───────┐  + Hub-ASCII (BLE) + Web(192.168.4.1)
        ▼                                       ▼
   M5Dial (m5stack-123)              Waveshare (waveshare-vdo-clock)
   + optional Direct-123-BLE         + optional Direct-123-BLE
```

Datenpfad-Priorität in den Displays: **ESP-NOW → Direct-123-BLE → Hub-BLE →
WiFi-HTTP** (NVS-gesteuert; `feat_espnow`, `data_path`, `ble_mode` etc., siehe
`CODEX-HANDOFF.md`).

## Geteilte Artefakte

### ✅ Bereits kanonisch geteilt
- **`include/spartan_cockpit_frame.h`** — binäres ESP-NOW-Frame (17B, v2, CRC8,
  Encode/Decode-Helper). Datei trägt den Hinweis *„Copy this header into
  M5/Waveshare firmware"* → **single source of truth**; bei Änderungen in **alle**
  Display-Repos kopieren und `kSpartanCockpitVersion` hochziehen.

### ⚠️ Dupliziert (Dedup-Kandidat)
- **123-Frame-Decoder** (`decode123Frame` + NUS-Handshake) liegt **inline und
  mehrfach**: waveshare (`src/main.cpp:1441`), Hub, m5stack-123 — mit **leichten
  Abweichungen**:
  - Hub decodiert `0x35` (Coil `raw/8.65`); **Waveshare-Display fehlt `0x35`**.
  - Hub/M5 enthalten die `$`/`\r`-Wake- und `$`-Keepalive-Logik; im Display ist
    der Direct-Modus entsprechend abzugleichen.
- **Empfehlung (nicht beauftragt):** analog zu `spartan_cockpit_frame.h` einen
  gemeinsamen **`tune123_decode.h`** (NUS-UUIDs + ASCII-Hex-Parse + Skalierungen
  + `0x35`) herausziehen und in alle drei Firmwares kopieren — beseitigt die
  Divergenz und die `0x35`-Lücke an einer Stelle.

## Konsistenz-Checkliste bei 123-/ESP-NOW-Änderungen
1. Opcode-Tabelle (inkl. `0x35`) in allen drei Firmwares gleich? → Protokoll-Doku.
2. NUS-UUIDs + `$`/`\r`-Handshake identisch? → Handshake-Doku.
3. `SpartanCockpitFrame`-Version & Felder synchron kopiert?
4. Datenpfad-Priorität/NVS-Keys dokumentiert? → `CODEX-HANDOFF.md`.

## Verwandte Dokumente
- [`123TUNE-BLE-Protokoll.md`](123TUNE-BLE-Protokoll.md) — UUIDs, Frame, Skalierung
- [`123TUNE-HUB-HANDSHAKE.md`](123TUNE-HUB-HANDSHAKE.md) — Connect/Start/Keepalive
- [`RECHERCHE-Gauge-Projekte.md`](RECHERCHE-Gauge-Projekte.md) — externe Vorbilder/Libraries
- `AGENTS.md`, `CODEX-HANDOFF.md`, `FUTURE.md`
