# Cockpit-Ökosystem (niedi74) — Reuse-Matrix & Datenfluss

> Überblick, welches Repo welche Logik hält und was zwischen den Projekten
> geteilt wird/werden sollte. Bezug: VW-T2b-Cockpit mit Spartan-Hub als Gateway
> und mehreren Displays.
>
> **Firmware-Quelle = Trunk `claude/cranky-proskuriakova-cafad7`** (kanonisch:
> dessen `HANDOFF.md`). Dieser deep-search-Branch pflegt **Docs/Research/
> Protokoll-Header**, keine Firmware-Features.

## Repos & Rollen

| Repo | Rolle | 123-BLE | ESP-NOW | CAN | Anzeige |
| --- | --- | --- | --- | --- | --- |
| **waveshare-vdo-clock** *(dieses)* | VDO-Rund-Display | Client (direct/Hub, opt.) | – (gestrichen) | **0x510** (geplant) + WiFi-HTTP | ST7701 480×480 |
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

> ⚠️ **Stand veraltet (Hub-zentriert / ESP-NOW).** Der Firmware-Trunk
> `claude/cranky-proskuriakova-cafad7` (Quelle: dessen `HANDOFF.md`) hat **ESP-NOW
> gestrichen**. Aktuelle Display-Datenpfad-Priorität:
> **WiFi-HTTP `/api/status` (primär) → CAN `0x510` → BLE-Hub (opt.) → 123-direkt
> (opt.)**. Das Diagramm oben zeigt den alten ESP-NOW-Broadcast-Weg (Legacy).

## Geteilte Artefakte

### ⚠️ Legacy (ESP-NOW gestrichen)
- **`include/spartan_cockpit_frame.h`** — binäres ESP-NOW-Frame (17B, v2, CRC8).
  Der Trunk hat **ESP-NOW gestrichen**; dieser Header wird **nicht mehr gepflegt**
  und im Display-Build nicht genutzt (nur historisch relevant für den Hub).

### ✅ Neu kanonisch geteilt: `include/tune123_decode.h`
- **123-Frame-Decoder** (NUS-UUIDs + Opcode-/Handshake-Konstanten +
  ASCII-Hex-Parse + Skalierungen inkl. `0x35`) ist jetzt aus dem Inline-Code
  in **`include/tune123_decode.h`** extrahiert — gleiches „Copy this header into
  M5/Hub firmware"-Muster wie `spartan_cockpit_frame.h`.
  - Waveshare (`src/main.cpp` `decode123Frame`) nutzt den Header bereits; die
    `NUS_SVC/RX/TX`-Defines verweisen auf `TUNE123_NUS_*_UUID`.
  - **TODO (separate Repos):** Header nach `m5stack-123` und
    `spartan3v2-can-adapter` kopieren und deren Inline-Decoder darauf umstellen,
    damit alle drei garantiert synchron bleiben.
  - Handshake-Konstanten (`kTune123Wake 0x24`, `kTune123Enter 0x0D`,
    `kTune123PingIntervalMs 1650`) liegen ebenfalls im Header; die Schreib-/
    Keepalive-Logik bleibt pro Firmware.

## Konsistenz-Checkliste bei 123-Änderungen
1. Opcode-Tabelle (inkl. `0x35`) in allen drei Firmwares gleich? → am besten
   `include/tune123_decode.h` 1:1 kopieren statt nachpflegen.
2. NUS-UUIDs + `$`/`\r`-Handshake identisch? → `tune123_decode.h` / Handshake-Doku.
3. Datenpfad-Priorität (HTTP → CAN 0x510 → BLE-Hub → 123) konsistent? → Trunk
   `HANDOFF.md`.

## Verwandte Dokumente
- [`123TUNE-BLE-Protokoll.md`](123TUNE-BLE-Protokoll.md) — UUIDs, Frame, Skalierung
- [`123TUNE-HUB-HANDSHAKE.md`](123TUNE-HUB-HANDSHAKE.md) — Connect/Start/Keepalive
- [`RECHERCHE-Gauge-Projekte.md`](RECHERCHE-Gauge-Projekte.md) — externe Vorbilder/Libraries
- `AGENTS.md`, `CODEX-HANDOFF.md`, `FUTURE.md`
