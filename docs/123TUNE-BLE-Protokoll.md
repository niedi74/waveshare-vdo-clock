# 123TUNE+ BLE-Protokoll (verifiziert) + iafilius-Referenz

> Was die **reale 123TUNE+-Hardware in diesem Setup** spricht — abgeleitet aus
> dem **eigenen, funktionierenden Code** (Hub + dieses Display), nicht aus
> Vermutungen. Das reverse-engineerte Repo
> [`iafilius/123Tune-plus-Simulator`](https://github.com/iafilius/123Tune-plus-Simulator)
> beschreibt eine **ältere/abweichende** Protokoll-Variante und ist hier nur als
> Referenz zur Einordnung aufgeführt — **nicht** als Quelle der UUIDs/Frames.

---

## ⚠️ Wichtigste Korrektur: NUS, nicht „Ctrl-RX/TX"

Die reale 123TUNE+-Einheit in diesem Projekt nutzt den **Nordic UART Service
(NUS)** als Transport — **nicht** die im iafilius-Repo genannten
`da2b84f1…`-/`Ctrl-RX BF03260C…`-UUIDs. Das ist verifiziert durch:
- `src/main.cpp:71` (`NUS_SVC/NUS_RX/NUS_TX`) in diesem Repo, und
- `connectTune()` im Hub (`niedi74/spartan3v2-can-adapter`, `src/main.cpp`,
  `kTuneNusServiceUuid/RxUuid/TxUuid`).

---

## 1) GATT — verifiziert (eigener Code)

| Rolle | UUID | Properties |
| --- | --- | --- |
| **NUS Service** | `6e400001-b5a3-f393-e0a9-e50e24dcca9e` | — |
| **NUS RX** (Client → 123 schreibt) | `6e400002-b5a3-f393-e0a9-e50e24dcca9e` | WRITE |
| **NUS TX** (123 → Client, Live-Daten) | `6e400003-b5a3-f393-e0a9-e50e24dcca9e` | NOTIFY |

Verbindung erfolgt per MAC (Default `ef:a8:b2:de:e0:9e`, `DEFAULT_123_MAC` in
`src/main.cpp:71`). Start-/Keepalive-Sequenz → siehe
[`123TUNE-HUB-HANDSHAKE.md`](123TUNE-HUB-HANDSHAKE.md).

> 📦 **Zentral im Code:** UUIDs, Opcodes, Skalierungen und Handshake-Konstanten
> liegen kanonisch in **`include/tune123_decode.h`** (`tune123DecodeFrame()`),
> zum Kopieren in Hub/M5. `src/main.cpp` (`NUS_*`-Defines, `decode123Frame`)
> nutzt diesen Header.

---

## 2) Live-Daten-Frame — verifiziert (eigener Code)

**Frame = `{ Opcode, Hi, Lo }`** (≥ 3 Bytes), wobei **`Hi` und `Lo` ASCII-Hex-
Zeichen** sind (`'0'..'9'`, `'A'..'F'`), die per `bleHexNib()` in 4-Bit-Nibbles
gewandelt werden. **Kein** rohes Binär-MSB/LSB.
`raw = (hi << 4) | lo`.

| Opcode | Messwert | Formel (verifiziert) | Quelle |
| --- | --- | --- | --- |
| `0x30` | **RPM** | `hi*800 + lo*50` | `main.cpp:1447` / Hub |
| `0x31` | **Advance** (°) | `hi*3.2 + lo*0.2` | `main.cpp:1448` / Hub |
| `0x32` | **MAP** (kPa) | `raw` | `main.cpp:1449` / Hub |
| `0x33` | **Temperatur** (°C) | `raw − 30` | `main.cpp:1450` / Hub |
| `0x35` | **Zündspulen-Strom** (A) | `raw / 8.65` | `main.cpp` (`decode123Frame`) / Hub |
| `0x41` | **Bordspannung** (V) | `raw / 4.54` | `main.cpp:1454` / Hub |

> ✅ **Lücke geschlossen:** `decode123Frame()` (`src/main.cpp`) decodiert `0x35`
> jetzt ebenfalls (`g_g123Coil = raw / 8.65f`); `applyEspNowFrame()` übernimmt den
> Coil-Strom aus dem ESP-NOW-Frame (`spartanCockpitCoil()`). Damit ist der
> Zündspulen-Strom auf **allen** Pfaden (Direct-123, ESP-NOW, Hub-ASCII) erfasst
> und wird auf der Motor-Seite (`COIL`) sowie im Web-Dashboard/`/api/status`
> (`coil`/`coil_valid`) angezeigt.

---

## 3) Hub-ASCII-Payload (Hub-BLE-Relay) — verifiziert

Wenn das Display **am Hub** hängt (statt direkt am 123), liefert der Hub eine
**ASCII-Zeile** (geparst in `parseSpartanPayload()`, `src/main.cpp:1469`):

```
L{lambda}R{rpm}A{adv}M{map}[V{volt}][W{auxVolt}][S{speedKmh}][I{123volt}T{123temp}C{123coil}]
```
Beispiel: `L0.95R2500A15M45V13.5S65I12.5T85C3.2`
- Pflichtteil `L…R…A…M…`; `V/W/S/I…T…C…` optional.
- Das `I…T…C…`-Segment trägt die 123-internen Volt/Temp/Coil.

---

## 4) ESP-NOW-Frame (LEGACY — Pfad gestrichen)

> ⚠️ Der Firmware-Trunk `claude/cranky-proskuriakova-cafad7` hat **ESP-NOW
> gestrichen**. Dieser Abschnitt + `spartan_cockpit_frame.h` sind nur noch
> historisch; primärer Pfad ist **WiFi-HTTP `/api/status` → CAN `0x510`**.
> Der NUS-Decoder/`tune123_decode.h` (oben) bleibt gültig und kanonisch.

Binäres 17-Byte-Frame `SpartanCockpitFrame` (`include/spartan_cockpit_frame.h`,
v2, Magic `0x53`, CRC8). Felder u.a.: `lambda_x1000`, `rpm`, `advance_x10`,
`map`, `tune_volt_x10`, `tune_temp_c`, `tune_coil_x10`, `flags`
(`LambdaValid/TuneFresh/TuneConnected`). Dieser Header ist die **kanonische,
zwischen Hub/M5/Waveshare kopierte** Definition (Datei sagt explizit *„Copy this
header into M5/Waveshare firmware"*).

Datenpfad-Priorität im Display: **ESP-NOW → Direct-123-BLE → Hub-BLE →
WiFi-HTTP** (siehe `AGENTS.md`/`CODEX-HANDOFF.md`).

---

## 5) Referenz: iafilius-Repo (ältere/abweichende Variante)

Nur zur Einordnung — **diese UUIDs/Frames entsprechen NICHT der hier genutzten
Hardware** (die spricht NUS, s. o.):
- Service `da2b84f1-6279-48de-bdc0-afbea0226079`; Characteristics
  Ctrl-TX `18CDA784-…` (NOTIFY), Ctrl-RX `BF03260C-…` (WRITE), Info `99564A02-…`,
  Body `A87988B9-…`; Advertising-Name `123\TUNE+`, Flags `0x1a`.
- Beschreibt ein 5-Byte-PDU-Modell mit Checksum + Kurven (Body), `v@`-Version,
  Pincode, „noop `0x24`", unverschlüsselt.
- **Praxisnutzen:** Konzepte (Kurven-Read/Write, Versionsabfrage) als Ideen;
  das `0x24`-Byte taucht in **unserer** Welt als **Keepalive-Ping** wieder auf
  (s. Handshake-Doku), nicht als Frame-Terminator.

> TODO verify (optional): ob neuere 123-Firmware den `da2b84f1`-Service parallel
> anbietet. Für dieses Setup irrelevant — NUS ist der bestätigte Pfad.

## Quellen
- `src/main.cpp:71` (UUIDs/MAC), `:1441` `decode123Frame`, `:1469`
  `parseSpartanPayload` · `include/spartan_cockpit_frame.h`
- `niedi74/spartan3v2-can-adapter` `src/main.cpp` (`connectTune`, Frame-Decode inkl. `0x35`)
- https://github.com/iafilius/123Tune-plus-Simulator (Referenz, ältere Variante)
