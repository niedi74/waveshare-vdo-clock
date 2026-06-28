# 123TUNE+ BLE — Verbindungs- & Start-Handshake (Hub/Client)

> Wie ein BLE-Client (Spartan-Hub **oder** Display im Direct-123-Modus) den
> 123TUNE+ verbindet und den **Live-Daten-Stream zuverlässig startet**.
> Quelle = **eigener, funktionierender Code** (`connectTune()` im Hub
> `niedi74/spartan3v2-can-adapter`, `src/main.cpp`). Genau diese Sequenz war der
> Durchbruch, der sich erst nach Auswertung der Original-App ergab.
>
> Wichtig: Die Hardware spricht **Nordic UART Service (NUS)**, nicht die
> iafilius-„Ctrl-RX/TX"-UUIDs. Details:
> [`123TUNE-BLE-Protokoll.md`](123TUNE-BLE-Protokoll.md).

---

## UUIDs (NUS)
- Service `6e400001-b5a3-f393-e0a9-e50e24dcca9e`
- **RX** (Client schreibt) `6e400002-b5a3-f393-e0a9-e50e24dcca9e`
- **TX** (123 notifyt Live-Daten) `6e400003-b5a3-f393-e0a9-e50e24dcca9e`

## Ablauf `connectTune()` (verifiziert)
1. **Verbinden** zur 123-MAC (Default `ef:a8:b2:de:e0:9e`).
2. Service/Characteristics discovern (NUS RX + TX holen).
3. **CCCD kurz aus** — `0x0000` auf Descriptor `0x2902` der TX-Characteristic
   schreiben (Notifications definiert zurücksetzen).
4. **TX abonnieren**: `tx->subscribe(true, onTuneNotify, true)`.
5. **Wake-Sequenz auf RX** schreiben:
   - Byte `'$'` (`0x24`), dann
   - `delay(120)`, dann
   - Byte `'\r'` (`0x0D`).
   ```cpp
   const uint8_t ping = '$';   // 0x24
   const uint8_t enter = '\r'; // 0x0D
   tuneNusRx->writeValue(&ping, 1, true);
   delay(120);
   tuneNusRx->writeValue(&enter, 1, true);
   ```
6. **Keepalive**: alle **1650 ms** (`kTunePingIntervalMs`) erneut `'$'` (`0x24`)
   auf RX schreiben, sonst versiegt der Stream.
   ```cpp
   void sendTunePing() { const uint8_t ping = '$'; tuneNusRx->writeValue(&ping, 1, true); }
   ```

## Datenempfang
Auf der TX-Notify kommen Frames `{ Opcode, HiHexChar, LoHexChar }` (ASCII-Hex-
Nibbles). Decodierung + Skalierung → siehe Protokoll-Doku, Abschnitt 2.

## Auswertung pro Projekt
- **Spartan-Hub** (`spartan3v2-can-adapter`): nutzt diese Sequenz bereits; sie ist
  hier die Referenz. Hub decodiert zusätzlich `0x35` (Coil) und sendet alles per
  ESP-NOW + Hub-ASCII weiter.
- **VDO-Display** (dieses Repo, Direct-123-Modus): nutzt denselben Mechanismus
  (`bleNotify123CB` → `decode123Frame`). Falls der Direct-Modus mal nicht
  streamt, zuerst die **`$`-Keepalive** prüfen — fehlender Ping ist die
  wahrscheinlichste Ursache.
- **M5Dial** (`m5stack-123`): gleiche NUS-/`$`-Logik (Original-Reverse-Engineering
  des Nutzers). Bei Abweichungen ist `m5stack-123` die Vergleichsquelle.

## Offene Punkte (TODO verify im Quellcode bei Bedarf)
- Ob ein **Pincode** vor dem Streaming nötig ist (in diesem Setup offenbar nicht
  — Wake = `$`/`\r` genügt; iafilius erwähnt Pincode-Validierung für seine
  ältere Variante).
- Exakte Bedeutung des `\r` nach `$` (Bestätigung/Zeilenende) vs. reiner Wake.

## Quellen
- `niedi74/spartan3v2-can-adapter` `src/main.cpp` — `connectTune()`, `sendTunePing()`
- Dieses Repo: `src/main.cpp:1524` `bleNotify123CB`, `:1441` `decode123Frame`
- Referenz/Einordnung: https://github.com/iafilius/123Tune-plus-Simulator
