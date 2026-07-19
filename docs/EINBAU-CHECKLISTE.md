# Einbau- & Fehlersuch-Checkliste (VDO-Display im T2b)

Schritt-für-Schritt beim Einbau/Anschluss abhaken. Hintergrund: Vorfall 14.7. —
Display lief einen Tag gut, dann nach Anschluss von CAN + Reed + loser Lambda-
Sonde plötzlich falsche Lambda-Werte (→ Vergaser verstellt) + Einfrieren. Ursache
waren **zwei getrennte Probleme gleichzeitig**. Diese Liste trennt sie sauber.

## Wichtigste Regel: erst gucken, was das Display sagt

Die aktuelle Firmware (ab `fae29b5`) unterscheidet die Fehlerklassen selbst —
**zuerst am Display ablesen**, dann gezielt suchen:

| Was am Display steht | Klasse | Wo suchen |
|---|---|---|
| Quelle **CAN/HTTP** + Lambda **ERR/WAIT/HEAT** oder **`--`** | Sonde/Spartan | Lambda-Sonden-Stecker, Sonde selbst |
| Quelle **SIM** | Hub simuliert | Test-/Demo-Modus am Hub aus (`lambda_test`) |
| **„kein Hub"** / Werte eingefroren auf `--` | Verbindung | CAN-Verkabelung ODER WLAN |
| Bild **schwarz / eingefroren** direkt nach Stecken | Falscher Stecker | CAN-Adapter im I2C- statt UART-Stecker |

**Merke:** Lambda-Sonde (am Spartan) und CAN-Bus (Hub↔Display) sind **elektrisch
getrennt**. Ein loser Sonden-Stecker macht falsche Lambda-Werte, aber **nicht** das
Einfrieren. Ein CAN-Problem macht „kein Hub"/Freeze, aber **nicht** falsches Lambda.
Wenn beides gleichzeitig auftritt = zwei Baustellen, getrennt abarbeiten.

## A) CAN-Bus (Hub ↔ Display, für die Fahrt)

1. **Richtiger Stecker**: CAN-Adapter am **UART**-Stecker (RXD/TXD/3V3/GND) —
   **NICHT** am gleich aussehenden I2C-Stecker daneben. Falscher Stecker stört den
   I2C-Bus → LCD-Reset/Expander spinnt → **Display schwarz/eingefroren**.
2. **Nicht kreuzen**: H↔H, L↔L (der SN65HVD230 ist ein CAN-Transceiver, kein UART!).
   GPIO43=TXD/D, GPIO44=RXD/R.
3. **Termination**: Display-Ende hat fest 120 Ω. Der **Spartan 3 terminiert das
   andere Ende ab Werk selbst** (Abschlusswiderstand default AKTIV; beim ADV per
   Serial-Kommando `SETCANR1`/`SETCANR0` schaltbar, kein Jumper). Bei nur
   Hub+Display am Bus: Spartan-Termination AN lassen = beide Enden ok.
   **ACHTUNG Messung:** Der Spartan-Abschluss ist ein ELEKTRONISCHER Widerstand —
   stromlos gemessen zeigt er ~120 Ω für ~8 s und faellt dann auf 0 (14Point7-
   Forum). Die klassische stromlose 60-Ω-Messung ist am Spartan daher NICHT
   aussagekraeftig. Verlaesslicher: Funktionstest (canRx zaehlt, rx_err=0,
   Hub-TX-Fehler stabil bei ACK-Modus).
4. **Display-CAN-Modus = NORMAL/ACK** (nicht LISTEN!). Bei nur Hub + Display am Bus
   muss das Display die Frames bestätigen, sonst sammelt der Hub tausende TX-Fehler.
   WebGUI Dev-Tab → CAN → Modus, oder Serial `can:normal`.
5. **CAN aktiv**: WebGUI/`can:on`. Prüfen: `/version` → `daten.canRx` zählt hoch,
   `daten.quelle` = „CAN".
6. Bei Busfehler heilt die FW jetzt selbst (Bus-Off-Recovery) — trotzdem obige
   Punkte prüfen, Recovery ist Notnagel, kein Ersatz für sauberen Bus.

## B) Lambda-Sonde (am Spartan/Hub, nicht am Display)

1. **Sonden-Stecker fest einrasten** — der Vorfall 14.7. war ein nicht ganz
   eingerasteter Stecker.
2. Kontrolle am Display: Lambda springt von `ERR`/`--` beim Kaltstart über
   **WAIT → HEAT → OK** (Sonde heizt auf, wenige Sek. bis <30 s), dann echter Wert.
3. Bleibt es dauerhaft `ERR`/`HEAT`: Sonde/Heizung/Verkabelung am Spartan prüfen
   (Unterspannung, defekte Heizung) — **nicht** am Display suchen.
4. **Nie** einen angezeigten Lambda-Wert zum Vergaser-Einstellen nehmen, solange
   nicht **OK** dransteht bzw. die Quelle **nicht** „SIM" ist.

## C) Display-WLAN (Datenweg + Bedienbarkeit)

1. **Im Bus: festes Hub-AP-Profil, kein Fallback** (verhindert Pendeln + Aussetzer):
   - `wauto:off` (WLAN-Auto aus) — sonst pendelt es Hub-AP↔Heim → kurze „kein Hub"
   - Profil fest **Hub-AP** (Setup → WIFI antippen bis „Hub-AP", oder WebGUI WLAN-Tab)
   - **Test-Hub AUS** (`thub:off`) — sonst zieht es die feste Test-IP statt des Hub-AP
   - Der Hub liegt dann automatisch auf `192.168.4.1` (Gateway), keine IP eintippen
2. **Freeze bei schwachem WLAN**: Der HTTP-Poll läuft aktuell im Haupt-Loop und
   blockiert ihn bei schlechtem/pendelndem WLAN → Touch/Bild frieren ein. Kurzfrist-
   Abhilfe: stabile Verbindung (fest Hub-AP, guter Empfang). **Endgültiger Fix
   (Backlog, noch nicht gebaut): Netz-Task auf Core 0** — dann kann WLAN den Loop
   nie mehr blockieren. Siehe Memory `touch-hang-http-block`.

## D) Fallbacks (am Schreibtisch/Bus meist AUS)

- **BLE-Hub**: kabelloser Reserve-Weg, nur ohne CAN-Kabel sinnvoll. Kostet Loop-Zeit
  → normal `ble:off`.
- **123TUNE+ direkt**: letzter Notweg direkt zur Zündbox (kein Lambda!). `123:off`.
- **WLAN-Auto**: nur wenn mehrere Netze automatisch gewählt werden sollen; im festen
  Bus-Betrieb aus.

## Schnell-Diagnose per HTTP (wenn Display im WLAN erreichbar)

- `http://<display>/version` → Firmware-Stand, Profil, Feature-Schalter, `canRx`/`httpRx`
- `http://<display>/live` → aktuelle Werte + Quelle + Lambda (`null` = nicht gültig)
- `http://<display>/log` → SD-Ereignis-Log (Boots, WLAN, Alarme, CAN-Bus-Off)
- `http://<display>/screen` → 160×160-Screenshot des Bildschirms
