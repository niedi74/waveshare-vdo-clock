# WLAN & Hub - Kurzuebersicht fuer M5 und Waveshare

> **Firmware-Quelle = Trunk `claude/cranky-proskuriakova-cafad7`** (kanonisch:
> dessen `HANDOFF.md`). Diese Doku spiegelt den Trunk-Stand; ESP-NOW ist dort
> **gestrichen**.

## Neue Ausrichtung

Der Motorraum-Hub `Spartan3-Hub` ist die zentrale Datenquelle. Im Fahrbetrieb soll er moeglichst wenig BLE-Radio-Last haben:

| Verbindung | Rolle im Fahrbetrieb |
| --- | --- |
| Spartan 3 v2 -> Hub | CAN Lambda, UART Setup |
| 123TUNE+ -> Display | Direkt-BLE am Display, wenn benoetigt |
| Hub -> Displays | **WiFi-HTTP `/api/status`** (primaer) + CAN `0x510` (geplant) |
| Displays -> Hub | HTTP `/api/status` als Fallback und fuer Zeit |
| Hub -> Internet | Optional ueber Handy-Hotspot fuer NTP |

WiFi-HTTP ist der **primaere** Live-Pfad (ESP-NOW gestrichen). **CAN `0x510`** (listen-only, RX=GPIO44/TX=GPIO43) ist die geplante Primaerquelle, sobald verkabelt.

## Profile

Trunk-Profile `g_wprof[3]`:

| Slot | Profil | SSID / PW | Hub-Host | In Auto-Kette? |
| --- | --- | --- | --- | --- |
| 0 | Heim | `Z00-Station` | Heim-LAN | ja |
| 1 | **Hub-AP** | `Spartan3-TestHub` / `lambda123` | Hub-AP (`192.168.4.x`) | **ja, primaer** |
| 2 | S24 | `Android-AP1` / `Frankfurt1` (mDNS `spartanhub.local`) | Gateway | **nur manuell** |

**Auto-Fallback** `wifiAutoTick()`: **ohne Scan** (Scan crasht), probiert
**Hub-AP > Heim** je ~6 s. Beim Boot ist der Hub-AP die primaere Datenverbindung;
S24 reisst die Auto-Kette nicht an sich. AP-Kanal folgt zwangsweise dem STA-Kanal
(ein Funkmodul) — kein Hardcode.

## Datenpfad-Prioritaet

> **ESP-NOW entfernt** (`ENABLE_ESP_NOW_CLIENT=0`). Hub-Livedaten laufen jetzt
> primaer ueber WiFi-HTTP.

1. **WiFi-HTTP** `GET /api/status` (primaer, Hub-AP bzw. Heim-LAN)
2. **CAN `0x510`** (geplant, sobald verkabelt)
3. BLE-Hub-Notify (ASCII `L..R..A..M..`, optional/default aus)
4. Direct 123TUNE+ BLE (Drehzahl/Zuendung direkt, optional/default aus)

> ESP-NOW ist **gestrichen**; `include/spartan_cockpit_frame.h` = Legacy.

## Zeit

Zeitquelle in Prioritaet:

1. Hub `/api/status` mit `ntp_synced:true` und `time_epoch`
2. Lokales NTP ueber Home/Phone WiFi
3. RTC oder Build-Zeit

Der Hub ist bis zur stabilen RTC-Versorgung der Time-Master.

## Bedienung

| Geraet | Umschalten |
| --- | --- |
| Waveshare 2.8C | Setup -> WIFI: kurz=Profil wechseln, lang=Tastatur; **WLAN-Seite Page 11** (WPS / Passwort / Profil) |
| M5 Dial | Settings/System/WiFi, Profil `Bus` fuer Fahrbetrieb |
| Hub | WebGUI, Funktionen: AP an, CAN 0x510, BLE 123/BM6 nur bei Bedarf |

## Zugangsdaten

Lokale Zugangsdaten bleiben in `src/wifi_secret.h` und werden nicht committed. Vorlage: `src/wifi_secret.example.h`.
