# WLAN & Hub - Kurzuebersicht fuer M5 und Waveshare

## Neue Ausrichtung

Der Motorraum-Hub `Spartan3-Hub` ist die zentrale Datenquelle. Im Fahrbetrieb soll er moeglichst wenig BLE-Radio-Last haben:

| Verbindung | Rolle im Fahrbetrieb |
| --- | --- |
| Spartan 3 v2 -> Hub | CAN Lambda, UART Setup |
| 123TUNE+ -> Display | Direkt-BLE am Display, wenn benoetigt |
| Hub -> Displays | ESP-NOW Broadcast, Kanal 6 |
| Displays -> Hub | HTTP `/api/status` als Fallback und fuer Zeit |
| Hub -> Internet | Optional ueber Handy-Hotspot fuer NTP |

ESP-NOW ist fuer Live-Daten der bevorzugte Weg. HTTP bleibt wichtig fuer Diagnose, Setup und `time_epoch`.

## Profile

| Profil | SSID | Display-IP | Hub-Host | Zweck |
| --- | --- | --- | --- | --- |
| Home | `Z00-Station` | DHCP | `192.168.0.87` oder gespeicherter Host | Werkstatt, Debug, OTA |
| Phone | `Android-AP1` | DHCP | Gateway-IP oder gespeicherter Host | Unterwegs mit Handy-Uplink |
| Bus | `Spartan3-Setup` | `192.168.4.3` | `192.168.4.1` | Fahrtprofil, ESP-NOW ch6 + HTTP-Fallback |

M5 Dial nutzt im Bus-Profil `.2`, Waveshare nutzt `.3`, der Hub bleibt `.1`.

## Datenpfad-Prioritaet

> **ESP-NOW entfernt** (`ENABLE_ESP_NOW_CLIENT=0`). Hub-Livedaten laufen jetzt
> primaer ueber WiFi-HTTP.

1. WiFi HTTP `GET /api/status` (primaer, Hub-AP `192.168.4.1` oder Heim-LAN)
2. BLE Hub-Notify (ASCII `L..R..A..M..`)
3. Direct 123TUNE+ BLE (Drehzahl/Zuendung direkt vom Verteiler)
4. RTC/Build-Zeit als letzter Rueckfall

> Hinweis: `include/spartan_cockpit_frame.h` bleibt fuer den Hub relevant, wird
> im Display-Build aber nicht mehr genutzt.

## Zeit

Zeitquelle in Prioritaet:

1. Hub `/api/status` mit `ntp_synced:true` und `time_epoch`
2. Lokales NTP ueber Home/Phone WiFi
3. RTC oder Build-Zeit

Der Hub ist bis zur stabilen RTC-Versorgung der Time-Master.

## Bedienung

| Geraet | Umschalten |
| --- | --- |
| Waveshare 2.8C | Touch Setup oder WebGUI, Profil `Bus` fuer Fahrbetrieb |
| M5 Dial | Settings/System/WiFi, Profil `Bus` fuer Fahrbetrieb |
| Hub | WebGUI `Spartan3-Setup`, Funktionen: ESP-NOW an, AP an, BLE 123/BM6 nur bei Bedarf |

## Zugangsdaten

Lokale Zugangsdaten bleiben in `src/wifi_secret.h` und werden nicht committed. Vorlage: `src/wifi_secret.example.h`.
