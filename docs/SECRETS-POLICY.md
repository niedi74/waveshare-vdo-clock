# Private Zugangsdaten — Ablage & Einspielen

> **Grundsatz:** Echte Passwörter/Schlüssel **nie** in dieses (geteilte) Repo —
> weder in eine Datei noch in einen „privaten" Branch (Branches sind **nicht**
> zugriffsgeschützt und landen dauerhaft in der History). Private Daten leben in
> einem **separaten privaten Repo** und werden lokal in die gitignored Pfade
> kopiert. Diese Datei beschreibt nur **wo** und **wie** — sie enthält **keine
> echten Werte**.

## Was ist „privat"
- WiFi-Passwörter: Heim (`Z00-Station`), Handy/S24-Hotspot, ggf. Hub-AP.
- Ggf. Geräte-MACs (falls nicht als Default im Code) — 123-Verteiler, Hub, BM6.

> Hinweis: Manche Defaults (Hub-AP-PW, 123-MAC) stehen bereits als `#define` im
> Quellcode. Was **wirklich** geheim ist (deine Heim-/Hotspot-Passwörter), gehört
> ausschließlich ins private Repo bzw. die gitignored Dateien.

## Wo jedes Projekt die Daten erwartet
| Projekt | Datei (gitignored) | Vorlage | Felder |
| --- | --- | --- | --- |
| **waveshare-vdo-clock** | `src/wifi_secret.h` | `src/wifi_secret.example.h` | `WIFI_SSID`, `WIFI_PASSWORD`, `S24_AP_PASS` |
| **m5stack-123** | `src/wifi_secret.h` | analog | analog |
| **spartan3v2-can-adapter** | `src/wifi_secret.h` | analog | analog |

Zusätzlich (Gerät, nicht Repo): `wifi.txt` im **LittleFS** (Profile) — optionaler
Seed über `data/wifi.txt`. In allen Projekten via `.gitignore` gesperrt:
`src/wifi_secret.h`, `wifi.txt`, `data/wifi.txt`, `**/wifi.txt`.

## Privates Repo `cockpit-secrets` (private!)
Eine Quelle für alle drei Cockpit-Projekte. Vorschlag-Struktur:
```
cockpit-secrets/            (GitHub: niedi74/cockpit-secrets, PRIVATE)
├─ waveshare/wifi_secret.h  # echte Werte
├─ m5/wifi_secret.h
├─ hub/wifi_secret.h
├─ macs.h                   # optional, gemeinsame MAC-Defines
├─ setup.sh                 # kopiert in die Projekt-Pfade (Linux/Mac)
└─ setup.ps1                # dito (Windows)
```

setup.ps1 (Windows, Beispiel — Pfade anpassen):
```powershell
$root = "D:\_claude"
Copy-Item waveshare\wifi_secret.h "$root\waveshare-vdo-clock\src\wifi_secret.h" -Force
Copy-Item m5\wifi_secret.h        "$root\m5stack-123\src\wifi_secret.h" -Force
Copy-Item hub\wifi_secret.h       "$root\spartan3v2-can-adapter\src\wifi_secret.h" -Force
Write-Host "Secrets eingespielt."
```

setup.sh (Linux/Mac):
```bash
#!/usr/bin/env bash
set -e
ROOT="${1:-$HOME/projects}"
cp waveshare/wifi_secret.h "$ROOT/waveshare-vdo-clock/src/wifi_secret.h"
cp m5/wifi_secret.h        "$ROOT/m5stack-123/src/wifi_secret.h"
cp hub/wifi_secret.h       "$ROOT/spartan3v2-can-adapter/src/wifi_secret.h"
echo "Secrets eingespielt."
```

## Bootstrap (einmalig, privat anlegen)
```bash
# 1) Privates Repo anlegen (per gh-CLI oder GitHub-Web "New repository, Private")
gh repo create niedi74/cockpit-secrets --private

# 2) Lokal befuellen
mkdir cockpit-secrets && cd cockpit-secrets
# wifi_secret.h je Projekt anlegen (aus den *.example.h-Vorlagen), echte Werte eintragen
git init && git add . && git commit -m "cockpit secrets (private)"
git branch -M main
git remote add origin https://github.com/niedi74/cockpit-secrets.git
git push -u origin main
```

## Ablauf beim Auschecken eines Projekts
```bash
git clone .../waveshare-vdo-clock && cd waveshare-vdo-clock
# Secrets einspielen:
../cockpit-secrets/setup.sh          # bzw. setup.ps1
pio run -e waveshare_s3_28c
```

> Ich (dieser Chat) kann das private Repo **nicht** anlegen — mein GitHub-Zugriff
> ist auf `waveshare-vdo-clock` beschränkt. Fuehre die Bootstrap-Befehle bei dir
> aus; diese Doku ist die Anleitung dazu.
