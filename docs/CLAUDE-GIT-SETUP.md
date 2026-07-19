# Claude auf dem Server mit Git arbeiten lassen

Kurzanleitung, damit ein Claude-/Hermes-Agent auf deinem Server dieses Repo
eigenständig bearbeiten, committen und pushen kann.

## 1. Voraussetzungen auf dem Server

- **Git** installiert (`git --version`)
- **GitHub CLI** `gh` installiert (für Releases/PRs): https://cli.github.com
- **Claude Code** (CLI) installiert und angemeldet
- **PlatformIO** (nur wenn Firmware gebaut werden soll): `pip install platformio`

## 2. Einmalige Git-Einrichtung

```bash
# Identität (erscheint in den Commits)
git config --global user.name  "niedi74"
git config --global user.email "Karsten@niederhaeuser.de"

# GitHub-Login für push + gh (Token mit repo-Rechten)
gh auth login          # interaktiv, einmalig
# ODER per Token (headless):
echo "$GITHUB_TOKEN" | gh auth login --with-token
git remote set-url origin https://github.com/niedi74/waveshare-vdo-clock.git
```

> **Wichtig:** Der Token braucht `repo`-Scope (Push) und `workflow`, wenn Releases
> hochgeladen werden. Token NIE ins Repo committen — nur als Env-Variable/Secret.

## 3. Repo holen

```bash
git clone https://github.com/niedi74/waveshare-vdo-clock.git
cd waveshare-vdo-clock
claude          # Claude Code im Repo-Verzeichnis starten
```

Claude liest beim Start automatisch `AGENTS.md` (Projekt-Briefing) und die Dateien
unter `docs/`. Damit kennt der Agent die Regeln, Geräte-IPs und Invarianten.

## 4. Arbeits-Workflow (so arbeitet Claude im Repo)

1. **Vor der Arbeit** aktuellen Stand holen:
   ```bash
   git pull origin main
   ```
2. **Ändern** (Claude editiert `src/main.cpp` etc.), dann **prüfen**:
   ```bash
   git status
   git diff
   ```
3. **Committen** — deutsche Message, Anlass → Fix → Verifikation, mit Co-Author:
   ```bash
   git add <geänderte Dateien>     # gezielt, nicht blind "git add ."
   git commit -m "Kurzbeschreibung

   Details ...

   Co-Authored-By: Claude <noreply@anthropic.com>"
   ```
4. **Pushen** — Branch UND main (dieses Projekt hält main = Arbeitsstand):
   ```bash
   git push
   git push origin HEAD:main
   ```

## 5. Firmware-Release (Standing Order nach jedem Code-Commit)

Nur wenn sich die Firmware geändert hat (nicht bei reinen Doku-Commits):

```bash
# aus SAUBEREM committeten Stand bauen (wegen GIT_REV/RTC-Zeit)
pio run -e waveshare_s3_28c

# Binaries ins Release fw-live-latest (rollend + hash-benannt, Rollback-Historie)
HASH=$(git rev-parse --short HEAD)
cp .pio/build/waveshare_s3_28c/firmware.bin "/tmp/firmware-$HASH.bin"
gh release upload fw-live-latest .pio/build/waveshare_s3_28c/firmware.bin --clobber
gh release upload fw-live-latest "/tmp/firmware-$HASH.bin" --clobber
gh release edit fw-live-latest --title "FW Live Display ($HASH) - <kurz>"
```

Alte Release-Assets **nie löschen** (Rollback). Details/Regeln: `AGENTS.md`.

## 6. Sicherheits-Regeln für den Server-Agenten

- **Keine Secrets committen**: `src/wifi_secret.h`, SD-`wifi.txt` sind gitignored —
  so lassen. Vor jedem Push kurz prüfen, dass keine Zugangsdaten im Diff sind.
- **Live-Geräte** (Display .76, Hub .71/.91) nur nach ausdrücklicher Freigabe
  flashen/verstellen — der Server-Agent hat i.d.R. eh keinen Netzzugriff dorthin.
- **Destruktive Git-Aktionen** (`reset --hard`, `push --force`, `clean`) nur mit
  vorherigem Blick auf `git status`; sonst neue Commits statt Historie umschreiben.

## 7. Optional: Hermes-Projekt

Repo als Hermes-Projekt einbinden (Beschreibung siehe `hermes project create
--description ...`, Text im Projekt-Chat). Hermes klont das Repo; die `AGENTS.md`
im Root übernimmt dann das Onboarding jedes Agenten automatisch.
