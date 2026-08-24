#!/bin/bash
# Synology-Sync fuer VDO-Clock-Logs (/log + /datalog) ueber die vorhandene HTTP-API.
# Kein FTP/SMB/WebDAV auf dem ESP32 noetig (siehe docs/EINBAU-CHECKLISTE.md) - dieses
# Skript laeuft auf der Synology (Task Scheduler, User-definiertes Skript) und holt
# neue/geaenderte Dateien ab, wenn das jeweilige Display gerade im Netz erreichbar ist.
# Ist es nicht erreichbar (Bus unterwegs, Test-Board aus), bricht der Lauf still ab -
# kein Fehler, einfach beim naechsten Mal wieder versuchen.
#
# Einrichtung (Synology DSM):
#   Systemsteuerung -> Aufgabenplaner -> Erstellen -> Geplante Aufgabe -> Benutzerdefiniertes Skript
#   Zeitplan: z.B. alle 15-30 Minuten
#   Skript ausfuehren als: dein Benutzer (mit Schreibrecht auf DEST_BASE)
#   Befehl: bash /volume1/.../sync-vdo-logs.sh
#
# Reine GET-Requests, kein Login noetig (Geraet ist im eigenen WLAN).

set -u

# ---- Konfiguration ----
DEST_BASE="/volume1/vdo-clock-logs"     # Zielordner auf der Synology, bitte anpassen
DEVICES=(
  "test:192.168.0.96"
  "live:192.168.0.76"
)
DIRS=("/log" "/datalog")
CURL_TIMEOUT=6

mkdir -p "$DEST_BASE"

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"; }

# Ein Verzeichnis (JSON-Antwort von /files?dir=...) parsen: liefert Zeilen "name<TAB>size"
# Bewusst ohne jq/python (nicht auf jeder Synology vorinstalliert) - reines grep/sed auf
# das simple, flache JSON-Format {"name":"...","dir":true/false,"size":123}.
parse_files_json() {
  local json="$1"
  echo "$json" | grep -oE '\{"name":"[^"]*","dir":(true|false),"size":[0-9]+\}' | \
    sed -E 's/\{"name":"([^"]*)","dir":(true|false),"size":([0-9]+)\}/\1\t\2\t\3/'
}

for entry in "${DEVICES[@]}"; do
  name="${entry%%:*}"
  ip="${entry##*:}"

  # Erreichbarkeits-Check - /version antwortet schnell und ist leichtgewichtig.
  if ! curl -s --max-time "$CURL_TIMEOUT" "http://$ip/version" -o /dev/null; then
    log "$name ($ip): nicht erreichbar, ueberspringe"
    continue
  fi
  log "$name ($ip): erreichbar, syncen"

  for remoteDir in "${DIRS[@]}"; do
    localDir="$DEST_BASE/$name${remoteDir}"
    mkdir -p "$localDir"

    json="$(curl -s --max-time "$CURL_TIMEOUT" "http://$ip/files?dir=$remoteDir")"
    [ -z "$json" ] && continue

    while IFS=$'\t' read -r fname isDir fsize; do
      [ -z "$fname" ] && continue
      [ "$isDir" = "true" ] && continue   # keine Unterordner erwartet, aber sicherheitshalber

      localFile="$localDir/$fname"
      localSize=0
      [ -f "$localFile" ] && localSize=$(stat -c%s "$localFile" 2>/dev/null || echo 0)

      # Nur laden, wenn neu oder auf dem Geraet gewachsen (typischer Fall: heutige Datei
      # waechst waehrend des Tages). Bereits abgeschlossene Vortage werden nicht erneut
      # angefasst, sobald die Groesse einmal uebereinstimmt.
      if [ "$fsize" != "$localSize" ]; then
        if curl -s --max-time 30 "http://$ip/raw?f=$remoteDir/$fname" -o "$localFile.tmp"; then
          mv "$localFile.tmp" "$localFile"
          log "  $remoteDir/$fname: $localSize -> $fsize Bytes geladen"
        else
          log "  $remoteDir/$fname: Download fehlgeschlagen"
          rm -f "$localFile.tmp"
        fi
      fi
    done < <(parse_files_json "$json")
  done
done

log "fertig"
