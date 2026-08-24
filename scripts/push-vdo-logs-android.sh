#!/data/data/com.termux/files/usr/bin/bash
# VDO-Clock-Logs von unterwegs aufs NAS bringen (Termux/Android).
#
# Voraussetzung/Kette (siehe docs/EINBAU-CHECKLISTE.md fuer den Hintergrund):
#   1. Handy-Hotspot an, Display verbindet sich per WLAN-Profil "S24" (Slot 2,
#      Setup -> WIFI am Display durchtippen, oder WebGUI WLAN-Tab) mit dem Handy -
#      Display und Handy sind dann im selben lokalen Netz (Handy-Hotspot).
#   2. WireGuard-App auf dem Handy verbindet zur FritzBox (FritzOS-eigenes WireGuard,
#      QR-Code aus der FritzBox-Oberflaeche einscannen) -> Handy ist zusaetzlich im
#      Heimnetz, NAS per LAN-IP erreichbar - KEIN Split-Tunnel-Gefrickel noetig, weil
#      Hotspot (Display) und Heimnetz (NAS) fuer das Handy zwei unabhaengige, gleich-
#      zeitig nutzbare Wege sind (WLAN zum eigenen Hotspot + VPN-Tunnel fuers Heimnetz).
#   3. Dieses Skript in Termux: zieht Dateien vom Display (lokal, Hotspot-IP), legt sie
#      erst in $LOCAL_TMP ab, schiebt sie dann per scp ueber den WireGuard-Tunnel aufs
#      NAS und raeumt bei Erfolg lokal wieder auf.
#
# Termux-Pakete vorher installieren:
#   pkg install curl grep sed openssh
#
# Einmalig SSH-Key einrichten (auf der Synology unter Benutzer -> SSH-Schluessel
# hinterlegen, DSM: Systemsteuerung -> Terminal & SNMP -> SSH-Dienst aktivieren):
#   ssh-keygen -t ed25519 -f ~/.ssh/vdo_nas
#   ssh-copy-id -i ~/.ssh/vdo_nas.pub -p 22 DEIN_USER@NAS_IP
#
# Manuell starten sobald Hotspot+VPN stehen, oder per Termux:Widget/Tasker antriggern.

set -u

# ---- Konfiguration - bitte anpassen ----
DISPLAY_IP="192.168.43.42"        # IP des Displays im Handy-Hotspot-Netz (einmalig am
                                   # Display unter Setup -> WIFI ablesen, Android-Hotspots
                                   # vergeben meist eine feste erste Client-IP, kann aber
                                   # variieren - im Zweifel per WebGUI/Router-Client-Liste pruefen)
NAS_IP="192.168.178.10"           # NAS im Heimnetz (ueber den WireGuard-Tunnel erreichbar)
NAS_USER="dein_synology_user"
NAS_SSH_PORT=22
NAS_KEY="$HOME/.ssh/vdo_nas"
NAS_DEST="/volume1/vdo-clock-logs/live"   # Zielordner auf dem NAS

LOCAL_TMP="$HOME/vdo-tmp"
DIRS=("/log" "/datalog")
CURL_TIMEOUT=6

mkdir -p "$LOCAL_TMP"
log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*"; }

parse_files_json() {
  local json="$1"
  echo "$json" | grep -oE '\{"name":"[^"]*","dir":(true|false),"size":[0-9]+\}' | \
    sed -E 's/\{"name":"([^"]*)","dir":(true|false),"size":([0-9]+)\}/\1\t\2\t\3/'
}

# --- Schritt 1: Display im Hotspot-Netz erreichbar? ---
if ! curl -s --max-time "$CURL_TIMEOUT" "http://$DISPLAY_IP/version" -o /dev/null; then
  log "Display ($DISPLAY_IP) nicht erreichbar - Hotspot an? Display auf Profil S24?"
  exit 1
fi
log "Display erreichbar, ziehe Dateien"

got_any=0
for remoteDir in "${DIRS[@]}"; do
  localDir="$LOCAL_TMP$remoteDir"
  mkdir -p "$localDir"
  json="$(curl -s --max-time "$CURL_TIMEOUT" "http://$DISPLAY_IP/files?dir=$remoteDir")"
  [ -z "$json" ] && continue
  while IFS=$'\t' read -r fname isDir fsize; do
    [ -z "$fname" ] && continue
    [ "$isDir" = "true" ] && continue
    localFile="$localDir/$fname"
    localSize=0
    [ -f "$localFile" ] && localSize=$(stat -c%s "$localFile" 2>/dev/null || echo 0)
    if [ "$fsize" != "$localSize" ]; then
      if curl -s --max-time 30 "http://$DISPLAY_IP/raw?f=$remoteDir/$fname" -o "$localFile.tmp"; then
        mv "$localFile.tmp" "$localFile"
        log "  $remoteDir/$fname: $localSize -> $fsize Bytes gezogen"
        got_any=1
      else
        log "  $remoteDir/$fname: Download fehlgeschlagen"
        rm -f "$localFile.tmp"
      fi
    fi
  done < <(parse_files_json "$json")
done

if [ "$got_any" = "0" ]; then
  log "nichts Neues vom Display - fertig"
  exit 0
fi

# --- Schritt 2: NAS ueber den WireGuard-Tunnel erreichbar? ---
if ! (echo > "/dev/tcp/$NAS_IP/$NAS_SSH_PORT") 2>/dev/null; then
  log "NAS ($NAS_IP:$NAS_SSH_PORT) nicht erreichbar - WireGuard-VPN aktiv?"
  log "Dateien bleiben lokal in $LOCAL_TMP, naechster Lauf versucht's erneut"
  exit 1
fi
log "NAS erreichbar, schiebe Dateien per scp"

for remoteDir in "${DIRS[@]}"; do
  localDir="$LOCAL_TMP$remoteDir"
  [ -d "$localDir" ] || continue
  nasDir="$NAS_DEST$remoteDir"
  ssh -i "$NAS_KEY" -p "$NAS_SSH_PORT" -o StrictHostKeyChecking=accept-new \
      "$NAS_USER@$NAS_IP" "mkdir -p '$nasDir'" 2>/dev/null
  for f in "$localDir"/*; do
    [ -f "$f" ] || continue
    if scp -i "$NAS_KEY" -P "$NAS_SSH_PORT" -q "$f" "$NAS_USER@$NAS_IP:$nasDir/"; then
      log "  $(basename "$f") -> NAS $remoteDir/ hochgeladen"
    else
      log "  $(basename "$f") -> Upload fehlgeschlagen, bleibt lokal fuer naechsten Versuch"
    fi
  done
done

log "fertig"
