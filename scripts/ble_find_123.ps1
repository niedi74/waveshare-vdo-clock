# Live BLE 123-finder.
#
# Liest den ADVDUMP-Debugstrom vom Display (COM20) und markiert jedes
# Advertisement, das wie eine 123\TUNE(+)/Raytac aussieht:
#   Company 133 (BlueRadios) / 1674 (Raytac) / 2330 (Albertronic)
#   oder NUS-Service (6e400001-...) oder Name mit 123/tune.
#
# Aufruf:  powershell -ExecutionPolicy Bypass -File scripts\ble_find_123.ps1 [COM20] [Sekunden]
# Halte die 123 dabei NAH (< 1-2 m) ans Display.
param([string]$Port = "COM20", [int]$Seconds = 60)

$sp = New-Object System.IO.Ports.SerialPort $Port,115200,None,8,one
$sp.RtsEnable = $false; $sp.DtrEnable = $false   # kein Reset beim Connect
$sp.ReadTimeout = 2000
try { $sp.Open() } catch { Write-Host "OPEN-FAIL $Port : $_" -ForegroundColor Red; return }
Write-Host "Lausche auf $Port fuer $Seconds s ... 123 jetzt nah ans Display halten." -ForegroundColor Cyan

$end = (Get-Date).AddSeconds($Seconds)
$hits = @{}
$all  = @{}
while ((Get-Date) -lt $end) {
  try { $l = $sp.ReadLine() } catch { continue }
  if ($l -notmatch 'ADVDUMP (\S+) type=(\d+) rssi=(-?\d+) name=(\S+) nus=(\d) mfgN=(\d+) mfg=(\S+)') { continue }
  $addr=$matches[1]; $rssi=$matches[3]; $name=$matches[4]; $nus=$matches[5]; $mfg=$matches[7]
  $comp = -1
  if ($mfg.Length -ge 4 -and $mfg -ne "-") {
    $comp = [Convert]::ToInt32($mfg.Substring(2,2),16)*256 + [Convert]::ToInt32($mfg.Substring(0,2),16)
  }
  $all[$addr] = "rssi=$rssi comp=$comp name=$name"
  $is123 = ($comp -in 133,1674,2330) -or ($nus -eq '1') -or ($name -match '123|tune|ign')
  if ($is123) {
    $hits[$addr] = "rssi=$rssi comp=$comp name=$name mfg=$mfg"
    Write-Host ("  >>> 123-TREFFER  $addr  rssi=$rssi comp=$comp name=$name") -ForegroundColor Green
  }
}
$sp.Close()
Write-Host ""
Write-Host ("Geraete gesamt: " + $all.Count + "   123-Treffer: " + $hits.Count) -ForegroundColor Yellow
if ($hits.Count -eq 0) {
  Write-Host "Kein 123-Advert empfangen -> 123 ausser Reichweite, verbunden (Handy-App), oder stromlos." -ForegroundColor Red
} else {
  $hits.GetEnumerator() | ForEach-Object { Write-Host ("  " + $_.Key + "  " + $_.Value) -ForegroundColor Green }
}
