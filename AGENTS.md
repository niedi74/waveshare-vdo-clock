# AGENTS.md

## Agent Entry

This repository is the Waveshare ESP32-S3-Touch-LCD-2.8C round cockpit display.
It is not the motor-room hub. The hub lives in `niedi74/spartan3v2-can-adapter`.

## Current Direction

- Primary driving data path: ESP-NOW broadcast from `Spartan3-Hub` on channel 6.
- Fallback/status path: WiFi HTTP poll of `http://192.168.4.1/api/status` on the bus profile.
- BLE to `Spartan3-Hub` is only a fallback/debug path.
- Direct 123TUNE+ BLE can stay for standalone testing, but the normal bus setup should avoid competing with M5/Hub BLE.
- Hub time is the time master when `/api/status` reports `ntp_synced:true` and `time_epoch`.

## Build

```powershell
pio run -e waveshare_s3_28c
pio run -e waveshare_s3_28c -t upload --upload-port COM13
pio device monitor --port COM13 --baud 115200
```

## Important Files

- `src/main.cpp` - app, touch menu, WiFi/HTTP, ESP-NOW, BLE fallback, WebGUI.
- `include/spartan_cockpit_frame.h` - shared ESP-NOW binary frame copied from the hub repo.
- `platformio.ini` - COM13, ESP32-S3 2.8C, ESP-NOW channel 6.
- `docs/WLAN-MATRIX.md` - operational profiles for Home, Phone, and Bus.
- `docs/123TUNE-BLE-Protokoll.md` - verified 123TUNE+ BLE (NUS) UUIDs, frame & scaling.
- `docs/123TUNE-HUB-HANDSHAKE.md` - 123 BLE connect/start/keepalive sequence.
- `docs/COCKPIT-ECOSYSTEM.md` - cross-repo reuse matrix & data flow (niedi74 cockpit repos).

## Git

Expected origin:

```text
https://github.com/niedi74/waveshare-vdo-clock.git
```

The old ESP32-S3-Touchscreen/Copit repo is a different project and should not be used as this repo's remote.
