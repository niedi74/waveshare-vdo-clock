# AGENTS.md

## Agent Entry

This repository is the Waveshare ESP32-S3-Touch-LCD-2.8C round cockpit display.
It is not the motor-room hub. The hub lives in `niedi74/spartan3v2-can-adapter`.

## Firmware source of truth

- The firmware **trunk** is branch `claude/cranky-proskuriakova-cafad7` (canonical:
  its `HANDOFF.md`). This deep-search branch maintains **docs/research/protocol
  headers only** — no firmware features.

## Current Direction (trunk)

- Primary data path: **WiFi-HTTP `/api/status`** -> **CAN `0x510`** -> BLE-Hub
  (opt., default off) -> 123TUNE+ direct (opt., default off). **ESP-NOW is dropped.**
- WiFi auto-fallback (no scan): **Hub-AP > Home**, ~6 s each; S24 manual only.
- WLAN page (Page 11) + WPS; on-screen keyboard (Page 10) as fallback.
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
- `include/tune123_decode.h` - shared 123TUNE+ NUS decoder (UUIDs, opcodes, scaling); copy into hub/M5.
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
