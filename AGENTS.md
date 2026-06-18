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

## Git

Expected origin:

```text
https://github.com/niedi74/waveshare-vdo-clock.git
```

The old ESP32-S3-Touchscreen/Copit repo is a different project and should not be used as this repo's remote.

## Cursor Cloud specific instructions

- This is ESP32-S3 firmware built with PlatformIO. There is no host application to run; the
  "application" is firmware that is flashed onto the physical Waveshare board over `COM13`. That
  hardware is not present in the cloud VM, so `-t upload` and `pio device monitor` cannot be used
  here. The build (`pio run -e waveshare_s3_28c`) is the end-to-end verification in this environment.
- `pio` is installed in a venv at `~/.pio-venv` and symlinked onto `PATH` (`/usr/local/bin/pio`).
  The startup update script recreates/refreshes it, so just run `pio ...` directly.
- First build downloads the pioarduino espressif32 platform + toolchain + NimBLE (a few minutes);
  incremental rebuilds are ~20s. Build artifacts land in `.pio/build/waveshare_s3_28c/`
  (`firmware.bin`, `firmware.elf`), which is gitignored.
- `scripts/inject_time.py` (pre) injects the host build time as `RTC_BUILD_*` flags; the build is
  therefore non-deterministic by design (RTC seed changes each build) — this is expected.
- There is no separate lint step configured; `pio run` (compiler warnings/errors) is the check.
- `platformio.ini` pins `upload_port`/`monitor_port` to `COM13` (Windows host). These are ignored
  for plain `pio run`.
