#pragma once

#include <stdint.h>
#include <stddef.h>

// Shared 123TUNE+ BLE decoder for the cockpit firmwares (Spartan hub, M5 Dial,
// Waveshare). Copy this header into each firmware so the UUIDs, opcodes and
// scaling stay in sync (the same way spartan_cockpit_frame.h is shared).
//
// Transport: the real 123TUNE+ unit speaks the Nordic UART Service (NUS).
// Live data arrives as NOTIFY on TX as { Opcode, HiHexChar, LoHexChar } where
// Hi/Lo are ASCII hex characters ('0'..'9','A'..'F'); raw = (hi<<4)|lo.
// See docs/123TUNE-BLE-Protokoll.md and docs/123TUNE-HUB-HANDSHAKE.md.

// --- NUS UUIDs (string literals, usable directly with NimBLEUUID) ---
#define TUNE123_NUS_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define TUNE123_NUS_RX_UUID      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // client writes
#define TUNE123_NUS_TX_UUID      "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // device notifies

// --- Handshake constants (write logic stays in each firmware) ---
constexpr uint8_t  kTune123Wake          = 0x24;  // '$' : wake / start the stream
constexpr uint8_t  kTune123Enter         = 0x0D;  // '\r': sent once after wake
constexpr uint32_t kTune123PingIntervalMs = 1650; // re-send '$' as keepalive

// --- Opcodes (first frame byte) ---
constexpr uint8_t kTune123OpRpm     = 0x30;
constexpr uint8_t kTune123OpAdvance = 0x31;
constexpr uint8_t kTune123OpMap     = 0x32;
constexpr uint8_t kTune123OpTemp    = 0x33;
constexpr uint8_t kTune123OpCoil    = 0x35;
constexpr uint8_t kTune123OpVolt    = 0x41;

// Decoded live values. A field is only refreshed when its opcode arrives;
// tuneValid/voltValid track whether the 123-internal temp/coil resp. voltage
// have been seen.
struct Tune123Values {
  float rpm = 0;
  float advance = 0;
  float map = 0;
  float temp = 0;
  float coil = 0;
  float voltage = 0;
  bool  tuneValid = false;
  bool  voltValid = false;
};

inline int tune123HexNib(uint8_t c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0;
}

// Decode one NUS notification frame into v. Returns the handled opcode, or -1
// if the frame is too short / the opcode is unknown.
inline int tune123DecodeFrame(const uint8_t *d, size_t n, Tune123Values &v)
{
  if (d == nullptr || n < 3) return -1;
  const int hi = tune123HexNib(d[1]);
  const int lo = tune123HexNib(d[2]);
  const int raw = (hi << 4) | lo;
  switch (d[0]) {
    case kTune123OpRpm:     v.rpm = hi * 800.0f + lo * 50.0f; break;
    case kTune123OpAdvance: v.advance = hi * 3.2f + lo * 0.2f; break;
    case kTune123OpMap:     v.map = (float)raw; break;
    case kTune123OpTemp:    v.temp = (float)(raw - 30); v.tuneValid = true; break;
    case kTune123OpCoil:    v.coil = raw / 8.65f; v.tuneValid = true; break;
    case kTune123OpVolt:    v.voltage = raw / 4.54f; v.voltValid = v.voltage > 0.5f; break;
    default: return -1;
  }
  return d[0];
}
