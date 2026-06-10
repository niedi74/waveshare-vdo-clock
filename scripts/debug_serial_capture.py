"""Capture NDJSON debug lines from ESP32 serial into debug-bd0449.log."""
import json
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial required: pip install pyserial", file=sys.stderr)
    sys.exit(1)

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM13"
DURATION = int(sys.argv[2]) if len(sys.argv) > 2 else 30
LOG_PATH = sys.argv[3] if len(sys.argv) > 3 else "debug-bd0449.log"

ser = serial.Serial(PORT, 115200, timeout=0.2)
end = time.time() + DURATION
count = 0
with open(LOG_PATH, "a", encoding="utf-8") as log:
    while time.time() < end:
        line = ser.readline().decode("utf-8", errors="replace").strip()
        if not line.startswith("{"):
            continue
        try:
            json.loads(line)
            log.write(line + "\n")
            log.flush()
            count += 1
            print(line)
        except json.JSONDecodeError:
            pass
ser.close()
print(f"Captured {count} debug lines -> {LOG_PATH}")
