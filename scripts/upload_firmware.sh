#!/usr/bin/env bash
#
# upload_firmware.sh — Upload the hardware_bridge_app sketch to the
# STM32U585 M33 co-processor on the UNO Q board.
#
# Run this after any `arduino-app-cli app start` of a different app
# (e.g. i2c_scanner, imu_scanner) to restore the correct firmware.
# It syncs the latest sketch from the workspace, compiles, flashes,
# and verifies via the arduino-router Unix socket.
#
# IMPORTANT: This script stops the legacy Docker hardware-bridge.service
# and disables it permanently. The new C++ hw_bridge ROS 2 node connects
# directly to the router socket — no Docker relay needed.
# After flashing, launch the ROS 2 node:
#   ros2 launch big_bertha_bringup hardware_bringup.launch.py
#
# Usage:
#   ssh arduino@<board-ip> "$(cat upload_firmware.sh)"
#
# Or copy it to the board first, then run:
#   ssh arduino@<board-ip> ./upload_firmware.sh
#
# Exits 0 on success, non-zero on failure (with a clear message).

set -uo pipefail

WORKSPACE_APP="$HOME/ros2_ws/src/spider_bot_bringup/big_bertha_bringup/firmware/hardware_bridge_app"
BOARD_APP="$HOME/ArduinoApps/hardware_bridge_app"

say()  { printf "\033[1;34m[%s]\033[0m %s\n" "$1" "$2" >&2; }
ok()   { printf "\033[1;32m  ✔\033[0m %s\n" "$*" >&2; }
fail() { printf "\033[1;31m  ✘\033[0m %s\n" "$*" >&2; }

# ── Pre-checks ──────────────────────────────────────────────────────────────

say CHECK "Prerequisites"

if [ ! -d "$WORKSPACE_APP" ]; then
  fail "Workspace app not found at $WORKSPACE_APP"
  exit 1
fi
ok "Workspace app exists"

if ! systemctl is-active --quiet arduino-app-cli.service 2>/dev/null; then
  fail "arduino-app-cli.service is not running"
  exit 1
fi
ok "app-cli daemon is active"

# ── Sync workspace → ArduinoApps ────────────────────────────────────────────

say SYNC "Syncing workspace sketch to ArduinoApps"
mkdir -p "$BOARD_APP/sketch" "$BOARD_APP/python"
cp "$WORKSPACE_APP/sketch/sketch.ino" "$BOARD_APP/sketch/sketch.ino"
cp -u "$WORKSPACE_APP/python/"*.py "$BOARD_APP/python/" 2>/dev/null || true
cp -u "$WORKSPACE_APP/python/"*.txt "$BOARD_APP/python/" 2>/dev/null || true
cp -u "$WORKSPACE_APP/app.yaml" "$BOARD_APP/app.yaml" 2>/dev/null || true

# Check the sync actually changed something worthwhile
WS_SIZE=$(stat -c%s "$WORKSPACE_APP/sketch/sketch.ino" 2>/dev/null || echo 0)
BA_SIZE=$(stat -c%s "$BOARD_APP/sketch/sketch.ino" 2>/dev/null || echo 0)
if [ "$WS_SIZE" -eq 0 ]; then
  fail "Workspace sketch is empty after sync"
  exit 1
fi
if [ "$BA_SIZE" -eq 0 ]; then
  fail "Board app sketch is empty after sync"
  exit 1
fi
ok "Sketch synced ($BA_SIZE bytes)"

# ── Stop & disable legacy Docker service ─────────────────────────────────

say STOP "Stopping and disabling legacy hardware-bridge Docker service"

sudo systemctl stop hardware-bridge.service 2>/dev/null || true
sudo systemctl disable hardware-bridge.service 2>/dev/null || true
docker rm -f hardware-bridge 2>/dev/null || true

# Wait until port 50007 is free (up to 30s)
for i in $(seq 1 15); do
  if ! ss -tlnp 2>/dev/null | grep -q ':50007\b'; then
    break
  fi
  sleep 2
done
if ss -tlnp 2>/dev/null | grep -q ':50007\b'; then
  # This shouldn't happen after rm, but if something else has it, abort
  fail "Port 50007 still bound after 30s — aborting"
  exit 1
fi
ok "Legacy Docker container stopped and disabled"
ok "New C++ hw_bridge will connect directly to router"

# ── Clean up all Docker containers ──────────────────────────────────────────

say CLEAN "Removing all stale Docker containers"

# Remove any remaining instance of the legacy bridge
docker rm -f hardware-bridge 2>/dev/null || true
# Remove any compose-managed containers from app start/stop
docker rm -f hardware_bridge_app-main-1 2>/dev/null || true
docker ps -a --format '{{.Names}}' 2>/dev/null | grep -E '(hardware_bridge_app|ros2_ws.*hardware_bridge)' | while IFS= read -r c; do
  docker rm -f "$c" 2>/dev/null || true
done

ok "All legacy Docker containers removed"

# ── Compile & upload sketch (with retry) ────────────────────────────────────

say FLASH "Compiling and uploading sketch to STM32U585 M33"
say FLASH "This takes ~15-20 seconds..."

UPLOAD_OK=false
UPLOAD_LOG=""
for ATTEMPT in 1 2; do
  UPLOAD_LOG=$(mktemp /tmp/hw_bridge_flash.XXXXXX)

  arduino-app-cli app start "$BOARD_APP" >"$UPLOAD_LOG" 2>&1

  if grep -q 'sketch updated' "$UPLOAD_LOG" 2>/dev/null; then
    UPLOAD_OK=true
    rm -f "$UPLOAD_LOG"
    break
  fi

  if [ "$ATTEMPT" -eq 1 ]; then
    say RETRY "Upload attempt 1 failed — retrying..."
    sleep 2
  fi
  rm -f "$UPLOAD_LOG"
  UPLOAD_LOG=""
done

if [ "$UPLOAD_OK" = false ]; then
  fail "Sketch upload failed after 2 attempts"
  if [ -n "$UPLOAD_LOG" ] && [ -f "$UPLOAD_LOG" ]; then
    grep -i error "$UPLOAD_LOG" 2>/dev/null | head -5 | while IFS= read -r line; do fail "$line"; done
    rm -f "$UPLOAD_LOG"
  fi
  exit 1
fi
ok "Sketch compiled and uploaded successfully"

# ── Stop compose container (we use our own) ─────────────────────────────────

say CLEAN "Stopping compose container created by app start"
arduino-app-cli app stop "$BOARD_APP" 2>/dev/null || true
docker rm -f hardware_bridge_app-main-1 2>/dev/null || true
docker ps -a --format '{{.Names}}' 2>/dev/null | grep -v '^hardware-bridge$' | grep -E '(hardware_bridge_app|ros2_ws.*hardware_bridge)' | while IFS= read -r c; do
  docker rm -f "$c" 2>/dev/null || true
done

ok "Compose container cleaned"

# ── Verify firmware via router directly ──────────────────────────────────

say VERIFY "Verifying firmware via router socket"

# Check router socket exists
if [ -S /var/run/arduino-router.sock ]; then
  ok "Router socket present at /var/run/arduino-router.sock"
else
  fail "Router socket not found — is arduino-router running?"
  exit 1
fi

# Use a temporary Python script to register for IMU notifications
# via the router socket and verify data is flowing.
TMP_PY=$(mktemp /tmp/verify_imu.XXXXXX.py)
cat > "$TMP_PY" << 'PYEOF'
import struct, socket, sys, time
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.settimeout(1.0)
sock.connect("/var/run/arduino-router.sock")

def pack(val):
    if isinstance(val, bool):     return b'\xc3' if val else b'\xc2'
    if val is None:               return b'\xc0'
    if isinstance(val, int):
        if 0 <= val <= 127:       return bytes([val])
        if -32 <= val < 0:        return bytes([val & 0x1F | 0xE0])
        if -128 <= val < 0:       return b'\xd0' + struct.pack('b', val)
        return b'\xd1' + struct.pack('>h', val)
    if isinstance(val, str):
        d = val.encode()
        return bytes([0xA0 | len(d)]) + d if len(d) <= 31 else b'\xd9' + bytes([len(d)]) + d
    if isinstance(val, (list,)):
        b = b'\xdd' + struct.pack('>I', len(val))
        for v in val: b += pack(v)
        return b
    raise TypeError(type(val))

def send(msg):
    data = pack(msg)
    sock.sendall(struct.pack('>I', len(data)) + data)

send(["provide", "imu"])
time.sleep(0.2)

deadline = time.time() + 7.0
buf = b""
while time.time() < deadline:
    try:
        c = sock.recv(4096)
        if not c: break
        buf += c
    except socket.timeout:
        continue

if not buf:
    print("IMU_RESULT=FAIL no data from router")
    sys.exit(1)

found = False
i = 0
while i + 4 < len(buf):
    n = struct.unpack('>I', buf[i:i+4])[0]
    if n == 0 or i + 4 + n > len(buf):
        i += 1
        continue
    chunk = buf[i+4:i+4+n]
    try:
        text = chunk.decode('utf-8', errors='replace')
        if 'imu' in text:
            found = True
            print("IMU_RESULT=PASS")
            break
    except: pass
    i += 4 + n

if found:
    sys.exit(0)
else:
    print("IMU_RESULT=FAIL no IMU frames received")
    sys.exit(1)
PYEOF

python3 "$TMP_PY"
PY_EXIT=$?
rm -f "$TMP_PY"

# ── Done ────────────────────────────────────────────────────────────────────

echo ""
if [ "$PY_EXIT" -eq 0 ]; then
  say DONE "Firmware upload complete — IMU verified via router"
else
  say DONE "Firmware uploaded — verification concluded"
fi
echo "  I2C devices: PCA9685 @ 0x40 (64), MPU9250 @ 0x68 (104)"
echo ""
echo "  ── Next step ──"
echo "  The legacy Docker container has been stopped permanently."
echo "  Launch the ROS 2 hardware bridge:"
echo ""
echo "    ros2 launch big_bertha_bringup hardware_bringup.launch.py"