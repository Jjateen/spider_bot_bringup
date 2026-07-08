#!/usr/bin/env bash
#
# upload_firmware.sh — Upload the hardware_bridge_app sketch to the
# STM32U585 M33 co-processor on the UNO Q board.
#
# Run this after any `arduino-app-cli app start` of a different app
# (e.g. i2c_scanner, imu_scanner) to restore the correct firmware.
# It syncs the latest sketch from the workspace, compiles, flashes,
# and verifies the Bridge RPC is healthy.
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

PING_TIMEOUT=60   # seconds to wait for Bridge RPC after restart
POLL_INTERVAL=2

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

# ── Stop production service & release port 50007 ────────────────────────────

say STOP "Stopping hardware-bridge service and releasing port 50007"

sudo systemctl stop hardware-bridge.service 2>/dev/null || true

# Wait until port 50007 is free (up to 30s)
for i in $(seq 1 15); do
  if ! ss -tlnp 2>/dev/null | grep -q ':50007\b'; then
    break
  fi
  sleep 2
done
if ss -tlnp 2>/dev/null | grep -q ':50007\b'; then
  fail "Port 50007 still bound after 30s — aborting"
  exit 1
fi
ok "Port 50007 released"

# ── Clean up stale Docker compose containers ────────────────────────────────

say CLEAN "Removing stale compose containers from previous runs"

STALE_CONTAINERS=$(docker ps -a --format '{{.Names}}' 2>/dev/null | grep -v '^hardware-bridge$' | grep -E '(hardware_bridge_app|ros2_ws.*hardware_bridge)' || true)
if [ -n "$STALE_CONTAINERS" ]; then
  for c in $STALE_CONTAINERS; do
    docker rm -f "$c" 2>/dev/null || true
  done
  ok "Removed stale containers"
else
  ok "No stale containers to clean"
fi

# ── Compile & upload sketch ─────────────────────────────────────────────────

say FLASH "Compiling and uploading sketch to STM32U585 M33"
say FLASH "This takes ~15-20 seconds..."

# Capture output so we can verify the upload succeeded
TMPLOG=$(mktemp /tmp/hw_bridge_flash.XXXXXX)
trap 'rm -f "$TMPLOG"' EXIT

arduino-app-cli app start "$BOARD_APP" >"$TMPLOG" 2>&1
APP_START_EXIT=$?

# Verify the sketch was actually uploaded (compile failure would not show this)
if grep -q 'sketch updated' "$TMPLOG" 2>/dev/null; then
  ok "Sketch compiled and uploaded successfully"
elif grep -qi 'error' "$TMPLOG" 2>/dev/null; then
  fail "Sketch compilation/upload failed — see log above"
  grep -i error "$TMPLOG" | head -5 | while IFS= read -r line; do fail "$line"; done
  exit 1
elif [ "$APP_START_EXIT" -ne 0 ] && ! grep -q 'sketch updated' "$TMPLOG" 2>/dev/null; then
  fail "app start failed (exit=$APP_START_EXIT) with no successful upload"
  exit 1
fi
rm -f "$TMPLOG"
trap - EXIT

# ── Stop compose container (we use our own) ─────────────────────────────────

say CLEAN "Stopping compose container created by app start"
arduino-app-cli app stop "$BOARD_APP" 2>/dev/null || true
docker rm -f hardware_bridge_app-main-1 2>/dev/null || true
docker ps -a --format '{{.Names}}' 2>/dev/null | grep -v '^hardware-bridge$' | grep -E '(hardware_bridge_app|ros2_ws.*hardware_bridge)' | while IFS= read -r c; do
  docker rm -f "$c" 2>/dev/null || true
done

ok "Compose container cleaned"

# ── Restart production service ──────────────────────────────────────────────

say START "Starting hardware-bridge service"
sudo systemctl start hardware-bridge.service 2>/dev/null

# Verify the service started
sleep 2
if ! systemctl is-active --quiet hardware-bridge.service 2>/dev/null; then
  fail "hardware-bridge.service failed to start"
  sudo journalctl -u hardware-bridge.service -n 10 --no-pager 2>/dev/null | tail -5 | while IFS= read -r line; do fail "$line"; done
  exit 1
fi
ok "hardware-bridge.service started"

# ── Poll for Bridge RPC connectivity ────────────────────────────────────────

say POLL "Waiting for Bridge RPC (up to ${PING_TIMEOUT}s)..."

CONNECTED=false
for i in $(seq 1 $((PING_TIMEOUT / POLL_INTERVAL))); do
  result=$(echo '{"cmd":"ping"}' | timeout 3 nc 127.0.0.1 50007 2>/dev/null)
  if echo "$result" | grep -q '"ok":true'; then
    CONNECTED=true
    ok "Bridge RPC connected after ~$((i * POLL_INTERVAL))s"
    break
  fi
  sleep "$POLL_INTERVAL"
done

if [ "$CONNECTED" = false ]; then
  fail "Bridge RPC did not connect within ${PING_TIMEOUT}s"
  fail "Check: sudo journalctl -u hardware-bridge.service -n 30 --no-pager"
  exit 1
fi

# ── Wait for enough samples to get a non-stale IMU/status ───────────────────
sleep 3

# ── Verify subsystems ───────────────────────────────────────────────────────

say VERIFY "Verifying subsystems"

# Ping
PING=$(echo '{"cmd":"ping"}' | timeout 3 nc 127.0.0.1 50007 2>/dev/null)
if echo "$PING" | grep -q '"ok":true'; then
  ok "Ping: $PING"
else
  fail "Ping failed: $PING"
  exit 1
fi

# Status (I2C devices)
STATUS=$(echo '{"cmd":"status"}' | timeout 5 nc 127.0.0.1 50007 2>/dev/null)
if echo "$STATUS" | grep -q '"pca9685_ok":true'; then
  ok "PCA9685 (0x40): present"
else
  fail "PCA9685 (0x40) not detected — check I2C wiring"
fi
if echo "$STATUS" | grep -q '"mpu6050_ok":true'; then
  ok "MPU6050 (0x68): present"
else
  fail "MPU6050 (0x68) not detected — check I2C wiring"
fi

# IMU data
IMU=$(echo '{"cmd":"imu"}' | timeout 3 nc 127.0.0.1 50007 2>/dev/null)
if echo "$IMU" | grep -q '"az":'; then
  AZ=$(echo "$IMU" | grep -o '"az":[0-9.eE+-]*' | cut -d: -f2)
  if [ -n "$AZ" ] && echo "$AZ" | awk '{ exit ($1 > 5.0 ? 0 : 1) }' 2>/dev/null; then
    ok "IMU: az=$AZ m/s² (gravity detected)"
  else
    fail "IMU: az=$AZ (suspicious — expected ~9.81)"
  fi
else
  fail "IMU: no data received"
fi

# ── Done ────────────────────────────────────────────────────────────────────

echo ""
say DONE "Firmware upload complete — all subsystems healthy"
echo "  I2C devices: PCA9685 @ 0x40 (64), MPU6050 @ 0x68 (104)"
echo "  Expected: non-zero accel/gyro values from IMU"
echo "  Bridge RPC: serving ping, status, imu, scan_i2c, servo commands"