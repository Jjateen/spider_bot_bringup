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
# python/ holds only a no-op stub main.py — arduino-app-cli app start requires
# one to exist to build+flash the sketch. The real bridge is the C++ node.
cp "$WORKSPACE_APP/sketch/sketch.ino" "$BOARD_APP/sketch/sketch.ino"
cp "$WORKSPACE_APP/python/main.py" "$BOARD_APP/python/main.py" 2>/dev/null || true
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

# ── Compile & upload sketch ─────────────────────────────────────────────────

say FLASH "Compiling and uploading sketch to STM32U585 M33"
say FLASH "This takes ~15-20 seconds..."

# Stop the app first so `app start` is allowed to re-flash. This stops the MCU
# and the stub container; `app start` below re-runs the MCU (the sketch that
# pushes IMU), and we stop only the container afterwards — NOT via `app stop`
# again, which would halt the MCU and kill the IMU stream.
arduino-app-cli app stop "$BOARD_APP" 2>/dev/null || true

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

# ── Stop the stub container `app start` auto-created ────────────────────────
# The stub python/main.py does nothing — the real bridge is the C++ node.
# Stop the auto-started compose container so nothing holds the router route.
# IMPORTANT: use `docker stop` (container only), NOT `app stop` — app stop
# also stops the microcontroller, which kills the sketch that pushes IMU.

say CLEAN "Stopping the stub compose container from app start"
docker stop hardware_bridge_app-main-1 2>/dev/null || true
docker rm -f hardware_bridge_app-main-1 2>/dev/null || true

# ── Poll for Bridge RPC connectivity ────────────────────────────────────────

say POLL "Waiting for Bridge RPC (up to ${PING_TIMEOUT}s)..."
say POLL "The hardware_bridge_node reconnects to the router socket on its own"
say POLL "after the MCU resets — nothing needs to be stopped or restarted."

CONNECTED=false
for i in $(seq 1 $((PING_TIMEOUT / POLL_INTERVAL))); do
  if [ -S /run/arduino-router/rpc.sock ] || [ -S /var/run/arduino-router.sock ]; then
    CONNECTED=true
    ok "Router socket present after ~$((i * POLL_INTERVAL))s"
    break
  fi
  sleep "$POLL_INTERVAL"
done

if [ "$CONNECTED" = false ]; then
  fail "Router socket not found within ${PING_TIMEOUT}s"
  fail "Check: systemctl status arduino-router"
  exit 1
fi

# ── Wait for enough samples to get a non-stale IMU/status ───────────────────
sleep 3

# ── Verify subsystems via ROS 2 services ───────────────────────────────────

say VERIFY "Verifying subsystems"

# The hardware_bridge_node exposes diagnostics as ROS 2 services. The verify
# is a soft check — the node may not be running yet (it is launched via
# ros2 launch big_bertha_bringup big_bertha.launch.py, or the repo-side
# hardware-bridge.service unit).
if command -v ros2 >/dev/null 2>&1; then
  STATUS=$(ros2 service call /hardware_bridge/status std_srvs/srv/Trigger 2>/dev/null | tail -2)
  if echo "$STATUS" | grep -q "success=True" || echo "$STATUS" | grep -q '"success": true'; then
    ok "hardware_bridge status: $(echo "$STATUS" | tr '\n' ' ')"
  else
    fail "hardware_bridge status failed: $STATUS"
    fail "Start the node: ros2 launch big_bertha_bringup big_bertha.launch.py"
  fi
else
  say VERIFY "ros2 CLI not found on this host — skipping service checks"
fi

# ── Done ────────────────────────────────────────────────────────────────────

echo ""
say DONE "Firmware upload complete — sketch flashed, router healthy"
# Report what the MCU actually found, rather than asserting it. The previous
# version printed a hardcoded device list unconditionally, so anyone running
# this was told the IMU was present whether or not it existed. That cost real
# debugging time during the BNO055 investigation.
echo "  Verify devices with:  ros2 service call /hardware_bridge/status std_srvs/srv/Trigger"
echo "  (scan bit0 = PCA9685 missing, bit1 = IMU missing)"
echo "  Start the bridge: ros2 launch big_bertha_bringup big_bertha.launch.py"