#!/usr/bin/env bash
#
# upload_firmware.sh — Upload the hardware_bridge_app sketch to the
# STM32U585 M33 co-processor on the UNO Q board.
#
# Run this after any `arduino-app-cli app start` of a different app
# (e.g. i2c_scanner, imu_scanner) to restore the correct firmware.
#
# Usage:
#   ssh arduino@<board-ip> "$(cat upload_firmware.sh)"
#
# Or copy it to the board first, then run:
#   ssh arduino@<board-ip> ./upload_firmware.sh

set -euo pipefail

APP_PATH="/home/arduino/ArduinoApps/hardware_bridge_app"

echo "=== hardware_bridge_app firmware upload ==="

# 1. Stop the production container (frees port 50007)
echo "[1/5] Stopping hardware-bridge systemd service..."
sudo systemctl stop hardware-bridge.service 2>/dev/null || true

# 2. Upload the sketch to M33 (this also resets the M33)
echo "[2/5] Compiling and uploading sketch to STM32U585..."
arduino-app-cli app start "$APP_PATH"
echo "      Sketch uploaded OK"

# 3. Stop the compose container (we don't need it running)
echo "[3/5] Stopping app-cli compose container..."
arduino-app-cli app stop "$APP_PATH"

# 4. Restart the production container (host networking)
echo "[4/5] Starting hardware-bridge systemd service..."
sudo systemctl start hardware-bridge.service

# 5. Wait for Bridge RPC to connect and verify
echo "[5/5] Waiting for Bridge RPC connection..."
sleep 15

echo ""
echo "=== Verification ==="
echo "Ping:"
echo '{"cmd":"ping"}' | timeout 3 nc 127.0.0.1 50007
echo ""
echo "Status:"
echo '{"cmd":"status"}' | timeout 3 nc 127.0.0.1 50007
echo ""
echo "I2C scan:"
echo '{"cmd":"scan_i2c"}' | timeout 8 nc 127.0.0.1 50007
echo ""
echo "IMU:"
echo '{"cmd":"imu"}' | timeout 3 nc 127.0.0.1 50007

echo ""
echo "=== Done ==="
echo "Expected I2C devices: PCA9685 @ 0x40 (64), MPU6050 @ 0x68 (104)"
echo "Expected IMU: non-zero accel/gyro values"
