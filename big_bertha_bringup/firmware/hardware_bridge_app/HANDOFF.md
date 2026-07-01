# Handoff — hardware_bridge_app Container Crashes & Fix

## What

The `hardware_bridge_app` Docker container kept dying with exit code 137 (SIGKILL) ~10–15 seconds after starting. Port 50007 was unreachable, all commands (ping, imu, servo, scan_i2c) returned `Connection refused`. The legacy `hardware-bridge.service` pointed at a non-existent C++ binary (`~/poc/build/hardware_bridge`) and was systemd-masked.

## Why

Two independent root causes, either one sufficient to kill the container:

### 1. `loop()` was a no-op

```python
# BEFORE (crashes):
def loop():
    pass
```

The `arduino.app_utils.App.run(user_loop=loop)` calls `loop()` in its main thread. When `loop` returns immediately, the App framework considers the app "done" and initiates shutdown. The Docker container exits ~10s later with code 137.

### 2. Stale `.cache/.venv`

The `arduino-app-cli` creates a virtual environment at `/app/.cache/.venv` on first run. On subsequent runs, `/run.sh` reactivates it. If this venv was created by an incompatible Python version or by a different container instance, it silently fails — the container starts (prints "App started") but the Bridge library malfunctions, triggering a container kill.

### 3. Legacy systemd service pointed at missing binary

`/etc/systemd/system/hardware-bridge.service` had `ExecStart=/home/arduino/poc/build/hardware_bridge`. The `~/poc/` directory never existed on this board. The service was masked (`systemctl mask`) to prevent it from failing repeatedly.

---

## How (fixes applied)

### A. Fixed `loop()` in `main.py`

Changed to an infinite sleep loop so the App framework stays alive:

```python
# AFTER (stable):
def loop():
    while True:
        time.sleep(1)
```

Also added a `ping` command handler and `lin_acc_{x,y,z}` fields to the IMU response (per the handoff spec).

**File:** `big_bertha_bringup/firmware/hardware_bridge_app/python/main.py:145-148`

### B. Cleared stale `.cache/.venv`

Deleted `~/ArduinoApps/hardware_bridge_app/.cache/.venv/` to force the container to create a fresh virtual environment on the next start. The systemd service now clears this cache automatically on every restart via:

```ini
ExecStartPre=-/bin/rm -rf /home/arduino/ArduinoApps/hardware_bridge_app/.cache/.venv
```

### C. Rewrote systemd service

Replaced the dead C++ binary reference with a Docker-based service:

| Before | After |
|---|---|
| `ExecStart=/home/arduino/poc/build/hardware_bridge` | `ExecStart=/usr/bin/docker run --rm --name hardware-bridge --network host ...` |
| `Restart=on-failure` | `Restart=always` |
| masked | enabled |

Key choices in the new service:
- **`--network host`** — the app-cli default is a Docker bridge network (`hardware_bridge_app_default`); host networking avoids port-mapping issues and makes the container directly reachable on `127.0.0.1:50007`
- **`ExecStartPre` clears `.venv`** — prevents stale-cache crashes on future updates
- **`Restart=always`** — if the container dies, systemd brings it back within 5 seconds

**File:** `/etc/systemd/system/hardware-bridge.service`

```
[Unit]
Description=Hardware Bridge — Docker relay between ROS 2 and arduino-router
After=network.target docker.service arduino-router.service
Requires=docker.service arduino-router.service

[Service]
Type=simple
User=arduino
ExecStartPre=-/usr/bin/docker rm -f hardware-bridge 2>/dev/null
ExecStartPre=-/bin/rm -rf /home/arduino/ArduinoApps/hardware_bridge_app/.cache/.venv
ExecStart=/usr/bin/docker run --rm --name hardware-bridge --network host \
  -v /home/arduino/ArduinoApps/hardware_bridge_app:/app \
  -v /var/run/arduino-router.sock:/var/run/arduino-router.sock \
  -v /dev:/dev \
  ghcr.io/arduino/app-bricks/python-apps-base:0.10.1
ExecStop=-/usr/bin/docker stop -t 5 hardware-bridge
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

---

## Current State

| Check | Status |
|---|---|
| `hardware-bridge.service` | `active (running)` since boot, `Restart=always` |
| Port 50007 | Listening on `0.0.0.0:50007` |
| `{"cmd":"ping"}` | `{"ok":true}` |
| `{"cmd":"imu"}` | Returns real MPU6050 data with `lin_acc_*` fields |
| `{"cmd":"status"}` | `pca9685_ok: true, mpu6050_ok: true` |
| `{"cmd":"scan_i2c"}` | `{"addrs": [64, 104]}` (PCA9685 @ 0x40, MPU6050 @ 0x68) |
| `{"cmd":"servo","pwms":[...]}` | `{"ok":true}` |
| Docker container name | `hardware-bridge` (not `hardware_bridge_app-main-1`) |
| Stale containers | All old `*_main-1` containers removed |

## Updating the Sketch

When you need to update the MCU sketch (`.ino`):

1. Flash via app-cli (this creates a compose-managed container — that's fine):
   ```bash
   arduino-app-cli app start ~/ArduinoApps/hardware_bridge_app
   ```

2. Stop the compose container (it uses bridge networking, don't use it):
   ```bash
   arduino-app-cli app stop ~/ArduinoApps/hardware_bridge_app
   ```

3. The systemd service will auto-restart the production container within 5 seconds.
