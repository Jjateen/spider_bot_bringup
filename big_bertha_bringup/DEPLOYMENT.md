# Hardware Bridge Deployment Guide

Complete guide for deploying the native C++ hardware bridge to the Arduino UNO Q board.

## Prerequisites

### Board-Side (Arduino UNO Q)
- Ubuntu 24.04 (or compatible Linux distribution)
- ROS 2 Jazzy installed and sourced
- `arduino-app-cli` daemon running
- `arduino-router` service running
- I2C devices connected:
  - PCA9685 @ 0x40 (servo controller)
  - MPU-6500 @ 0x68 (IMU; sold as an "MPU-9265" but WHO_AM_I reads 0x70, a
    six-axis die with no magnetometer — see Known Limitations)
- Workspace at `~/ros2_ws/src/spider_bot_bringup`

### Host-Side (Development Machine)
- Git access to repository
- SSH access to board
- ROS 2 Jazzy sourced (for testing)

### Hardware Setup
- 12× MG995 servos connected to PCA9685 channels (see config)
- Power supply adequate for simultaneous servo operation (recommend 6V 10A+)
- IMU mounted securely with known orientation
- Emergency stop accessible (power switch or E-stop button)

## Initial Setup (First-Time Deployment)

### Step 1: Clone Repository on Board

```bash
ssh arduino@<board-ip>
cd ~/ros2_ws/src
git clone git@github.com:Jjateen/spider_bot_bringup.git
cd spider_bot_bringup
git checkout pr/hw-bringup
```

**Verification:**
```bash
git log --oneline -1
# Should show: 88e1d25 feat(bringup): replace python tcp relay with native bridge rpc client
```

### Step 2: Build ROS Workspace

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select big_bertha_bringup
source install/setup.bash
```

**Expected output:**
```
Starting >>> big_bertha_bringup
Finished <<< big_bertha_bringup [0.xx s]
Summary: 1 package finished [x.xx s]
```

**Verification:**
```bash
ros2 pkg list | grep big_bertha_bringup
# Should show: big_bertha_bringup

ros2 run big_bertha_bringup hardware_bridge_node --help
# Should show usage (or start attempting connection)
```

### Step 3: Upload Firmware to STM32U585

```bash
cd ~/ros2_ws/src/spider_bot_bringup
./scripts/upload_firmware.sh
```

**Expected output:**
```
[CHECK] Prerequisites
  ✔ Workspace app exists
  ✔ app-cli daemon is active
[SYNC] Syncing workspace sketch to ArduinoApps
  ✔ Sketch synced (XXXXX bytes)
[FLASH] Compiling and uploading sketch to STM32U585 M33
  ✔ Sketch compiled and uploaded successfully
[DOCKER] Stopping Python relay container (not needed)
  ✔ Container stopped
[VERIFY] Checking Bridge RPC health...
  ✔ arduino-router is running
  ✔ Bridge RPC socket exists
  ✔ Ping test successful (round-trip < 10 ms)
```

**Troubleshooting upload:**
- If `app-cli daemon is not running`: `sudo systemctl start arduino-app-cli`
- If sketch won't compile: Check `journalctl -u arduino-app-cli -n 50` for errors
- If upload times out: Power cycle the board, ensure M33 co-processor is responsive

### Step 4: Test Bridge Manually

```bash
# Terminal 1: Run the bridge node
ros2 run big_bertha_bringup hardware_bridge_node --ros-args --log-level info
```

**Expected startup log:**
```
[INFO] [hardware_bridge]: Connected to arduino-router at /run/arduino-router/rpc.sock
[INFO] [hardware_bridge]: calibrating sensors (200 samples)...
[INFO] [hardware_bridge]: gyro bias: gx=0.XXXXXX gy=0.XXXXXX gz=0.XXXXXX rad/s
[INFO] [hardware_bridge]: accel bias: ax=0.XXXXXX ay=0.XXXXXX az=0.XXXXXX m/s²
[INFO] [hardware_bridge]: calibration complete (200 samples)
```

```bash
# Terminal 2: Verify topics
ros2 topic list
# Should include: /imu, /joint_states

ros2 topic hz /imu
# Should report: ~125 Hz

ros2 topic echo /imu --once
# Should show realistic IMU data
```

```bash
# Terminal 3: Test services
ros2 service call /hardware_bridge/status std_srvs/srv/Trigger
# Should return: success=true, message with hardware status

ros2 service call /hardware_bridge/scan_i2c std_srvs/srv/Trigger
# Should return: addrs=[64, 104] (0x40=PCA9685, 0x68=IMU)
```

**If all tests pass, proceed to Step 5. If not, see Troubleshooting section.**

### Step 5: Install Systemd Service (Optional but Recommended)

```bash
sudo cp ~/ros2_ws/src/spider_bot_bringup/scripts/hardware-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable hardware-bridge
sudo systemctl start hardware-bridge
```

**Verification:**
```bash
sudo systemctl status hardware-bridge
# Should show: active (running)

journalctl -u hardware-bridge -n 20
# Should show calibration complete, no errors
```

**Configure service (if needed):**
Edit `/etc/systemd/system/hardware-bridge.service` to adjust:
- `User=` (default: arduino)
- `WorkingDirectory=`
- Environment variables

Then: `sudo systemctl daemon-reload && sudo systemctl restart hardware-bridge`

## Updates (Pulling Changes)

### Standard Update (Code Changes Only)

When updating C++ node code without firmware changes:

```bash
ssh arduino@<board-ip>
cd ~/ros2_ws/src/spider_bot_bringup
git pull origin pr/hw-bringup
cd ~/ros2_ws
colcon build --packages-select big_bertha_bringup
source install/setup.bash

# If using systemd:
sudo systemctl restart hardware-bridge

# If running manually:
# Ctrl+C the running node, then re-run:
ros2 run big_bertha_bringup hardware_bridge_node
```

### Update with Firmware Change

When sketch.ino or firmware logic changes:

```bash
ssh arduino@<board-ip>
cd ~/ros2_ws/src/spider_bot_bringup
git pull origin pr/hw-bringup

# Re-flash the firmware
./scripts/upload_firmware.sh

# Rebuild ROS node
cd ~/ros2_ws
colcon build --packages-select big_bertha_bringup
source install/setup.bash

# Restart bridge
sudo systemctl restart hardware-bridge
```

### Update Configuration Only

When only `config/hardware_bridge.yaml` changes:

```bash
ssh arduino@<board-ip>
cd ~/ros2_ws/src/spider_bot_bringup
git pull origin pr/hw-bringup

# No rebuild needed, just restart
sudo systemctl restart hardware-bridge
```

## Validation Checklist

After any deployment or update, complete the [VALIDATION_CHECKLIST.md](VALIDATION_CHECKLIST.md):

**Quick validation (30 seconds):**
```bash
# 1. Check node is running
sudo systemctl status hardware-bridge

# 2. Verify IMU publishes
ros2 topic hz /imu
# Expect: ~125 Hz

# 3. Check hardware status
ros2 service call /hardware_bridge/status std_srvs/srv/Trigger
# Expect: scan=0 (all devices present)

# 4. Quick I2C scan
ros2 service call /hardware_bridge/scan_i2c std_srvs/srv/Trigger
# Expect: addrs=[64, 104]
```

**Full validation:** See [VALIDATION_CHECKLIST.md](VALIDATION_CHECKLIST.md) for comprehensive testing procedure.

## Troubleshooting

### Node Won't Start

**Symptom:** `Failed to connect to arduino-router (candidates probed)`

**Diagnosis:**
```bash
# Check arduino-router is running
systemctl status arduino-router
# Should show: active (running)

# Check socket exists
ls -l /run/arduino-router/rpc.sock
# Should show: srwxrwxrwx ... /run/arduino-router/rpc.sock

# Check arduino-app-cli
systemctl status arduino-app-cli
arduino-app-cli app status
# Should show hardware_bridge_app running
```

**Solutions:**
1. Restart arduino-router: `sudo systemctl restart arduino-router`
2. Restart app: `arduino-app-cli app restart ~/ArduinoApps/hardware_bridge_app`
3. Check socket permissions: `sudo chmod 666 /run/arduino-router/rpc.sock` (if needed)
4. Verify sketch uploaded: `./scripts/upload_firmware.sh`

### No IMU Data

**Symptom:** `/imu` topic not publishing or publishing zeros

**Diagnosis:**
```bash
# Check I2C devices present
ros2 service call /hardware_bridge/scan_i2c std_srvs/srv/Trigger
# Should return addrs including 104 (0x68)

# Check IMU diagnostic
ros2 service call /hardware_bridge/imu_diag std_srvs/srv/Trigger
# Should show WHO_AM_I and device detection
```

**Solutions:**
1. **I2C wiring:** Verify SDA/SCL connections, pull-up resistors
2. **I2C address conflict:** Run scan, ensure 0x68 is the only device at that address
3. **Power issue:** Check IMU VCC/GND, measure with multimeter (should be 3.3V or 5V)
4. **Firmware issue:** Re-upload firmware, check `journalctl -u arduino-app-cli -f`
5. **Calibration timeout:** Check node logs, ensure robot is stationary during startup

### Servo Commands Not Working

**Symptom:** Publishing to `/position_controller/commands` but servos don't move

**Diagnosis:**
```bash
# Check PCA9685 present
ros2 service call /hardware_bridge/scan_i2c std_srvs/srv/Trigger
# Should include addrs with 64 (0x40)

# Run servo diagnostic
ros2 service call /hardware_bridge/servo_diag std_srvs/srv/Trigger
# Should report PWM writes successful

# Check hardware status
ros2 service call /hardware_bridge/status std_srvs/srv/Trigger
# Check pwm_write_fails counter
```

**Solutions:**
1. **PCA9685 not detected:** Check I2C wiring, run scan
2. **PWM writes failing:** Check `pwm_write_fails` counter in status, indicates I2C NAKs
3. **Servo power:** Ensure external 6V power supply connected to PCA9685 V+ terminal
4. **Servo channels:** Verify `servo_channel` config matches physical wiring
5. **Rate limiting too aggressive:** Check if commands are within rate limit (see API.md)

### Socket Path Not Found

**Symptom:** Node tries all socket candidates, none exist

**Diagnosis:**
```bash
# Find actual socket path
find /run /var/run /tmp -name "rpc.sock" -o -name "arduino-router.sock" 2>/dev/null
```

**Solution:**
Edit `config/hardware_bridge.yaml`, set `router_socket: "/actual/path/rpc.sock"`, then restart.

### I2C Timeouts/Consecutive Failures

**Symptom:** Firmware logs show `g_i2c_consecutive_fails` increasing, IMU stops publishing

**Diagnosis:**
Check firmware diagnostic counters via `~/status` service.

**Solutions:**
1. **Bad connections:** Reseat I2C wires, check for loose connections
2. **EMI/noise:** Shorten I2C wires, add ferrite beads, separate from power lines
3. **Clock issue:** Sketch uses 50 kHz (hardcoded), already below spec for reliability
4. **Power brownout:** Check power supply is stable under servo load
5. **STM32 I2C bug:** Known Zephyr #83550, 50 kHz is workaround but not perfect
6. **Nuclear option:** Power cycle board: `sudo reboot`

### Calibration Fails

**Symptom:** Node logs "IMU appears to be missing — all samples were zero"

**Diagnosis:**
IMU is not responding or all data is zero.

**Solutions:**
1. Check I2C scan finds IMU
2. Verify IMU power (VCC should be 3.3V or 5V)
3. Check WHO_AM_I register via `~/imu_diag` service
4. If IMU is detected but returns all zeros, suspect firmware I2C read issue

### High Gyro Drift

**Symptom:** `gyro accumulated drift` logs show large values (> 0.1 rad after 10s)

**Cause:** No magnetometer on this hardware (MPU-6500 instead of MPU-9250), so yaw has no absolute reference.

**Mitigation:**
1. **Increase calibration samples:** Edit config, set `gyro_calibration_samples: 500`
2. **Ensure stationary startup:** Robot must be perfectly still during calibration
3. **Temperature stabilization:** Let IMU warm up for 30 seconds before starting node
4. **Accept limitation:** Without magnetometer, some drift is inevitable

**Long-term solution:** Replace IMU with genuine MPU-9250 or add external magnetometer.

## Rollback Procedure

### Rollback to Previous Commit

If the current version has issues:

```bash
ssh arduino@<board-ip>
cd ~/ros2_ws/src/spider_bot_bringup

# View recent commits
git log --oneline -10

# Rollback to specific commit
git checkout <commit-hash>

# If sketch changed, re-upload firmware
./scripts/upload_firmware.sh

# Rebuild
cd ~/ros2_ws
colcon build --packages-select big_bertha_bringup
source install/setup.bash

# Restart
sudo systemctl restart hardware-bridge
```

### Rollback to TCP Relay (Pre-Native Bridge)

If native bridge is fundamentally broken and you need the Python relay:

```bash
git checkout 5de3e5e  # Last commit before native bridge (88e1d25)
cd ~/ros2_ws
colcon build --packages-select big_bertha_bringup
source install/setup.bash

# Python relay needs Docker container
cd ~/ros2_ws/src/spider_bot_bringup/big_bertha_bringup/firmware/hardware_bridge_app
# Follow old deployment procedure (Docker setup, TCP ports)
```

**Note:** TCP relay architecture is deprecated. Only use for emergency recovery.

### Emergency Stop

To immediately stop all servo commands:

```bash
# Stop the bridge node
sudo systemctl stop hardware-bridge

# Stop the firmware (halts IMU and servos)
arduino-app-cli app stop ~/ArduinoApps/hardware_bridge_app

# Power off servos (if separate power supply)
# Flip external power switch or disconnect PCA9685 V+ terminal
```

## Logs & Diagnostics

### ROS Node Logs

```bash
# If using systemd
journalctl -u hardware-bridge -f

# If running manually
ros2 run big_bertha_bringup hardware_bridge_node --ros-args --log-level debug
```

**Key log messages:**
- `Connected to arduino-router at ...` — Socket connection successful
- `calibrating sensors (N samples)...` — Calibration started
- `gyro bias: ...` — Calibration results
- `gyro accumulated drift: ...` — Every 10 seconds, shows drift accumulation

### Firmware Logs (MCU)

```bash
journalctl -u arduino-app-cli -f
```

**Key indicators:**
- `Bridge.begin(460800)` — Bridge RPC initialized
- `provide("set_servo_pwms", ...)` — Handlers registered
- I2C transaction errors logged by sketch

### Systemd Service Logs

```bash
# Last 100 lines
journalctl -u hardware-bridge -n 100

# Follow live
journalctl -u hardware-bridge -f

# Since last boot
journalctl -u hardware-bridge -b
```

### Network Diagnostics (If Remote)

```bash
# Test connectivity
ping <board-ip>

# Verbose SSH
ssh -v arduino@<board-ip>

# Check SSH daemon
systemctl status ssh
```

## Performance Tuning

### IMU Publish Rate

**Default:** 125 Hz (8 ms interval in sketch)

**To adjust:**
Edit `firmware/hardware_bridge_app/sketch/sketch.ino`, line 85:
```cpp
static const unsigned long IMU_INTERVAL = 8;  // Change to desired milliseconds
```

Then re-upload: `./scripts/upload_firmware.sh`

**Recommended:** 125 Hz matches policy controller update rate.

### Servo Rate Limiting

**Config:** `config/hardware_bridge.yaml`

```yaml
command_rate_hz: 50.0               # PCA9685 refresh rate, not the policy rate
max_joint_rate_rad_s: 3.0           # slew ceiling (MG995 free-running max is 6.54)
# Computed: rate_limit_rad = 3.0 / 50.0 = 0.06 rad/step
```

**To adjust:**
- These two are a matched pair. `rate_limit_rad` is their ratio, so raising the
  send rate without raising the ceiling (or the reverse) silently clamps every
  joint: 50 Hz sends against a 200 Hz limiter pins all twelve at 1.635 rad/s,
  which is what stopped the legs swinging.
- Do not drop `max_joint_rate_rad_s` below ~2.0 to slow the robot down. That
  clips the ankles, which are what lift the feet. Lower the commanded `vx`
  instead, which lowers the clock boost and the required rate with it.
- See the comments in `config/hardware_bridge.yaml` for the derivation.

### Calibration Duration

**Config:** `config/hardware_bridge.yaml`

```yaml
gyro_calibration_samples: 200       # ~2.2 seconds @ 91 Hz
accel_calibration_samples: 200
```

**To adjust:**
- Increase if bias estimate is noisy or drift is high
- Decrease for faster startup (minimum ~50 samples)
- Measured IMU rate is ~91 Hz, so 200 samples ≈ 2.2 seconds

## Security Considerations

1. **Unix socket permissions:** Bridge RPC socket should be accessible only by authorized users
2. **No authentication:** Bridge RPC has no auth mechanism, assumes trusted local environment
3. **Servo safety:** Rate limiting prevents abrupt motion, but no collision detection or torque limits
4. **Emergency stop required:** Physical E-stop or power switch mandatory for safety
5. **Network exposure:** If board is network-accessible, ensure firewall rules prevent unauthorized access

## Known Limitations

1. **No encoder feedback:** MG995 servos have no position sensors, so commanded ≠ actual position
2. **No magnetometer:** Yaw drift is unbounded without absolute heading reference
3. **I2C 50 kHz:** Below spec, but required for STM32U585 reliability
4. **Startup calibration required:** Robot must be stationary for 2-3 seconds at node start
5. **No runtime reconfiguration:** Parameter changes require node restart
6. **The lidar and the servos cannot share the battery.** Powering the servos
   collapses the USB rail and the lidar drops off the bus. Verified 2026-08-12:
   the moment the servos were energised the external USB hub began
   re-enumerating (`usb 1-1: USB disconnect` cycling through device numbers
   104 to 110 in about seven seconds, with `error -32` on suspend and
   `error -71` on descriptor read, both signal-integrity failures), `lsusb`
   dropped to the two root hubs, and `/dev/ttyUSB0` disappeared. The battery
   was flat shortly after.

   This presents as a software fault and is not one. The driver logs
   `Device is not open` and `Failed to get scan`, the lifecycle manager sits
   in `Waiting for service ydlidar_ros2_driver_node/get_state`, and the
   pipeline looks like it is failing to start. Check `ls /dev/ttyUSB*` before
   debugging anything above it: if the device node is gone, the problem is
   power.

   Until the lidar has its own supply or a powered hub, run one or the other:
   `with_lidar:=false` for servo work, or leave the policy disarmed for
   mapping.

## Quick start: the composed stack

One command, from the repo root on the board:

```bash
./scripts/run_stack.sh
```

That brings up robot_state_publisher, the lidar, the control container
(hardware_bridge + madgwick + leg_odometry + policy_controller +
scan_ground_filter, all intra-process) and the navigation container (slam or
AMCL, plus the five Nav2 servers). It also exports the Cyclone profile, which
is what makes topics visible from a dev machine.

The policy starts armed. For bench work where nothing should move:

```bash
./scripts/run_stack.sh start_enabled:=false with_nav:=false
./scripts/run_stack.sh with_lidar:=false          # servos only, no lidar
```

### What to check, and what it should read

Measured on hardware 2026-08-12 with the stack running on the adapter:

| check | expected |
| --- | --- |
| `ros2 topic hz /imu` | ~113 Hz |
| `ros2 topic hz /filtered/imu` | ~110 Hz |
| `ros2 topic hz /odom` | ~110 Hz |
| `ros2 topic hz /scan` | ~11 Hz, `frame_id: lidar_link` |
| `ros2 topic hz /scan_filtered` | ~11 Hz |
| `ros2 topic hz /map` | 1 Hz |
| `ros2 run tf2_ros tf2_echo map base_link` | near origin, z exactly 0 |
| `ros2 param get /slam_toolbox map_start_pose` | `[0.0, 0.0, 0.0]` |
| `ros2 service call /hardware_bridge/status std_srvs/srv/Trigger` | `pwm_write_fails=0` |
| `ros2 lifecycle get /slam_toolbox` | `active [3]` |

Arm or disarm the gait without restarting anything:

```bash
ros2 service call /set_policy_enabled spider_msgs/srv/SetPolicyEnabled "{enabled: true}"
```

With the gait armed and a `/cmd_vel`, `/joint_states` jumps from 5 Hz (the
leg_odometry startup timer) to ~190 Hz, and `pwm_write_attempts` climbs at
~44 Hz against the configured 50.

### Known-good failure signatures

`/joint_states` stuck at exactly 5 Hz means the policy is publishing nothing:
either it is disarmed, or it never received an IMU. Check `/filtered/imu`
before anything else.

`/odom` silent while `/imu` streams means leg_odometry is not receiving the
topic named by `imu_topic`. That is a remap or a topic-name problem, not an
odometry one.

The lidar failing with `Device is not open` is almost always power. See Known
Limitations 6.

## Full-Stack Operation Flow (Autonomy Stack on Real Hardware)

The full stack runs **entirely on the board** (operate via SSH). Data flow:

```
arduino-router (systemd)            -> MCU <-> bridge RPC (/var/run/arduino-router.sock)
hardware_bridge_node                -> /imu (raw MPU-6500, 125 Hz)
imu_filter_madgwick                 -> /filtered/imu (orientation, /imu_topic default)
leg_odometry (publish_tf:=true)     -> /joint_states + odom->base_link (/odom)
ydlidar_ros2_driver  [SEPARATE]     -> /scan (YDLidar X2)
scan_ground_filter                  -> /scan_filtered (IMU-gated floor culling)
slam_toolbox  |  AMCL (map->odom)   -> /map, map->odom
Nav2 (nav2.launch.py)               -> /cmd_vel
policy_controller                   -> /position_controller/commands (12 joints)
hardware_bridge_node                -> PCA9685 servos
```

### Bringup order (each step has a readiness gate before the next)

1. **Router up** — `systemctl status arduino-router` → active, socket at
   `/var/run/arduino-router.sock`.
2. **Firmware streaming** — `arduino-app-cli app list` shows
   `hardware_bridge_app` running; `./scripts/upload_firmware.sh` if not.
   Gate: `/imu` streams ~125 Hz.
3. **Bridge + Madgwick** —
   `ros2 launch big_bertha_bringup hardware_bringup.launch.py` (bridge +
   madgwick). Source the board's DDS profile first:
   `export FASTRTPS_DEFAULT_PROFILES_FILE=~/ros2_ws/fastdds_shm_udp.xml`.
   The board also has a `~/ros2_ws/launch_hardware_bringup.sh` wrapper that
   does both, but it lives only on the board and is not in this repo.
   Gate: `/filtered/imu` has a non-degenerate orientation quaternion.
4. **Lidar (SEPARATE step — not in any launch file)** —
   `ros2 launch ydlidar_ros2_driver ydlidar_launch.py` using that driver's own
   params file, with `frame_id: lidar_link` to match the URDF. The params live
   with the driver, which is not vendored here.
   Gate: `ros2 topic hz /scan` ≈ 10 Hz, `frame_id: lidar_link`.
   > **Hardware note:** a YDLidar X2 must be physically connected; verify
   > `ls /dev/ttyUSB*` (currently **no lidar device is present on the board**).
5. **Full stack** — `ros2 launch big_bertha_bringup big_bertha.launch.py slam:=true`
   (SLAM default on hardware; `slam:=false` = known-map/AMCL). This launches
   rsp, bridge, policy, leg_odometry, mapping|localization, scan_ground_filter
   and Nav2. It does **not** start the lidar.
   Gates: `ros2 topic hz /scan_filtered` ≈ 10 Hz; `tf` has `map -> odom ->
   base_link`; Nav2 lifecycle nodes all `active`.

### Readiness gate cheat-sheet

```bash
ros2 topic hz /imu             # ~125 Hz (raw MPU)
ros2 topic hz /filtered/imu    # ~125 Hz (Madgwick orientation)
ros2 topic hz /scan            # ~10 Hz (lidar)
ros2 topic hz /scan_filtered   # ~10 Hz (ground filter)
ros2 run tf2_ros tf2_echo map base_link   # map->odom->base_link chain
ros2 lifecycle get /slam_toolbox get_state  # active
ros2 node list                 # all 5 Nav2 servers present
```

### Cross-machine ROS discovery (dev machine <-> board)

- Multicast discovery does **not** cross this WiFi (AP client isolation), so
  cross-machine `ros2` topics (e.g. dev-side RViz) are **not available**.
- Operate the stack via SSH on the board (this is the supported flow).
- Board DDS config: `~/ros2_ws/fastdds_shm_udp.xml` (Fast DDS, UDP whitelisted
  to `wlan0` + `tailscale0`); local same-host discovery works.

## Next Steps After Deployment

1. **Run full validation:** Complete [VALIDATION_CHECKLIST.md](VALIDATION_CHECKLIST.md)
2. **Launch full stack:** `ros2 launch big_bertha_bringup big_bertha.launch.py`
3. **Verify policy integration:** Ensure policy controller receives IMU + joint_states
4. **Calibrate servo offsets:** Adjust `servo_offset` if legs are misaligned at rest
5. **Tune IMU axis signs:** Adjust `imu_axis_sign` if orientation is inverted
6. **Characterize drift:** Log gyro drift over 5 minutes, decide if acceptable
7. **Test locomotion:** Run policy controller, verify smooth walking gait

## Support

For issues not covered here:
- Check logs: `journalctl -u hardware-bridge -n 100`
- Review API docs: [API.md](API.md)
- Test systematically: [VALIDATION_CHECKLIST.md](VALIDATION_CHECKLIST.md)
- Check firmware logs: `journalctl -u arduino-app-cli -n 100`
- File GitHub issue with logs attached

**Emergency contact:** Maintain physical access to board for power cycling and hardware inspection.
