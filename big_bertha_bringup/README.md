# big_bertha_bringup — Hardware Bridge for Physical Robot

ROS 2 package for bringing up the physical Big Bertha quadruped robot.

## Architecture

### Direct Native Bridge (C++ → Firmware)

```
ROS 2 Node (C++)
  ↓ MsgPack-RPC unix socket
arduino-router
  ↓ UART (internal)
STM32U585 M33 (sketch.ino)
  ↓ I2C (50 kHz)
PCA9685 (servos) + BNO055 (fused IMU)
```

**No Python relay, no TCP, no Docker containers.**

The hardware bridge node communicates directly with the STM32U585 co-processor via the arduino-router's MsgPack-RPC unix socket. This eliminates the Python TCP relay that was previously used, reducing latency and simplifying deployment.

### Components

| Component | Purpose | Interface |
|-----------|---------|-----------|
| `hardware_bridge_node` | ROS 2 bridge to firmware | Bridge RPC (MsgPack) |
| `sketch.ino` | Real-time servo/IMU controller | I2C @ 50 kHz |
| `servo_converter` | Joint angles → PWM | C++ library |
| IMU calibration | Gyro/accel bias removal | Startup (200 samples) |

### ROS 2 Interface

**Published Topics:**
- `/imu` (sensor_msgs/Imu) — 100 Hz, bias-corrected, axis-mapped, with fused orientation
- `/imu/mag` (sensor_msgs/MagneticField) — raw BNO055 magnetometer field

> `/joint_states` is **not** published by the bridge. On hardware it is owned by
> `legged_odometry` — its EWMA simulates the MG995 lag, which is the policy's
> joint feedback (feeding back raw commands would close a positive-feedback loop).

**Subscribed Topics:**
- `/position_controller/commands` (std_msgs/Float64MultiArray) — 12 joint targets from policy

**Services (Diagnostics):**
- `~/status` — Hardware status (I2C health, PWM counters, error tracking)
- `~/scan_i2c` — Trigger I2C bus scan, returns device addresses
- `~/servo_diag` — Run servo write/readback test (5s timeout)
- `~/imu_diag` — BNO055 identity, fusion mode, calibration status nibbles

### Key Parameters

See `config/hardware_bridge.yaml` for full list. Notable parameters:

- `router_socket` — arduino-router unix socket path (auto-probes if empty)
- `pwm_min/pwm_max` — Servo PWM range (12-bit, 50 Hz)
- `servo_*` arrays — Per-joint limits, offsets, channels, directions (12 elements)
- `command_rate_hz` / `max_joint_rate_rad_s` — Rate limiting for servo safety
- `gyro/accel_calibration_samples` — Startup calibration (200 = ~2.2s @ 91 Hz)
- `imu_axis_sign` — Chip frame → base_link transform

## Quick Start

### 1. Upload Firmware
```bash
ssh arduino@<board-ip>
cd ~/ros2_ws/src/spider_bot_bringup
./scripts/upload_firmware.sh
```

### 2. Launch Bridge
```bash
ros2 launch big_bertha_bringup hardware_bringup.launch.py
```

### 3. Run Diagnostics
```bash
python3 scripts/servo_diag.py
```

## Documentation

- **[DEPLOYMENT.md](DEPLOYMENT.md)** — Full deployment guide, troubleshooting, rollback procedures
- **[API.md](API.md)** — Complete ROS 2 interface reference (topics, services, parameters)
- **[VALIDATION_CHECKLIST.md](VALIDATION_CHECKLIST.md)** — Hardware testing checklist before production use
- **[firmware/hardware_bridge_app/README.md](firmware/hardware_bridge_app/README.md)** — Firmware architecture and Bridge RPC protocol

## Hardware

- **Compute:** Arduino UNO Q (4GB RAM, Cortex-A35 + STM32U585 M33)
- **Servos:** 12× MG995 (max angular velocity: 6.54 rad/s)
- **IMU:** BNO055 9-axis with on-chip sensor fusion (accel+gyro+mag, NDOF)
- **Servo Controller:** PCA9685 16-channel PWM driver
- **I2C Bus:** 50 kHz (empirically reliable on STM32U585, workaround for Zephyr #83550)

## Development

### Build
```bash
colcon build --packages-select big_bertha_bringup
```

### Test
```bash
colcon test --packages-select big_bertha_bringup
ros2 run big_bertha_bringup hardware_bridge_node --ros-args --log-level debug
```

### Lint
```bash
colcon test --packages-select big_bertha_bringup --event-handlers console_direct+
```

## Known Issues

1. **On-chip fusion needs initial calibration:** The BNO055's NDOF filter
   converges on its own but the magnetometer must be calibrated before yaw is
   absolute: rotate the chassis in a figure-of-eight while level until
   `~/imu_diag` shows `calib_mag=3`. Until then heading drifts like the old
   magnetometer-less MPU-6500.
2. **I2C clock 50 kHz:** Below standard (100 kHz) but required for STM32U585 I2C v2 peripheral reliability (Zephyr issue #83550).
3. **Startup calibration required:** Robot must remain stationary for 2-3 seconds during node startup for gyro/accel bias estimation.
4. **No position feedback:** MG995 servos have no encoders. `/joint_states` publishes commanded positions, not actual positions.
5. **Magnetic interference:** MG995 servo/motor fields near the IMU can corrupt the absolute heading. Mount the BNO055 clear of high-current wiring.

## Migration from TCP Relay

The Python TCP relay architecture (ports 50007/50008, Docker container) was removed in commit `88e1d25`. The new native bridge eliminates:
- Python relay process (main.py, ~364 lines → 31 line stub)
- TCP JSON protocol overhead
- Docker runtime dependency

**Previous architecture (removed):**
```
ROS2 → TCP JSON (50007/50008) → Python relay (Docker) → router socket → firmware
```

If you need to roll back to the TCP relay:
```bash
git checkout 5de3e5e  # commit before native bridge
colcon build --packages-select big_bertha_bringup
```

## Deployment

For first-time deployment or updates, see [DEPLOYMENT.md](DEPLOYMENT.md).

For systematic hardware validation before production, see [VALIDATION_CHECKLIST.md](VALIDATION_CHECKLIST.md).

## Bill of Materials (BOM)

Full hardware specification from [PLAN.md §11](../PLAN.md):

| Subsystem | Part | Quantity | Notes |
|-----------|------|----------|-------|
| Compute | Arduino UNO Q (4 GB) | 1 | ROS 2 Jazzy (arm64) |
| Structure | 3D-printed frame + legs | — | Spider quadruped design |
| Actuators | MG995 servos | 12 | 4 legs × 3 joints |
| Lidar | YDLidar X2 (2D) | 1 | `/scan` topic |
| IMU | BNO055 (9-axis, fused) | 1 | On-chip accel+gyro+mag fusion |
| Servo Driver | PCA9685 | 1 | 16-channel PWM @ 50 Hz |

## Troubleshooting

**Node won't connect:**
- Check `systemctl status arduino-router`
- Verify socket exists: `ls -l /run/arduino-router/rpc.sock`
- Check firmware running: `arduino-app-cli app status`

**No IMU data:**
- Run I2C scan: `ros2 service call /hardware_bridge/scan_i2c std_srvs/srv/Trigger`
- Check IMU diagnostic: `ros2 service call /hardware_bridge/imu_diag std_srvs/srv/Trigger`

**Servos not responding:**
- Check PCA9685: `ros2 service call /hardware_bridge/scan_i2c std_srvs/srv/Trigger`
- Run servo diagnostic: `ros2 service call /hardware_bridge/servo_diag std_srvs/srv/Trigger`

See [DEPLOYMENT.md](DEPLOYMENT.md) for comprehensive troubleshooting guide.

## License

Apache 2.0 — See [LICENSE](../LICENSE) for details.

## Maintainer

Jjateen Gundesha (@Jjateen)
