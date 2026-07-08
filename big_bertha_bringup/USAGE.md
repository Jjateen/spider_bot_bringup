# hw_bridge — Direct Router Bridge RPC Node

C++ rclcpp node that connects straight to the arduino-router Unix socket,
eliminating the Python relay, TCP middle layer, and Docker container.

## Architecture

```
Policy Controller        ──→  /position_controller/commands  (12 rad @ 50 Hz)
                                  │
                                  ▼
┌──────────────────────────────────────────────┐
│  hw_bridge (rclcpp)                          │
│                                              │
│  notify("set_servo_pwms", 12 rad)  ──────┐   │
│                                            │   │
│  provide("imu")   ←──────────────────┐    │   │
│                                        │    │   │
│  pub /imu (sensor_msgs/Imu)           │    │   │
└──────────────────────────────────────────────┘
         │                          ▲
         │  MsgPack-RPC             │  MsgPack-RPC
         │  (notify)                │  (notify)
         ▼                          │
┌──────────────────────────────────────────────┐
│  arduino-router (Unix socket)                │
│  /run/arduino-router/rpc.sock                │
└──────────────────────────────────────────────┘
         │                          ▲
         │  UART (460800 baud)      │  UART
         │  set_servo_pwms(rad)     │  notify("imu", ...)
         ▼                          │
┌──────────────────────────────────────────────┐
│  STM32U585 (sketch.ino)                      │
│                                              │
│  Per-joint calibration table                 │
│  Watchdog (150 ms → safe crouch)             │
│  IMU @ 200 Hz with sample + micros()         │
│  I2C: PCA9685 (servos) + MPU9250 (IMU)       │
└──────────────────────────────────────────────┘
```

**Two hops, one language, zero Python, zero Docker.**

## Prerequisites

- **Ubuntu 24.04 / ROS 2 Jazzy** — or any ROS 2 distro with C++17
- **Arduino UNO Q** — with `arduino-router` running (stock systemd service)
- **STM32U585 sketch** — `hardware_bridge_app` flashed via `arduino-app-cli`
- **Router UART at 460800 baud** — the sketch calls `Bridge.begin(460800)`. If the router
  defaults to 115200, reconfigure it (see `Configure router baud` below)

## Quickstart

### 1. Build

```bash
colcon build --packages-select big_bertha_bringup
source install/setup.bash
```

### 2. Launch the bridge (standalone)

```bash
ros2 launch big_bertha_bringup hardware_bringup.launch.py
```

You should see:
```
[hardware_bridge]: Connected to router at /run/arduino-router/rpc.sock
```

### 3. Verify IMU stream

```bash
ros2 topic echo /imu
```

Expect `ax`, `ay`, `az`, `gx`, `gy`, `gz` fields updating at ~200 Hz.

### 4. Full stack

```bash
ros2 launch big_bertha_bringup big_bertha.launch.py
```

This brings up the hardware bridge, gait controller, leg odometry, state
estimation (EKF), mapping or localization, and Nav2 planning.

## Per-Joint Calibration

Calibration lives in the firmware sketch at
`big_bertha_bringup/firmware/hardware_bridge_app/sketch/sketch.ino`
in the `CAL[12]` array:

```cpp
static const JointCal CAL[12] = {
  // ch   lower upper offset dir  center_rad
  {14,  45.0, 180.0,  0.0,  1,   0.0},      //  0: FL hip
  {10,  30.0, 150.0,  0.0,  1,   0.0},      //  1: FL upper
  ...
};
```

| Field | Meaning |
|-------|---------|
| `channel` | PCA9685 output (0-15) |
| `lower_deg` | Mechanical minimum angle (degrees) |
| `upper_deg` | Mechanical maximum angle (degrees) |
| `offset_deg` | Mounting offset from ideal center (degrees) |
| `direction` | +1 = keep sign, -1 = flip sign |
| `policy_center_rad` | Policy output at physical center |

### Updating calibration

1. Edit the `CAL[]` array in `sketch.ino`
2. Re-flash via `arduino-app-cli`:
   ```bash
   arduino-app-cli app start ~/ArduinoApps/hardware_bridge_app
   arduino-app-cli app stop ~/ArduinoApps/hardware_bridge_app
   ```
3. The systemd `hw_bridge` service (if configured) reconnects automatically

**Why firmware?** Calibration is mechanical — it changes only when servos are
replaced or the chassis is modified. Compile-time constants avoid an init
protocol between host and MCU, keep the boot path simple, and guarantee
the calibration is always in sync with the firmware version.

## Watchdog

If the host stops sending `set_servo_pwms` for ≥150 ms (e.g., the
`hw_bridge` node crashes or the Ethernet link drops), the firmware:

1. Detects the gap in `g_last_command_ms`
2. Ramps all servos to a **safe crouch** position over ~50 ms
3. Holds that position until commands resume

The crouch PWM values are defined in `CROUCH_PWM[12]` in the sketch.
They are tuned to a low, stable stance that will not tip the robot.

When commands resume, the watchdog resets instantly (next `set_servo_pwms`
clears `g_watchdog_active`).

## IMU Payload

The firmware pushes IMU data at 200 Hz via:

```cpp
Bridge.notify("imu", ax, ay, az, gx, gy, gz,
              (float)g_imu_sample++, (float)micros());
```

The C++ node receives this as a `provide("imu")` notification and publishes:

- `header.stamp` = host receive time (`rclcpp::Clock::now()`)
- `header.frame_id` = `"imu_link"`
- Linear acceleration and angular velocity with 180° mount rotation applied
  (negate X and Y to match the URDF `imu_joint rpy="0 0 pi"`)
- MCU sample counter and microsecond timestamp logged at DEBUG level

### MCU timestamp vs host time

There is no PTP sync between the Qualcomm and STM32. The ROS header stamp
uses the host receive time, which is sufficient for the EKF at the low
latencies of the local UART link (<1 ms typical). The MCU `micros()` value
is available for offline latency analysis if needed.

## Running the PoC Benchmark

The `bridge_poc` tool validates the router can sustain 50 Hz servo commands
down + 100+ Hz IMU up at 460800 baud.

```bash
colcon build --packages-select big_bertha_bringup --cmake-target bridge_poc

# Run (default: 30 seconds)
./build/big_bertha_bringup/bridge_poc

# Custom duration / socket path
./build/big_bertha_bringup/bridge_poc --duration 60 --socket /tmp/router.sock
```

### Pass criteria (#54)

| Metric | Target | Measured |
|--------|--------|----------|
| Servo TX | ≥45 Hz | |
| IMU RX | ≥100 Hz | |
| RTT P99 | <5 ms | |

If the PoC fails, the fallback is bare Zephyr firmware with a custom packed
binary protocol at 1-2 Mbaud (abandons the Arduino app lab).

## Configuration

`config/hardware_bridge.yaml`:

```yaml
/**:
  ros__parameters:
    router_socket: "/run/arduino-router/rpc.sock"
    orientation_covariance: [-1.0, 0.0, ...]
    linear_acceleration_covariance: [0.001, 0.0, ...]
    angular_velocity_covariance: [0.00001, 0.0, ...]
```

Only IMU covariance and the socket path remain. All calibration moved to
firmware.

## Migrating from the Python Relay

**You are here because the old architecture was:**
```
hardware_bridge_node.cpp → TCP :50007 → Python relay (Docker) → Bridge RPC → STM32
```

### What changed

| Component | Old | New |
|-----------|-----|-----|
| ROS node | `hardware_bridge_node` | `hw_bridge` |
| Transport | TCP JSON | MsgPack-RPC over Unix socket |
| Middleware | Python relay in Docker | None |
| PWM conversion | C++ node (YAML config) | Firmware (JointCal[]) |
| Smoothing/rate-limit | C++ node | Removed (50 Hz policy is smooth) |
| Sensor calibration | C++ node at startup | Removed (done offline if needed) |
| Watchdog | None | Firmware (150 ms → crouch) |

### Steps

1. **Build and test** the new node (see Quickstart above)
2. **Stop the old systemd service:**
   ```bash
   sudo systemctl stop hardware-bridge.service
   sudo systemctl disable hardware-bridge.service
   ```
3. **Remove the Docker container** (optional):
   ```bash
   docker rm -f hardware-bridge
   ```
4. **Remove the Python relay files** (after confirming the new stack works):
   ```bash
   rm -rf big_bertha_bringup/firmware/hardware_bridge_app/python/
   rm big_bertha_bringup/firmware/hardware_bridge_app/app.yaml
   ```
5. **Remove the old C++ node** (after confirming hw_bridge works):
   ```bash
   rm big_bertha_bringup/src/hardware_bridge_node.cpp
   ```
   Then remove the `hardware_bridge_node` target from `CMakeLists.txt`.

## Troubleshooting

### "Failed to connect to router"

```
[hardware_bridge]: Failed to connect to router at /run/arduino-router/rpc.sock
```

- Check the router is running: `systemctl status arduino-router`
- Check the socket exists: `ls -la /run/arduino-router/rpc.sock`
- The `hw_bridge` node will keep trying (background reconnect loop).

### No IMU data on `/imu`

- Check the sketch is flashed and the STM32 is running (LED blink codes)
- Check the MPU9250 I2C wiring and power (3.3 V)
- Run diagnostics: `ros2 topic echo /diagnostics` (if configured)
- The firmware prints `"IMU reading all zeros"` on the serial monitor if
  the sensor is missing or I2C is broken.

### Servos not moving

- Check PCA9685 power (5 V to servos, logic from STM32)
- Verify the watchdog hasn't triggered (LED blink code 1 = PCA9685 missing)
- Check the policy controller is publishing on `/position_controller/commands`:
  ```bash
  ros2 topic echo /position_controller/commands
  ```

### Configure router baud

The sketch calls `Bridge.begin(460800)`. The `arduino-router` must use the
same baud rate. To check or reconfigure:

```bash
# Check current router config
systemctl cat arduino-router

# Add or edit the baud flag (typically in /etc/default/arduino-router or
# the systemd service ExecStart line):
#   --baud 460800

sudo systemctl restart arduino-router
```

## File Layout

```
big_bertha_bringup/
├── src/
│   ├── bridge_rpc_client.hpp    # MsgPack-RPC client (header-only)
│   ├── hw_bridge.cpp            # ROS 2 node (this is it)
│   └── hardware_bridge_node.cpp # Legacy TCP node (remove after migration)
├── tools/
│   └── bridge_poc.cpp           # Standalone PoC benchmark
├── config/
│   └── hardware_bridge.yaml     # Parameters
├── launch/
│   ├── hardware_bringup.launch.py
│   └── big_bertha.launch.py
├── firmware/
│   └── hardware_bridge_app/
│       ├── sketch/sketch.ino    # STM32 firmware
│       └── python/              # Legacy Python relay (remove after migration)
├── CMakeLists.txt
├── package.xml
└── USAGE.md                     # This file
```
