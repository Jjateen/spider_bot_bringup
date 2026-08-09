# Hardware Bridge API Reference

Complete ROS 2 interface documentation for the `hardware_bridge_node`.

## Node Information

**Package:** `big_bertha_bringup`
**Executable:** `hardware_bridge_node`
**Node Name:** `hardware_bridge`
**Namespace:** `/` (global by default)

---

## Published Topics

### `/imu` (sensor_msgs/msg/Imu)

IMU data with bias correction and axis mapping applied. Orientation is the
BNO055 on-chip fused quaternion (absolute heading once magnetometer
calibration completes).

**Rate:** 100 Hz (10 ms interval in firmware; BNO055 fusion ceiling)
**QoS:** `SensorDataQoS` (best effort, volatile)
**Frame ID:** `imu_link`

#### Fields

| Field | Type | Description |
|-------|------|-------------|
| `header.stamp` | Time | ROS time when data received |
| `header.frame_id` | string | `"imu_link"` |
| `linear_acceleration.{x,y,z}` | float64 | m/s², bias-corrected, axis-mapped |
| `angular_velocity.{x,y,z}` | float64 | rad/s, bias-corrected, axis-mapped |
| `orientation.{x,y,z,w}` | float64 | Fused quaternion from BNO055 (NDOF) |
| `orientation_covariance` | float64[9] | From config (diagonal, ~0.5° stddev each) |
| `linear_acceleration_covariance` | float64[9] | From config (diagonal: 0.001) |
| `angular_velocity_covariance` | float64[9] | From config (diagonal: 0.00001) |

#### Calibration

First 200 samples (configurable via `gyro_calibration_samples` / `accel_calibration_samples`) are used for bias estimation. Robot must be **stationary** during node startup.

**Gyro bias removal:**
```
angular_velocity_corrected = (raw - bias) * axis_sign
```

**Accel bias removal:**
```
linear_acceleration_corrected = (raw - bias + gravity_compensation) * axis_sign
```

#### Axis Mapping

Default `imu_axis_sign: [-1.0, -1.0, 1.0]` was carried from the old MPU-6500
mount (180° about Z). It — and the orientation quaternion convention — must be
re-validated on the bench after the BNO055 swap before relying on it.

#### Example

```bash
ros2 topic echo /imu --once
```

```yaml
header:
  stamp:
    sec: 1722451200
    nanosec: 123456789
  frame_id: imu_link
linear_acceleration:
  x: -0.012
  y: 0.034
  z: -9.789
angular_velocity:
  x: 0.0021
  y: -0.0015
  z: 0.0008
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0
```

---

### `/imu/mag` (sensor_msgs/msg/MagneticField)

Raw BNO055 magnetometer field (on-die, also fused internally for absolute yaw).

**Rate:** 100 Hz (with `/imu`)
**QoS:** `SensorDataQoS`
**Frame ID:** `imu_link`

#### Status

Publishes whenever a non-zero reading arrives (the mag register is always
read; NDOF uses it internally for heading). Field scale and sign follow
`mag_axis_sign`.

#### Fields

| Field | Type | Description |
|-------|------|-------------|
| `header.stamp` | Time | ROS time when data received |
| `header.frame_id` | string | `"imu_link"` |
| `magnetic_field.{x,y,z}` | float64 | Tesla, axis-mapped |
| `magnetic_field_covariance` | float64[9] | Unknown (all zeros) |

---

### `/joint_states` (sensor_msgs/msg/JointState)

> Not published by the hardware bridge. On hardware this topic is owned by
> `legged_odometry` — its EWMA simulates the MG995 lag, which is the policy's
> joint feedback. See `leg_odometry/config/legged_odometry.yaml`.

---

## Subscribed Topics

### `/position_controller/commands` (std_msgs/msg/Float64MultiArray)

Joint position commands from policy controller.

**Expected Rate:** 200 Hz (from `big_bertha_policy_controller`)
**QoS:** `QoS(1)` (reliable, volatile)

#### Format

12-element float64 array: `[joint_0, joint_1, ..., joint_11]` in radians.

#### Processing Chain

1. **Validate:** Check `data.size() == 12`, warn and discard if not
2. **Rate limit:** Clamp Δposition to `rate_limit_rad` per step
3. **Smooth (optional):** EWMA with `smoothing_alpha` (default 1.0 = disabled)
4. **Joint limit:** Clamp to `±joint_limit` (default ±π)
5. **Servo convert:** Apply `policy_center`, `direction`, `offset`, angle limits
6. **PWM compute:** Map degrees to 12-bit PWM (102-512 for 500-2500µs @ 50Hz)
7. **Bridge RPC notify:** Send comma-separated PWM string to firmware

#### Example

```bash
# Publish test command (all joints to 0.1 rad)
ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}"
```

**⚠️ Safety:** Start with small values (±0.1 rad) to verify correct servo direction and limits.

---

## Services

All services use `std_srvs/srv/Trigger`:

```yaml
# Request (empty)
---
# Response
bool success
string message
```

### `~/status`

Returns last `hw_status` notification from firmware.

**Namespace:** `<node_namespace>/hardware_bridge/status`
**Default:** `/hardware_bridge/status`

#### Response Format

**success:** `true` if status has been received, `false` if no status yet
**message:** Comma-separated key-value pairs

**Example response:**
```
scan=0,ai_ok=true,pca9685_ok=true,imu_ok=true,servo_calls=1234,
ping_count=567,pwm_write_attempts=1230,pwm_write_fails=0,
pwm_last_fail_ch=-1,pwm_last_fail_code=0,set_servo_last_len=47,
set_servo_last_idx=12,pwm_readback_ch0=307
```

#### Fields

| Key | Type | Description |
|-----|------|-------------|
| `scan` | int | I2C scan bitmask (0=all present, bit0=PCA9685 missing, bit1=BNO055 missing) |
| `ai_ok` | bool | arduino-app-cli health from firmware POV |
| `pca9685_ok` | bool | `(scan & 1) == 0` (servo controller present) |
| `imu_ok` | bool | `(scan & 2) == 0` (BNO055 IMU present) |
| `servo_calls` | int | Total `set_servo_pwms` calls received by firmware |
| `ping_count` | int | Total `ping` calls received |
| `pwm_write_attempts` | int | Total PWM write cycles attempted |
| `pwm_write_fails` | int | Cycles where ≥1 channel failed to write |
| `pwm_last_fail_ch` | int | Last physical channel that failed (-1 if none) |
| `pwm_last_fail_code` | int | I2C error code (0=ok, 2=NACK addr, 3=NACK data, 4=other) |
| `set_servo_last_len` | int | String length of last `set_servo_pwms` data |
| `set_servo_last_idx` | int | Parsed field count (12=clean, <12=truncated) |
| `pwm_readback_ch0` | int | PCA9685 channel 0 OFF register readback (-1=read failed) |

#### Usage

```bash
ros2 service call /hardware_bridge/status std_srvs/srv/Trigger
```

---

### `~/scan_i2c`

Triggers I2C bus scan on firmware, returns device addresses.

**Namespace:** `<node_namespace>/hardware_bridge/scan_i2c`
**Timeout:** 2 seconds

#### Response Format

**success:** `true` if devices found, `false` if scan returned empty
**message:** `addrs=[...]` (decimal addresses)

**Example response:**
```
addrs=[64, 40]
```

- `64` = 0x40 (PCA9685 servo controller)
- `40` = 0x28 (BNO055 IMU; 0x29 if ADR pin high)

#### Usage

```bash
ros2 service call /hardware_bridge/scan_i2c std_srvs/srv/Trigger
```

#### Behavior

1. Node sends `notify("scan_i2c")` to firmware
2. Firmware scans I2C addresses 1-126, stores found addresses
3. Firmware sends `notify("i2c_scan", [addr1, addr2, ...])` back
4. Node waits up to 2 seconds for response, returns addresses

---

### `~/servo_diag`

Runs on-MCU servo diagnostic test.

**Namespace:** `<node_namespace>/hardware_bridge/servo_diag`
**Timeout:** 5 seconds

#### Test Procedure (Firmware-Side)

1. Write test PWM values to all 12 channels (pattern: 204, 307, 204, 307, ...)
2. Wait 200ms for servos to settle
3. Read back PCA9685 OFF registers for all 12 channels
4. Verify readback matches commanded
5. Return JSON report

#### Response Format

**success:** `true` if test completed, `false` if timeout
**message:** JSON string with test results

**Example response (success):**
```json
{
  "phase": "complete",
  "channels_ok": 12,
  "channels_fail": 0,
  "test_pwms": [204, 307, 204, 307, 204, 307, 204, 307, 204, 307, 204, 307],
  "readback": [204, 307, 204, 307, 204, 307, 204, 307, 204, 307, 204, 307],
  "mismatches": []
}
```

**Example response (failure):**
```json
{
  "phase": "complete",
  "channels_ok": 11,
  "channels_fail": 1,
  "test_pwms": [204, 307, ...],
  "readback": [204, 0, ...],
  "mismatches": [{"ch": 1, "expected": 307, "got": 0}]
}
```

#### Usage

```bash
ros2 service call /hardware_bridge/servo_diag std_srvs/srv/Trigger
```

**⚠️ Warning:** Servos will move during this test. Ensure robot is secured.

---

### `~/imu_diag`

Returns last IMU diagnostic string from firmware.

**Namespace:** `<node_namespace>/hardware_bridge/imu_diag`

#### Response Format

**success:** `true` if diagnostic string has been received, `false` otherwise
**message:** Diagnostic string from firmware

**Example response:**
```
chip_id=0xA0,op_mode=0x0c,sys_status=0x04,sys_err=0x00,calib_acc=3,calib_mag=3,calib_gyr=3,calib_sys=3,unit_sel=0x00,axis_cfg=0x24,axis_sign=0x00
```

#### Identity & Calibration

| Field | Meaning |
|-------|---------|
| `chip_id` | 0xA0 = BNO055 answering |
| `op_mode` | 0x00 CONFIG, 0x08 IMU (6-axis rel), 0x0C NDOF (9-axis abs) |
| `sys_status` | Fusion system state (0x04 = running with fusion, 0x05 = running no fusion) |
| `sys_err` | Fusion error code (0x00 = none) |
| `calib_acc/mag/gyr/sys` | Calibration nibbles, each 0..3; `calib_sys` = min of the three |
| `unit_sel` | Unit selection written at init |
| `axis_cfg` / `axis_sign` | AXIS_MAP_CONFIG/SIGN (identity 0x24/0x00 = no host remap) |

#### Usage

```bash
ros2 service call /hardware_bridge/imu_diag std_srvs/srv/Trigger
```

---

## Parameters

See `config/hardware_bridge.yaml` for defaults.

### Connection

#### `router_socket` (string)

arduino-router unix socket path.

**Default:** `"/run/arduino-router/rpc.sock"`
**Auto-probe:** If empty string, tries candidates in order:
1. Value from parameter (if non-empty)
2. `/run/arduino-router/rpc.sock`
3. `/var/run/arduino-router.sock`
4. `/tmp/arduino-router.sock`

**Example:**
```yaml
router_socket: "/custom/path/rpc.sock"
```

---

### Servo PWM

#### `pwm_min` (int)

12-bit PWM value for minimum servo pulse (500 µs @ 50 Hz).

**Default:** `102`
**Range:** 0-4095 (12-bit)
**Calculation:** `(500µs / 20000µs) * 4096 ≈ 102`

#### `pwm_max` (int)

12-bit PWM value for maximum servo pulse (2500 µs @ 50 Hz).

**Default:** `512`
**Calculation:** `(2500µs / 20000µs) * 4096 ≈ 512`

#### `joint_limit` (float64)

Absolute joint position clamp (radians), applied before servo conversion.

**Default:** `3.14159` (π rad)
**Safety:** Prevents commands outside ±π from reaching servos.

---

### Servo Calibration Arrays

All arrays must have exactly **12 elements** (one per joint, in policy order).

#### `servo_lower_limit` (float64[12])

Physical lower angle limit (degrees).

**Default:** `[45, 30, 180, 140, 135, 140, 50, 50, 40, 180, 150, 0]`

#### `servo_upper_limit` (float64[12])

Physical upper angle limit (degrees).

**Default:** `[180, 150, 50, 0, 0, 0, 180, 180, 180, 40, 0, 150]`

#### `servo_offset` (float64[12])

Mounting offset from ideal angle (degrees).

**Default:** `[0, 0, 0, 0, 0, 0, 10, 10, 0, 8, 2, 5]`

#### `servo_channel` (int[12])

PCA9685 PWM channel (0-15) for each joint.

**Default:** `[14, 10, 2, 6, 13, 9, 1, 5, 12, 8, 0, 4]`

#### `servo_direction` (int[12])

Direction multiplier (±1).

**Default:** `[1, 1, 1, 1, 1, 1, -1, -1, -1, -1, 1, 1]`
**Values:** `1` = keep sign, `-1` = flip sign

#### `policy_center` (float64[12])

Policy output value (radians) at which servo is at physical center.

**Default:** `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.57, 1.57, 1.57, 1.57]`
**Note:** Hip/thigh centered at 0, ankle at π/2 (90°)

---

### Motion Control

#### `command_rate_hz` (float64)

Expected policy publish rate (Hz).

**Default:** `200.0`
**Used for:** Computing `rate_limit_rad`

#### `max_joint_rate_rad_s` (float64)

Maximum joint angular velocity (rad/s).

**Default:** `6.54` (MG995 spec: 60°/0.16s ≈ 6.54 rad/s @ 6V)
**Calculation:** `rate_limit_rad = max_joint_rate_rad_s / command_rate_hz`
**Example:** `6.54 / 200 = 0.0327 rad/step`

#### `smoothing_alpha` (float64)

EWMA smoothing coefficient.

**Default:** `1.0` (disabled)
**Range:** 0.0-1.0
**Formula:** `smoothed = alpha * new + (1 - alpha) * old`
**Note:** Rate limiting is primary safety mechanism; smoothing is optional.

---

### IMU Covariance

#### `orientation_covariance` (float64[9])

Orientation quaternion covariance (row-major).

**Default:** `[-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`
**Note:** First element -1.0 = orientation not estimated

#### `linear_acceleration_covariance` (float64[9])

Acceleration covariance (m/s²)².

**Default:** `[0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]`
**Diagonal:** 0.001 (inherited from the MPU-6500 era, kept as a safe margin)

#### `angular_velocity_covariance` (float64[9])

Gyro covariance (rad/s)².

**Default:** `[0.00001, 0.0, 0.0, 0.0, 0.00001, 0.0, 0.0, 0.0, 0.00001]`
**Diagonal:** 0.00001

---

### IMU Calibration

#### `gyro_calibration_enabled` (bool)

Enable gyro bias estimation at startup.

**Default:** `true`

#### `gyro_calibration_samples` (int)

Number of IMU samples to collect for gyro bias.

**Default:** `200` (~2.2 seconds @ 91 Hz measured rate)

#### `accel_calibration_enabled` (bool)

Enable accel bias estimation at startup.

**Default:** `true`

#### `accel_calibration_samples` (int)

Number of IMU samples to collect for accel bias.

**Default:** `200`

#### `imu_axis_sign` (float64[3])

Axis flip for chip frame → base_link.

**Default:** `[-1.0, -1.0, 1.0]`
**Explanation:** Carried over from the old MPU-6500 mount (180° about Z);
must be re-validated on the bench after the BNO055 swap.

#### `mag_axis_sign` (float64[3])

Magnetometer axis mapping (raw BNO055 mag field).

**Default:** `[-1.0, 1.0, -1.0]`
**Note:** Same caveat as `imu_axis_sign` — not yet bench-validated.

---

### Debug/Testing

#### `single_joint_mode` (bool)

Test mode: drive only one joint.

**Default:** `false`

#### `single_joint_index` (int)

Which joint to drive in single-joint mode.

**Default:** `10` (FR calf, Revolute_118)

---

## Bridge RPC Protocol (Internal)

Communication between C++ node and firmware via arduino-router unix socket.

### Wire Format

MsgPack-RPC over unix socket:

- **Notification:** `[2, "method", [params...]]`
- **Request:** `[0, msgid, "method", [params...]]`
- **Response:** `[1, msgid, error, result]`
- **Registration:** `[0, msgid, "$/register", ["method"]]`

### Outbound (Node → Firmware)

#### `set_servo_pwms` (string)

Notification with comma-separated PWM values.

**Format:** `[2, "set_servo_pwms", ["307,153,204,..."]]`
**Params:** Single string with 12 comma-separated integers

#### `scan_i2c` (no params)

Trigger I2C bus scan.

**Format:** `[2, "scan_i2c", []]`

#### `servo_diag` (no params)

Trigger servo diagnostic test.

**Format:** `[2, "servo_diag", []]`

#### `ping` (no params)

Connectivity test (not used by node).

**Format:** `[2, "ping", []]`

### Inbound (Firmware → Node)

#### `imu` (string CSV, 15 fields)

IMU data notification — ONE comma-separated string (a multi-arg notify would
exceed the router argument limit).

**Format:** `[2, "imu", ["qw,qx,qy,qz,gx,gy,gz,ax,ay,az,mx,my,mz,sample,timestamp"]]`
**Params:**
- `qw, qx, qy, qz` — Fused orientation quaternion (BNO055)
- `gx, gy, gz` — Angular velocity (rad/s)
- `ax, ay, az` — Linear acceleration incl. gravity (m/s²)
- `mx, my, mz` — Magnetic field (Tesla, raw BNO055)
- `sample` — Sample counter
- `timestamp` — Firmware timestamp (ms)

#### `hw_status` (11 ints)

Hardware status notification.

**Format:** `[2, "hw_status", [scan, ai, servo_calls, ping_count, ...]]`
**Params:** See `~/status` service documentation for field meanings.

#### `i2c_scan` (array of ints)

I2C scan result.

**Format:** `[2, "i2c_scan", [64, 40, ...]]`
**Params:** Device addresses found (7-bit, decimal)

#### `imu_diag` (string)

IMU diagnostic report.

**Format:** `[2, "imu_diag", ["chip_id=0xA0, op_mode=0x0c, calib_sys=3, ..."]]`
**Params:** Single diagnostic string

#### `servo_diag_result` (string)

Servo diagnostic test result.

**Format:** `[2, "servo_diag_result", ["{\"phase\":\"complete\", ...}"]]`
**Params:** JSON string with test results

#### `pong` (int)

Ping response.

**Format:** `[2, "pong", [count]]`
**Params:** Ping counter

---

## C++ API (Internal Use)

For developers extending the node.

### BridgeRPCClient

**Header:** `big_bertha_bringup/src/bridge_rpc_client.hpp`

#### Constructor

```cpp
explicit BridgeRPCClient(std::vector<std::string> socket_candidates);
```

#### Methods

```cpp
bool start();  // Connect, register handlers, start reader thread
void stop();   // Shutdown cleanly

void notify(const std::string & method);
void notify(const std::string & method, const std::vector<double> & params);
void notify_int(const std::string & method, const std::vector<int64_t> & params);
void notify_str(const std::string & method, const std::string & text);

void provide(const std::string & method, FloatHandler handler);
void provide_str(const std::string & method, StringHandler handler);

std::string connected_path();
bool is_connected();
```

#### Handler Types

```cpp
using FloatHandler = std::function<void(const std::vector<double> &)>;
using StringHandler = std::function<void(const std::string &)>;
```

**Thread Safety:** All methods thread-safe. Handlers called from internal reader thread.

---

## Example Usage

### Monitor IMU in Real-Time

```bash
ros2 topic echo /imu
```

### Send Test Servo Command

```bash
# Small command (safe for testing)
ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

### Run Full Diagnostics

```bash
python3 scripts/servo_diag.py
```

### Check Hardware Health

```bash
ros2 service call /hardware_bridge/status std_srvs/srv/Trigger
```

### Debug I2C Issues

```bash
ros2 service call /hardware_bridge/scan_i2c std_srvs/srv/Trigger
```

---

## Troubleshooting

See [DEPLOYMENT.md](DEPLOYMENT.md) for comprehensive troubleshooting guide.

**Quick checks:**
- No `/imu`: Check I2C scan, verify IMU present
- Servos won't move: Check PCA9685 in scan, verify external power
- High latency: Check `ros2 topic hz /imu`, should be ~100 Hz
- Connection fails: Check `ls -l /run/arduino-router/rpc.sock`
