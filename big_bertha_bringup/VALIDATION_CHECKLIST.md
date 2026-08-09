# Hardware Bridge Validation Checklist

Systematic testing procedure for hardware bridge deployment. Complete this checklist before trusting the system with autonomous operation.

**Purpose:** Ensure bridge node, firmware, and hardware are working correctly before running full locomotion stack.

**Duration:** ~30-45 minutes for full checklist

**Safety:** Keep emergency stop (power switch) accessible throughout testing.

---

## Pre-Flight Checks

### Environment Setup

- [ ] **Arduino UNO Q powered on** and responsive
- [ ] **All 12 servos connected** to PCA9685 (verify channel mapping matches config)
- [ ] **External power supply connected** to PCA9685 V+ terminal (6V, 10A recommended)
- [ ] **IMU wired correctly** (SDA, SCL, VCC, GND)
- [ ] **Robot secured** (test stand, bench mount, or held off ground)
- [ ] **Emergency stop accessible** (power switch or E-stop button within reach)
- [ ] **No obstacles** in servo range of motion
- [ ] **Workspace clear** of tools, wires that could snag

### Software Prerequisites

- [ ] **ROS 2 Jazzy sourced** (`echo $ROS_DISTRO` → `jazzy`)
- [ ] **Workspace built** (`colcon build --packages-select big_bertha_bringup`)
- [ ] **Install sourced** (`source ~/ros2_ws/install/setup.bash`)
- [ ] **arduino-app-cli running** (`systemctl status arduino-app-cli` → active)
- [ ] **arduino-router running** (`systemctl status arduino-router` → active)
- [ ] **Correct branch** (`git branch --show-current` → `pr/hw-bringup`)

---

## Phase 1: Firmware Upload & Verification

### Firmware Upload

- [ ] **Navigate to workspace**
  ```bash
  cd ~/ros2_ws/src/spider_bot_bringup
  ```

- [ ] **Run upload script**
  ```bash
  ./scripts/upload_firmware.sh
  ```

- [ ] **Verify all steps pass** (all ✔ symbols, no ✘)
  - [ ] Prerequisites check passed
  - [ ] Sketch synced (file size > 0)
  - [ ] Compilation successful
  - [ ] Upload successful
  - [ ] Bridge RPC socket exists
  - [ ] Ping test successful

### Firmware Status

- [ ] **Check app status**
  ```bash
  arduino-app-cli app status
  ```
  Expected: `hardware_bridge_app` running

- [ ] **Verify socket exists**
  ```bash
  ls -l /run/arduino-router/rpc.sock
  ```
  Expected: `srwxrwxrwx` (socket file present)

- [ ] **Check firmware logs** (no I2C errors)
  ```bash
  journalctl -u arduino-app-cli -n 50 | grep -i error
  ```
  Expected: No critical errors

---

## Phase 2: Node Startup & Calibration

### Node Launch

- [ ] **Start bridge node** (Terminal 1)
  ```bash
  ros2 run big_bertha_bringup hardware_bridge_node --ros-args --log-level info
  ```

### Startup Logs

- [ ] **Connected to arduino-router**
  ```
  [INFO] [hardware_bridge]: Connected to arduino-router at /run/arduino-router/rpc.sock
  ```

- [ ] **Calibration started**
  ```
  [INFO] [hardware_bridge]: calibrating sensors (200 samples)...
  ```

- [ ] **Robot remained stationary** during calibration (2-3 seconds)

- [ ] **Gyro bias logged** (reasonable values)
  ```
  [INFO] [hardware_bridge]: gyro bias: gx=0.XXXXXX gy=0.XXXXXX gz=0.XXXXXX rad/s
  ```
  ✅ **Expected:** |gx|, |gy|, |gz| < 0.1 rad/s (typically < 0.05)
  ⚠️ **Warning if:** Any axis > 0.1 rad/s (recalibrate or increase sample count)

- [ ] **Accel bias logged**
  ```
  [INFO] [hardware_bridge]: accel bias: ax=0.XXXXXX ay=0.XXXXXX az=0.XXXXXX m/s²
  ```
  ✅ **Expected:** |ax|, |ay| < 0.5 m/s², |az| < 0.5 m/s² (after gravity subtraction)

- [ ] **Accel variance logged** (noise estimate)
  ```
  [INFO] [hardware_bridge]: accel var:  ax=X.XXXe-XX ay=X.XXXe-XX az=X.XXXe-XX (m/s²)²
  ```

- [ ] **Calibration complete**
  ```
  [INFO] [hardware_bridge]: calibration complete (200 samples)
  ```

- [ ] **No errors or warnings** in startup logs

### Calibration Quality Check

If calibration values seem wrong, restart node and ensure:
- Robot is perfectly stationary (no vibrations, fans off)
- IMU has warmed up (wait 30s after power-on)
- Surface is level (accel should read ~9.8 m/s² on Z-axis before correction)

---

## Phase 3: Topic Validation

Open a second terminal for topic checks.

### `/imu` Topic

- [ ] **Topic exists**
  ```bash
  ros2 topic list | grep /imu
  ```
  Expected: `/imu` present

- [ ] **Correct message type**
  ```bash
  ros2 topic info /imu
  ```
  Expected: `Type: sensor_msgs/msg/Imu`, QoS: best effort

- [ ] **Publishing at expected rate**
  ```bash
  ros2 topic hz /imu
  ```
  ✅ **Expected:** 95-105 Hz (target 100 Hz — BNO055 fusion ceiling)
  ⚠️ **Warning if:** < 80 Hz or > 120 Hz

- [ ] **Data looks reasonable** (at rest)
  ```bash
  ros2 topic echo /imu --once
  ```
  Check:
  - [ ] `header.frame_id` = `"imu_link"`
  - [ ] `linear_acceleration.z` ≈ -9.8 m/s² (gravity, base_link Z-up)
  - [ ] `linear_acceleration.x`, `.y` ≈ 0 ± 0.5 m/s²
  - [ ] `angular_velocity.{x,y,z}` ≈ 0 ± 0.05 rad/s (after calibration)
  - [ ] `orientation` all zeros (not estimated)

- [ ] **Responds to motion**
  - [ ] Tilt board forward → `linear_acceleration.x` changes
  - [ ] Tilt board right → `linear_acceleration.y` changes
  - [ ] Rotate board about Z → `angular_velocity.z` non-zero

### `/imu/mag` Topic

- [ ] **Topic does NOT exist** (current hardware has no magnetometer)
  ```bash
  ros2 topic list | grep /imu/mag
  ```
  Expected: No output (topic not present)

If `/imu/mag` appears, this is unexpected (firmware should not publish with no mag).

### `/joint_states` Topic

> The hardware bridge never publishes `/joint_states`. `legged_odometry` owns
> this topic — its EWMA simulates the MG995 lag that is the policy's feedback.
> Run `legged_odometry` alongside the bridge for these checks.

- [ ] **Topic exists**
  ```bash
  ros2 topic list | grep /joint_states
  ```
  Expected: `/joint_states` present (from `legged_odometry`)

- [ ] **NOT publishing yet** (no commands sent)
  ```bash
  timeout 2 ros2 topic hz /joint_states
  ```
  Expected: Timeout (no messages yet)

This will be tested in Phase 5 (servo commands).

---

## Phase 4: Service Validation

### Hardware Status Service

- [ ] **Service exists**
  ```bash
  ros2 service list | grep status
  ```
  Expected: `/hardware_bridge/status`

- [ ] **Call status service**
  ```bash
  ros2 service call /hardware_bridge/status std_srvs/srv/Trigger
  ```

- [ ] **Response success=true**

- [ ] **Parse response message**, verify:
  - [ ] `scan=0` (no devices missing)
  - [ ] `pca9685_ok=true` (servo controller present)
  - [ ] `imu_ok=true` (BNO055 present)
  - [ ] `servo_calls >= 0` (counter initialized)
  - [ ] `pwm_write_fails=0` (no I2C errors yet)
  - [ ] `pwm_last_fail_ch=-1` (no failures)
  - [ ] `set_servo_last_idx=0` (no commands yet)

### I2C Scan Service

- [ ] **Call scan service**
  ```bash
  ros2 service call /hardware_bridge/scan_i2c std_srvs/srv/Trigger
  ```

- [ ] **Response success=true**

- [ ] **Response message shows expected devices**
  ```
  addrs=[64, 40]
  ```
  - [ ] `64` (0x40) = PCA9685 servo controller
  - [ ] `40` (0x28) = BNO055 IMU

  ⚠️ **If missing 64:** PCA9685 not detected, check wiring/power
  ⚠️ **If missing 40:** BNO055 not detected, check wiring/address pin (0x29 if ADR high)

### IMU Diagnostic Service

- [ ] **Call imu_diag service**
  ```bash
  ros2 service call /hardware_bridge/imu_diag std_srvs/srv/Trigger
  ```

- [ ] **Response success=true**

- [ ] **Parse diagnostic string**, verify:
  - [ ] `chip_id=0xA0` (BNO055 answering)
  - [ ] `op_mode=0x0c` (NDOF fusion active)
  - [ ] `sys_status=0x04` (fusion running) and `sys_err=0x00` (no error)

### BNO055 Calibration Service

The on-chip fusion converges on its own, but absolute yaw needs a level
figure-of-eight mag pass before it holds. Check progress via `~/imu_diag`
(call repeatedly; the `calib_*` nibbles update every 1 s).

- [ ] **Stationary gyro calib:** place robot level and still until `calib_gyr=3`
- [ ] **Accel calib:** tip the chassis (each face up briefly) until `calib_acc=3`
- [ ] **Mag calib:** rotate the chassis in figure-of-eight while level until `calib_mag=3`
- [ ] **System calib:** `calib_sys=3` (equals the minimum of the three above) —
      heading is now absolute and stable

### IMU Axis / Sign Validation (BENCH — after swap)

- [ ] **Pitch:** nose-up → `/imu` roll/pitch increases in the expected sign
- [ ] **Roll:** right-tilt → `/imu` roll increases in the expected sign
- [ ] **Yaw:** rotate 90° clockwise → fused yaw (quaternion) turns the same way
- [ ] If any sign is opposite, update `imu_axis_sign` (and revisit the quaternion
      convention) in `config/hardware_bridge.yaml`, then repeat

### Servo Diagnostic Service (Deferred)

**Do NOT run yet** — this moves servos. Will be tested in Phase 5 after verifying safe motion.

---

## Phase 5: Servo Command Test (⚠️ SAFETY CRITICAL)

**BEFORE PROCEEDING:**
- [ ] Emergency stop is within reach
- [ ] Robot is secured (cannot fall or collide)
- [ ] No hands/objects near servos
- [ ] Observer ready to cut power if needed

### Single-Joint Test (Safe)

- [ ] **Publish test command** (all joints to neutral, small offset on one joint)
  ```bash
  ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray \
    "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0]}"
  ```
  (Joint 9 = HR hip, 0.1 rad offset)

- [ ] **Observe servo motion**
  - [ ] One servo moved (HR hip, channel per config)
  - [ ] Motion was smooth (not jerky)
  - [ ] Direction correct (check with hand, verify no binding)
  - [ ] Servo stopped at target (no oscillation)

- [ ] **Check `/joint_states` now publishes**
  ```bash
  ros2 topic hz /joint_states
  ```
  Expected: ~200 Hz (matches command rate; publisher is `legged_odometry`)

- [ ] **Verify joint_states content**
  ```bash
  ros2 topic echo /joint_states --once
  ```
  - [ ] 12 joint names present
  - [ ] Position[9] ≈ 0.1 (commanded value)
  - [ ] Velocities are finite (not NaN or inf)

### Return to Neutral

- [ ] **Command all joints to 0**
  ```bash
  ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray \
    "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
  ```

- [ ] **Servo returned smoothly** (no sudden jerk)

### Test Each Leg (One at a Time)

**FL (Front-Left, joints 0-2):**
- [ ] Hip: `{data: [0.2, 0.0, 0.0, ...]}`
- [ ] Thigh: `{data: [0.0, 0.2, 0.0, ...]}`
- [ ] Calf: `{data: [0.0, 0.0, 0.2, ...]}`
- [ ] Verify motion direction matches expected (hip=yaw, thigh=lift, calf=extend)
- [ ] Return to neutral between tests

**FR (Front-Right, joints 3-5):**
- [ ] Hip: `{data: [0.0, 0.0, 0.0, 0.2, ...]}`
- [ ] Thigh: `{data: [0.0, 0.0, 0.0, 0.0, 0.2, ...]}`
- [ ] Calf: `{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.2, ...]}`
- [ ] Verify motion
- [ ] Return to neutral

**HL (Hind-Left, joints 6-8):**
- [ ] Hip, Thigh, Calf (same pattern)
- [ ] Verify motion
- [ ] Return to neutral

**HR (Hind-Right, joints 9-11):**
- [ ] Hip, Thigh, Calf
- [ ] Verify motion
- [ ] Return to neutral

### Full 12-Joint Test

- [ ] **Command small motion on all joints**
  ```bash
  ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray \
    "{data: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}"
  ```

- [ ] **All 12 servos moved simultaneously**
- [ ] **No binding or collision** between legs
- [ ] **Motion was smooth**

- [ ] **Return to neutral**
  ```bash
  ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray \
    "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
  ```

### Servo Diagnostic Service

Now safe to run full diagnostic.

- [ ] **Call servo_diag service**
  ```bash
  ros2 service call /hardware_bridge/servo_diag std_srvs/srv/Trigger
  ```

- [ ] **Servos moved briefly** (test pattern: 204, 307 alternating)

- [ ] **Response success=true**

- [ ] **Parse JSON response**, verify:
  - [ ] `"phase": "complete"`
  - [ ] `"channels_ok": 12` (all channels passed)
  - [ ] `"channels_fail": 0` (no failures)
  - [ ] `"mismatches": []` (empty array)

  ⚠️ **If any failures:** Check `mismatches` array, verify PCA9685 wiring/power

---

## Phase 6: Stress Testing

### Continuous IMU Stream

- [ ] **Let IMU run for 2 minutes**, check logs
  ```bash
  # Watch for "gyro accumulated drift" messages (every 10 seconds)
  ```

- [ ] **Drift accumulation reasonable**
  - [ ] After 60s: |drift_x|, |drift_y|, |drift_z| < 0.05 rad
  - [ ] After 120s: < 0.1 rad

  ⚠️ **If drift > 0.1 rad/min:** Consider increasing calibration samples

- [ ] **No I2C timeout errors** in logs

### Rapid Servo Commands

- [ ] **Publish commands at 200 Hz** (use policy controller or script)
  ```bash
  # If policy controller available:
  ros2 launch big_bertha_policy_controller policy_controller.launch.py
  # Let run for 30 seconds
  ```

- [ ] **Check hardware status** after stress
  ```bash
  ros2 service call /hardware_bridge/status std_srvs/srv/Trigger
  ```
  - [ ] `pwm_write_fails` still low (< 5 failures acceptable)
  - [ ] `servo_calls` > 5000 (200 Hz × 30s = 6000)

- [ ] **No servo overheating** (touch servo bodies, should be warm but not hot)

### Reconnection Test

- [ ] **Kill and restart firmware**
  ```bash
  arduino-app-cli app restart ~/ArduinoApps/hardware_bridge_app
  ```

- [ ] **Node log shows reconnection**
  ```
  [WARN] [hardware_bridge]: socket read error, reconnecting...
  [INFO] [hardware_bridge]: Connected to arduino-router at ...
  ```

- [ ] **IMU resumes publishing** within 5 seconds

- [ ] **Servo commands work** after reconnection

---

## Phase 7: Known Issues Verification

### No Magnetometer

- [ ] **Confirmed:** `/imu/mag` does not publish (as expected)
- [ ] **IMU diag shows:** `chip_id=0xa0` (BNO055, `op_mode=0x08` NDOF) and `calib_sys=3`
- [ ] **Understood:** Yaw will drift without absolute heading reference

### I2C Clock 50 kHz

- [ ] **Check firmware logs** for I2C speed (should be 50 kHz, not 100 kHz)
  ```bash
  journalctl -u arduino-app-cli -n 100 | grep -i "i2c\|clock"
  ```

- [ ] **Acceptable:** Some I2C timeouts may occur (< 1% failure rate)

### Calibration Required

- [ ] **Verified:** Node requires 2-3 second stationary startup
- [ ] **Documented:** Robot must not move during calibration phase

### No Position Feedback

- [ ] **Understood:** `/joint_states` publishes commanded positions, not actual
- [ ] **Limitation noted:** Cannot detect servo stalls, binding, or load

---

## Phase 8: Integration Readiness

### Prerequisites for Full Stack

- [ ] **All Phase 1-7 checks passed**
- [ ] **Emergency stop tested and accessible**
- [ ] **Robot mechanically sound** (no loose screws, worn linkages)
- [ ] **Power supply adequate** (measured: no voltage drop under load)
- [ ] **Workspace clear** (no trip hazards, cables secured)

### Final Go/No-Go

**GO if:**
- ✅ All critical checks passed (IMU publishing, servos responding, diagnostics clean)
- ✅ No I2C errors or hardware failures
- ✅ Servo motion is smooth and predictable
- ✅ Rate limiting working (no sudden jerks)

**NO-GO if:**
- ❌ IMU not publishing or publishing zeros
- ❌ Servo diagnostic shows failures
- ❌ I2C scan missing devices
- ❌ Servos move erratically or unpredictably
- ❌ pwm_write_fails > 10% of attempts
- ❌ Calibration produces wildly incorrect bias (|bias| > 0.2 rad/s)

---

## Post-Validation Actions

### If All Tests Passed (GO)

1. **Document results** (save logs, note any warnings)
2. **Proceed to full stack:** `ros2 launch big_bertha_bringup big_bertha.launch.py`
3. **Monitor closely** during first autonomous operation
4. **Keep emergency stop ready**

### If Tests Failed (NO-GO)

1. **Document failures** (which checks, what symptoms)
2. **Save logs**
   ```bash
   journalctl -u hardware-bridge > /tmp/bridge.log
   journalctl -u arduino-app-cli > /tmp/firmware.log
   ```
3. **Troubleshoot** using [DEPLOYMENT.md](DEPLOYMENT.md) troubleshooting section
4. **If unresolved:** File GitHub issue with logs

### Periodic Re-Validation

Run this checklist:
- **After firmware updates** (sketch.ino changes)
- **After configuration changes** (servo params, IMU calibration)
- **After hardware maintenance** (servo replacement, wiring changes)
- **Monthly** (even if no changes, verify hardware health)

---

## Troubleshooting Quick Reference

| Symptom | Check | Fix |
|---------|-------|-----|
| Node won't connect | Socket exists? | Restart arduino-router |
| IMU not publishing | I2C scan shows 40 (0x28)? | Check BNO055 wiring |
| Servos won't move | I2C scan shows 64? | Check PCA9685 power |
| High gyro drift | Calibration bias > 0.1? | Increase samples, ensure stationary |
| I2C timeouts | pwm_write_fails high? | Check wiring, reseat connectors |
| Servo jitters | Rate limit too high? | Decrease max_joint_rate_rad_s |

See [DEPLOYMENT.md](DEPLOYMENT.md) for comprehensive troubleshooting.

---

## Validation Sign-Off

**Tested by:** ___________________
**Date:** ___________________
**Result:** ☐ PASS  ☐ FAIL (with issues documented)
**Notes:**

---

**Next Steps:**
- If PASS: Proceed to full locomotion testing
- If FAIL: Troubleshoot and re-run checklist

**Safety Reminder:** Always maintain emergency stop capability during autonomous operation.
