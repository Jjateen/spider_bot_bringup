# Migrating the IMU: MPU-6500 → BNO055

Plan for swapping the current IMU for a GY-BNO055 board. Written 2026-07-28,
before the hardware is in hand — the verification gates matter more than the
code sketch, because several assumptions here need checking against the real
part.

## Why

The fitted IMU has no magnetometer. `WHO_AM_I (0x75)` reads `0x70`, which is an
**MPU-6500** — a six-axis die. It is sold as an "MPU-9265", which is not an
InvenSense part number; TDK only ever made the MPU-9250 (`0x71`) and MPU-9255
(`0x73`). Three independent checks agree the magnetometer is absent rather
than unreachable:

| check | result |
|---|---|
| `WHO_AM_I` | `0x70` = MPU-6500, no magnetometer die |
| main-bus sweep 1..126, `BYPASS_EN` confirmed set | only `0x40` (PCA9685), `0x68` (IMU) |
| aux-bus sweep `0x08..0x77` via MPU I2C master, at 400 kHz and 258 kHz | 0 devices, both passes with `st_or=0x1` proving slaves actively NACKed |

Consequence today: the stack is 6-DOF. Roll and pitch come from gravity; **yaw
free-runs on gyro integration with no absolute reference** and drifts (~5.6°
observed over a single 40 s demo run). A BNO055 fixes that, because it has a
real magnetometer.

## What does NOT change

Establish this first — it is most of the reason the swap is cheap.

**Training: nothing.** The policy consumes `projected_gravity_b` (derived from
an orientation quaternion) plus angular velocity. It has no idea which chip
produced them, and Isaac trains against ground-truth root states with no IMU
model. `_imu_negate` in `big_bertha_env.py` is `[1.0, 1.0, 1.0]` — a no-op hook,
not a live correction — and `ImuCfg` uses `rot=(1,0,0,0)`. No retrain, no ONNX
re-export, no contract change.

**URDF: nothing.** `imu_joint` keeps `rpy="0 0 0"` — that encodes the training
contract ("the IMU reports in base_link frame"), not the chip's mounting. The
`xyz` can also stay: the only consumer that would care about sensor position is
lever-arm linear acceleration, and `ekf.yaml` fuses exactly one IMU channel:

```yaml
imu0_config: [false, false, false,
              false, false, false,
              false, false, false,
              false, false, true,     # vyaw (yaw rate) -- the only true
              false, false, false]    # ax, ay, az all false
```

Nothing else consumes `imu_link` beyond the bridge stamping `frame_id`.

**So the entire migration lives in `big_bertha_bringup`.**

## Two routes — pick before starting

### Route B (recommended): AMG mode, keep Madgwick

The chip streams raw accel / gyro / mag; `imu_filter_madgwick` fuses them as it
does now, with `use_mag: true` finally meaningful.

- axis correction stays **per-component**, in the existing `imu_axis_sign` and
  `mag_axis_sign` config vectors — no quaternion maths anywhere in the stack
- keeps the current pipeline shape; smallest diff; easiest to roll back
- costs: fusion runs on the A35 instead of the BNO055's dedicated M0, and
  magnetometer hard/soft-iron calibration becomes our job

### Route A: NDOF mode, chip does the fusion

The BNO055 outputs an absolute-referenced quaternion directly.

- better fusion, self-calibrating, `imu_filter_madgwick` gets deleted
- axis correction **cannot** use `imu_axis_sign`: you cannot negate a quaternion
  component-wise and get a rotation. Use the chip's `AXIS_MAP_CONFIG (0x41)` /
  `AXIS_MAP_SIGN (0x42)` so it emits in base_link frame directly, and set
  `imu_axis_sign` to `[1,1,1]`
- `AXIS_MAP` expresses only the 24 orthogonal remaps (axis permutation + sign).
  A non-90° mount needs a correction quaternion composed in the bridge instead

Route B unless the yaw quality from A is specifically wanted.

## Steps

### 1. Bench-check the part before touching the robot

Power it standalone and confirm over I2C:

- address responds at **`0x28`** (or `0x29` if ADR is pulled high)
- `CHIP_ID (0x00)` = `0xA0`; `ACC_ID 0x01`=`0xFB`, `MAG_ID 0x02`=`0x32`,
  `GYR_ID 0x03`=`0x0F`
- allow **~650 ms after power-on** before the chip answers at all

Do not skip this. It is the same check that would have caught the MPU-6500
substitution on day one, and BNO055 clones exist too.

### 2. Firmware — `firmware/hardware_bridge_app/sketch/sketch.ino`

Replace the MPU driver. Register essentials:

| what | register | note |
|---|---|---|
| chip id | `0x00` | expect `0xA0` |
| page select | `0x07` | data lives on page 0 |
| unit select | `0x3B` | default: accel m/s², gyro dps. Set explicitly, don't assume |
| operating mode | `0x3D` | `0x00` CONFIG, `0x07` AMG (route B), `0x0C` NDOF (route A) |
| reset | `0x3F` bit 5 | then wait ~650 ms |
| accel | `0x08`, 6 B | **100 LSB = 1 m/s²** |
| mag | `0x0E`, 6 B | **16 LSB = 1 µT** → ×1e-6 for `sensor_msgs/MagneticField` (tesla) |
| gyro | `0x14`, 6 B | **16 LSB = 1 dps** → ×π/180 for rad/s |
| quaternion | `0x20`, 8 B | route A only. **2^14 LSB = 1.0** |
| calib status | `0x35` | `[7:6]` sys, `[5:4]` gyr, `[3:2]` acc, `[1:0]` mag, each 0–3 |

Mode changes must pass through CONFIG mode. CONFIG→operating takes ~7 ms,
operating→CONFIG ~19 ms.

**Critical — the RPC argument limit.** The `imu` notify already carries 11
arguments. Bridge RPC silently drops arguments past ~12, and a dropped argument
arrives on the Python side as its default, which is indistinguishable from a
real zero. This already produced a false "no devices found" during the
magnetometer investigation. Route A adds 4 quaternion values (15 total) and
**must** use the single-string form, as `set_servo_pwms` and `imu_diag` already
do. Route B stays at 11 and is safe as-is.

Add calib status to the `imu_diag` string either way.

### 3. Bridge node — `src/hardware_bridge_node.cpp`

- route B: no structural change. The magnetometer guard
  (`mag_ok && |m| > 1e-9`) starts passing on its own, so `/imu/mag` begins
  publishing for the first time
- route A: populate `msg.orientation` (currently never set) and replace
  `orientation_covariance: [-1, ...]` — `-1` means "orientation not estimated"
  and consumers act on it

### 4. Config

- `config/hardware_bridge.yaml` — measure and set `imu_axis_sign` and
  `mag_axis_sign` on the bench (see step 6). Delete the "no magnetometer on
  this board" notes and the warning that the signs cannot express the MPU-9250
  X/Y swap; both are specific to the old part
- `config/imu_filter_madgwick.yaml` — route B: `use_mag: true`, and set
  `mag_bias_x/y/z` from calibration. Route A: delete the node from
  `launch/hardware_bringup.launch.py` and `launch/big_bertha.launch.py`
- `config/ekf.yaml` — **the real prize.** With a magnetometer-referenced
  absolute yaw, the IMU can own yaw instead of contributing rate only. Change
  `imu0_config` index 5 (yaw) to `true`. Note the existing comment warning
  about two absolute-yaw sources fighting: if IMU yaw is enabled, odom must
  stop providing absolute yaw, or they will conflict exactly as before

### 5. Magnetometer calibration

Route A self-calibrates — monitor `CALIB_STAT (0x35)` and gate startup until
mag reaches 3.

Route B needs hard-iron offset and soft-iron scaling done manually. Madgwick
exposes `mag_bias_x/y/z` (hard iron only); a soft-iron 3×3 has to be applied in
the bridge before publishing. Reference:
<https://www.hackster.io/walid-abdelazeem/mpu9250-mpu9265-calibration-9-dof-c9da8e>

Calibrate **on the robot, powered, with motors on** — the servos and wiring are
the main hard-iron sources and calibrating the bare board is worthless.

### 6. Determining the axis mapping

The BNO055's X does not point the same way as the MPU-6500's X on these
boards. Do not guess it from the silkscreen:

1. lay the robot level, read `/imu` — the axis reading ≈ +9.81 is the one
   pointing **up**; it must map to base_link **+Z**
2. pitch the robot nose-down; the axis that goes positive is base_link **+X**
3. roll left; the axis that goes positive is base_link **+Y**
4. write the resulting permutation/signs into `imu_axis_sign` (route B) or
   `AXIS_MAP_CONFIG`/`AXIS_MAP_SIGN` (route A)
5. sanity check: at rest `projected_gravity_b.z` must be ≈ **-1**, which is what
   the policy was trained on

### 7. Accel calibration — still needed

The current sensor reads **11.07 m/s²** at rest against 9.81, and
`accel_cfg`/`gyro_cfg` both read `0x0`, so the ±2 g / ±250 dps divisors are
correct — it is a genuine sensor scale error, not a mis-set range. The bridge
currently hides it by folding 1.284 m/s² into a startup z-bias, which only
cancels while the robot is level and returns the moment it tilts.

Re-measure on the BNO055 before assuming it is inherited. If `|a|` at rest is
not ≈9.81 in **several orientations**, run the 6-position routine and extend the
bridge with a per-axis scale (it currently supports bias only).

## Verification gates

Run in order; do not proceed past a failure.

1. `CHIP_ID` = `0xA0`, and calib status reachable
2. `/imu` publishing, `|accel|` ≈ 9.81 at rest **in at least 3 orientations**
   (one orientation only proves the bias trick, not the scale)
3. `/imu/mag` publishing non-zero, `|B|` in the 25–65 µT range — the plausible
   band for Earth's field. A wildly different magnitude means calibration or
   scaling is wrong
4. `/filtered/imu` orientation populated; `projected_gravity_b.z` ≈ -1 at rest
5. **yaw stability** — the whole point. Leave the robot still for 5 minutes and
   confirm yaw does not walk. The old setup drifted ~5.6° in 40 s
6. rotate the robot 90° by hand; confirm the reported yaw changes by 90° in the
   same direction
7. `demo_straight` still meets its gate, and the patrol still completes 3/3

## Risks

**I2C clock stretching.** The BNO055 is known for stretching SCL beyond spec
and upsetting masters. This STM32U585 already runs at 50 kHz because of driver
problems (Zephyr #83550), so the two issues could compound. If the bus proves
unreliable, the BNO055 also supports **UART at 115200**, which sidesteps I2C
entirely — plan for the possibility rather than discovering it late.

**100 Hz cap.** Fusion output tops out at 100 Hz versus the 125 Hz pushed now.
The policy runs at 50 Hz, so this is headroom, not a problem — just do not
expect the current rate.

**Magnetic environment.** A magnetometer on a robot surrounded by twelve servos
and their currents will read the robot as much as the Earth. Expect to
calibrate on the assembled machine, and expect heading quality to degrade while
the servos draw hard.

## Rollback

Keep the MPU-6500 path behind a config switch until the gates above pass. The
existing firmware is on `pr/hardware-bringup`; the diagnostics added during the
magnetometer investigation (`imu_diag` string with `whoami`, `mag_wia`,
`aux_n`, per-pass sweep health, and the config-register snapshot) work for any
part and should be kept — they are what turns "the sensor seems wrong" into a
specific answer.
