# big_bertha_bringup (hardware)

Hardware bringup for the **physical** Big Bertha quadruped. Contains the full
three-layer bridge between ROS 2 and the Arduino UNO Q hardware:

| Layer | File | Role |
|---|---|---|
| **C++ ROS 2 node** | `src/hardware_bridge_node.cpp` | Subscribes to `/position_controller/commands` (12 joint targets), converts radians → PWM, sends JSON over TCP; background thread polls IMU, publishes `sensor_msgs/Imu` on `/imu` |
| **Python TCP relay** | `firmware/hardware_bridge_app/python/main.py` | Runs on the UNO Q Linux side, bridges TCP ↔ Bridge RPC to the STM32U585 |
| **STM32U585 firmware** | `firmware/hardware_bridge_app/sketch/sketch.ino` | I2C driver for PCA9685 (12-ch PWM, 50 Hz) and MPU6050 (6-axis IMU, SI units) |

The autonomy stack is hardware-agnostic: `state_estimation`, `mapping`,
`localization`, and `planning` reuse the **same** configs as the sim, just with
`use_sim_time:=false`. Only the bottom layer changes — real sensor drivers and a
servo bridge replace the Gazebo + `ros_gz_bridge` simulation layer.

## Bill of materials (BOM)

From [PLAN.md §11](../PLAN.md). All runtime nodes are C++.

| Subsystem | Part | Qty | ROS interface |
|---|---|---|---|
| Compute | **Arduino UNO Q (4 GB)** running **ROS 2 Jazzy** | 1 | hosts the whole stack — **arm64**, hence the arm64 CI leg |
| Structure | **3D-printed** frame + leg linkages | — | — |
| Actuators | **MG995 servos** (4 legs × 3 joints) | 12 | `hardware_bridge_node` → TCP → Bridge RPC → PCA9685 → servos |
| Lidar | **YDLidar X2** (2D) | 1 | `ydlidar_ros2_driver` → `/scan` |
| IMU | **MPU6050** | 1 | STM32U585 firmware (I2C) → Bridge RPC → TCP → `hardware_bridge_node` → `/imu` |

## Sim → hardware mapping (per module)

- **simulation** → real sensors: the YDLidar X2 driver + the MPU6050 driver
  replace `ros_gz_bridge`.
- **locomotion** → the **same** `policy.onnx` runs on the UNO Q (arm64 ONNX
  Runtime); `/position_controller/commands` is consumed by the MG995 servo
  bridge instead of `gz_ros2_control`.
- **state_estimation / mapping / localization / planning** are
  **hardware-agnostic** — identical configs from `big_bertha_sim_bringup`, just
  launched with `use_sim_time:=false`.

> arm64 CI is a hard gate, not a nicety: the deploy target is the arm64 UNO Q,
> so a green arm64 build proves the stack runs on the real compute module.

## Servo Calibration

The 12 MG995 servos (4 legs × 3 joints) are driven by a PCA9685 PWM controller over I2C. The `hardware_bridge_node` converts policy joint targets (radians) → servo angles (degrees) → PWM pulses using per-joint calibration parameters from `config/hardware_bridge.yaml`.

### Conversion pipeline

```
policy joint target (rad)
  → servo_angle(°) = (target_rad × 180/π − policy_center_rad × 180/π) × direction + offset + 90
  → PWM = round(angle/180 × (pwm_max − pwm_min) + pwm_min)
  → PCA9685 12-bit duty cycle (0–4095)
```

### Center mapping

Each servo has a **servo center** — the physically measured angle (in degrees) where the servo horn is centered. This maps to the **policy center** — the radian value the policy outputs when the servo should be at its center.

| Quantity | Relationship |
|---|---|
| Servo center (°) | `offset + 90` |
| Policy center (rad) | `policy_center` from `config/hardware_bridge.yaml` |
| At `target = policy_center` | `servo_angle = offset + 90 = servo_center` |
| Offset (°) | `offset = servo_center − 90` — compensates for mounting imperfections |

### Direction convention

`servo_direction` encodes which way the servo moves relative to the policy command:

| direction | policy rad ↓ | policy rad ↑ |
|---|---|---|
| **−1** | Servo moves toward **upper** physical limit | Servo moves toward **lower** physical limit |
| **+1** | Servo moves toward **lower** physical limit | Servo moves toward **upper** physical limit |

The physical range used for clamping is always `[min(lower, upper), max(lower, upper)]`.

### Calibration table

Values are listed in **policy output order** (Isaac articulation: all hips → all knees → all ankles).

| # | Revolute | arm name | range min–max (°) | servo center (°) | policy center (rad) | offset (°) | direction | PCA9685 ch |
|---|---|---|---|---|---|---|---|---|
| 0 | 110 | arm_a_4_1 | 45–180 | 90 | 0.00 | 0 | +1 | 14 |
| 1 | 113 | arm_a_1_1 | 30–150 | 90 | 0.00 | 0 | +1 | 10 |
| 2 | 116 | arm_a_2_1 | 50–180 | 90 | 0.00 | 0 | +1 | 2 |
| 3 | 119 | arm_a_3_1 | 0–140 | 90 | 0.00 | 0 | +1 | 6 |
| 4 | 111 | arm_b_4_1 | 0–135 | 90 | 0.00 | 0 | +1 | 13 |
| 5 | 114 | arm_b_1_1 | 0–140 | 90 | 0.00 | 0 | +1 | 9 |
| 6 | 117 | arm_b_2_1 | 50–180 | 100 | 0.00 | 10 | −1 | 1 |
| 7 | 120 | arm_b_3_1 | 50–180 | 100 | 0.00 | 10 | −1 | 5 |
| 8 | 112 | arm_c_4_1 | 40–180 | 90 | 1.57 | 0 | −1 | 12 |
| 9 | 115 | arm_c_1_1 | 40–180 | 98 | 1.57 | 8 | −1 | 8 |
| 10 | 118 | arm_c_2_1 | 0–150 | 92 | 1.57 | 2 | +1 | 0 |
| 11 | 121 | arm_c_3_1 | 0–150 | 95 | 1.57 | 5 | +1 | 4 |

### Verification

Use `scripts/servo_diag.py` to test PCA9685 PWM output with readback verification. The single-source-of-truth for all calibration values is `config/hardware_bridge.yaml`.

## Firmware structure

```
firmware/hardware_bridge_app/
├── app.yaml                    # Arduino app metadata (bricks: python + sketch)
├── python/
│   ├── app.yaml                # Python brick config
│   ├── app.py                  # Entry point (delegates to main.py)
│   ├── main.py                 # TCP relay implementation
│   └── requirements.txt        # Python deps (stdlib only)
├── sketch/
│   ├── sketch.ino              # STM32U585 firmware (PCA9685 + MPU6050)
│   └── sketch.yaml             # Platform config (arduino:zephyr:unoq)
└── README.md                   # Upload & run instructions
```

## Quick start

```bash
# 1. Deploy firmware app to UNO Q
scp -r firmware/hardware_bridge_app user@<uno-q-ip>:~/ArduinoApps/

# 2. On the UNO Q
arduino-app-cli app start ~/ArduinoApps/hardware_bridge_app

# 3. On the UNO Q (same machine), launch the ROS 2 node
ros2 launch big_bertha_bringup hardware_bringup.launch.py
```
