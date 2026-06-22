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
