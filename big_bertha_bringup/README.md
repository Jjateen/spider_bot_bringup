# big_bertha_bringup (hardware) — TODO stub

Hardware bringup for the **physical** Big Bertha quadruped. This package is an
intentional empty stub: `launch/` and `config/` are placeholders (each holds a
`.gitkeep`) until the real robot is wired up. It mirrors the sim/hardware split
of `big_bertha_sim_bringup`, exactly as `mpauv_bringup` does.

The autonomy stack is hardware-agnostic: `state_estimation`, `mapping`,
`localization`, and `planning` reuse the **same** configs as the sim, just with
`use_sim_time:=false`. Only the bottom layer changes — real sensor drivers and a
servo bridge replace the Gazebo + `ros_gz_bridge` simulation layer.

## Bill of materials (BOM)

From [PLAN.md §11](../PLAN.md). All runtime nodes are C++.

| Subsystem | Part | Qty | ROS interface (future hw bringup, all C++) |
|---|---|---|---|
| Compute | **Arduino UNO Q (4 GB)** running **ROS 2 Jazzy** | 1 | hosts the whole stack — **arm64**, hence the arm64 CI leg |
| Structure | **3D-printed** frame + leg linkages | — | — |
| Actuators | **MG995 servos** (4 legs × 3 joints) | 12 | C++ serial/PWM bridge: policy degrees → servo commands (ports the legacy `serial_bridge`) |
| Lidar | **YDLidar X2** (2D) | 1 | `ydlidar_ros2_driver` → `/scan` |
| IMU | **MPU6050** | 1 | C++ IMU driver node → `/imu` (matches the ±2g / LSB scaling in the legacy node) |

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

## TODO (when the robot is built)

- [ ] `launch/hardware.launch.py` — sensor drivers + servo bridge + the
      hardware-agnostic autonomy stack (`use_sim_time:=false`).
- [ ] `config/` — servo channel map, MPU6050 calibration, YDLidar X2 params.
- [ ] C++ MG995 servo bridge node (`/position_controller/commands` → PWM).
- [ ] C++ MPU6050 IMU driver node (`/imu`).
- [ ] YDLidar X2 driver wiring (`/scan`).
