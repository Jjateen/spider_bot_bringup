# Big Bertha Bringup

ROS 2 **Jazzy** bringup for the **Big Bertha** quadruped: a PPO locomotion policy
(exported to ONNX, run from a C++ node) drives a learned gait, while SLAM builds a
map and Nav2 plans collision-free point-A-to-point-B paths — demonstrated in a
Gazebo Harmonic world full of obstacles.

> Simulation bringup first; hardware bringup is scaffolded but empty.
> Everything is built **for Big Bertha** for now (URDF, meshes, weights).

See **[PLAN.md](./PLAN.md)** for the full architecture, module decomposition,
diagrams, CI, and execution plan.

---

## Packages

| Package | Type | Role |
|---|---|---|
| [`spider_msgs`](./spider_msgs) | `ament_cmake` | Robot-agnostic interfaces (shared with the future Lil Spider) |
| [`big_bertha_description`](./big_bertha_description) | `ament_cmake` | URDF/xacro, meshes, `ros2_control` |
| [`big_bertha_policy_controller`](./big_bertha_policy_controller) | `ament_cmake` (C++) | ONNX gait node: `/cmd_vel` → 12 joint targets |
| [`big_bertha_sim_bringup`](./big_bertha_sim_bringup) | `ament_cmake` | Gazebo sim: world, SLAM, Nav2, RViz |
| [`big_bertha_bringup`](./big_bertha_bringup) | `ament_cmake` | Hardware bringup (empty stub — see BOM in PLAN.md) |

## Quick start (simulation)

```bash
# 1. Install the stack (once)
bash scripts/install_jazzy.sh

# 2. Build
source /opt/ros/jazzy/setup.bash
colcon build && source install/setup.bash

# 3. Bring up the full sim (obstacle world + gait + state est + Nav2)
#    known-map mode (AMCL) by default; add rviz:=true for the live view
ros2 launch big_bertha_sim_bringup bringup.launch.py rviz:=true

# SLAM mode (build the map live instead of loading the saved one)
ros2 launch big_bertha_sim_bringup bringup.launch.py slam:=true rviz:=true
```

`bringup.launch.py` chains every module and takes `slam:=` (SLAM vs known-map),
`rviz:=` / `rviz_config:=` (simulation|mapping|planning|integration),
`use_sim_time:=`, `gui:=`, `world:=`, `sim_drive:=`, `map:=`, and the spawn pose.

## Autonomy stack (functional modules)

```
description → simulation → locomotion → state_estimation → mapping → localization → planning
```

## Visualization

```bash
# RViz with a specific view (simulation | mapping | planning | integration)
ros2 launch big_bertha_sim_bringup rviz.launch.py config:=planning

# PlotJuggler layout (sensors: /imu /joint_states /odom;
#                     control: /cmd_vel vs /odom + spider_msgs/PolicyStatus)
ros2 launch big_bertha_sim_bringup plotjuggler.launch.py layout:=control
```

Configs live under `big_bertha_sim_bringup/config/{rviz,plotjuggler}/`.

## Hardware target

Arduino UNO Q (4 GB, ROS 2 Jazzy, **arm64**) · 3D-printed frame · 12× MG995 servos ·
1× YDLidar X2 · 1× MPU6050 IMU. (Hardware bringup is future work; the arm64 CI leg
exists because the deploy target is arm64.)

## License

Apache-2.0 — see [LICENSE](./LICENSE).
