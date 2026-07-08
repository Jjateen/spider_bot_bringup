# Big Bertha Bringup

ROS 2 **Jazzy** bringup for the **Big Bertha** quadruped: a PPO locomotion policy
(exported to ONNX, run from a C++ node) drives a learned gait, while SLAM builds a
map and Nav2 plans collision-free point-A-to-point-B paths — demonstrated in a
Gazebo Harmonic world full of obstacles.

> Simulation bringup first; hardware bringup is scaffolded but empty.
> Everything is built **for Big Bertha** for now (URDF, meshes, weights).

The full architecture, module decomposition, diagrams, CI, and execution plan
are maintained as a local design doc (not tracked in the repo).

---

## Packages

| Package | Type | Role |
|---|---|---|
| [`spider_msgs`](./spider_msgs) | `ament_cmake` | Robot-agnostic interfaces (shared with the future Lil Spider) |
| [`big_bertha_description`](./big_bertha_description) | `ament_cmake` | URDF/xacro, meshes, `ros2_control` |
| [`big_bertha_policy_controller`](./big_bertha_policy_controller) | `ament_cmake` (C++) | ONNX gait node: `/cmd_vel` → 12 joint targets |
| [`big_bertha_sim_bringup`](./big_bertha_sim_bringup) | `ament_cmake` | Gazebo sim: world, SLAM, Nav2, RViz |
| [`big_bertha_bringup`](./big_bertha_bringup) | `ament_cmake` | Hardware bringup (empty stub — BOM in its README) |

## Quick start (simulation)

```bash
# 1. Install the stack (once)
bash scripts/install_jazzy.sh

# 2. Build
source /opt/ros/jazzy/setup.bash
colcon build && source install/setup.bash

# 3. One-command A->B demo (full stack + RViz + auto-sent Nav2 goal;
#    known-map mode with ground-truth localization by default)
ros2 launch big_bertha_sim_bringup demo.launch.py

# 4-goal perimeter patrol instead of the single goal
ros2 launch big_bertha_sim_bringup demo.launch.py patrol:=true rviz_config:=patrol

# Stack only, no auto goal (add slam:=true to map live instead of known-map)
ros2 launch big_bertha_sim_bringup bringup.launch.py rviz:=true
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
1× YDLidar X2 · 1× MPU9250 IMU. (Hardware bringup is future work; the arm64 CI leg
exists because the deploy target is arm64.)

### Physical Params
| Joint | lower limit | upper_limit | servo center| policy map | channel | direcetion |
|-----|-----|-----|-----|-----|-----|-----|
| arm_a_2_1 | 180 | 50 | 90 | 0 | 2 | -1 |
| arm_b_2_1 | 50 | 180 | 100| 0 | 1 | -1 |
| arm_c_2_1 | 150 | 0 | 92 | 1.57 | 0 | -1 |
| arm_a_1_1 | 30 | 150 | 90| 0 | 10 | +1 |
| arm_b_1_1 | 140 | 0 | 90 | 0 | 9 | -1 |
| arm_c_1_1 | 180 | 40 | 98| 1.57 | 8 | -1 |
| arm_a_3_1 | 140 | 0 | 90 | 0 | 6 | -1 |
| arm_b_3_1 | 50 | 180 | 100| 0 | 5 | -1 |
| arm_c_3_1 | 0 | 150 | 95 | 1.57 | 4 | -1 |
| arm_a_4_1 | 45 | 180 | 90 | 0 | 14 | +1 |
| arm_b_4_1 | 135 | 0 |  90 | 0 | 13 | -1 |
| arm_c_4_1 | 40 | 180 | 90 | 1.57 | 12 | -1 |


## License

Apache-2.0 — see [LICENSE](./LICENSE).
