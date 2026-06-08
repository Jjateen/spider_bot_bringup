# big_bertha_sim_bringup

Gazebo Harmonic (gz-sim 8) simulation bringup for the Big Bertha quadruped on
ROS 2 Jazzy. Holds the obstacle world, robot spawn, `ros_gz_bridge`,
ros2_control spawners, and the per-module launch/config (simulation,
state_estimation, mapping, localization, planning, and the full bringup).

## Obstacle world

`worlds/obstacle_world.sdf` is a ~10 m x 10 m walled arena with a
friction-tuned ground plane (mu = 1.0), 4 perimeter walls (loop-closure
geometry for SLAM), and interior obstacles (3 boxes + 2 cylindrical pillars)
positioned so the straight diagonal from A to B is blocked.

Demo coordinates:

- A (spawn) = `(-3.5, -3.5, 0.12)`, yaw `0.785` rad (facing B)
- B (goal)  = `( 3.5,  3.5)`

## Run

Headless (server only):

    ros2 launch big_bertha_sim_bringup simulation.launch.py gui:=false

With the Gazebo GUI client:

    ros2 launch big_bertha_sim_bringup simulation.launch.py gui:=true

Bridged ROS 2 topics: `/clock`, `/scan`, `/imu`, `/odom`, `/tf`, plus
`/joint_states` and `/position_controller/commands` from ros2_control.

## Verify (acceptance gate, issue #4)

    test/verify_simulation.sh

Asserts non-zero rates on `/scan /imu /odom /clock`, finite laser ranges, and
a NaN-free spawn pose. Evidence is written to
`verification_artifacts/simulation/`.
