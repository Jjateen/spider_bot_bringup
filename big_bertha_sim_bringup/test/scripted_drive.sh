#!/usr/bin/env bash
# Scripted patrol drive for the mapping / planning verification.
#
# Publishes a sequence of /cmd_vel twists (forward + turns) so the sim robot
# (driven by the gz VelocityControl system, sim_drive:=true) traverses the
# obstacle_world arena and the lidar sweeps enough geometry for slam_toolbox
# to build a closed map. Pure ROS 2 CLI; no runtime node.
#
# Usage: test/scripted_drive.sh [duration_per_leg_s]
# Source ROS first (its setup scripts trip 'set -u'), then enable strict mode.
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -uo pipefail

LEG="${1:-3}"

pub() {  # pub <vx> <wz> <seconds>
  timeout "$3" ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: $1}, angular: {z: $2}}" > /dev/null 2>&1
}

echo "[drive] patrol: a rectangle around the arena with turns"
pub 0.4 0.0  "${LEG}"     # forward (NE from A)
pub 0.0 0.8  2            # turn left
pub 0.4 0.0  "${LEG}"
pub 0.0 0.8  2
pub 0.4 0.0  "${LEG}"
pub 0.0 0.8  2
pub 0.4 0.0  "${LEG}"
pub 0.0 -0.8 2            # turn right (sweep more)
pub 0.4 0.0  "${LEG}"
pub 0.0 0.0  1            # stop
echo "[drive] done"
