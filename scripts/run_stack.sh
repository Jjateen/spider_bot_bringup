#!/usr/bin/env bash
# Bring up the whole Big Bertha stack on the board, with the DDS profile set.
#
#   ./scripts/run_stack.sh                          # everything
#   ./scripts/run_stack.sh with_lidar:=false        # servo/bench work
#   ./scripts/run_stack.sh with_nav:=false start_enabled:=false
#   ./scripts/run_stack.sh slam:=false map:=/path/to/map.yaml
#
# The DDS exports are the point of this script. Without CYCLONEDDS_URI the
# profile is not loaded, discovery falls back to multicast, and multicast
# reaches nothing here: wlan0 has AP client isolation and Tailscale carries no
# multicast at all. Topics then work on the board and are invisible from a dev
# machine, which looks like a broken stack rather than a missing env var.
set -euo pipefail

WS="${BB_WS:-$HOME/ros2_ws}"

if [ ! -f "$WS/install/setup.bash" ]; then
  echo "no ROS workspace at $WS (override with BB_WS=/path)" >&2
  exit 1
fi

# -u has to come off around this: colcon's generated setup.bash reads
# COLCON_TRACE without a default, so sourcing it under `set -u` aborts with
# "COLCON_TRACE: unbound variable" before a single node starts.
set +u
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$(ros2 pkg prefix big_bertha_bringup)/share/big_bertha_bringup/config/dds/cyclonedds.xml"

echo "RMW:   $RMW_IMPLEMENTATION"
echo "DDS:   $CYCLONEDDS_URI"

# The lidar is on USB and the servos brown that rail out (see DEPLOYMENT.md
# Known Limitations 6). Say so up front rather than letting the driver fail
# with "Device is not open" ten seconds in.
if [ ! -e /dev/ttyUSB0 ] && [[ " $* " != *"with_lidar:=false"* ]]; then
  echo "WARNING: /dev/ttyUSB0 is missing, so the lidar will not start." >&2
  echo "         Check its power before debugging anything above it," >&2
  echo "         or pass with_lidar:=false." >&2
fi

exec ros2 launch big_bertha_bringup composed_stack.launch.py "$@"
