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

# Optional unicast peer, for a network where multicast discovery does not
# cross to the dev machine. Only set this while that host is actually up: an
# unreachable peer makes Cyclone retain samples for a reader that will never
# read them, and once writer history fills the publish call blocks, which
# silently kills /scan a few minutes in.
#   BB_DDS_PEER=10.42.0.1 ./scripts/run_stack.sh
if [ -n "${BB_DDS_PEER:-}" ]; then
  _base="${CYCLONEDDS_URI#file://}"
  _tmp="$(mktemp /tmp/cyclonedds_peer_XXXX.xml)"
  sed "s|</Discovery>|  <Peers><Peer address=\"${BB_DDS_PEER}\"/></Peers>\n    </Discovery>|" \
    "$_base" > "$_tmp"
  export CYCLONEDDS_URI="file://$_tmp"
  echo "PEER:  ${BB_DDS_PEER} (unicast; unset BB_DDS_PEER when that host goes away)"
fi

# Clear any survivors from a previous run before starting. run_stack uses
# exec + ros2 launch, so Ctrl+C goes straight to launch and shuts its nodes
# down cleanly -- but the lidar driver does BLOCKING serial reads, so if it is
# wedged when SIGINT arrives it can outlive the shutdown and keep /dev/ttyLIDAR
# open. The next run then fails with "port busy". This is not the 5-minute USB
# drop (that is the hub power-cycling in dmesg, which no process can cause), it
# is only the re-run conflict. Sweeping here makes a stale driver harmless.
_stale=$(pgrep -f "ydlidar_ros2_driver_node|component_container" || true)
if [ -n "$_stale" ]; then
  echo "CLEANUP: killing leftover stack processes from a previous run"
  # shellcheck disable=SC2086
  kill $_stale 2>/dev/null || true
  sleep 2
  _stale=$(pgrep -f "ydlidar_ros2_driver_node|component_container" || true)
  # shellcheck disable=SC2086
  [ -n "$_stale" ] && kill -9 $_stale 2>/dev/null || true
  sleep 1
fi

echo "RMW:   $RMW_IMPLEMENTATION"
echo "DDS:   $CYCLONEDDS_URI"

# The MCU sketch does not survive a reboot: arduino-app-cli leaves
# hardware_bridge_app in "uninitialized" and nothing starts it. Every topic
# downstream is then silent while the ROS side looks perfectly healthy, all
# eleven components loaded and /imu, /filtered/imu and /odom publishing
# nothing. Cheaper to start it here than to rediscover that every reboot.
if command -v arduino-app-cli >/dev/null 2>&1; then
  if ! arduino-app-cli app list 2>/dev/null | grep -q "hardware_bridge_app.*running"; then
    echo "FIRMWARE: hardware_bridge_app is not running, starting it"
    arduino-app-cli app start user:hardware_bridge_app 2>&1 | tail -1
    sleep 8
  else
    echo "FIRMWARE: hardware_bridge_app already running"
  fi
fi

# The lidar is on USB and the servos brown that rail out (see DEPLOYMENT.md
# Known Limitations 6). Say so up front rather than letting the driver fail
# with "Device is not open" ten seconds in.
if [ ! -e /dev/ttyUSB0 ] && [[ " $* " != *"with_lidar:=false"* ]]; then
  echo "WARNING: /dev/ttyUSB0 is missing, so the lidar will not start." >&2
  echo "         Check its power before debugging anything above it," >&2
  echo "         or pass with_lidar:=false." >&2
fi

exec ros2 launch big_bertha_bringup composed_stack.launch.py "$@"
