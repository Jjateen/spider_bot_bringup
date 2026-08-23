#!/usr/bin/env bash
# Localization + waypoint demo, board side.
#
#   ./scripts/run_localization.sh                      # default checked-in map
#   ./scripts/run_localization.sh map:=/path/to/map.yaml x:=0 y:=0 yaw:=0
#
# This reuses run_stack.sh's board-side setup exactly (Cyclone DDS env, MCU
# sketch autostart, stale-process sweep) and only swaps the launch file, so the
# SLAM demo (run_stack.sh with no override) is completely untouched.
#
# RViz runs separately on the dev laptop: ros2 launch big_bertha_bringup
# nav_rviz.launch.py. See DEPLOYMENT.md, "Waypoint navigation demo".
set -euo pipefail
export BB_LAUNCH_FILE=localization_demo.launch.py
exec "$(dirname "$0")/run_stack.sh" "$@"
