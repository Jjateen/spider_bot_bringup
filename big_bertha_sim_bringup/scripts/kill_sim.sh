#!/usr/bin/env bash
# Tear down a Big Bertha sim stack completely. Ctrl-C does not always reap
# every child; a surviving parameter_bridge republishes /clock, which jumps
# time backwards and makes RViz flush TF every frame. Run between launches.
#
# Usage: kill_sim.sh
set -uo pipefail

PATTERNS='parameter_bridge|ros_gz|gz sim|ruby.*gz|rviz2|policy_controller_node'
PATTERNS+='|robot_state_publisher|static_transform_publisher|nav2_|map_server'
PATTERNS+='|lifecycle_manager|controller_manager|ros2 launch [a-z_]*big_bertha|scan_ground_filter'
# ekf_node was missing here: a surviving EKF wakes on the NEXT run's /odom and
# publishes odom->base_link with old-epoch stamps, poisoning every TF cache
# (goals rejected, "message earlier than all data in cache"). Same class of
# survivor: slam_toolbox, amcl, a mid-tour send_patrol loop.
PATTERNS+='|ekf_node|slam_toolbox|amcl|send_patrol'

pids="$(ps ax -o pid=,command= | grep -E "${PATTERNS}" | grep -v grep \
        | awk '{print $1}')"

if [ -n "${pids}" ]; then
  echo "[kill_sim] killing: ${pids//$'\n'/ }"
  # shellcheck disable=SC2086
  kill -9 ${pids} 2>/dev/null || true
  sleep 2
else
  echo "[kill_sim] nothing to kill"
fi

# Flush the discovery cache so `ros2 node list` doesn't show ghosts.
ros2 daemon stop  >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

echo "[kill_sim] done"
