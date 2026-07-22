#!/usr/bin/env bash
# Tear down a Big Bertha sim stack completely.
#
# `ros2 launch` spawns a web of child processes (gz sim, ros_gz parameter_bridge,
# robot_state_publisher, tf2 static_transform_publisher, nav2 lifecycle/map
# nodes, controller_manager, rviz2). Ctrl-C on the launch does NOT always reap
# all of them -- in particular the ros_gz parameter_bridge tends to survive, and
# each survivor keeps republishing /clock. Several stale bridges make /clock jump
# backwards, which makes RViz clear its TF buffer every frame (links freeze, the
# RobotModel flashes red). Run this between launches so exactly one stack is ever
# live. Kills by process name (grep is broader than pkill -f here) then restarts
# the ROS 2 daemon to flush stale node registrations from the discovery cache.
#
# Usage: kill_sim.sh
set -uo pipefail

PATTERNS='parameter_bridge|ros_gz|gz sim|ruby.*gz|rviz2|policy_controller_node'
PATTERNS+='|robot_state_publisher|static_transform_publisher|nav2_|map_server'
PATTERNS+='|lifecycle_manager|controller_manager|ros2 launch|scan_ground_filter'

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
