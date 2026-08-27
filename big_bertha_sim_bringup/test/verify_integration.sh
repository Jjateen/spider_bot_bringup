#!/usr/bin/env bash
# Headless acceptance gate for the integration module (issue #10).
#
# Launches the one-shot bringup.launch.py (description -> simulation ->
# locomotion -> state_estimation -> {mapping | localization} -> planning) fully
# headless and asserts that the whole stack comes up without crashes: every
# expected node is present in `ros2 node list`, the Nav2 lifecycle is active,
# and the odom->base_link transform exists. Runs in known-map mode by default
# (SLAM:=false); set SLAM=true to gate the SLAM variant. Evidence ->
# verification_artifacts/integration/.
#
# Usage: test/verify_integration.sh            # known-map mode
#        SLAM=true test/verify_integration.sh  # SLAM mode
echo "[verify] sourcing ROS 2 + workspace"
# Source ROS first (its setup scripts trip 'set -u'), then enable strict mode.
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -uo pipefail

ART="${ART:-verification_artifacts/integration}"
WARMUP="${WARMUP:-55}"
SLAM="${SLAM:-false}"
mkdir -p "${ART}"
: > "${ART}/gate.txt"
PASS=0

# The sim worlds + installed models must be resolvable by gz.
export GZ_SIM_RESOURCE_PATH="$(pwd)/big_bertha_sim_bringup/worlds:$(pwd)/install:${GZ_SIM_RESOURCE_PATH:-}"

cleanup() {
  echo "[verify] cleaning up"
  kill "${LAUNCH_PID}" 2>/dev/null
  pkill -f "gz sim" 2>/dev/null
  pkill -f "ruby.*gz" 2>/dev/null
  pkill -f parameter_bridge 2>/dev/null
  pkill -f robot_state_publisher 2>/dev/null
  pkill -f "nav2|lifecycle_manager|amcl|map_server|slam_toolbox" 2>/dev/null
  pkill -f "controller_server|planner_server|smoother_server" 2>/dev/null
  pkill -f "behavior_server|bt_navigator|ekf_node|policy_controller" 2>/dev/null
  sleep 2
}
trap cleanup EXIT

echo "[verify] launching full headless bringup (slam:=${SLAM})"
ros2 launch big_bertha_sim_bringup bringup.launch.py \
  slam:="${SLAM}" rviz:=false use_sim_time:=true \
  > "${ART}/bringup.log" 2>&1 &
LAUNCH_PID=$!

echo "[verify] warming up (${WARMUP}s for lifecycle activation)"
sleep "${WARMUP}"

# Refresh the discovery daemon so a single node-list query is complete.
ros2 daemon stop >/dev/null 2>&1; sleep 2; ros2 daemon start >/dev/null 2>&1
sleep 3

echo "[verify] capturing node list"
ros2 node list 2>/dev/null | sort -u > "${ART}/node_list.txt"
NODES=$(cat "${ART}/node_list.txt")
echo "${NODES}"

# Modules expected in every mode (simulation + locomotion + state_estimation +
# planning). The map->odom source is checked separately per mode below.
EXPECTED=(
  controller_manager gz_ros_control joint_state_broadcaster position_controller
  robot_state_publisher ros_gz_bridge policy_controller ekf_filter_node
  controller_server planner_server smoother_server behavior_server bt_navigator
  lifecycle_manager_navigation global_costmap local_costmap
)
if [ "${SLAM}" = "true" ]; then
  EXPECTED+=(slam_toolbox lifecycle_manager_slam)
else
  # bringup.launch.py's known-map default is localization:=ground_truth (a
  # static map->odom identity, see localization.launch.py), not amcl.
  EXPECTED+=(map_to_odom_ground_truth map_server lifecycle_manager_localization)
fi

echo "[verify] asserting expected nodes are present"
for n in "${EXPECTED[@]}"; do
  if echo "${NODES}" | grep -q "${n}"; then
    echo "[PASS] node ${n}" | tee -a "${ART}/gate.txt"
  else
    echo "[FAIL] missing node ${n}" | tee -a "${ART}/gate.txt"; PASS=1
  fi
done

echo "[verify] no node crashed during bringup"
# Count real deaths only. The one-shot ros2_control spawners legitimately exit
# (and sometimes exit 1 on a duplicate-load race) -> those are excluded. Count
# explicitly rather than relying on a grep pipeline's exit code (this host's
# grep returns 0 on empty input, which would mask a clean run).
DEATHS=$(grep -E 'process has died' "${ART}/bringup.log" 2>/dev/null | wc -l)
REAL_DEATHS=$(grep -E 'process has died' "${ART}/bringup.log" 2>/dev/null \
  | grep -vE 'joint_state_broadcaster_spawner|position_controller_spawner' \
  | wc -l)
if [ "${REAL_DEATHS}" -gt 0 ]; then
  echo "[FAIL] a node process died (${REAL_DEATHS}, see bringup.log)" \
    | tee -a "${ART}/gate.txt"
  grep -E 'process has died' "${ART}/bringup.log" \
    | grep -vE 'spawner' | tail -3 | tee -a "${ART}/gate.txt"
  PASS=1
else
  echo "[PASS] no node crash (${DEATHS} benign spawner exit(s))" \
    | tee -a "${ART}/gate.txt"
fi

echo "[verify] odom->base_link transform present (EKF owns it)"
TF=$(timeout 6 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | head -10)
if echo "${TF}" | grep -qE 'Translation'; then
  echo "[PASS] odom->base_link tf available" | tee -a "${ART}/gate.txt"
else
  echo "[FAIL] odom->base_link tf missing" | tee -a "${ART}/gate.txt"; PASS=1
fi

echo "[verify] Nav2 lifecycle active"
ACT=$(timeout 8 ros2 service call /lifecycle_manager_navigation/is_active \
        std_srvs/srv/Trigger 2>/dev/null)
if echo "${ACT}" | grep -q 'success=True'; then
  echo "[PASS] Nav2 lifecycle is active" | tee -a "${ART}/gate.txt"
else
  echo "[FAIL] Nav2 lifecycle not active" | tee -a "${ART}/gate.txt"; PASS=1
fi

echo "================ GATE ================"; cat "${ART}/gate.txt"
[ "${PASS}" -eq 0 ] && echo "[verify] INTEGRATION GATE: PASS" \
  || echo "[verify] INTEGRATION GATE: FAIL"
exit "${PASS}"
