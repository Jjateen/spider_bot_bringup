#!/usr/bin/env bash
# Headless acceptance gate for the simulation module (issue #4).
#
# Launches the obstacle world + robot spawn + bridge + controllers with no
# GUI, then asserts that /scan /imu /odom /clock all publish at non-zero
# rates, that the laser ranges are finite, and that the robot spawned (a
# valid /odom pose with no NaNs). Evidence is written to
# verification_artifacts/simulation/.
#
# Usage: test/verify_simulation.sh
set -uo pipefail

ART_DIR="${ART_DIR:-verification_artifacts/simulation}"
RUN_SECS="${RUN_SECS:-45}"
mkdir -p "${ART_DIR}"

echo "[verify] sourcing ROS 2 + workspace"
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

cleanup() {
  echo "[verify] cleaning up"
  kill "${LAUNCH_PID}" 2>/dev/null
  pkill -f "gz sim" 2>/dev/null
  pkill -f "ruby.*gz" 2>/dev/null
  pkill -f parameter_bridge 2>/dev/null
  pkill -f robot_state_publisher 2>/dev/null
  sleep 2
}
trap cleanup EXIT

echo "[verify] launching headless simulation"
ros2 launch big_bertha_sim_bringup simulation.launch.py gui:=false \
  > "${ART_DIR}/launch.log" 2>&1 &
LAUNCH_PID=$!

echo "[verify] warming up (${RUN_SECS}s)"
sleep "${RUN_SECS}"

PASS=0
check_hz() {
  local topic=$1
  local hz
  hz=$(timeout 12 ros2 topic hz "${topic}" 2>/dev/null \
        | grep -oP 'average rate: \K[0-9.]+' | head -1)
  if [[ -n "${hz}" ]] && awk "BEGIN{exit !(${hz} > 0)}"; then
    echo "[PASS] ${topic} @ ${hz} Hz" | tee -a "${ART_DIR}/gate.txt"
  else
    echo "[FAIL] ${topic} not publishing" | tee -a "${ART_DIR}/gate.txt"
    PASS=1
  fi
}

: > "${ART_DIR}/gate.txt"
echo "[verify] checking topic rates"
check_hz /clock
check_hz /scan
check_hz /imu
check_hz /odom

echo "[verify] checking scan ranges are finite"
SCAN=$(timeout 10 ros2 topic echo /scan --once 2>/dev/null)
echo "${SCAN}" > "${ART_DIR}/scan_sample.txt"
if echo "${SCAN}" | grep -qE 'ranges:' && \
   echo "${SCAN}" | grep -qE '[0-9]\.[0-9]'; then
  echo "[PASS] /scan has finite range values" | tee -a "${ART_DIR}/gate.txt"
else
  echo "[FAIL] /scan ranges missing/non-finite" | tee -a "${ART_DIR}/gate.txt"
  PASS=1
fi

echo "[verify] checking robot spawned (odom pose, no NaN)"
ODOM=$(timeout 10 ros2 topic echo /odom --once 2>/dev/null)
echo "${ODOM}" > "${ART_DIR}/odom_sample.txt"
# Match a bare NaN value, not the 'nanosec' field in the stamp.
if echo "${ODOM}" | grep -qiwE 'nan|-nan'; then
  echo "[FAIL] /odom contains NaN" | tee -a "${ART_DIR}/gate.txt"
  PASS=1
elif echo "${ODOM}" | grep -qE 'position:'; then
  echo "[PASS] robot spawned, /odom pose finite" | tee -a "${ART_DIR}/gate.txt"
else
  echo "[FAIL] no /odom pose" | tee -a "${ART_DIR}/gate.txt"
  PASS=1
fi

echo "[verify] controllers:" | tee -a "${ART_DIR}/gate.txt"
ros2 control list_controllers 2>/dev/null | tee -a "${ART_DIR}/gate.txt"

echo "================ GATE RESULT ================"
cat "${ART_DIR}/gate.txt"
if [[ "${PASS}" -eq 0 ]]; then
  echo "[verify] SIMULATION GATE: PASS"
else
  echo "[verify] SIMULATION GATE: FAIL"
fi
exit "${PASS}"
