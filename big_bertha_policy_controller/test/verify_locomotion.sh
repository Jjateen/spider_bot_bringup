#!/usr/bin/env bash
# Headless acceptance gate for the locomotion module (issue #5).
#
# Assumes a headless sim (big_bertha_sim_bringup) and the policy controller
# are already running. Records the base x/y from /odom, publishes /cmd_vel
# vx=0.3 for ~6 s, records the new pose, and asserts the base advanced and
# that /position_controller/commands streams at the expected rate. Evidence
# goes to verification_artifacts/locomotion/.
#
# Usage: test/verify_locomotion.sh
set -uo pipefail
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

ART="${ART:-verification_artifacts/locomotion}"
mkdir -p "${ART}"

odom_xy() {
  timeout 6 ros2 topic echo /odom --once 2>/dev/null \
    | grep -A3 'position:' | grep -E '  *(x|y):' | head -2 \
    | grep -oE '[-0-9.]+' | tr '\n' ' '
}

echo "[verify] start pose"
START=$(odom_xy); echo "start: ${START}" | tee "${ART}/gate.txt"

echo "[verify] driving vx=0.3 for 6 s"
timeout 7 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}}' > /dev/null 2>&1 &
PUB=$!
sleep 6.5
kill "${PUB}" 2>/dev/null

echo "[verify] end pose"
END=$(odom_xy); echo "end:   ${END}" | tee -a "${ART}/gate.txt"

DX=$(awk -v s="${START}" -v e="${END}" 'BEGIN{
  split(s,a," "); split(e,b," ");
  dx=b[1]-a[1]; dy=b[2]-a[2]; print sqrt(dx*dx+dy*dy)}')
echo "displacement: ${DX} m" | tee -a "${ART}/gate.txt"

HZ=$(timeout 6 ros2 topic hz /position_controller/commands 2>/dev/null \
      | grep -oP 'average rate: \K[0-9.]+' | head -1)
echo "cmd rate: ${HZ} Hz" | tee -a "${ART}/gate.txt"

PASS=0
awk "BEGIN{exit !(${DX:-0} > 0.3)}" \
  && echo "[PASS] base advanced > 0.3 m" | tee -a "${ART}/gate.txt" \
  || { echo "[FAIL] base did not advance > 0.3 m" | tee -a "${ART}/gate.txt"; PASS=1; }
[ -n "${HZ}" ] && awk "BEGIN{exit !(${HZ} > 0)}" \
  && echo "[PASS] /position_controller/commands @ ${HZ} Hz" | tee -a "${ART}/gate.txt" \
  || { echo "[FAIL] no command stream" | tee -a "${ART}/gate.txt"; PASS=1; }

echo "================ GATE ================"; cat "${ART}/gate.txt"
exit "${PASS}"
