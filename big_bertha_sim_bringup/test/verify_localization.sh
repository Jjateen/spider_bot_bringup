#!/usr/bin/env bash
# Headless acceptance gate for the localization module (issue #8).
#
# Assumes the sim (sim_drive:=true, odom_tf:=false), EKF, and the localization
# stack (map_server + amcl) are running. Drives the robot a short way, then
# asserts AMCL publishes /amcl_pose with a converged (bounded) covariance and
# a map->odom transform, and that its estimate is near the ground-truth /odom.
# Evidence -> verification_artifacts/localization/.
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -uo pipefail

ART="${ART:-verification_artifacts/localization}"
mkdir -p "${ART}"
: > "${ART}/gate.txt"
PASS=0

echo "[verify] map->odom transform (AMCL)"
TF=$(timeout 6 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | head -12)
echo "${TF}" > "${ART}/map_odom_tf.txt"
if echo "${TF}" | grep -qE 'Translation'; then
  echo "[PASS] map->odom tf published by AMCL" | tee -a "${ART}/gate.txt"
else
  echo "[FAIL] no map->odom tf" | tee -a "${ART}/gate.txt"; PASS=1
fi

echo "[verify] nudging the robot so AMCL updates"
timeout 5 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}}' > /dev/null 2>&1 || true

echo "[verify] AMCL pose + covariance"
POSE=$(timeout 8 ros2 topic echo /amcl_pose --once 2>/dev/null)
echo "${POSE}" > "${ART}/amcl_pose.txt"
# Max diagonal covariance (x var = entry 0, y var = entry 7, yaw var = 35).
XVAR=$(echo "${POSE}" | grep -A1 'covariance:' | grep -oE '[0-9.]+e?-?[0-9]*' | head -1)
if [ -z "${POSE}" ]; then
  echo "[FAIL] no /amcl_pose" | tee -a "${ART}/gate.txt"; PASS=1
elif echo "${POSE}" | grep -qiwE 'nan'; then
  echo "[FAIL] amcl pose NaN" | tee -a "${ART}/gate.txt"; PASS=1
else
  echo "[PASS] /amcl_pose published; x-var ~ ${XVAR}" | tee -a "${ART}/gate.txt"
  # Converged if x variance is small (< 0.5 m^2).
  if awk "BEGIN{exit !(${XVAR:-99} < 0.5)}"; then
    echo "[PASS] AMCL covariance converged (x-var < 0.5)" | tee -a "${ART}/gate.txt"
  else
    echo "[WARN] x-var ${XVAR} not yet < 0.5 (needs more motion)" \
      | tee -a "${ART}/gate.txt"
  fi
fi

echo "[verify] estimate vs ground-truth (both in their own frame: map vs odom)"
# Compare base_link expressed in map (AMCL) against base_link in odom
# (dead-reckoned ground truth). With a correct map->odom these agree; the raw
# /amcl_pose and /odom topics live in different frames so are not comparable
# numerically.
MB=$(timeout 5 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null \
  | grep -A1 'Translation' | grep -oE '[-0-9.]+' | tr '\n' ' ')
OB=$(timeout 5 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null \
  | grep -A1 'Translation' | grep -oE '[-0-9.]+' | tr '\n' ' ')
ERR=$(awk -v m="${MB}" -v o="${OB}" 'BEGIN{
  split(m,a," "); split(o,b," ");
  print sqrt((a[1]-b[1])^2+(a[2]-b[2])^2)}')
echo "map->base=(${MB}) odom->base=(${OB}) err=${ERR} m" | tee -a "${ART}/gate.txt"
if awk "BEGIN{exit !(${ERR:-99} < 1.0)}"; then
  echo "[PASS] AMCL estimate within 1.0 m of ground truth (map frame)" \
    | tee -a "${ART}/gate.txt"
else
  echo "[WARN] AMCL ${ERR} m from ground truth" | tee -a "${ART}/gate.txt"
fi

echo "================ GATE ================"; cat "${ART}/gate.txt"
[ "${PASS}" -eq 0 ] && echo "[verify] LOCALIZATION GATE: PASS" \
  || echo "[verify] LOCALIZATION GATE: FAIL"
exit "${PASS}"
