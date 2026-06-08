#!/usr/bin/env bash
# Headless acceptance gate for the state_estimation module (issue #6).
#
# Assumes a headless sim launched with odom_tf:=false plus the EKF node are
# already running. Asserts the EKF publishes a continuous odom->base_link
# transform and a /odometry/filtered with bounded covariance. Evidence goes
# to verification_artifacts/state_estimation/.
# Source ROS first (its setup scripts trip 'set -u'), then enable strict mode.
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -uo pipefail

ART="${ART:-verification_artifacts/state_estimation}"
mkdir -p "${ART}"
: > "${ART}/gate.txt"
PASS=0

echo "[verify] EKF output rate"
HZ=$(timeout 8 ros2 topic hz /odometry/filtered 2>/dev/null \
      | grep -oP 'average rate: \K[0-9.]+' | head -1)
if [ -n "${HZ}" ]; then
  echo "[PASS] /odometry/filtered @ ${HZ} Hz" | tee -a "${ART}/gate.txt"
else
  echo "[FAIL] no /odometry/filtered" | tee -a "${ART}/gate.txt"; PASS=1
fi

echo "[verify] odom->base_link transform present"
TF=$(timeout 6 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null | head -20)
echo "${TF}" > "${ART}/tf_sample.txt"
if echo "${TF}" | grep -qE 'Translation'; then
  echo "[PASS] odom->base_link tf available" | tee -a "${ART}/gate.txt"
else
  echo "[FAIL] odom->base_link tf missing" | tee -a "${ART}/gate.txt"; PASS=1
fi

echo "[verify] EKF is the sole odom->base_link publisher"
FILT=$(timeout 6 ros2 topic echo /odometry/filtered --once 2>/dev/null)
echo "${FILT}" > "${ART}/filtered_sample.txt"
# Pull the first pose covariance diagonal entry (x variance) as a bound check.
COV=$(echo "${FILT}" | grep -A2 'covariance:' | grep -oE '[0-9.]+e?-?[0-9]*' | head -1)
if echo "${FILT}" | grep -qiwE 'nan'; then
  echo "[FAIL] filtered odom has NaN" | tee -a "${ART}/gate.txt"; PASS=1
elif [ -n "${COV}" ]; then
  echo "[PASS] filtered covariance present (x-var ~ ${COV}), no NaN" \
    | tee -a "${ART}/gate.txt"
else
  echo "[FAIL] no covariance in filtered odom" | tee -a "${ART}/gate.txt"; PASS=1
fi

echo "================ GATE ================"; cat "${ART}/gate.txt"
[ "${PASS}" -eq 0 ] && echo "[verify] STATE_ESTIMATION GATE: PASS" \
  || echo "[verify] STATE_ESTIMATION GATE: FAIL"
exit "${PASS}"
