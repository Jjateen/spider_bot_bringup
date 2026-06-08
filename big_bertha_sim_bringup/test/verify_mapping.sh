#!/usr/bin/env bash
# Headless acceptance gate for the mapping module (issue #7).
#
# Assumes the sim (sim_drive:=true, odom_tf:=false), EKF, and slam_toolbox are
# running and a scripted drive (test/scripted_drive.sh) has already traversed
# the arena. Asserts /map exists with a plausible extent and a meaningful
# fraction of known cells, then saves maps/obstacle_world.{yaml,pgm} via
# nav2 map_saver_cli and validates the grid. Evidence -> verification_artifacts/mapping/.
# Source ROS first (its setup scripts trip 'set -u'), then enable strict mode.
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -uo pipefail

ART="${ART:-verification_artifacts/mapping}"
MAP_DIR="${MAP_DIR:-big_bertha_sim_bringup/maps}"
mkdir -p "${ART}" "${MAP_DIR}"
: > "${ART}/gate.txt"
PASS=0

echo "[verify] /map metadata"
META=$(timeout 8 ros2 topic echo /map --once --field info 2>/dev/null)
echo "${META}" > "${ART}/map_info.txt"
W=$(echo "${META}" | grep -oP 'width:\s*\K[0-9]+' | head -1)
H=$(echo "${META}" | grep -oP 'height:\s*\K[0-9]+' | head -1)
RES=$(echo "${META}" | grep -oP 'resolution:\s*\K[0-9.]+' | head -1)
echo "map: ${W} x ${H} cells @ ${RES} m/cell" | tee -a "${ART}/gate.txt"
if [ -n "${W}" ] && [ -n "${H}" ] && [ "${W}" -gt 50 ] && [ "${H}" -gt 50 ]; then
  EXT_W=$(awk "BEGIN{print ${W}*${RES}}"); EXT_H=$(awk "BEGIN{print ${H}*${RES}}")
  echo "[PASS] plausible extent ~${EXT_W} x ${EXT_H} m" | tee -a "${ART}/gate.txt"
else
  echo "[FAIL] map extent too small / missing" | tee -a "${ART}/gate.txt"; PASS=1
fi

echo "[verify] saving map"
timeout 30 ros2 run nav2_map_server map_saver_cli \
  -f "${MAP_DIR}/obstacle_world" --ros-args -p save_map_timeout:=20.0 \
  > "${ART}/map_saver.log" 2>&1
if [ -f "${MAP_DIR}/obstacle_world.yaml" ] && [ -f "${MAP_DIR}/obstacle_world.pgm" ]; then
  echo "[PASS] saved obstacle_world.yaml + .pgm" | tee -a "${ART}/gate.txt"
else
  echo "[FAIL] map_saver did not write grid" | tee -a "${ART}/gate.txt"; PASS=1
fi

echo "[verify] grid validity (known cells fraction)"
if [ -f "${MAP_DIR}/obstacle_world.pgm" ]; then
  # PGM: count free(254/255) + occupied(0) vs unknown(205). Use python for
  # a robust binary read.
  python3 - "${MAP_DIR}/obstacle_world.pgm" <<'PY' | tee -a "${ART}/gate.txt"
import sys
p = sys.argv[1]
with open(p, 'rb') as f:
    data = f.read()
# Parse PGM header (P5 binary).
idx = 0
def token():
    global idx
    while data[idx:idx+1].isspace():
        idx += 1
    s = idx
    while not data[idx:idx+1].isspace():
        idx += 1
    return data[s:idx]
magic = token(); w = int(token()); h = int(token()); maxv = int(token())
idx += 1  # single whitespace after maxval
pix = data[idx:idx+w*h]
known = sum(1 for b in pix if b != 205)
total = w*h
frac = known/total if total else 0
print(f"grid {w}x{h}, known cells {known}/{total} = {frac:.2%}")
print("KNOWN_OK" if frac > 0.1 else "KNOWN_LOW")
PY
else
  echo "KNOWN_FAIL"; PASS=1
fi

if grep -q "KNOWN_LOW\|KNOWN_FAIL" "${ART}/gate.txt"; then
  echo "[FAIL] too few known cells" | tee -a "${ART}/gate.txt"; PASS=1
else
  echo "[PASS] map has a meaningful known-cell fraction" | tee -a "${ART}/gate.txt"
fi

echo "================ GATE ================"; cat "${ART}/gate.txt"
[ "${PASS}" -eq 0 ] && echo "[verify] MAPPING GATE: PASS" || echo "[verify] MAPPING GATE: FAIL"
exit "${PASS}"
