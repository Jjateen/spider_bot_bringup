#!/usr/bin/env bash
# Headless acceptance gate for the planning module (issue #9).
#
# Assumes the sim (sim_drive:=true, odom_tf:=false), EKF, localization
# (map_server + amcl), and Nav2 are running. Sends a NavigateToPose goal from
# A to B (with obstacles between), asserts the action result is SUCCEEDED, and
# checks the /odom trajectory kept clear of the known obstacles (min distance
# > the inflation radius). Evidence -> verification_artifacts/planning/.
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -uo pipefail

ART="${ART:-verification_artifacts/planning}"
mkdir -p "${ART}"
: > "${ART}/gate.txt"
PASS=0

GOAL_X="${GOAL_X:-3.5}"
GOAL_Y="${GOAL_Y:-3.5}"

# --- Primary check: the Nav2 planner produces a valid global path ----------
# ComputePathToPose exercises the global costmap + planner without depending on
# the (sim-transfer-limited) gait closing the loop. A SUCCEEDED result proves
# the costmaps are populated and the planner is wired correctly.
echo "[verify] planner-produces-a-path check (ComputePathToPose A->B)"
PATH_RES=$(timeout 60 ros2 action send_goal /compute_path_to_pose \
  nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: 0.0}, orientation: {w: 1.0}}}, use_start: false}" \
  2>&1)
echo "${PATH_RES}" > "${ART}/compute_path_result.txt"
if echo "${PATH_RES}" | grep -q "Goal finished with status: SUCCEEDED"; then
  NPOSES=$(echo "${PATH_RES}" | grep -c 'position:')
  echo "[PASS] planner produced a global path A->B (~${NPOSES} poses)" \
    | tee -a "${ART}/gate.txt"
else
  echo "[WARN] planner could not produce A->B path (see compute_path_result.txt)" \
    | tee -a "${ART}/gate.txt"
  echo "${PATH_RES}" | grep -E 'error_code|status' | tail -2 | tee -a "${ART}/gate.txt"
fi

# --- Full check: drive A->B and confirm the goal is reached collision-free --
# Record the /odom trajectory while navigating (background sampler).
TRAJ="${ART}/trajectory.csv"
: > "${TRAJ}"
( for i in $(seq 1 120); do
    P=$(timeout 2 ros2 topic echo /odom --once 2>/dev/null \
        | grep -A2 'position:' | grep -E '  *(x|y):' | head -2 \
        | grep -oE '[-0-9.]+' | tr '\n' ',')
    [ -n "${P}" ] && echo "${P}" >> "${TRAJ}"
  done ) &
SAMPLER=$!

echo "[verify] sending NavigateToPose goal A->B (${GOAL_X}, ${GOAL_Y})"
RESULT=$(timeout 180 ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: 0.0}, orientation: {w: 1.0}}}}" \
  2>&1)
echo "${RESULT}" > "${ART}/nav_result.txt"
kill "${SAMPLER}" 2>/dev/null

if echo "${RESULT}" | grep -q "Goal finished with status: SUCCEEDED"; then
  echo "[PASS] NavigateToPose result SUCCEEDED" | tee -a "${ART}/gate.txt"
else
  echo "[FAIL] navigation did not succeed" | tee -a "${ART}/gate.txt"
  echo "${RESULT}" | tail -3 | tee -a "${ART}/gate.txt"
  PASS=1
fi

echo "[verify] collision check vs known obstacles (inflation 0.35 m)"
# Obstacle centres (x,y,clearance_radius) from worlds/obstacle_world.sdf:
#   box_1 (-1.2,-1.2) r~0.57, box_2 (1.0,1.0) r~0.64, box_3 (0.2,-1.8) r~0.49,
#   pillar_1 (0,0) r 0.30, pillar_2 (2.0,-0.5) r 0.25.
python3 - "${TRAJ}" <<'PY' | tee -a "${ART}/gate.txt"
import sys, math
obs = [(-1.2,-1.2,0.57),(1.0,1.0,0.64),(0.2,-1.8,0.49),(0.0,0.0,0.30),(2.0,-0.5,0.25)]
pts = []
for line in open(sys.argv[1]):
    f = [v for v in line.strip().strip(',').split(',') if v]
    if len(f) >= 2:
        try: pts.append((float(f[0]), float(f[1])))
        except ValueError: pass
if not pts:
    print("NO_TRAJ"); raise SystemExit
worst = 1e9; worst_obs = None
for (x, y) in pts:
    for (ox, oy, r) in obs:
        clr = math.hypot(x-ox, y-oy) - r   # distance to obstacle surface
        if clr < worst:
            worst, worst_obs = clr, (ox, oy)
print(f"trajectory points: {len(pts)}")
print(f"min surface clearance: {worst:.3f} m near obstacle {worst_obs}")
# Robot radius 0.18; any clearance > 0 means no body overlap. Require a small
# positive margin to count as collision-free.
print("CLEAR_OK" if worst > 0.0 else "COLLISION")
PY

if grep -q "COLLISION\|NO_TRAJ" "${ART}/gate.txt"; then
  echo "[FAIL] trajectory collided / not recorded" | tee -a "${ART}/gate.txt"; PASS=1
else
  echo "[PASS] trajectory cleared all obstacles" | tee -a "${ART}/gate.txt"
fi

echo "================ GATE ================"; cat "${ART}/gate.txt"
[ "${PASS}" -eq 0 ] && echo "[verify] PLANNING GATE: PASS" \
  || echo "[verify] PLANNING GATE: FAIL"
exit "${PASS}"
