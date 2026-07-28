#!/usr/bin/env bash
# Two-phase autonomy demo: SLAM explores and maps the arena on its own, the
# map is saved, then a 3-goal patrol runs against the freshly built map with
# AMCL localizing on it (known-map mode).
#
# Phase 1  bringup slam:=true; FRONTIER EXPLORATION builds the map -- the
#          robot repeatedly drives to the nearest boundary between free and
#          unknown space, letting Nav2 avoid obstacles, until no frontier is
#          left, then returns to its start point. No preset waypoints: where
#          it goes depends on what it has seen, which is what makes it a SLAM
#          demo rather than a patrol with mapping switched on.
#          slam_toolbox anchors the map at the world spawn (map_start_pose in
#          slam_toolbox.yaml), so the built map is world-aligned and world
#          goal coordinates work in both phases.
# Phase 2  demo.launch.py patrol:=true localization:=amcl map:=<built map>.
#          AMCL is seeded at spawn (amcl.yaml) and localizes against the map
#          from phase 1 while the patrol plans through the obstacle field.
#
# Usage: demo_slam_patrol.sh [work_dir] [rviz] [map_speed]
#   work_dir  where the built map + phase logs land (default /tmp/slam_patrol)
#   rviz      true|false (default false; true for recording/watching)
#   map_speed mapping-phase Nav2 speed (default 0.29)
set -uo pipefail

WORK="${1:-/tmp/slam_patrol}"
RVIZ="${2:-false}"
# Mapping-phase Nav2 speed. Lower it if exploration starts aborting goals:
# scan matching and RPP's collision projection both degrade with speed, and
# phase 1 drives near walls while the map is still forming.
MAP_SPEED="${3:-0.29}"
PKG_SHARE=$(ros2 pkg prefix big_bertha_sim_bringup 2>/dev/null)/share/big_bertha_sim_bringup
[ -d "$PKG_SHARE" ] || { echo "workspace not sourced (big_bertha_sim_bringup not found)"; exit 1; }
mkdir -p "$WORK"

teardown() { bash "$PKG_SHARE/scripts/kill_sim.sh" >/dev/null 2>&1; }
# Ctrl-C/kill must not strand gz/Nav2/EKF (the stale-survivor class kill_sim
# exists for).
trap teardown EXIT INT TERM

echo "=== PHASE1 frontier exploration start $(date +%s) ==="
teardown
sleep 3
ros2 launch big_bertha_sim_bringup bringup.launch.py \
  slam:=true nav_speed:="$MAP_SPEED" rviz:="$RVIZ" rviz_config:=mapping \
  use_sim_time:=true \
  > "$WORK/phase1.log" 2>&1 &
sleep 25  # controllers + Nav2 up; the action client also blocks on the server
# Frontier exploration: drive whatever is still unknown, avoiding obstacles via
# Nav2, until no frontier remains, then come home. The old fixed 4-corner tour
# went to preset coordinates whether or not they were informative, and one
# aborted goal stalled the whole run; the explorer blacklists a failed frontier
# and continues.
python3 "$PKG_SHARE/scripts/frontier_explore.py" \
  2>&1 | tee "$WORK/phase1_goals.log"
N1=$(grep -c "SUCCEEDED" "$WORK/phase1_goals.log")
echo "=== PHASE1 exploration done ($N1 goals reached) ==="
grep -q "explore complete" "$WORK/phase1_goals.log" || {
  echo "=== FAILED: exploration did not finish, not saving a map ==="; exit 1; }
[ "$N1" -ge 1 ] || { echo "=== FAILED: no goal reached, not saving a map ==="; exit 1; }

echo "=== saving built map ==="
# slam_toolbox publishes /map at ~1 Hz; the saver's default 2 s subscription
# window can miss it, so widen the timeout and retry.
for try in 1 2 3; do
  ros2 run nav2_map_server map_saver_cli -f "$WORK/built_map" \
    --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0 \
    >> "$WORK/phase1.log" 2>&1
  [ -f "$WORK/built_map.yaml" ] && break
  echo "=== map save try ${try} failed; retrying ==="
  sleep 3
done
teardown
if [ ! -f "$WORK/built_map.yaml" ]; then
  echo "=== FAILED: map save produced no yaml ==="; exit 1
fi
echo "=== map saved: $WORK/built_map.yaml ==="

echo "=== PHASE2 patrol on built map start $(date +%s) ==="
# Cool-down: relaunching seconds after teardown left the EKF stuck on
# "Waiting for clock to start" (stale DDS/clock state) -> whole Nav2
# lifecycle inactive. 20 s + a daemon restart makes phase 2 boot clean.
teardown
sleep 20
ros2 launch big_bertha_sim_bringup demo.launch.py \
  patrol:=true localization:=amcl map:="$WORK/built_map.yaml" \
  rviz:="$RVIZ" rviz_config:=patrol use_sim_time:=true \
  > "$WORK/phase2.log" 2>&1 &
LP=$!
for i in $(seq 1 90); do
  grep -q "patrol complete" "$WORK/phase2.log" && break
  kill -0 $LP 2>/dev/null || break
  sleep 10
done
N2=$(grep -c "Goal succeeded" "$WORK/phase2.log")
teardown
echo "=== PHASE2 done: $N2/3 goals succeeded on the built map ==="
[ "$N2" = "3" ] && { echo "=== DEMO PASS ==="; exit 0; }
echo "=== DEMO FAIL ==="
exit 1
