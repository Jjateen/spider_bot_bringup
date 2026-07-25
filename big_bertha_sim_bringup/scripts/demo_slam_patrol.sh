#!/usr/bin/env bash
# Two-phase autonomy demo: SLAM builds the arena map on a live 3-goal tour,
# the map is saved, then the SAME patrol runs against the freshly built map
# with AMCL localizing on it (known-map mode).
#
# Phase 1  bringup slam:=true; the 3-goal tour doubles as autonomous
#          exploration (planner allows unknown space, rolling costmap); the
#          map grows behind the robot and loop-closes on the return leg.
#          slam_toolbox anchors the map at the world spawn (map_start_pose in
#          slam_toolbox.yaml), so the built map is world-aligned and the same
#          goal coordinates work in both phases.
# Phase 2  demo.launch.py patrol:=true localization:=amcl map:=<built map>.
#          AMCL is seeded at spawn (amcl.yaml) and localizes against the map
#          from phase 1.
#
# Usage: demo_slam_patrol.sh [work_dir] [rviz]
#   work_dir  where the built map + phase logs land (default /tmp/slam_patrol)
#   rviz      true|false (default false; true for recording/watching)
set -o pipefail

WORK="${1:-/tmp/slam_patrol}"
RVIZ="${2:-false}"
PKG_SHARE=$(ros2 pkg prefix big_bertha_sim_bringup)/share/big_bertha_sim_bringup
mkdir -p "$WORK"

teardown() { bash "$PKG_SHARE/scripts/kill_sim.sh" >/dev/null 2>&1; }

echo "=== PHASE1 mapping tour start $(date +%s) ==="
teardown
sleep 3
ros2 launch big_bertha_sim_bringup bringup.launch.py \
  slam:=true rviz:="$RVIZ" rviz_config:=mapping use_sim_time:=true \
  > "$WORK/phase1.log" 2>&1 &
sleep 25  # controllers + Nav2 up; send_goal also blocks on the action server
bash "$PKG_SHARE/scripts/send_patrol.sh" "3.5,3.5" "-3.5,3.5" "-3.5,-3.5" \
  2>&1 | tee "$WORK/phase1_goals.log"
N1=$(grep -c "Goal accepted" "$WORK/phase1_goals.log")
echo "=== PHASE1 tour done ($N1 goals sent) ==="

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
