#!/usr/bin/env bash
# Verdict on whether the stack is actually working. Run it on the board, then
# run the same thing on the dev laptop.
#
# The point is that those two runs answer DIFFERENT questions, and conflating
# them has cost this project days:
#   board  PASS + laptop FAIL  -> the stack is fine, DDS discovery is not
#   board  FAIL                -> the stack is broken, ignore the laptop
#
# Exit code is 0 only if every check passes.
set -uo pipefail

WS="${BB_WS:-$HOME/ros2_ws}"
FAIL=0
ON_BOARD=0
[ -e /dev/ttyLIDAR ] || [ -S /var/run/arduino-router.sock ] && ON_BOARD=1

ok()   { printf "  \033[32mPASS\033[0m  %-34s %s\n" "$1" "${2:-}"; }
bad()  { printf "  \033[31mFAIL\033[0m  %-34s %s\n" "$1" "${2:-}"; FAIL=1; }
note() { printf "  ....  %-34s %s\n" "$1" "${2:-}"; }

# shellcheck disable=SC1091
set +u; source "$WS/install/setup.bash" 2>/dev/null; set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
_cfg="$(ros2 pkg prefix big_bertha_bringup 2>/dev/null)/share/big_bertha_bringup/config/dds/cyclonedds.xml"
[ -f "$_cfg" ] && export CYCLONEDDS_URI="file://$_cfg"

# rate <topic> <min_hz> -- ros2 topic hz needs time to discover AND sample;
# short timeouts here produce false negatives, which fooled me repeatedly.
rate () {
  local t="$1" min="$2"
  local r
  r=$(timeout 14 ros2 topic hz "$t" 2>/dev/null | grep -o "average rate: [0-9.]*" | head -1 | awk '{print $3}')
  if [ -z "$r" ]; then bad "$t" "silent"; return; fi
  if awk "BEGIN{exit !($r >= $min)}"; then ok "$t" "${r} Hz"; else bad "$t" "${r} Hz, want >= ${min}"; fi
}

echo "=== where am I ==="
if [ "$ON_BOARD" = 1 ]; then note "context" "ON THE BOARD"; else note "context" "remote (dev machine)"; fi

if [ "$ON_BOARD" = 1 ]; then
  echo "=== board-local prerequisites ==="
  [ -e /dev/ttyLIDAR ] && ok "/dev/ttyLIDAR" "$(readlink -f /dev/ttyLIDAR)" \
    || bad "/dev/ttyLIDAR" "missing -> install udev/99-big-bertha-lidar.rules"
  if arduino-app-cli app list 2>/dev/null | grep -q "hardware_bridge_app.*running"; then
    ok "MCU sketch" "running"
  else
    bad "MCU sketch" "not running -> every topic below will be silent"
  fi
fi

echo "=== sensor + control chain ==="
rate /scan            5
rate /scan_filtered   5
rate /imu            80
rate /filtered/imu   80
rate /odom           80

echo "=== SLAM ==="
# /map is TRANSIENT_LOCAL and slam only republishes when the robot MOVES, so
# `ros2 topic hz /map` reads silent even when the map is perfectly healthy.
# Read it latched instead.
_w=$(timeout 20 ros2 topic echo /map --once --qos-durability transient_local \
       --field info.width 2>/dev/null | head -1 | tr -dc '0-9')
if [ -n "${_w:-}" ] && [ "${_w:-0}" -gt 1 ]; then ok "/map" "${_w} cells wide"
else bad "/map" "empty or unreadable (latched read)"; fi

if timeout 12 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | grep -q Translation; then
  ok "tf map->base_link" "resolves"
else
  bad "tf map->base_link" "missing -> slam has not localised"
fi

echo "=== lifecycle ==="
for n in slam_toolbox controller_server planner_server bt_navigator; do
  s=$(timeout 10 ros2 lifecycle get "/$n" 2>/dev/null)
  # Match "active [3]" only. A plain *active* glob also matches "inactive [2]"
  # and "unconfigured", so a server that never activated read as PASS.
  case "$s" in
    active\ *) ok "$n" "$s";;
    "") bad "$n" "no response";;
    *) bad "$n" "$s";;
  esac
done

echo
if [ "$FAIL" = 0 ]; then
  echo "VERDICT: everything passed."
  [ "$ON_BOARD" = 0 ] && echo "  (remote run, so DDS discovery works too)"
else
  echo "VERDICT: failures above."
  if [ "$ON_BOARD" = 0 ]; then
    echo "  Run this ON THE BOARD before concluding the stack is broken."
    echo "  Board passing + laptop failing means discovery, not the stack:"
    echo "    BB_DDS_PEER=<this machine's ip> ./scripts/run_stack.sh"
  fi
fi
exit $FAIL
