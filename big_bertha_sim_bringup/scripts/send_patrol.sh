#!/usr/bin/env bash
# Multi-goal patrol: send NavigateToPose goals in sequence, each blocking until
# the robot reaches it (ros2 action send_goal waits for the result), so the bot
# visits every waypoint in order. Used by demo.launch.py with patrol:=true.
# Usage: send_patrol.sh "x1,y1" "x2,y2" ...   (map frame, metres)
set +u
i=0
for wp in "$@"; do
  i=$((i + 1))
  x="${wp%,*}"
  y="${wp#*,}"
  echo "=== patrol goal ${i} -> (${x}, ${y}) ==="
  # Retry rejected goals: Nav2 accepts connections before its servers are
  # active, so a goal sent during bringup is rejected instead of queued.
  for try in 1 2 3 4 5 6; do
    out=$(ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: map}, pose: {position: {x: ${x}, y: ${y}, z: 0.0}, orientation: {w: 1.0}}}}" 2>&1)
    echo "${out}" | grep -E "Goal accepted|Goal was rejected|Goal finished|status"
    echo "${out}" | grep -q "Goal was rejected" || break
    echo "=== goal ${i} rejected (try ${try}); retrying in 10s ==="
    sleep 10
  done
done
echo "=== patrol complete: ${i} goals, robot back at the start ==="
