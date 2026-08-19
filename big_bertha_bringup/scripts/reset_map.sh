#!/usr/bin/env bash
# Copyright 2026 Jjateen Gundesha
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Throw away the current map and both costmaps, then rebuild from what the
# lidar sees now.
#
# Why this exists: slam_toolbox only folds a scan into the map after the robot
# has moved minimum_travel_distance (0.1 m) or turned minimum_travel_heading
# (0.1 rad). Standing still it processes nothing, so whatever was in the very
# first scan is in the map permanently. Powering the servos means a person
# walks up to the buck-converter switch, so that person gets mapped standing
# next to the robot, their footprint overlaps the robot's, and Nav2 refuses to
# plan with a collision warning. Waiting does not help; nothing is being
# integrated. Step clear, run this, and the map is rebuilt without you in it.
#
# Usage: reset_map.sh
set -uo pipefail

echo "[reset_map] clearing the slam_toolbox map"
if ros2 service list 2>/dev/null | grep -q '/slam_toolbox/reset'; then
  ros2 service call /slam_toolbox/reset slam_toolbox/srv/Reset "{}" >/dev/null \
    && echo "[reset_map]   slam_toolbox reset" \
    || echo "[reset_map]   reset call failed"
else
  # older slam_toolbox has no reset: clearing the pose-graph changes is the
  # closest equivalent that does not need a relaunch.
  ros2 service call /slam_toolbox/clear_changes \
    slam_toolbox/srv/Clear "{}" >/dev/null 2>&1 \
    && echo "[reset_map]   cleared pose-graph changes" \
    || echo "[reset_map]   no reset service; restart the slam node to rebuild"
fi

# The costmaps keep their own copy, so the stale obstacle survives a map reset
# until they are cleared too.
for srv in /global_costmap/clear_entirely_global_costmap \
           /local_costmap/clear_entirely_local_costmap; do
  ros2 service call "$srv" nav2_msgs/srv/ClearEntireCostmap \
    "{request: {}}" >/dev/null 2>&1 \
    && echo "[reset_map]   cleared ${srv##*/}" \
    || echo "[reset_map]   ${srv##*/} not available"
done

echo "[reset_map] done. Drive a short distance so slam re-integrates scans."
