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
# Tell the stack the area is clear and start mapping.
#
# Why this exists: slam_toolbox only folds a scan into the map after the robot
# has moved minimum_travel_distance (0.1 m) or turned minimum_travel_heading
# (0.1 rad). Standing still it processes nothing, so whatever was in the very
# first scan is in the map permanently. Powering the servos means a person
# walks up to the buck-converter switch, so that person gets mapped standing
# next to the robot, their footprint overlaps the robot's, and Nav2 refuses to
# plan with a collision warning.
#
# So on hardware big_bertha.launch.py runs with mapping_autostart:=false, which
# leaves slam_toolbox unconfigured and not subscribed to /scan. Nothing is
# mapped until this script runs. Flip the servo switch, walk back, run this.
#
# Usage: start_mapping.sh [timeout_s]
set -uo pipefail

MGR=/lifecycle_manager_slam/manage_nodes
TIMEOUT=${1:-30}

# Wait for the manager rather than failing on a stack that is still coming up.
echo "[start_mapping] waiting for ${MGR}"
deadline=$((SECONDS + TIMEOUT))
until ros2 service list 2>/dev/null | grep -q "^${MGR}$"; do
  if [ "$SECONDS" -ge "$deadline" ]; then
    echo "[start_mapping] ${MGR} never appeared after ${TIMEOUT}s." >&2
    echo "[start_mapping] Is the stack up, and was it launched with slam:=true?" >&2
    exit 1
  fi
  sleep 1
done

# command 0 is STARTUP in nav2_msgs/srv/ManageLifecycleNodes: configure then
# activate every node the manager owns, which here is slam_toolbox alone.
echo "[start_mapping] starting slam_toolbox"
if ros2 service call "$MGR" nav2_msgs/srv/ManageLifecycleNodes \
    "{command: 0}" | grep -q 'success=True'; then
  echo "[start_mapping] mapping is live. Drive the robot to build the map."
else
  echo "[start_mapping] STARTUP was refused. Check the stack console." >&2
  exit 1
fi
