#!/usr/bin/env python3
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
"""
One-command A->B demo with RViz (known-map mode by default).

Brings up the full stack and auto-sends a NavigateToPose goal from the pre-fed
point A (spawn) to point B, with the combined RViz view (map, lidar /scan,
global + local costmaps, planned path, robot).

Localization mode
-----------------
By default the demo runs in KNOWN-MAP mode (``slam:=false``): map_server serves
the saved ``maps/obstacle_world.yaml`` (which covers the whole arena) and AMCL
localizes against it. The global costmap is therefore fully populated from the
static map, so a goal at world B is plannable and the A->B run is repeatable.

Pass ``slam:=true`` to run live SLAM (slam_toolbox) instead. In SLAM mode the
map only covers the area explored so far, so a goal far outside it may be
unplannable until the robot drives there; the NavfnPlanner is configured with
``allow_unknown: true`` so it at least degrades gracefully.

Coordinates
-----------
In known-map mode the saved map is world-axis-aligned, so the ``map`` frame
matches the Gazebo world. The robot spawns at world A = (-3.5, -3.5) and the
goal is world B = (3.5, 3.5) -- the default below. The straight A->B diagonal is
blocked by the obstacle_world pillars/boxes, so Nav2 must route around them.

    ros2 launch big_bertha_sim_bringup demo.launch.py
    ros2 launch big_bertha_sim_bringup demo.launch.py slam:=true
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Full-stack known-map (or SLAM) + RViz + automatic A->B navigation."""
    pkg = get_package_share_directory('big_bertha_sim_bringup')

    slam = LaunchConfiguration('slam')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_delay = LaunchConfiguration('goal_delay')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'bringup.launch.py')),
        launch_arguments={
            'slam': slam,              # default false: known-map (AMCL) mode
            'rviz': 'true',            # open RViz
            'rviz_config': 'integration',  # map + lidar + costmaps + path + robot
            'use_sim_time': 'true',
            # Spawn at world A=(-3.5,-3.5) yaw 0.785 (matches the AMCL seed in
            # amcl.yaml so localization starts converged at A).
        }.items(),
    )

    # Goal as a single NavigateToPose YAML argument (frame_id: map).
    goal_yaml = [
        '{pose: {header: {frame_id: map}, pose: {position: {x: ',
        goal_x, ', y: ', goal_y,
        ', z: 0.0}, orientation: {w: 1.0}}}}',
    ]

    send_goal = TimerAction(
        period=goal_delay,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'action', 'send_goal', '/navigate_to_pose',
                    'nav2_msgs/action/NavigateToPose', goal_yaml,
                ],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam', default_value='false',
            description='true: live SLAM; false: known-map (AMCL) for the '
                        'repeatable A->B demo (default)'),
        DeclareLaunchArgument(
            'goal_x', default_value='3.5',
            description='Goal B x in the map frame (world B; map frame is '
                        'world-aligned in known-map mode)'),
        DeclareLaunchArgument(
            'goal_y', default_value='3.5',
            description='Goal B y in the map frame (world B)'),
        DeclareLaunchArgument(
            'goal_delay', default_value='45.0',
            description='Seconds to wait for localization + Nav2 to activate '
                        'before sending the goal'),
        bringup,
        send_goal,
    ])
