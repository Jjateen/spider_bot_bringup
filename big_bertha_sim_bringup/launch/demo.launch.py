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
One-command A->B demo with live SLAM and RViz.

Brings up the full stack in SLAM mode (sim + learned gait + EKF + slam_toolbox
+ Nav2) with the combined RViz view (live map, lidar /scan, costmaps, planned
path, robot), then auto-sends a NavigateToPose goal from the pre-fed point A
(spawn) to point B.

Coordinates: the robot spawns at world A = (-3.5, -3.5) with yaw 0, so the
slam_toolbox ``map`` frame starts world-axis-aligned at A. World B = (3.5, 3.5)
is therefore B - A = (7.0, 7.0) in the ``map`` frame -- the default goal below.
The straight A->B line is blocked by the obstacle_world obstacles, so Nav2 must
route around them.

    ros2 launch big_bertha_sim_bringup demo.launch.py
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
    """Full-stack SLAM + RViz + automatic A->B navigation."""
    pkg = get_package_share_directory('big_bertha_sim_bringup')

    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_delay = LaunchConfiguration('goal_delay')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'bringup.launch.py')),
        launch_arguments={
            'slam': 'true',            # build the map live
            'rviz': 'true',            # open RViz
            'rviz_config': 'integration',  # map + lidar + costmaps + path + robot
            'use_sim_time': 'true',
            'yaw': '0.0',              # map frame world-aligned -> goal (7,7) = B
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
            'goal_x', default_value='7.0',
            description='Goal B x in the map frame (default = world B relative to spawn A)'),
        DeclareLaunchArgument(
            'goal_y', default_value='7.0',
            description='Goal B y in the map frame'),
        DeclareLaunchArgument(
            'goal_delay', default_value='45.0',
            description='Seconds to wait for SLAM + Nav2 to activate before sending the goal'),
        bringup,
        send_goal,
    ])
