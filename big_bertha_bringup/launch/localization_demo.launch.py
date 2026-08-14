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
Localization + waypoint navigation on a pre-built map.

This is the SLAM demo's counterpart, kept as its OWN launch file so the SLAM
path is untouched. composed_stack.launch.py already brings up localization when
slam:=false, so this is a thin wrapper that forces that mode and requires a
map, rather than a fork of the stack:

    SLAM demo (unchanged) : ros2 launch ... composed_stack.launch.py
    this demo             : ros2 launch ... localization_demo.launch.py map:=...

What comes up: description, lidar, the composed control chain, then
map_server (serving the saved map), AMCL (estimating map->odom against it), and
Nav2. leg_odometry still owns odom->base_link, so map->odom from AMCL plus
odom->base_link gives the robot's pose in the map frame, which is what RViz
shows and what Nav2 plans in.

x/y/yaw are the robot's best-guess starting pose in the map frame; they seed
AMCL's initial_pose. They do not have to be exact, the 2D Pose Estimate tool in
RViz corrects it, but a rough guess speeds convergence.

Run RViz on the DEV LAPTOP (the board cannot render), see nav_rviz.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Bring up the localization + waypoint demo."""
    bringup_pkg = get_package_share_directory('big_bertha_bringup')
    sim_pkg = get_package_share_directory('big_bertha_sim_bringup')

    # Default is the checked-in map. Pass map:=/path/to/your.yaml to use a map
    # recorded with the SLAM demo + map_saver_cli, which is what a real run
    # wants: the checked-in one is a single-viewpoint capture, not a survey.
    default_map = os.path.join(sim_pkg, 'maps', 'big_bertha_room.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Saved map yaml to localise against'),
        DeclareLaunchArgument(
            'x', default_value='0.0',
            description='Best-guess start pose in the map frame (seeds AMCL)'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'start_enabled', default_value='true',
            description='Policy armed so Nav2 /cmd_vel actually drives the gait'),
        DeclareLaunchArgument('with_lidar', default_value='true'),
        DeclareLaunchArgument('nav_speed', default_value='0.29'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_pkg, 'launch', 'composed_stack.launch.py')),
            launch_arguments={
                # The one line that makes this localization rather than SLAM.
                'slam': 'false',
                'map': LaunchConfiguration('map'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'yaw': LaunchConfiguration('yaw'),
                'start_enabled': LaunchConfiguration('start_enabled'),
                'with_lidar': LaunchConfiguration('with_lidar'),
                'with_nav': 'true',
                'nav_speed': LaunchConfiguration('nav_speed'),
            }.items(),
        ),
    ])
