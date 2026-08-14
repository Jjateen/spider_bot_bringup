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
RViz for the waypoint demo. Runs on the DEV LAPTOP, not the board.

The board has no display, and the whole point of the demo is a human clicking
waypoints, so RViz lives on the laptop and talks to the board over DDS. That
crossing is the fragile part: /map is TRANSIENT_LOCAL and large, the costmaps
are large, and they have to arrive over the network. Run this on the LAN where
the laptop is the access point (fast, local) rather than over Tailscale.

    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ros2 launch big_bertha_bringup nav_rviz.launch.py

If displays stay empty, it is DDS discovery, not RViz: check `ros2 topic list`
sees /map from the laptop. The board-side cyclonedds.xml discovers by multicast
on the LAN; if the laptop's network blocks it, point CYCLONEDDS_URI at a config
listing the board as a unicast <Peer>.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch RViz with the hardware navigation view."""
    pkg = get_package_share_directory('big_bertha_bringup')
    default_config = os.path.join(pkg, 'config', 'rviz', 'nav_hardware.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=default_config),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
    ])
