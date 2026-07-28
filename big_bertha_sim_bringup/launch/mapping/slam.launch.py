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
Mapping bringup: slam_toolbox (online async) for Big Bertha.

Consumes /scan + odom->base_link (from the EKF) and produces /map and the
map->odom transform. The async node is a lifecycle node, so a
nav2_lifecycle_manager with autostart configures + activates it on startup
(so it subscribes to /scan and starts mapping without a manual transition).
After a scripted drive, ``ros2 run nav2_map_server map_saver_cli`` writes
maps/obstacle_world.{yaml,pgm}.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the slam_toolbox launch description."""
    pkg = get_package_share_directory('big_bertha_sim_bringup')
    default_slam = os.path.join(pkg, 'config', 'slam_toolbox.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_config = LaunchConfiguration('slam_config')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['slam_toolbox'],
            'bond_timeout': 0.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('slam_config', default_value=default_slam),

        slam_node,
        lifecycle_manager,
    ])
