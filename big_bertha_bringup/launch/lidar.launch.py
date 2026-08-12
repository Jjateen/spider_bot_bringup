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
YDLidar X2 driver, brought up to the active state.

This used to be a manual step outside every launch file, which meant the
documented bringup sequence could not actually be run as one command. It is
here now so `composed_stack.launch.py` brings up the whole pipeline.

Two details the driver forces on us.

It is a LifecycleNode, and its own ydlidar_launch.py does not transition it,
so a plain include leaves the driver sitting in `unconfigured` publishing
nothing. nav2_lifecycle_manager drives it to `active` instead of hand-rolled
transition event handlers, because it is already a dependency here and it
retries rather than racing the port open.

The frame_id override lives in config/ydlidar_x2.yaml, not here. See that file
for why the driver default is actively harmful on this robot.

The driver is NOT composed into a container. It owns a serial port and does
its own blocking reads, so it gains nothing from intra-process comms (its only
output, /scan, goes to a different container) while a hang in it would take
the control loop down with it.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Bring the X2 up to active."""
    pkg = get_package_share_directory('big_bertha_bringup')
    default_params = os.path.join(pkg, 'config', 'ydlidar_x2.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=default_params),

        LifecycleNode(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            namespace='',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_lidar',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'autostart': True,
                'bond_timeout': 0.0,
                'node_names': ['ydlidar_ros2_driver_node'],
            }],
        ),
    ])
