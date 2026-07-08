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
Launch the Big Bertha hardware bridge node.

Subscribes to ``/position_controller/commands`` (12 joint targets in radians),
converts them to PWM values, and forwards them over UART to the STM32U585
co-processor which drives the PCA9685 servo driver and reads the MPU9250 IMU.
Incoming IMU data is published on ``/imu`` as ``sensor_msgs/Imu``.

Use ``use_sim_time:=false`` (default) for real hardware.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Build the hardware-bridge launch description."""
    pkg = get_package_share_directory('big_bertha_bringup')
    default_params = os.path.join(pkg, 'config', 'hardware_bridge.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=default_params),

        Node(
            package='big_bertha_bringup',
            executable='hardware_bridge_node',
            name='hardware_bridge',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
