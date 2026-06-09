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
Launch the Big Bertha C++ ONNX policy controller.

Loads ``config/policy.yaml`` and points ``model_path`` at the bundled
``models/policy.onnx`` (overridable). The node drives the learned gait from
``/cmd_vel`` onto ``/position_controller/commands``.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Build the policy-controller launch description."""
    pkg = get_package_share_directory('big_bertha_policy_controller')
    default_model = os.path.join(pkg, 'models', 'policy.onnx')
    default_params = os.path.join(pkg, 'config', 'policy.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model_path = LaunchConfiguration('model_path')
    params_file = LaunchConfiguration('params_file')
    start_enabled = LaunchConfiguration('start_enabled')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('model_path', default_value=default_model),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('start_enabled', default_value='true'),

        Node(
            package='big_bertha_policy_controller',
            executable='policy_controller_node',
            name='policy_controller',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    'model_path': model_path,
                    'start_enabled': start_enabled,
                },
            ],
        ),
    ])
