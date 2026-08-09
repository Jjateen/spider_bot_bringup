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
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the policy-controller launch description."""
    pkg = get_package_share_directory('big_bertha_policy_controller')
    default_model = os.path.join(pkg, 'models', 'policy.onnx')
    default_params = os.path.join(pkg, 'config', 'policy.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model_path = LaunchConfiguration('model_path')
    params_file = LaunchConfiguration('params_file')
    start_enabled = LaunchConfiguration('start_enabled')
    heading_lock = LaunchConfiguration('heading_lock')
    heading_lock_yaw = LaunchConfiguration('heading_lock_yaw')
    heading_hold = LaunchConfiguration('heading_hold')
    steer_kp = LaunchConfiguration('steer_kp')
    steer_max = LaunchConfiguration('steer_max')
    lateral_hold = LaunchConfiguration('lateral_hold')
    imu_topic = LaunchConfiguration('imu_topic')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('model_path', default_value=default_model),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('start_enabled', default_value='true'),
        # Absolute heading lock for the straight-line demo (off for nav,
        # where Nav2 owns the heading). heading_lock_yaw is the world
        # heading to hold.
        DeclareLaunchArgument('heading_lock', default_value='false'),
        DeclareLaunchArgument('heading_lock_yaw', default_value='0.0'),
        # Outer-loop compensation knobs, overridable for A/B characterization
        # against a given policy (the gains in policy.yaml were tuned against
        # older checkpoints' drift; defaults here match policy.yaml so normal
        # launches are unaffected unless explicitly overridden).
        DeclareLaunchArgument('heading_hold', default_value='true'),
        DeclareLaunchArgument('steer_kp', default_value='1.8'),
        DeclareLaunchArgument('steer_max', default_value='0.26'),
        DeclareLaunchArgument('lateral_hold', default_value='true'),
        # IMU topic the gait controller reads orientation from. On hardware the
        # BNO055 publishes a fused orientation on /imu directly; sim supplies
        # /imu with orientation too, so the default is shared.
        DeclareLaunchArgument('imu_topic', default_value='/imu'),

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
                    'heading_lock': ParameterValue(
                        heading_lock, value_type=bool),
                    'heading_lock_yaw': ParameterValue(
                        heading_lock_yaw, value_type=float),
                    'heading_hold': ParameterValue(
                        heading_hold, value_type=bool),
                    'steer_kp': ParameterValue(
                        steer_kp, value_type=float),
                    'steer_max': ParameterValue(
                        steer_max, value_type=float),
                    'lateral_hold': ParameterValue(
                        lateral_hold, value_type=bool),
                    'imu_topic': imu_topic,
                },
            ],
        ),
    ])
