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
Robot State Publisher launch for Big Bertha.

Expands ``urdf/big_bertha.urdf.xacro`` and feeds it to
``robot_state_publisher`` so the full tf tree (12 leg joints + ``lidar_link``
+ ``imu_link``) is published. Other modules (simulation, locomotion, ...)
include this launch file.

Launch arguments
----------------
use_sim_time
    Use the ``/clock`` topic (default: ``true``).
use_gz
    Emit ``gz_ros2_control`` + gz sensor tags in the URDF (default: ``true``;
    set ``false`` for a plain rsp / RViz check).
publish_jsp
    Also run ``joint_state_publisher`` so tf is complete without a controller
    or simulator (default: ``false``; the sim and the policy controller
    publish ``/joint_states`` instead).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the robot_state_publisher launch description."""
    pkg_share = get_package_share_directory('big_bertha_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'big_bertha.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gz = LaunchConfiguration('use_gz')
    publish_jsp = LaunchConfiguration('publish_jsp')

    ros2_control_config = PathJoinSubstitution(
        [FindPackageShare('big_bertha_description'),
         'config', 'ros2_control.yaml']
    )

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_gz:=', use_gz,
            ' ros2_control_config:=', ros2_control_config,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (/clock) time'),
        DeclareLaunchArgument(
            'use_gz', default_value='true',
            description='Emit gz_ros2_control + gz sensor tags in the URDF'),
        DeclareLaunchArgument(
            'publish_jsp', default_value='false',
            description='Also run joint_state_publisher (no sim/controller)'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(publish_jsp),
        ),
    ])
