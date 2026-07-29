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
RViz visualization launch for Big Bertha.

Opens RViz2 with one of the in-package configs under ``config/rviz/``:

* ``simulation``  robot model + TF + laser scan + odometry (odom frame).
* ``mapping``     live ``/map`` + SLAM pose graph + scan (map frame).
* ``planning``    costmaps + global/local plan + goal + footprint (map frame).
* ``integration`` combined view for the full A->B demo.

Launch arguments
----------------
config         Config basename in ``config/rviz/`` without extension
               (default: ``integration``).
use_sim_time   Use the ``/clock`` topic (default: ``true``).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the RViz launch description."""
    config = LaunchConfiguration('config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Resolve config/rviz/<config>.rviz from the installed package share.
    rviz_config = PathJoinSubstitution([
        FindPackageShare('big_bertha_sim_bringup'),
        'config', 'rviz',
        [config, TextSubstitution(text='.rviz')],
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value='integration',
            description='RViz config basename in config/rviz/ (no extension)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use /clock time'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
