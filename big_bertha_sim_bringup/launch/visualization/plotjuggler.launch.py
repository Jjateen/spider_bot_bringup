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
PlotJuggler visualization launch for Big Bertha.

Opens PlotJuggler with one of the in-package layouts under
``config/plotjuggler/`` and the DataStreamROS2 plugin already selected:

* ``sensors``  /imu, /joint_states, /odom raw feeds.
* ``control``  /cmd_vel vs /odom command tracking + spider_msgs/PolicyStatus.

Equivalent to::

    ros2 run plotjuggler plotjuggler -n -l <layout.xml>

(``-n`` skips the splash; ``-l`` loads the layout). The layout's ROS2 streamer
subscribes live once the sim + policy controller publish the topics.

Launch arguments
----------------
layout         Layout basename in ``config/plotjuggler/`` without extension
               (default: ``control``).
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
    """Build the PlotJuggler launch description."""
    layout = LaunchConfiguration('layout')

    layout_file = PathJoinSubstitution([
        FindPackageShare('big_bertha_sim_bringup'),
        'config', 'plotjuggler',
        [layout, TextSubstitution(text='.xml')],
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'layout', default_value='control',
            description='Layout basename in config/plotjuggler/ (no extension)'),

        Node(
            package='plotjuggler',
            executable='plotjuggler',
            name='plotjuggler',
            output='screen',
            arguments=['-n', '-l', layout_file],
        ),
    ])
