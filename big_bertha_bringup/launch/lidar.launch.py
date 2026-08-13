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
YDLidar X2 driver.

This used to be a manual step outside every launch file, which meant the
documented bringup sequence could not actually be run as one command. It is
here now so `composed_stack.launch.py` brings up the whole pipeline.

The vendored driver (deps/ydlidar_ros2_driver) is a plain rclcpp::Node, NOT a
lifecycle node: its main() calls laser.initialize() + laser.turnOn() at
startup and publishes /scan unconditionally. So it must be launched as a plain
Node, and there is no nav2_lifecycle_manager to activate it -- the earlier
LifecycleNode treatment made lifecycle_manager_lidar spin forever on
"Waiting for service ydlidar_ros2_driver_node/get_state..." (the driver never
provides that service) while the driver was in fact already scanning.

The frame_id override lives in config/ydlidar_x2.yaml, not here. See that file
for why the driver default is actively harmful on this robot.

The driver is NOT composed into a container. It owns a serial port and does
its own blocking reads, so it gains nothing from intra-process comms (its only
output, /scan, goes to a different container) while a hang in it would take
the control loop down with it.

respawn is on: the driver exits cleanly whenever /dev/ttyLIDAR is absent (the
lidar's USB rail blips and the device re-enumerates), and launch relaunches it
so /scan comes back once the port reappears. Paired with auto_reconnect in
config/ydlidar_x2.yaml and the udev symlink, a power blip no longer kills the
lidar for the rest of the session.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Bring the X2 up and let it publish /scan."""
    pkg = get_package_share_directory('big_bertha_bringup')
    default_params = os.path.join(pkg, 'config', 'ydlidar_x2.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=default_params),

        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            namespace='',
            output='screen',
            respawn=True,
            respawn_delay=3.0,
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
    ])
