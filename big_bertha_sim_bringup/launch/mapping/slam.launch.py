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
from typing import List

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the slam_toolbox launch description."""
    pkg = get_package_share_directory('big_bertha_sim_bringup')
    default_slam = os.path.join(pkg, 'config', 'slam_toolbox.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_config = LaunchConfiguration('slam_config')

    # start_delay: hold mapping off for N seconds after launch.
    #
    # slam_toolbox only integrates a scan once the robot has moved
    # minimum_travel_distance (0.1 m) or turned minimum_travel_heading
    # (0.1 rad). A robot standing still therefore NEVER updates its map: the
    # very first scan is frozen in for good. On hardware someone has to walk
    # over and flip the servo buck-converter switch, so that person is stood
    # next to the robot when the first scan lands, gets mapped as an obstacle
    # overlapping the footprint, and Nav2 reports a collision before it will
    # plan anything. Waiting does not clear it, which is what was observed.
    # Delaying the first scan past the power-up ritual is the fix;
    # big_bertha_bringup/scripts/reset_map.sh recovers a map that already
    # caught someone.
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {
                'use_sim_time': use_sim_time,
                # Where the robot sits in the map frame at startup. The yaml
                # default is the Gazebo spawn pose, which is only correct for
                # the simulator. On hardware the robot starts wherever you put
                # it, and inheriting -3.5,-3.5 built the entire map that far
                # from the real start. Callers pass their own via x/y/yaw.
                'map_start_pose': ParameterValue(
                    PythonExpression(
                        ['[', LaunchConfiguration('x'), ', ',
                         LaunchConfiguration('y'), ', ',
                         LaunchConfiguration('yaw'), ']'],
                    ),
                    value_type=List[float],
                ),
            },
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

    delayed_slam = TimerAction(
        period=LaunchConfiguration('start_delay'),
        actions=[slam_node],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'start_delay', default_value='0.0',
            description='seconds to hold mapping off so the operator can step '
                        'clear before the first scan is frozen into the map'),
        DeclareLaunchArgument('slam_config', default_value=default_slam),
        # Defaults are the Gazebo spawn pose, so the sim bringup is unchanged.
        # big_bertha.launch.py passes its own (0,0,0 by default) on hardware.
        DeclareLaunchArgument('x', default_value='-3.5'),
        DeclareLaunchArgument('y', default_value='-3.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        delayed_slam,
        lifecycle_manager,
    ])
