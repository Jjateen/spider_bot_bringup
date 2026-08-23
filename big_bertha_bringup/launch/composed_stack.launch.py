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
The whole hardware stack, one command, composed.

    ros2 launch big_bertha_bringup composed_stack.launch.py

That is the point of this file. The previous bringup could not be started in
one step: the lidar was documented as a separate manual command, so anyone
following DEPLOYMENT.md ended up with a stack that looked healthy and had no
/scan.

    description        robot_state_publisher (URDF -> /tf_static)
    lidar              YDLidar X2, driven to active
    control container  bridge + madgwick + leg_odometry + policy + scan filter
    nav container(s)   slam (or map_server + amcl) + the five Nav2 servers

Two containers, not one, and no EKF. See composed_hardware.launch.py for why
the control chain gets its own executor, composed_navigation.launch.py for why
intra-process is off on the nav side, and big_bertha.launch.py for why
robot_localization is gone on hardware (it was fusing /odom with the IMU that
/odom is derived from).

x/y/yaw are the robot's starting pose in the map frame. They seed
slam_toolbox's map_start_pose in SLAM mode and AMCL's initial_pose in
known-map mode, so the default of 0,0,0 means "the map origin is wherever the
robot is right now". Do not leave these at the Gazebo spawn pose.

DDS: set the profile before launching, or discovery will not cross to a dev
machine for RViz.

    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=file://$(ros2 pkg prefix big_bertha_bringup)\
/share/big_bertha_bringup/config/dds/cyclonedds.xml

The unicast peer list in that file is what makes cross-machine topics work;
multicast reaches nothing here (AP client isolation on wlan0, and Tailscale
carries no multicast at all).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Build the full composed hardware stack."""
    bringup_pkg = get_package_share_directory('big_bertha_bringup')
    desc_pkg = get_package_share_directory('big_bertha_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    imu_topic = LaunchConfiguration('imu_topic')
    start_enabled = LaunchConfiguration('start_enabled')
    slam = LaunchConfiguration('slam')
    map_yaml = LaunchConfiguration('map')
    nav_speed = LaunchConfiguration('nav_speed')
    with_lidar = LaunchConfiguration('with_lidar')
    with_nav = LaunchConfiguration('with_nav')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    def include(pkg_dir, rel, args, condition=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_dir, rel)),
            launch_arguments=args.items(),
            condition=condition,
        )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'imu_topic', default_value='/filtered/imu',
            description='Orientation source; Madgwick output on hardware'),
        DeclareLaunchArgument('start_enabled', default_value='true'),
        # Default OFF on hardware: mapping waits for scripts/start_mapping.sh,
        # which the operator runs once they are back from the servo buck switch
        # and clear of the lidar. See composed_navigation.launch.py.
        DeclareLaunchArgument(
            'mapping_autostart', default_value='false',
            description='true: map from launch; false: wait for '
                        'scripts/start_mapping.sh so the operator can step '
                        'clear before the first scan is frozen into the map'),
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='true = live mapping, false = known map via AMCL'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('nav_speed', default_value='0.29'),
        DeclareLaunchArgument(
            'with_lidar', default_value='true',
            description='false to run the control loop with no lidar attached'),
        DeclareLaunchArgument(
            'with_nav', default_value='true',
            description='false for bench work: control loop only, no Nav2'),
        DeclareLaunchArgument(
            'x', default_value='0.0',
            description='Start pose in the map frame; seeds slam and AMCL'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        include(desc_pkg, os.path.join('launch', 'rsp.launch.py'), {
            'use_sim_time': use_sim_time,
            'use_gz': 'false',
            'publish_odom': 'false',
            'odom_tf': 'false',
            'publish_jsp': 'false',
            'sim_drive': 'false',
        }),
        include(bringup_pkg, os.path.join('launch', 'lidar.launch.py'), {
            'use_sim_time': use_sim_time,
        }, condition=IfCondition(with_lidar)),
        include(
            bringup_pkg,
            os.path.join('launch', 'composed_hardware.launch.py'),
            {
                'use_sim_time': use_sim_time,
                'imu_topic': imu_topic,
                'start_enabled': start_enabled,
            },
        ),
        include(
            bringup_pkg,
            os.path.join('launch', 'composed_navigation.launch.py'),
            {
                'use_sim_time': use_sim_time,
                'slam': slam,
                'map': map_yaml,
                'nav_speed': nav_speed,
                'mapping_autostart': LaunchConfiguration('mapping_autostart'),
                'x': x, 'y': y, 'yaw': yaw,
            },
            condition=IfCondition(with_nav),
        ),
    ])
