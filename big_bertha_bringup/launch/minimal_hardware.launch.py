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
Minimal hardware bringup for Big Bertha on the bench.

Launches the four pieces needed to close the locomotion loop on real hardware,
nothing more (no EKF, Nav2, or scan-ground filter):

    rsp.launch.py            -> /tf (robot_state_publisher, no Gazebo)
    hardware_bringup.launch -> hardware_bridge_node (servos + raw /imu) and
                               imu_filter_madgwick (/filtered/imu, orientation)
    legged_odometry         -> /joint_states (EWMA servo feedback) + /odom
    policy_controller       -> /position_controller/commands (12 joint targets)

The policy expects an orientation-bearing IMU, so imu_topic defaults to the
Madgwick output (/filtered/imu); override to /imu for raw-only bringup.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Build the minimal hardware bringup."""
    bringup_pkg = get_package_share_directory('big_bertha_bringup')
    desc_pkg = get_package_share_directory('big_bertha_description')
    policy_pkg = get_package_share_directory('big_bertha_policy_controller')
    leg_pkg = get_package_share_directory('leg_odometry')

    use_sim_time = LaunchConfiguration('use_sim_time')
    imu_topic = LaunchConfiguration('imu_topic')
    start_enabled = LaunchConfiguration('start_enabled')

    def include(pkg_dir, rel_path, args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_dir, rel_path)),
            launch_arguments=args.items(),
        )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'imu_topic', default_value='/filtered/imu',
            description='IMU orientation source for policy + leg_odometry'),
        DeclareLaunchArgument(
            'start_enabled', default_value='true',
            description='Policy starts armed; 3 s warmup holds the stance pose'),

        # 1. tf tree (URDF) — no Gazebo, no gz odom/tf, no joint_state_publisher
        include(desc_pkg, os.path.join('launch', 'rsp.launch.py'), {
            'use_sim_time': use_sim_time,
            'use_gz': 'false',
            'publish_odom': 'false',
            'odom_tf': 'false',
            'publish_jsp': 'false',
            'sim_drive': 'false',
        }),
        # 2. Bridge (servos + raw /imu) + Madgwick filter (/filtered/imu).
        include(bringup_pkg, os.path.join('launch', 'hardware_bringup.launch.py'), {
            'use_sim_time': use_sim_time,
        }),
        # 3. Joint feedback (EWMA MG995 model) + dead-reckon /odom.
        include(leg_pkg, os.path.join('launch', 'legged_odometry.launch.py'), {
            'imu_topic': imu_topic,
            'publish_joint_states': 'true',
        }),
        # 4. Gait /position_controller/commands -> bridge + leg_odometry.
        include(policy_pkg, os.path.join('launch', 'policy_controller.launch.py'), {
            'use_sim_time': use_sim_time,
            'imu_topic': imu_topic,
            'start_enabled': start_enabled,
        }),
    ])
