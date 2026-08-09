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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('leg_odometry')
    default_params = os.path.join(pkg, 'config', 'legged_odometry.yaml')
    imu_topic = LaunchConfiguration('imu_topic')
    publish_joint_states = LaunchConfiguration('publish_joint_states')
    publish_tf = LaunchConfiguration('publish_tf')

    return LaunchDescription([
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('publish_joint_states', default_value='true'),
        # Publish odom -> base_link from the dead-reckoned /odom. Off by default
        # (the EKF owns that transform in sim); the hardware bringup turns it on
        # so SLAM/Nav2 get an odom frame without the EKF.
        DeclareLaunchArgument('publish_tf', default_value='false'),
        Node(
            package='leg_odometry',
            executable='legged_odometry_node',
            name='legged_odometry',
            output='screen',
            parameters=[
                default_params,
                {
                    'imu_topic': imu_topic,
                    'publish_joint_states': publish_joint_states,
                    'publish_tf': ParameterValue(publish_tf, value_type=bool),
                },
            ],
        ),
    ])
