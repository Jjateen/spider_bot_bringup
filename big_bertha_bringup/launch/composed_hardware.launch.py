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
The hardware control chain in one process, with intra-process comms.

Loads hardware_bridge, leg_odometry, policy_controller and scan_ground_filter
into a single container. Three of the hot links then skip serialisation and
the loopback transport entirely:

    policy_controller -> /position_controller/commands -> hardware_bridge
    leg_odometry      -> /joint_states                 -> policy_controller
    leg_odometry      -> /odom                         -> policy_controller

imu_filter_madgwick stays a separate process for now. It is an upstream
package and its component registration has not been verified on the board, so
folding it in would risk a container that refuses to start. Doing so later
would additionally save the /imu and /filtered/imu hops.

The win here is CPU, not latency. Same-host Fast DDS already measures ~190 us
mean (PR #58), which is 1% of the 20 ms control period, so the loop was never
DDS-bound. What the A35 is short of is cores: serialising twelve-float arrays
at 50 Hz and IMU at 125 Hz costs real CPU, and callback starvation is what
forced imu_max_age wide in the first place.

MultiThreadedExecutor is deliberate. The policy runs ONNX inference inline in
its callback, and on a single-threaded container that would stall the bridge's
servo writes behind it. Nodes sharing this container must therefore be
thread-safe: leg_odometry takes pose_mutex_ for exactly this reason.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the composed hardware container."""
    bringup_pkg = get_package_share_directory('big_bertha_bringup')
    policy_pkg = get_package_share_directory('big_bertha_policy_controller')
    leg_pkg = get_package_share_directory('leg_odometry')

    use_sim_time = LaunchConfiguration('use_sim_time')
    imu_topic = LaunchConfiguration('imu_topic')
    start_enabled = LaunchConfiguration('start_enabled')

    bridge_params = os.path.join(bringup_pkg, 'config', 'hardware_bridge.yaml')
    policy_params = os.path.join(policy_pkg, 'config', 'policy.yaml')
    policy_model = os.path.join(policy_pkg, 'models', 'policy.onnx')
    leg_params = os.path.join(leg_pkg, 'config', 'legged_odometry.yaml')

    common = {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}

    nodes = [
        ComposableNode(
            package='big_bertha_bringup',
            plugin='big_bertha_bringup::HardwareBridgeNode',
            name='hardware_bridge',
            parameters=[bridge_params, common],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='leg_odometry',
            plugin='leg_odometry::LeggedOdometryNode',
            name='legged_odometry',
            parameters=[
                leg_params,
                common,
                {
                    'imu_topic': imu_topic,
                    'publish_joint_states': True,
                    # No EKF on hardware, so this node owns odom -> base_link.
                    'publish_tf': True,
                },
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='big_bertha_policy_controller',
            plugin='big_bertha_policy_controller::PolicyControllerNode',
            name='policy_controller',
            parameters=[
                policy_params,
                common,
                {
                    'model_path': policy_model,
                    'imu_topic': imu_topic,
                    'start_enabled': ParameterValue(start_enabled, value_type=bool),
                    # Same reason as big_bertha.launch.py: all three of these
                    # outer loops read pose off /odom, which on hardware is
                    # dead reckoning from a magnetometer-less IMU.
                    'heading_hold': False,
                    'position_hold': False,
                    'lateral_hold': False,
                },
            ],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ]

    nodes.append(
        # Its links (/scan in, /scan_filtered out) are both to external
        # processes, so it gains nothing from intra-process itself. It rides
        # along because one container is simpler than two and it costs nothing.
        ComposableNode(
            package='big_bertha_sim_bringup',
            plugin='big_bertha_sim_bringup::ScanGroundFilter',
            name='scan_ground_filter',
            parameters=[common, {'imu_topic': imu_topic, 'imu_max_age': 0.25}],
            extra_arguments=[{'use_intra_process_comms': True}],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'imu_topic', default_value='/filtered/imu',
            description='Orientation source (Madgwick output on hardware)'),
        DeclareLaunchArgument(
            'start_enabled', default_value='true',
            description='Policy starts armed'),
        ComposableNodeContainer(
            name='big_bertha_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=nodes,
            output='screen',
        ),
    ])
