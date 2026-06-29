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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

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
    rt_priority = LaunchConfiguration('rt_priority')
    dds_shm = LaunchConfiguration('dds_shm')
    cyclonedds_shm_uri = LaunchConfiguration('cyclonedds_shm_uri')

    # This node only ever subscribes continuous streams (/odom, /imu,
    # /joint_states, /cmd_vel) -- never a one-shot TRANSIENT_LOCAL topic -- so
    # it's safe to always run with SHM on when dds_shm:=true (see
    # simulation.launch.py for why robot_state_publisher/spawn_robot can't).
    # cyclonedds_shm_uri is supplied by the caller (it owns the path to its
    # own dds config) rather than looked up here, to avoid a reverse package
    # dependency on big_bertha_sim_bringup.
    shm_on = SetEnvironmentVariable(
        'CYCLONEDDS_URI', cyclonedds_shm_uri, condition=IfCondition(dds_shm))

    # '' (rt_priority=0, default) -> no prefix, runs at normal SCHED_OTHER
    # priority. Opt-in only: SCHED_FIFO needs CAP_SYS_NICE or an rtprio ulimit,
    # which most dev/CI shells don't have -- this must not break the default
    # launch path on a host without it.
    rt_prefix = PythonExpression([
        "'chrt -f ' + '", rt_priority, "' if '", rt_priority, "' != '0' else ''",
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('model_path', default_value=default_model),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('start_enabled', default_value='true'),
        # Absolute heading lock for the straight-line demo (off for nav, where
        # Nav2 owns the heading). heading_lock_yaw is the world heading to hold.
        DeclareLaunchArgument('heading_lock', default_value='false'),
        DeclareLaunchArgument('heading_lock_yaw', default_value='0.0'),
        # SCHED_FIFO priority (1-99) for the policy node, via chrt. Protects the
        # 50Hz gait loop from being starved by SLAM/Nav2/perception on a weak,
        # CPU-constrained target (the UNO Q's 4x Cortex-A55) under contention --
        # not needed on a dev machine with idle cores. 0 = disabled (default).
        DeclareLaunchArgument('rt_priority', default_value='0'),
        # Per-process Cyclone DDS shared-memory opt-in (see comment above).
        # No-op unless dds_shm:=true with a real cyclonedds_shm_uri supplied.
        DeclareLaunchArgument('dds_shm', default_value='false'),
        DeclareLaunchArgument('cyclonedds_shm_uri', default_value=''),

        shm_on,
        Node(
            package='big_bertha_policy_controller',
            executable='policy_controller_node',
            name='policy_controller',
            output='screen',
            prefix=rt_prefix,
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    'model_path': model_path,
                    'start_enabled': start_enabled,
                    'heading_lock': ParameterValue(heading_lock, value_type=bool),
                    'heading_lock_yaw': ParameterValue(heading_lock_yaw, value_type=float),
                },
            ],
        ),
    ])
