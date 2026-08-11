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
Launch the full Big Bertha autonomy stack on real hardware.

Data-flow order (see PLAN.md sec 3):

    description -> hardware_bridge -> locomotion -> leg_odometry
                -> {mapping | localization}
                -> perception -> planning

The stack reuses the simulation sub-launches from big_bertha_sim_bringup with
use_sim_time:=false. Only the bottom layer changes — real sensor drivers and a
servo bridge replace the Gazebo + ros_gz_bridge simulation layer. There is no
EKF: leg_odometry (publish_tf:=true) owns ``odom -> base_link`` directly.

Default mode is SLAM (live mapping) since there is no pre-saved map in the real
world. Set slam:=false for known-map (AMCL) mode with a saved map.

Launch arguments
----------------
slam           ``true`` SLAM mode (mapping), ``false`` known-map (localization).
               Default: ``true``.
use_sim_time   Use the ``/clock`` topic. Default: ``false`` (real hardware).
map            Saved map YAML for known-map mode.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the full hardware bringup launch description."""
    bringup_pkg = get_package_share_directory('big_bertha_bringup')
    desc_pkg = get_package_share_directory('big_bertha_description')
    policy_pkg = get_package_share_directory('big_bertha_policy_controller')
    leg_pkg = get_package_share_directory('leg_odometry')
    sim_pkg = get_package_share_directory('big_bertha_sim_bringup')

    slam = LaunchConfiguration('slam')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    imu_topic = LaunchConfiguration('imu_topic')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    def include(pkg_dir, rel_path, args, condition=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_dir, rel_path)),
            launch_arguments=args.items(),
            condition=condition,
        )

    default_map = PathJoinSubstitution(
        [FindPackageShare('big_bertha_sim_bringup'),
         'maps', 'obstacle_world.yaml'])

    # ── 1. Robot state publisher (URDF + tf, no Gazebo tags) ───────────
    rsp = include(
        desc_pkg,
        os.path.join('launch', 'rsp.launch.py'),
        {
            'use_sim_time': use_sim_time,
            'use_gz': 'false',
            'publish_jsp': 'false',
        },
    )

    # ── 2. Hardware bridge (router socket -> STM32 -> servos + IMU) ──
    bridge_params = os.path.join(bringup_pkg, 'config', 'hardware_bridge.yaml')
    bridge = Node(
        package='big_bertha_bringup',
        executable='hardware_bridge_node',
        name='hardware_bridge',
        output='screen',
        parameters=[
            bridge_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    # ── 2b. IMU filter: /imu (raw) -> /filtered/imu (with orientation) ──
    # The hardware bridge publishes raw accel+gyro with no orientation. This
    # node runs the Madgwick AHRS algorithm to estimate orientation from
    # gravity + gyro integration. No magnetometer (use_mag: false) so yaw
    # drifts (pure gyro, no magnetometer on this MPU-6500). The IMU yaw rate
    # is fused by the EKF; /odom yaw is not fused as absolute.
    imu_filter_config = os.path.join(bringup_pkg, 'config', 'imu_filter_madgwick.yaml')
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[imu_filter_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('imu/data_raw', '/imu'),
            ('imu/data', '/filtered/imu'),
        ],
    )

    # ── 3. Gait controller (/cmd_vel -> 12 joint targets) ─────────────
    locomotion = GroupAction([
        include(
            policy_pkg,
            os.path.join('launch', 'policy_controller.launch.py'),
            {'use_sim_time': use_sim_time, 'imu_topic': imu_topic},
        ),
    ], scoped=True)

    # ── 4. Leg odometry (commanded joints -> /odom) ──────────────────
    # leg_odometry owns /joint_states: its EWMA simulates the MG995 lag, which
    # is the policy's joint feedback on hardware (the bridge publishes raw
    # commands; feeding those back would close a positive-feedback loop).
    # publish_tf:=true makes leg_odometry own odom -> base_link too (there is
    # no EKF in the hardware bringup), so slam_toolbox/Nav2 get an odom frame.
    leg_odom = include(
        leg_pkg,
        os.path.join('launch', 'legged_odometry.launch.py'),
        {
            'imu_topic': imu_topic,
            'publish_joint_states': 'true',
            'publish_tf': 'true',
        },
    )

    # ── 5. map->odom: SLAM (live mapping, default) or AMCL ───────────
    mapping = include(
        sim_pkg,
        os.path.join('launch', 'mapping', 'slam.launch.py'),
        {'use_sim_time': use_sim_time},
        condition=IfCondition(slam),
    )
    localization = include(
        sim_pkg,
        os.path.join('launch', 'localization', 'localization.launch.py'),
        {
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'localization': 'amcl',
            'x': x,
            'y': y,
            'yaw': yaw,
        },
        condition=UnlessCondition(slam),
    )

    # ── 6. Perception: IMU-gated scan ground filter ──────────────────
    scan_filter = Node(
        package='big_bertha_sim_bringup',
        executable='scan_ground_filter',
        name='scan_ground_filter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'imu_topic': imu_topic,
            # Real IMU/scan stamps are ~90 ms apart; on the loaded UNO Q the SBC
            # starves callbacks and they spike to ~0.3-0.35 s, so a tight bound
            # makes the filter pass scans through unfiltered. 0.45 covers the
            # load spikes while staying under the ~0.7 s gait period (body tilt
            # is small, so a slightly old attitude still culls floor correctly).
            'imu_max_age': 0.45,
        }],
    )

    # ── 7. Planning: Nav2 servers ─────────────────────────────────────
    planning = include(
        sim_pkg,
        os.path.join('launch', 'planning', 'nav2.launch.py'),
        {'use_sim_time': use_sim_time},
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='true: SLAM (live mapping); false: known-map (AMCL)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock time (false for real hardware)'),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Saved map YAML for known-map mode'),
        DeclareLaunchArgument(
            'imu_topic', default_value='/filtered/imu',
            description='Orientation source for hardware consumers '
                        '(filtered Imu from imu_filter_madgwick)'),
        DeclareLaunchArgument(
            'x', default_value='0.0',
            description='AMCL initial pose x (known-map mode)'),
        DeclareLaunchArgument(
            'y', default_value='0.0',
            description='AMCL initial pose y (known-map mode)'),
        DeclareLaunchArgument(
            'yaw', default_value='0.0',
            description='AMCL initial pose yaw (known-map mode)'),

        rsp,
        bridge,
        imu_filter,
        locomotion,
        leg_odom,
        mapping,
        localization,
        scan_filter,
        planning,
    ])
