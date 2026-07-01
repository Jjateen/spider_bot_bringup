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
                -> state_estimation -> {mapping | localization}
                -> perception -> planning

The stack reuses the simulation sub-launches from big_bertha_sim_bringup with
use_sim_time:=false. Only the bottom layer changes — real sensor drivers and a
servo bridge replace the Gazebo + ros_gz_bridge simulation layer.

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
        },
    )

    # ── 2. Hardware bridge (TCP -> STM32 -> servos + IMU) ─────────────
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

    # ── 3. Gait controller (/cmd_vel -> 12 joint targets) ─────────────
    locomotion = GroupAction([
        include(
            policy_pkg,
            os.path.join('launch', 'policy_controller.launch.py'),
            {'use_sim_time': use_sim_time},
        ),
    ], scoped=True)

    # ── 4. Leg odometry (commanded joints -> /odom) ──────────────────
    leg_odom = include(
        leg_pkg,
        os.path.join('launch', 'legged_odometry.launch.py'),
        {},
    )

    # ── 5. State estimation: robot_localization EKF ───────────────────
    state_estimation = include(
        sim_pkg,
        os.path.join('launch', 'state_estimation', 'ekf.launch.py'),
        {'use_sim_time': use_sim_time},
    )

    # ── 6. map->odom: SLAM (live mapping, default) or AMCL ───────────
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
        },
        condition=UnlessCondition(slam),
    )

    # ── 7. Perception: IMU-gated scan ground filter ──────────────────
    scan_filter = Node(
        package='big_bertha_sim_bringup',
        executable='scan_ground_filter',
        name='scan_ground_filter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 8. Planning: Nav2 servers ─────────────────────────────────────
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

        rsp,
        bridge,
        locomotion,
        leg_odom,
        state_estimation,
        mapping,
        localization,
        scan_filter,
        planning,
    ])
