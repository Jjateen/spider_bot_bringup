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
Planning bringup: Nav2 for Big Bertha.

Brings up the Nav2 servers (controller, planner, smoother, behaviors,
bt_navigator) parameterised by config/nav2_params.yaml and managed by a
nav2_lifecycle_manager (autostart). The lidar feeds the local + global
costmaps; Nav2 emits /cmd_vel to drive A->B around the obstacles.

The map->odom transform comes from one of two sources, selected by the parent
bringup launch via ``slam`` (false -> known-map mode via AMCL; true -> live
SLAM). This launch starts only the Nav2 servers; the chosen localization or
mapping stack plus the EKF + sim are launched alongside (see bringup.launch.py).

The behavior_tree XML paths are rewritten into the bt_navigator parameters so
Nav2 uses the trees shipped under config/behavior_trees/ in this package
(navigate_to_pose / navigate_through_poses, replanning + recovery).

Launch arguments
----------------
use_sim_time   Use /clock time (default: ``true``).
params_file    Nav2 parameter file (default: config/nav2_params.yaml).
autostart      Lifecycle-autostart the Nav2 servers (default: ``true``).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Build the Nav2 planning launch description."""
    pkg = get_package_share_directory('big_bertha_sim_bringup')
    default_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    bt_dir = os.path.join(pkg, 'config', 'behavior_trees')
    bt_to_pose = os.path.join(
        bt_dir, 'navigate_to_pose_w_replanning_and_recovery.xml')
    bt_through_poses = os.path.join(
        bt_dir, 'navigate_through_poses_w_replanning_and_recovery.xml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    nav_speed = LaunchConfiguration('nav_speed')

    # Point the bt_navigator at the in-package behavior trees and force the
    # use_sim_time value through every server (RewrittenYaml resolves these at
    # launch so the installed share path is absolute and correct).
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'default_nav_to_pose_bt_xml': bt_to_pose,
            'default_nav_through_poses_bt_xml': bt_through_poses,
            'desired_linear_vel': nav_speed,
        },
        convert_types=True,
    )

    # (node name, ROS package) for each Nav2 server. The lifecycle manager
    # transitions them in order. velocity_smoother / collision_monitor are
    # intentionally omitted so controller_server publishes /cmd_vel directly to
    # the policy controller (and the sim drive) without extra remaps.
    nav2_nodes = [
        ('controller_server', 'nav2_controller'),
        ('smoother_server', 'nav2_smoother'),
        ('planner_server', 'nav2_planner'),
        ('behavior_server', 'nav2_behaviors'),
        ('bt_navigator', 'nav2_bt_navigator'),
    ]

    server_nodes = [
        Node(
            package=pkg_name,
            executable=exe,
            name=exe,
            output='screen',
            respawn=False,
            # use_sim_time is also passed explicitly: RewrittenYaml's
            # param_rewrites only REPLACES keys that already exist in the yaml,
            # and nav2_params.yaml carries none, so the rewrite alone left every
            # server on the wall clock -> "Transform data too old (map->odom)"
            # and the controller never produced /cmd_vel.
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
        )
        for exe, pkg_name in nav2_nodes
    ]

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'bond_timeout': 0.0,
            'node_names': [exe for exe, _ in nav2_nodes],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('autostart', default_value='true'),
        # Overrides FollowPath desired_linear_vel. Mapping runs slower than the
        # 0.29 default: the coverage loop drives into the arena corners while
        # the map is still forming, where 0.29 tripped RPP's collision
        # projection and the progress checker (goal aborts, see
        # demo_slam_patrol.sh).
        DeclareLaunchArgument('nav_speed', default_value='0.29'),

        *server_nodes,
        lifecycle_manager,
    ])
