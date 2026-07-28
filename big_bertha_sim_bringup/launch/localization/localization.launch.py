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
Localization bringup for Big Bertha known-map mode: map_server + map->odom.

Two map->odom providers, selected by ``localization``:

* ``amcl`` -- nav2_amcl localizes against the saved map from /scan and
  odom->base_link, publishing map->odom. Realistic; its historical failure
  here was OBSERVABILITY, not arena symmetry (the obstacle layout is unique):
  the low tilt-blind lidar often sees only one near wall, which constrains
  nothing along the wall direction. Seeded from the spawn args below.

* ``ground_truth`` (default for the A->B demo) -- a static identity map->odom.
  The gz OdometryPublisher already reports the WORLD pose and the EKF passes it
  through as odom->base_link, so map->base_link tracks the true world pose with
  a zero map->odom. Nav2 still plans and controls; it just gets a correct pose,
  isolating the locomotion demo from AMCL's arena ambiguity.

Both keep map_server: AMCL localizes against /map and the global costmap's
static layer plans on it.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the localization launch description."""
    pkg = get_package_share_directory('big_bertha_sim_bringup')
    default_amcl = os.path.join(pkg, 'config', 'amcl.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    amcl_config = LaunchConfiguration('amcl_config')
    map_yaml = LaunchConfiguration('map')
    mode = LaunchConfiguration('localization')

    use_amcl = IfCondition(PythonExpression(["'", mode, "' == 'amcl'"]))
    use_gt = IfCondition(PythonExpression(["'", mode, "' == 'ground_truth'"]))

    default_map = PathJoinSubstitution(
        [FindPackageShare('big_bertha_sim_bringup'),
         'maps', 'obstacle_world.yaml'])

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
        }],
    )

    # Seed AMCL from the spawn launch args (the yaml's hardcoded pose silently
    # diverged whenever the spawn changed; with recovery injection off that
    # was terminal).
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        condition=use_amcl,
        parameters=[
            amcl_config,
            {
                'use_sim_time': use_sim_time,
                'initial_pose.x': ParameterValue(
                    LaunchConfiguration('x'), value_type=float),
                'initial_pose.y': ParameterValue(
                    LaunchConfiguration('y'), value_type=float),
                'initial_pose.yaw': ParameterValue(
                    LaunchConfiguration('yaw'), value_type=float),
            },
        ],
    )

    # Ground-truth provider: static map->odom = IDENTITY. The gz
    # OdometryPublisher reports the robot's WORLD pose (the odom frame already
    # coincides with the world/map origin), and the EKF passes that through as
    # odom->base_link, so map->base_link is the true world pose with a zero
    # map->odom. (A spawn-pose offset here double-counts and puts the robot off
    # the map, which is what broke Nav2.) sx/sy/syaw are accepted but unused.
    static_map_odom = Node(
        package='big_bertha_sim_bringup',
        executable='map_to_odom_publisher',
        name='map_to_odom_ground_truth',
        output='screen',
        condition=use_gt,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # AMCL is lifecycle-managed; in ground-truth mode only map_server is.
    lifecycle_amcl = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        condition=use_amcl,
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
            'bond_timeout': 0.0,
        }],
    )
    lifecycle_gt = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        condition=use_gt,
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
            'bond_timeout': 0.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('amcl_config', default_value=default_amcl),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument(
            'localization', default_value='ground_truth',
            description="map->odom provider: 'ground_truth' (static, demo "
                        "default) or 'amcl' (scan-match localization)"),

        map_server,
        amcl,
        static_map_odom,
        lifecycle_amcl,
        lifecycle_gt,
    ])
