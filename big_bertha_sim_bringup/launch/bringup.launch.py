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
One-shot Big Bertha sim bringup: chains every functional module.

Data-flow order (see PLAN.md sec 3):

    description -> simulation -> locomotion -> state_estimation
                -> {mapping | localization} -> planning [-> rviz]

The simulation launch already includes ``description`` (robot_state_publisher),
so it is the entry point. The EKF owns the ``odom -> base_link`` transform, so
the sim is started with ``odom_tf:=false``. The ``map -> odom`` transform comes
from exactly one of two sources, selected by the ``slam`` argument:

* ``slam:=true``  -> mapping (slam_toolbox) builds the map live (SLAM mode).
* ``slam:=false`` -> localization (AMCL + map_server) against the saved map
                     (known-map mode, the default for the repeatable A->B demo).

Launch arguments
----------------
slam           ``true`` SLAM mode (mapping), ``false`` known-map (localization).
               Default: ``false``.
rviz           Also launch RViz with the integration view. Default: ``false``.
use_sim_time   Use the ``/clock`` topic. Default: ``true``.
gui            Run the Gazebo GUI client. Default: ``false`` (headless).
world          World SDF basename in ``worlds/``. Default: ``obstacle_world.sdf``.
sim_drive      Enable the sim-only kinematic gz VelocityControl drive (a
               verification aid; the learned gait does not transfer to Gazebo,
               see issue #5). Default: ``false``.
map            Saved map YAML for known-map mode. Default: the bundled
               ``maps/obstacle_world.yaml``.
x, y, z, yaw   Spawn pose (demo point A). Defaults: ``-3.5 -3.5 0.12 0.785``.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the full sim bringup launch description."""
    sim_pkg = get_package_share_directory('big_bertha_sim_bringup')
    policy_pkg = get_package_share_directory('big_bertha_policy_controller')
    launch_dir = os.path.join(sim_pkg, 'launch')

    slam = LaunchConfiguration('slam')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    sim_drive = LaunchConfiguration('sim_drive')
    map_yaml = LaunchConfiguration('map')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    default_map = PathJoinSubstitution(
        [FindPackageShare('big_bertha_sim_bringup'),
         'maps', 'obstacle_world.yaml'])
    default_rviz = os.path.join(sim_pkg, 'config', 'rviz', 'integration.rviz')

    def include(rel_path, args, condition=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, rel_path)),
            launch_arguments=args.items(),
            condition=condition,
        )

    # description + simulation (rsp is included by the sim launch). The EKF
    # owns odom->base_link, so the gz odom tf is disabled (odom_tf:=false).
    simulation = include(
        os.path.join('simulation', 'simulation.launch.py'),
        {
            'use_sim_time': use_sim_time,
            'gui': gui,
            'world': world,
            'odom_tf': 'false',
            'sim_drive': sim_drive,
            'spawn_controllers': 'true',
            'x': x, 'y': y, 'z': z, 'yaw': yaw,
        },
    )

    # locomotion: the C++ ONNX gait controller (/cmd_vel -> joint targets).
    # Scoped so its ``params_file:=policy.yaml`` default stays local (see the
    # leak note in include()).
    locomotion = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    policy_pkg, 'launch', 'policy_controller.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ], scoped=True)

    # state_estimation: robot_localization EKF (fuses /odom + /imu).
    state_estimation = include(
        os.path.join('state_estimation', 'ekf.launch.py'),
        {'use_sim_time': use_sim_time},
    )

    # map->odom: SLAM (mapping) OR known-map (localization), never both.
    mapping = include(
        os.path.join('mapping', 'slam.launch.py'),
        {'use_sim_time': use_sim_time},
        condition=IfCondition(slam),
    )
    localization = include(
        os.path.join('localization', 'localization.launch.py'),
        {'use_sim_time': use_sim_time, 'map': map_yaml},
        condition=UnlessCondition(slam),
    )

    # planning: Nav2 servers (planner/controller/costmaps/BT).
    planning = include(
        os.path.join('planning', 'nav2.launch.py'),
        {'use_sim_time': use_sim_time},
    )

    # Optional RViz integration view.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam', default_value='false',
            description='true: SLAM (mapping); false: known-map (localization)'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Also launch RViz with the integration view'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use /clock time'),
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Run the Gazebo GUI client'),
        DeclareLaunchArgument(
            'world', default_value='obstacle_world.sdf',
            description='World SDF basename in worlds/'),
        DeclareLaunchArgument(
            'sim_drive', default_value='false',
            description='Enable the sim-only gz VelocityControl drive'),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Saved map YAML for known-map mode'),
        DeclareLaunchArgument('x', default_value='-3.5'),
        DeclareLaunchArgument('y', default_value='-3.5'),
        DeclareLaunchArgument('z', default_value='0.12'),
        DeclareLaunchArgument('yaw', default_value='0.785'),

        simulation,
        locomotion,
        state_estimation,
        mapping,
        localization,
        planning,
        rviz_node,
    ])
