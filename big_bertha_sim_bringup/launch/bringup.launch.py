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
               verification aid for the mapping/localization/planning gates,
               independent of the gait). Default: ``false``.
map            Saved map YAML for known-map mode. Default: the bundled
               ``maps/obstacle_world.yaml``.
x, y, z, yaw   Spawn pose (demo point A). Defaults: ``-3.5 -3.5 0.12 0.0``
               (facing +x East).
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
    rviz_enabled = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
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
        {'use_sim_time': use_sim_time, 'map': map_yaml,
         'localization': LaunchConfiguration('localization'),
         'x': x, 'y': y, 'yaw': yaw},
        condition=UnlessCondition(slam),
    )

    # perception: IMU-gated ghost-wall filter. The body lidar tilts with the
    # gait; this drops floor hits so the costmap stops seeing flickering walls.
    # Costmaps consume /scan_filtered (see nav2_params.yaml).
    scan_filter = Node(
        package='big_bertha_sim_bringup',
        executable='scan_ground_filter',
        name='scan_ground_filter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # planning: Nav2 servers (planner/controller/costmaps/BT).
    planning = include(
        os.path.join('planning', 'nav2.launch.py'),
        {'use_sim_time': use_sim_time,
         'nav_speed': LaunchConfiguration('nav_speed')},
    )

    # Optional RViz view (the visualization module's rviz.launch.py picks the
    # config under config/rviz/; defaults to the combined integration view).
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'visualization', 'rviz.launch.py')),
        launch_arguments={
            'config': rviz_config,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(rviz_enabled),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam', default_value='false',
            description='true: SLAM (mapping); false: known-map (localization)'),
        DeclareLaunchArgument(
            'localization', default_value='ground_truth',
            description="known-map map->odom provider: 'ground_truth' (static "
                        "identity, honest pose, default) or 'amcl' "
                        '(scan-match; observability-limited by the low lidar, '
                        'see localization.launch.py)'),
        DeclareLaunchArgument(
            'nav_speed', default_value='0.29',
            description='Nav2 FollowPath desired_linear_vel (m/s). 0.29 '
                        'matches the trained demo speed; mapping passes a '
                        'lower value (see nav2.launch.py)'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Also launch RViz'),
        DeclareLaunchArgument(
            'rviz_config', default_value='integration',
            description='RViz config in config/rviz/ (simulation|mapping|'
                        'planning|integration)'),
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
        # Face +x (East): the A->B demo traverses the obstacle-free bottom
        # corridor (y=-3.5, all obstacles sit at y>=-1.8) as a straight shot,
        # which the drift-cancelled gait can hold. (The diagonal toward (3.5,3.5)
        # is walled off by box_1/pillar_1/box_2 and needs hard turns the gait
        # cannot make.) Must match the AMCL seed in amcl.yaml.
        DeclareLaunchArgument('yaw', default_value='0.0'),

        simulation,
        locomotion,
        state_estimation,
        scan_filter,
        mapping,
        localization,
        planning,
        rviz,
    ])
