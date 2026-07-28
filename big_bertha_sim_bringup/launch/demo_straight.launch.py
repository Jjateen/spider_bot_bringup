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
Straight-line locomotion demo: gait only, no Nav2 / AMCL / map.

This is the minimal "does the robot actually crawl?" bringup. It deliberately
strips out everything in the navigation/localization stack so the only things
running are the physics sim, the learned-gait policy controller, and a fixed
forward ``/cmd_vel``. It exists to isolate locomotion from the Nav2 layer:

* No AMCL  -> nothing rewrites ``map -> odom``, so the robot cannot "teleport"
  in RViz the way it does when AMCL re-localizes against a symmetric/garbage
  scan. The fixed frame here is ``odom``.
* No Nav2  -> nothing commands ``vx = 0`` on a phantom "collision ahead",
  so the policy's stand-still gate never freezes the legs mid-traverse.
* Ground-truth odometry tf -- the gz ``OdometryPublisher`` publishes
  ``odom -> base_link`` directly (``odom_tf:=true``), so the EKF is not needed
  and is left out (one less moving part for the locomotion check).

The robot spawns at the clear bottom corridor A = (-3.5, -3.5) facing +x (East)
and is driven straight East at a constant ``speed`` for the whole run (with an
optional ``vy`` lateral feedforward to trim the DART crab). Use this to confirm
the gait crawls straight before re-enabling turning / Nav2.

    ros2 launch big_bertha_sim_bringup demo_straight.launch.py
    ros2 launch big_bertha_sim_bringup demo_straight.launch.py gui:=true
    ros2 launch big_bertha_sim_bringup demo_straight.launch.py speed:=0.10
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the straight-line locomotion demo launch description."""
    sim_pkg = get_package_share_directory('big_bertha_sim_bringup')
    policy_pkg = get_package_share_directory('big_bertha_policy_controller')
    launch_dir = os.path.join(sim_pkg, 'launch')

    gui = LaunchConfiguration('gui')
    rviz_enabled = LaunchConfiguration('rviz')
    show_map = LaunchConfiguration('show_map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    speed = LaunchConfiguration('speed')
    vy = LaunchConfiguration('vy')
    hip_bias = LaunchConfiguration('hip_bias')
    cmd_delay = LaunchConfiguration('cmd_delay')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    heading_hold = LaunchConfiguration('heading_hold')
    steer_kp = LaunchConfiguration('steer_kp')
    steer_max = LaunchConfiguration('steer_max')
    lateral_hold = LaunchConfiguration('lateral_hold')

    # simulation (rsp + gz + spawn + ros2_control). Ground-truth odom tf
    # (odom_tf:=true) means gz publishes odom->base_link directly, so no EKF is
    # required to place the robot under the odom frame.
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'simulation', 'simulation.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui,
            'odom_tf': 'true',
            'sim_drive': 'false',
            'spawn_controllers': 'true',
            'x': x, 'y': y, 'z': z, 'yaw': yaw,
        }.items(),
    )

    # locomotion: the C++ ONNX gait controller (/cmd_vel -> joint targets).
    # Scoped so the policy launch's params_file default stays local.
    locomotion = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    policy_pkg, 'launch', 'policy_controller.launch.py')),
            # Lock the heading to the spawn yaw (default 0 = due East) so
            # the gait holds a true straight line instead of the ~14 deg
            # south-of-East heading the robot settles into during the DART
            # warmup drop.
            launch_arguments={
                'use_sim_time': use_sim_time,
                'heading_lock': LaunchConfiguration('heading_lock'),
                'heading_lock_yaw': LaunchConfiguration('yaw'),
                'heading_hold': heading_hold,
                'steer_kp': steer_kp,
                'steer_max': steer_max,
                'lateral_hold': lateral_hold,
            }.items(),
        ),
    ], scoped=True)

    # Static known map (the obstacle world's walls) so RViz shows the arena.
    # demo_straight runs on ground-truth odom (odom == world), and the saved
    # map is world-aligned, so map -> odom is identity. No AMCL/SLAM -- this is
    # purely the static map for visualization (and a sanity reference for how
    # far the gait drifts toward the real walls). Gated by show_map.
    map_yaml = PathJoinSubstitution([
        FindPackageShare('big_bertha_sim_bringup'),
        'maps', 'obstacle_world.yaml'])
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'frame_id': 'map',
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(show_map),
    )
    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }],
        condition=IfCondition(show_map),
    )
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static',
        output='screen',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        condition=IfCondition(show_map),
    )

    # RViz: robot + tf + scan + odometry + the static map walls (odom frame).
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'visualization', 'rviz.launch.py')),
        launch_arguments={
            'config': 'straight',
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(rviz_enabled),
    )

    # Constant forward /cmd_vel (+ optional lateral vy). The policy node times
    # out a stale command in 0.5 s, so publish continuously at 20 Hz. Held off
    # cmd_delay seconds so the controllers have loaded (gz_ros2_control spawns
    # ~6-8 s after gz starts).
    drive_straight = TimerAction(
        period=cmd_delay,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '-r', '20', '/cmd_vel',
                    'geometry_msgs/msg/Twist',
                    ['{linear: {x: ', speed, ', y: ', vy, ', z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'],
                ],
                output='screen',
            ),
        ],
    )

    # Constant drift-cancel hip bias on /debug_hip_bias; deployment trim only,
    # re-tune (or 0.0) per deployed policy.
    drift_trim = TimerAction(
        period=cmd_delay,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '-r', '20', '/debug_hip_bias',
                    'std_msgs/msg/Float64MultiArray',
                    ['{data: [', hip_bias, ', ', hip_bias, ', ',
                     hip_bias, ', ', hip_bias, ']}'],
                ],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Run the Gazebo GUI client (default headless)'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz in the odom frame'),
        DeclareLaunchArgument(
            'show_map', default_value='true',
            description='Publish the static obstacle-world map so RViz '
                        'renders the arena walls (map==odom, identity tf)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use /clock time'),
        DeclareLaunchArgument(
            'speed', default_value='0.29',
            description='Constant forward vx (m/s); trained range is '
                        '0-0.40, 0.29 here matches the training seq gif '
                        'so sim and Isaac show the same gait, not a '
                        'range bound'),
        DeclareLaunchArgument(
            'vy', default_value='0.0',
            description='Constant lateral vy (m/s, +left); closed-loop '
                        'lateral_hold handles the crab instead'),
        DeclareLaunchArgument(
            'hip_bias', default_value='0.0',
            description='Drift-cancel hip bias (rad) on /debug_hip_bias; '
                        '0.0 for the current policy (calibration knob)'),
        DeclareLaunchArgument(
            'cmd_delay', default_value='12.0',
            description='Seconds to wait for controllers before driving'),
        # Outer-loop compensation knobs, overridable for A/B characterization
        # (e.g. heading_hold:=false isolates the raw policy's own
        # straight-tracking with zero correction). Defaults match
        # policy.yaml so a bare launch is unaffected.
        DeclareLaunchArgument('heading_hold', default_value='true'),
        DeclareLaunchArgument('steer_kp', default_value='1.8'),
        DeclareLaunchArgument('steer_max', default_value='0.26'),
        DeclareLaunchArgument('lateral_hold', default_value='true'),
        # Lock heading to the spawn yaw for a true straight line (default). Set
        # false to let a commanded angular.z through (e.g. a turn-in-place test).
        DeclareLaunchArgument('heading_lock', default_value='true'),
        DeclareLaunchArgument('x', default_value='-3.5'),
        DeclareLaunchArgument('y', default_value='-3.5'),
        DeclareLaunchArgument('z', default_value='0.12'),
        # Face +x (East): straight traverse of the clear bottom corridor
        # (y=-3.5; all obstacles sit at y>=-1.8, east wall at x=5).
        DeclareLaunchArgument('yaw', default_value='0.0'),

        simulation,
        locomotion,
        map_server,
        map_lifecycle,
        map_to_odom,
        rviz,
        drive_straight,
        drift_trim,
    ])
