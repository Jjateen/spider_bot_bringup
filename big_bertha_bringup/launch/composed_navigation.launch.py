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
SLAM (or AMCL) plus the five Nav2 servers.

The Nav2 servers run as regular Nodes, NOT composed, and that is deliberate:
when controller_server is loaded as a ComposableNode in a container, the
costmap child-nodes it creates internally lose their parameter namespace. The
controller component only receives its own ``controller_server:`` section from
nav2_params.yaml, so the sibling ``local_costmap:``/``global_costmap:``
sections never land where the costmaps can read them. The costmaps then build
with Nav2 defaults (static/obstacle/inflation layers, map frame, no sensor
source) and never publish /costmap. Running the servers as plain Nodes makes
each process load the whole params file, and the costmaps resolve their
sections correctly - the same layout the sim's nav2.launch.py uses.

The slam / localization containers stay composed: each is a single node loading
its own params file directly, so there is no namespace problem there.

Separate processes from the control container on purpose. The control loop must
keep hitting 50 Hz while a planner replans or slam_toolbox closes a loop, and
those are exactly the operations that occupy a thread for tens of
milliseconds. Sharing one executor would let a loop closure stall a servo
write. Two containers keeps each side's scheduling independent.

Intra-process comms default to FALSE here, unlike the control container, and
that is deliberate rather than an oversight. /map is TRANSIENT_LOCAL: slam
publishes it latched and the global costmap subscribes latched
(map_subscribe_transient_local in nav2_params.yaml). Intra-process handling of
transient-local durability is where PR #58 hit the rmw_cyclonedds hang
(ros2/rmw_cyclonedds#401), and a costmap that never receives its map is a much
worse failure than one extra copy per second. Flip intra_process:=true to
experiment; the costmap topics and /scan_filtered would benefit.

Verified on the board:
    slam_toolbox::AsynchronousSlamToolbox   nav2_amcl::AmclNode
    nav2_map_server::MapServer              nav2_controller::ControllerServer
    nav2_planner::PlannerServer             nav2_smoother::SmootherServer
    behavior_server::BehaviorServer         nav2_bt_navigator::BtNavigator
"""

import os
from typing import List

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Build the composed navigation container."""
    sim_pkg = get_package_share_directory('big_bertha_sim_bringup')
    bt_dir = get_package_share_directory('nav2_bt_navigator')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_yaml = LaunchConfiguration('map')
    nav_speed = LaunchConfiguration('nav_speed')
    intra_process = LaunchConfiguration('intra_process')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    nav2_params = os.path.join(sim_pkg, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(sim_pkg, 'config', 'slam_toolbox.yaml')
    amcl_params = os.path.join(sim_pkg, 'config', 'amcl.yaml')

    bt_to_pose = os.path.join(
        bt_dir, 'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml')
    bt_through_poses = os.path.join(
        bt_dir, 'behavior_trees',
        'navigate_through_poses_w_replanning_and_recovery.xml')

    # Same rewrite the standalone nav2.launch.py does. RewrittenYaml only
    # REPLACES keys that already exist, and nav2_params.yaml carries no
    # use_sim_time, so it is also passed explicitly per component below.
    configured_nav2 = RewrittenYaml(
        source_file=nav2_params,
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'default_nav_to_pose_bt_xml': bt_to_pose,
            'default_nav_through_poses_bt_xml': bt_through_poses,
            'desired_linear_vel': nav_speed,
        },
        convert_types=True,
    )

    ipc = [{'use_intra_process_comms': ParameterValue(
        intra_process, value_type=bool)}]
    common = {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}

    # The five Nav2 servers run as REGULAR Nodes, not composed (see the module
    # docstring): composing controller_server loses the local/global costmap
    # parameter sections, leaving the costmaps on defaults and silent. Each
    # process loads the full rewritten nav2_params.yaml like the sim launch.
    nav2_nodes = [
        (pkg, exe)
        for pkg, exe in [
            ('nav2_controller', 'controller_server'),
            ('nav2_smoother', 'smoother_server'),
            ('nav2_planner', 'planner_server'),
            ('nav2_behaviors', 'behavior_server'),
            ('nav2_bt_navigator', 'bt_navigator'),
        ]
    ]

    nav2_server_nodes = [
        Node(
            package=pkg, executable=exe, name=exe, output='screen',
            respawn=False,
            parameters=[configured_nav2, common],
        )
        for pkg, exe in nav2_nodes
    ]

    # ── map -> odom, one of two ways ──────────────────────────────────
    # SLAM: live mapping. x/y/yaw seed map_start_pose so the grid is built
    # around where the robot actually is, not the Gazebo spawn pose baked
    # into slam_toolbox.yaml.
    slam_node = ComposableNode(
        package='slam_toolbox',
        plugin='slam_toolbox::AsynchronousSlamToolbox',
        name='slam_toolbox',
        parameters=[
            slam_params,
            common,
            {
                'map_start_pose': ParameterValue(
                    PythonExpression(['[', x, ', ', y, ', ', yaw, ']']),
                    value_type=List[float]),
            },
        ],
        extra_arguments=ipc,
    )

    localization_nodes = [
        ComposableNode(
            package='nav2_map_server', plugin='nav2_map_server::MapServer',
            name='map_server',
            parameters=[common, {'yaml_filename': map_yaml}],
            extra_arguments=ipc,
        ),
        ComposableNode(
            package='nav2_amcl', plugin='nav2_amcl::AmclNode', name='amcl',
            parameters=[
                amcl_params,
                common,
                {
                    'initial_pose.x': ParameterValue(x, value_type=float),
                    'initial_pose.y': ParameterValue(y, value_type=float),
                    'initial_pose.yaw': ParameterValue(yaw, value_type=float),
                },
            ],
            extra_arguments=ipc,
        ),
    ]

    def container(name, nodes, condition=None):
        return ComposableNodeContainer(
            name=name, namespace='', package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=nodes,
            output='screen', condition=condition,
        )

    # Lifecycle managers stay ordinary nodes: they only transition others.
    def lifecycle(name, node_names, condition=None):
        return Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name=name, output='screen', condition=condition,
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'autostart': True,
                'bond_timeout': 0.0,
                'node_names': node_names,
            }],
        )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='true = live mapping, false = known map via AMCL'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('nav_speed', default_value='0.29'),
        DeclareLaunchArgument(
            'intra_process', default_value='false',
            description='See the module docstring: /map is TRANSIENT_LOCAL'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        container('big_bertha_slam_container', [slam_node],
                  condition=IfCondition(slam)),
        lifecycle('lifecycle_manager_slam', ['slam_toolbox'],
                  condition=IfCondition(slam)),

        container('big_bertha_localization_container', localization_nodes,
                  condition=UnlessCondition(slam)),
        lifecycle('lifecycle_manager_localization', ['map_server', 'amcl'],
                  condition=UnlessCondition(slam)),

        *nav2_server_nodes,
        lifecycle('lifecycle_manager_navigation', [
            'controller_server', 'smoother_server', 'planner_server',
            'behavior_server', 'bt_navigator']),
    ])
