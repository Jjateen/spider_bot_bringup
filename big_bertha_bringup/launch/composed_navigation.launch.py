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
SLAM (or AMCL) plus the five Nav2 servers, composed.

Separate container from the control chain on purpose. The control loop must
keep hitting 50 Hz while a planner replans or slam_toolbox closes a loop, and
those hold a thread for tens of milliseconds. Two containers keeps each side's
scheduling independent, and component_container_isolated gives every component
its own executor inside them.

Getting parameters to the costmaps is the subtle part, and the first attempt
got it wrong. A ComposableNode's `parameters` reach only that named node, so
controller_server received its own controller_server: section and the sibling
local_costmap:/global_costmap: sections never reached the costmap CHILD nodes.
Those silently fell back to Nav2 defaults (static layer, no scan source) and
never published, which reads downstream as "no costmap received". The same
params file is therefore ALSO passed to the container process, exactly as
upstream nav2_bringup does it, which makes it a process-level source the child
nodes can resolve against.

Intra-process comms default to FALSE here, unlike the control container. /map
is TRANSIENT_LOCAL: slam publishes it latched and the global costmap subscribes
latched (map_subscribe_transient_local in nav2_params.yaml). That durability is
where PR #58 hit the rmw_cyclonedds hang (ros2/rmw_cyclonedds#401), and a
costmap that never receives its map is worse than one extra copy per second.
Flip intra_process:=true to experiment.
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

    # Composed again, but this time the params reach the costmaps. The earlier
    # attempt gave the file only to each ComposableNode, so controller_server
    # got its own controller_server: section while the sibling local_costmap:/
    # global_costmap: sections never reached the costmap CHILD nodes. They fell
    # back to Nav2 defaults (static layer, no scan source) and never published.
    # The fix is on the container below: upstream nav2_bringup puts the same
    # params file on the CONTAINER process, making it a process-level source
    # that child nodes can resolve against.
    nav2_nodes = [
        ComposableNode(
            package=pkg, plugin=plugin, name=name,
            parameters=[configured_nav2, common],
            extra_arguments=ipc,
        )
        for pkg, plugin, name in [
            ('nav2_controller', 'nav2_controller::ControllerServer', 'controller_server'),
            ('nav2_smoother', 'nav2_smoother::SmootherServer', 'smoother_server'),
            ('nav2_planner', 'nav2_planner::PlannerServer', 'planner_server'),
            ('nav2_behaviors', 'behavior_server::BehaviorServer', 'behavior_server'),
            ('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator', 'bt_navigator'),
        ]
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

    def container(name, nodes, condition=None, params=None):
        # component_container_isolated, not _mt: each component gets its own
        # executor, so a planner replan or a slam loop closure cannot stall a
        # sibling. Upstream nav2_bringup uses it for the same reason.
        #
        # `params` is the part that was missing before. Passing the params file
        # here makes it a PROCESS-level parameter source, so nodes created
        # INSIDE a component (Costmap2DROS under controller_server) can resolve
        # their own local_costmap:/global_costmap: sections. A ComposableNode's
        # own `parameters` only ever reach that one named node.
        return ComposableNodeContainer(
            name=name, namespace='', package='rclcpp_components',
            executable='component_container_isolated',
            composable_node_descriptions=nodes,
            parameters=params,
            output='screen', condition=condition,
        )

    # Lifecycle managers stay ordinary nodes: they only transition others.
    def lifecycle(name, node_names, condition=None, autostart=True):
        return Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name=name, output='screen', condition=condition,
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'autostart': autostart,
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
        DeclareLaunchArgument(
            'mapping_autostart', default_value='false',
            description='true: map from launch; false: wait for '
                        'scripts/start_mapping.sh so the operator can step '
                        'clear before the first scan is frozen into the map'),

        container('big_bertha_slam_container', [slam_node],
                  condition=IfCondition(slam)),
        # mapping_autostart:=false parks slam_toolbox unconfigured, so it holds
        # no /scan subscription and maps nothing until the operator runs
        # scripts/start_mapping.sh. Powering the servos means someone walks to
        # the buck-converter switch, and slam_toolbox freezes its first scan in
        # permanently while the robot stands still (it needs 0.1 m of travel
        # before it integrates another), so that person becomes an obstacle
        # overlapping the footprint and Nav2 refuses to plan. Only this manager
        # is gated; Nav2 itself still comes up and waits for a map.
        lifecycle('lifecycle_manager_slam', ['slam_toolbox'],
                  condition=IfCondition(slam),
                  autostart=ParameterValue(
                      LaunchConfiguration('mapping_autostart'),
                      value_type=bool)),

        container('big_bertha_localization_container', localization_nodes,
                  condition=UnlessCondition(slam)),
        lifecycle('lifecycle_manager_localization', ['map_server', 'amcl'],
                  condition=UnlessCondition(slam)),

        container('big_bertha_nav_container', nav2_nodes,
                  params=[configured_nav2]),
        lifecycle('lifecycle_manager_navigation', [
            'controller_server', 'smoother_server', 'planner_server',
            'behavior_server', 'bt_navigator']),
    ])
