# Copyright 2026 Paulo
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
Gazebo + localization + Nav2 + waypoint action server.

Two scenarios, selected with slam:=
  slam:=true  (default) mapping: slam_toolbox builds the map while the
              robot navigates/explores (map → odom from slam_toolbox).
  slam:=false known map: map_server loads map:=<file.yaml> and AMCL
              localizes the robot on it (map → odom from AMCL).

    ros2 launch robot_bringup navigation.launch.py
    ros2 launch robot_bringup navigation.launch.py slam:=false
    ros2 launch robot_bringup navigation.launch.py slam:=false controller:=mppi

Then send goals with RViz's "Nav2 Goal" tool or with
    ros2 run robot_navigation navigation_client 1.5,0 1.5,1.5,90 0,0,180
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile

# Lifecycle nodes managed by lifecycle_manager_navigation, in startup order.
# Must match lifecycle_manager_navigation.node_names in nav2_params.yaml.
NAV2_NODES = [
    ('nav2_controller', 'controller_server'),
    ('nav2_planner', 'planner_server'),
    ('nav2_behaviors', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator'),
]


def generate_launch_description():

    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    pkg_robot_description = get_package_share_directory('robot_description')
    config_dir = os.path.join(pkg_robot_bringup, 'config')

    # ── Launch arguments ──────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'slam', default_value='true', choices=['true', 'false'],
            description='true: map with slam_toolbox while navigating; '
                        'false: saved map (map:=) + AMCL localization'),
        DeclareLaunchArgument(
            'map',
            # Resolved against the directory the launch is started from
            # (the repository root in the README), like slam.launch.py bag:=
            default_value=os.path.join(os.getcwd(), 'maps', 'my_map.yaml'),
            description='Map YAML loaded by map_server when slam:=false'),
        DeclareLaunchArgument(
            'controller', default_value='dwb', choices=['dwb', 'mppi'],
            description='Local controller: config/controller_<name>.yaml'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz with the Nav2 view'),
        DeclareLaunchArgument(
            'use_joystick', default_value='true',
            description='Teleoperate the robot with a gamepad'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                pkg_robot_description, 'worlds', 'obstacles_world.sdf'),
            description='Path to the Gazebo world file'),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=os.path.join(config_dir, 'nav2_params.yaml'),
            description='Nav2 parameters file'),
        DeclareLaunchArgument(
            'max_speed', default_value='0.3',
            description='Initial max_speed of the waypoint server (m/s)'),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')

    # allow_substs: resolves $(find-pkg-share ...) inside the YAML.
    nav2_params = ParameterFile(
        LaunchConfiguration('nav2_params'), allow_substs=True)
    # config/controller_dwb.yaml or config/controller_mppi.yaml. Passed
    # after nav2_params: for the same parameter, the last file wins.
    controller_params = ParameterFile(
        [os.path.join(config_dir, 'controller_'),
         LaunchConfiguration('controller'), '.yaml'])

    # ── Gazebo + robot + bridge (+ gamepad) ───────────────────────────
    # Scoped so the arguments passed here do not leak into this file
    # (use_rviz:=false below would otherwise turn off our own RViz).
    simulation = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_robot_bringup, 'launch',
                             'simulation.launch.py')),
            launch_arguments={
                'use_rviz': 'false',  # Nav2 RViz view is started below
                'world': LaunchConfiguration('world'),
                'use_joystick': LaunchConfiguration('use_joystick'),
            }.items())
    ], scoped=True)

    # ── Localization, option A: SLAM (slam:=true) ─────────────────────
    # slam_toolbox publishes /map and map → odom. Started after 5 s to give
    # Gazebo and the bridge time to come up (same as slam.launch.py).
    # The scoped GroupAction goes *inside* the TimerAction: the timer runs
    # its actions later, after an outer group's scope would already be over.
    slam_localization = TimerAction(period=5.0, actions=[GroupAction(
        condition=IfCondition(slam),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_robot_bringup, 'launch',
                                 'slam_toolbox.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time}.items())
        ])])

    # ── Localization, option B: saved map + AMCL (slam:=false) ────────
    # map_server publishes the saved map on /map; AMCL matches /scan
    # against it and publishes map → odom. Both are Nav2 lifecycle nodes,
    # so (unlike slam_toolbox) a lifecycle_manager with bonds can drive them.
    amcl_localization = TimerAction(period=5.0, actions=[GroupAction(
        condition=UnlessCondition(slam),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[nav2_params,
                            {'yaml_filename': LaunchConfiguration('map')}],
                output='screen'),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                parameters=[nav2_params],
                output='screen'),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                parameters=[nav2_params],
                output='screen'),
        ])])

    # ── Nav2 ──────────────────────────────────────────────────────────
    nav2_nodes = [
        Node(
            package=package,
            executable=executable,
            name=executable,
            parameters=([nav2_params, controller_params]
                        if executable == 'controller_server'
                        else [nav2_params]),
            output='screen')
        for package, executable in NAV2_NODES
    ]

    # Drives the lifecycle nodes above through configure → activate and
    # keeps a bond with each one (if a node dies, the stack is brought down).
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[nav2_params],
        output='screen')

    # ── Waypoint action server (Phase 4) ──────────────────────────────
    waypoint_server = Node(
        package='robot_navigation',
        executable='waypoint_action_server',
        name='waypoint_action_server',
        parameters=[{'max_speed': LaunchConfiguration('max_speed')}],
        output='screen')

    # Localization starts at 5 s; Nav2 needs map → odom and /map for the
    # global costmap, so start a bit later.
    # SetParameter applies use_sim_time to every node in the group.
    navigation = TimerAction(
        period=8.0,
        actions=[GroupAction([
            SetParameter('use_sim_time', use_sim_time),
            *nav2_nodes,
            lifecycle_manager,
            waypoint_server,
        ])])

    # ── RViz: map, costmaps, plans, AMCL particles, Nav2 panel ────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            pkg_robot_description, 'rviz', 'nav2.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen')

    return LaunchDescription(args + [
        simulation,
        slam_localization,
        amcl_localization,
        navigation,
        rviz,
    ])
