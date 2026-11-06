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
Gazebo + SLAM + minimal Nav2 + waypoint action server.

    ros2 launch robot_bringup navigation.launch.py
    ros2 run robot_navigation navigation_client 1.5,0 1.5,1.5,90 0,0,180

The robot localizes with slam_toolbox while it navigates, so the map grows
as it drives. Nav2 runs only the servers the default NavigateToPose
behavior tree needs (see config/nav2_params.yaml).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile

# Lifecycle nodes managed by nav2_lifecycle_manager, in startup order.
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

    # ── Launch arguments ──────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz with the SLAM view'),
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
            default_value=os.path.join(
                pkg_robot_bringup, 'config', 'nav2_params.yaml'),
            description='Nav2 parameters file'),
        DeclareLaunchArgument(
            'max_speed', default_value='0.3',
            description='Initial max_speed of the waypoint server (m/s)'),
    ]
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Gazebo + robot + slam_toolbox (map → odom) ────────────────────
    # Scoped so the arguments passed here do not leak into this file.
    slam = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_robot_bringup, 'launch', 'slam.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_rviz': LaunchConfiguration('use_rviz'),
                'use_joystick': LaunchConfiguration('use_joystick'),
                'world': LaunchConfiguration('world'),
            }.items())
    ], scoped=True)

    # ── Nav2 ──────────────────────────────────────────────────────────
    # allow_substs: resolves $(find-pkg-share ...) inside the YAML.
    nav2_params = ParameterFile(
        LaunchConfiguration('nav2_params'), allow_substs=True)

    nav2_nodes = [
        Node(
            package=package,
            executable=executable,
            name=executable,
            parameters=[nav2_params],
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

    # slam.launch.py starts slam_toolbox after 5 s; Nav2 needs the map →
    # odom transform and /map for the global costmap, so start a bit later.
    # SetParameter applies use_sim_time to every node in the group.
    navigation = TimerAction(
        period=8.0,
        actions=[GroupAction([
            SetParameter('use_sim_time', use_sim_time),
            *nav2_nodes,
            lifecycle_manager,
            waypoint_server,
        ])])

    return LaunchDescription(args + [slam, navigation])
