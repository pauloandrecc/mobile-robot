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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():

    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    pkg_robot_description = get_package_share_directory('robot_description')

    slam_params_file = os.path.join(
        pkg_robot_bringup, 'config', 'slam_toolbox_params.yaml')

    # ── Launch arguments ──────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Include simulation launch ─────────────────────────────────────
    # Brings up Gazebo, robot, bridge and RViz
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_robot_bringup, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={'use_rviz': 'false'}.items()
    )

    # ── slam_toolbox node ─────────────────────────────────────────────
    # Runs async mapping: subscribes to /scan and /tf,
    # publishes /map and the map→odom transform
    slam_toolbox = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='sync_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[
                    slam_params_file,
                    {'use_sim_time': use_sim_time}
                ],
                output='screen'
            )
        ]
    )

    # ── RViz with SLAM-specific config ────────────────────────────────
    rviz_config = os.path.join(
        pkg_robot_description, 'rviz', 'slam.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    lifecycle_manager = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_slam',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['slam_toolbox']
                }],
                output='screen'
            )
        ]
    )
    return LaunchDescription([
        use_sim_time_arg,
        simulation,
        slam_toolbox,
        lifecycle_manager,
        rviz,
    ])