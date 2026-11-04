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
Rebuild a map with slam_toolbox from a bag recorded by slam.launch.py.

No Gazebo: the bag provides the Lidar scans, the odometry and the clock.

  ros2 launch robot_bringup slam_replay.launch.py bag:=bags/mapping_<date>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    pkg_robot_description = get_package_share_directory('robot_description')

    # ── Launch arguments ──────────────────────────────────────────────
    bag_arg = DeclareLaunchArgument(
        'bag',
        description='Bag folder recorded with slam.launch.py record:=true'
    )
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Playback speed (e.g. 2.0 = twice as fast)'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz with the SLAM view'
    )

    # Every node follows the clock republished from the bag
    sim_time = {'use_sim_time': True}

    # ── Robot model ───────────────────────────────────────────────────
    # Republishes the static frames (lidar_link, ...) and, from the
    # recorded /joint_states, the wheel transforms.
    urdf_file = os.path.join(
        pkg_robot_description, 'urdf', 'robot.urdf.xacro')
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file]), value_type=str),
            **sim_time,
        }]
    )

    # ── /tf filter ────────────────────────────────────────────────────
    # The bag's /tf is played on /tf_bag; only odom→base_footprint is
    # forwarded to /tf (the recorded map→odom would fight the new SLAM).
    bag_tf_filter = Node(
        package='robot_bringup',
        executable='bag_tf_filter.py',
        name='bag_tf_filter',
        parameters=[{'child_frames': ['base_footprint'], **sim_time}],
        output='screen'
    )

    # ── slam_toolbox (configured + activated) ─────────────────────────
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_bringup, 'launch', 'slam_toolbox.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # ── Bag playback ──────────────────────────────────────────────────
    # --clock: publish /clock from the bag timeline (the bag was recorded
    #          with --use-sim-time, so it is the original Gazebo time)
    # --remap: /tf goes to the filter instead of straight to /tf
    # /tf_static is not played: robot_state_publisher provides it
    play_bag = TimerAction(
        period=4.0,  # let slam_toolbox become active first
        actions=[ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'),
                 '--clock', '100',
                 '--rate', LaunchConfiguration('rate'),
                 '--remap', '/tf:=/tf_bag',
                 '--exclude-topics', '/tf_static'],
            output='screen'
        )]
    )

    # ── RViz ──────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            pkg_robot_description, 'rviz', 'slam.rviz')],
        parameters=[sim_time],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )

    return LaunchDescription([
        bag_arg,
        rate_arg,
        use_rviz_arg,
        robot_state_publisher,
        bag_tf_filter,
        slam_toolbox,
        play_bag,
        rviz,
    ])
