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

from datetime import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            GroupAction, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Topics recorded with record:=true. Enough to rebuild the map offline with
# slam_replay.launch.py: Lidar, odometry (/odom and odom→base_footprint in
# /tf), joint states, plus IMU and velocity commands for later analysis.
# /clock is not recorded: the bag is stamped with simulation time
# (--use-sim-time) and slam_replay.launch.py republishes the clock from it.
RECORDED_TOPICS = [
    '/scan', '/odom', '/tf', '/tf_static', '/joint_states', '/imu',
    '/cmd_vel',
]


def generate_launch_description():

    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    pkg_robot_description = get_package_share_directory('robot_description')

    # ── Launch arguments ──────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz with the SLAM view'
    )

    record_arg = DeclareLaunchArgument(
        'record',
        default_value='false',
        description='Record the mapping session to a bag (MCAP)'
    )
    bag_arg = DeclareLaunchArgument(
        'bag',
        default_value=os.path.join(
            'bags', 'mapping_' + datetime.now().strftime('%Y-%m-%d_%H-%M-%S')),
        description='Output folder of the bag (relative to the current dir)'
    )

    # Forwarded to simulation.launch.py
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            pkg_robot_description, 'worlds', 'obstacles_world.sdf'),
        description='Path to the Gazebo world file'
    )
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='obstacles_world',
        description='Gazebo world name (used for joint state topic)'
    )
    use_joystick_arg = DeclareLaunchArgument(
        'use_joystick',
        default_value='true',
        description='Teleoperate the robot with a gamepad'
    )

    # ── Include simulation launch ─────────────────────────────────────
    # Brings up Gazebo, robot, bridge and gamepad teleop.
    # Wrapped in a scoped GroupAction: arguments passed to an include leak
    # into this file otherwise (use_rviz:=false would also disable the RViz
    # started below).
    simulation = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    pkg_robot_bringup, 'launch', 'simulation.launch.py')
            ),
            launch_arguments={
                'use_rviz': 'false',  # RViz with the SLAM config is below
                'world': LaunchConfiguration('world'),
                'world_name': LaunchConfiguration('world_name'),
                'use_joystick': LaunchConfiguration('use_joystick'),
            }.items()
        )
    ], scoped=True)

    # ── slam_toolbox ──────────────────────────────────────────────────
    # Configured and activated by slam_toolbox.launch.py.
    # Give Gazebo and the bridge a few seconds to come up first.
    slam_toolbox = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg_robot_bringup, 'launch', 'slam_toolbox.launch.py')
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )

    # ── Bag recording (record:=true) ──────────────────────────────────
    record_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--use-sim-time',
             '--storage', 'mcap',
             '--output', LaunchConfiguration('bag'),
             '--topics'] + RECORDED_TOPICS,
        condition=IfCondition(LaunchConfiguration('record')),
        output='screen'
    )

    # ── RViz with SLAM-specific config ────────────────────────────────
    rviz_config = os.path.join(
        pkg_robot_description, 'rviz', 'slam.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_rviz_arg,
        record_arg,
        bag_arg,
        world_arg,
        world_name_arg,
        use_joystick_arg,
        simulation,
        slam_toolbox,
        record_bag,
        rviz,
    ])
