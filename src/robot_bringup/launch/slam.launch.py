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
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction,
                            IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


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

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz with the SLAM view'
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
    simulation_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_robot_bringup, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'use_rviz': 'false',  # RViz with the SLAM config is started below
            'world': LaunchConfiguration('world'),
            'world_name': LaunchConfiguration('world_name'),
            'use_joystick': LaunchConfiguration('use_joystick'),
        }.items()
    )
    simulation = GroupAction([simulation_include], scoped=True)

    # ── slam_toolbox node ─────────────────────────────────────────────
    # Synchronous mapping: subscribes to /scan and /tf,
    # publishes /map and the map→odom transform.
    # slam_toolbox is a lifecycle node: it starts "unconfigured" and must be
    # configured and then activated before it does anything.
    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Step 1: unconfigured → inactive
    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    # Step 2: as soon as it is inactive → active
    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[slam.launch] Activating slam_toolbox'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        )
    )

    # Give Gazebo and the bridge a few seconds to come up first
    start_slam = TimerAction(
        period=5.0,
        actions=[activate_slam, slam_toolbox, configure_slam]
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
        world_arg,
        world_name_arg,
        use_joystick_arg,
        simulation,
        start_slam,
        rviz,
    ])
