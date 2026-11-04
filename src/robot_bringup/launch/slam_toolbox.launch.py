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

"""Start slam_toolbox (synchronous mapping) and bring it to the active state.

Shared by slam.launch.py (live mapping in Gazebo) and slam_replay.launch.py
(mapping from a recorded bag).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():

    pkg_robot_bringup = get_package_share_directory('robot_bringup')

    # ── Launch arguments ──────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock (Gazebo or bag /clock)'
    )
    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            pkg_robot_bringup, 'config', 'slam_toolbox_params.yaml'),
        description='slam_toolbox parameter file'
    )

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
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
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
                LogInfo(msg='[slam_toolbox.launch] Activating slam_toolbox'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_params_arg,
        activate_slam,
        slam_toolbox,
        configure_slam,
    ])
