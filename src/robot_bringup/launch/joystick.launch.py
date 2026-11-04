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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    pkg_robot_bringup = get_package_share_directory('robot_bringup')

    # ── Launch arguments ──────────────────────────────────────────────
    joy_config_arg = DeclareLaunchArgument(
        'joy_config',
        default_value=os.path.join(
            pkg_robot_bringup, 'config', 'joystick.yaml'),
        description='Parameter file for joy_node and teleop_twist_joy'
    )
    joy_config = LaunchConfiguration('joy_config')

    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='0',
        description='Joystick device index (0 = first controller found)'
    )
    joy_dev = LaunchConfiguration('joy_dev')

    # ── Composable nodes ──────────────────────────────────────────────
    # Both nodes are loaded as components into a single process (the
    # container) instead of running as two separate processes. With
    # intra-process communication enabled, /joy messages are passed by
    # pointer inside the process, without serialization through DDS.
    # They are still normal ROS 2 nodes: /joy and /cmd_vel are visible to
    # the rest of the system (ros2 topic echo, rqt_graph, ...).
    #   ros2 component list   → shows the container and its components
    gamepad_container = ComposableNodeContainer(
        name='gamepad_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Reads the gamepad and publishes sensor_msgs/Joy on /joy, using
            # SDL's GameController mapping so button/axis indices are the
            # same for any supported controller (see config/joystick.yaml)
            ComposableNode(
                package='joy',
                plugin='joy::GameController',
                name='game_controller_node',
                parameters=[joy_config, {'device_id': joy_dev}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # Converts /joy into geometry_msgs/Twist on /cmd_vel
            ComposableNode(
                package='teleop_twist_joy',
                plugin='teleop_twist_joy::TeleopTwistJoy',
                name='teleop_twist_joy_node',
                parameters=[joy_config],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen'
    )

    return LaunchDescription([
        joy_config_arg,
        joy_dev_arg,
        gamepad_container,
    ])
