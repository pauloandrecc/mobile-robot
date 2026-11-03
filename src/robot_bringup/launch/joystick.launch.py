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
from launch_ros.actions import Node


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

    # ── Nodes ─────────────────────────────────────────────────────────

    # Reads the gamepad and publishes sensor_msgs/Joy on /joy, using SDL's
    # GameController mapping so button/axis indices are the same for any
    # supported controller (see config/joystick.yaml for the layout)
    joy_node = Node(
        package='joy',
        executable='game_controller_node',
        name='game_controller_node',
        parameters=[joy_config, {'device_id': joy_dev}],
        output='screen'
    )

    # Converts /joy into geometry_msgs/Twist on /cmd_vel
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_config],
        output='screen'
    )

    return LaunchDescription([
        joy_config_arg,
        joy_dev_arg,
        joy_node,
        teleop_node,
    ])
