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
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression

def generate_launch_description():

    pkg_robot_description = get_package_share_directory('robot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    urdf_file = os.path.join(
        pkg_robot_description, 'urdf', 'robot.urdf.xacro')

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='obstacles_world',
        description='Gazebo world name (used for joint state topic)'
    )
    world_name = LaunchConfiguration('world_name')

    # ── Launch arguments ──────────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz alongside Gazebo'
    )
    use_rviz = LaunchConfiguration('use_rviz')

    # ── Robot description (processed from Xacro) ──────────────────────
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # ── Nodes and includes ────────────────────────────────────────────

    # Publishes /robot_description and static TF transforms
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True   # sync with Gazebo clock
        }]
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            pkg_robot_description, 'worlds', 'obstacles_world.sdf'),
        description='Path to the Gazebo world file'
    )
    world = LaunchConfiguration('world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': world}.items()
    )

    # Spawns the robot into the running Gazebo simulation
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=[
            '-name', 'mobile_robot',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Bridge between Gazebo topics and ROS 2 topics
    # Format: gz_topic@ros_msg_type[gz_msg_type  (gz → ROS)
    #         gz_topic@ros_msg_type]gz_msg_type  (ROS → gz)
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            [
                '/world/', world_name,
                '/model/mobile_robot/joint_state'
                '@sensor_msgs/msg/JointState[gz.msgs.Model'
            ],
        ],
        remappings=[
            (
                ['/world/', world_name, '/model/mobile_robot/joint_state'],
                '/joint_states'
            ),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # RViz for visualization
    rviz_config = os.path.join(
        pkg_robot_description, 'rviz', 'robot.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        use_rviz_arg,
        world_arg,
        world_name_arg,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        gz_bridge,
        rviz,
    ])