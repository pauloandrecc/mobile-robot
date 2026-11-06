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
Integration test: the robot reaches a navigation goal in simulation.

Starts the whole stack with navigation.launch.py (headless Gazebo, SLAM,
Nav2, waypoint action server), sends one goal to /navigate_to_waypoints
and checks, against Gazebo's ground truth, that the robot stops within
0.3 m of it in under 60 s of simulated time.

    colcon test --packages-select robot_bringup \
        --ctest-args -L simulation --event-handlers console_direct+
    # or directly (after sourcing install/setup.zsh):
    launch_test src/robot_bringup/test/test_navigation.launch.py

How launch_testing works: generate_test_description() returns the launch
description to start; ReadyToTest() tells the framework to run the
unittest.TestCase classes (in parallel with the launch) as soon as the
processes are started. When the tests finish, the launch is shut down.
"""

import atexit
import math
import os
import re
import signal
import subprocess
import time
import unittest

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from robot_interfaces.action import NavigateToWaypoints
from std_srvs.srv import Trigger

# Goal behind box_1 (centered at 2,2): the straight line from the spawn
# pose passes too close to the box, so Nav2 has to plan around it.
GOAL_X, GOAL_Y, GOAL_YAW = 3.2, 2.0, math.pi / 2
POSITION_TOLERANCE = 0.3  # m
TIME_LIMIT = 60.0         # s of simulated time
# Wall time for Gazebo + SLAM + Nav2 to become active (usually ~12 s).
STARTUP_TIMEOUT = 60.0

# A private Gazebo partition: this test's Gazebo must not exchange
# messages with another simulation running on the same machine. Set
# before the launch starts so every child process inherits it. (The ROS
# side is isolated by the test runner, which picks a free ROS_DOMAIN_ID.)
GZ_PARTITION = f'robot_nav_test_{os.getpid()}'
os.environ['GZ_PARTITION'] = GZ_PARTITION


@atexit.register
def stop_orphaned_gazebo():
    """
    Terminate the Gazebo server this test started, if it outlived the launch.

    ros_gz_sim runs `gz sim` (a Ruby wrapper) which forks gz-sim-main. On
    shutdown the launch signals only the wrapper, so the server survives as
    an orphan and keeps publishing /clock and /tf. It is found by the
    GZ_PARTITION in its environment, so only this test's server is touched.
    """
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                if b'gz-sim-main' not in f.read():
                    continue
            with open(f'/proc/{pid}/environ', 'rb') as f:
                env = f.read().split(b'\0')
            if f'GZ_PARTITION={GZ_PARTITION}'.encode() in env:
                os.kill(int(pid), signal.SIGTERM)
                print(f'Stopped orphaned gz-sim-main (pid {pid})')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            pass  # process exited meanwhile or belongs to another user


@pytest.mark.launch_test
def generate_test_description():
    bringup = get_package_share_directory('robot_bringup')
    world = os.path.join(get_package_share_directory('robot_description'),
                         'worlds', 'obstacles_world.sdf')
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'navigation.launch.py')),
        launch_arguments={
            # -s: server only (no GUI), -r: run immediately (no play
            # button), --headless-rendering: sensors render without a
            # display (Lidar and camera need rendering).
            'world': f'-s -r --headless-rendering {world}',
            # SLAM: no saved map needed, the map origin is the spawn pose
            # (0, 0, 0), i.e. the map frame coincides with Gazebo's world.
            'slam': 'true',
            'use_rviz': 'false',
            'use_joystick': 'false',
        }.items())
    return LaunchDescription([
        navigation,
        launch_testing.actions.ReadyToTest(),
    ])


def ground_truth_xy():
    """Robot position from Gazebo itself (not from odometry or SLAM)."""
    out = subprocess.run(
        ['gz', 'model', '-m', 'mobile_robot', '-p'],
        capture_output=True, text=True, timeout=30, check=True).stdout
    # Output: "... Pose [ XYZ (m) ] [ RPY (rad) ]:\n  [x y z]\n  [r p y]"
    xyz = re.search(r'Pose.*?\[([-\d.e+ ]+)\]\s*\[([-\d.e+ ]+)\]',
                    out, re.DOTALL)
    assert xyz, f'unexpected gz model output:\n{out}'
    x, y, _ = (float(v) for v in xyz.group(1).split())
    return x, y


class TestNavigation(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        # Sim time: the time limit is about the robot, not about how fast
        # this machine runs the simulation.
        cls.node = rclpy.create_node(
            'navigation_test',
            parameter_overrides=[Parameter('use_sim_time', value=True)])

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, condition, timeout, period=0.1):
        """Spin the test node until condition() is true or timeout (wall)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=period)
            if condition():
                return True
        return False

    def call(self, client, request, timeout=5.0):
        future = client.call_async(request)
        self.spin_until(future.done, timeout)
        return future.result()

    def wait_for_nav2(self):
        """Block until lifecycle_manager_navigation reports Nav2 active."""
        is_active = self.node.create_client(
            Trigger, '/lifecycle_manager_navigation/is_active')
        deadline = time.monotonic() + STARTUP_TIMEOUT
        while time.monotonic() < deadline:
            if is_active.wait_for_service(timeout_sec=1.0):
                response = self.call(is_active, Trigger.Request())
                if response is not None and response.success:
                    return
            rclpy.spin_once(self.node, timeout_sec=1.0)
        # Known intermittent failure (about 1 run in 4): a lifecycle
        # change_state response is lost ("failed to send response to
        # /<node>/change_state (timeout)" in the log) and lifecycle_manager
        # waits forever. Rerun, e.g. with --ctest-args --repeat until-pass:3
        self.fail(f'Nav2 not active after {STARTUP_TIMEOUT:.0f} s; look for '
                  '"failed to send response" in the log (see AGENTS.md)')

    def test_robot_reaches_goal(self, proc_output):
        self.wait_for_nav2()

        client = ActionClient(
            self.node, NavigateToWaypoints, 'navigate_to_waypoints')
        self.assertTrue(client.wait_for_server(timeout_sec=30.0),
                        '/navigate_to_waypoints not available')

        goal = NavigateToWaypoints.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'  # stamp 0: latest TF
        pose.pose.position.x = GOAL_X
        pose.pose.position.y = GOAL_Y
        pose.pose.orientation.z = math.sin(GOAL_YAW / 2)
        pose.pose.orientation.w = math.cos(GOAL_YAW / 2)
        goal.waypoints.append(pose)

        # Wait until /clock is flowing so the start time is meaningful
        self.assertTrue(self.spin_until(
            lambda: self.node.get_clock().now().nanoseconds > 0, 10.0),
            'no /clock from Gazebo')
        start = self.node.get_clock().now()

        send_future = client.send_goal_async(goal)
        self.assertTrue(self.spin_until(send_future.done, 10.0))
        goal_handle = send_future.result()
        self.assertTrue(goal_handle.accepted, 'goal rejected')

        # Generous wall-clock bound; the real limit is checked in sim time
        result_future = goal_handle.get_result_async()
        finished = self.spin_until(result_future.done, 3 * TIME_LIMIT)
        elapsed = (self.node.get_clock().now() - start).nanoseconds / 1e9
        if not finished:
            goal_handle.cancel_goal_async()
            self.fail(f'navigation did not finish ({elapsed:.1f} s sim)')

        response = result_future.result()
        x, y = ground_truth_xy()
        error = math.hypot(x - GOAL_X, y - GOAL_Y)
        print(f'\nNavigation: status {response.status}, {elapsed:.1f} s '
              f'(sim), drove {response.result.total_distance_m:.2f} m, '
              f'final ground truth ({x:.2f}, {y:.2f}), error {error:.3f} m')

        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(response.result.success)
        self.assertEqual(list(response.result.visited_waypoints), ['wp_0'])
        self.assertLess(elapsed, TIME_LIMIT)
        self.assertLess(error, POSITION_TOLERANCE)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_waypoint_server_exits_cleanly(self, proc_info):
        # Runs after the launch was shut down (SIGINT to every process).
        # Our own node must handle it and exit with 0; third-party nodes
        # (Gazebo, Nav2) are not checked.
        codes = [info.returncode for info in proc_info
                 if 'waypoint_action_server'
                 in info.action.process_details['name']]
        self.assertEqual(codes, [0])
