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
Tests for waypoint_action_server against a fake Nav2.

The real waypoint_action_server runs as a process (started by
launch_testing). Instead of Gazebo and Nav2, the test itself offers the
navigate_to_pose action (FakeNav2) and decides how each Nav2 goal ends:
succeed, abort, reject, or keep running until canceled. That makes the
error paths (which are hard to provoke in a simulation) testable in a few
seconds, without Gazebo, so it also runs in CI.

    test node                                waypoint_action_server
    ─────────                                ──────────────────────
    client  ──NavigateToWaypoints──────────▶ server
    FakeNav2 ◀──NavigateToPose (one per wp)── client
    subscriber ◀──/speed_limit─────────────── publisher
    client  ──/set_speed_limit─────────────▶ service

Run with `colcon test --packages-select robot_navigation`, or directly:
`python3 -m pytest src/robot_navigation/test/test_waypoint_action_server.py`
(after sourcing install/setup.zsh).
"""

import random
import threading
import time
import unittest

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
import pytest
import rclpy
from rclpy.action import (ActionClient, ActionServer, CancelResponse,
                          GoalResponse)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from robot_interfaces.action import NavigateToWaypoints
from robot_interfaces.srv import SetSpeedLimit

# A private DDS domain, so this test neither sees nor disturbs a robot or
# simulation running on the same machine (the default domain is 0).
DOMAIN_ID = random.randint(100, 199)
MAX_SPEED = 0.3   # server parameter set in the launch below
TIMEOUT = 10.0    # s, per wait in the tests
HANG_LIMIT = 15.0  # s, longest a FakeNav2 'hang' goal runs


@pytest.mark.launch_test
def generate_test_description():
    server = launch_ros.actions.Node(
        package='robot_navigation',
        executable='waypoint_action_server',
        name='waypoint_action_server',
        # Short feedback period: the server also checks for cancel
        # requests at this rate, so the tests run faster.
        parameters=[{'max_speed': MAX_SPEED, 'feedback_period': 0.2}],
        additional_env={'ROS_DOMAIN_ID': str(DOMAIN_ID)},
        output='screen')
    return launch.LaunchDescription([
        server,
        launch_testing.actions.ReadyToTest(),
    ]), {'server': server}


def wait_for(future, timeout=TIMEOUT):
    """Block until an rclpy future is done (the executor runs elsewhere)."""
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    assert done.wait(timeout), 'timed out waiting for a response'
    return future.result()


def wait_until(condition, timeout=TIMEOUT):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return True
        time.sleep(0.05)
    return False


def pose(x, y, frame='map'):
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.w = 1.0
    return p


class FakeNav2:
    """
    navigate_to_pose action server whose outcome is chosen by the test.

    mode:
      'succeed' → one feedback at the goal pose, then SUCCEEDED
      'abort'   → ABORTED with an error code, like a failed plan
      'reject'  → goal rejected
      'hang'    → keeps running until it is canceled (aborts after
                  HANG_LIMIT s or at shutdown, so a server that never
                  forwards the cancel makes a test fail instead of hang)
    """

    def __init__(self, node, callback_group):
        self.mode = 'succeed'
        self.goals = []         # poses received, in order
        self.canceled = 0
        self.stop = threading.Event()  # set at shutdown
        self._server = ActionServer(
            node, NavigateToPose, 'navigate_to_pose',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            # rclpy rejects cancel requests by default
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=callback_group)

    def reset(self, mode):
        self.mode = mode
        self.goals = []
        self.canceled = 0

    def _on_goal(self, _request):
        if self.mode == 'reject':
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        target = goal_handle.request.pose
        self.goals.append(target)
        result = NavigateToPose.Result()

        if self.mode == 'abort':
            goal_handle.abort()
            result.error_code = NavigateToPose.Result.UNKNOWN
            result.error_msg = 'fake planner failure'
            return result

        if self.mode == 'hang':
            deadline = time.monotonic() + HANG_LIMIT
            while not goal_handle.is_cancel_requested:
                if self.stop.wait(0.05) or time.monotonic() > deadline:
                    goal_handle.abort()
                    return result
            goal_handle.canceled()
            self.canceled += 1
            return result

        # 'succeed': report the robot at the goal, then finish. Feedback
        # (a topic) and result (a service) travel separately; the pause
        # lets the feedback arrive first, as with a real robot.
        feedback = NavigateToPose.Feedback()
        feedback.current_pose = target
        goal_handle.publish_feedback(feedback)
        time.sleep(0.3)
        goal_handle.succeed()
        return result


class TestWaypointActionServer(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.Context()
        rclpy.init(context=cls.context, domain_id=DOMAIN_ID)
        cls.node = rclpy.create_node('waypoint_server_test',
                                     context=cls.context)
        group = ReentrantCallbackGroup()
        cls.nav2 = FakeNav2(cls.node, group)
        cls.client = ActionClient(cls.node, NavigateToWaypoints,
                                  'navigate_to_waypoints',
                                  callback_group=group)
        cls.speed_client = cls.node.create_client(
            SetSpeedLimit, '/set_speed_limit', callback_group=group)
        cls.speed_limits = []
        cls.node.create_subscription(
            SpeedLimit, '/speed_limit',
            lambda msg: cls.speed_limits.append(msg.speed_limit), 10,
            callback_group=group)

        # Spin in a background thread: FakeNav2's execute callback blocks
        # (sleep, 'hang') while the tests wait on futures.
        cls.executor = MultiThreadedExecutor(context=cls.context)
        cls.executor.add_node(cls.node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin,
                                           daemon=True)
        cls.spin_thread.start()

        assert cls.client.wait_for_server(timeout_sec=TIMEOUT)
        assert cls.speed_client.wait_for_service(timeout_sec=TIMEOUT)
        # The server rejects goals until its own Nav2 client has discovered
        # FakeNav2; its feedback subscription appearing is the visible sign.
        assert wait_until(lambda: cls.node.count_subscribers(
            '/navigate_to_pose/_action/feedback') > 0)
        time.sleep(1.0)

    @classmethod
    def tearDownClass(cls):
        cls.nav2.stop.set()  # release a FakeNav2 goal still 'hanging'
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def send(self, waypoints, max_speed=0.0, feedback=None):
        goal = NavigateToWaypoints.Goal(waypoints=waypoints,
                                        max_speed=max_speed)
        return wait_for(self.client.send_goal_async(
            goal, feedback_callback=feedback))

    def result(self, goal_handle):
        return wait_for(goal_handle.get_result_async())

    # ── Happy path ────────────────────────────────────────────────────
    def test_visits_all_waypoints_in_order(self):
        self.nav2.reset('succeed')
        self.speed_limits.clear()
        # Empty frame_id → the server fills in its default_frame ('map')
        handle = self.send([pose(1, 0), pose(1, 2, frame='')],
                           max_speed=0.1)
        self.assertTrue(handle.accepted)
        response = self.result(handle)

        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(response.result.success)
        self.assertEqual(list(response.result.visited_waypoints),
                         ['wp_0', 'wp_1'])
        # One Nav2 goal per waypoint, in order, all in the map frame
        self.assertEqual([(g.pose.position.x, g.pose.position.y,
                           g.header.frame_id) for g in self.nav2.goals],
                         [(1.0, 0.0, 'map'), (1.0, 2.0, 'map')])
        # Distance driven = path through the poses Nav2 reported
        self.assertAlmostEqual(response.result.total_distance_m, 2.0,
                               places=6)
        # The goal's limit (stricter than the parameter) was applied
        # while driving and the parameter restored when the goal ended.
        self.assertTrue(wait_until(lambda: self.speed_limits[-1:] ==
                                   [MAX_SPEED]))
        self.assertIn(0.1, self.speed_limits)

    # ── Goal validation ───────────────────────────────────────────────
    def test_rejects_goal_without_waypoints(self):
        self.nav2.reset('succeed')
        self.assertFalse(self.send([]).accepted)
        self.assertEqual(self.nav2.goals, [])

    def test_rejects_negative_max_speed(self):
        self.nav2.reset('succeed')
        self.assertFalse(self.send([pose(1, 0)], max_speed=-0.1).accepted)

    def test_rejects_second_goal_while_busy(self):
        self.nav2.reset('hang')
        first = self.send([pose(1, 0)])
        self.assertTrue(first.accepted)
        self.assertTrue(wait_until(lambda: len(self.nav2.goals) == 1))

        self.assertFalse(self.send([pose(2, 0)]).accepted)

        wait_for(first.cancel_goal_async())
        self.assertEqual(self.result(first).status,
                         GoalStatus.STATUS_CANCELED)

    # ── Nav2 failures ─────────────────────────────────────────────────
    def test_aborts_when_nav2_fails(self):
        self.nav2.reset('abort')
        response = self.result(self.send([pose(1, 0), pose(2, 0)]))

        self.assertEqual(response.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(response.result.success)
        self.assertEqual(list(response.result.visited_waypoints), [])
        self.assertEqual(len(self.nav2.goals), 1)  # stopped at the first

    def test_aborts_when_nav2_rejects(self):
        self.nav2.reset('reject')
        response = self.result(self.send([pose(1, 0)]))
        self.assertEqual(response.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(response.result.success)

    # ── Cancel and feedback ───────────────────────────────────────────
    def test_cancel_is_forwarded_to_nav2(self):
        self.nav2.reset('hang')
        feedback = []
        handle = self.send([pose(1, 0), pose(2, 0)],
                           feedback=lambda msg: feedback.append(msg.feedback))
        self.assertTrue(handle.accepted)
        # Feedback flows while Nav2 is driving to the first waypoint
        self.assertTrue(wait_until(lambda: len(feedback) >= 2))
        self.assertEqual(feedback[-1].current_waypoint, 'wp_0')

        wait_for(handle.cancel_goal_async())
        response = self.result(handle)

        self.assertEqual(response.status, GoalStatus.STATUS_CANCELED)
        self.assertFalse(response.result.success)
        self.assertEqual(self.nav2.canceled, 1)  # Nav2 goal canceled too
        self.assertEqual(len(self.nav2.goals), 1)

    # ── Speed limit service ───────────────────────────────────────────
    def test_set_speed_limit_service(self):
        self.speed_limits.clear()
        ok = wait_for(self.speed_client.call_async(
            SetSpeedLimit.Request(max_speed=0.2)))
        self.assertTrue(ok.success, ok.message)
        self.assertTrue(wait_until(lambda: 0.2 in self.speed_limits))

        # Out of the parameter's range (ParameterDescriptor): refused
        # by rclpy before the node's callback runs; nothing published.
        self.speed_limits.clear()
        bad = wait_for(self.speed_client.call_async(
            SetSpeedLimit.Request(max_speed=1.0)))
        self.assertFalse(bad.success)
        time.sleep(0.5)
        self.assertEqual(self.speed_limits, [])

        # Restore the value the other tests expect
        restore = wait_for(self.speed_client.call_async(
            SetSpeedLimit.Request(max_speed=MAX_SPEED)))
        self.assertTrue(restore.success)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_server_exits_cleanly(self, proc_info, server):
        launch_testing.asserts.assertExitCodes(proc_info, process=server)
