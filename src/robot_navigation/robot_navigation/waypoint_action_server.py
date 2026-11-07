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
Waypoint navigation action server.

Offers /navigate_to_waypoints (robot_interfaces/action/NavigateToWaypoints)
and drives the robot through the waypoints by sending one
nav2_msgs/action/NavigateToPose goal per waypoint to Nav2's bt_navigator.

This node is both an action *server* (towards our client) and an action
*client* (towards Nav2):

    navigation_client ──NavigateToWaypoints──▶ this node
                                                  │ NavigateToPose (x N)
                                                  ▼
                                             bt_navigator (Nav2)

Speed limit:
  - Parameter `max_speed` (m/s) is the global limit. It can be changed with
    `ros2 param set` or with the /set_speed_limit service.
  - A goal may carry its own `max_speed`; the lower of the two is used.
  - The limit is applied by publishing nav2_msgs/SpeedLimit on /speed_limit,
    which Nav2's controller_server uses to cap the controller's speed.
"""

import threading

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from rcl_interfaces.msg import FloatingPointRange, ParameterDescriptor
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from robot_interfaces.action import NavigateToWaypoints
from robot_interfaces.srv import SetSpeedLimit

from robot_navigation.waypoint_math import (effective_speed_limit, path_length,
                                            percent_complete, planar_distance)

MAX_SPEED_MIN = 0.05  # m/s
MAX_SPEED_MAX = 0.5   # m/s, matches the controller's max_vel_x


def _xy(pose_stamped):
    p = pose_stamped.pose.position
    return (p.x, p.y)


def _wait(future, timeout=None):
    """
    Block the calling thread until an rclpy future is done.

    Only safe because this node runs on a MultiThreadedExecutor: another
    executor thread processes the response that completes the future.
    """
    event = threading.Event()
    future.add_done_callback(lambda _: event.set())
    return event.wait(timeout)


class WaypointActionServer(Node):
    """Follow a list of waypoints using Nav2's NavigateToPose action."""

    def __init__(self):
        super().__init__('waypoint_action_server')

        # ── Parameters ────────────────────────────────────────────────
        # The descriptor documents the parameter (`ros2 param describe`)
        # and makes rclpy reject out-of-range values on its own.
        self.declare_parameter(
            'max_speed', 0.3,
            ParameterDescriptor(
                description='Maximum linear speed during navigation (m/s)',
                floating_point_range=[FloatingPointRange(
                    from_value=MAX_SPEED_MIN, to_value=MAX_SPEED_MAX)]))
        self.declare_parameter(
            'feedback_period', 1.0,
            ParameterDescriptor(description='Feedback period (s)'))
        self.declare_parameter(
            'default_frame', 'map',
            ParameterDescriptor(
                description='Frame used for waypoints with empty frame_id'))

        # Runs after a parameter change was accepted (validation already
        # happened against the descriptor) → apply the new limit right away.
        self.add_post_set_parameters_callback(self._on_parameters_set)

        # One reentrant group: the long-running execute callback must not
        # block the Nav2 client responses, the service or the timers.
        cb_group = ReentrantCallbackGroup()

        self._goal_lock = threading.Lock()
        self._active_goal = None
        self._goal_speed = 0.0  # per-goal limit, 0 = none

        # ── Nav2 action client ────────────────────────────────────────
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=cb_group)

        # ── Our action server ─────────────────────────────────────────
        self._action_server = ActionServer(
            self, NavigateToWaypoints, 'navigate_to_waypoints',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=cb_group)

        # ── Speed limit service + Nav2 speed limit topic ──────────────
        self._speed_srv = self.create_service(
            SetSpeedLimit, '/set_speed_limit', self._on_set_speed_limit,
            callback_group=cb_group)
        self._speed_pub = self.create_publisher(SpeedLimit, '/speed_limit', 10)

        self.get_logger().info(
            'Ready: action /navigate_to_waypoints, service /set_speed_limit '
            f'(max_speed={self._max_speed():.2f} m/s)')

    # ── Speed limit ───────────────────────────────────────────────────
    def _max_speed(self):
        return self.get_parameter('max_speed').value

    def _publish_speed_limit(self):
        limit = effective_speed_limit(self._max_speed(), self._goal_speed)
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = False  # absolute value in m/s
        msg.speed_limit = limit
        self._speed_pub.publish(msg)
        self.get_logger().info(f'Speed limit set to {limit:.2f} m/s')

    def _on_parameters_set(self, params):
        if any(p.name == 'max_speed' for p in params):
            self._publish_speed_limit()

    def _on_set_speed_limit(self, request, response):
        # The service is a thin wrapper around the parameter: set_parameters
        # returns rcl_interfaces/SetParametersResult, the same result
        # `ros2 param set` gets from the node's /set_parameters service.
        result = self.set_parameters(
            [Parameter('max_speed', Parameter.Type.DOUBLE,
                       float(request.max_speed))])[0]
        response.success = result.successful
        response.message = (
            f'max_speed = {request.max_speed:.2f} m/s' if result.successful
            else result.reason)
        return response

    # ── Action server callbacks ───────────────────────────────────────
    def _on_goal(self, goal_request):
        """Accept or reject a new goal before it starts executing."""
        if not goal_request.waypoints:
            self.get_logger().warning('Rejected goal: no waypoints')
            return GoalResponse.REJECT
        if goal_request.max_speed < 0.0:
            self.get_logger().warning('Rejected goal: negative max_speed')
            return GoalResponse.REJECT
        if not self._nav_client.server_is_ready():
            self.get_logger().warning(
                'Rejected goal: Nav2 navigate_to_pose is not available')
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._active_goal is not None:
                self.get_logger().warning(
                    'Rejected goal: another goal is running')
                return GoalResponse.REJECT
            self._active_goal = True  # reserved until execute finishes
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        try:
            return self._run_waypoints(goal_handle)
        finally:
            self._goal_speed = 0.0
            self._publish_speed_limit()
            with self._goal_lock:
                self._active_goal = None

    def _run_waypoints(self, goal_handle):
        request = goal_handle.request
        waypoints = list(request.waypoints)
        names = [f'wp_{i}' for i in range(len(waypoints))]
        period = self.get_parameter('feedback_period').value
        frame = self.get_parameter('default_frame').value

        self._goal_speed = request.max_speed
        self.get_logger().info(
            f'New goal: {len(waypoints)} waypoints, '
            f'max_speed={request.max_speed:.2f} m/s')

        result = NavigateToWaypoints.Result()
        # Shared with the Nav2 feedback callback (another executor thread)
        state = {'traveled': 0.0, 'last_xy': None, 'leg_remaining': None}

        def on_nav_feedback(msg):
            fb = msg.feedback
            xy = _xy(fb.current_pose)
            if state['last_xy'] is not None:
                state['traveled'] += planar_distance(state['last_xy'], xy)
            state['last_xy'] = xy
            state['leg_remaining'] = fb.distance_remaining

        for i, wp in enumerate(waypoints):
            if not wp.header.frame_id:
                wp.header.frame_id = frame
            self.get_logger().info(
                f'→ {names[i]}: ({wp.pose.position.x:.2f}, '
                f'{wp.pose.position.y:.2f}) in {wp.header.frame_id}')

            # Re-assert the limit for every leg: /speed_limit is a volatile
            # topic, so a controller that (re)started after the last
            # publication would not know it.
            self._publish_speed_limit()

            # 1) Send the goal to Nav2 and wait for accept/reject
            state['leg_remaining'] = None
            send_future = self._nav_client.send_goal_async(
                NavigateToPose.Goal(pose=wp), feedback_callback=on_nav_feedback)
            # Timeout: if Nav2 never answers (e.g. bt_navigator not active)
            # we must not block this goal forever.
            if not _wait(send_future, timeout=10.0):
                self.get_logger().error(
                    f'Nav2 did not answer the goal for {names[i]}')
                return self._finish(goal_handle, result, state, 'abort')
            nav_handle = send_future.result()
            if nav_handle is None or not nav_handle.accepted:
                self.get_logger().error(f'Nav2 rejected {names[i]}')
                return self._finish(goal_handle, result, state, 'abort')

            # 2) While Nav2 drives, publish our feedback every `period` s
            result_future = nav_handle.get_result_async()
            while not result_future.done():
                if goal_handle.is_cancel_requested:
                    _wait(nav_handle.cancel_goal_async(), timeout=5.0)
                    _wait(result_future, timeout=5.0)
                    self.get_logger().info('Goal canceled')
                    return self._finish(goal_handle, result, state, 'cancel')

                goal_handle.publish_feedback(
                    self._feedback(names[i], waypoints[i:], state))
                _wait(result_future, timeout=period)

            # 3) Check how Nav2 finished this leg
            nav_result = result_future.result()
            if nav_result.status != GoalStatus.STATUS_SUCCEEDED:
                err = nav_result.result
                self.get_logger().error(
                    f'Nav2 failed at {names[i]} (status {nav_result.status}, '
                    f'error {err.error_code}: {err.error_msg})')
                return self._finish(goal_handle, result, state, 'abort')

            result.visited_waypoints.append(names[i])
            self.get_logger().info(f'Reached {names[i]}')

        return self._finish(goal_handle, result, state, 'succeed')

    def _feedback(self, current_name, remaining_waypoints, state):
        """Build feedback; remaining_waypoints[0] is the current target."""
        # Remaining = Nav2's path length to the current waypoint (or the
        # straight line before Nav2's first feedback) + straight lines
        # between the waypoints after it.
        later = path_length([_xy(w) for w in remaining_waypoints])
        leg = state['leg_remaining']
        if leg is None:
            leg = (planar_distance(state['last_xy'],
                                   _xy(remaining_waypoints[0]))
                   if state['last_xy'] is not None else 0.0)
        remaining = leg + later

        fb = NavigateToWaypoints.Feedback()
        fb.current_waypoint = current_name
        fb.distance_remaining = remaining
        fb.percent_complete = percent_complete(state['traveled'], remaining)
        return fb

    def _finish(self, goal_handle, result, state, outcome):
        result.total_distance_m = state['traveled']
        result.success = outcome == 'succeed'
        # Final state of our goal: SUCCEEDED, ABORTED or CANCELED
        method, label = {'succeed': ('succeed', 'succeeded'),
                         'abort': ('abort', 'aborted'),
                         'cancel': ('canceled', 'canceled')}[outcome]
        getattr(goal_handle, method)()
        self.get_logger().info(
            f'Goal {label}: visited {len(result.visited_waypoints)} '
            f'waypoints, drove {result.total_distance_m:.2f} m')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = WaypointActionServer()
    # Several threads: the execute callback blocks while waiting for Nav2,
    # so other callbacks (Nav2 responses, feedback, service) need threads.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
