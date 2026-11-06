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
Command-line client for the /navigate_to_waypoints action.

Usage:
    ros2 run robot_navigation navigation_client 1.0,0.0 2.0,1.5,90
    ros2 run robot_navigation navigation_client --max-speed 0.15 1,0 0,0,180

Each waypoint is 'x,y' or 'x,y,yaw_deg' in the map frame.
Ctrl+C cancels the goal (the robot stops) and waits for the result.
"""

import argparse
import sys

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from robot_interfaces.action import NavigateToWaypoints

from robot_navigation.waypoint_math import parse_waypoint, yaw_to_quaternion

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
}


def build_goal(waypoints, frame, max_speed):
    """Build a NavigateToWaypoints goal from (x, y, yaw) tuples."""
    goal = NavigateToWaypoints.Goal()
    goal.max_speed = max_speed
    for x, y, yaw in waypoints:
        pose = PoseStamped()
        # stamp stays 0 = "latest available transform". Stamping with this
        # node's clock would break in simulation: Nav2 runs on sim time and
        # would look up TF at a wall-clock instant far in the future.
        pose.header.frame_id = frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z, pose.pose.orientation.w = (
            yaw_to_quaternion(yaw))
        goal.waypoints.append(pose)
    return goal


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Send waypoints to /navigate_to_waypoints')
    parser.add_argument(
        'waypoints', nargs='+', type=parse_waypoint,
        help="waypoints as 'x,y' or 'x,y,yaw_deg'")
    parser.add_argument(
        '--max-speed', type=float, default=0.0,
        help='per-goal speed limit in m/s (0 = server default)')
    parser.add_argument(
        '--frame', default='map', help='frame of the waypoints')
    # Negative numbers (e.g. -1,2) look like options to argparse; '--'
    # before the waypoints also works.
    return parser.parse_args(argv)


class NavigationClient(Node):
    """Send one NavigateToWaypoints goal and print its progress."""

    def __init__(self):
        super().__init__('navigation_client')
        self._client = ActionClient(
            self, NavigateToWaypoints, 'navigate_to_waypoints')

    def run(self, goal):
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available')
            return 1

        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                'Goal rejected (see the server log for the reason)')
            return 1
        self.get_logger().info('Goal accepted, Ctrl+C to cancel')

        result_future = goal_handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result_future)
        except KeyboardInterrupt:
            # rclpy was initialized without its signal handler, so Ctrl+C
            # reaches us as KeyboardInterrupt and the node is still alive to
            # ask the server to cancel.
            self.get_logger().info('Canceling goal...')
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            rclpy.spin_until_future_complete(self, result_future)

        response = result_future.result()
        result = response.result
        status = STATUS_NAMES.get(response.status, str(response.status))
        self.get_logger().info(
            f'Finished: {status} | success={result.success} | '
            f'distance={result.total_distance_m:.2f} m | '
            f'visited={list(result.visited_waypoints)}')
        return 0 if result.success else 1

    def _on_feedback(self, msg):
        fb = msg.feedback
        self.get_logger().info(
            f'{fb.current_waypoint}: {fb.distance_remaining:.2f} m left, '
            f'{fb.percent_complete}%')


def main(args=None):
    # Separate ROS arguments (--ros-args ...) from ours
    argv = remove_ros_args(args if args is not None else sys.argv)[1:]
    options = parse_args(argv)

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = NavigationClient()
    goal = build_goal(options.waypoints, options.frame, options.max_speed)
    try:
        code = node.run(goal)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
