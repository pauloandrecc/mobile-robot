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
Tests for the generated robot_interfaces Python classes.

The .action/.srv files are a contract between the waypoint server, the
CLI client and anyone calling them from the command line. These tests
pin that contract: renaming or retyping a field makes them fail here
instead of at runtime in a node.
"""

from geometry_msgs.msg import PoseStamped
import pytest
from rclpy.serialization import deserialize_message, serialize_message
from robot_interfaces.action import NavigateToWaypoints
from robot_interfaces.srv import SetSpeedLimit


# ── Field names and types ─────────────────────────────────────────────
@pytest.mark.parametrize('msg_type, expected', [
    (NavigateToWaypoints.Goal, {
        'waypoints': 'sequence<geometry_msgs/PoseStamped>',
        'max_speed': 'double',
    }),
    (NavigateToWaypoints.Result, {
        'success': 'boolean',
        'total_distance_m': 'double',
        'visited_waypoints': 'sequence<string>',
    }),
    (NavigateToWaypoints.Feedback, {
        'current_waypoint': 'string',
        'distance_remaining': 'double',
        'percent_complete': 'uint8',
    }),
    (SetSpeedLimit.Request, {'max_speed': 'double'}),
    (SetSpeedLimit.Response, {'success': 'boolean', 'message': 'string'}),
], ids=lambda v: v.__name__ if isinstance(v, type) else '')
def test_fields_and_types(msg_type, expected):
    assert msg_type.get_fields_and_field_types() == expected


def test_action_is_built_from_services_and_topics():
    # An action is not a new transport: rosidl generates two services
    # (send goal, get result), a feedback topic message and reuses the
    # standard cancel service and status topic.
    impl = NavigateToWaypoints.Impl
    send_goal = impl.SendGoalService.Request.get_fields_and_field_types()
    assert send_goal == {
        'goal_id': 'unique_identifier_msgs/UUID',
        'goal': 'robot_interfaces/NavigateToWaypoints_Goal',
    }
    feedback = impl.FeedbackMessage.get_fields_and_field_types()
    assert feedback['feedback'] == 'robot_interfaces/NavigateToWaypoints_Feedback'
    assert impl.GetResultService is not None
    assert impl.CancelGoalService is not None


# ── Default values ────────────────────────────────────────────────────
def test_goal_defaults():
    goal = NavigateToWaypoints.Goal()
    assert list(goal.waypoints) == []
    # 0.0 means "use the server's max_speed parameter" (see the .action)
    assert goal.max_speed == 0.0


def test_result_and_feedback_defaults():
    result = NavigateToWaypoints.Result()
    assert result.success is False
    assert list(result.visited_waypoints) == []
    assert NavigateToWaypoints.Feedback().percent_complete == 0


# ── Type and range checks ─────────────────────────────────────────────
# Field checks are off by default since ROS 2 Jazzy (they cost time on
# every assignment); check_fields=True (or ROS_PYTHON_CHECK_FIELDS=1)
# turns them on.
@pytest.mark.parametrize('value', [0, 100, 255])
def test_percent_complete_accepts_uint8(value):
    feedback = NavigateToWaypoints.Feedback(check_fields=True)
    feedback.percent_complete = value
    assert feedback.percent_complete == value


@pytest.mark.parametrize('value', [-1, 256, 50.0, '50'])
def test_percent_complete_rejects_non_uint8(value):
    feedback = NavigateToWaypoints.Feedback(check_fields=True)
    with pytest.raises(AssertionError):
        feedback.percent_complete = value


def test_max_speed_rejects_wrong_type():
    goal = NavigateToWaypoints.Goal(check_fields=True)
    with pytest.raises(AssertionError):
        goal.max_speed = 'fast'


def test_unchecked_uint8_overflow_wraps_silently():
    # Without checks an out-of-range value is serialized modulo 256.
    # That is why the server clamps percent_complete to 0-100 itself
    # (waypoint_math.percent_complete) before filling the message.
    feedback = NavigateToWaypoints.Feedback(check_fields=False)
    feedback.percent_complete = 300
    received = deserialize_message(serialize_message(feedback),
                                   NavigateToWaypoints.Feedback)
    assert received.percent_complete == 300 % 256


# ── Serialization round trip ──────────────────────────────────────────
def test_goal_serialization_round_trip():
    goal = NavigateToWaypoints.Goal(max_speed=0.15)
    for x, y in [(1.0, 0.0), (2.5, -1.5)]:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        goal.waypoints.append(pose)

    received = deserialize_message(serialize_message(goal),
                                   NavigateToWaypoints.Goal)
    assert received == goal
    assert [(p.pose.position.x, p.pose.position.y)
            for p in received.waypoints] == [(1.0, 0.0), (2.5, -1.5)]


def test_service_serialization_round_trip():
    response = SetSpeedLimit.Response(success=True, message='max_speed = 0.10')
    received = deserialize_message(serialize_message(response),
                                   SetSpeedLimit.Response)
    assert received == response
