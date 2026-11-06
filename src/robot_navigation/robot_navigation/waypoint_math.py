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
Pure math helpers for waypoint navigation (no ROS dependencies).

Kept apart from the nodes so they can be unit tested with plain pytest.
"""

import math


def yaw_to_quaternion(yaw):
    """
    Return the (z, w) quaternion components of a rotation about +Z.

    A planar robot only rotates about Z, so x = y = 0 and
    q = (0, 0, sin(yaw / 2), cos(yaw / 2)).
    """
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_to_yaw(z, w):
    """Inverse of yaw_to_quaternion for a rotation about +Z only."""
    return 2.0 * math.atan2(z, w)


def parse_waypoint(text):
    """
    Parse 'x,y' or 'x,y,yaw_deg' into (x, y, yaw_rad).

    Raises ValueError on malformed input.
    """
    parts = [p.strip() for p in text.split(',')]
    if len(parts) not in (2, 3):
        raise ValueError(
            f"waypoint '{text}' must be 'x,y' or 'x,y,yaw_deg'")
    x, y = float(parts[0]), float(parts[1])
    yaw = math.radians(float(parts[2])) if len(parts) == 3 else 0.0
    return x, y, yaw


def planar_distance(a, b):
    """Euclidean distance between two (x, y) points."""
    return math.hypot(b[0] - a[0], b[1] - a[1])


def path_length(points):
    """Length of the polyline through a list of (x, y) points."""
    return sum(planar_distance(points[i], points[i + 1])
               for i in range(len(points) - 1))


def percent_complete(traveled, remaining):
    """
    Progress in percent (0-100) from distance traveled and remaining.

    The total is estimated as traveled + remaining, so it adapts when
    Nav2 replans around obstacles and the path gets longer or shorter.
    """
    total = traveled + remaining
    if total <= 0.0:
        return 0
    return max(0, min(100, int(round(100.0 * traveled / total))))


def effective_speed_limit(param_speed, goal_speed):
    """
    Speed limit to apply: the global parameter, capped by the goal's one.

    goal_speed <= 0 means "no per-goal limit".
    """
    if goal_speed > 0.0:
        return min(param_speed, goal_speed)
    return param_speed
