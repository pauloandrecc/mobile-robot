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

"""Unit tests for robot_navigation.diff_drive_kinematics (no ROS needed)."""

import math

import pytest
from robot_navigation.diff_drive_kinematics import (body_twist,
                                                    integrate_pose,
                                                    integrate_pose_euler,
                                                    normalize_angle,
                                                    wheel_speeds)

# Same values as the URDF / DiffDrive plugin (test_urdf.py in
# robot_description checks that the two agree).
SEPARATION = 0.26
RADIUS = 0.05


def close(a, b, tol=1e-9):
    return math.isclose(a, b, abs_tol=tol)


def test_straight_line_wheels_turn_equally():
    left, right = wheel_speeds(0.3, 0.0, SEPARATION, RADIUS)
    assert close(left, right)
    assert close(left, 0.3 / RADIUS)  # 6 rad/s


def test_spin_in_place_wheels_turn_opposite():
    left, right = wheel_speeds(0.0, 1.0, SEPARATION, RADIUS)
    assert close(left, -right)
    assert right > 0.0  # turning left (CCW): right wheel goes forward


@pytest.mark.parametrize('v, w', [(0.3, 0.0), (0.0, 1.0), (0.2, -0.5),
                                  (-0.1, 0.8)])
def test_inverse_then_forward_kinematics_round_trip(v, w):
    left, right = wheel_speeds(v, w, SEPARATION, RADIUS)
    v2, w2 = body_twist(left, right, SEPARATION, RADIUS)
    assert close(v, v2)
    assert close(w, w2)


@pytest.mark.parametrize('angle, expected', [
    (0.0, 0.0),
    (math.pi / 2, math.pi / 2),
    (3 * math.pi / 2, -math.pi / 2),
    (-3 * math.pi / 2, math.pi / 2),
    (4 * math.pi + 0.1, 0.1),
])
def test_normalize_angle(angle, expected):
    assert close(normalize_angle(angle), expected)


def test_integrate_straight_line():
    x, y, theta = integrate_pose(0.0, 0.0, math.pi / 2, 0.5, 0.0, 2.0)
    assert close(x, 0.0) and close(y, 1.0) and close(theta, math.pi / 2)


def test_integrate_spin_in_place_does_not_move():
    x, y, theta = integrate_pose(1.0, 2.0, 0.0, 0.0, math.pi / 2, 1.0)
    assert close(x, 1.0) and close(y, 2.0)
    assert close(theta, math.pi / 2)


def test_integrate_quarter_circle():
    # v / w = 1 m radius, CCW, starting at the origin facing +x: after a
    # quarter turn the robot is at (1, 1) facing +y.
    x, y, theta = integrate_pose(0.0, 0.0, 0.0, 1.0, 1.0, math.pi / 2)
    assert close(x, 1.0) and close(y, 1.0) and close(theta, math.pi / 2)


def test_full_circle_returns_to_start():
    pose = (0.5, -0.5, 0.3)
    v, w, steps = 0.2, 0.4, 100
    dt = (2 * math.pi / w) / steps
    for _ in range(steps):
        pose = integrate_pose(*pose, v, w, dt)
    assert close(pose[0], 0.5, 1e-6) and close(pose[1], -0.5, 1e-6)
    assert close(normalize_angle(pose[2] - 0.3), 0.0, 1e-6)


def test_euler_converges_to_exact_with_small_steps():
    # Odometry at 20 Hz (the plugin's rate) on a tight turn: one Euler step
    # is noticeably off, many small steps approach the exact arc.
    v, w, total = 0.3, 1.0, 1.0
    exact = integrate_pose(0.0, 0.0, 0.0, v, w, total)

    def euler(steps):
        pose = (0.0, 0.0, 0.0)
        for _ in range(steps):
            pose = integrate_pose_euler(*pose, v, w, total / steps)
        return math.hypot(pose[0] - exact[0], pose[1] - exact[1])

    assert euler(1) > 0.02          # > 2 cm off after a single step
    assert euler(20) < euler(1) / 10
    assert euler(1000) < 1e-3
