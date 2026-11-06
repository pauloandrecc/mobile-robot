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
Differential drive kinematics and odometry (no ROS dependencies).

This is the model the gz-sim DiffDrive system applies to this robot:
/cmd_vel (v, w) → wheel speeds, and wheel speeds → /odom pose. Written
out in plain Python so it can be unit tested and reused (e.g. to check
odometry offline from a bag).

Conventions (REP 103): x forward, y left, theta counter-clockwise,
v in m/s, w in rad/s, wheel speeds in rad/s (positive = forward).

    separation: distance between the two wheel contact points (m)
    radius:     wheel radius (m)
"""

import math


def wheel_speeds(v, w, separation, radius):
    """
    Inverse kinematics: body twist (v, w) → wheel speeds (left, right).

    Each wheel moves along the ground at v ∓ w·separation/2 (the inner
    wheel of a turn is slower), and a wheel rolling without slipping
    turns at ground_speed / radius.
    """
    half = separation / 2.0
    return (v - w * half) / radius, (v + w * half) / radius


def body_twist(left, right, separation, radius):
    """
    Forward kinematics: wheel speeds (left, right) → body twist (v, w).

    v is the mean of the two wheels' ground speeds; w is their difference
    divided by the separation (how fast one side overtakes the other).
    """
    v_left, v_right = left * radius, right * radius
    return (v_left + v_right) / 2.0, (v_right - v_left) / separation


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def integrate_pose(x, y, theta, v, w, dt):
    """
    Dead reckoning: pose after driving at a constant (v, w) for dt seconds.

    With constant v and w the robot follows a circular arc of radius v/w,
    which has a closed-form solution (exact integration). For w ≈ 0 the
    arc degenerates into a straight line and the formula would divide by
    zero, so that case is handled separately.
    """
    if abs(w) < 1e-9:
        return (x + v * dt * math.cos(theta),
                y + v * dt * math.sin(theta),
                theta)
    new_theta = theta + w * dt
    r = v / w
    return (x + r * (math.sin(new_theta) - math.sin(theta)),
            y - r * (math.cos(new_theta) - math.cos(theta)),
            normalize_angle(new_theta))


def integrate_pose_euler(x, y, theta, v, w, dt):
    """
    First-order (Euler) step: move straight along theta, then rotate.

    Cheaper approximation used by many odometry implementations. Its error
    grows with w·dt, so it needs small time steps on tight turns.
    """
    return (x + v * dt * math.cos(theta),
            y + v * dt * math.sin(theta),
            normalize_angle(theta + w * dt))
