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

"""Unit tests for robot_navigation.waypoint_math (no ROS needed)."""

import math

import pytest
from robot_navigation.waypoint_math import (effective_speed_limit,
                                            parse_waypoint, path_length,
                                            percent_complete,
                                            quaternion_to_yaw,
                                            yaw_to_quaternion)


@pytest.mark.parametrize('yaw', [0.0, math.pi / 2, -math.pi / 2, 3.0])
def test_yaw_quaternion_round_trip(yaw):
    z, w = yaw_to_quaternion(yaw)
    assert math.isclose(z * z + w * w, 1.0)  # unit quaternion
    assert math.isclose(quaternion_to_yaw(z, w), yaw, abs_tol=1e-9)


def test_yaw_90_degrees():
    z, w = yaw_to_quaternion(math.pi / 2)
    assert math.isclose(z, math.sqrt(0.5))
    assert math.isclose(w, math.sqrt(0.5))


def test_parse_waypoint():
    assert parse_waypoint('1.5,-2') == (1.5, -2.0, 0.0)
    x, y, yaw = parse_waypoint(' 1 , 2 , 90 ')
    assert (x, y) == (1.0, 2.0)
    assert math.isclose(yaw, math.pi / 2)


@pytest.mark.parametrize('bad', ['1', '1,2,3,4', 'a,b', ''])
def test_parse_waypoint_invalid(bad):
    with pytest.raises(ValueError):
        parse_waypoint(bad)


def test_path_length():
    assert path_length([]) == 0.0
    assert path_length([(0, 0)]) == 0.0
    assert math.isclose(path_length([(0, 0), (3, 4), (3, 0)]), 9.0)


def test_percent_complete():
    assert percent_complete(0.0, 0.0) == 0
    assert percent_complete(0.0, 5.0) == 0
    assert percent_complete(2.5, 2.5) == 50
    assert percent_complete(5.0, 0.0) == 100


def test_effective_speed_limit():
    assert effective_speed_limit(0.3, 0.0) == 0.3   # no per-goal limit
    assert effective_speed_limit(0.3, 0.1) == 0.1   # goal is stricter
    assert effective_speed_limit(0.2, 0.4) == 0.2   # parameter is stricter
