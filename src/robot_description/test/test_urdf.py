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
Consistency tests for the robot model (robot.urdf.xacro).

They catch mistakes that xacro and robot_state_publisher accept silently
but that break the simulation or the odometry later: a broken TF tree,
physically impossible inertias, wheels floating above the ground, or a
DiffDrive plugin whose wheel geometry differs from the URDF's.
"""

import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro

TOL = 1e-6


@pytest.fixture(scope='module')
def robot():
    path = os.path.join(get_package_share_directory('robot_description'),
                        'urdf', 'robot.urdf.xacro')
    return ET.fromstring(xacro.process_file(path).toxml())


def floats(text):
    return [float(v) for v in text.split()]


def joint(robot, name):
    element = robot.find(f"joint[@name='{name}']")
    assert element is not None, f'joint {name} not found'
    return element


def joint_xyz(robot, name):
    return floats(joint(robot, name).find('origin').get('xyz'))


def diff_drive(robot):
    for plugin in robot.iter('plugin'):
        if plugin.get('filename') == 'gz-sim-diff-drive-system':
            return plugin
    pytest.fail('gz-sim-diff-drive-system plugin not found')


# ── TF tree ───────────────────────────────────────────────────────────
def test_tf_tree_has_a_single_root_base_footprint(robot):
    links = {link.get('name') for link in robot.findall('link')}
    children = [j.find('child').get('link') for j in robot.findall('joint')]
    parents = {j.find('parent').get('link') for j in robot.findall('joint')}

    assert parents <= links and set(children) <= links
    # A tree: every link has at most one parent ...
    assert len(children) == len(set(children))
    # ... and exactly one link (the root) has none.
    assert links - set(children) == {'base_footprint'}


@pytest.mark.parametrize('frame', [
    'base_footprint',       # Nav2 robot_base_frame, odom child frame
    'base_link',
    'lidar_link',           # /scan frame_id
    'imu_link',             # /imu frame_id
    'camera_optical_link',  # /camera/image_raw frame_id
])
def test_frames_used_by_the_stack_exist(robot, frame):
    assert robot.find(f"link[@name='{frame}']") is not None


# ── Inertia ───────────────────────────────────────────────────────────
def test_inertias_are_physically_valid(robot):
    for link in robot.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue  # massless frame (e.g. base_footprint)
        name = link.get('name')
        mass = float(inertial.find('mass').get('value'))
        i = inertial.find('inertia')
        ixx, iyy, izz = (float(i.get(k)) for k in ('ixx', 'iyy', 'izz'))

        assert mass > 0.0, name
        assert min(ixx, iyy, izz) > 0.0, name
        # Principal moments of a real body obey the triangle inequality;
        # physics engines reject (or explode on) tensors that do not.
        assert ixx + iyy >= izz - TOL, name
        assert iyy + izz >= ixx - TOL, name
        assert izz + ixx >= iyy - TOL, name


# ── Wheels and DiffDrive plugin ───────────────────────────────────────
def test_wheels_spin_about_the_lateral_axis(robot):
    for side in ('left', 'right'):
        j = joint(robot, f'{side}_wheel_joint')
        assert j.get('type') == 'continuous'
        roll, pitch, yaw = floats(j.find('origin').get('rpy'))
        assert floats(j.find('axis').get('xyz')) == [0.0, 0.0, 1.0]
        # Joint z axis rotated by -90° about x → points to +y, so a
        # positive wheel speed drives the robot forward.
        assert math.isclose(roll, -math.pi / 2, abs_tol=TOL)
        assert math.isclose(pitch, 0.0) and math.isclose(yaw, 0.0)


def test_diff_drive_matches_wheel_geometry(robot):
    # The plugin turns /cmd_vel into wheel speeds and wheel speeds into
    # /odom using its own wheel_separation and wheel_radius. If they differ
    # from the physical wheels, the robot turns at a different rate than
    # commanded and the odometry drifts on every turn.
    plugin = diff_drive(robot)
    y_left = joint_xyz(robot, 'left_wheel_joint')[1]
    y_right = joint_xyz(robot, 'right_wheel_joint')[1]
    wheel = robot.find("link[@name='left_wheel']/collision/geometry/sphere")

    assert plugin.findtext('left_joint') == 'left_wheel_joint'
    assert plugin.findtext('right_joint') == 'right_wheel_joint'
    assert math.isclose(float(plugin.findtext('wheel_separation')),
                        y_left - y_right, abs_tol=TOL)
    assert math.isclose(float(plugin.findtext('wheel_radius')),
                        float(wheel.get('radius')), abs_tol=TOL)


def test_odometry_frames(robot):
    plugin = diff_drive(robot)
    assert plugin.findtext('frame_id') == 'odom'
    assert plugin.findtext('child_frame_id') == 'base_footprint'


def test_wheels_and_caster_touch_the_ground(robot):
    # base_footprint is on the ground (z = 0). A wheel center must be
    # exactly one radius above it; otherwise the robot rests tilted and
    # the Lidar scan plane hits the floor.
    base_z = joint_xyz(robot, 'base_footprint_joint')[2]
    for side in ('left', 'right'):
        radius = float(robot.find(
            f"link[@name='{side}_wheel']/collision/geometry/sphere"
        ).get('radius'))
        wheel_z = base_z + joint_xyz(robot, f'{side}_wheel_joint')[2]
        assert math.isclose(wheel_z, radius, abs_tol=TOL), side

    caster_radius = float(robot.find(
        "link[@name='caster_wheel']/collision/geometry/sphere").get('radius'))
    caster_z = base_z + joint_xyz(robot, 'caster_joint')[2]
    assert math.isclose(caster_z, caster_radius, abs_tol=TOL)
