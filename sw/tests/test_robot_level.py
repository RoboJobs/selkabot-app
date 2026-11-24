"""Tests for leveling the camera with the wrist (level.py). No hardware."""

from __future__ import annotations

import pytest
from scipy.spatial.transform import Rotation

from sw.robot import level
from sw.robot.pose import camera_angles, level_rotation, rotation_to_sdk_euler
from sw.tests.fakes.fake_kinematics import FakeKinematics, _pose_to_joints


def _joints_for(pos, pan, tilt, roll):
    """Fake-IK joints of a camera at `pos` with the given pan, tilt and roll."""
    rot = (
        level_rotation(pan, tilt)
        @ Rotation.from_euler("z", roll, degrees=True).as_matrix()
    )
    return _pose_to_joints([*pos, *rotation_to_sdk_euler(rot)])


def test_level_keyframe_removes_roll_and_keeps_the_framing():
    fk = FakeKinematics().fk
    joints = _joints_for((600, 0, 400), pan=20, tilt=-10, roll=7.0)
    result = level.level_keyframe(fk, joints)
    pan, tilt, roll = camera_angles(*result.pose[3:])
    assert result.roll_before == pytest.approx(7.0, abs=1e-6)
    assert abs(roll) < 1e-3
    assert pan == pytest.approx(20, abs=1e-3)
    assert tilt == pytest.approx(-10, abs=1e-3)
    assert result.joints[:3] == joints[:3]  # the arm (reach) is untouched


def test_solve_wrist_reaches_a_requested_pan_and_tilt():
    fk = FakeKinematics().fk
    joints = _joints_for((600, 0, 400), pan=0, tilt=0, roll=0)
    solved = level.solve_wrist(fk, joints, pan=15.0, tilt=5.0)
    pan, tilt, roll = camera_angles(*fk(solved)[3:])
    assert (pan, tilt) == pytest.approx((15.0, 5.0), abs=1e-3)
    assert abs(roll) < 1e-3


def test_near_vertical_tilt_is_refused():
    fk = FakeKinematics().fk
    joints = _joints_for((600, 0, 400), pan=0, tilt=0, roll=0)
    with pytest.raises(level.LevelError, match="vertical"):
        level.solve_wrist(fk, joints, pan=0.0, tilt=89.0)


def test_a_wrist_that_cannot_change_the_orientation_fails():
    fixed = FakeKinematics().fk(_joints_for((600, 0, 400), pan=0, tilt=0, roll=10))

    def stuck_fk(joints):  # the wrist has no effect: roll stays 10deg
        return list(fixed)

    with pytest.raises(level.LevelError, match="could not level"):
        level.solve_wrist(stuck_fk, [0.0] * 6, pan=0.0, tilt=0.0)
