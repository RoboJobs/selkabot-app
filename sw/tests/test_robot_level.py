"""Tests for leveling the camera with the wrist (level.py). No hardware."""

from __future__ import annotations

import pytest
from scipy.spatial.transform import Rotation

from sw.robot import level
from sw.robot.pose import camera_angles, level_rotation, rotation_to_sdk_euler
from sw.tests.fakes.fake_kinematics import FakeKinematics, _pose_to_joints
from sw.tests.fakes.wrist_kinematics import (
    SINGULAR_PORTRAIT_MOUNT,
    WristFK,
    j5_margin_deg,
)


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


def test_solve_wrist_accepts_a_small_residual_within_the_given_tolerances():
    """Near the wrist singularity the solve stops short of exact; the
    validator accepts that within the design's tolerances."""
    fixed = FakeKinematics().fk(_joints_for((600, 0, 400), pan=0, tilt=0, roll=0.09))

    def stuck_fk(joints):  # roll stays 0.09deg off, pan/tilt exact
        return list(fixed)

    with pytest.raises(level.LevelError, match="roll off by 0.09"):
        level.solve_wrist(stuck_fk, [0.0] * 6, pan=0.0, tilt=0.0)  # exact default
    solved = level.solve_wrist(
        stuck_fk, [0.0] * 6, pan=0.0, tilt=0.0, roll_tol_deg=0.5, point_tol_deg=0.1
    )
    assert len(solved) == 6
    with pytest.raises(level.LevelError, match="pan/tilt off"):
        level.solve_wrist(stuck_fk, [0.0] * 6, pan=0.3, tilt=0.0, roll_tol_deg=0.5)


# -- portrait (roll 90) with a real spherical wrist (WristFK) -----------------

PORTRAIT_SEED = [0.0, 0.0, 0.0, 0.0, -90.0, 0.0]


def _wrist_joints(kin, pan, tilt, roll, pos=(600.0, 0.0, 400.0)):
    pose = [*pos, *rotation_to_sdk_euler(level_rotation(pan, tilt, roll))]
    return kin.ik(pose, PORTRAIT_SEED)


@pytest.mark.parametrize("guess_offset", [None, (90.0, 0.0, 0.0), (0.0, 0.0, -90.0)])
def test_solve_wrist_reaches_portrait_from_a_landscape_pose(guess_offset):
    kin = WristFK()
    joints = _wrist_joints(kin, pan=10, tilt=-5, roll=0)
    guess = None
    if guess_offset is not None:
        guess = [j + o for j, o in zip(joints[3:], guess_offset, strict=True)]
    solved = level.solve_wrist(kin.fk, joints, 10.0, -5.0, guess, roll=90.0)
    pan, tilt, roll = camera_angles(*kin.fk(solved)[3:])
    assert (pan, tilt, roll) == pytest.approx((10.0, -5.0, 90.0), abs=1e-3)
    assert solved[:3] == joints[:3]  # J1-J3 untouched


def test_solve_wrist_returns_to_landscape_from_portrait():
    kin = WristFK()
    joints = _wrist_joints(kin, pan=-20, tilt=8, roll=90)
    solved = level.solve_wrist(kin.fk, joints, -20.0, 8.0, roll=0.0)
    assert camera_angles(*kin.fk(solved)[3:])[2] == pytest.approx(0.0, abs=1e-3)


def test_roll_target_is_wrapped_and_upside_down_is_not_portrait():
    kin = WristFK()
    joints = _wrist_joints(kin, pan=0, tilt=0, roll=90)
    # roll -270 is the same as +90: already solved, nothing moves.
    same = level.solve_wrist(kin.fk, joints, 0.0, 0.0, roll=-270.0)
    assert same == pytest.approx(joints, abs=1e-6)
    # an upside-down portrait (roll -90) does not satisfy target +90.
    upside_down = _wrist_joints(kin, pan=0, tilt=0, roll=-90)
    _, _, roll = camera_angles(*kin.fk(upside_down)[3:])
    assert abs(roll - 90.0) > 170.0


def test_level_keyframe_to_portrait_keeps_the_view_and_reports_the_move():
    kin = WristFK()
    joints = _wrist_joints(kin, pan=15, tilt=-10, roll=3.0)
    result = level.level_keyframe(kin.fk, joints, roll=90.0)
    pan, tilt, roll = camera_angles(*result.pose[3:])
    assert (pan, tilt, roll) == pytest.approx((15.0, -10.0, 90.0), abs=1e-3)
    assert result.roll_before == pytest.approx(3.0, abs=1e-6)
    assert result.moved_mm > 1.0  # the camera is off the flange: it moves


def test_a_singular_portrait_wrist_may_solve_but_has_no_j5_margin():
    """No new solver contract: near J5 = 0 the solver may succeed; rejecting
    the result is check_wrist_singularity's job (see test_robot_validate)."""
    kin = WristFK(mount=SINGULAR_PORTRAIT_MOUNT)
    joints = _wrist_joints(kin, pan=0, tilt=0, roll=0)
    try:
        solved = level.solve_wrist(kin.fk, joints, 0.0, 0.0, roll=90.0)
    except level.LevelError:
        return
    assert camera_angles(*kin.fk(solved)[3:])[2] == pytest.approx(90.0, abs=1e-3)
    assert j5_margin_deg(solved) < 15.0
