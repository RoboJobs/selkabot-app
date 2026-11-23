"""Orientation maths for the level camera (sw/robot/pose.py). No hardware."""

import math

import numpy as np
import pytest

from sw.robot import pose


def test_level_rotation_is_a_proper_rotation():
    rng = np.random.default_rng(1)
    for _ in range(200):
        pan = rng.uniform(-720, 720)
        tilt = rng.uniform(-80, 80)
        r = pose.level_rotation(pan, tilt)
        assert np.allclose(r.T @ r, np.eye(3), atol=1e-12)
        assert np.linalg.det(r) == pytest.approx(1.0, abs=1e-12)


def test_roll_is_zero_for_every_level_orientation():
    rng = np.random.default_rng(7)
    for _ in range(500):
        r = pose.level_rotation(rng.uniform(-180, 180), rng.uniform(-80, 80))
        assert abs(pose.roll_error_deg(r)) < 1e-9


def test_image_right_axis_stays_horizontal_for_any_pan_and_tilt():
    rng = np.random.default_rng(2)
    for _ in range(1000):
        pan = rng.uniform(-180, 180)
        tilt = rng.uniform(-80, 80)
        r = pose.level_rotation(pan, tilt)
        assert abs(r[2, 0]) < 1e-12  # x_cam . z_base == 0
        assert pose.roll_error_deg(r) < 1e-9


def test_optical_axis_heading_and_elevation_match_pan_and_tilt():
    for pan, tilt in [(0, 0), (30, -10), (-135, 45), (200, 20)]:
        axis = pose.level_rotation(pan, tilt)[:, 2]
        heading = math.degrees(math.atan2(axis[1], axis[0]))
        elevation = math.degrees(math.asin(axis[2]))
        assert heading == pytest.approx(((pan + 180) % 360) - 180, abs=1e-9)
        assert elevation == pytest.approx(tilt, abs=1e-9)


@pytest.mark.parametrize(
    "target, expected_pan, expected_tilt",
    [
        ((1, 0, 0), 0.0, 0.0),  # ahead
        ((0, 1, 0), 90.0, 0.0),  # left
        ((1, 1, 0), 45.0, 0.0),  # diagonal
        ((1, 0, 1), 0.0, 45.0),  # up and ahead
        ((-1, 0, -1), 180.0, -45.0),  # behind and below
    ],
)
def test_look_at_angles_for_known_targets(target, expected_pan, expected_tilt):
    pan, tilt = pose.look_at_angles([0, 0, 0], target)
    assert pan == pytest.approx(expected_pan, abs=1e-9)
    assert tilt == pytest.approx(expected_tilt, abs=1e-9)


def test_look_at_points_the_optical_axis_at_the_target():
    rng = np.random.default_rng(3)
    for _ in range(200):
        pos = rng.uniform(-500, 500, 3)
        target = pos + rng.uniform(-400, 400, 3) + np.array([0, 0, 1e-3])
        pan, tilt = pose.look_at_angles(pos, target)
        r = pose.level_rotation(pan, tilt)
        assert pose.pointing_error_deg(r, pos, target) < 1e-9


def test_look_at_rejects_coincident_points():
    with pytest.raises(pose.DegenerateTarget):
        pose.look_at_angles([1, 2, 3], [1, 2, 3])
    with pytest.raises(pose.DegenerateTarget):
        pose.pointing_error_deg(np.eye(3), [0, 0, 0], [0, 0, 0])


def test_roll_error_equals_an_injected_roll():
    # A roll about the camera's own optical axis (z_cam) is what the level
    # constraint forbids. Injecting it must produce exactly that roll angle.
    roll = 0.3
    cos, sin = math.cos(math.radians(roll)), math.sin(math.radians(roll))
    about_z_cam = np.array([[cos, -sin, 0.0], [sin, cos, 0.0], [0.0, 0.0, 1.0]])
    for pan, tilt in [(25, 10), (-70, -60), (0, 80)]:
        rolled = pose.level_rotation(pan, tilt) @ about_z_cam
        assert pose.roll_error_deg(rolled) == pytest.approx(roll, abs=1e-9)


def test_sdk_euler_round_trip_for_random_rotations():
    rng = np.random.default_rng(4)
    for _ in range(500):
        angles = rng.uniform(-170, 170, 3)
        angles[1] = rng.uniform(-80, 80)  # avoid the gimbal singularity
        rotation = pose.sdk_euler_to_rotation(*angles)
        back = pose.rotation_to_sdk_euler(rotation)
        assert np.allclose(pose.sdk_euler_to_rotation(*back), rotation, atol=1e-9)


def test_sdk_euler_matches_fixed_axis_xyz_convention():
    # Rotation of 90 deg about fixed z after 180 deg about fixed x. This is the
    # convention confirmed on the simulator (Phase 0, Q1).
    r = pose.sdk_euler_to_rotation(180.0, 0.0, 90.0)
    tool_z = r[:, 2]
    assert np.allclose(tool_z, [0.0, 0.0, -1.0], atol=1e-12)


def test_unwrap_handles_wrap_across_180():
    unwrapped = pose.unwrap_deg([170.0, -175.0, -160.0])
    assert unwrapped == pytest.approx([170.0, 185.0, 200.0], abs=1e-9)


def test_unwrap_keeps_multiple_turns():
    values = [0.0, 150.0, 300.0, 450.0, 600.0]
    wrapped = [((v + 180) % 360) - 180 for v in values]
    assert pose.unwrap_deg(wrapped) == pytest.approx(values, abs=1e-9)
