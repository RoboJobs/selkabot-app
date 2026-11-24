"""Path planning: channels, keyframe velocities, sampling rule (no hardware)."""

import numpy as np
import pytest

from sw.robot import path, pose
from sw.robot.schema import design_from_dict


def _angles(keyframes, period=8, anchor=None):
    raw = {
        "name": "path_demo",
        "tool_id": 1,
        "sample_period_ms": period,
        "seed_joints": [0, -90, 90, -90, -90, 0],
        "orientation": {"mode": "angles"},
        "workspace": {"min": [-5000] * 3, "max": [5000] * 3},
        "keyframes": keyframes,
    }
    if anchor is not None:
        raw["anchor"] = anchor
    return design_from_dict(raw, "path_demo")


def _target(keyframes, target=None, period=8):
    raw = {
        "name": "path_demo",
        "tool_id": 1,
        "sample_period_ms": period,
        "seed_joints": [0, -90, 90, -90, -90, 0],
        "orientation": {"mode": "target", **({"target": target} if target else {})},
        "workspace": {"min": [-5000] * 3, "max": [5000] * 3},
        "keyframes": keyframes,
    }
    return design_from_dict(raw, "path_demo")


THREE = [
    {"t": 0.0, "pos": [600, -400, 450], "pan": 20, "tilt": -5},
    {"t": 1.2, "pos": [600, 400, 450], "pan": -20, "tilt": -5},
    {"t": 2.0, "pos": [650, 400, 700], "pan": -20, "tilt": 5},
]
TWO = [
    {"t": 0.0, "pos": [600, -400, 450], "pan": 20, "tilt": -5},
    {"t": 2.0, "pos": [600, 400, 450], "pan": -20, "tilt": -5},
]


@pytest.mark.parametrize("keyframes", [TWO, THREE])
def test_path_passes_through_every_keyframe(keyframes):
    d = _angles(keyframes)
    traj = path.build_trajectory(d)
    for kf in d.keyframes:
        s = traj.sample(kf.t)
        assert np.allclose(s.pos, kf.pos, atol=1e-6)
        assert s.pan == pytest.approx(kf.pan, abs=1e-6)
        assert s.tilt == pytest.approx(kf.tilt, abs=1e-6)


def test_two_keyframes_build_a_minimum_jerk_segment():
    d = _angles(TWO)
    traj = path.build_trajectory(d)
    mid = traj.sample(1.0).pos
    assert np.allclose(mid, [600, 0, 450], atol=1e-6)  # symmetric midpoint


def test_velocity_and_acceleration_are_zero_at_the_ends():
    d = _angles(THREE)
    traj = path.build_trajectory(d)
    for t in (0.0, d.keyframes[-1].t):
        eps = 1e-4
        v = np.asarray(traj.sample(t + eps).pos) - np.asarray(traj.sample(t).pos)
        assert np.linalg.norm(v) / eps < 1e-2


def test_acceleration_is_zero_at_every_interior_keyframe():
    # Checked on the segment polynomials, not by finite differences: the jerk
    # jumps at keyframes, so a central second difference straddling a keyframe
    # reads an O(h) artifact even though the acceleration is exactly zero.
    d = _angles(THREE)
    traj = path.build_trajectory(d)
    for name in ("x", "y", "z", "pan", "tilt"):
        segments = traj._channels[name]._segments
        for h, coeffs in segments:
            second = np.polyder(coeffs[::-1], 2)
            assert np.polyval(second, 0.0) == pytest.approx(0.0, abs=1e-9)
            assert np.polyval(second, 1.0) == pytest.approx(0.0, abs=1e-9)


def test_locality_changing_one_keyframe_only_changes_its_segments():
    base = _angles(THREE)
    changed_kfs = [dict(kf) for kf in THREE]
    changed_kfs[1] = {**THREE[1], "pos": [600, 420, 450]}
    changed = _angles(changed_kfs)
    a, b = path.build_trajectory(base), path.build_trajectory(changed)
    # Before keyframe 1 (t < 1.2) only segment 0 is involved, so the first
    # segment's end changes but the path far from keyframe 1 is untouched.
    assert a.sample(0.0).pos == pytest.approx(b.sample(0.0).pos, abs=1e-9)
    assert a.sample(2.0).pos == pytest.approx(b.sample(2.0).pos, abs=1e-9)


def test_pchip_defaults_have_no_overshoot_for_monotone_keyframes():
    d = _angles(
        [
            {"t": 0.0, "pos": [0, 0, 0], "pan": 0, "tilt": 0},
            {"t": 0.6, "pos": [100, 0, 0], "pan": 10, "tilt": 0},
            {"t": 1.2, "pos": [110, 0, 0], "pan": 20, "tilt": 0},
            {"t": 2.0, "pos": [400, 0, 0], "pan": 30, "tilt": 0},
        ]
    )
    traj = path.build_trajectory(d)
    xs = [traj.sample(t).pos[0] for t in np.linspace(0, 2.0, 801)]
    assert max(xs) <= 400.0 + 1e-6
    assert min(xs) >= -1e-6


def test_stop_flag_gives_zero_velocity_at_that_keyframe():
    kfs = [dict(kf) for kf in THREE]
    kfs[1] = {**THREE[1], "stop": True}
    d = _angles(kfs)
    traj = path.build_trajectory(d)
    eps = 1e-4
    v = (
        np.linalg.norm(
            np.asarray(traj.sample(1.2 + eps).pos) - np.asarray(traj.sample(1.2).pos)
        )
        / eps
    )
    assert v < 1e-2


def test_velocity_override_is_honoured_at_the_keyframe():
    kfs = [dict(kf) for kf in TWO]
    kfs = [
        kfs[0],
        {"t": 1.0, "pos": [600, 0, 450], "pan": 0, "tilt": -5, "vel": {"y": 300.0}},
        kfs[1],
    ]
    d = _angles(kfs)
    traj = path.build_trajectory(d)
    eps = 1e-5
    v_y = (traj.sample(1.0 + eps).pos[1] - traj.sample(1.0).pos[1]) / eps
    assert v_y == pytest.approx(300.0, rel=1e-3)


def test_sample_count_rule_is_exact():
    assert path.sample_count(2.0, 8) == 251  # 250 steps + 1
    assert path.sample_count(1.2, 8) == 151  # no float-noise extra sample
    assert path.sample_count(2.003, 8) == 252  # T not a multiple: extra sample


def test_last_sample_equals_final_keyframe_when_t_is_not_a_multiple():
    kfs = [dict(kf) for kf in TWO]
    kfs[1] = {**TWO[1], "t": 2.003}
    d = _angles(kfs)
    samples = path.sample_path(d)
    assert len(samples) == 252
    assert samples[-1].t == pytest.approx(2.003)
    assert np.allclose(samples[-1].pos, [600, 400, 450], atol=1e-9)
    spacing = np.diff([s.t for s in samples])
    assert np.allclose(spacing[:-1], 0.008, atol=1e-12)


def test_first_sample_is_at_time_zero_and_times_are_uniform():
    samples = path.sample_path(_angles(THREE))
    assert samples[0].t == 0.0
    spacing = np.diff([s.t for s in samples])
    assert np.allclose(spacing, 0.008, atol=1e-12)


def test_explicit_pan_values_are_literal_not_shortest_path():
    kfs = [
        {"t": 0.0, "pos": [600, 0, 450], "pan": 170, "tilt": 0},
        {"t": 2.0, "pos": [600, 0, 450], "pan": -170, "tilt": 0},
    ]
    samples = path.sample_path(_angles(kfs))
    assert samples[-1].pan == pytest.approx(-170.0)
    assert samples[0].pan == pytest.approx(170.0)


def test_target_mode_points_the_optical_axis_at_the_target():
    d = _target(
        [{"t": 0.0, "pos": [600, -400, 450]}, {"t": 2.0, "pos": [600, 400, 450]}],
        target=[0, 0, 300],
    )
    for s in path.sample_path(d)[::25]:
        r = pose.level_rotation(s.pan, s.tilt)
        assert pose.pointing_error_deg(r, s.pos, s.target) < 1e-9


def test_per_keyframe_target_trajectory_interpolates_the_target():
    d = _target(
        [
            {"t": 0.0, "pos": [600, -400, 450], "target": [0, 0, 300]},
            {"t": 2.0, "pos": [600, 400, 450], "target": [0, 200, 300]},
        ]
    )
    samples = path.sample_path(d)
    assert samples[0].target == pytest.approx((0.0, 0.0, 300.0), abs=1e-9)
    assert samples[-1].target == pytest.approx((0.0, 200.0, 300.0), abs=1e-9)
    assert any(abs(s.target[1] - 100.0) < 5 for s in samples)


def test_pan_stays_continuous_when_the_path_circles_the_target():
    # A circle of radius 400 mm around the origin; the camera looks at the origin.
    keyframes = []
    for i in range(9):
        ang = np.radians(i * 45)
        keyframes.append(
            {"t": i * 0.5, "pos": [400 * np.cos(ang), 400 * np.sin(ang), 300]}
        )
    d = _target(keyframes, target=[0, 0, 0])
    pans = [s.pan for s in path.sample_path(d)]
    steps = np.abs(np.diff(pans))
    assert max(steps) < 2.0  # no 360 degree jumps
    assert pans[-1] - pans[0] == pytest.approx(360.0, abs=5.0)  # one full turn


def test_every_sample_is_level_in_both_modes():
    for d in (
        _angles(THREE),
        _target(
            [{"t": 0.0, "pos": [600, -400, 450]}, {"t": 2.0, "pos": [600, 400, 450]}],
            target=[0, 0, 300],
        ),
    ):
        for s in path.sample_path(d)[::10]:
            assert pose.roll_error_deg(pose.level_rotation(s.pan, s.tilt)) < 1e-9


def test_sdk_pose_has_six_values_and_the_position_of_the_sample():
    s = path.sample_path(_angles(TWO))[0]
    pose6 = path.to_sdk_pose(s)
    assert len(pose6) == 6
    assert pose6[:3] == pytest.approx(list(s.pos))


def test_to_sdk_pose_with_portrait_roll():
    s = path.Sample(0.0, (500.0, 100.0, 300.0), 25.0, -10.0, None)
    landscape, portrait = path.to_sdk_pose(s), path.to_sdk_pose(s, 90.0)
    assert landscape == path.to_sdk_pose(s, 0.0)  # default unchanged
    assert portrait[:3] == landscape[:3]
    pan, tilt, roll = pose.camera_angles(*portrait[3:])
    assert (pan, tilt, roll) == pytest.approx((25.0, -10.0, 90.0), abs=1e-6)
