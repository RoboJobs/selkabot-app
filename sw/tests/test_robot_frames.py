"""Frame conversion between authoring frame A and base frame B (no hardware)."""

import numpy as np
import pytest

from sw.robot import frames, pose
from sw.robot.schema import Anchor, design_from_dict


def _design(anchor=None, target_mode=False):
    base = {
        "name": "frames_demo",
        "tool_id": 1,
        "sample_period_ms": 8,
        "seed_joints": [0, -90, 90, -90, -90, 0],
        "workspace": {"min": [-2000, -2000, -2000], "max": [2000, 2000, 2000]},
    }
    if anchor is not None:
        base["anchor"] = anchor
    if target_mode:
        base["orientation"] = {"mode": "target", "target": [0, 0, 300]}
        base["keyframes"] = [
            {"t": 0.0, "pos": [600, -400, 450]},
            {"t": 2.0, "pos": [600, 400, 450]},
        ]
    else:
        base["orientation"] = {"mode": "angles"}
        base["keyframes"] = [
            {"t": 0.0, "pos": [600, -400, 450], "pan": 20, "tilt": -5},
            {"t": 2.0, "pos": [600, 400, 450], "pan": -20, "tilt": -5},
        ]
    return design_from_dict(base)


def test_identity_anchor_is_a_no_op():
    d = _design()
    converted = frames.design_to_base(d)
    assert converted.keyframes[0].pos == d.keyframes[0].pos
    assert converted.keyframes[0].pan == d.keyframes[0].pan


def test_yaw_rotates_positions_and_shifts_pan():
    anchor = {"pos": [100, 0, 0], "yaw_deg": 90}
    d = frames.design_to_base(_design(anchor=anchor))
    kf = d.keyframes[0]
    # p_A = (600, -400, 450): Rz(90)·p_A = (400, 600, 450), then + (100, 0, 0).
    assert kf.pos == pytest.approx((500.0, 600.0, 450.0), abs=1e-9)
    assert kf.pan == pytest.approx(20.0 + 90.0)


def test_target_is_converted_with_the_same_transform():
    anchor = {"pos": [10, 20, 30], "yaw_deg": 35}
    d = frames.design_to_base(_design(anchor=anchor, target_mode=True))
    expected = frames.authoring_to_base([0, 0, 300], Anchor((10, 20, 30), 35))
    assert np.allclose(d.orientation.target, expected)


def test_base_to_authoring_inverts_authoring_to_base():
    anchor = Anchor((5, -7, 11), 123.0)
    rng = np.random.default_rng(5)
    for _ in range(50):
        p = rng.uniform(-1000, 1000, 3)
        back = frames.base_to_authoring(frames.authoring_to_base(p, anchor), anchor)
        assert np.allclose(back, p, atol=1e-9)


def test_level_property_is_preserved_by_a_yaw_anchor():
    anchor = Anchor((0, 0, 0), 77.0)
    for pan, tilt in [(0, 0), (40, -15), (-130, 60)]:
        pan_b = pan + anchor.yaw_deg
        assert pose.roll_error_deg(pose.level_rotation(pan_b, tilt)) < 1e-9


def test_workspace_is_checked_in_the_authoring_frame():
    anchor = {"pos": [0, 0, 0], "yaw_deg": 90}
    d = design_from_dict(
        {
            "name": "ws_demo",
            "tool_id": 1,
            "sample_period_ms": 8,
            "seed_joints": [0, -90, 90, -90, -90, 0],
            "anchor": anchor,
            "orientation": {"mode": "angles"},
            "workspace": {"min": [-100, -100, 0], "max": [100, 100, 500]},
            "keyframes": [
                {"t": 0.0, "pos": [50, 80, 100], "pan": 0, "tilt": 0},
                {"t": 1.0, "pos": [50, 80, 100], "pan": 0, "tilt": 0},
            ],
        },
        "ws_demo",
    )
    # A point inside the box in A, rotated by 90 deg into B, is still inside.
    inside_b = frames.authoring_to_base([50, 80, 100], Anchor((0, 0, 0), 90))
    # A point well outside the box in A.
    outside_b = frames.authoring_to_base([500, 0, 100], Anchor((0, 0, 0), 90))
    mask = frames.within_workspace(np.array([inside_b, outside_b]), d)
    assert mask.tolist() == [True, False]
