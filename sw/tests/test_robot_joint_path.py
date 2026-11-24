"""Tests for joint-space interpolation. No hardware."""

from __future__ import annotations

import numpy as np
import pytest

from sw.robot import joint_path, schema

DESIGN = {
    "name": "demo",
    "tool_id": 1,
    "interpolation": "joint",
    "sample_period_ms": 8,
    "seed_joints": [0, 0, 0, 0, 0, 0],
    "orientation": {"mode": "angles"},
    "workspace": {"min": [-1, -1, -1], "max": [1, 1, 1]},
    "keyframes": [
        {"t": 0.0, "joints": [90, -90, 0, 30, -90, 45]},
        {"t": 2.0, "joints": [90, -10, 20, -60, -90, 45]},
        {"t": 4.0, "joints": [90, 71, 0, -181, -90, 45]},
    ],
}
KF = [kf["joints"] for kf in DESIGN["keyframes"]]


def _sample():
    d = schema.design_from_dict(DESIGN, "demo")
    return joint_path.sample_joint_path(d, KF)


def test_passes_through_every_keyframe_and_uses_the_sampling_rule():
    times, joints = _sample()
    assert len(times) == 501  # 4 s at 8 ms, N + 1 samples
    by_time = dict(zip(times, joints, strict=True))
    for kf in DESIGN["keyframes"]:
        assert by_time[kf["t"]] == pytest.approx(kf["joints"])


def test_starts_and_ends_at_rest():
    times, joints = _sample()
    arr = np.array(joints)
    assert np.abs(arr[1] - arr[0]).max() < 1e-3
    assert np.abs(arr[-1] - arr[-2]).max() < 1e-3


def test_monotone_keyframes_do_not_overshoot():
    _times, joints = _sample()
    arr = np.array(joints)
    j2 = arr[:, 1]  # -90 -> -10 -> 71, monotone
    assert np.all(np.diff(j2) >= -1e-9)
    assert j2.min() >= -90 - 1e-9 and j2.max() <= 71 + 1e-9


def test_needs_one_joint_vector_per_keyframe():
    d = schema.design_from_dict(DESIGN, "demo")
    with pytest.raises(ValueError):
        joint_path.sample_joint_path(d, KF[:2])
