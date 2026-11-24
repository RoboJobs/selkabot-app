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


# -- cruise profile (ramp_fraction) ------------------------------------------------


@pytest.mark.parametrize("ramp", [0.1, 0.25, 0.5])
def test_cruise_shape_runs_0_to_1_monotonically_and_smoothly(ramp):
    taus = np.linspace(0, 1, 2001)
    x = np.array([joint_path.cruise_shape(t, ramp) for t in taus])
    assert x[0] == pytest.approx(0) and x[-1] == pytest.approx(1)
    assert np.all(np.diff(x) >= -1e-12)
    v = np.gradient(x, taus)
    assert abs(v[0]) < 1e-2 and abs(v[-1]) < 1e-2  # rest at both ends
    assert v.max() == pytest.approx(1 / (1 - ramp), rel=1e-3)  # peak/average


def test_cruise_peak_speed_is_lower_than_the_quintic_for_the_same_time():
    raw = {**DESIGN, "keyframes": [DESIGN["keyframes"][0], DESIGN["keyframes"][2]]}
    kf = [raw["keyframes"][0]["joints"], raw["keyframes"][1]["joints"]]
    quintic = schema.design_from_dict(raw, "demo")
    cruise = schema.design_from_dict({**raw, "ramp_fraction": 0.25}, "demo")

    def peak(design):
        _t, joints = joint_path.sample_joint_path(design, kf)
        return np.abs(np.diff(np.array(joints), axis=0)).max()

    assert peak(cruise) / peak(quintic) == pytest.approx((4 / 3) / 1.875, rel=1e-2)


def test_ramp_fraction_only_reshapes_rest_to_rest_segments():
    d = schema.design_from_dict({**DESIGN, "ramp_fraction": 0.25}, "demo")
    times, joints = joint_path.sample_joint_path(d, KF)
    by_time = dict(zip(times, joints, strict=True))
    for kf in DESIGN["keyframes"]:  # still passes through every keyframe
        assert by_time[kf["t"]] == pytest.approx(kf["joints"])


@pytest.mark.parametrize("bad", [0.0, 0.6, -0.1])
def test_ramp_fraction_range_is_checked(bad):
    with pytest.raises(schema.SchemaError) as exc:
        schema.design_from_dict({**DESIGN, "ramp_fraction": bad}, "demo")
    assert exc.value.field == "ramp_fraction"


def test_ramp_fraction_needs_joint_interpolation():
    raw = {**DESIGN, "interpolation": "cartesian", "ramp_fraction": 0.25}
    raw["keyframes"] = [
        {"t": 0.0, "pos": [600, 0, 300], "pan": 0, "tilt": 0},
        {"t": 2.0, "pos": [600, 0, 800], "pan": 0, "tilt": 0},
    ]
    with pytest.raises(schema.SchemaError) as exc:
        schema.design_from_dict(raw, "demo")
    assert exc.value.field == "ramp_fraction"
