"""Tests for the offline validator, using FakeKinematics. No hardware."""

from __future__ import annotations

import copy

import pytest

from sw.robot import path, schema, validate
from sw.tests.fakes.fake_kinematics import FakeKinematics

ANGLES = {
    "name": "demo",
    "tool_id": 0,
    "sample_period_ms": 8,
    "seed_joints": [0, -90, 90, -90, -90, 0],
    "orientation": {"mode": "angles"},
    "workspace": {"min": [0, -1000, 0], "max": [1000, 1000, 1000]},
    "keyframes": [
        {"t": 0.0, "pos": [600, -400, 450], "pan": 20, "tilt": -5},
        {"t": 2.4, "pos": [600, 400, 450], "pan": -20, "tilt": -5},
        {"t": 4.0, "pos": [650, 400, 700], "pan": -20, "tilt": 5},
    ],
}

TARGET = {
    "name": "demo",
    "tool_id": 0,
    "sample_period_ms": 8,
    "seed_joints": [0, -90, 90, -90, -90, 0],
    "orientation": {"mode": "target", "target": [0, 0, 300]},
    "workspace": {"min": [0, -1000, 0], "max": [1000, 1000, 1000]},
    "keyframes": [
        {"t": 0.0, "pos": [600, -400, 450]},
        {"t": 2.0, "pos": [600, 400, 450]},
    ],
}


def _design(raw):
    return schema.design_from_dict(raw, "demo", folder_name="demo")


def _samples(design):
    return path.sample_path(design)


def test_valid_angles_design_passes_every_check(ANGLES=ANGLES):
    design = _design(ANGLES)
    samples = _samples(design)
    report = validate.validate(design, samples, FakeKinematics())
    assert report.passed
    assert all(c.passed for c in report.checks)
    assert len(report.joints) == len(samples)


def test_valid_target_design_passes_every_check():
    design = _design(TARGET)
    samples = _samples(design)
    report = validate.validate(design, samples, FakeKinematics())
    assert report.passed


def test_ik_failure_names_the_sample_index_and_time():
    design = _design(ANGLES)
    samples = _samples(design)
    kin = FakeKinematics(fail_ik_at={5})
    report = validate.validate(design, samples, kin)
    assert not report.passed
    ik_check = next(c for c in report.checks if c.name == "check_ik")
    assert not ik_check.passed
    assert "sample 5" in ik_check.detail
    t5 = samples[5].t
    assert f"t={t5:.3f}s" in ik_check.detail
    # downstream checks that need full joints should not run / should not crash
    assert report.joints == [] or len(report.joints) == 5


def test_large_joint_jump_fails_check_joint_steps():
    design = _design(ANGLES)
    samples = _samples(design)
    kin = FakeKinematics()

    # Monkeypatch-free jump injection: wrap ik to add a one-off offset at a
    # single sample only (a constant per-joint bias would cancel in the diff).
    calls = {"n": 0}
    real_ik = kin.ik

    def jumping_ik(pose6, ref6):
        calls["n"] += 1
        result = real_ik(pose6, ref6)
        if calls["n"] == 10 and result is not None:
            result = list(result)
            result[2] += 30.0  # one-sample 30deg jump on joint 3
        return result

    kin.ik = jumping_ik
    report = validate.validate(design, samples, kin)
    assert not report.passed
    step_check = next(c for c in report.checks if c.name == "check_joint_steps")
    assert not step_check.passed
    assert "joint step" in step_check.detail


def test_joint_beyond_soft_limit_fails():
    design = _design(ANGLES)
    samples = _samples(design)
    kin = FakeKinematics(soft_limits=[(-1, 1)] * 6)  # tiny limits, easy to exceed
    report = validate.validate(design, samples, kin)
    assert not report.passed
    limit_check = next(c for c in report.checks if c.name == "check_joint_limits")
    assert not limit_check.passed
    assert "soft limit" in limit_check.detail


def test_too_fast_design_fails_speed_and_reports_peaks():
    raw = copy.deepcopy(ANGLES)
    # Shrink keyframe times to make the same path much faster.
    for kf in raw["keyframes"]:
        kf["t"] = kf["t"] / 20.0
    design = _design(raw)
    samples = _samples(design)
    kin = FakeKinematics()
    report = validate.validate(design, samples, kin)
    assert not report.passed
    speed_check = next(c for c in report.checks if c.name == "check_tcp_speed")
    assert not speed_check.passed
    assert report.peaks.tcp_speed_mm_s > design.limits.max_tcp_speed_mm_s


def test_point_outside_box_fails_check_workspace():
    raw = copy.deepcopy(ANGLES)
    raw["keyframes"][1]["pos"] = [600, 5000, 450]  # far outside the box
    design = _design(raw)
    samples = _samples(design)
    report = validate.validate(design, samples, FakeKinematics())
    assert not report.passed
    ws_check = next(c for c in report.checks if c.name == "check_workspace")
    assert not ws_check.passed


def test_overshooting_vel_override_fails_check_path_overshoot():
    raw = copy.deepcopy(ANGLES)
    # A huge velocity override at an interior keyframe forces overshoot.
    raw["keyframes"][1]["vel"] = {"y": 50000.0}
    design = _design(raw)
    samples = _samples(design)
    report = validate.validate(design, samples, FakeKinematics())
    assert not report.passed
    overshoot_check = next(c for c in report.checks if c.name == "check_path_overshoot")
    assert not overshoot_check.passed


def test_pan_turn_above_180_fails_unless_allowed():
    """check_pan_turn looks at the designer's literal keyframe values, not the
    interpolated samples, so the keyframe timing here does not matter."""
    raw = copy.deepcopy(ANGLES)
    raw["keyframes"][0]["pan"] = 170
    raw["keyframes"][1]["pan"] = -170  # a -340deg turn, |step| > 180
    design = _design(raw)
    samples = _samples(design)
    report = validate.validate(design, samples, FakeKinematics())
    pan_check = next(c for c in report.checks if c.name == "check_pan_turn")
    assert not pan_check.passed

    raw["orientation"]["allow_long_pan"] = True
    raw["orientation"]["max_cumulative_pan_deg"] = 1000.0
    design2 = _design(raw)
    samples2 = _samples(design2)
    report2 = validate.validate(design2, samples2, FakeKinematics())
    pan_check2 = next(c for c in report2.checks if c.name == "check_pan_turn")
    assert pan_check2.passed


def test_cumulative_pan_above_limit_fails_even_with_allow_long_pan():
    raw = copy.deepcopy(ANGLES)
    raw["orientation"]["allow_long_pan"] = True
    raw["keyframes"][0]["pan"] = 0
    raw["keyframes"][1]["pan"] = 180  # each step <= 180, but the sum exceeds 360
    raw["keyframes"][2]["pan"] = 365
    design = _design(raw)
    samples = _samples(design)
    report = validate.validate(design, samples, FakeKinematics())
    pan_check = next(c for c in report.checks if c.name == "check_pan_turn")
    assert not pan_check.passed
    assert "cumulative pan" in pan_check.detail


def test_target_too_close_fails_check_target_geometry():
    raw = copy.deepcopy(TARGET)
    raw["orientation"]["target"] = [600, -400, 450]  # == first keyframe position
    design = _design(raw)
    samples = _samples(design)
    report = validate.validate(design, samples, FakeKinematics())
    assert not report.passed
    geom_check = next(c for c in report.checks if c.name == "check_target_geometry")
    assert not geom_check.passed


def test_wrong_active_tool_fails():
    design = _design(ANGLES)
    samples = _samples(design)
    kin = FakeKinematics(tool_id=5)  # design wants tool 0
    report = validate.validate(design, samples, kin)
    assert not report.passed
    tool_check = next(c for c in report.checks if c.name == "check_active_tool")
    assert not tool_check.passed


def test_nonzero_install_angle_fails():
    design = _design(ANGLES)
    samples = _samples(design)
    kin = FakeKinematics(install_angle_deg=(1.0, 0.0))
    report = validate.validate(design, samples, kin)
    assert not report.passed
    angle_check = next(c for c in report.checks if c.name == "check_install_angle")
    assert not angle_check.passed


def test_fk_offset_fails_check_fk_roundtrip():
    design = _design(ANGLES)
    samples = _samples(design)
    kin = FakeKinematics(fk_offset_mm=5.0)
    report = validate.validate(design, samples, kin)
    assert not report.passed
    fk_check = next(c for c in report.checks if c.name == "check_fk_roundtrip")
    assert not fk_check.passed


def test_fk_roundtrip_angle_wraparound_is_not_a_false_failure():
    """rz near +180deg vs -180deg is the same rotation, not a 360deg error."""
    from sw.robot import validate as validate_module

    assert validate_module._angle_diff_deg(179.999, -179.999) == pytest.approx(0.002)
    assert validate_module._angle_diff_deg(180.0, -180.0) == 0.0
    assert validate_module._angle_diff_deg(10.0, -10.0) == 20.0
    assert validate_module._angle_diff_deg(0.0, 170.0) == 170.0


def test_report_serialises_to_validated_json_schema():
    design = _design(ANGLES)
    samples = _samples(design)
    report = validate.validate(design, samples, FakeKinematics())
    assert report.passed
    data = validate.report_to_validated(report, design, samples)
    assert data["name"] == "demo"
    assert data["sample_period_ms"] == 8
    assert data["tool_id"] == 0
    assert data["num_points"] == len(samples)
    assert len(data["samples"]["t"]) == len(samples)
    assert len(data["samples"]["pose"]) == len(samples)
    assert len(data["samples"]["joints"]) == len(samples)
    assert all(c["passed"] for c in data["checks"])


@pytest.mark.parametrize("design_raw", [ANGLES, TARGET])
def test_ik_is_seeded_with_the_previous_solution(design_raw):
    """Each ik() call after the first gets the previous solution as ref6."""
    design = _design(design_raw)
    samples = _samples(design)
    seen_refs = []
    kin = FakeKinematics()
    real_ik = kin.ik

    def recording_ik(pose6, ref6):
        seen_refs.append(list(ref6))
        return real_ik(pose6, ref6)

    kin.ik = recording_ik
    report = validate.validate(design, samples, kin)
    assert report.passed
    assert seen_refs[0] == list(design.seed_joints)
    for i in range(1, len(seen_refs)):
        assert seen_refs[i] == report.joints[i - 1]


# -- singularity (cartesian mode) and IK hints ----------------------------------


def test_cartesian_path_near_the_elbow_singularity_fails():
    design = _design(ANGLES)
    kin = FakeKinematics(joint_offset={2: -45.0})  # J3 = z/100: nearly straight
    report = validate.validate(design, _samples(design), kin)
    check = next(c for c in report.checks if c.name == "check_singularity")
    assert not check.passed
    assert "interpolation: joint" in check.detail


def test_cartesian_path_near_the_wrist_singularity_fails():
    design = _design(ANGLES)
    kin = FakeKinematics(joint_offset={4: 90.0})  # J5 = ry: near 0
    report = validate.validate(design, _samples(design), kin)
    check = next(c for c in report.checks if c.name == "check_singularity")
    assert not check.passed
    assert "wrist" in check.detail


def test_ik_hint_explains_the_elbow_limit_and_full_extension():
    assert "160deg limit" in validate._ik_hint([0, 0, 155.0, 0, -90, 0])
    assert "singularity" in validate._ik_hint([0, 0, 1.0, 0, -90, 0])
    assert validate._ik_hint([0, 0, 90.0, 0, -90, 0]) == ""


def test_min_time_scale_uses_speed_and_sqrt_of_acceleration():
    design = _design(ANGLES)  # limits: tcp 800, joint 120 deg/s, 600 deg/s^2
    peaks = validate.Peaks(
        tcp_speed_mm_s=400.0,
        joint_speed_deg_s=(60.0,) * 6,
        joint_acc_deg_s2=(2400.0,) * 6,  # 4x the limit -> needs 2x the time
        max_joint_step_deg=0.5,
    )
    assert validate.min_time_scale(design, peaks) == pytest.approx(2.0)
    fast = validate.Peaks(400.0, (60.0,) * 6, (150.0,) * 6, 0.5)
    assert validate.min_time_scale(design, fast) == pytest.approx(0.5)


# -- interpolation: joint ------------------------------------------------------

JOINT = {
    "name": "demo",
    "tool_id": 0,
    "interpolation": "joint",
    "sample_period_ms": 8,
    "seed_joints": [0, -90, 90, -90, -90, 0],
    "orientation": {"mode": "angles"},
    "workspace": {"min": [0, -1000, 0], "max": [1000, 1000, 1000]},
    "keyframes": [  # constant pan/tilt: a pure crane, level throughout
        {"t": 0.0, "pos": [600, 0, 300], "pan": 0, "tilt": 0},
        {"t": 4.0, "pos": [600, 0, 800], "pan": 0, "tilt": 0},
    ],
}


def test_joint_design_passes_and_reports_fk_samples():
    design = _design(JOINT)
    report = validate.validate(design, [], FakeKinematics())
    assert report.passed, [c for c in report.checks if not c.passed]
    assert len(report.joints) == len(report.samples) == len(report.poses) == 501
    assert report.samples[0].pos == pytest.approx((600, 0, 300), abs=1e-6)
    assert report.samples[-1].pos == pytest.approx((600, 0, 800), abs=1e-6)
    assert report.peaks.max_roll_deg < 1e-6
    names = [c.name for c in report.checks]
    assert "check_keyframe_ik" in names and "check_roll" in names
    assert "check_ik" not in names  # IK runs only at the keyframes


def test_joint_design_solves_ik_only_at_keyframes_with_their_seeds():
    raw = copy.deepcopy(JOINT)
    raw["keyframes"][1]["seed"] = [1, 2, 3, 4, 5, 6]
    design = _design(raw)
    kin = FakeKinematics()
    refs = []
    real_ik = kin.ik
    kin.ik = lambda pose, ref: refs.append(list(ref)) or real_ik(pose, ref)
    validate.validate(design, [], kin)
    assert refs == [list(design.seed_joints), [1, 2, 3, 4, 5, 6]]


def test_joint_design_uses_joints_keyframes_as_given():
    raw = copy.deepcopy(JOINT)
    end = validate.Sample(4.0, (600, 0, 800), 0, 0, None)
    end_joints = FakeKinematics().ik(validate.to_sdk_pose(end), [0] * 6)
    raw["keyframes"][1] = {"t": 4.0, "joints": end_joints}
    report = validate.validate(_design(raw), [], FakeKinematics())
    assert report.passed
    assert report.joints[-1] == pytest.approx(end_joints)


def test_joint_design_keyframe_ik_failure_names_the_keyframe():
    report = validate.validate(_design(JOINT), [], FakeKinematics(fail_ik_at={1}))
    assert not report.passed
    check = next(c for c in report.checks if c.name == "check_keyframe_ik")
    assert "keyframe 1" in check.detail and "seed" in check.detail
    assert report.joints == []


def test_check_roll_fails_for_a_rolled_camera():
    from scipy.spatial.transform import Rotation

    from sw.robot.pose import level_rotation, rotation_to_sdk_euler

    rolled = (
        level_rotation(0, 0) @ Rotation.from_euler("z", 1.0, degrees=True).as_matrix()
    )
    pose = [0, 0, 0, *rotation_to_sdk_euler(rolled)]
    result, worst = validate.check_roll([pose], limit_deg=0.5)
    assert not result.passed
    assert worst == pytest.approx(1.0, abs=1e-6)
    ok, _ = validate.check_roll([pose], limit_deg=2.0)
    assert ok.passed


def test_joint_report_serialises_fk_poses_and_interpolation():
    design = _design(JOINT)
    report = validate.validate(design, [], FakeKinematics())
    data = validate.report_to_validated(report, design, [])
    assert data["interpolation"] == "joint"
    assert data["num_points"] == 501
    assert data["samples"]["pose"][0] == pytest.approx(report.poses[0])
    assert "max_roll_deg" in data["peaks"]
