"""Offline validation of a sampled design against the controller's own kinematics.

`validate(design, samples, kin)` runs every check below and returns a
ValidationReport. Each check is a separate function so it can be unit
tested alone with a FakeKinematics. IK is called sample by sample, seeded
with the previous sample's solution (the first sample is seeded with
design.seed_joints), since consecutive samples are close together and a good
seed avoids IK branch flips.

Peaks (speed, acceleration, joint steps) come from finite differences of the
samples at the controller's own sample_period_ms, which is dense enough to
bound the real values for the slow, brisk motions this project targets.
"""

from __future__ import annotations

import datetime
import math
from dataclasses import dataclass, field
from typing import Any, Protocol

import numpy as np

from sw.robot.frames import within_workspace
from sw.robot.path import Sample, to_sdk_pose
from sw.robot.schema import Design

JOINT_LIMIT_MARGIN_DEG = 2.0
FK_ROUNDTRIP_STRIDE = 10  # check every 10th sample
FK_ROUNDTRIP_MAX_POS_MM = 0.1
FK_ROUNDTRIP_MAX_ROT_DEG = 0.05


class Kinematics(Protocol):
    def ik(self, pose6: list[float], ref6: list[float]) -> list[float] | None: ...
    def fk(self, joints6: list[float]) -> list[float]: ...
    def active_tool(self) -> int: ...
    def install_angle(self) -> tuple[float, float]: ...
    def joint_soft_limits(self) -> list[tuple[float, float]]: ...


@dataclass
class CheckResult:
    name: str
    passed: bool
    detail: str = ""


@dataclass
class Peaks:
    tcp_speed_mm_s: float = 0.0
    joint_speed_deg_s: tuple[float, ...] = (0.0,) * 6
    joint_acc_deg_s2: tuple[float, ...] = (0.0,) * 6
    max_joint_step_deg: float = 0.0
    max_fk_pos_err_mm: float = 0.0
    max_fk_rot_err_deg: float = 0.0


@dataclass
class ValidationReport:
    passed: bool
    checks: list[CheckResult] = field(default_factory=list)
    peaks: Peaks = field(default_factory=Peaks)
    joints: list[list[float]] = field(default_factory=list)  # per-sample IK solution


def _dt(design: Design) -> float:
    return design.sample_period_ms / 1000.0


# -- checks that need only the design, not samples or the robot --------------


def check_pan_turn(design: Design) -> CheckResult:
    """Checks the designer's literal keyframe pan values (not interpolated
    samples): a step above 180deg between adjacent keyframes, and the total
    cumulative pan across the whole design."""
    if design.orientation.mode != "angles":
        return CheckResult("check_pan_turn", True, "not applicable (target mode)")
    limit = design.orientation.max_cumulative_pan_deg
    allow_long = design.orientation.allow_long_pan
    kfs = design.keyframes
    cumulative = 0.0
    for i in range(1, len(kfs)):
        step = kfs[i].pan - kfs[i - 1].pan
        if abs(step) > 180.0 and not allow_long:
            return CheckResult(
                "check_pan_turn",
                False,
                f"keyframe {i} (t={kfs[i].t:.3f}s): pan step {step:.1f}deg "
                "exceeds 180deg; set allow_long_pan to permit it",
            )
        cumulative += abs(step)
    if cumulative > limit:
        return CheckResult(
            "check_pan_turn",
            False,
            f"cumulative pan {cumulative:.1f}deg exceeds max_cumulative_pan_deg "
            f"({limit})",
        )
    return CheckResult("check_pan_turn", True)


def check_target_geometry(design: Design, samples: list[Sample]) -> CheckResult:
    if design.orientation.mode != "target":
        return CheckResult("check_target_geometry", True, "not applicable")
    orient = design.orientation
    period_s = _dt(design)
    max_pan_step = orient.max_pan_rate_deg_s * period_s
    prev_pan = None
    for i, s in enumerate(samples):
        d = np.asarray(s.target, dtype=float) - np.asarray(s.pos, dtype=float)
        dist = float(np.linalg.norm(d))
        if dist < orient.min_target_distance_mm:
            return CheckResult(
                "check_target_geometry",
                False,
                f"sample {i} (t={s.t:.3f}s): target distance {dist:.1f}mm below "
                f"min_target_distance_mm ({orient.min_target_distance_mm})",
            )
        horiz = float(math.hypot(d[0], d[1]))
        if horiz < orient.min_horizontal_mm:
            return CheckResult(
                "check_target_geometry",
                False,
                f"sample {i} (t={s.t:.3f}s): horizontal target distance "
                f"{horiz:.1f}mm below min_horizontal_mm ({orient.min_horizontal_mm})",
            )
        if abs(s.tilt) > orient.max_tilt_deg:
            return CheckResult(
                "check_target_geometry",
                False,
                f"sample {i} (t={s.t:.3f}s): tilt {s.tilt:.1f}deg exceeds "
                f"max_tilt_deg ({orient.max_tilt_deg})",
            )
        if prev_pan is not None and abs(s.pan - prev_pan) > max_pan_step:
            return CheckResult(
                "check_target_geometry",
                False,
                f"sample {i} (t={s.t:.3f}s): pan rate exceeds "
                f"max_pan_rate_deg_s ({orient.max_pan_rate_deg_s})",
            )
        prev_pan = s.pan
    return CheckResult("check_target_geometry", True)


def check_path_overshoot(design: Design, samples: list[Sample]) -> CheckResult:
    """Each segment's sampled range stays within its endpoints, plus a margin."""
    times = [kf.t for kf in design.keyframes]
    pct = design.limits.overshoot_pct / 100.0

    def channel_values(name: str) -> list[float]:
        if name == "pan":
            return [s.pan for s in samples]
        if name == "tilt":
            return [s.tilt for s in samples]
        idx = {"x": 0, "y": 1, "z": 2}[name]
        return [s.pos[idx] for s in samples]

    channels = ["x", "y", "z"]
    if design.orientation.mode == "angles":
        channels += ["pan", "tilt"]
    sample_times = [s.t for s in samples]
    for name in channels:
        values = channel_values(name)
        for seg in range(len(times) - 1):
            lo_t, hi_t = times[seg], times[seg + 1]
            idxs = [
                i for i, t in enumerate(sample_times) if lo_t - 1e-9 <= t <= hi_t + 1e-9
            ]
            if not idxs:
                continue
            seg_values = [values[i] for i in idxs]
            endpoints = [values[idxs[0]], values[idxs[-1]]]
            span = abs(endpoints[1] - endpoints[0])
            margin = max(0.5 if name in ("x", "y", "z") else 0.1, pct * span)
            lo = min(endpoints) - margin
            hi = max(endpoints) + margin
            bad = [v for v in seg_values if v < lo - 1e-9 or v > hi + 1e-9]
            if bad:
                return CheckResult(
                    "check_path_overshoot",
                    False,
                    f"segment {seg} ({lo_t:.3f}-{hi_t:.3f}s), channel '{name}': "
                    f"sample value {bad[0]:.3f} outside [{lo:.3f}, {hi:.3f}]",
                )
    return CheckResult("check_path_overshoot", True)


def check_workspace(design: Design, samples: list[Sample]) -> CheckResult:
    points = np.array([s.pos for s in samples])
    inside = within_workspace(points, design)
    if not inside.all():
        i = int(np.argmin(inside))
        return CheckResult(
            "check_workspace",
            False,
            f"sample {i} (t={samples[i].t:.3f}s): position outside the "
            "workspace box (authoring frame)",
        )
    return CheckResult("check_workspace", True)


def check_tcp_speed(design: Design, samples: list[Sample], peaks: Peaks) -> CheckResult:
    limit = design.limits.max_tcp_speed_mm_s
    if peaks.tcp_speed_mm_s > limit:
        return CheckResult(
            "check_tcp_speed",
            False,
            f"peak TCP speed {peaks.tcp_speed_mm_s:.1f}mm/s exceeds "
            f"max_tcp_speed_mm_s ({limit})",
        )
    return CheckResult("check_tcp_speed", True)


# -- checks that need the robot's kinematics ---------------------------------


def check_active_tool(design: Design, kin: Kinematics) -> CheckResult:
    actual = kin.active_tool()
    if actual != design.tool_id:
        return CheckResult(
            "check_active_tool",
            False,
            f"controller's active tool is {actual}, design expects {design.tool_id}",
        )
    return CheckResult("check_active_tool", True)


def check_install_angle(kin: Kinematics) -> CheckResult:
    yangle, zangle = kin.install_angle()
    if abs(yangle) > 0.1 or abs(zangle) > 0.1:
        return CheckResult(
            "check_install_angle",
            False,
            f"install angle ({yangle:.3f}, {zangle:.3f}) is not (0, 0) within 0.1deg",
        )
    return CheckResult("check_install_angle", True)


def check_ik(
    design: Design, samples: list[Sample], kin: Kinematics
) -> tuple[CheckResult, list[list[float]]]:
    """Solve IK sample by sample, seeded with the previous solution.

    Returns the check result and the list of joint solutions found so far
    (shorter than `samples` if IK failed partway through).
    """
    joints: list[list[float]] = []
    ref = list(design.seed_joints)
    for i, s in enumerate(samples):
        pose = to_sdk_pose(s)
        solution = kin.ik(pose, ref)
        if solution is None:
            return (
                CheckResult(
                    "check_ik",
                    False,
                    f"sample {i} (t={s.t:.3f}s): no IK solution for pose {pose}",
                ),
                joints,
            )
        joints.append(solution)
        ref = solution
    return CheckResult("check_ik", True), joints


def check_joint_steps(design: Design, joints: list[list[float]]) -> CheckResult:
    limit = design.limits.max_joint_step_deg
    for i in range(1, len(joints)):
        step = max(abs(a - b) for a, b in zip(joints[i], joints[i - 1], strict=True))
        if step > limit:
            return CheckResult(
                "check_joint_steps",
                False,
                f"sample {i}: joint step {step:.2f}deg exceeds "
                f"max_joint_step_deg ({limit})",
            )
    return CheckResult("check_joint_steps", True)


def check_joint_limits(joints: list[list[float]], kin: Kinematics) -> CheckResult:
    limits = kin.joint_soft_limits()
    margin = JOINT_LIMIT_MARGIN_DEG
    for i, js in enumerate(joints):
        for j, (value, (lo, hi)) in enumerate(zip(js, limits, strict=True)):
            if value < lo + margin or value > hi - margin:
                return CheckResult(
                    "check_joint_limits",
                    False,
                    f"sample {i}: joint {j + 1} value {value:.2f}deg is within "
                    f"{JOINT_LIMIT_MARGIN_DEG}deg of its soft limit ({lo}, {hi})",
                )
    return CheckResult("check_joint_limits", True)


def check_joint_speed_acc(
    design: Design, joints: list[list[float]], peaks: Peaks
) -> CheckResult:
    limit_speed = design.limits.max_joint_speed_deg_s
    limit_acc = design.limits.max_joint_acc_deg_s2
    for j in range(6):
        if peaks.joint_speed_deg_s[j] > limit_speed:
            return CheckResult(
                "check_joint_speed_acc",
                False,
                f"joint {j + 1} peak speed {peaks.joint_speed_deg_s[j]:.1f}deg/s "
                f"exceeds max_joint_speed_deg_s ({limit_speed})",
            )
        if peaks.joint_acc_deg_s2[j] > limit_acc:
            return CheckResult(
                "check_joint_speed_acc",
                False,
                f"joint {j + 1} peak acceleration "
                f"{peaks.joint_acc_deg_s2[j]:.1f}deg/s^2 exceeds "
                f"max_joint_acc_deg_s2 ({limit_acc})",
            )
    return CheckResult("check_joint_speed_acc", True)


def check_fk_roundtrip(
    samples: list[Sample], joints: list[list[float]], kin: Kinematics
) -> tuple[CheckResult, float, float]:
    """Returns the check result plus the max position/rotation error found,
    even when the check fails, so callers can report the peaks either way."""
    max_pos_err = 0.0
    max_rot_err = 0.0
    failure: str | None = None
    for i in range(0, len(joints), FK_ROUNDTRIP_STRIDE):
        pose = kin.fk(joints[i])
        expected = to_sdk_pose(samples[i])
        pos_err = math.dist(pose[:3], expected[:3])
        rot_err = max(abs(a - b) for a, b in zip(pose[3:], expected[3:], strict=True))
        max_pos_err = max(max_pos_err, pos_err)
        max_rot_err = max(max_rot_err, rot_err)
        if failure is None and (
            pos_err > FK_ROUNDTRIP_MAX_POS_MM or rot_err > FK_ROUNDTRIP_MAX_ROT_DEG
        ):
            failure = (
                f"sample {i}: FK(IK(pose)) differs by {pos_err:.4f}mm / "
                f"{rot_err:.4f}deg from the planned pose"
            )
    result = CheckResult("check_fk_roundtrip", failure is None, failure or "")
    return result, max_pos_err, max_rot_err


# -- peaks from finite differences -------------------------------------------


def _compute_peaks(
    design: Design, samples: list[Sample], joints: list[list[float]]
) -> Peaks:
    dt = _dt(design)
    peaks = Peaks()
    if len(samples) >= 2:
        positions = np.array([s.pos for s in samples])
        diffs = np.diff(positions, axis=0)
        speeds = np.linalg.norm(diffs, axis=1) / dt
        peaks.tcp_speed_mm_s = float(speeds.max())
    if len(joints) >= 2:
        arr = np.array(joints)
        joint_speeds = np.abs(np.diff(arr, axis=0)) / dt
        peaks.joint_speed_deg_s = tuple(float(v) for v in joint_speeds.max(axis=0))
        peaks.max_joint_step_deg = float(np.abs(np.diff(arr, axis=0)).max())
        if len(joints) >= 3:
            joint_acc = np.abs(np.diff(arr, n=2, axis=0)) / (dt * dt)
            peaks.joint_acc_deg_s2 = tuple(float(v) for v in joint_acc.max(axis=0))
    return peaks


# -- top-level entry point ---------------------------------------------------


def validate(
    design: Design, samples: list[Sample], kin: Kinematics
) -> ValidationReport:
    """Run every check and return a ValidationReport.

    Checks run in an order that fails fast on cheap, design-only problems
    before the (relatively expensive) sample-by-sample IK pass.
    """
    checks: list[CheckResult] = []

    checks.append(check_pan_turn(design))
    checks.append(check_target_geometry(design, samples))
    checks.append(check_path_overshoot(design, samples))
    checks.append(check_workspace(design, samples))
    checks.append(check_active_tool(design, kin))
    checks.append(check_install_angle(kin))

    ik_result, joints = check_ik(design, samples, kin)
    checks.append(ik_result)

    peaks = _compute_peaks(design, samples, joints)
    checks.append(check_tcp_speed(design, samples, peaks))

    if ik_result.passed:
        checks.append(check_joint_steps(design, joints))
        checks.append(check_joint_limits(joints, kin))
        checks.append(check_joint_speed_acc(design, joints, peaks))
        fk_result, pos_err, rot_err = check_fk_roundtrip(samples, joints, kin)
        peaks.max_fk_pos_err_mm = pos_err
        peaks.max_fk_rot_err_deg = rot_err
        checks.append(fk_result)

    passed = all(c.passed for c in checks)
    return ValidationReport(passed=passed, checks=checks, peaks=peaks, joints=joints)


def report_to_validated(
    report: ValidationReport, design: Design, samples: list[Sample]
) -> dict[str, Any]:
    """Build the validated.json content. Only meaningful when report.passed."""
    duration_s = design.sample_period_ms * (len(samples) - 1) / 1000.0
    design_duration_s = float(design.keyframes[-1].t)
    return {
        "name": design.name,
        "design_sha256": None,  # filled by the caller, which knows the file path
        "created": datetime.datetime.now(datetime.UTC).isoformat(),
        "sample_period_ms": design.sample_period_ms,
        "tool_id": design.tool_id,
        "install_angle": None,  # filled by the caller from kin.install_angle()
        "num_points": len(samples),
        "duration_s": duration_s,
        "design_duration_s": design_duration_s,
        "samples": {
            "t": [s.t for s in samples],
            "pose": [to_sdk_pose(s) for s in samples],
            "pan": [s.pan for s in samples],
            "tilt": [s.tilt for s in samples],
            "joints": report.joints,
        },
        "peaks": {
            "tcp_speed_mm_s": report.peaks.tcp_speed_mm_s,
            "joint_speed_deg_s": list(report.peaks.joint_speed_deg_s),
            "joint_acc_deg_s2": list(report.peaks.joint_acc_deg_s2),
            "max_joint_step_deg": report.peaks.max_joint_step_deg,
            "max_fk_pos_err_mm": report.peaks.max_fk_pos_err_mm,
            "max_fk_rot_err_deg": report.peaks.max_fk_rot_err_deg,
        },
        "checks": [
            {"name": c.name, "passed": c.passed, "detail": c.detail}
            for c in report.checks
        ],
    }
