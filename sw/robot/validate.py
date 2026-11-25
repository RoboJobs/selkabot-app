"""Offline validation of a sampled design against the controller's own kinematics.

`validate(design, samples, kin)` runs every check below and returns a
ValidationReport. Each check is a separate function so it can be unit
tested alone with a FakeKinematics.

Two pipelines, chosen by `design.interpolation`:
- cartesian: the camera pose is planned at every sample (path.py) and IK is
  called sample by sample, seeded with the previous sample's solution (the
  first with design.seed_joints). The camera is level exactly by construction.
- joint: IK runs only at the keyframes (or the keyframe gives `joints`
  directly). J1-J3 follow the joint-space blend (joint_path.py); at every
  sample the wrist J4-J6 is solved (level.py) so the camera is exactly level
  with pan/tilt blended between the keyframes'. FK of every sample gives the
  camera poses; check_roll verifies the result.

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

from sw.robot.frames import design_to_base, within_workspace
from sw.robot.joint_path import make_channel, sample_joint_path
from sw.robot.level import LevelError, solve_wrist
from sw.robot.path import Sample, to_sdk_pose
from sw.robot.pose import (
    camera_angles,
    pose_rotation_diff_deg,
    roll_deviation_deg,
    sdk_euler_to_rotation,
    unwrap_deg,
)
from sw.robot.schema import Design

JOINT_LIMIT_MARGIN_DEG = 2.0
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
    max_roll_deg: float = 0.0  # max deviation from the format's roll


@dataclass
class ValidationReport:
    passed: bool
    checks: list[CheckResult] = field(default_factory=list)
    peaks: Peaks = field(default_factory=Peaks)
    joints: list[list[float]] = field(default_factory=list)  # per-sample joints
    samples: list[Sample] = field(default_factory=list)  # the samples validated
    poses: list[list[float]] = field(default_factory=list)  # per-sample SDK pose


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
    kfs = [kf for kf in design.keyframes if kf.pan is not None]
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
        pose = to_sdk_pose(s, design.camera_roll_deg)
        solution = kin.ik(pose, ref)
        if solution is None:
            return (
                CheckResult(
                    "check_ik",
                    False,
                    f"sample {i} (t={s.t:.3f}s): no IK solution for pose {pose}"
                    + _ik_hint(ref),
                ),
                joints,
            )
        joints.append(solution)
        ref = solution
    return CheckResult("check_ik", True), joints


def _ik_hint(last_joints: list[float]) -> str:
    """Explain the usual causes of an IK failure from the last good solution."""
    elbow = abs(last_joints[2])
    if elbow > 150.0:
        return (
            f". The elbow (J3) was already at {last_joints[2]:.1f}deg, near its "
            "160deg limit: the straight Cartesian path folds the arm too far. Use "
            "'interpolation: joint' for large swings, or add a keyframe that keeps "
            "the camera farther from the base"
        )
    if elbow < 5.0:
        return (
            f". The elbow (J3) was at {last_joints[2]:.1f}deg: the arm is nearly "
            "straight (elbow singularity) and the pose is at the edge of the reach. "
            "Use 'interpolation: joint', or keep the elbow bent a few degrees"
        )
    return ""


def check_singularity(
    design: Design, samples: list[Sample], joints: list[list[float]]
) -> CheckResult:
    """Cartesian mode: keep away from the elbow and wrist singularities.

    On this arm the elbow is singular when J3 = 0 (arm straight) and the
    wrist when J5 = 0 or 180. Following a Cartesian path near either makes
    the joint speeds and accelerations blow up; time scaling cannot fix it.
    Joint mode does not need this check: its joints are smooth by
    construction even at a singular keyframe.
    """
    elbow_min = design.limits.min_elbow_margin_deg
    for i, js in enumerate(joints):
        if abs(js[2]) < elbow_min:
            return CheckResult(
                "check_singularity",
                False,
                f"sample {i} (t={samples[i].t:.3f}s): elbow J3={js[2]:.2f}deg is "
                f"within {elbow_min}deg of straight (singular). Use 'interpolation: "
                "joint' for moves that reach full extension, or bend the elbow more",
            )
    wrist = check_wrist_singularity(design, samples, joints)
    return CheckResult("check_singularity", wrist.passed, wrist.detail)


def check_wrist_singularity(
    design: Design, samples: list[Sample], joints: list[list[float]]
) -> CheckResult:
    """|J5| stays min_wrist_margin_deg away from 0 and 180 (wrist singular).

    Used by both modes: joint mode solves the wrist at every sample to keep
    the camera level, which is ill-conditioned there too.
    """
    wrist_min = math.sin(math.radians(design.limits.min_wrist_margin_deg))
    for i, js in enumerate(joints):
        if abs(math.sin(math.radians(js[4]))) < wrist_min:
            return CheckResult(
                "check_wrist_singularity",
                False,
                f"sample {i} (t={samples[i].t:.3f}s): wrist J5={js[4]:.2f}deg is "
                f"within {design.limits.min_wrist_margin_deg}deg of 0/180 (singular). "
                "Keyframes whose J5 have opposite signs need the wrist to flip "
                "through this point: re-jog one of them with the other's wrist",
            )
    return CheckResult("check_wrist_singularity", True)


def check_roll(
    poses: list[list[float]], limit_deg: float, target_roll: float = 0.0
) -> tuple[CheckResult, float]:
    """The camera's roll about its optical axis stays within limit_deg of
    target_roll (0 landscape, 90 portrait). Returns the worst deviation."""
    worst, worst_i = 0.0, 0
    for i, pose in enumerate(poses):
        dev = abs(roll_deviation_deg(sdk_euler_to_rotation(*pose[3:]), target_roll))
        if dev > worst:
            worst, worst_i = dev, i
    if worst > limit_deg:
        return (
            CheckResult(
                "check_roll",
                False,
                f"sample {worst_i}: camera roll is {worst:.3f}deg off the wanted "
                f"{target_roll:g}deg, more than max_roll_deg ({limit_deg})",
            ),
            worst,
        )
    detail = f"max deviation {worst:.3f}deg from roll {target_roll:g}deg"
    return CheckResult("check_roll", True, detail), worst


def check_keyframe_ik(
    design: Design, kin: Kinematics
) -> tuple[CheckResult, list[list[float]]]:
    """Joint mode: one joint vector per keyframe.

    A `joints` keyframe is used as given. A pose keyframe is solved by IK for
    its level camera pose, seeded with its own `seed`, else the previous
    keyframe's joints, else design.seed_joints.
    """
    base = design_to_base(design)
    result: list[list[float]] = []
    prev = list(design.seed_joints)
    for i, kf in enumerate(base.keyframes):
        if kf.joints is not None:
            joints = list(kf.joints)
        else:
            pose = to_sdk_pose(
                Sample(kf.t, kf.pos, kf.pan, kf.tilt, None), design.camera_roll_deg
            )
            seed = list(kf.seed) if kf.seed is not None else prev
            joints = kin.ik(pose, seed)
            if joints is None:
                return (
                    CheckResult(
                        "check_keyframe_ik",
                        False,
                        f"keyframe {i} (t={kf.t:.3f}s): no IK solution near seed "
                        f"{[round(v, 3) for v in seed]} for pose "
                        f"{[round(v, 3) for v in pose]}. Give the keyframe a 'seed' "
                        "(the joints read on the pendant at that pose) or 'joints'",
                    ),
                    result,
                )
        result.append(list(joints))
        prev = list(joints)
    return CheckResult("check_keyframe_ik", True), result


def min_time_scale(design: Design, peaks: Peaks) -> float:
    """Smallest uniform time scale k (multiply every keyframe time by k) that
    brings the measured peaks within the design's limits.

    Speeds and joint steps scale as 1/k, accelerations as 1/k^2, so
    k = max(v/v_max, sqrt(a/a_max)). k < 1 means the motion could run faster.
    Exact for joint mode; for cartesian mode it is an estimate (re-validate).
    """
    lim = design.limits
    ratios = [
        peaks.tcp_speed_mm_s / lim.max_tcp_speed_mm_s,
        max(peaks.joint_speed_deg_s) / lim.max_joint_speed_deg_s,
        math.sqrt(max(peaks.joint_acc_deg_s2) / lim.max_joint_acc_deg_s2),
        peaks.max_joint_step_deg / lim.max_joint_step_deg,
    ]
    return max(ratios)


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
    samples: list[Sample],
    joints: list[list[float]],
    kin: Kinematics,
    roll_deg: float = 0.0,
) -> tuple[CheckResult, float, float, list[list[float]]]:
    """FK every solved sample and compare it with the *planned* pose
    (position, and orientation as a rotation: Euler triples near ry = 90 are
    not unique). Returns the result, the max position/rotation errors (even
    when the check fails) and the FK poses, which become the artifact's pose
    columns (the controller's own representation)."""
    max_pos_err = 0.0
    max_rot_err = 0.0
    failure: str | None = None
    fk_poses: list[list[float]] = []
    for i, js in enumerate(joints):
        pose = list(kin.fk(js))
        fk_poses.append(pose)
        expected = to_sdk_pose(samples[i], roll_deg)
        pos_err = math.dist(pose[:3], expected[:3])
        rot_err = pose_rotation_diff_deg(pose, expected)
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
    return result, max_pos_err, max_rot_err, fk_poses


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

    `samples` is path.sample_path(design) for cartesian designs; joint designs
    ignore it (their samples come from FK of the joint blend).
    """
    if design.interpolation == "joint":
        return _validate_joint(design, kin)
    return _validate_cartesian(design, samples, kin)


def _sample_from_pose(t: float, pose: list[float]) -> Sample:
    """A Sample (base frame) whose pan/tilt are read off an SDK pose."""
    pan, tilt, _roll = camera_angles(*pose[3:])
    return Sample(t, (pose[0], pose[1], pose[2]), pan, tilt, None)


def _keyframe_view(design: Design, kin: Kinematics, kf_joints) -> tuple[list, list]:
    """Pan and tilt wanted at each keyframe: the pose keyframe's own values,
    or where the camera points at a `joints` keyframe. Pans are unwrapped."""
    base = design_to_base(design)
    pans, tilts = [], []
    for kf, joints in zip(base.keyframes, kf_joints, strict=True):
        if kf.joints is None:
            pans.append(kf.pan)
            tilts.append(kf.tilt)
        else:
            pan, tilt, _roll = camera_angles(*kin.fk(joints)[3:])
            pans.append(pan)
            tilts.append(tilt)
    return unwrap_deg(pans), tilts


def level_joint_path(
    design: Design, kin: Kinematics, kf_joints: list[list[float]]
) -> tuple[CheckResult, list[float], list[list[float]]]:
    """Joint-mode samples: J1-J3 from the joint blend, J4-J6 solved at every
    sample so the camera is level with pan/tilt blended between keyframes.

    The blended wrist is the starting guess; the previous sample's solution
    (shifted by the blend's change) warm-starts the next, so each solve takes
    a handful of FK calls.
    """
    times, blend = sample_joint_path(design, kf_joints)
    if not design.level_wrist:
        return (
            CheckResult(
                "check_wrist_level",
                True,
                "skipped: level_wrist is false, J4-J6 follow the joint blend",
            ),
            times,
            [list(q) for q in blend],
        )
    kf_times = [kf.t for kf in design.keyframes]
    stop = [i for i, kf in enumerate(design.keyframes) if kf.stop]
    kf_pans, kf_tilts = _keyframe_view(design, kin, kf_joints)
    pan_ch = make_channel(kf_times, kf_pans, stop, design.ramp_fraction)
    tilt_ch = make_channel(kf_times, kf_tilts, stop, design.ramp_fraction)
    joints: list[list[float]] = []
    guess = blend[0][3:]
    for i, (t, q) in enumerate(zip(times, blend, strict=True)):
        if i > 0:
            guess = [g + q[k + 3] - blend[i - 1][k + 3] for k, g in enumerate(guess)]
        try:
            solved = solve_wrist(
                kin.fk,
                q,
                pan_ch.value(t),
                tilt_ch.value(t),
                guess,
                roll=design.camera_roll_deg,
            )
        except LevelError as exc:
            return (
                CheckResult(
                    "check_wrist_level",
                    False,
                    f"sample {i} (t={t:.3f}s): {exc}",
                ),
                times,
                joints,
            )
        joints.append(solved)
        guess = solved[3:]
    return CheckResult("check_wrist_level", True), times, joints


def _validate_joint(design: Design, kin: Kinematics) -> ValidationReport:
    checks: list[CheckResult] = [
        check_pan_turn(design),
        check_active_tool(design, kin),
        check_install_angle(kin),
    ]
    kf_result, kf_joints = check_keyframe_ik(design, kin)
    checks.append(kf_result)
    if not kf_result.passed:
        return ValidationReport(passed=False, checks=checks)

    level_result, times, joints = level_joint_path(design, kin, kf_joints)
    checks.append(level_result)
    if not level_result.passed:
        return ValidationReport(passed=False, checks=checks)
    poses = [list(kin.fk(j)) for j in joints]
    raw = [_sample_from_pose(t, p) for t, p in zip(times, poses, strict=True)]
    pans = unwrap_deg([s.pan for s in raw])
    samples = [
        Sample(s.t, s.pos, pan, s.tilt, None) for s, pan in zip(raw, pans, strict=True)
    ]

    peaks = _compute_peaks(design, samples, joints)
    # No wrist-singularity check in joint mode: it existed because MoveTPD
    # refuses paths at J5 = 0; the Lua ServoJ playback plays them (real robot,
    # 2026-11-25). The joint speed/acceleration/step checks bound J4/J6.
    roll_result, peaks.max_roll_deg = check_roll(
        poses, design.limits.max_roll_deg, design.camera_roll_deg
    )
    checks += [
        check_workspace(design, samples),
        check_tcp_speed(design, samples, peaks),
        check_joint_steps(design, joints),
        check_joint_limits(joints, kin),
        check_joint_speed_acc(design, joints, peaks),
        roll_result,
    ]
    passed = all(c.passed for c in checks)
    return ValidationReport(passed, checks, peaks, joints, samples, poses)


def _validate_cartesian(
    design: Design, samples: list[Sample], kin: Kinematics
) -> ValidationReport:
    """Checks run in an order that fails fast on cheap, design-only problems
    before the (relatively expensive) sample-by-sample IK pass."""
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
        checks.append(check_singularity(design, samples, joints))
        checks.append(check_joint_steps(design, joints))
        checks.append(check_joint_limits(joints, kin))
        checks.append(check_joint_speed_acc(design, joints, peaks))
        fk_result, pos_err, rot_err, fk_poses = check_fk_roundtrip(
            samples, joints, kin, design.camera_roll_deg
        )
        peaks.max_fk_pos_err_mm = pos_err
        peaks.max_fk_rot_err_deg = rot_err
        checks.append(fk_result)
        roll_result, peaks.max_roll_deg = check_roll(
            fk_poses, design.limits.max_roll_deg, design.camera_roll_deg
        )
        checks.append(roll_result)
        # The artifact's pose columns are the controller's FK of the joints
        # (its own Euler representation), checked above against the plan.
        poses = fk_poses
    else:
        poses = [to_sdk_pose(s, design.camera_roll_deg) for s in samples]

    passed = all(c.passed for c in checks)
    return ValidationReport(passed, checks, peaks, joints, list(samples), poses)


def report_to_validated(
    report: ValidationReport, design: Design, samples: list[Sample]
) -> dict[str, Any]:
    """Build the validated.json content. Only meaningful when report.passed.

    Uses the samples and poses the report validated (for joint designs those
    come from FK, not from `samples`); `samples` is kept for older callers.
    """
    samples = report.samples or samples
    poses = report.poses or [to_sdk_pose(s, design.camera_roll_deg) for s in samples]
    duration_s = design.sample_period_ms * (len(samples) - 1) / 1000.0
    design_duration_s = float(design.keyframes[-1].t)
    return {
        "name": design.name,
        "design_sha256": None,  # filled by the caller, which knows the file path
        "created": datetime.datetime.now(datetime.UTC).isoformat(),
        "sample_period_ms": design.sample_period_ms,
        "tool_id": design.tool_id,
        "format": design.format,
        "interpolation": design.interpolation,
        "install_angle": None,  # filled by the caller from kin.install_angle()
        "num_points": len(samples),
        "duration_s": duration_s,
        "design_duration_s": design_duration_s,
        "samples": {
            "t": [s.t for s in samples],
            "pose": poses,
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
            "max_roll_deg": report.peaks.max_roll_deg,
        },
        "checks": [
            {"name": c.name, "passed": c.passed, "detail": c.detail}
            for c in report.checks
        ],
    }
