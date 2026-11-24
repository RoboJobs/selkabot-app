"""Path planning in the base frame: keyframes -> smooth channels -> samples.

Each channel is a piecewise quintic Hermite function of time, one polynomial per
keyframe segment. At every keyframe the acceleration is zero, so acceleration
is continuous. Velocities at interior keyframes default to PCHIP slopes (no
overshoot for monotone keyframes); the first and last keyframes are at rest.
Changing one keyframe only changes its two neighbouring segments.

Sampling rule (exact): with p = sample_period_ms and T the last keyframe time,
N = ceil(round(T * 1000 / p, 6)). Samples are at t_i = i * p / 1000 for
i = 0..N, so there are N + 1 uniformly spaced samples. Times beyond T are
clamped to T, where the path is at rest, so the last sample equals the final
keyframe pose.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.interpolate import PchipInterpolator

from sw.robot.frames import design_to_base
from sw.robot.pose import level_rotation, rotation_to_sdk_euler, unwrap_deg
from sw.robot.schema import Design


@dataclass(frozen=True)
class Sample:
    t: float
    pos: tuple[float, float, float]  # base frame, mm
    pan: float  # deg, base frame
    tilt: float  # deg
    target: tuple[float, float, float] | None  # base frame, target mode only


def sample_count(duration_s: float, period_ms: int) -> int:
    """Number of samples (N + 1) for a path of duration T at period p."""
    return math.ceil(round(duration_s * 1000.0 / period_ms, 6)) + 1


def sample_times(duration_s: float, period_ms: int) -> list[float]:
    count = sample_count(duration_s, period_ms)
    return [min(i * period_ms / 1000.0, duration_s) for i in range(count)]


def _quintic_coeffs(p0: float, p1: float, d0: float, d1: float) -> np.ndarray:
    """Coefficients c0..c5 of s-polynomial with zero second derivative at both ends.

    d0 and d1 are first derivatives with respect to s (already scaled by the
    segment length). Constraints: p(0), p'(0), p''(0)=0, p(1), p'(1), p''(1)=0.
    """
    matrix = np.array(
        [
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 2, 0, 0, 0],
            [1, 1, 1, 1, 1, 1],
            [0, 1, 2, 3, 4, 5],
            [0, 0, 2, 6, 12, 20],
        ],
        dtype=float,
    )
    rhs = np.array([p0, d0, 0.0, p1, d1, 0.0])
    return np.linalg.solve(matrix, rhs)


class Channel:
    """One piecewise quintic channel with zero acceleration at the knots."""

    def __init__(self, times, values, velocities) -> None:
        self._times = np.asarray(times, dtype=float)
        self._segments = []
        for k in range(len(self._times) - 1):
            h = self._times[k + 1] - self._times[k]
            coeffs = _quintic_coeffs(
                float(values[k]),
                float(values[k + 1]),
                float(velocities[k]) * h,
                float(velocities[k + 1]) * h,
            )
            self._segments.append((h, coeffs))

    def value(self, t: float) -> float:
        t = min(max(t, self._times[0]), self._times[-1])
        k = min(
            int(np.searchsorted(self._times, t, side="right")) - 1,
            len(self._segments) - 1,
        )
        k = max(k, 0)
        h, c = self._segments[k]
        s = (t - self._times[k]) / h
        return float(np.polyval(c[::-1], s))


def keyframe_velocities(times, values, overrides, stop) -> np.ndarray:
    """Per-keyframe velocities (units/s) for one channel.

    times/values are the knots. overrides maps keyframe index -> velocity for
    this channel. stop lists the keyframe indices where velocity is forced to 0.
    """
    n = len(times)
    if n == 2:
        slope = (values[1] - values[0]) / (times[1] - times[0])
        vel = np.array([slope, slope], dtype=float)
    else:
        vel = PchipInterpolator(times, values)(times, nu=1).astype(float)
    vel[0] = 0.0
    vel[-1] = 0.0
    for idx, v in overrides.items():
        vel[idx] = v
    for idx in stop:
        vel[idx] = 0.0
    return vel


class Trajectory:
    """The base-frame path of a design. Build with build_trajectory(design)."""

    def __init__(self, design: Design) -> None:
        self.design = design
        kfs = design.keyframes
        self.duration = float(kfs[-1].t)
        times = [kf.t for kf in kfs]
        self.mode = design.orientation.mode
        names = {
            "x": [kf.pos[0] for kf in kfs],
            "y": [kf.pos[1] for kf in kfs],
            "z": [kf.pos[2] for kf in kfs],
        }
        if self.mode == "angles":
            names["pan"] = [kf.pan for kf in kfs]
            names["tilt"] = [kf.tilt for kf in kfs]
        elif design.orientation.target is not None:
            fixed = design.orientation.target
            names.update({"tx": [fixed[0]] * len(kfs), "ty": [fixed[1]] * len(kfs),
                          "tz": [fixed[2]] * len(kfs)})  # fmt: skip
        else:
            names.update({"tx": [kf.target[0] for kf in kfs],
                          "ty": [kf.target[1] for kf in kfs],
                          "tz": [kf.target[2] for kf in kfs]})  # fmt: skip
        self._channels: dict[str, Channel] = {}
        for name, values in names.items():
            overrides = {i: kf.vel[name] for i, kf in enumerate(kfs) if name in kf.vel}
            stop = [i for i, kf in enumerate(kfs) if kf.stop]
            vel = keyframe_velocities(times, values, overrides, stop)
            self._channels[name] = Channel(times, values, vel)

    def _raw(self, t: float) -> dict[str, float]:
        t = min(max(t, 0.0), self.duration)
        return {name: ch.value(t) for name, ch in self._channels.items()}

    def sample(self, t: float) -> Sample:
        """Sample at time t. In target mode pan/tilt come from the target direction
        without unwrapping; sample_path unwraps them across the whole path."""
        raw = self._raw(t)
        pos = (raw["x"], raw["y"], raw["z"])
        if self.mode == "angles":
            return Sample(t, pos, raw["pan"], raw["tilt"], None)
        target = (raw["tx"], raw["ty"], raw["tz"])
        d = np.asarray(target) - np.asarray(pos)
        pan = math.degrees(math.atan2(d[1], d[0]))
        tilt = math.degrees(math.atan2(d[2], math.hypot(d[0], d[1])))
        return Sample(t, pos, pan, tilt, target)


def build_trajectory(design: Design) -> Trajectory:
    """Build the trajectory of a design. The design is converted to base first."""
    return Trajectory(design_to_base(design))


def sample_path(design: Design) -> list[Sample]:
    """All samples of the design, in base frame, at the controller period."""
    traj = build_trajectory(design)
    samples = [
        traj.sample(t) for t in sample_times(traj.duration, design.sample_period_ms)
    ]
    if traj.mode == "target":
        unwrapped = unwrap_deg([s.pan for s in samples])
        samples = [
            Sample(s.t, s.pos, p, s.tilt, s.target)
            for s, p in zip(samples, unwrapped, strict=True)
        ]
    return samples


def to_sdk_pose(sample: Sample, roll_deg: float = 0.0) -> list[float]:
    """SDK pose [x, y, z, rx, ry, rz] of the camera tool in the base frame,
    with a constant roll about the optical axis (0 landscape, 90 portrait)."""
    rotation = level_rotation(sample.pan, sample.tilt, roll_deg)
    rx, ry, rz = rotation_to_sdk_euler(rotation)
    return [*sample.pos, rx, ry, rz]
