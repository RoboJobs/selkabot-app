"""Joint-space interpolation for `interpolation: joint` designs.

Each joint is its own piecewise quintic Hermite channel through the keyframe
joint vectors (the same machinery as the Cartesian channels in path.py: zero
acceleration at every keyframe, PCHIP velocities at interior keyframes, rest
at the first and last keyframe). This is what the teach pendant's own joint
move does, and it is the right primitive for large swings: joint limits are a
box, so the blend of two in-limit keyframes stays in limits, and the joints
move smoothly even when a keyframe sits at a singular configuration (e.g. the
arm fully extended), where a Cartesian path would make the joints blow up.

The camera orientation between keyframes is whatever the arm does. On this
arm it stays level exactly when J5 and J6 are equal at every keyframe (J1
then only pans and J2+J3+J4 only tilts); validate.check_roll measures it.
"""

from __future__ import annotations

import math

import numpy as np

from sw.robot.path import Channel, keyframe_velocities, sample_times
from sw.robot.schema import Design


def cruise_shape(tau: float, ramp: float) -> float:
    """Progress 0..1 at normalised time tau for a rest-to-rest move that
    ramps up for `ramp` of the time, cruises, then ramps down symmetrically.

    The ramps are raised-cosine in velocity, so acceleration is continuous
    and zero at both ends. Peak velocity is 1/(1 - ramp) times the average,
    against 1.875 for the minimum-jerk quintic: for the same peak joint speed
    the move finishes sooner (ramp 0.25: 29% less time). ramp = 0.5 means no
    cruise.
    """
    tau = min(max(tau, 0.0), 1.0)
    if tau > 0.5:
        return 1.0 - cruise_shape(1.0 - tau, ramp)
    vmax = 1.0 / (1.0 - ramp)
    if tau < ramp:
        return vmax * (
            tau / 2.0 - ramp / (2.0 * math.pi) * math.sin(math.pi * tau / ramp)
        )
    return vmax * (ramp / 2.0 + (tau - ramp))


class ShapedChannel(Channel):
    """A Channel whose rest-to-rest segments (zero velocity at both ends)
    follow cruise_shape instead of the minimum-jerk quintic. Other segments
    (moving through a keyframe) keep the quintic Hermite."""

    def __init__(self, times, values, velocities, ramp: float | None) -> None:
        super().__init__(times, values, velocities)
        self._values = [float(v) for v in values]
        self._rest = [
            ramp is not None and velocities[k] == 0 and velocities[k + 1] == 0
            for k in range(len(times) - 1)
        ]
        self._ramp = ramp

    def value(self, t: float) -> float:
        t = min(max(t, self._times[0]), self._times[-1])
        k = max(
            min(int(np.searchsorted(self._times, t, side="right")) - 1,
                len(self._segments) - 1),
            0,
        )  # fmt: skip
        if not self._rest[k]:
            return super().value(t)
        h = self._times[k + 1] - self._times[k]
        p0, p1 = self._values[k], self._values[k + 1]
        return p0 + (p1 - p0) * cruise_shape((t - self._times[k]) / h, self._ramp)


def make_channel(times, values, stop, ramp: float | None) -> Channel:
    """The channel joint mode uses for joints and for pan/tilt targets, so
    the arm and the camera's aim progress together."""
    velocities = keyframe_velocities(times, values, {}, stop)
    if ramp is None:
        return Channel(times, values, velocities)
    return ShapedChannel(times, values, velocities, ramp)


def joint_channels(
    times: list[float],
    keyframe_joints: list[list[float]],
    stop: list[int],
    ramp: float | None = None,
) -> list[Channel]:
    """One channel per joint through the keyframe joint vectors."""
    return [
        make_channel(times, [kj[j] for kj in keyframe_joints], stop, ramp)
        for j in range(6)
    ]


def sample_joint_path(
    design: Design, keyframe_joints: list[list[float]]
) -> tuple[list[float], list[list[float]]]:
    """Sample times and joint vectors at the design's period (path.py's rule)."""
    if len(keyframe_joints) != len(design.keyframes):
        raise ValueError("need one joint vector per keyframe")
    times = [kf.t for kf in design.keyframes]
    stop = [i for i, kf in enumerate(design.keyframes) if kf.stop]
    channels = joint_channels(times, keyframe_joints, stop, design.ramp_fraction)
    ts = sample_times(times[-1], design.sample_period_ms)
    return ts, [[ch.value(t) for ch in channels] for t in ts]
