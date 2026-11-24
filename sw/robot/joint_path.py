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

from sw.robot.path import Channel, keyframe_velocities, sample_times
from sw.robot.schema import Design


def joint_channels(
    times: list[float], keyframe_joints: list[list[float]], stop: list[int]
) -> list[Channel]:
    """One Channel per joint through the keyframe joint vectors."""
    channels = []
    for j in range(6):
        values = [kj[j] for kj in keyframe_joints]
        velocities = keyframe_velocities(times, values, {}, stop)
        channels.append(Channel(times, values, velocities))
    return channels


def sample_joint_path(
    design: Design, keyframe_joints: list[list[float]]
) -> tuple[list[float], list[list[float]]]:
    """Sample times and joint vectors at the design's period (path.py's rule)."""
    if len(keyframe_joints) != len(design.keyframes):
        raise ValueError("need one joint vector per keyframe")
    times = [kf.t for kf in design.keyframes]
    stop = [i for i, kf in enumerate(design.keyframes) if kf.stop]
    channels = joint_channels(times, keyframe_joints, stop)
    ts = sample_times(times[-1], design.sample_period_ms)
    return ts, [[ch.value(t) for ch in channels] for t in ts]
