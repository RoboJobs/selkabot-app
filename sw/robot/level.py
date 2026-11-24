"""Level the camera by moving only the wrist (J4, J5, J6).

Used to clean up hand-jogged keyframes (`motionctl capture`) and, at every
sample, to keep a joint-mode motion exactly level (validate.py).

Why the wrist and not a rotation about the optical axis: the camera sits
about 400 mm from the flange (tool 1), so rotating it in place swings the
flange around a large circle. Keyframes for big moves are near full reach,
where that pushes the arm out of reach. Holding J1-J3 keeps the reach; the
wrist alone sets the camera's full orientation (3 joints, 3 constraints:
roll = 0 and the wanted pan and tilt), at the cost of moving the camera a
few millimetres. Near the wrist singularity (J5 = 0 or 180) the problem is
ill-conditioned and the solve fails or the wrist moves fast; the joint
speed/acceleration checks catch that.

Pure module: needs only an `fk(joints6) -> pose6` callable (the controller's
FK via FR5, or a fake in tests).
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

from scipy.optimize import least_squares

from sw.robot.pose import camera_angles

FK = Callable[[list[float]], list[float]]
MAX_ABS_TILT_DEG = 85.0  # beyond this "level" (and pan) are ill-defined
TOLERANCE_DEG = 1e-3  # roll/pan/tilt residual accepted as solved; the
# controller returns float32 poses, so angles near 180deg only resolve to
# about 1e-5..1e-4deg. 0.001deg of roll is invisible on camera.
DIFF_STEP = 1e-4  # relative finite-difference step (~0.01deg): well above float32 noise


class LevelError(ValueError):
    """The wrist could not level the camera at this arm configuration."""


@dataclass(frozen=True)
class LevelResult:
    joints: list[float]  # J1-J3 unchanged, J4-J6 solved
    pose: list[float]  # FK of `joints`
    pan: float
    tilt: float
    roll_before: float
    moved_mm: float  # how far the camera moved


def _wrap(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def solve_wrist(
    fk: FK,
    joints: list[float],
    pan: float,
    tilt: float,
    wrist_guess: list[float] | None = None,
    roll: float = 0.0,
) -> list[float]:
    """J4-J6 that put the camera at the given pan and tilt with a constant
    roll about its optical axis (0 = landscape, 90 = portrait), keeping J1-J3
    of `joints`. Returns the full joint vector; raises LevelError when the
    orientation cannot be reached. A singular wrist (J5 near 0/180) may still
    succeed here: rejecting it is validate.check_wrist_singularity's job."""
    if abs(tilt) > MAX_ABS_TILT_DEG:
        raise LevelError(
            f"tilt {tilt:.1f}deg is too close to vertical for a level horizon"
        )
    arm = list(joints[:3])

    def residual(wrist):
        p, t, r = camera_angles(*fk([*arm, *wrist])[3:])
        return [_wrap(r - roll), _wrap(p - pan), t - tilt]

    start = list(wrist_guess) if wrist_guess is not None else list(joints[3:])
    sol = least_squares(residual, start, diff_step=DIFF_STEP, xtol=1e-10, ftol=1e-10)
    worst = max(abs(v) for v in sol.fun)
    if worst > TOLERANCE_DEG:
        raise LevelError(
            f"wrist could not level the camera (residual {worst:.4f}deg; J5 near "
            "0 or 180 is the wrist singularity)"
        )
    return [*arm, *(float(v) for v in sol.x)]


def level_keyframe(
    fk: FK,
    joints: list[float],
    pan: float | None = None,
    tilt: float | None = None,
    roll: float = 0.0,
    wrist_guess: list[float] | None = None,
) -> LevelResult:
    """Level a captured keyframe to `roll` (0 landscape, 90 portrait). pan/tilt
    default to where the camera already points, so the viewing direction is
    kept; the camera itself may move (moved_mm), since it sits off the flange."""
    before = list(fk(joints))
    pan0, tilt0, roll0 = camera_angles(*before[3:])
    pan = pan0 if pan is None else pan
    tilt = tilt0 if tilt is None else tilt
    leveled = solve_wrist(fk, joints, pan, tilt, wrist_guess, roll)
    after = list(fk(leveled))
    p, t, _r = camera_angles(*after[3:])
    return LevelResult(
        joints=leveled,
        pose=after,
        pan=p,
        tilt=t,
        roll_before=roll0,
        moved_mm=math.dist(before[:3], after[:3]),
    )
