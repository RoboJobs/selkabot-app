"""A fake `validate.Kinematics` with a real spherical wrist, for portrait tests.

`FakeKinematics` maps the wrist joints straight to the SDK Euler angles, so a
portrait camera (ry = 90, the Euler singularity) would be singular inside the
fake itself. `WristFK` instead models:

- the arm: J1-J3 place the wrist centre (a toy linear map, like FakeKinematics);
- the wrist: flange rotation Rz(J4) . Ry(J5) . Rz(J6) (intrinsic ZYZ), singular
  at J5 = 0/180 like the real wrist;
- the camera mount: camera rotation = flange rotation . MOUNT, and the camera
  sits `offset_mm` from the wrist centre (in the flange frame), so re-orienting
  the camera with the wrist moves it.

With the default mount, the flange z axis is the reversed optical axis: a
horizontal camera has J5 = -90 in landscape and in portrait (portrait differs
by J6), so portrait is not singular. With `SINGULAR_PORTRAIT_MOUNT` the flange
z axis is the reversed image-x axis: landscape has J5 = -90 but portrait
(image-x down) puts J5 at 0, the wrist singularity.

This fake exercises the contracts (roll targets, singularity, Euler
representation); it does not prove the real mount supports any maneuver.
"""

from __future__ import annotations

import math
import warnings
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation

from sw.robot.pose import rotation_to_sdk_euler, sdk_euler_to_rotation

J3_OFFSET = 45.0
ARM_SCALE = 100.0  # mm per degree for the toy arm

# Flange z = reversed optical axis (row 3 of the mount = (0, 0, -1)); the
# extra Rz puts a level landscape camera at J6 = 0 (portrait at J6 = -90).
DEFAULT_MOUNT = (
    Rotation.from_euler("z", -90, degrees=True).as_matrix()
    @ Rotation.from_euler("x", 180, degrees=True).as_matrix()
)
# Flange z = reversed image-x axis (row 3 of the mount = (-1, 0, 0)).
SINGULAR_PORTRAIT_MOUNT = Rotation.from_euler("y", 90, degrees=True).as_matrix()


def _wrist_rotation(j4: float, j5: float, j6: float) -> np.ndarray:
    return Rotation.from_euler("ZYZ", [j4, j5, j6], degrees=True).as_matrix()


def _near(angle: float, ref: float) -> float:
    """`angle` shifted by a multiple of 360 to be closest to `ref`."""
    return angle + 360.0 * round((ref - angle) / 360.0)


def _alternate_euler(rx: float, ry: float, rz: float) -> list[float]:
    """The other SDK Euler triple of the same rotation (fixed XYZ)."""

    def wrap(a: float) -> float:
        return (a + 180.0) % 360.0 - 180.0

    return [wrap(rx + 180.0), wrap(180.0 - ry), wrap(rz + 180.0)]


@dataclass
class WristFK:
    tool_id: int = 1
    mount: np.ndarray = field(default_factory=lambda: DEFAULT_MOUNT.copy())
    offset_mm: tuple[float, float, float] = (100.0, 0.0, 150.0)
    install_angle_deg: tuple[float, float] = (0.0, 0.0)
    soft_limits: list[tuple[float, float]] = field(
        default_factory=lambda: (
            [(-175, 175), (-265, 85), (-160, 160)]
            + [(-265, 85), (-175, 175), (-175, 175)]
        )
    )
    # Return the other Euler triple of the same rotation (tests that pose
    # comparisons use rotations, not Euler angles).
    alternate_euler: bool = False
    fk_rot_offset_deg: float = 0.0  # extra rotation about base z, to break FK

    def fk(self, joints6: list[float]) -> list[float]:
        j1, j2, j3, j4, j5, j6 = joints6
        flange = _wrist_rotation(j4, j5, j6)
        camera = flange @ self.mount
        if self.fk_rot_offset_deg:
            camera = (
                Rotation.from_euler(
                    "z", self.fk_rot_offset_deg, degrees=True
                ).as_matrix()
                @ camera
            )
        centre = np.array([j1, j2, j3 - J3_OFFSET]) * ARM_SCALE
        pos = centre + flange @ np.asarray(self.offset_mm)
        euler = list(rotation_to_sdk_euler(camera))
        if self.alternate_euler:
            euler = _alternate_euler(*euler)
        return [*map(float, pos), *euler]

    def ik(self, pose6: list[float], ref6: list[float]) -> list[float] | None:
        camera = sdk_euler_to_rotation(*pose6[3:])
        flange = camera @ self.mount.T
        with warnings.catch_warnings():
            warnings.filterwarnings("ignore", message="Gimbal lock detected")
            a, b, c = Rotation.from_matrix(flange).as_euler("ZYZ", degrees=True)
        best = None
        for cand in ((a, b, c), (a + 180.0, -b, c + 180.0)):
            wrist = [_near(v, r) for v, r in zip(cand, ref6[3:], strict=True)]
            dist = sum(abs(v - r) for v, r in zip(wrist, ref6[3:], strict=True))
            if best is None or dist < best[0]:
                best = (dist, wrist)
        assert best is not None
        wrist = best[1]
        centre = np.asarray(pose6[:3]) - _wrist_rotation(*wrist) @ np.asarray(
            self.offset_mm
        )
        arm = [
            centre[0] / ARM_SCALE,
            centre[1] / ARM_SCALE,
            centre[2] / ARM_SCALE + J3_OFFSET,
        ]
        return [*map(float, arm), *map(float, wrist)]

    def active_tool(self) -> int:
        return self.tool_id

    def install_angle(self) -> tuple[float, float]:
        return self.install_angle_deg

    def joint_soft_limits(self) -> list[tuple[float, float]]:
        return list(self.soft_limits)


def j5_margin_deg(joints6: list[float]) -> float:
    """Distance (deg) of J5 from the wrist singularity at 0/180."""
    return math.degrees(math.asin(min(1.0, abs(math.sin(math.radians(joints6[4]))))))
