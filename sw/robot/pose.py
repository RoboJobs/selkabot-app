"""Orientation maths for the level camera.

The camera orientation is built from two angles, never interpolated as a
rotation: pan (heading of the optical axis about base +Z) and tilt (elevation
of the optical axis). Because the image-right axis stays horizontal for every
(pan, tilt), the horizon stays level by construction.

Frames: base B has z up. The camera tool frame has z = optical axis,
x = image right, y = image down.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np
from scipy.spatial.transform import Rotation

# Fixed-axis XYZ, extrinsic. The manual defines rx, ry, rz as rotations about
# fixed axes, and the simulator confirmed it (Phase 0, Q1).
SDK_EULER = "xyz"

# Camera axes at pan = 0, tilt = 0: x_cam = (0,-1,0), y_cam = (0,0,-1),
# z_cam = (1,0,0). Columns are the camera axes in base coordinates.
_R_LEVEL0 = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ]
)


class DegenerateTarget(ValueError):
    """The camera and the target are at the same point."""


def _rz(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rx(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def level_rotation(pan_deg: float, tilt_deg: float) -> np.ndarray:
    """Camera-to-base rotation for a pan and tilt, with zero roll."""
    return _rz(pan_deg) @ _R_LEVEL0 @ _rx(tilt_deg)


def look_at_angles(
    pos: Sequence[float], target: Sequence[float]
) -> tuple[float, float]:
    """Pan and tilt (deg) that point the optical axis from pos at target.

    Raises DegenerateTarget when the two points coincide. Near-vertical
    targets return a finite value, but pan is then unstable; the validator
    rejects those (see validate.check_target_geometry).
    """
    d = np.asarray(target, dtype=float) - np.asarray(pos, dtype=float)
    if np.linalg.norm(d) < 1e-9:
        raise DegenerateTarget("camera position and target coincide")
    pan = math.degrees(math.atan2(d[1], d[0]))
    tilt = math.degrees(math.atan2(d[2], math.hypot(d[0], d[1])))
    return pan, tilt


def roll_error_deg(rotation: np.ndarray) -> float:
    """Roll (deg) of the camera about its optical axis, relative to a level frame.

    The level reference u is the horizontal direction perpendicular to the
    optical axis, u = normalise(z_cam x z_base). The roll is the signed angle
    from u to the image-right axis x_cam, measured about z_cam. Measuring
    "x_cam out of the horizontal plane" instead would scale a true roll by
    cos(tilt), which is wrong for a tilted camera.
    """
    z_cam = rotation[:, 2]
    x_cam = rotation[:, 0]
    u = np.cross(z_cam, [0.0, 0.0, 1.0])  # equals x_cam when the camera is level
    norm = np.linalg.norm(u)
    if norm < 1e-12:  # optical axis vertical: the level reference is undefined
        return 0.0
    u = u / norm
    sin_roll = float(np.dot(np.cross(u, x_cam), z_cam))
    cos_roll = float(np.dot(u, x_cam))
    return math.degrees(math.atan2(sin_roll, cos_roll))


def pointing_error_deg(
    rotation: np.ndarray, pos: Sequence[float], target: Sequence[float]
) -> float:
    """Angle (deg) between the optical axis and the direction to the target."""
    d = np.asarray(target, dtype=float) - np.asarray(pos, dtype=float)
    norm = np.linalg.norm(d)
    if norm < 1e-9:
        raise DegenerateTarget("camera position and target coincide")
    axis = rotation[:, 2]
    # atan2 of |cross| and dot is accurate for small angles, where acos is not.
    angle = math.atan2(np.linalg.norm(np.cross(axis, d)), float(np.dot(axis, d)))
    return math.degrees(angle)


def rotation_to_sdk_euler(rotation: np.ndarray) -> tuple[float, float, float]:
    """SDK (rx, ry, rz) in degrees for a rotation matrix."""
    rx, ry, rz = Rotation.from_matrix(rotation).as_euler(SDK_EULER, degrees=True)
    return float(rx), float(ry), float(rz)


def sdk_euler_to_rotation(rx: float, ry: float, rz: float) -> np.ndarray:
    """Rotation matrix for SDK (rx, ry, rz) in degrees."""
    return Rotation.from_euler(SDK_EULER, [rx, ry, rz], degrees=True).as_matrix()


def camera_angles(rx: float, ry: float, rz: float) -> tuple[float, float, float]:
    """(pan, tilt, roll) in degrees of a camera whose SDK orientation is rx, ry, rz.

    pan and tilt describe where the optical axis points; roll is the rotation
    about it relative to level (roll_error_deg). "Horizontal" means roll = 0:
    image-right has no vertical component (x_cam . z_base = 0) and image-down
    points downward (y_cam . z_base < 0).
    """
    rotation = sdk_euler_to_rotation(rx, ry, rz)
    z_cam = rotation[:, 2]
    pan = math.degrees(math.atan2(z_cam[1], z_cam[0]))
    tilt = math.degrees(math.atan2(z_cam[2], math.hypot(z_cam[0], z_cam[1])))
    return pan, tilt, roll_error_deg(rotation)


def unwrap_deg(values: Sequence[float]) -> list[float]:
    """Remove ±360° jumps from a sequence of angles in degrees."""
    radians = np.unwrap(np.radians(np.asarray(values, dtype=float)))
    return [float(v) for v in np.degrees(radians)]
