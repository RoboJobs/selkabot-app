"""Frame conversion between the authoring frame A and the robot base frame B.

Keyframes, targets and the workspace box are written in A. design_to_base
converts everything to B once; all later maths (sampling, look-at directions,
IK, validation) works in B, so camera position and target are always in the
same frame. The conversion is a translation plus a rotation about the vertical,
so it preserves the level-horizon property.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np

from sw.robot.schema import Anchor, Design, Orientation


def _rz(deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def authoring_to_base(point, anchor: Anchor) -> np.ndarray:
    """Map a point in A into B: p_B = Rz(yaw) · p_A + pos."""
    p = np.asarray(point, dtype=float)
    return _rz(anchor.yaw_deg) @ p + np.asarray(anchor.pos, dtype=float)


def base_to_authoring(point, anchor: Anchor) -> np.ndarray:
    """Inverse of authoring_to_base: p_A = Rz(-yaw) · (p_B - pos)."""
    p = np.asarray(point, dtype=float) - np.asarray(anchor.pos, dtype=float)
    return _rz(-anchor.yaw_deg) @ p


def _tuple(point) -> tuple[float, float, float]:
    return tuple(float(v) for v in point)


def design_to_base(design: Design) -> Design:
    """Return a copy with every position, target and pan expressed in B.

    The anchor is kept unchanged so that the workspace box, which is defined
    in A, can still be checked with base_to_authoring.
    """
    anchor = design.anchor
    keyframes = []
    for kf in design.keyframes:
        pan = kf.pan
        if pan is not None:
            pan = pan + anchor.yaw_deg
        target = kf.target
        if target is not None:
            target = _tuple(authoring_to_base(target, anchor))
        keyframes.append(
            dataclasses.replace(
                kf,
                pos=_tuple(authoring_to_base(kf.pos, anchor)),
                pan=pan,
                target=target,
            )
        )
    orientation: Orientation = design.orientation
    if orientation.target is not None:
        orientation = dataclasses.replace(
            orientation, target=_tuple(authoring_to_base(orientation.target, anchor))
        )
    return dataclasses.replace(
        design, keyframes=tuple(keyframes), orientation=orientation
    )


def within_workspace(points_base: np.ndarray, design: Design) -> np.ndarray:
    """Boolean mask: which base-frame points lie inside the box, checked in A."""
    pts_a = np.array([base_to_authoring(p, design.anchor) for p in points_base])
    lo = np.asarray(design.workspace_min, dtype=float)
    hi = np.asarray(design.workspace_max, dtype=float)
    return np.all((pts_a >= lo) & (pts_a <= hi), axis=1)
