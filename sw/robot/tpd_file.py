"""Reads and writes the TPD recording file format (see design doc, "TPD file
format"). This is the format the teach pendant records, and the format
`LoadTPD`/`MoveTPD` load and play on the real robot.

Header: `N,1,period_ms,tool_id,0,0`. Each of the N data rows has 15
comma-separated values: 6 joint angles (deg), the TCP pose x,y,z (mm) and
rx,ry,rz (deg), then `period_ms,0,17`. Numbers are written with 4 decimals,
matching the recordings this format was reverse-engineered from.
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class TpdFile:
    num_points: int
    period_ms: int
    tool_id: int
    joints: list[list[float]]
    poses: list[list[float]]


def write(
    path: str | Path,
    joints: list[list[float]],
    poses: list[list[float]],
    period_ms: int,
    tool_id: int,
) -> None:
    """Write a TPD file. Raises ValueError if joints/poses mismatch in length
    or any row does not have exactly 6 values."""
    if len(joints) != len(poses):
        raise ValueError(
            f"joints has {len(joints)} rows, poses has {len(poses)}; must match"
        )
    for i, (j, p) in enumerate(zip(joints, poses, strict=True)):
        if len(j) != 6:
            raise ValueError(f"joints[{i}] has {len(j)} values, expected 6")
        if len(p) != 6:
            raise ValueError(f"poses[{i}] has {len(p)} values, expected 6")

    n = len(joints)
    lines = [f"{n},1,{period_ms},{tool_id},0,0"]
    for j, p in zip(joints, poses, strict=True):
        values = [*j, *p]
        formatted = ",".join(f"{v:.4f}" for v in values)
        lines.append(f"{formatted},{period_ms},0,17")
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


def read(path: str | Path) -> TpdFile:
    """Read a TPD file. Does not validate N against the actual row count;
    callers that need that check it themselves (e.g. against GetTPDStartPose
    or GetTrajectoryPointNum)."""
    text = Path(path).read_text(encoding="utf-8")
    lines = [line for line in text.splitlines() if line.strip()]
    if not lines:
        raise ValueError(f"{path}: empty file")
    header = [float(v) for v in lines[0].split(",")]
    if len(header) != 6:
        raise ValueError(f"{path}: header has {len(header)} values, expected 6")
    num_points, _one, period_ms, tool_id = (int(v) for v in header[:4])

    joints: list[list[float]] = []
    poses: list[list[float]] = []
    for line in lines[1:]:
        values = [float(v) for v in line.split(",")]
        if len(values) != 15:
            raise ValueError(f"{path}: data row has {len(values)} values, expected 15")
        joints.append(values[0:6])
        poses.append(values[6:12])
    return TpdFile(num_points, period_ms, tool_id, joints, poses)


def md5(path: str | Path) -> str:
    return hashlib.md5(Path(path).read_bytes()).hexdigest()
