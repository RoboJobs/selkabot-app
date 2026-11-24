"""A fake `validate.Kinematics` with analytic, invertible IK/FK. No hardware."""

from __future__ import annotations

from dataclasses import dataclass, field

# Offsets keep J3 (elbow) and J5 (wrist) away from 0, like a real bent arm, so
# the singularity check does not reject every fake design.
J3_OFFSET = 45.0
J5_OFFSET = -90.0


def _pose_to_joints(pose6: list[float]) -> list[float]:
    """A fixed, invertible, deterministic stand-in for real IK."""
    x, y, z, rx, ry, rz = pose6
    return [x / 100.0, y / 100.0, z / 100.0 + J3_OFFSET, rx, ry + J5_OFFSET, rz]


def _joints_to_pose(joints6: list[float]) -> list[float]:
    j1, j2, j3, j4, j5, j6 = joints6
    return [j1 * 100.0, j2 * 100.0, (j3 - J3_OFFSET) * 100.0, j4, j5 - J5_OFFSET, j6]


@dataclass
class FakeKinematics:
    tool_id: int = 0
    install_angle_deg: tuple[float, float] = (0.0, 0.0)
    soft_limits: list[tuple[float, float]] = field(
        default_factory=lambda: [(-175, 175)] * 6
    )
    # Test hooks:
    fail_ik_at: set[int] = field(default_factory=set)  # sample indices -> None
    joint_offset: dict[int, float] = field(default_factory=dict)  # joint -> deg bias
    fk_offset_mm: float = 0.0  # added to the FK position, to break the round trip
    _ik_calls: int = field(default=0, init=False)

    def ik(self, pose6: list[float], ref6: list[float]) -> list[float] | None:
        index = self._ik_calls
        self._ik_calls += 1
        if index in self.fail_ik_at:
            return None
        joints = _pose_to_joints(pose6)
        for j, offset in self.joint_offset.items():
            joints[j] += offset
        return joints

    def fk(self, joints6: list[float]) -> list[float]:
        pose = _joints_to_pose(joints6)
        pose[0] += self.fk_offset_mm
        return pose

    def active_tool(self) -> int:
        return self.tool_id

    def install_angle(self) -> tuple[float, float]:
        return self.install_angle_deg

    def joint_soft_limits(self) -> list[tuple[float, float]]:
        return list(self.soft_limits)
