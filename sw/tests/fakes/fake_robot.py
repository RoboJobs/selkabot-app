"""A fake `fr5.RobotClient` for testing `FR5` without hardware.

`GetInverseKinRef`/`GetForwardKin` are exact inverses of each other, built
from a fixed deterministic mapping (not real kinematics), so tests can round
trip a pose through "IK then FK" and get the same pose back.

The three SDK upload calls we must never use (`TrajectoryJUpLoad`,
`LuaUpload`, `PointTableUpLoad`) raise `AssertionError` if called, so a test
that accidentally exercises them fails loudly instead of silently passing.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


def _pose_to_joints(pose6: list[float]) -> list[float]:
    """A fixed, invertible, deterministic stand-in for real IK."""
    x, y, z, rx, ry, rz = pose6
    return [x / 100.0, y / 100.0, z / 100.0, rx, ry, rz]


def _joints_to_pose(joints6: list[float]) -> list[float]:
    j1, j2, j3, j4, j5, j6 = joints6
    return [j1 * 100.0, j2 * 100.0, j3 * 100.0, j4, j5, j6]


@dataclass
class FakeStatePkg:
    """The SDK's state cache. `motion_done` can be scripted: each read pops
    the next value of `motion_script` (the last value repeats)."""

    jt_cur_pos: list[float] = field(default_factory=lambda: [0.0] * 6)
    tl_cur_pos: list[float] = field(default_factory=lambda: [0.0] * 6)
    motion_script: list[int] = field(default_factory=lambda: [1])
    mc_queue_len: int = 0
    robot_mode: int = 1  # 1 = manual, 0 = auto
    main_code: int = 0
    sub_code: int = 0
    tool: int = 0
    # Program state (1 stopped, 2 running, 3 paused), scripted like motion_done.
    program_script: list[int] = field(default_factory=lambda: [1])

    @property
    def robot_state(self) -> int:
        if len(self.program_script) > 1:
            return self.program_script.pop(0)
        return self.program_script[0]

    @property
    def motion_done(self) -> int:
        if len(self.motion_script) > 1:
            return self.motion_script.pop(0)
        return self.motion_script[0]

    @motion_done.setter
    def motion_done(self, value: int) -> None:
        self.motion_script = [value]


class _FakeProxy:
    """Stands in for `RobotClient.robot`, the raw XML-RPC proxy."""

    def __init__(self, outer: "FakeRobotClient") -> None:
        self._outer = outer

    def FileUpload(self, file_type: int, name: str) -> int:
        self._outer.file_upload_calls.append((file_type, name))
        return self._outer.file_upload_rtn

    def LuaUpLoadUpdate(self, name: str) -> list:
        self._outer.lua_registered.append(name)
        return [self._outer.lua_update_rtn, ""]


class FakeRobotClient:
    """Records every call. `errors[name] = code` injects a failure for `name`."""

    def __init__(self, ip: str = "192.168.50.2") -> None:
        self.ip_address = ip
        self.robot_state_pkg = FakeStatePkg()
        self.robot = _FakeProxy(self)
        self.calls: list[tuple[str, tuple[Any, ...], dict[str, Any]]] = []
        self.errors: dict[str, int] = {}
        self.closed = False

        self.file_upload_rtn = 0
        self.file_upload_calls: list[tuple[int, str]] = []
        self.lua_registered: list[str] = []  # names passed to LuaUpLoadUpdate
        self.lua_update_rtn = 0
        self.files: dict[str, bytes] = {}  # remote path -> bytes, for ComputeFileMD5
        self.deleted_files: list[str] = []
        self.tpd_loaded: str | None = None
        self.tpd_start_poses: dict[str, list[float]] = {}
        self.move_tpd_calls: list[tuple[str, int, float]] = []
        self.move_j_calls: list[tuple[list[float], int, int, float]] = []
        # motion_done values a move produces (started, then done); tests
        # override these to simulate a motion that never starts or never ends.
        self.move_tpd_script: list[int] = [0, 0, 1]
        self.move_j_script: list[int] = [0, 1]
        self.move_tpd_block: Any = None  # a threading.Event to block MoveTPD on
        self.speeds: list[int] = []
        self.modes: list[int] = []
        self.servo_j_calls: list[tuple[list[float], list[float], float]] = []
        self.servo_move_started = False
        self.servo_move_ended = False
        self.programs_loaded: list[str] = []
        self.program_runs = 0
        self.program_stops = 0
        # robot_state values a ProgramRun produces (running, then stopped).
        self.program_run_script: list[int] = [2, 2, 1]

        self.install_angle: list[float] = [0.0, 0.0]
        self.soft_limits: list[float] = [
            -175, 175, -265, 85, -160, 160, -265, 85, -175, 175, -175, 175,
        ]  # fmt: skip

    def _record(self, name: str, args: tuple, kwargs: dict) -> None:
        self.calls.append((name, args, kwargs))

    def _err(self, name: str) -> int | None:
        return self.errors.get(name)

    # -- banned upload calls --------------------------------------------

    def TrajectoryJUpLoad(self, *a, **k) -> None:
        raise AssertionError("TrajectoryJUpLoad must never be called (see fr5.py)")

    def LuaUpload(self, *a, **k) -> None:
        raise AssertionError("LuaUpload must never be called (see fr5.py)")

    def PointTableUpLoad(self, *a, **k) -> None:
        raise AssertionError("PointTableUpLoad must never be called (see fr5.py)")

    # -- kinematics -------------------------------------------------------

    def GetInverseKinRef(
        self, type: int, desc_pos: list[float], joint_pos_ref: list[float]
    ) -> tuple[int, list[float] | None]:
        self._record("GetInverseKinRef", (type, desc_pos, joint_pos_ref), {})
        code = self._err("GetInverseKinRef")
        if code:
            return code, None
        return 0, _pose_to_joints(desc_pos)

    def GetForwardKin(self, joint_pos: list[float]) -> tuple[int, list[float] | None]:
        self._record("GetForwardKin", (joint_pos,), {})
        code = self._err("GetForwardKin")
        if code:
            return code, None
        return 0, _joints_to_pose(joint_pos)

    def GetActualTCPNum(self, flag: int = 1) -> tuple[int, int | None]:
        self._record("GetActualTCPNum", (), {"flag": flag})
        code = self._err("GetActualTCPNum")
        if code:
            return code, None
        return 0, self.robot_state_pkg.tool

    def GetJointSoftLimitDeg(self, flag: int = 1) -> tuple[int, list[float] | None]:
        self._record("GetJointSoftLimitDeg", (), {"flag": flag})
        code = self._err("GetJointSoftLimitDeg")
        if code:
            return code, None
        return 0, list(self.soft_limits)

    def GetRobotInstallAngle(self) -> tuple[int, list[float] | None]:
        self._record("GetRobotInstallAngle", (), {})
        code = self._err("GetRobotInstallAngle")
        if code:
            return code, None
        return 0, list(self.install_angle)

    # -- controller files -------------------------------------------------

    def ComputeFileMD5(self, file_path: str) -> tuple[int, str | None]:
        self._record("ComputeFileMD5", (file_path,), {})
        code = self._err("ComputeFileMD5")
        if code:
            return code, None
        import hashlib

        data = self.files.get(file_path)
        if data is None:
            return 4, None  # controller's "not found" code
        return 0, hashlib.md5(data).hexdigest()

    def TrajectoryJDelete(self, fileName: str) -> int:
        self._record("TrajectoryJDelete", (fileName,), {})
        code = self._err("TrajectoryJDelete")
        if code:
            return code
        self.deleted_files.append(fileName)
        return 0

    # -- TPD ----------------------------------------------------------------

    def LoadTPD(self, name: str) -> int:
        self._record("LoadTPD", (name,), {})
        code = self._err("LoadTPD")
        if code:
            return code
        if name not in self.tpd_start_poses:
            return 8  # "tpd file does not exist"
        self.tpd_loaded = name
        return 0

    def GetTPDStartPose(self, name: str) -> tuple[int, list[float] | None]:
        self._record("GetTPDStartPose", (name,), {})
        code = self._err("GetTPDStartPose")
        if code:
            return code, None
        if name not in self.tpd_start_poses:
            return 8, None  # "tpd file does not exist"
        return 0, list(self.tpd_start_poses[name])

    def MoveTPD(self, name: str, blend: int, ovl: float) -> int:
        self._record("MoveTPD", (name, blend, ovl), {})
        code = self._err("MoveTPD")
        if code:
            return code
        self.move_tpd_calls.append((name, blend, ovl))
        if self.move_tpd_block is not None:
            self.move_tpd_block.wait()
        self.robot_state_pkg.motion_script = list(self.move_tpd_script)
        return 0

    # -- mode, speed, motion --------------------------------------------

    def Mode(self, state: int) -> int:
        self._record("Mode", (state,), {})
        code = self._err("Mode")
        if code:
            return code
        self.modes.append(state)
        self.robot_state_pkg.robot_mode = state
        return 0

    def SetSpeed(self, vel: int) -> int:
        self._record("SetSpeed", (vel,), {})
        code = self._err("SetSpeed")
        if code:
            return code
        self.speeds.append(vel)
        return 0

    def MoveJ(
        self,
        joint_pos: list[float],
        tool: int,
        user: int,
        vel: float = 20.0,
        blendT: float = -1.0,
    ) -> int:
        kwargs = {"vel": vel} if blendT == -1.0 else {"vel": vel, "blendT": blendT}
        self._record("MoveJ", (joint_pos, tool, user), kwargs)
        code = self._err("MoveJ")
        if code:
            return code
        self.move_j_calls.append((list(joint_pos), tool, user, vel))
        self.robot_state_pkg.jt_cur_pos = list(joint_pos)
        if blendT != -1.0:
            self.robot_state_pkg.motion_script = list(self.move_j_script)
        return 0

    def StopMotion(self) -> int:
        self._record("StopMotion", (), {})
        code = self._err("StopMotion")
        if code:
            return code
        return 0

    def CloseRPC(self) -> None:
        self.closed = True

    # -- servo streaming (D9, simulator preview only) -----------------------

    def ServoMoveStart(self) -> int:
        self._record("ServoMoveStart", (), {})
        code = self._err("ServoMoveStart")
        if code:
            return code
        self.servo_move_started = True
        return 0

    def ServoMoveEnd(self) -> int:
        self._record("ServoMoveEnd", (), {})
        code = self._err("ServoMoveEnd")
        if code:
            return code
        self.servo_move_ended = True
        return 0

    def ServoJ(
        self, joint_pos: list[float], axisPos: list[float], cmdT: float = 0.008
    ) -> int:
        self._record("ServoJ", (joint_pos, axisPos), {"cmdT": cmdT})
        code = self._err("ServoJ")
        if code:
            return code
        self.servo_j_calls.append((list(joint_pos), list(axisPos), cmdT))
        self.robot_state_pkg.jt_cur_pos = list(joint_pos)
        return 0

    # -- Lua programs ---------------------------------------------------------

    def ProgramLoad(self, program_name: str) -> int:
        self._record("ProgramLoad", (program_name,), {})
        code = self._err("ProgramLoad")
        if code:
            return code
        self.programs_loaded.append(program_name)
        return 0

    def ProgramRun(self) -> int:
        self._record("ProgramRun", (), {})
        code = self._err("ProgramRun")
        if code:
            return code
        self.program_runs += 1
        self.robot_state_pkg.program_script = list(self.program_run_script)
        return 0

    def ProgramStop(self) -> int:
        self._record("ProgramStop", (), {})
        code = self._err("ProgramStop")
        if code:
            return code
        self.program_stops += 1
        self.robot_state_pkg.program_script = [1]
        return 0
