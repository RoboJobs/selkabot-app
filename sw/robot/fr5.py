"""Hardware boundary for the Fairino FR5. Only this module imports `fairino`.

Every call into the robot goes through `FR5`, which wraps a `RobotClient`
(the SDK's `Robot.RPC` instance, or a fake for tests). See the design doc,
"Robot call reference" and "Controller file transfer", for the calls used
here and why. In particular: never use the SDK's `TrajectoryJUpLoad`,
`LuaUpload` or `PointTableUpLoad` -- they race the controller and fail with
-1. `upload_file` implements the protocol ourselves.
"""

from __future__ import annotations

import hashlib
import os
import socket
import time
from contextlib import contextmanager
from typing import Any, Protocol

DATA_PORT = 20010
LUA_FILE_TYPE = 0
ALLOWED_UPLOAD_TYPES = (LUA_FILE_TYPE, 20)  # 0 = Lua, 20 = Trajectory J (TPD files)
TRAJ_DIR = "/fruser/traj"
# MoveTPD blend: 1 = controller smoothing. Default for every motion: without
# it the joints were audibly rough at low playback speeds (2026-11-25).
TPD_BLEND_SMOOTH = 1
TPD_BLEND_NONE = 0


class FairinoError(RuntimeError):
    """A robot RPC call returned a non-zero error code."""

    def __init__(self, code: int, call: str) -> None:
        self.code = code
        self.call = call
        super().__init__(f"{call} failed with code {code}")


class FileTransferError(RuntimeError):
    """A controller file upload failed.

    `stage` is request/connect/transfer/register/verify.
    """

    def __init__(self, stage: str, detail: Any = None) -> None:
        self.stage = stage
        self.detail = detail
        super().__init__(f"upload failed at {stage}: {detail!r}")


class RobotClient(Protocol):
    """The subset of `fairino.Robot.RPC` that `FR5` calls, for fakes to satisfy."""

    robot: Any  # the XML-RPC proxy, used directly for FileUpload
    robot_state_pkg: Any
    ip_address: str

    def GetInverseKinRef(
        self, type: int, desc_pos: list[float], joint_pos_ref: list[float]
    ) -> tuple[int, list[float] | None]: ...
    def GetForwardKin(
        self, joint_pos: list[float]
    ) -> tuple[int, list[float] | None]: ...
    def GetActualTCPNum(self, flag: int = 1) -> tuple[int, int | None]: ...
    def GetJointSoftLimitDeg(self, flag: int = 1) -> tuple[int, list[float] | None]: ...
    def GetRobotInstallAngle(self) -> tuple[int, list[float] | None]: ...
    def ComputeFileMD5(self, file_path: str) -> tuple[int, str | None]: ...
    def TrajectoryJDelete(self, fileName: str) -> int: ...
    def LoadTPD(self, name: str) -> int: ...
    def GetTPDStartPose(self, name: str) -> tuple[int, list[float] | None]: ...
    def MoveTPD(self, name: str, blend: int, ovl: float) -> int: ...
    def Mode(self, state: int) -> int: ...
    def SetSpeed(self, vel: int) -> int: ...
    def MoveJ(
        self,
        joint_pos: list[float],
        tool: int,
        user: int,
        vel: float = 20.0,
        blendT: float = -1.0,
    ) -> int: ...
    def StopMotion(self) -> int: ...
    def CloseRPC(self) -> None: ...
    def ServoMoveStart(self) -> int: ...
    def ServoMoveEnd(self) -> int: ...
    def ServoJ(
        self, joint_pos: list[float], axisPos: list[float], cmdT: float = 0.008
    ) -> int: ...


def _call(client: RobotClient, name: str, *args: Any, **kwargs: Any) -> Any:
    """Invoke `name` on the client; raise FairinoError on a non-zero code.

    For an (code, value) return, give back value. For a plain int return,
    give back None.
    """
    result = getattr(client, name)(*args, **kwargs)
    if isinstance(result, tuple):
        code, value = result
        if code != 0:
            raise FairinoError(code, name)
        return value
    if result != 0:
        raise FairinoError(result, name)
    return None


def _send_file(sock: socket.socket, data: bytes) -> bytes:
    """Same wire framing as the SDK's `__FileUpLoad` (Robot.py:7181)."""
    total = len(data) + 46 + 4
    md5 = hashlib.md5(data).hexdigest()
    sock.settimeout(20)
    sock.sendall(f"/f/b{total:10d}{md5}".encode())
    sock.sendall(data)
    sock.sendall(b"/b/f")
    time.sleep(0.5)
    return sock.recv(1024)


class FR5:
    """Wraps a `RobotClient`. Every method either returns a plain value or
    raises `FairinoError` / `FileTransferError`. Nothing here moves the robot
    except `move_j`, `move_tpd`, `stream_joints` and `run_program`.
    """

    def __init__(
        self,
        client: RobotClient,
        ip: str,
        data_port: int = DATA_PORT,
        poll_s: float = 5.0,
        poll_step_s: float = 0.05,
    ) -> None:
        self.client = client
        self.ip = ip
        self._data_port = data_port
        self._poll_s = poll_s
        self._poll_step_s = poll_step_s

    @classmethod
    def connect(cls, ip: str) -> "FR5":
        from fairino import Robot  # lazy: tests never need the native SDK

        return cls(Robot.RPC(ip), ip)

    def close(self) -> None:
        self.client.CloseRPC()

    # -- Kinematics (satisfies the validate.Kinematics Protocol) -----------

    def ik(self, pose6: list[float], ref_joints6: list[float]) -> list[float] | None:
        try:
            return _call(self.client, "GetInverseKinRef", 0, pose6, ref_joints6)
        except FairinoError:
            return None

    def fk(self, joints6: list[float]) -> list[float]:
        return _call(self.client, "GetForwardKin", joints6)

    def active_tool(self) -> int:
        return _call(self.client, "GetActualTCPNum")

    def install_angle(self) -> tuple[float, float]:
        yangle, zangle = _call(self.client, "GetRobotInstallAngle")
        return yangle, zangle

    def joint_soft_limits(self) -> list[tuple[float, float]]:
        values = _call(self.client, "GetJointSoftLimitDeg")
        return [(values[2 * i], values[2 * i + 1]) for i in range(6)]

    # -- State (from the cache; never blocks on the socket) -----------------

    def joints(self) -> list[float]:
        return list(self.client.robot_state_pkg.jt_cur_pos)

    def tcp_pose(self) -> list[float]:
        return list(self.client.robot_state_pkg.tl_cur_pos)

    def motion_done(self) -> bool:
        return bool(self.client.robot_state_pkg.motion_done)

    def error_codes(self) -> tuple[int, int]:
        pkg = self.client.robot_state_pkg
        return pkg.main_code, pkg.sub_code

    def robot_mode(self) -> int:
        return self.client.robot_state_pkg.robot_mode

    # -- Mode and speed -------------------------------------------------

    def ensure_auto(self, wait_s: float = 1.0) -> None:
        """Mode(0), then wait for the state cache to catch up and confirm."""
        _call(self.client, "Mode", 0)
        time.sleep(wait_s)
        if self.robot_mode() != 0:
            raise FairinoError(self.robot_mode(), "Mode(0)")

    def set_speed(self, pct: int) -> None:
        _call(self.client, "SetSpeed", pct)

    @contextmanager
    def speed(self, pct: int, safe_default_pct: int = 10):
        """Set speed to `pct`; always try to restore `safe_default_pct` after."""
        self.set_speed(pct)
        try:
            yield
        finally:
            try:
                self.set_speed(safe_default_pct)
            except FairinoError:
                pass

    # -- Motion -----------------------------------------------------------

    def move_j(
        self, joints: list[float], tool_id: int, vel_pct: float, blocking: bool = True
    ) -> None:
        """MoveJ. Blocking waits for the motion to end inside the RPC; with
        blocking=False the call returns at once (blendT=0) and the caller
        polls motion_done(), so a stop request is never stuck behind it."""
        if blocking:
            _call(self.client, "MoveJ", joints, tool_id, 0, vel=vel_pct)
        else:
            _call(self.client, "MoveJ", joints, tool_id, 0, vel=vel_pct, blendT=0.0)

    def move_tpd(self, name: str, blend: int, ovl: float) -> None:
        _call(self.client, "MoveTPD", name, blend, ovl)

    def stop(self) -> None:
        _call(self.client, "StopMotion")

    def stream_joints(
        self, rows: list[list[float]], period_s: float, ovl: int = 100
    ) -> None:
        """Simulator preview only (D9): replay `rows` (6 joint angles each)
        with ServoJ, one row per call, paced by `period_s * 100 / ovl`.

        This is a preview, not the production path: it does not use
        LoadTPD/MoveTPD or exercise the controller's own TPD interpolation.
        Never ServoJT (torque control) -- see the design doc, "Simulator
        playback via ServoJ".
        """
        sleep_s = period_s * 100.0 / ovl
        axis_pos = [0.0, 0.0, 0.0, 0.0]
        _call(self.client, "ServoMoveStart")
        try:
            for row in rows:
                start = time.monotonic()
                _call(self.client, "ServoJ", row, axis_pos, cmdT=period_s)
                elapsed = time.monotonic() - start
                remaining = sleep_s - elapsed
                if remaining > 0:
                    time.sleep(remaining)
        finally:
            _call(self.client, "ServoMoveEnd")

    # -- Lua programs (joint playback on the controller) -------------------

    def run_program(self, remote_path: str) -> None:
        """ProgramLoad + ProgramRun. Needs auto mode (ensure_auto)."""
        _call(self.client, "ProgramLoad", remote_path)
        _call(self.client, "ProgramRun")

    def stop_program(self) -> None:
        _call(self.client, "ProgramStop")

    def program_state(self) -> int:
        """1 = stopped / no program, 2 = running, 3 = paused (state cache)."""
        return self.client.robot_state_pkg.robot_state

    # -- TPD loading --------------------------------------------------------

    def load_tpd(self, name: str) -> None:
        _call(self.client, "LoadTPD", name)

    def tpd_start_pose(self, name: str) -> list[float]:
        return _call(self.client, "GetTPDStartPose", name)

    # -- Controller files -----------------------------------------------

    def delete_traj(self, filename: str) -> None:
        _call(self.client, "TrajectoryJDelete", filename)

    def file_md5(self, remote_path: str) -> str | None:
        """None means the controller reports the file as not found (code 4)."""
        try:
            return _call(self.client, "ComputeFileMD5", remote_path)
        except FairinoError as exc:
            if exc.code == 4:
                return None
            raise

    def upload_file(self, local_path: str, file_type: int = 20) -> str:
        """Upload `local_path`, verify it, and return its remote path.

        Never calls the SDK's TrajectoryJUpLoad/LuaUpload (the race they have
        is why this method exists; see the module docstring).
        """
        if file_type not in ALLOWED_UPLOAD_TYPES:
            raise ValueError(f"file_type must be one of {ALLOWED_UPLOAD_TYPES}")
        name = os.path.basename(local_path)
        with open(local_path, "rb") as f:
            data = f.read()

        rtn = self.client.robot.FileUpload(file_type, name)
        if rtn != 0:
            raise FileTransferError("request", rtn)

        sock = self._connect_data_port()
        if sock is None:
            raise FileTransferError("connect")
        try:
            reply = _send_file(sock, data)
        finally:
            sock.close()
        if reply[:7] != b"SUCCESS":
            raise FileTransferError("transfer", reply[:64])
        if file_type == LUA_FILE_TYPE:
            # The SDK's LuaUpload follows the transfer with LuaUpLoadUpdate,
            # which registers the program in /fruser/; without it the file is
            # not there (real robot, 2026-11-25: ComputeFileMD5 "not found").
            rtn = self.client.robot.LuaUpLoadUpdate(name)
            code = rtn[0] if isinstance(rtn, (list, tuple)) else rtn
            if code != 0:
                raise FileTransferError("register", rtn)

        remote = f"{TRAJ_DIR}/{name}" if file_type == 20 else f"/fruser/{name}"
        local_md5 = hashlib.md5(data).hexdigest()
        remote_md5 = self.file_md5(remote)
        if remote_md5 is None or remote_md5.lower() != local_md5:
            raise FileTransferError("verify", remote_md5)
        return remote

    def _connect_data_port(self) -> socket.socket | None:
        start = time.monotonic()
        while time.monotonic() - start < self._poll_s:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)
            try:
                sock.connect((self.ip, self._data_port))
                return sock
            except OSError:
                sock.close()
                time.sleep(self._poll_step_s)
        return None
