"""Tests for the FR5 robot boundary, using FakeRobotClient. No hardware."""

from __future__ import annotations

import hashlib
import socket
import threading
import time

import pytest

from sw.robot import fr5
from sw.tests.fakes.fake_robot import FakeRobotClient


@pytest.fixture
def client():
    return FakeRobotClient()


@pytest.fixture
def robot(client):
    return fr5.FR5(client, client.ip_address)


# -- kinematics and state --------------------------------------------------


def test_ik_calls_get_inverse_kin_ref_with_type_0(robot, client):
    pose = [100.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ref = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    joints = robot.ik(pose, ref)
    assert joints == [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    name, args, kwargs = client.calls[-1]
    assert name == "GetInverseKinRef"
    assert args == (0, pose, ref)


def test_ik_returns_none_on_error(robot, client):
    client.errors["GetInverseKinRef"] = 4
    assert robot.ik([0.0] * 6, [0.0] * 6) is None


def test_fk_inverts_ik_round_trip(robot):
    pose = [250.0, -300.0, 450.0, 10.0, 20.0, 30.0]
    joints = robot.ik(pose, [0.0] * 6)
    back = robot.fk(joints)
    assert back == pytest.approx(pose)


def test_fk_raises_fairino_error_on_nonzero_code(robot, client):
    client.errors["GetForwardKin"] = 3
    with pytest.raises(fr5.FairinoError) as exc:
        robot.fk([0.0] * 6)
    assert exc.value.code == 3
    assert exc.value.call == "GetForwardKin"


def test_active_tool_unpacks_code_value(robot, client):
    client.robot_state_pkg.tool = 1
    assert robot.active_tool() == 1


def test_install_angle_returns_tuple(robot, client):
    client.install_angle = [1.5, 2.5]
    assert robot.install_angle() == (1.5, 2.5)


def test_joint_soft_limits_pairs_values(robot):
    limits = robot.joint_soft_limits()
    assert limits[0] == (-175, 175)
    assert limits[1] == (-265, 85)
    assert len(limits) == 6


def test_state_readers_use_the_cache_not_an_rpc(robot, client):
    client.robot_state_pkg.jt_cur_pos = [1, 2, 3, 4, 5, 6]
    client.robot_state_pkg.tl_cur_pos = [10, 20, 30, 0, 0, 0]
    client.robot_state_pkg.motion_done = 0
    client.robot_state_pkg.main_code = 7
    client.robot_state_pkg.sub_code = 8
    assert robot.joints() == [1, 2, 3, 4, 5, 6]
    assert robot.tcp_pose() == [10, 20, 30, 0, 0, 0]
    assert robot.motion_done() is False
    assert robot.error_codes() == (7, 8)
    assert client.calls == []  # no RPC calls made


# -- error handling contract -------------------------------------------------


def test_nonzero_code_raises_fairino_error_with_call_name(robot, client):
    client.errors["ProgramLoad"] = 8
    with pytest.raises(fr5.FairinoError) as exc:
        robot.load_program("/fruser/missing.lua")
    assert exc.value.code == 8
    assert exc.value.call == "ProgramLoad"


def test_plain_int_return_raises_on_nonzero(robot, client):
    client.errors["StopMotion"] = -4
    with pytest.raises(fr5.FairinoError) as exc:
        robot.stop()
    assert exc.value.code == -4


def test_plain_int_return_ok_on_zero(robot):
    robot.stop()  # must not raise


# -- auto mode ----------------------------------------------------------


def test_ensure_auto_sets_mode_and_waits(robot, client, monkeypatch):
    sleeps = []
    monkeypatch.setattr(fr5.time, "sleep", lambda s: sleeps.append(s))
    client.robot_state_pkg.robot_mode = 0  # the fake's Mode() sets this anyway
    robot.ensure_auto()
    assert client.modes == [0]
    assert sleeps == [1.0]


def test_ensure_auto_raises_if_mode_stays_manual(robot, client, monkeypatch):
    monkeypatch.setattr(fr5.time, "sleep", lambda s: None)

    def fake_mode(state):
        client.modes.append(state)
        return 0  # Mode() call succeeds, but the cache never updates

    client.Mode = fake_mode
    client.robot_state_pkg.robot_mode = 1
    with pytest.raises(fr5.FairinoError):
        robot.ensure_auto()


# -- speed context manager -------------------------------------------------


def test_speed_sets_and_restores_default(robot, client):
    with robot.speed(50, safe_default_pct=10):
        assert client.speeds == [50]
    assert client.speeds == [50, 10]


def test_speed_restores_default_on_exception(robot, client):
    with pytest.raises(ValueError):
        with robot.speed(50, safe_default_pct=10):
            raise ValueError("boom")
    assert client.speeds == [50, 10]


def test_speed_tolerates_a_failing_restore(robot, client):
    with pytest.raises(ValueError):
        with robot.speed(50, safe_default_pct=10):
            client.errors["SetSpeed"] = -1
            raise ValueError("boom")
    # the restore attempt (which failed) must not mask the original exception


# -- motion ----------------------------------------------------------------


def test_move_j_calls_with_tool_and_user_zero(robot, client):
    robot.move_j([1, 2, 3, 4, 5, 6], tool_id=3, vel_pct=25)
    name, args, kwargs = client.calls[-1]
    assert name == "MoveJ"
    assert args == ([1, 2, 3, 4, 5, 6], 3, 0)
    assert kwargs == {"vel": 25}


# -- Lua programs ------------------------------------------------------------


def test_run_program_loads_then_runs(robot, client):
    robot.run_program("/fruser/motion_demo.lua")
    assert [c[0] for c in client.calls] == ["ProgramLoad", "ProgramRun"]
    assert client.programs_loaded == ["/fruser/motion_demo.lua"]
    assert robot.program_state() == 2


def test_run_program_load_failure_does_not_run(robot, client):
    client.errors["ProgramLoad"] = 3
    with pytest.raises(fr5.FairinoError):
        robot.run_program("/fruser/missing.lua")
    assert client.program_runs == 0


def test_stop_program_calls_program_stop(robot, client):
    robot.stop_program()
    assert client.program_stops == 1
    assert robot.program_state() == 1


# -- stream_joints (D9, simulator preview only) -----------------------------


def test_stream_joints_calls_servo_move_start_and_end(robot, client, monkeypatch):
    monkeypatch.setattr(fr5.time, "sleep", lambda s: None)
    rows = [[1, 2, 3, 4, 5, 6], [7, 8, 9, 10, 11, 12]]
    robot.stream_joints(rows, period_s=0.008)
    assert client.servo_move_started
    assert client.servo_move_ended


def test_stream_joints_sends_one_servo_j_per_row_with_zero_axis_pos(
    robot, client, monkeypatch
):
    monkeypatch.setattr(fr5.time, "sleep", lambda s: None)
    rows = [[1, 2, 3, 4, 5, 6], [7, 8, 9, 10, 11, 12]]
    robot.stream_joints(rows, period_s=0.008)
    assert client.servo_j_calls == [
        ([1, 2, 3, 4, 5, 6], [0.0, 0.0, 0.0, 0.0], 0.008),
        ([7, 8, 9, 10, 11, 12], [0.0, 0.0, 0.0, 0.0], 0.008),
    ]


def test_stream_joints_passes_the_rows_own_period_as_cmdt(robot, client, monkeypatch):
    monkeypatch.setattr(fr5.time, "sleep", lambda s: None)
    robot.stream_joints([[0.0] * 6], period_s=0.002)
    assert client.servo_j_calls[0][2] == 0.002


def test_stream_joints_sleeps_period_scaled_by_ovl(robot, client, monkeypatch):
    sleeps = []
    monkeypatch.setattr(fr5.time, "sleep", lambda s: sleeps.append(s))
    robot.stream_joints([[0.0] * 6, [0.0] * 6], period_s=0.008, ovl=50)
    # At ovl=50 the sleep between rows is twice the file's own period, minus
    # the (tiny, real) time the ServoJ call itself took.
    assert sleeps and sleeps[0] == pytest.approx(0.016, abs=2e-3)


def test_stream_joints_servo_move_end_runs_even_if_a_servo_j_fails(
    robot, client, monkeypatch
):
    monkeypatch.setattr(fr5.time, "sleep", lambda s: None)
    client.errors["ServoJ"] = -1
    with pytest.raises(fr5.FairinoError):
        robot.stream_joints([[0.0] * 6], period_s=0.008)
    assert client.servo_move_started
    assert client.servo_move_ended


def test_stream_joints_never_calls_servo_jt(robot, client, monkeypatch):
    """ServoJT (torque control) must never be used for preview; FakeRobotClient
    does not even implement it, so a mistaken call raises AttributeError."""
    monkeypatch.setattr(fr5.time, "sleep", lambda s: None)
    robot.stream_joints([[0.0] * 6], period_s=0.008)
    assert not hasattr(client, "ServoJT") or "ServoJT" not in [
        c[0] for c in client.calls
    ]


# -- TPD loading --------------------------------------------------------


def test_robot_has_no_movetpd_calls(robot):
    """Every motion is a Lua program; MoveTPD refuses the wrist singularity."""
    for name in ("move_tpd", "load_tpd", "tpd_start_pose"):
        assert not hasattr(robot, name)


# -- file_md5 -------------------------------------------------------------


def test_file_md5_returns_none_for_code_4(robot):
    assert robot.file_md5("/fruser/traj/missing.txt") is None


def test_file_md5_returns_hash_for_existing_file(robot, client):
    data = b"hello world"
    client.files["/fruser/traj/motion_demo.txt"] = data
    expected = hashlib.md5(data).hexdigest()
    assert robot.file_md5("/fruser/traj/motion_demo.txt") == expected


def test_file_md5_raises_for_other_error_codes(robot, client):
    client.errors["ComputeFileMD5"] = -4
    with pytest.raises(fr5.FairinoError) as exc:
        robot.file_md5("/fruser/traj/x.txt")
    assert exc.value.code == -4


def test_delete_traj(robot, client):
    robot.delete_traj("motion_demo.txt")
    assert client.deleted_files == ["motion_demo.txt"]


# -- upload_file, against a local listener standing in for port 20010 -------


class _FakeDataPort:
    """A real TCP listener that mimics the controller's upload socket.

    `store_as`, if given, is a (client, remote_path) pair: the uploaded bytes
    (stripped of the "/f/b<size><md5>" header and the trailing "/b/f") are
    written into `client.files[remote_path]`, so `ComputeFileMD5` sees them,
    exactly as the real controller stores what it received.
    """

    def __init__(
        self,
        open_after_s: float = 0.0,
        reply: bytes = b"SUCCESS",
        store_as: tuple[FakeRobotClient, str] | None = None,
    ):
        self.open_after_s = open_after_s
        self.reply = reply
        self.store_as = store_as
        self.received: bytes | None = None
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.bind(("127.0.0.1", 0))
        self.port = self._server.getsockname()[1]
        self._server.listen(0)
        self._server.settimeout(10)
        self._thread: threading.Thread | None = None

    def start(self):
        def run():
            time.sleep(self.open_after_s)
            conn, _ = self._server.accept()
            with conn:
                conn.settimeout(10)
                chunks = []
                while True:
                    chunk = conn.recv(65536)
                    if not chunk:
                        break
                    chunks.append(chunk)
                    if chunks and b"".join(chunks).endswith(b"/b/f"):
                        break
                self.received = b"".join(chunks)
                if self.store_as is not None:
                    client, remote_path = self.store_as
                    header_len = len("/f/b") + 10 + 32
                    client.files[remote_path] = self.received[header_len:-4]
                conn.sendall(self.reply)

        self._thread = threading.Thread(target=run, daemon=True)
        self._thread.start()
        return self

    def close(self):
        self._server.close()


@pytest.fixture
def data_port():
    ports = []

    def make(**kwargs):
        p = _FakeDataPort(**kwargs).start()
        ports.append(p)
        return p

    yield make
    for p in ports:
        p.close()


def _robot_for_upload(client, port, **kwargs):
    return fr5.FR5(client, "127.0.0.1", data_port=port, **kwargs)


def test_upload_file_succeeds_when_listener_starts_late(client, data_port, tmp_path):
    local = tmp_path / "motion_demo.txt"
    local.write_text("1,1,8,0,0,0\n")
    remote = "/fruser/traj/motion_demo.txt"
    server = data_port(open_after_s=0.1, store_as=(client, remote))
    robot = _robot_for_upload(client, server.port, poll_step_s=0.02)

    remote = robot.upload_file(str(local), file_type=20)

    assert remote == "/fruser/traj/motion_demo.txt"
    assert client.file_upload_calls == [(20, "motion_demo.txt")]
    data = local.read_bytes()
    expected_head = f"/f/b{len(data) + 50:10d}{hashlib.md5(data).hexdigest()}".encode()
    assert server.received.startswith(expected_head)
    assert server.received.endswith(data + b"/b/f")


def test_upload_lua_registers_the_program_before_verifying(client, data_port, tmp_path):
    local = tmp_path / "motion_demo.lua"
    local.write_text("P = {}\n")
    server = data_port(store_as=(client, "/fruser/motion_demo.lua"))
    robot = _robot_for_upload(client, server.port, poll_step_s=0.02)

    remote = robot.upload_file(str(local), file_type=0)

    assert remote == "/fruser/motion_demo.lua"
    assert client.file_upload_calls == [(0, "motion_demo.lua")]
    assert client.lua_registered == ["motion_demo.lua"]


def test_upload_lua_raises_register_when_the_controller_refuses(
    client, data_port, tmp_path
):
    local = tmp_path / "motion_demo.lua"
    local.write_text("P = {}\n")
    server = data_port(store_as=(client, "/fruser/motion_demo.lua"))
    client.lua_update_rtn = 7
    robot = _robot_for_upload(client, server.port, poll_step_s=0.02)
    with pytest.raises(fr5.FileTransferError) as exc:
        robot.upload_file(str(local), file_type=0)
    assert exc.value.stage == "register"


def test_upload_traj_does_not_register_a_program(client, data_port, tmp_path):
    local = tmp_path / "motion_demo.txt"
    local.write_text("1,1,8,0,0,0\n")
    server = data_port(store_as=(client, "/fruser/traj/motion_demo.txt"))
    robot = _robot_for_upload(client, server.port, poll_step_s=0.02)
    robot.upload_file(str(local), file_type=20)
    assert client.lua_registered == []


def test_upload_file_raises_connect_when_listener_never_starts(client, tmp_path):
    local = tmp_path / "motion_demo.txt"
    local.write_text("x")
    # An unused port on localhost that nothing listens on.
    unused = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    unused.bind(("127.0.0.1", 0))
    port = unused.getsockname()[1]
    unused.close()
    robot = _robot_for_upload(client, port, poll_s=0.2, poll_step_s=0.05)

    with pytest.raises(fr5.FileTransferError) as exc:
        robot.upload_file(str(local), file_type=20)
    assert exc.value.stage == "connect"


def test_upload_file_raises_request_when_file_upload_rejects(client, tmp_path):
    local = tmp_path / "motion_demo.txt"
    local.write_text("x")
    client.file_upload_rtn = -7
    robot = _robot_for_upload(client, 0)

    with pytest.raises(fr5.FileTransferError) as exc:
        robot.upload_file(str(local), file_type=20)
    assert exc.value.stage == "request"
    assert exc.value.detail == -7


def test_upload_file_raises_transfer_when_reply_is_not_success(
    client, data_port, tmp_path
):
    local = tmp_path / "motion_demo.txt"
    local.write_text("x")
    server = data_port(reply=b"FAIL")
    robot = _robot_for_upload(client, server.port)

    with pytest.raises(fr5.FileTransferError) as exc:
        robot.upload_file(str(local), file_type=20)
    assert exc.value.stage == "transfer"


def test_upload_file_raises_verify_when_md5_differs(client, data_port, tmp_path):
    local = tmp_path / "motion_demo.txt"
    local.write_text("x")
    server = data_port()  # SUCCESS reply, but the fake client "stores" nothing
    robot = _robot_for_upload(client, server.port)

    with pytest.raises(fr5.FileTransferError) as exc:
        robot.upload_file(str(local), file_type=20)
    assert exc.value.stage == "verify"


def test_upload_file_rejects_bad_file_type_before_any_call(client, tmp_path):
    local = tmp_path / "x.txt"
    local.write_text("x")
    robot = _robot_for_upload(client, 0)

    with pytest.raises(ValueError):
        robot.upload_file(str(local), file_type=5)
    assert client.file_upload_calls == []


def test_upload_file_never_calls_banned_sdk_methods(client, data_port, tmp_path):
    """FR5 must not fall back to TrajectoryJUpLoad/LuaUpload/PointTableUpLoad."""
    local = tmp_path / "motion_demo.txt"
    local.write_text("1,1,8,0,0,0\n")
    remote = "/fruser/traj/motion_demo.txt"
    server = data_port(store_as=(client, remote))
    robot = _robot_for_upload(client, server.port)

    robot.upload_file(str(local), file_type=20)  # must not raise AssertionError


def test_upload_lua_accepts_a_rewritten_program_but_not_a_missing_one(
    client, data_port, tmp_path
):
    local = tmp_path / "motion_demo.lua"
    local.write_text("P = {}\n")
    server = data_port()  # stores nothing: the controller has no such file
    robot = _robot_for_upload(client, server.port, poll_step_s=0.02)
    with pytest.raises(fr5.FileTransferError) as exc:
        robot.upload_file(str(local), file_type=0)
    assert exc.value.stage == "verify"

    client.files["/fruser/motion_demo.lua"] = b"-- rewritten by the controller\n"
    server = data_port()
    robot = _robot_for_upload(client, server.port, poll_step_s=0.02)
    assert robot.upload_file(str(local), file_type=0) == "/fruser/motion_demo.lua"
