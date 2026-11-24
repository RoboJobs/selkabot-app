"""Tests for MotionRunner with FakeRobotClient. No hardware."""

from __future__ import annotations

import hashlib
import threading

import pytest

from sw.robot import motion_runner as mr
from sw.robot.fr5 import FR5, FairinoError
from sw.robot.schema import RobotSettings, motion_config_from_dict
from sw.tests.fakes.fake_robot import FakeRobotClient

START_JOINTS = [10.0, -90.0, 90.0, -90.0, -90.0, 0.0]
START_POSE = [600.0, 0.0, 400.0, -90.0, 0.0, -90.0]


def _config(name="demo", data=b"tpd file", **overrides):
    raw = {
        "name": name,
        "backend": "tpd",
        "controller_file": f"/fruser/traj/motion_{name}.txt",
        "tpd_name": f"traj/motion_{name}",
        "md5": hashlib.md5(data).hexdigest(),
        "num_points": 6,
        "sample_period_ms": 8,
        "duration_s": 0.04,
        "design_duration_s": 0.04,
        "start_joints": START_JOINTS,
        "start_pose": START_POSE,
        "tool_id": 1,
        "approved_ovl": 100,
        "design_sha256": "a" * 64,
        "published_at": "2026-11-25T10:00:00",
    }
    raw.update(overrides)
    return motion_config_from_dict(raw) if "skip_check" not in raw else raw


@pytest.fixture
def client():
    c = FakeRobotClient()
    c.files["/fruser/traj/motion_demo.txt"] = b"tpd file"
    c.tpd_start_poses["traj/motion_demo"] = list(START_POSE)
    return c


def _runner(client, motions=None, **kwargs):
    fr5 = FR5(client, "192.168.58.2")
    kwargs.setdefault("poll_s", 0.001)
    kwargs.setdefault("start_timeout_s", 0.05)
    kwargs.setdefault("watchdog_margin_s", 0.2)
    return mr.MotionRunner(
        lambda: fr5,
        RobotSettings(),
        motions if motions is not None else {"demo": _config()},
        **kwargs,
    )


@pytest.fixture
def runner(client):
    r = _runner(client)
    assert r.connect_and_check().result(timeout=5) == {"demo": None}
    yield r
    r.close()


def _calls(client, name):
    return [c for c in client.calls if c[0] == name]


# -- startup check ---------------------------------------------------------------


def test_each_startup_problem_disables_only_that_motion(client):
    client.files["/fruser/traj/motion_wrong.txt"] = b"something else"
    motions = {
        "demo": _config(),
        "missing": _config("missing"),
        "wrong": _config("wrong"),
    }
    r = _runner(client, motions)
    try:
        reasons = r.connect_and_check().result(timeout=5)
    finally:
        r.close()
    assert reasons == {
        "demo": None,
        "missing": "file missing on controller",
        "wrong": "md5 mismatch",
    }
    assert set(r.disabled) == {"missing", "wrong"}


def test_hash_query_failure_is_reported_with_its_code(client):
    client.errors["ComputeFileMD5"] = -4
    r = _runner(client)
    try:
        reasons = r.connect_and_check().result(timeout=5)
    finally:
        r.close()
    assert reasons == {"demo": "hash query failed (code -4)"}


def test_startup_sets_the_safe_default_speed(runner, client):
    assert client.speeds[-1] == RobotSettings().safe_default_speed_pct


def test_disabled_or_unknown_motions_raise_value_error(client):
    r = _runner(client, {"missing": _config("missing")})
    try:
        r.connect_and_check().result(timeout=5)
        with pytest.raises(ValueError, match="disabled"):
            r.load("missing")
        with pytest.raises(ValueError, match="unknown"):
            r.run("nope")
    finally:
        r.close()


# -- load, move to start, at_start ------------------------------------------------


def test_load_checks_the_start_pose_and_never_moves(runner, client):
    runner.load("demo").result(timeout=5)
    assert client.tpd_loaded == "traj/motion_demo"
    assert not _calls(client, "MoveJ") and not _calls(client, "MoveTPD")


def test_load_with_a_wrong_start_pose_fails_and_disables_the_motion(runner, client):
    client.tpd_start_poses["traj/motion_demo"] = [0.0] * 6
    with pytest.raises(mr.MotionError, match="start pose"):
        runner.load("demo").result(timeout=5)
    assert "demo" in runner.disabled


def test_move_to_start_at_10_percent_then_restores_the_speed(
    runner, client, monkeypatch
):
    monkeypatch.setattr("sw.robot.fr5.time.sleep", lambda s: None)  # ensure_auto
    client.robot_state_pkg.robot_mode = 0
    runner.move_to_start("demo").result(timeout=5)
    (move,) = _calls(client, "MoveJ")
    assert move[1] == (START_JOINTS, 1, 0)
    # global speed carries the 10%, MoveJ itself runs at 100% (speeds multiply)
    assert move[2] == {"vel": 100, "blendT": 0.0}  # non-blocking
    assert client.speeds[-2:] == [10, 10]  # set for the move, then restored
    assert runner.at_start("demo") == (True, 0.0)


def test_move_to_start_skips_when_already_there(runner, client):
    client.robot_state_pkg.jt_cur_pos = list(START_JOINTS)
    runner.move_to_start("demo").result(timeout=5)
    assert not _calls(client, "MoveJ")


def test_at_start_uses_the_tolerance_and_reports_the_error(runner, client):
    tol = RobotSettings().start_tolerance_deg
    client.robot_state_pkg.jt_cur_pos = [START_JOINTS[0] + tol * 0.9, *START_JOINTS[1:]]
    ok, err = runner.at_start("demo")
    assert ok and err == pytest.approx(tol * 0.9)
    client.robot_state_pkg.jt_cur_pos = [START_JOINTS[0] + tol * 1.1, *START_JOINTS[1:]]
    assert runner.at_start("demo")[0] is False


# -- run ---------------------------------------------------------------------------


@pytest.fixture
def auto(client, monkeypatch):
    monkeypatch.setattr("sw.robot.fr5.time.sleep", lambda s: None)
    client.robot_state_pkg.robot_mode = 0


def test_run_resolves_when_motion_done_goes_0_then_1(runner, client, auto):
    runner.run("demo").result(timeout=5)
    assert client.move_tpd_calls == [("traj/motion_demo", 0, 100)]
    assert client.speeds[-2:] == [100, 10]  # approved ovl as global speed, restored
    assert runner.state is mr.RunnerState.IDLE


def test_run_fails_when_the_motion_never_starts(runner, client, auto):
    client.move_tpd_script = [1]
    with pytest.raises(mr.MotionError, match="did not start"):
        runner.run("demo").result(timeout=5)
    assert runner.state is mr.RunnerState.FAULT


def test_run_raises_and_stops_on_a_robot_error(runner, client, auto):
    client.move_tpd_script = [0]
    client.robot_state_pkg.main_code = 5
    with pytest.raises(FairinoError):
        runner.run("demo").result(timeout=5)
    assert _calls(client, "StopMotion")


def test_watchdog_stops_a_motion_that_never_finishes(runner, client, auto):
    client.move_tpd_script = [0]  # starts, never completes
    with pytest.raises(mr.MotionTimeout):
        runner.run("demo").result(timeout=10)
    assert _calls(client, "StopMotion")


def test_a_fault_blocks_motions_until_cleared(runner, client, auto):
    client.move_tpd_script = [1]
    with pytest.raises(mr.MotionError):
        runner.run("demo").result(timeout=5)
    with pytest.raises(mr.MotionError, match="fault"):
        runner.run("demo")
    runner.clear_fault()
    client.move_tpd_script = [0, 0, 1]
    runner.run("demo").result(timeout=5)


def test_stop_during_a_run_ends_it_without_a_fault(runner, client, auto):
    client.move_tpd_script = [0]
    future = runner.run("demo")
    threading.Timer(0.05, runner.stop).start()
    with pytest.raises(mr.MotionStopped):
        future.result(timeout=5)
    assert runner.state is mr.RunnerState.IDLE


# -- stop and close ---------------------------------------------------------------


def test_stop_never_raises_and_is_idempotent(runner, client, monkeypatch):
    monkeypatch.setattr(mr, "STOP_RETRY_S", 0.0)
    assert runner.stop() is True and runner.stop() is True
    client.errors["StopMotion"] = -1
    assert runner.stop() is False  # all attempts failed, still no exception
    assert len(_calls(client, "StopMotion")) == 2 + mr.STOP_ATTEMPTS


def test_stop_uses_a_fresh_connection_when_an_rpc_blocks(client, auto, monkeypatch):
    monkeypatch.setattr(mr, "STOP_LOCK_TIMEOUT_S", 0.05)
    gate = threading.Event()
    client.move_tpd_block = gate
    r = _runner(client, watchdog_margin_s=0.3)
    try:
        r.connect_and_check().result(timeout=5)
        future = r.run("demo")
        threading.Timer(0.1, r.stop).start()
        threading.Timer(0.5, gate.set).start()
        with pytest.raises(mr.MotionError):
            future.result(timeout=10)
        assert _calls(client, "StopMotion")  # sent while MoveTPD was blocked
    finally:
        gate.set()
        r.close()


def test_close_during_a_run_stops_and_rejects_new_jobs(client, auto):
    client.move_tpd_script = [0]
    r = _runner(client, watchdog_margin_s=5.0)
    r.connect_and_check().result(timeout=5)
    future = r.run("demo")
    closer = threading.Timer(0.05, r.close)
    closer.start()
    with pytest.raises((mr.MotionError, mr.RunnerClosed)):
        future.result(timeout=5)
    closer.join(timeout=5)
    assert _calls(client, "StopMotion")
    with pytest.raises(mr.RunnerClosed):
        r.run("demo")
    r.close()  # idempotent
    assert r.state is mr.RunnerState.CLOSED and client.closed
