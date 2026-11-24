"""Robot motion controls in the livestream window (Phase 6).

The real LivestreamApp is built on a real Tk root with a fake GoPro controller
and a FakeMotionRunner, so nothing touches the camera or the robot. Needs a
display; skipped when Tk cannot open one.
"""

from __future__ import annotations

import concurrent.futures
import dataclasses
import gc
import json
import tkinter as tk
from pathlib import Path
from types import SimpleNamespace

import pytest

from sw import livestream as l
from sw.camera import hero10_table as table
from sw.robot.motion_runner import MotionError, MotionStopped, RunnerState
from sw.robot.schema import RobotSettings, load_motion_config
from sw.robot.ui_state import UiState

MOTION_JSON = (
    Path(__file__).resolve().parents[1]
    / "robot"
    / "motions"
    / "up_down_horizontal"
    / "motion.json"
)


class FakeController:
    """GoPro controller whose operations stay pending until a test resolves
    them by coroutine name ("start_recording", "stop_recording", ...)."""

    catalog = table.build_catalog()
    port = 8554

    def __init__(self, output_dir: Path, log: list) -> None:
        self.output_dir = output_dir
        self.log = log
        self.pending: dict[str, concurrent.futures.Future] = {}

    async def connect(self, mode):
        return "GP-TEST"

    async def start_recording(self, *args):
        return None

    async def stop_recording(self):
        return []

    async def shutdown(self):
        return None

    def submit(self, coro):
        name = coro.cr_code.co_name
        coro.close()
        self.log.append(("gopro", name))
        future: concurrent.futures.Future = concurrent.futures.Future()
        if name == "shutdown":
            future.set_result(None)
        else:
            self.pending[name] = future
        return future

    def resolve(self, name: str, result=None, error: Exception | None = None):
        future = self.pending.pop(name)
        if error is not None:
            future.set_exception(error)
        else:
            future.set_result(result)


class FakeMotionRunner:
    """Records calls into the shared log. Jobs resolve at once unless their
    name is in `hold`, in which case the test resolves them."""

    def __init__(self, log: list, reasons=None) -> None:
        self.log = log
        self.reasons = reasons
        self.state = RunnerState.IDLE
        self.at = (False, 7.5)
        self.hold: set[str] = set()
        self.fail: dict[str, Exception] = {}
        self.pending: dict[str, concurrent.futures.Future] = {}

    def _job(self, name, *args, result=None):
        self.log.append(("robot", name, *args))
        future: concurrent.futures.Future = concurrent.futures.Future()
        if name in self.hold:
            self.pending[name] = future
        elif name in self.fail:
            future.set_exception(self.fail[name])
        else:
            future.set_result(result)
        return future

    def connect_and_check(self):
        return self._job("connect_and_check", result=self.reasons)

    def load(self, name):
        return self._job("load", name)

    def move_to_start(self, name):
        return self._job("move_to_start", name)

    def run(self, name):
        return self._job("run", name)

    def at_start(self, name):
        return self.at

    def stop(self):
        self.log.append(("robot", "stop"))
        return True

    def clear_fault(self):
        self.log.append(("robot", "clear_fault"))
        self.state = RunnerState.IDLE

    def close(self):
        self.log.append(("robot", "close"))

    def resolve(self, name, result=None, error=None):
        future = self.pending.pop(name)
        if error is not None:
            future.set_exception(error)
        else:
            future.set_result(result)


@pytest.fixture
def root():
    try:
        tk_root = tk.Tk()
    except tk.TclError:
        pytest.skip("no display available for Tk")
    yield tk_root
    gc.collect()
    try:
        tk_root.destroy()
    except tk.TclError:
        pass  # _on_close already destroyed it


@pytest.fixture
def make_app(root, tmp_path, monkeypatch):
    monkeypatch.setattr(
        l,
        "PreviewReader",
        lambda *a, **k: SimpleNamespace(
            start=lambda: None, stop=lambda: None, latest_frame=lambda: None
        ),
    )
    apps = []
    base = load_motion_config(MOTION_JSON)
    motions = {
        "up_down_horizontal": base,
        "other": dataclasses.replace(base, name="other"),
    }

    def build(*, runner="fake", motion="", reasons=None, broken=None, **settings):
        log: list = []
        controller = FakeController(tmp_path, log)
        if runner == "fake":
            runner = FakeMotionRunner(log, reasons or {})
        defaults = {"pre_roll_s": 0.0, "post_roll_s": 0.0}
        defaults.update(settings)
        ui = l.LivestreamApp(
            root,
            controller,
            upload=False,
            folder="default",
            image="",
            fadein_image="",
            fadeout_image="",
            config_file=tmp_path / "config.json",
            motion_runner=runner,
            motions=motions if runner is not None else {},
            broken_motions=broken,
            motion=motion,
            robot_settings=RobotSettings(**defaults),
        )
        ui._stop_robot = lambda: runner.stop()  # synchronous, for ordering
        apps.append(ui)
        controller.resolve("connect", "GP-TEST")
        pump(ui)
        return ui, controller, runner, log

    yield build
    for ui in apps:
        ui.postprocessor.stop()


def pump(ui, times: int = 3) -> None:
    """Run due Tk timers and drain the app's event queue."""
    for _ in range(times):
        ui.root.update()
        while not ui._events.empty():
            ui._events.get_nowait()()


def robot_calls(log):
    return [entry[1:] for entry in log if entry[0] == "robot"]


def select(ui, name, monkeypatch, answer=True):
    monkeypatch.setattr(l.messagebox, "askyesno", lambda *a, **k: answer)
    ui._on_motion_click(name)
    pump(ui)


# -- --no-robot ---------------------------------------------------------------


def test_no_robot_builds_no_motion_controls(make_app):
    ui, *_ = make_app(runner=None)
    assert ui.motion_buttons == {}
    assert ui.robot_button is None
    assert ui._ui_state is UiState.UNAVAILABLE


def test_no_robot_record_uses_the_plain_flow(make_app):
    ui, controller, _, _ = make_app(runner=None)
    ui._on_record()
    assert "start_recording" in controller.pending


# -- startup ------------------------------------------------------------------


def test_startup_check_disables_failed_motions_but_keeps_them_visible(make_app):
    ui, _, runner, log = make_app(reasons={"other": "md5 mismatch"})
    assert ("connect_and_check",) in robot_calls(log)
    assert ui._ui_state is UiState.IDLE
    assert set(ui.motion_buttons) == {None, "up_down_horizontal", "other"}
    assert str(ui.motion_buttons["other"].cget("state")) == "disabled"
    assert str(ui.motion_buttons["up_down_horizontal"].cget("state")) == "normal"
    ui._on_motion_hover("other")
    assert ui.robot_status.get() == "'other' disabled: md5 mismatch"


def test_broken_motion_json_shows_as_disabled(make_app):
    ui, *_ = make_app(broken={"bad": "invalid motion.json: missing key(s): md5"})
    assert str(ui.motion_buttons["bad"].cget("state")) == "disabled"


def test_saved_motion_is_loaded_at_startup(make_app):
    ui, _, _, log = make_app(motion="up_down_horizontal")
    assert ("load", "up_down_horizontal") in robot_calls(log)
    assert ui._motion == "up_down_horizontal"
    assert ui._ui_state is UiState.IDLE


def test_invalid_saved_motion_resets_in_memory_only(make_app, tmp_path):
    (tmp_path / "config.json").write_text(json.dumps({"motion": "other"}))
    ui, _, _, log = make_app(motion="other", reasons={"other": "md5 mismatch"})
    assert ui._motion is None
    assert ("load", "other") not in robot_calls(log)
    assert "md5 mismatch" in ui.robot_status.get()
    assert json.loads((tmp_path / "config.json").read_text())["motion"] == "other"


def test_unreachable_robot_disables_every_motion(make_app):
    runner = FakeMotionRunner([])
    runner.fail["connect_and_check"] = OSError("no route to host")
    ui, controller, _, _ = make_app(runner=runner, motion="up_down_horizontal")
    assert ui._ui_state is UiState.UNAVAILABLE
    assert ui._motion is None
    for name in ("up_down_horizontal", "other"):
        assert str(ui.motion_buttons[name].cget("state")) == "disabled"
    ui._on_record()  # plain recording still works
    assert "start_recording" in controller.pending


def test_startup_timeout_makes_a_late_answer_stale(make_app):
    runner = FakeMotionRunner([], {})
    runner.hold.add("connect_and_check")
    ui, *_ = make_app(runner=runner)
    assert ui._ui_state is UiState.CHECKING
    ui._robot_check_timeout(ui._robot_op)
    assert ui._ui_state is UiState.UNAVAILABLE
    runner.resolve("connect_and_check", {})
    pump(ui)
    assert ui._ui_state is UiState.UNAVAILABLE  # stale result ignored


# -- selecting a motion ---------------------------------------------------------


def test_confirming_a_motion_loads_it_and_saves_the_config(
    make_app, monkeypatch, tmp_path
):
    ui, _, _, log = make_app()
    select(ui, "up_down_horizontal", monkeypatch)
    calls = robot_calls(log)
    assert ("load", "up_down_horizontal") in calls
    assert not any(c[0] in ("move_to_start", "run") for c in calls)
    saved = json.loads((tmp_path / "config.json").read_text())
    assert saved["motion"] == "up_down_horizontal"
    assert "loaded" in ui.robot_status.get()
    assert str(ui.motion_buttons["up_down_horizontal"].cget("relief")) == "sunken"


def test_declining_changes_nothing(make_app, monkeypatch, tmp_path):
    ui, _, _, log = make_app()
    select(ui, "up_down_horizontal", monkeypatch, answer=False)
    assert ui._motion is None
    assert ("load", "up_down_horizontal") not in robot_calls(log)
    assert not (tmp_path / "config.json").exists()


def test_clicking_the_active_motion_opens_no_dialog(make_app, monkeypatch):
    ui, *_ = make_app(motion="up_down_horizontal")

    def boom(*a, **k):
        raise AssertionError("dialog opened")

    monkeypatch.setattr(l.messagebox, "askyesno", boom)
    ui._on_motion_click("up_down_horizontal")


def test_state_change_while_the_dialog_is_open_discards_the_choice(
    make_app, monkeypatch
):
    ui, _, _, log = make_app()

    def answer(*a, **k):
        ui._ui_state = UiState.MOVING_TO_START  # something happened meanwhile
        return True

    monkeypatch.setattr(l.messagebox, "askyesno", answer)
    ui._on_motion_click("up_down_horizontal")
    assert ui._motion is None
    assert ("load", "up_down_horizontal") not in robot_calls(log)


def test_load_failure_marks_the_motion_invalid_for_the_session(make_app, monkeypatch):
    ui, _, runner, _ = make_app()
    runner.fail["load"] = MotionError("start pose differs")
    runner.state = RunnerState.FAULT
    select(ui, "up_down_horizontal", monkeypatch)
    assert ui._motion is None
    assert "invalid at load" in ui._motion_reasons["up_down_horizontal"]
    assert ui._ui_state is UiState.FAULT
    assert ui.robot_button.cget("text") == "Clear fault"
    ui._on_robot_button()
    assert ("clear_fault",) in robot_calls(ui.motion_runner.log)
    assert ui._ui_state is UiState.IDLE


# -- move to start ------------------------------------------------------------


def test_move_to_start_calls_the_runner_and_is_orange_when_not_at_start(
    make_app, monkeypatch
):
    ui, _, runner, log = make_app(motion="up_down_horizontal")
    assert ui.robot_button.cget("text") == "Move to Start"
    assert ui.robot_button.cget("bg") == "#ef6c00"
    runner.hold.add("move_to_start")
    ui._on_robot_button()
    assert ("move_to_start", "up_down_horizontal") in robot_calls(log)
    assert ui._ui_state is UiState.MOVING_TO_START
    assert ui.robot_button.cget("text") == "Stop robot"
    runner.at = (True, 0.1)
    runner.resolve("move_to_start")
    pump(ui)
    assert ui._ui_state is UiState.IDLE
    assert ui.robot_button.cget("bg") == "#e0e0e0"


def test_stop_robot_during_move_to_start(make_app):
    ui, _, runner, log = make_app(motion="up_down_horizontal")
    runner.hold.add("move_to_start")
    ui._on_robot_button()
    ui._on_robot_button()  # now "Stop robot"
    assert ("stop",) in robot_calls(log)
    runner.resolve("move_to_start", error=MotionStopped("stopped"))
    pump(ui)
    assert ui._ui_state is UiState.IDLE


def test_move_to_start_is_disabled_without_a_motion(make_app):
    ui, *_ = make_app()
    assert str(ui.robot_button.cget("state")) == "disabled"


# -- record with a motion ---------------------------------------------------------


def test_record_not_at_start_starts_nothing(make_app):
    ui, controller, _, _ = make_app(motion="up_down_horizontal")
    ui._on_record()
    assert "start_recording" not in controller.pending
    assert "Not at start" in ui.robot_status.get()


def start_motion_recording(make_app, **kw):
    ui, controller, runner, log = make_app(motion="up_down_horizontal", **kw)
    runner.at = (True, 0.0)
    ui._on_record()
    assert "start_recording" in controller.pending
    return ui, controller, runner, log


def test_run_starts_only_after_recording_started_and_pre_roll(make_app):
    ui, controller, runner, log = start_motion_recording(make_app)
    pump(ui)
    assert ("run", "up_down_horizontal") not in robot_calls(log)
    runner.hold.add("run")
    controller.resolve("start_recording")
    pump(ui)
    assert ("run", "up_down_horizontal") in robot_calls(log)
    assert ui._ui_state is UiState.RUNNING


def test_motion_done_stops_the_recording(make_app):
    ui, controller, runner, log = start_motion_recording(make_app)
    controller.resolve("start_recording")
    pump(ui)
    assert "stop_recording" in controller.pending
    controller.resolve("stop_recording", [])
    pump(ui)
    assert ui._ui_state is UiState.IDLE


def test_motion_done_without_auto_stop_keeps_recording(make_app):
    ui, controller, *_ = start_motion_recording(make_app, auto_stop_after_motion=False)
    controller.resolve("start_recording")
    pump(ui)
    assert ui._ui_state is UiState.POST_ROLL
    assert "stop_recording" not in controller.pending


def test_failed_gopro_start_never_runs_the_motion(make_app):
    ui, controller, _, log = start_motion_recording(make_app)
    controller.resolve("start_recording", error=RuntimeError("camera busy"))
    pump(ui)
    assert ("run", "up_down_horizontal") not in robot_calls(log)
    assert ui._ui_state is UiState.IDLE


def test_close_during_pre_roll_never_runs_the_motion(make_app):
    ui, controller, _, log = start_motion_recording(make_app, pre_roll_s=10.0)
    controller.resolve("start_recording")
    pump(ui)
    assert ui._ui_state is UiState.PRE_ROLL
    ui._on_close()
    calls = robot_calls(log)
    assert ("run", "up_down_horizontal") not in calls
    assert ("stop",) in calls and ("close",) in calls
    assert ("gopro", "shutdown") in log  # the shutdown stops the recording


def test_stop_during_a_motion_stops_the_robot_before_the_recording(make_app):
    ui, controller, runner, log = start_motion_recording(make_app)
    runner.hold.add("run")
    controller.resolve("start_recording")
    pump(ui)
    ui._on_record()  # the record button is "Stop recording" now
    assert log.index(("robot", "stop")) < log.index(("gopro", "stop_recording"))
    assert ui._ui_state is UiState.STOPPING
    # A late motion result is ignored, and the recording stops only once.
    runner.resolve("run", error=MotionStopped("stopped"))
    pump(ui)
    ui._on_record()
    assert log.count(("gopro", "stop_recording")) == 1
    controller.resolve("stop_recording", [])
    pump(ui)
    assert ui._ui_state is UiState.IDLE


def test_motion_failure_stops_robot_and_recording_then_faults(make_app):
    ui, controller, runner, log = start_motion_recording(make_app)
    runner.hold.add("run")
    controller.resolve("start_recording")
    pump(ui)
    runner.state = RunnerState.FAULT
    runner.resolve("run", error=MotionError("motion did not start"))
    pump(ui)
    assert ("stop",) in robot_calls(log)
    assert "stop_recording" in controller.pending
    controller.resolve("stop_recording", [])
    pump(ui)
    assert ui._ui_state is UiState.FAULT


def test_stale_robot_result_is_ignored(make_app):
    ui, _, runner, _ = make_app(motion="up_down_horizontal")
    runner.hold.add("move_to_start")
    ui._on_robot_button()
    ui._robot_op += 1  # a newer operation started
    runner.resolve("move_to_start")
    pump(ui)
    assert ui._ui_state is UiState.MOVING_TO_START


def test_close_closes_the_runner(make_app):
    ui, _, _, log = make_app()
    ui._on_close()
    assert ("stop",) in robot_calls(log)
    assert ("close",) in robot_calls(log)


# -- config helpers -----------------------------------------------------------


def test_motion_config_round_trip_keeps_other_keys(tmp_path):
    cfg = tmp_path / "c.json"
    cfg.write_text(json.dumps({"folder": "x", "robot": {"pre_roll_s": 1.0}}))
    l.save_motion(cfg, "sweep")
    assert l.load_robot_config(cfg) == ("sweep", {"pre_roll_s": 1.0})
    assert json.loads(cfg.read_text())["folder"] == "x"
    assert l.load_robot_config(tmp_path / "missing.json") == ("", None)


def test_load_motions_reports_broken_files(tmp_path):
    good = tmp_path / "up_down_horizontal"
    good.mkdir()
    (good / "motion.json").write_text(MOTION_JSON.read_text())
    bad = tmp_path / "bad"
    bad.mkdir()
    (bad / "motion.json").write_text("{}")
    (tmp_path / "draft").mkdir()
    configs, broken = l.load_motions(tmp_path)
    assert list(configs) == ["up_down_horizontal"]
    assert list(broken) == ["bad"]


def test_portrait_motions_are_marked_on_their_button(make_app, monkeypatch):
    ui, *_ = make_app()
    assert ui.motion_buttons["up_down_horizontal"].cget("text") == "up_down_horizontal"
    assert ui.motion_buttons[None].cget("text") == "No motion"
    portrait = dataclasses.replace(ui.motions["other"], format="portrait")
    ui.motions["other"] = portrait
    assert ui._motion_label("other") == "other (portrait)"
