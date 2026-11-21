"""Tests for the cascading mode rows in the livestream window.

They build only the mode rows on a real Tk root and stub out the camera, so
they need a display. They are skipped when Tk cannot open one.
"""

import gc
import tkinter as tk
from tkinter import ttk
from types import SimpleNamespace

import pytest

from sw import livestream as l
from sw.camera import hero10_table as table
from sw.camera.mode_picker import ModePicker


@pytest.fixture
def root():
    try:
        tk_root = tk.Tk()
    except tk.TclError:
        pytest.skip("no display available for Tk")
    yield tk_root
    gc.collect()  # drop Tk variables while the interpreter is still attached to Tk
    tk_root.destroy()


@pytest.fixture
def app(root):
    """A LivestreamApp with only the mode rows built and no camera behind it."""
    ui = l.LivestreamApp.__new__(l.LivestreamApp)
    ui._busy = False
    ui._recording = False
    ui.picker = ModePicker(table.build_catalog())
    ui.status = tk.StringVar(master=root)
    ui.controller = SimpleNamespace(set_mode=lambda mode: None)
    frame = ttk.Frame(root)
    frame.pack()
    ui._build_mode_rows(frame)
    ui._run_calls = []
    ui._run = lambda coro, status, done, failed: ui._run_calls.append((status, done, failed))
    return ui


def states(ui, axis):
    """(text, state) for each button on an axis row."""
    return [(b.cget("text"), str(b.cget("state"))) for b in ui._mode_buttons[axis]]


def enabled_texts(ui, axis):
    return [text for text, state in states(ui, axis) if state == "normal"]


def test_default_mode_enables_the_compatible_options(app):
    assert app.picker.mode.label == "16:9 2.7K 60fps Wide"
    assert enabled_texts(app, "ratio") == ["16:9", "4:3"]
    assert enabled_texts(app, "resolution") == ["5.3K", "4K", "2.7K"]
    assert enabled_texts(app, "zoom") == ["none", "1.4x"]


def test_choosing_4_3_disables_5_3k_after_refresh(app):
    app.picker.commit(app.picker.preview("ratio", "4:3"))
    app._refresh_mode_controls()
    assert app._mode_vars["ratio"].get() == "4:3"
    assert "5.3K" not in enabled_texts(app, "resolution")
    assert enabled_texts(app, "resolution") == ["4K", "2.7K"]


def test_busy_or_recording_disables_every_button(app):
    app._busy = True
    app._update_mode_states()
    assert all(state == "disabled" for axis in app._mode_buttons for _, state in states(app, axis))
    app._busy = False
    app._recording = True
    app._update_mode_states()
    assert all(state == "disabled" for axis in app._mode_buttons for _, state in states(app, axis))


def test_pressing_a_disabled_value_reverts_the_selection(app):
    app._mode_vars["zoom"].set("2x")  # not allowed on the default Wide 16:9 mode
    app._on_mode_pick("zoom")
    assert app._mode_vars["zoom"].get() == "none"
    assert app._run_calls == []


def test_valid_pick_is_sent_and_committed_when_the_camera_accepts(app):
    app._mode_vars["resolution"].set("4K")
    app._on_mode_pick("resolution")
    status, done, _failed = app._run_calls[0]
    assert status == "Switching to 16:9 4K 120fps Wide…"
    done(None)
    assert app.picker.mode.label == "16:9 4K 120fps Wide"
    assert app._mode_vars["fps"].get() == "120"
    assert app.status.get() == "Mode set to 16:9 4K 120fps Wide."


def test_failed_pick_keeps_the_committed_mode(app):
    app._mode_vars["resolution"].set("4K")
    app._on_mode_pick("resolution")
    _status, _done, failed = app._run_calls[0]
    failed(RuntimeError("camera rejected video resolution"))
    assert app.picker.mode.label == "16:9 2.7K 60fps Wide"
    assert app._mode_vars["resolution"].get() == "2.7K"


def test_repeating_the_current_value_sends_nothing(app):
    app._mode_vars["resolution"].set("2.7K")
    app._on_mode_pick("resolution")
    assert app._run_calls == []


def test_full_window_locks_the_selector_until_connected_then_enables_it(root, monkeypatch, tmp_path):
    """Build the real LivestreamApp (not just the rows) against a fake controller."""
    import concurrent.futures

    monkeypatch.setattr(
        l,
        "PreviewReader",
        lambda *a, **k: SimpleNamespace(start=lambda: None, stop=lambda: None, latest_frame=lambda: None),
    )

    class FakeController:
        catalog = table.build_catalog()
        port = 8554

        output_dir = tmp_path

        def __init__(self):
            self.future = concurrent.futures.Future()

        async def connect(self, mode):
            return "GP-TEST"

        def submit(self, coro):
            coro.close()  # the camera is not used here
            return self.future

    controller = FakeController()
    ui = l.LivestreamApp(
        root,
        controller,
        upload=False,
        folder="default",
        image="",
        fadein_image="",
        fadeout_image="",
        config_file=tmp_path / "config.json",
    )
    assert ui.picker.mode.label == "16:9 2.7K 60fps Wide"
    # Connecting: every option is locked until the camera answers.
    assert all(state == "disabled" for axis in ui._mode_buttons for _, state in states(ui, axis))

    try:
        controller.future.set_result("GP-TEST")
        ui._poll()
        assert enabled_texts(ui, "zoom") == ["none", "1.4x"]
        assert ui.status.get() == "Connected to GoPro GP-TEST. Ready to record at 16:9 2.7K 60fps Wide."
    finally:
        ui.postprocessor.stop()  # its thread would otherwise keep pytest alive
