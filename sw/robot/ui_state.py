"""Pure state machine for the livestream app's robot motion controls (Phase 6).

`transition(state, event, ctx)` returns the new state and the actions the Tk
app must carry out, in order. It never touches Tk, the robot or the camera,
so every (state, event) pair is unit-testable. Events not listed for a state
leave it unchanged and return no actions (the app logs them).

Recording without a motion ("No motion" selected, or no robot) does not go
through this machine at all: the app keeps its original record/stop flow.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass
from typing import Any


class UiState(enum.Enum):
    UNAVAILABLE = "unavailable"  # no robot: --no-robot or startup failure
    CHECKING = "checking"
    IDLE = "idle"
    LOADING = "loading"
    MOVING_TO_START = "moving_to_start"
    STARTING_RECORDING = "starting_recording"
    PRE_ROLL = "pre_roll"
    RUNNING = "running"
    POST_ROLL = "post_roll"
    STOPPING = "stopping"
    FAULT = "fault"
    CLOSED = "closed"


class Ev(enum.Enum):
    STARTUP_CHECKED = "startup_checked"
    STARTUP_FAILED = "startup_failed"  # arg: reason
    SELECT_CONFIRMED = "select_confirmed"  # arg: motion name or None
    LOADED = "loaded"
    LOAD_FAILED = "load_failed"  # arg: error
    MOVE_TO_START = "move_to_start"
    MOVE_DONE = "move_done"
    MOVE_FAILED = "move_failed"  # arg: error
    RECORD_PRESSED = "record_pressed"
    RECORDING_STARTED = "recording_started"
    RECORDING_FAILED = "recording_failed"  # arg: error
    PRE_ROLL_ELAPSED = "pre_roll_elapsed"
    MOTION_DONE = "motion_done"
    MOTION_FAILED = "motion_failed"  # arg: error
    POST_ROLL_ELAPSED = "post_roll_elapsed"
    STOP_PRESSED = "stop_pressed"
    RECORDING_STOPPED = "recording_stopped"  # also sent when the stop failed
    FAULT_CLEARED = "fault_cleared"
    CLOSE = "close"


class Act(enum.Enum):
    LOAD = "load"  # arg: motion name
    MOVE_TO_START = "move_to_start"  # arg: motion name
    START_RECORDING = "start_recording"
    SCHEDULE_PRE_ROLL = "schedule_pre_roll"
    RUN = "run"  # arg: motion name
    SCHEDULE_POST_ROLL = "schedule_post_roll"
    CANCEL_TIMERS = "cancel_timers"
    STOP_ROBOT = "stop_robot"
    STOP_RECORDING = "stop_recording"
    CLEAR_FAULT = "clear_fault"
    CLOSE_RUNNER = "close_runner"
    MARK_INVALID = "mark_invalid"  # arg: (motion name, reason)
    STATUS = "status"  # arg: text for the robot status line


@dataclass(frozen=True)
class Event:
    kind: Ev
    arg: Any = None


@dataclass(frozen=True)
class Action:
    kind: Act
    arg: Any = None


@dataclass(frozen=True)
class Context:
    """What the app knows when the event arrives."""

    motion: str | None = None  # selected motion
    at_start: bool = False
    start_error_deg: float = 0.0
    auto_stop: bool = True  # RobotSettings.auto_stop_after_motion
    runner_fault: bool = False  # MotionRunner.state is FAULT
    move_speed_pct: int = 25


S = UiState
MOTION_STATES = (S.PRE_ROLL, S.RUNNING, S.POST_ROLL)


def _status(text: str) -> Action:
    return Action(Act.STATUS, text)


def _after_failure(ctx: Context) -> UiState:
    return S.FAULT if ctx.runner_fault else S.IDLE


def transition(
    state: UiState, event: Event, ctx: Context
) -> tuple[UiState, list[Action]]:
    kind, arg, m = event.kind, event.arg, ctx.motion

    if kind is Ev.CLOSE:
        if state is S.CLOSED:
            return state, []
        return S.CLOSED, [
            Action(Act.CANCEL_TIMERS),
            Action(Act.STOP_ROBOT),
            Action(Act.CLOSE_RUNNER),
        ]

    if state is S.CHECKING:
        if kind is Ev.STARTUP_CHECKED:
            return S.IDLE, [_status("Robot ready.")]
        if kind is Ev.STARTUP_FAILED:
            return S.UNAVAILABLE, [_status(f"Robot unavailable: {arg}")]

    elif state is S.IDLE:
        if kind is Ev.SELECT_CONFIRMED:
            if arg is None:
                return S.IDLE, [_status("No motion selected.")]
            return S.LOADING, [Action(Act.LOAD, arg), _status(f"Loading '{arg}'…")]
        if kind is Ev.MOVE_TO_START and m is not None:
            return S.MOVING_TO_START, [
                Action(Act.MOVE_TO_START, m),
                _status(f"Moving to start of '{m}' ({ctx.move_speed_pct}%)…"),
            ]
        if kind is Ev.RECORD_PRESSED and m is not None:
            if ctx.at_start:
                return S.STARTING_RECORDING, [Action(Act.START_RECORDING)]
            return S.IDLE, [
                _status(
                    f"Not at start of '{m}' (max error {ctx.start_error_deg:.1f}°). "
                    "Press Move to Start."
                )
            ]

    elif state is S.LOADING:
        if kind is Ev.LOADED:
            return S.IDLE, [_status(f"Motion '{m}' loaded. Press Move to Start.")]
        if kind is Ev.LOAD_FAILED:
            return _after_failure(ctx), [
                Action(Act.MARK_INVALID, (m, f"invalid at load: {arg}")),
                _status(f"Loading '{m}' failed: {arg}"),
            ]

    elif state is S.MOVING_TO_START:
        if kind is Ev.MOVE_DONE:
            return S.IDLE, [_status(f"At start of '{m}'. Ready to record.")]
        if kind is Ev.MOVE_FAILED:
            return _after_failure(ctx), [_status(f"Move to start ended: {arg}")]
        if kind is Ev.STOP_PRESSED:
            # Stay until the runner reports the move ended (MotionStopped).
            return S.MOVING_TO_START, [Action(Act.STOP_ROBOT)]

    elif state is S.STARTING_RECORDING:
        if kind is Ev.RECORDING_STARTED:
            return S.PRE_ROLL, [Action(Act.SCHEDULE_PRE_ROLL)]
        if kind is Ev.RECORDING_FAILED:
            return S.IDLE, [
                _status(f"Recording did not start ({arg}); the robot did not move.")
            ]

    elif state is S.PRE_ROLL:
        if kind is Ev.PRE_ROLL_ELAPSED:
            return S.RUNNING, [Action(Act.RUN, m), _status(f"Running '{m}'…")]

    elif state is S.RUNNING:
        if kind is Ev.MOTION_DONE:
            if ctx.auto_stop:
                return S.POST_ROLL, [
                    Action(Act.SCHEDULE_POST_ROLL),
                    _status(f"'{m}' done."),
                ]
            return S.POST_ROLL, [_status(f"'{m}' done. Press Stop recording.")]
        if kind is Ev.MOTION_FAILED:
            return S.STOPPING, [
                Action(Act.STOP_ROBOT),
                Action(Act.STOP_RECORDING),
                _status(f"Motion failed: {arg}"),
            ]

    elif state is S.POST_ROLL:
        if kind is Ev.POST_ROLL_ELAPSED:
            return S.STOPPING, [Action(Act.STOP_RECORDING)]

    elif state is S.STOPPING:
        if kind is Ev.RECORDING_STOPPED:
            if ctx.runner_fault:
                return S.FAULT, [_status("Robot fault. Press Clear fault.")]
            return S.IDLE, []

    elif state is S.FAULT:
        if kind is Ev.FAULT_CLEARED:
            return S.IDLE, [Action(Act.CLEAR_FAULT), _status("Fault cleared.")]
        if kind is Ev.RECORD_PRESSED and m is not None:
            return S.FAULT, [_status("Robot fault: press Clear fault first.")]

    if state in MOTION_STATES and kind is Ev.STOP_PRESSED:
        return S.STOPPING, [
            Action(Act.CANCEL_TIMERS),
            Action(Act.STOP_ROBOT),
            Action(Act.STOP_RECORDING),
        ]

    return state, []
