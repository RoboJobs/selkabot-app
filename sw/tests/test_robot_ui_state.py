"""Table-driven tests for sw/robot/ui_state.py: every (state, event) pair."""

import itertools

import pytest

from sw.robot.ui_state import Act, Context, Ev, Event, UiState, transition

S = UiState
CTX = Context(motion="m", at_start=True, auto_stop=True, runner_fault=False)

# (state, event) -> (new state, action kinds) with CTX and arg "x".
EXPECTED = {
    (S.CHECKING, Ev.STARTUP_CHECKED): (S.IDLE, [Act.STATUS]),
    (S.CHECKING, Ev.STARTUP_FAILED): (S.UNAVAILABLE, [Act.STATUS]),
    (S.IDLE, Ev.SELECT_CONFIRMED): (S.LOADING, [Act.LOAD, Act.STATUS]),
    (S.IDLE, Ev.MOVE_TO_START): (S.MOVING_TO_START, [Act.MOVE_TO_START, Act.STATUS]),
    (S.IDLE, Ev.RECORD_PRESSED): (S.STARTING_RECORDING, [Act.START_RECORDING]),
    (S.LOADING, Ev.LOADED): (S.IDLE, [Act.STATUS]),
    (S.LOADING, Ev.LOAD_FAILED): (S.IDLE, [Act.MARK_INVALID, Act.STATUS]),
    (S.MOVING_TO_START, Ev.MOVE_DONE): (S.IDLE, [Act.STATUS]),
    (S.MOVING_TO_START, Ev.MOVE_FAILED): (S.IDLE, [Act.STATUS]),
    (S.MOVING_TO_START, Ev.STOP_PRESSED): (S.MOVING_TO_START, [Act.STOP_ROBOT]),
    (S.STARTING_RECORDING, Ev.RECORDING_STARTED): (
        S.PRE_ROLL,
        [Act.SCHEDULE_PRE_ROLL],
    ),
    (S.STARTING_RECORDING, Ev.RECORDING_FAILED): (S.IDLE, [Act.STATUS]),
    (S.PRE_ROLL, Ev.PRE_ROLL_ELAPSED): (S.RUNNING, [Act.RUN, Act.STATUS]),
    (S.RUNNING, Ev.MOTION_DONE): (S.POST_ROLL, [Act.SCHEDULE_POST_ROLL, Act.STATUS]),
    (S.RUNNING, Ev.MOTION_FAILED): (
        S.STOPPING,
        [Act.STOP_ROBOT, Act.STOP_RECORDING, Act.STATUS],
    ),
    (S.POST_ROLL, Ev.POST_ROLL_ELAPSED): (S.STOPPING, [Act.STOP_RECORDING]),
    (S.STOPPING, Ev.RECORDING_STOPPED): (S.IDLE, []),
    (S.FAULT, Ev.FAULT_CLEARED): (S.IDLE, [Act.CLEAR_FAULT, Act.STATUS]),
    (S.FAULT, Ev.RECORD_PRESSED): (S.FAULT, [Act.STATUS]),
}
STOP_SEQUENCE = [Act.CANCEL_TIMERS, Act.STOP_ROBOT, Act.STOP_RECORDING]
for _s in (S.PRE_ROLL, S.RUNNING, S.POST_ROLL):
    EXPECTED[(_s, Ev.STOP_PRESSED)] = (S.STOPPING, STOP_SEQUENCE)
CLOSE_SEQUENCE = [Act.CANCEL_TIMERS, Act.STOP_ROBOT, Act.CLOSE_RUNNER]
for _s in S:
    if _s is not S.CLOSED:
        EXPECTED[(_s, Ev.CLOSE)] = (S.CLOSED, CLOSE_SEQUENCE)


@pytest.mark.parametrize("state,kind", list(itertools.product(S, Ev)))
def test_every_state_event_pair(state, kind):
    new, actions = transition(state, Event(kind, "x"), CTX)
    expected_state, expected_kinds = EXPECTED.get((state, kind), (state, []))
    assert new is expected_state
    assert [a.kind for a in actions] == expected_kinds


def test_select_no_motion_stays_idle_without_loading():
    new, actions = transition(S.IDLE, Event(Ev.SELECT_CONFIRMED, None), CTX)
    assert new is S.IDLE
    assert [a.kind for a in actions] == [Act.STATUS]


def test_select_loads_the_named_motion():
    _, actions = transition(S.IDLE, Event(Ev.SELECT_CONFIRMED, "sweep"), CTX)
    assert actions[0].arg == "sweep"


def test_record_not_at_start_refuses_with_the_error():
    ctx = Context(motion="m", at_start=False, start_error_deg=12.34)
    new, actions = transition(S.IDLE, Event(Ev.RECORD_PRESSED), ctx)
    assert new is S.IDLE
    assert [a.kind for a in actions] == [Act.STATUS]
    assert "12.3°" in actions[0].arg


@pytest.mark.parametrize("kind", [Ev.RECORD_PRESSED, Ev.MOVE_TO_START])
def test_without_a_motion_record_and_move_are_ignored(kind):
    assert transition(S.IDLE, Event(kind), Context(motion=None)) == (S.IDLE, [])


def test_motion_done_without_auto_stop_waits_for_the_operator():
    ctx = Context(motion="m", auto_stop=False)
    new, actions = transition(S.RUNNING, Event(Ev.MOTION_DONE), ctx)
    assert new is S.POST_ROLL
    assert [a.kind for a in actions] == [Act.STATUS]


@pytest.mark.parametrize(
    "state,kind",
    [
        (S.LOADING, Ev.LOAD_FAILED),
        (S.MOVING_TO_START, Ev.MOVE_FAILED),
        (S.STOPPING, Ev.RECORDING_STOPPED),
    ],
)
def test_failures_go_to_fault_when_the_runner_is_in_fault(state, kind):
    ctx = Context(motion="m", runner_fault=True)
    new, _ = transition(state, Event(kind, "boom"), ctx)
    assert new is S.FAULT


def test_load_failure_marks_the_motion_invalid():
    _, actions = transition(S.LOADING, Event(Ev.LOAD_FAILED, "bad pose"), CTX)
    assert actions[0].arg == ("m", "invalid at load: bad pose")


def test_stopping_ignores_late_motion_results():
    for kind in (Ev.MOTION_DONE, Ev.MOTION_FAILED, Ev.STOP_PRESSED):
        assert transition(S.STOPPING, Event(kind), CTX) == (S.STOPPING, [])


def test_stop_robot_comes_before_stop_recording():
    _, actions = transition(S.RUNNING, Event(Ev.STOP_PRESSED), CTX)
    kinds = [a.kind for a in actions]
    assert kinds.index(Act.STOP_ROBOT) < kinds.index(Act.STOP_RECORDING)
