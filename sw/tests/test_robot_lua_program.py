"""Tests for sw/robot/lua_program.py (Lua ServoJ playback programs)."""

import re

import pytest

from sw.robot import lua_program

ROWS = [[0.0, 0, 0, 0, 0, 0], [1.0, 2, 3, 4, 5, 6], [2.0, 4, 6, 8, 10, 12]]


def _rows_in(text):
    return [
        [float(v) for v in m.group(1).split(",")]
        for m in re.finditer(r"^\{([^{}]*)\},$", text, re.M)
    ]


def test_render_lists_every_row_and_one_servoj_loop_without_a_wait():
    text = lua_program.render(ROWS, 8, "demo")
    assert "WaitMs" not in text  # ServoJ paces itself (real robot)
    assert _rows_in(text) == ROWS
    assert text.count("ServoJ(") == 1
    assert "ServoJ(P[i], E, 0, 0, 0.008, 0, 0)" in text
    assert "for i = 1, #P do" in text
    assert "E = {0, 0, 0, 0}" in text


def test_pace_wait_adds_waitms_and_queue_does_not():
    assert "WaitMs(8)" in lua_program.render(ROWS, 8, "demo", "wait")
    assert "WaitMs" not in lua_program.render(ROWS, 8, "demo", "queue")
    with pytest.raises(ValueError):
        lua_program.render(ROWS, 8, "demo", "fast")


def test_render_refuses_no_rows():
    with pytest.raises(ValueError):
        lua_program.render([], 8, "demo")


def test_resample_100_is_unchanged():
    assert lua_program.resample(ROWS, 100) == ROWS


def test_resample_50_doubles_the_intervals_on_the_same_path():
    out = lua_program.resample(ROWS, 50)
    assert len(out) == 5  # 2 intervals -> 4
    assert out[0] == ROWS[0] and out[-1] == ROWS[-1]
    assert out[1] == pytest.approx([0.5, 1, 1.5, 2, 2.5, 3])
    assert out[2] == pytest.approx(ROWS[1])


@pytest.mark.parametrize("ovl", [0, 101])
def test_resample_refuses_out_of_range_ovl(ovl):
    with pytest.raises(ValueError):
        lua_program.resample(ROWS, ovl)
