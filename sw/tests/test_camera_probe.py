"""Tests for the pure helpers in the hardware probe (no camera needed)."""

from open_gopro.models.constants import SettingId

from sw import camera_probe as probe
from sw.camera import hero10_table as t


def test_probe_cases_are_unique_and_cover_every_fps_and_lens():
    cases = probe.probe_cases()
    assert len(cases) == len(set(cases))
    expected = {(r.ratio, r.resolution, fps, lens) for r in t.ROWS for fps in r.fps for lens in r.lenses}
    assert {(c.ratio, c.resolution, c.fps, c.lens) for c in cases} == expected


def test_probe_cases_from_a_custom_row_list():
    row = t.TableRow("16:9", "2.7K", (60,), ("Wide",), ("Wide",))
    cases = probe.probe_cases([row, row])
    assert [c.label for c in cases] == ["16:9 2.7K 60fps Wide"]


def _state_for(case: probe.ProbeCase) -> dict:
    return {
        SettingId.VIDEO_RESOLUTION: t.RESOLUTION_ENUM[(case.ratio, case.resolution)],
        SettingId.FRAMES_PER_SECOND: t.FPS_ENUM[case.fps],
        SettingId.VIDEO_LENS: t.LENS_ENUM[case.lens],
    }


def test_compare_state_accepts_matching_state():
    case = probe.ProbeCase("16:9", "2.7K", 60, "Wide")
    assert probe.compare_state(case, _state_for(case)) == ()


def test_compare_state_reports_wrong_fps():
    case = probe.ProbeCase("16:9", "2.7K", 60, "Wide")
    state = _state_for(case)
    state[SettingId.FRAMES_PER_SECOND] = t.FPS_ENUM[120]
    problems = probe.compare_state(case, state)
    assert len(problems) == 1 and problems[0].startswith("fps:")


def test_compare_state_reports_missing_setting():
    case = probe.ProbeCase("4:3", "4K", 50, "Linear")
    state = _state_for(case)
    del state[SettingId.VIDEO_LENS]
    assert probe.compare_state(case, state) == ("lens: not reported",)


def test_format_report_counts_and_marks_failures():
    ok = probe.ProbeResult(probe.ProbeCase("16:9", "4K", 120, "Wide"), True)
    bad = probe.ProbeResult(probe.ProbeCase("16:9", "4K", 120, "Narrow"), False, ("lens: wanted X",))
    text = probe.format_report([ok, bad])
    lines = text.splitlines()
    assert lines[0] == "OK    16:9 4K 120fps Wide"
    assert lines[1] == "FAIL  16:9 4K 120fps Narrow  (lens: wanted X)"
    assert lines[-1] == "1 accepted, 1 rejected"


def test_format_report_shows_error_when_no_mismatch_list():
    bad = probe.ProbeResult(probe.ProbeCase("16:9", "4K", 120, "Wide"), False, error="timeout")
    assert "(timeout)" in probe.format_report([bad])
