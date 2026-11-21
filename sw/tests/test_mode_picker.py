"""Tests for the cascading selector's pure logic (sw/camera/mode_picker.py)."""

import pytest

from sw.camera import hero10_table as table
from sw.camera.mode_picker import ModePicker
from sw.camera.modes import VideoMode


@pytest.fixture
def picker() -> ModePicker:
    return ModePicker(table.build_catalog())


def enabled_texts(options) -> list[str]:
    return [o.text for o in options if o.enabled]


def test_starts_on_the_requested_default(picker):
    assert picker.mode.label == "16:9 2.7K 60fps Wide"
    assert picker.selected_text("ratio") == "16:9"
    assert picker.selected_text("fps") == "60"
    assert picker.zoom_range() == (0, 100)


def test_start_mode_must_be_in_the_catalog():
    with pytest.raises(ValueError):
        ModePicker(table.build_catalog(), VideoMode("16:9", "5.3K", 240, "Wide", 0))


def test_every_axis_lists_all_values_even_when_disabled(picker):
    assert [o.text for o in picker.options("resolution")] == ["5.3K", "4K", "2.7K"]


def test_ratio_choices_are_all_enabled_from_the_default(picker):
    assert enabled_texts(picker.options("ratio")) == ["16:9", "4:3"]


def test_only_compatible_resolutions_are_enabled_after_choosing_4_3():
    picker = ModePicker(table.build_catalog(), VideoMode("4:3", "2.7K", 120, "Wide", 0))
    assert enabled_texts(picker.options("resolution")) == ["4K", "2.7K"]


def test_fps_options_follow_the_committed_resolution(picker):
    assert enabled_texts(picker.options("fps")) == ["60", "120", "240"]
    assert [o.text for o in picker.options("fps")] == ["60", "120", "240"]
    picker.commit(VideoMode("16:9", "5.3K", 60, "Wide", 0))
    assert enabled_texts(picker.options("fps")) == ["60"]


def test_preview_returns_the_cascaded_mode_without_committing(picker):
    target = picker.preview("ratio", "4:3")
    assert target.label == "4:3 2.7K 120fps Wide"
    assert picker.mode.label == "16:9 2.7K 60fps Wide"


def test_preview_of_an_unknown_text_is_refused(picker):
    with pytest.raises(ValueError):
        picker.preview("fps", "30")


def test_zoom_range_depends_on_the_lens(picker):
    picker.commit(VideoMode("16:9", "4K", 120, "Narrow", 0))
    assert picker.zoom_range() == (0, 0)
    picker.commit(VideoMode("16:9", "4K", 120, "Wide", 0))
    assert picker.zoom_range() == (0, 100)


def test_preview_zoom_sets_any_percentage_in_range(picker):
    assert picker.preview_zoom(42).zoom == 42
    assert picker.mode.zoom == 0  # not committed


def test_preview_zoom_refuses_values_outside_the_range(picker):
    with pytest.raises(ValueError):
        picker.preview_zoom(101)
    picker.commit(VideoMode("16:9", "4K", 120, "Narrow", 0))
    with pytest.raises(ValueError):
        picker.preview_zoom(10)


def test_zoom_is_not_a_discrete_pick(picker):
    with pytest.raises(ValueError):
        picker.preview("zoom", "50")


def test_commit_moves_the_selector_to_the_new_mode(picker):
    picker.commit(picker.preview("ratio", "4:3"))
    assert picker.mode.label == "4:3 2.7K 120fps Wide"
    assert picker.selected_text("ratio") == "4:3"
    assert enabled_texts(picker.options("fps")) == ["120"]


def test_commit_refuses_a_mode_outside_the_catalog(picker):
    with pytest.raises(ValueError):
        picker.commit(VideoMode("4:3", "5.3K", 60, "Wide", 0))
    assert picker.mode.label == "16:9 2.7K 60fps Wide"
