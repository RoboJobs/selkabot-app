"""Consistency checks for the HERO10 mode table against GoPro's published rules."""

import pytest

from sw.camera import hero10_table as t


def test_table_has_only_ntsc_frame_rates():
    fps = {f for r in t.ROWS for f in r.fps}
    assert fps == {60, 120, 240}


def test_table_has_the_requested_rows():
    seen = {(r.ratio, r.resolution, r.fps) for r in t.ROWS}
    assert seen == {
        ("16:9", "5.3K", (60,)),
        ("16:9", "4K", (120,)),
        ("16:9", "2.7K", (240,)),
        ("16:9", "2.7K", (120,)),
        ("16:9", "2.7K", (60,)),
        ("4:3", "4K", (60,)),
        ("4:3", "2.7K", (120,)),
    }


def test_no_duplicate_rows():
    keys = [(r.ratio, r.resolution, r.fps) for r in t.ROWS]
    assert len(keys) == len(set(keys))


@pytest.mark.parametrize(
    "row", t.ROWS, ids=lambda r: f"{r.ratio}-{r.resolution}-{r.fps}"
)
def test_zoom_lenses_are_a_subset_of_lenses(row):
    assert set(row.zoom_lenses) <= set(row.lenses)


@pytest.mark.parametrize(
    "row", t.ROWS, ids=lambda r: f"{r.ratio}-{r.resolution}-{r.fps}"
)
def test_narrow_never_allows_zoom(row):
    assert t.LENS_NARROW not in row.zoom_lenses


@pytest.mark.parametrize(
    "row", t.ROWS, ids=lambda r: f"{r.ratio}-{r.resolution}-{r.fps}"
)
def test_superview_only_on_16_9(row):
    if t.LENS_SUPERVIEW in row.lenses:
        assert row.ratio == "16:9"


def test_4_3_has_no_superview():
    for row in t.ROWS:
        if row.ratio == "4:3":
            assert t.LENS_SUPERVIEW not in row.lenses


def test_zoom_is_continuous_up_to_100_on_zoom_lenses_only():
    assert t.ZOOM_MAX_PERCENT == 100
    for row in t.ROWS:
        for lens in row.lenses:
            mode = t.VideoMode(row.ratio, row.resolution, row.fps[0], lens)
            expected = 100 if lens in row.zoom_lenses else 0
            assert t.zoom_max(mode) == expected


def test_every_resolution_has_a_camera_enum():
    for row in t.ROWS:
        assert (row.ratio, row.resolution) in t.RESOLUTION_ENUM


def test_every_fps_has_a_camera_enum():
    for row in t.ROWS:
        for fps in row.fps:
            assert fps in t.FPS_ENUM


def test_every_lens_has_a_camera_enum():
    for row in t.ROWS:
        for lens in row.lenses:
            assert lens in t.LENS_ENUM


def test_enum_maps_have_no_duplicate_camera_values():
    assert len(set(t.RESOLUTION_ENUM.values())) == len(t.RESOLUTION_ENUM)
    assert len(set(t.FPS_ENUM.values())) == len(t.FPS_ENUM)
    assert len(set(t.LENS_ENUM.values())) == len(t.LENS_ENUM)
