"""Tests for the cascading video mode model and the HERO10 catalog."""

import dataclasses

import pytest

from sw.camera import hero10_table as t
from sw.camera.modes import AXES, NO_ZOOM, ModeCatalog, VideoMode

B = VideoMode("16:9", "4K", 120, "Wide")
C = VideoMode("16:9", "4K", 120, "Narrow")
D = VideoMode("16:9", "2.7K", 60, "Wide")
E = VideoMode("4:3", "2.7K", 60, "Linear")
F = VideoMode("4:3", "2.7K", 60, "Narrow")
SMALL_MODES = (B, C, D, E, F)


def small_zoom_max(mode: VideoMode) -> int:
    """Wide and Linear can zoom to 100%; Narrow cannot."""
    return 100 if mode.lens in ("Wide", "Linear") else 0


@pytest.fixture
def small() -> ModeCatalog:
    return ModeCatalog(SMALL_MODES, default=D, zoom_max=small_zoom_max)


def enabled(options):
    return {o.value for o in options if o.enabled}


# --- VideoMode -------------------------------------------------------------


def test_video_mode_is_immutable_and_hashable():
    assert hash(D) == hash(VideoMode("16:9", "2.7K", 60, "Wide"))
    with pytest.raises(dataclasses.FrozenInstanceError):
        D.fps = 30  # type: ignore[misc]


def test_label_includes_zoom_only_when_set():
    assert D.label == "16:9 2.7K 60fps Wide"
    assert (
        VideoMode("16:9", "4K", 120, "Wide", 42).label == "16:9 4K 120fps Wide zoom 42%"
    )


# --- ModeCatalog construction ----------------------------------------------


def test_duplicates_are_dropped_and_order_kept():
    assert ModeCatalog([D, B, D, B], default=D).modes == (D, B)


def test_catalog_modes_must_have_zero_zoom():
    with pytest.raises(ValueError):
        ModeCatalog(
            [VideoMode("16:9", "4K", 120, "Wide", 10)],
            default=VideoMode("16:9", "4K", 120, "Wide", 10),
        )


def test_default_must_be_in_catalog():
    with pytest.raises(ValueError):
        ModeCatalog([B], default=D)


def test_catalog_needs_at_least_one_mode():
    with pytest.raises(ValueError):
        ModeCatalog([], default=D)


def test_unknown_axis_is_rejected(small):
    with pytest.raises(ValueError):
        small.values("bogus")


def test_values_are_listed_in_first_appearance_order(small):
    assert small.values("ratio") == ("16:9", "4:3")
    assert small.values("lens") == ("Wide", "Narrow", "Linear")


# --- validity and zoom range ----------------------------------------------


def test_is_valid_accepts_zoom_within_the_range(small):
    assert small.is_valid(D)
    assert small.is_valid(dataclasses.replace(D, zoom=0))
    assert small.is_valid(dataclasses.replace(D, zoom=100))
    assert small.is_valid(dataclasses.replace(D, zoom=37))


def test_is_valid_refuses_zoom_outside_the_range_or_not_a_percentage(small):
    narrow = VideoMode("16:9", "2.7K", 60, "Narrow")
    assert not small.is_valid(
        dataclasses.replace(narrow, zoom=5)
    )  # no zoom with Narrow
    assert not small.is_valid(dataclasses.replace(D, zoom=101))
    assert not small.is_valid(dataclasses.replace(D, zoom=-1))
    assert not small.is_valid(dataclasses.replace(D, zoom="1.4x"))  # type: ignore[arg-type]
    assert not small.is_valid(dataclasses.replace(D, zoom=True))  # type: ignore[arg-type]


def test_zoom_max_comes_from_the_supplied_rule(small):
    assert small.zoom_max(D) == 100
    assert small.zoom_max(C) == 0


def test_default_catalog_has_no_zoom(small):
    plain = ModeCatalog(SMALL_MODES, default=D)
    assert plain.zoom_max(D) == 0
    assert not plain.is_valid(dataclasses.replace(D, zoom=10))


# --- options: which discrete values are enabled ---------------------------


def test_ratio_options_are_all_enabled(small):
    assert enabled(small.options("ratio", {})) == {"16:9", "4:3"}


def test_resolution_depends_on_ratio(small):
    assert enabled(small.options("resolution", {"ratio": "16:9"})) == {"4K", "2.7K"}
    assert enabled(small.options("resolution", {"ratio": "4:3"})) == {"2.7K"}


def test_disabled_values_are_still_listed(small):
    options = small.options("resolution", {"ratio": "4:3"})
    assert [o.value for o in options] == ["4K", "2.7K"]
    assert options[0].enabled is False


def test_fps_depends_on_ratio_and_resolution(small):
    assert enabled(small.options("fps", {"ratio": "16:9", "resolution": "2.7K"})) == {
        60
    }
    assert enabled(small.options("fps", {"ratio": "16:9", "resolution": "4K"})) == {120}


def test_lens_depends_on_earlier_axes(small):
    assert enabled(
        small.options("lens", {"ratio": "16:9", "resolution": "4K", "fps": 120})
    ) == {"Wide", "Narrow"}


def test_later_axes_do_not_affect_earlier_options(small):
    before = enabled(small.options("resolution", {"ratio": "16:9"}))
    after = enabled(small.options("resolution", {"ratio": "16:9", "lens": "Narrow"}))
    assert before == after


# --- select: a pick, with downstream re-cascade ----------------------------


def test_select_ratio_keeps_downstream_values_that_are_still_valid(small):
    # 2.7K and 60fps carry over; Wide is not in 4:3, so the lens falls back to Narrow.
    assert small.select(D, "ratio", "4:3") == F


def test_select_ratio_falls_back_for_axes_the_new_ratio_does_not_offer(small):
    # 4:3 has no 4K, no 120fps and no Wide lens here, so those axes fall back.
    assert small.select(B, "ratio", "4:3") == F


def test_select_resolution_recascades_fps(small):
    # D is 2.7K 60fps; 4K only offers 120fps, so fps must change.
    assert small.select(D, "resolution", "4K") == B


def test_select_lens_resets_zoom_when_the_lens_cannot_zoom(small):
    zoomed = dataclasses.replace(B, zoom=60)
    assert small.select(zoomed, "lens", "Narrow") == C


def test_select_zoom_keeps_the_rest_of_the_mode(small):
    assert small.select(B, "zoom", 60) == dataclasses.replace(B, zoom=60)


def test_select_zoom_is_refused_outside_the_range(small):
    with pytest.raises(ValueError):
        small.select(C, "zoom", 10)  # Narrow has no zoom
    with pytest.raises(ValueError):
        small.select(D, "zoom", 101)


def test_select_zoom_must_be_a_whole_percentage(small):
    with pytest.raises(ValueError):
        small.select(D, "zoom", "50")  # type: ignore[arg-type]


def test_select_discrete_value_rejects_a_disabled_choice(small):
    with pytest.raises(ValueError):
        small.select(D, "fps", 120)  # 2.7K 16:9 does not offer 120fps in this catalog


# --- resolve: nearest valid mode ------------------------------------------


def test_resolve_keeps_a_valid_mode(small):
    assert small.resolve(D) == D
    assert small.resolve(dataclasses.replace(D, zoom=45)) == dataclasses.replace(
        D, zoom=45
    )


def test_resolve_clamps_zoom_to_the_range(small):
    assert small.resolve(dataclasses.replace(D, zoom=500)).zoom == 100
    assert small.resolve(dataclasses.replace(D, zoom=-5)).zoom == 0


def test_resolve_drops_zoom_when_the_lens_changes_to_one_without_zoom(small):
    assert small.resolve(dataclasses.replace(C, zoom=30)) == C


def test_resolve_repairs_an_unknown_ratio(small):
    assert small.resolve(VideoMode("1:1", "2.7K", 60, "Wide")) == D


def test_resolve_always_returns_a_catalog_mode(small):
    assert small.is_valid(small.resolve(VideoMode("4:3", "4K", 120, "Narrow", 80)))


# --- HERO10 catalog ---------------------------------------------------------


@pytest.fixture(scope="module")
def hero() -> ModeCatalog:
    return t.build_catalog()


def test_hero_default_is_the_requested_starting_mode(hero):
    assert hero.default.label == "16:9 2.7K 60fps Wide"
    assert hero.is_valid(hero.default)


def test_hero_catalog_lists_each_mode_once_with_zero_zoom(hero):
    expected = sum(len(row.fps) * len(row.lenses) for row in t.ROWS)
    assert len(hero.modes) == expected
    assert all(m.zoom == NO_ZOOM for m in hero.modes)


def test_hero_resolution_after_ratio_16_9_includes_5_3k(hero):
    assert enabled(hero.options("resolution", {"ratio": "16:9"})) == {
        "5.3K",
        "4K",
        "2.7K",
    }


def test_hero_resolution_after_ratio_4_3_excludes_5_3k(hero):
    assert enabled(hero.options("resolution", {"ratio": "4:3"})) == {"4K", "2.7K"}


def test_hero_superview_is_disabled_in_4_3(hero):
    assert "SuperView" not in enabled(
        hero.options("lens", {"ratio": "4:3", "resolution": "4K", "fps": 60})
    )


def test_hero_zoom_range_depends_on_the_lens(hero):
    wide = VideoMode("16:9", "4K", 120, "Wide")
    linear = VideoMode("16:9", "4K", 120, "Linear")
    narrow = VideoMode("16:9", "4K", 120, "Narrow")
    superview = VideoMode("16:9", "2.7K", 60, "SuperView")
    assert hero.zoom_max(wide) == 100
    assert (
        hero.zoom_max(linear) == 0
    )  # 4K 120fps allows zoom on Wide and Linear+HL only
    assert hero.zoom_max(narrow) == 0
    assert hero.zoom_max(superview) == 0


def test_hero_narrow_never_allows_zoom(hero):
    for mode in hero.modes:
        if mode.lens == "Narrow":
            assert hero.zoom_max(mode) == 0


def test_hero_every_mode_is_reachable_and_zoomable_through_the_cascade(hero):
    for target in hero.modes:
        state = hero.default
        for axis in AXES[:-1]:
            state = hero.select(state, axis, getattr(target, axis))
        assert state == target, target.label
        if hero.zoom_max(target) >= 37:
            assert hero.select(state, "zoom", 37).zoom == 37


def test_hero_resolve_is_identity_on_valid_modes(hero):
    for mode in hero.modes:
        assert hero.resolve(mode) == mode
