"""Tests for the cascading video mode model (sw/camera/modes.py) and the HERO10 catalog."""

import dataclasses

import pytest

from sw.camera import hero10_table as t
from sw.camera.modes import AXES, NO_ZOOM, ModeCatalog, VideoMode

B = VideoMode("16:9", "4K", 120, "Wide", NO_ZOOM)
A = VideoMode("16:9", "4K", 120, "Wide", "1.4x")
C = VideoMode("16:9", "4K", 120, "Narrow", NO_ZOOM)
D = VideoMode("16:9", "2.7K", 60, "Wide", NO_ZOOM)
E = VideoMode("4:3", "2.7K", 60, "Linear", "2x")
F = VideoMode("4:3", "2.7K", 60, "Linear", NO_ZOOM)
SMALL_MODES = (B, A, C, D, E, F)


@pytest.fixture
def small() -> ModeCatalog:
    return ModeCatalog(SMALL_MODES, default=D)


def enabled(options):
    return {o.value for o in options if o.enabled}


# --- VideoMode -------------------------------------------------------------


def test_video_mode_is_immutable_and_hashable():
    assert hash(D) == hash(VideoMode("16:9", "2.7K", 60, "Wide", NO_ZOOM))
    with pytest.raises(dataclasses.FrozenInstanceError):
        D.fps = 30  # type: ignore[misc]


def test_label_includes_zoom_only_when_set():
    assert D.label == "16:9 2.7K 60fps Wide"
    assert A.label == "16:9 4K 120fps Wide 1.4x"


# --- ModeCatalog construction ----------------------------------------------


def test_duplicates_are_dropped_and_order_kept():
    catalog = ModeCatalog([D, B, D, B], default=D)
    assert catalog.modes == (D, B)


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
    assert small.values("zoom") == (NO_ZOOM, "1.4x", "2x")


def test_is_valid(small):
    assert small.is_valid(A)
    assert not small.is_valid(VideoMode("4:3", "4K", 120, "Wide", NO_ZOOM))


# --- options: which values are enabled at each cascade level ---------------


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
    assert enabled(small.options("fps", {"ratio": "16:9", "resolution": "2.7K"})) == {60}
    assert enabled(small.options("fps", {"ratio": "16:9", "resolution": "4K"})) == {120}


def test_lens_depends_on_earlier_axes(small):
    options = small.options("lens", {"ratio": "16:9", "resolution": "4K", "fps": 120})
    assert enabled(options) == {"Wide", "Narrow"}


def test_zoom_none_is_always_available_when_mode_exists(small):
    narrow = {"ratio": "16:9", "resolution": "4K", "fps": 120, "lens": "Narrow"}
    assert enabled(small.options("zoom", narrow)) == {NO_ZOOM}


def test_zoom_offers_only_the_factor_allowed_by_the_lens(small):
    wide = {"ratio": "16:9", "resolution": "4K", "fps": 120, "lens": "Wide"}
    assert enabled(small.options("zoom", wide)) == {NO_ZOOM, "1.4x"}
    four_three = {"ratio": "4:3", "resolution": "2.7K", "fps": 60, "lens": "Linear"}
    assert enabled(small.options("zoom", four_three)) == {NO_ZOOM, "2x"}


def test_later_axes_do_not_affect_earlier_options(small):
    # Lens should not constrain the resolution list.
    before = enabled(small.options("resolution", {"ratio": "16:9"}))
    after = enabled(small.options("resolution", {"ratio": "16:9", "lens": "Narrow"}))
    assert before == after


# --- select: user picks a value, downstream axes re-cascade ----------------


def test_select_ratio_keeps_downstream_values_that_are_still_valid(small):
    assert small.select(D, "ratio", "4:3") == F


def test_select_ratio_falls_back_for_axes_the_new_ratio_does_not_offer(small):
    # 4:3 has no 4K, no 120fps and no Wide lens in this catalog, so resolution,
    # fps and lens all fall back to their first valid values. Zoom "none" is kept.
    assert small.select(B, "ratio", "4:3") == F


def test_select_resolution_recascades_fps(small):
    # D is 2.7K 60fps; 4K only offers 120fps, so fps must change.
    assert small.select(D, "resolution", "4K") == B


def test_select_lens_recascades_zoom(small):
    # A is 4K 120 Wide 1.4x. Narrow does not allow zoom, so zoom becomes none.
    assert small.select(A, "lens", "Narrow") == C


def test_select_zoom_keeps_other_axes(small):
    assert small.select(B, "zoom", "1.4x") == A


def test_select_rejects_a_disabled_value(small):
    with pytest.raises(ValueError):
        small.select(D, "fps", 120)  # 2.7K 16:9 does not offer 120fps in this catalog


def test_select_rejects_zoom_the_lens_does_not_allow(small):
    with pytest.raises(ValueError):
        small.select(B, "zoom", "2x")


# --- resolve: nearest valid mode ------------------------------------------


def test_resolve_keeps_a_valid_mode(small):
    assert small.resolve(D) == D


def test_resolve_repairs_an_unknown_ratio(small):
    assert small.resolve(VideoMode("1:1", "2.7K", 60, "Wide", NO_ZOOM)) == D


def test_resolve_always_returns_a_catalog_mode(small):
    assert small.is_valid(small.resolve(VideoMode("4:3", "4K", 120, "Narrow", "1.4x")))


# --- HERO10 catalog ---------------------------------------------------------


@pytest.fixture(scope="module")
def hero() -> ModeCatalog:
    return t.build_catalog()


def test_hero_default_is_the_requested_starting_mode(hero):
    assert hero.default.label == "16:9 2.7K 60fps Wide"
    assert hero.is_valid(hero.default)


def test_hero_mode_count_matches_table(hero):
    expected = 0
    for row in t.ROWS:
        expected += len(row.fps) * len(row.lenses) + len(row.fps) * len(row.zoom_lenses)
    assert len(hero.modes) == expected


def test_hero_resolution_after_ratio_16_9_includes_5_3k(hero):
    assert enabled(hero.options("resolution", {"ratio": "16:9"})) == {"5.3K", "4K", "2.7K"}


def test_hero_resolution_after_ratio_4_3_excludes_5_3k(hero):
    assert enabled(hero.options("resolution", {"ratio": "4:3"})) == {"4K", "2.7K"}


def test_hero_superview_is_disabled_in_4_3(hero):
    options = hero.options("lens", {"ratio": "4:3", "resolution": "4K", "fps": 60})
    assert "SuperView" not in enabled(options)


def test_hero_linear_does_not_get_1_4x_on_4k_120(hero):
    linear = {"ratio": "16:9", "resolution": "4K", "fps": 120, "lens": "Linear"}
    assert "1.4x" not in enabled(hero.options("zoom", linear))


def test_hero_linear_hl_gets_1_4x_on_4k_120(hero):
    linear_hl = {"ratio": "16:9", "resolution": "4K", "fps": 120, "lens": "Linear+HL"}
    assert "1.4x" in enabled(hero.options("zoom", linear_hl))


def test_hero_narrow_never_offers_zoom(hero):
    for mode in hero.modes:
        if mode.lens == "Narrow":
            assert mode.zoom == NO_ZOOM


def test_hero_every_mode_is_reachable_by_selecting_axes_in_order(hero):
    for target in hero.modes:
        state = hero.default
        for axis in AXES:
            state = hero.select(state, axis, getattr(target, axis))
        assert state == target, target.label


def test_hero_resolve_is_identity_on_valid_modes(hero):
    for mode in hero.modes:
        assert hero.resolve(mode) == mode
