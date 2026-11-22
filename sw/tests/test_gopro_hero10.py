"""Tests for the HERO10 adapter, using a fake camera (no hardware)."""

import asyncio
import functools
from types import SimpleNamespace

import pytest
from open_gopro.models.constants import SettingId, StatusId

from sw.camera import gopro_hero10 as hero
from sw.camera import hero10_table as table
from sw.camera.base import CameraBackend
from sw.camera.modes import VideoMode


class Resp:
    def __init__(self, ok: bool = True) -> None:
        self.ok = ok
        self.status = "ErrorCode.SUCCESS" if ok else "ErrorCode.INVALID_PARAMETER"


class FakeGoPro:
    """Records every command and keeps a camera state that the commands update.

    Like the camera, zoom is available only on zoom lenses, zoom is sent as a
    percentage, and switching to a lens without zoom resets the level to 0.

    Args:
        reject: name of a setting ("resolution", "fps", "lens") to refuse.
        override: maps a setting name to a value to store instead of the one
            requested, which simulates the camera silently changing it.
    """

    KEYS = {
        "resolution": SettingId.VIDEO_RESOLUTION,
        "fps": SettingId.FRAMES_PER_SECOND,
        "lens": SettingId.VIDEO_LENS,
    }
    ZOOM_LENSES = (
        table.LENS_ENUM["Wide"],
        table.LENS_ENUM["Linear"],
        table.LENS_ENUM["Linear+HL"],
    )

    def __init__(self, reject: str | None = None, override: dict | None = None) -> None:
        self.log: list[tuple[str, object]] = []
        self.state: dict = {}
        self._reject = reject
        self._override = override or {}
        self.http_setting = SimpleNamespace(
            video_resolution=_Setting(self, "resolution"),
            frames_per_second=_Setting(self, "fps"),
            video_lens=_Setting(self, "lens"),
        )
        self.http_command = _Command(self)


class _Setting:
    def __init__(self, gopro: FakeGoPro, name: str) -> None:
        self._gopro = gopro
        self._name = name

    async def set(self, value):
        gopro = self._gopro
        gopro.log.append((self._name, value))
        if self._name == gopro._reject:
            return Resp(False)
        stored = gopro._override.get(self._name, value)
        gopro.state[gopro.KEYS[self._name]] = stored
        if self._name == "lens":
            available = stored in gopro.ZOOM_LENSES
            gopro.state[StatusId.ZOOM_AVAILABLE] = available
            if not available:
                gopro.state[StatusId.ZOOM_LEVEL] = 0
        return Resp(True)


class _Command:
    def __init__(self, gopro: FakeGoPro) -> None:
        self._gopro = gopro

    async def set_digital_zoom(self, *, percent: int):
        self._gopro.log.append(("zoom", percent))
        if percent > 100:
            return Resp(False)
        self._gopro.state[StatusId.ZOOM_LEVEL] = percent
        return Resp(True)

    async def get_camera_state(self):
        return SimpleNamespace(data=dict(self._gopro.state))


def sync(test):
    """Run an async test on its own event loop (no pytest-asyncio needed)."""

    @functools.wraps(test)
    def wrapper(*args, **kwargs):
        return asyncio.run(test(*args, **kwargs))

    return wrapper


def _mode(zoom=0):
    return VideoMode("16:9", "4K", 120, "Wide", zoom)


# --- apply order and success -----------------------------------------------


@sync
async def test_settings_are_sent_in_order_with_zoom_last():
    gopro = FakeGoPro()
    await hero.HeroBackend(gopro).apply_mode(_mode(42))
    assert gopro.log == [
        ("resolution", table.RESOLUTION_ENUM[("16:9", "4K")]),
        ("fps", table.FPS_ENUM[120]),
        ("lens", table.LENS_ENUM["Wide"]),
        ("zoom", 42),
    ]


@sync
async def test_no_zoom_is_sent_as_zero_on_a_zoom_lens():
    gopro = FakeGoPro()
    await hero.HeroBackend(gopro).apply_mode(_mode(0))
    assert gopro.log[-1] == ("zoom", 0)
    assert len(gopro.log) == 4


@sync
async def test_full_range_is_accepted_and_read_back():
    for percent in (1, 50, 100):
        gopro = FakeGoPro()
        await hero.HeroBackend(gopro).apply_mode(_mode(percent))
        assert gopro.log[-1] == ("zoom", percent)


@sync
async def test_lens_without_zoom_sends_no_zoom_command():
    gopro = FakeGoPro()
    await hero.HeroBackend(gopro).apply_mode(
        VideoMode("16:9", "2.7K", 120, "SuperView", 0)
    )
    assert [name for name, _ in gopro.log] == ["resolution", "fps", "lens"]


# --- refusals happen before anything is sent --------------------------------


@sync
async def test_invalid_mode_is_refused_without_sending_anything():
    gopro = FakeGoPro()
    with pytest.raises(ValueError):
        await hero.HeroBackend(gopro).apply_mode(
            VideoMode("4:3", "5.3K", 60, "Wide", 0)
        )
    assert gopro.log == []


@sync
async def test_zoom_on_a_lens_without_zoom_is_refused_without_sending_anything():
    gopro = FakeGoPro()
    with pytest.raises(ValueError):
        await hero.HeroBackend(gopro).apply_mode(
            VideoMode("16:9", "2.7K", 120, "SuperView", 30)
        )
    assert gopro.log == []


@sync
async def test_zoom_above_100_is_refused_without_sending_anything():
    gopro = FakeGoPro()
    with pytest.raises(ValueError):
        await hero.HeroBackend(gopro).apply_mode(_mode(101))
    assert gopro.log == []


# --- camera failures --------------------------------------------------------


@sync
async def test_rejected_setting_stops_the_sequence():
    gopro = FakeGoPro(reject="resolution")
    with pytest.raises(RuntimeError, match="video resolution"):
        await hero.HeroBackend(gopro).apply_mode(_mode())
    assert gopro.log == [("resolution", table.RESOLUTION_ENUM[("16:9", "4K")])]


@sync
async def test_readback_detects_a_setting_the_camera_changed():
    gopro = FakeGoPro(override={"fps": table.FPS_ENUM[60]})
    with pytest.raises(RuntimeError, match="fps"):
        await hero.HeroBackend(gopro).apply_mode(_mode())


@sync
async def test_readback_detects_a_missing_setting():
    gopro = FakeGoPro()
    original = gopro.http_command.get_camera_state

    async def without_lens():
        data = (await original()).data
        del data[SettingId.VIDEO_LENS]
        return SimpleNamespace(data=data)

    gopro.http_command.get_camera_state = without_lens
    with pytest.raises(RuntimeError, match="lens: not reported"):
        await hero.HeroBackend(gopro).apply_mode(_mode())


@sync
async def test_readback_detects_a_zoom_level_the_camera_did_not_keep():
    gopro = FakeGoPro()
    original = gopro.http_command.get_camera_state

    async def zoom_reset():
        data = (await original()).data
        data[StatusId.ZOOM_LEVEL] = 0
        return SimpleNamespace(data=data)

    gopro.http_command.get_camera_state = zoom_reset
    with pytest.raises(RuntimeError, match="zoom level"):
        await hero.HeroBackend(gopro).apply_mode(_mode(42))


# --- mapping, zoom status and protocol -------------------------------------


def test_every_catalog_mode_maps_to_camera_values():
    for mode in table.build_catalog().modes:
        assert None not in hero.camera_values(mode)


def test_camera_values_identify_each_catalog_mode():
    modes = table.build_catalog().modes
    assert len({hero.camera_values(m) for m in modes}) == len(modes)


def test_state_mismatches_is_empty_for_a_matching_state():
    state = {
        SettingId.VIDEO_RESOLUTION: table.RESOLUTION_ENUM[("16:9", "4K")],
        SettingId.FRAMES_PER_SECOND: table.FPS_ENUM[120],
        SettingId.VIDEO_LENS: table.LENS_ENUM["Wide"],
    }
    assert hero.state_mismatches(_mode(), state) == ()


def test_zoom_mismatches_are_empty_for_a_matching_zoom_status():
    state = {StatusId.ZOOM_AVAILABLE: True, StatusId.ZOOM_LEVEL: 42}
    assert hero.zoom_mismatches(_mode(42), state) == ()


def test_zoom_mismatches_report_wrong_level_and_availability():
    state = {StatusId.ZOOM_AVAILABLE: False, StatusId.ZOOM_LEVEL: 10}
    problems = hero.zoom_mismatches(_mode(42), state)
    assert len(problems) == 2
    assert problems[0].startswith("zoom available") and problems[1].startswith(
        "zoom level"
    )


def test_backend_satisfies_the_camera_backend_protocol():
    assert isinstance(hero.HeroBackend(FakeGoPro()), CameraBackend)
