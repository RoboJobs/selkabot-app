"""Tests for the HERO10 adapter, using a fake camera (no hardware)."""

import asyncio
import functools
from types import SimpleNamespace

import pytest
from open_gopro.models.constants import SettingId

from sw.camera import gopro_hero10 as hero
from sw.camera import hero10_table as table
from sw.camera.base import CameraBackend
from sw.camera.modes import NO_ZOOM, VideoMode


class Resp:
    def __init__(self, ok: bool = True) -> None:
        self.ok = ok
        self.status = "ErrorCode.SUCCESS" if ok else "ErrorCode.INVALID_PARAMETER"


class FakeGoPro:
    """Records every command and keeps a camera state that the commands update.

    Args:
        reject: name of a setting ("resolution", "fps", "lens") to refuse.
        override: maps a setting name to a value to store instead of the one
            requested, which simulates the camera silently changing it.
    """

    KEYS = {"resolution": SettingId.VIDEO_RESOLUTION, "fps": SettingId.FRAMES_PER_SECOND, "lens": SettingId.VIDEO_LENS}

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
        return Resp(True)


class _Command:
    def __init__(self, gopro: FakeGoPro) -> None:
        self._gopro = gopro

    async def set_digital_zoom(self, *, percent: int):
        self._gopro.log.append(("zoom", percent))
        return Resp(True)

    async def get_camera_state(self):
        return SimpleNamespace(data=dict(self._gopro.state))


def sync(test):
    """Run an async test on its own event loop (no pytest-asyncio needed)."""

    @functools.wraps(test)
    def wrapper(*args, **kwargs):
        return asyncio.run(test(*args, **kwargs))

    return wrapper


@pytest.fixture
def calibrated(monkeypatch):
    monkeypatch.setitem(table.ZOOM_PERCENT, "1.4x", 42)
    monkeypatch.setitem(table.ZOOM_PERCENT, "2x", 77)
    return monkeypatch


def _mode(zoom=NO_ZOOM):
    return VideoMode("16:9", "4K", 120, "Wide", zoom)


# --- apply order and success -----------------------------------------------


@sync
async def test_settings_are_sent_in_order_with_zoom_last(calibrated):
    gopro = FakeGoPro()
    await hero.HeroBackend(gopro).apply_mode(_mode("1.4x"))
    assert gopro.log == [
        ("resolution", table.RESOLUTION_ENUM[("16:9", "4K")]),
        ("fps", table.FPS_ENUM[120]),
        ("lens", table.LENS_ENUM["Wide"]),
        ("zoom", 42),
    ]


@sync
async def test_no_zoom_is_still_sent_as_zero_and_last(calibrated):
    gopro = FakeGoPro()
    await hero.HeroBackend(gopro).apply_mode(_mode())
    assert gopro.log[-1] == ("zoom", 0)
    assert len(gopro.log) == 4


@sync
async def test_2x_uses_its_own_calibrated_percent(calibrated):
    gopro = FakeGoPro()
    mode = VideoMode("4:3", "4K", 60, "Wide", "2x")
    await hero.HeroBackend(gopro).apply_mode(mode)
    assert gopro.log[-1] == ("zoom", 77)


# --- refusals happen before anything is sent --------------------------------


@sync
async def test_invalid_mode_is_refused_without_sending_anything(calibrated):
    gopro = FakeGoPro()
    with pytest.raises(ValueError):
        await hero.HeroBackend(gopro).apply_mode(VideoMode("4:3", "5.3K", 60, "Wide", NO_ZOOM))
    assert gopro.log == []


@sync
async def test_uncalibrated_zoom_is_refused_without_sending_anything():
    gopro = FakeGoPro()
    with pytest.raises(RuntimeError, match="not calibrated"):
        await hero.HeroBackend(gopro).apply_mode(_mode("1.4x"))
    assert gopro.log == []


# --- camera failures --------------------------------------------------------


@sync
async def test_rejected_setting_stops_the_sequence(calibrated):
    gopro = FakeGoPro(reject="resolution")
    with pytest.raises(RuntimeError, match="video resolution"):
        await hero.HeroBackend(gopro).apply_mode(_mode())
    assert gopro.log == [("resolution", table.RESOLUTION_ENUM[("16:9", "4K")])]


@sync
async def test_readback_detects_a_setting_the_camera_changed(calibrated):
    gopro = FakeGoPro(override={"fps": table.FPS_ENUM[60]})
    with pytest.raises(RuntimeError, match="fps"):
        await hero.HeroBackend(gopro).apply_mode(_mode())


@sync
async def test_readback_detects_a_missing_setting(calibrated):
    gopro = FakeGoPro()
    original = gopro.http_command.get_camera_state

    async def without_lens():
        data = (await original()).data
        del data[SettingId.VIDEO_LENS]
        return SimpleNamespace(data=data)

    gopro.http_command.get_camera_state = without_lens
    with pytest.raises(RuntimeError, match="lens: not reported"):
        await hero.HeroBackend(gopro).apply_mode(_mode())


# --- mapping and protocol ---------------------------------------------------


def test_every_catalog_mode_maps_to_camera_values():
    for mode in table.build_catalog().modes:
        resolution, fps, lens = hero.camera_values(mode)
        assert resolution is not None and fps is not None and lens is not None


def test_camera_values_plus_zoom_identify_each_mode():
    modes = table.build_catalog().modes
    keys = [(hero.camera_values(m), m.zoom) for m in modes]
    assert len(set(keys)) == len(modes)


def test_state_mismatches_is_empty_for_a_matching_state():
    mode = _mode()
    state = {
        SettingId.VIDEO_RESOLUTION: table.RESOLUTION_ENUM[("16:9", "4K")],
        SettingId.FRAMES_PER_SECOND: table.FPS_ENUM[120],
        SettingId.VIDEO_LENS: table.LENS_ENUM["Wide"],
    }
    assert hero.state_mismatches(mode, state) == ()


def test_backend_satisfies_the_camera_backend_protocol():
    assert isinstance(hero.HeroBackend(FakeGoPro()), CameraBackend)


@sync
async def test_lens_without_zoom_sends_no_zoom_command(calibrated):
    gopro = FakeGoPro()
    await hero.HeroBackend(gopro).apply_mode(VideoMode("16:9", "2.7K", 120, "SuperView", NO_ZOOM))
    assert [name for name, _ in gopro.log] == ["resolution", "fps", "lens"]


def test_zoom_supported_follows_the_table():
    assert table.zoom_supported(VideoMode("16:9", "2.7K", 120, "Wide", NO_ZOOM))
    assert not table.zoom_supported(VideoMode("16:9", "2.7K", 120, "SuperView", NO_ZOOM))
    assert not table.zoom_supported(VideoMode("16:9", "4K", 120, "Narrow", NO_ZOOM))
    assert not table.zoom_supported(VideoMode("16:9", "4K", 120, "Linear", NO_ZOOM))  # only Wide and Linear+HL
