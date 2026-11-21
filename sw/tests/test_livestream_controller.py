"""Tests for GoProController's mode handling, against a fake WiredGoPro (no hardware)."""

from types import SimpleNamespace

import pytest
from open_gopro.models.constants import SettingId, StatusId, constants

from sw import livestream as l
from sw.camera import hero10_table as table
from sw.camera.modes import NO_ZOOM, VideoMode


class Resp:
    def __init__(self, ok: bool = True) -> None:
        self.ok = ok
        self.status = "ErrorCode.SUCCESS" if ok else "ErrorCode.INVALID_PARAMETER"
        self.identifier = "fake"


class FakeWired:
    """Records the order of commands. Keeps camera state updated by settings."""

    instances: list["FakeWired"] = []
    reject: str | None = None  # setting name to refuse, e.g. "resolution"

    def __init__(self, serial) -> None:
        self.identifier = "GP-FAKE"
        self.log: list[object] = []
        self.state: dict = {}
        FakeWired.instances.append(self)
        self.http_setting = SimpleNamespace(
            video_resolution=_Setting(self, "resolution", SettingId.VIDEO_RESOLUTION),
            frames_per_second=_Setting(self, "fps", SettingId.FRAMES_PER_SECOND),
            video_lens=_Setting(self, "lens", SettingId.VIDEO_LENS),
        )
        self.http_command = _Command(self)

    async def open(self, timeout=None, retries=None) -> None:
        self.log.append("open")

    async def close(self) -> None:
        self.log.append("close")


class _Setting:
    def __init__(self, gopro: FakeWired, name: str, key: SettingId) -> None:
        self._gopro, self._name, self._key = gopro, name, key

    async def set(self, value):
        self._gopro.log.append(self._name)
        if self._name == FakeWired.reject:
            return Resp(False)
        self._gopro.state[self._key] = value
        if self._name == "lens":
            available = value in (table.LENS_ENUM["Wide"], table.LENS_ENUM["Linear"], table.LENS_ENUM["Linear+HL"])
            self._gopro.state[StatusId.ZOOM_AVAILABLE] = available
            if not available:
                self._gopro.state[StatusId.ZOOM_LEVEL] = 0
        return Resp(True)


class _Command:
    def __init__(self, gopro: FakeWired) -> None:
        self._gopro = gopro

    async def load_preset_group(self, group):
        self._gopro.log.append("load_preset")
        return Resp(True)

    async def set_preview_stream(self, mode, port=None):
        self._gopro.log.append("preview_on" if mode == constants.Toggle.ENABLE else "preview_off")
        return Resp(True)

    async def set_keep_alive(self):
        return Resp(True)

    async def set_digital_zoom(self, percent):
        self._gopro.log.append("zoom")
        self._gopro.state[StatusId.ZOOM_LEVEL] = percent
        return Resp(True)

    async def get_camera_state(self):
        return SimpleNamespace(data=dict(self._gopro.state))


@pytest.fixture
def controller(tmp_path, monkeypatch):
    FakeWired.instances = []
    FakeWired.reject = None
    monkeypatch.setattr(l, "WiredGoPro", FakeWired)
    monkeypatch.setitem(table.ZOOM_PERCENT, "1.4x", 42)
    ctl = l.GoProController(tmp_path, port=8554, serial=None)
    yield ctl
    ctl.submit(ctl.shutdown()).result(timeout=5)


def _commands(gopro: FakeWired) -> list[object]:
    return [c for c in gopro.log if c != "open"]


def test_catalog_is_the_hero10_catalog(controller):
    assert controller.catalog.default == table.DEFAULT_MODE
    assert controller.catalog.is_valid(table.DEFAULT_MODE)


def test_connect_stops_preview_then_applies_mode_then_starts_preview(controller):
    mode = VideoMode("16:9", "4K", 120, "Wide", NO_ZOOM)
    serial = controller.submit(controller.connect(mode)).result(timeout=5)
    assert serial == "GP-FAKE"
    gopro = FakeWired.instances[-1]
    assert _commands(gopro) == [
        "preview_off",
        "load_preset",
        "resolution",
        "fps",
        "lens",
        "zoom",
        "preview_on",
    ]


def test_set_mode_restarts_preview_around_the_change(controller):
    controller.submit(controller.connect(table.DEFAULT_MODE)).result(timeout=5)
    gopro = FakeWired.instances[-1]
    gopro.log.clear()
    controller.submit(controller.set_mode(VideoMode("16:9", "2.7K", 120, "Wide", "1.4x"))).result(timeout=5)
    assert _commands(gopro) == ["preview_off", "resolution", "fps", "lens", "zoom", "preview_on"]


def test_set_mode_restarts_preview_even_when_the_camera_refuses(controller):
    controller.submit(controller.connect(table.DEFAULT_MODE)).result(timeout=5)
    gopro = FakeWired.instances[-1]
    gopro.log.clear()
    FakeWired.reject = "resolution"
    with pytest.raises(RuntimeError):
        controller.submit(controller.set_mode(VideoMode("16:9", "4K", 120, "Wide", NO_ZOOM))).result(timeout=5)
    assert _commands(gopro) == ["preview_off", "resolution", "preview_on"]


def test_set_mode_before_connect_is_an_error(controller):
    with pytest.raises(AssertionError):
        controller.submit(controller.set_mode(table.DEFAULT_MODE)).result(timeout=5)


def test_connect_refuses_an_uncalibrated_zoom_mode(controller, monkeypatch):
    monkeypatch.setitem(table.ZOOM_PERCENT, "1.4x", None)
    with pytest.raises(RuntimeError, match="not calibrated"):
        controller.submit(controller.connect(VideoMode("16:9", "2.7K", 60, "Wide", "1.4x"))).result(timeout=5)
