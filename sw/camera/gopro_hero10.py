"""HERO10 adapter: applies a VideoMode to a WiredGoPro through the Open GoPro SDK."""

from __future__ import annotations

from collections.abc import Awaitable, Mapping
from typing import Any

from open_gopro.models.constants import SettingId, StatusId

from sw.camera import hero10_table as table
from sw.camera.modes import NO_ZOOM, ModeCatalog, VideoMode


def camera_values(mode: VideoMode) -> tuple[Any, Any, Any]:
    """SDK enums (resolution, fps, lens) for a mode. Zoom is a percentage, not an enum."""
    return (
        table.RESOLUTION_ENUM[(mode.ratio, mode.resolution)],
        table.FPS_ENUM[mode.fps],
        table.LENS_ENUM[mode.lens],
    )


def state_mismatches(mode: VideoMode, state: Mapping[Any, Any]) -> tuple[str, ...]:
    """Describe how the camera's reported state differs from `mode`.

    Only resolution, fps and lens can be read back. Zoom has no known state key.
    A missing setting counts as a mismatch.
    """
    resolution, fps, lens = camera_values(mode)
    checks = (
        (SettingId.VIDEO_RESOLUTION, resolution, "resolution"),
        (SettingId.FRAMES_PER_SECOND, fps, "fps"),
        (SettingId.VIDEO_LENS, lens, "lens"),
    )
    problems: list[str] = []
    for setting, wanted, name in checks:
        if setting not in state:
            problems.append(f"{name}: not reported")
        elif state[setting] != wanted:
            problems.append(f"{name}: wanted {wanted.name}, camera reports {state[setting]}")
    return tuple(problems)


def zoom_mismatches(mode: VideoMode, state: Mapping[Any, Any]) -> tuple[str, ...]:
    """Describe how the camera's zoom status differs from `mode`.

    The camera reports whether digital zoom is available for the current lens, and the
    zoom level in percent. Switching to a lens without zoom resets the level to 0.
    """
    expected_available = table.zoom_supported(mode)
    expected_level = table.ZOOM_PERCENT[mode.zoom]
    problems: list[str] = []
    if StatusId.ZOOM_AVAILABLE not in state:
        problems.append("zoom available: not reported")
    elif state[StatusId.ZOOM_AVAILABLE] != expected_available:
        problems.append(f"zoom available: wanted {expected_available}, camera reports {state[StatusId.ZOOM_AVAILABLE]}")
    if StatusId.ZOOM_LEVEL not in state:
        problems.append("zoom level: not reported")
    elif state[StatusId.ZOOM_LEVEL] != expected_level:
        problems.append(f"zoom level: wanted {expected_level}, camera reports {state[StatusId.ZOOM_LEVEL]}")
    return tuple(problems)


async def _send(command: Awaitable[Any], what: str) -> None:
    response = await command
    if not response.ok:
        raise RuntimeError(f"camera rejected {what}: {response.status}")


class HeroBackend:
    """Applies HERO10 video modes to a connected WiredGoPro.

    Settings are sent in order: resolution, fps, lens, then zoom. Changing the
    resolution can change which fps and lens values are valid, so zoom goes
    last and is always sent, including 0 for "no zoom". After the last command
    the camera state is read back and checked.

    Args:
        gopro: A connected WiredGoPro (or any object with the same attributes).
        catalog: Mode catalog. Defaults to the HERO10 table.
    """

    def __init__(self, gopro: Any, catalog: ModeCatalog | None = None) -> None:
        self._gopro = gopro
        self._catalog = catalog if catalog is not None else table.build_catalog()

    @property
    def catalog(self) -> ModeCatalog:
        return self._catalog

    async def apply_mode(self, mode: VideoMode) -> None:
        if not self._catalog.is_valid(mode):
            raise ValueError(f"{mode.label} is not a valid HERO10 mode")
        # The camera rejects digital zoom commands with lenses that have no zoom (SuperView, Narrow),
        # even for 0, so the command is only sent where the table allows zoom.
        send_zoom = mode.zoom != NO_ZOOM or table.zoom_supported(mode)
        percent = table.ZOOM_PERCENT[mode.zoom]
        if send_zoom and percent is None:
            raise RuntimeError(
                f"digital zoom {mode.zoom} is not calibrated yet; "
                "run sw/camera_probe.py --calibrate-zoom PERCENT and record it in hero10_table.ZOOM_PERCENT"
            )
        resolution, fps, lens = camera_values(mode)
        http = self._gopro.http_setting
        await _send(http.video_resolution.set(resolution), "video resolution")
        await _send(http.frames_per_second.set(fps), "frames per second")
        await _send(http.video_lens.set(lens), "video lens")
        if send_zoom:
            await _send(self._gopro.http_command.set_digital_zoom(percent=percent), "digital zoom")

        state = (await self._gopro.http_command.get_camera_state()).data
        problems = state_mismatches(mode, state) + zoom_mismatches(mode, state)
        if problems:
            raise RuntimeError(f"camera did not keep {mode.label}: " + "; ".join(problems))
