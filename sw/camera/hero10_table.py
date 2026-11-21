"""GoPro HERO10 Black video modes, transcribed from GoPro's official table.

Source: "HERO10 Black: Video Settings And Resolutions"
https://community.gopro.com/s/article/HERO10-Black-Video-Settings-and-Resolutions

Only the rows requested by the user are listed. Each row maps to the camera
enums used by the Open GoPro SDK. Digital zoom is continuous (0-100%) on the
lenses that support it. The camera reports zoom availability and level, so
the adapter can check them. This module is data only: it imports the SDK
enums but never talks to a camera.
"""

from __future__ import annotations

from dataclasses import dataclass

from open_gopro.models.constants.settings import (
    FramesPerSecond,
    VideoLens,
    VideoResolution,
)

from sw.camera.modes import NO_ZOOM, ModeCatalog, VideoMode

RATIO_16_9 = "16:9"
RATIO_4_3 = "4:3"

ZOOM_MAX_PERCENT = 100  # the camera accepts 0-100 with lenses that support zoom

LENS_SUPERVIEW = "SuperView"
LENS_WIDE = "Wide"
LENS_LINEAR = "Linear"
LENS_LINEAR_HL = "Linear+HL"
LENS_NARROW = "Narrow"


@dataclass(frozen=True)
class TableRow:
    """One line of GoPro's table: a resolution at one or more frame rates.

    Attributes:
        ratio: "16:9" or "4:3".
        resolution: Display name, e.g. "2.7K".
        fps: Frame rates offered by this row, e.g. (120,).
        lenses: Digital lens options offered by this row.
        zoom_lenses: Subset of `lenses` on which digital zoom is allowed.
    """

    ratio: str
    resolution: str
    fps: tuple[int, ...]
    lenses: tuple[str, ...]
    zoom_lenses: tuple[str, ...]


_ALL_NO_SV = (LENS_WIDE, LENS_LINEAR, LENS_LINEAR_HL, LENS_NARROW)
_ALL_SV = (LENS_SUPERVIEW,) + _ALL_NO_SV

ROWS: tuple[TableRow, ...] = (
    # 16:9
    TableRow(RATIO_16_9, "5.3K", (60,), _ALL_SV, (LENS_WIDE, LENS_LINEAR, LENS_LINEAR_HL)),
    TableRow(RATIO_16_9, "4K", (120,), _ALL_NO_SV, (LENS_WIDE, LENS_LINEAR_HL)),
    TableRow(RATIO_16_9, "2.7K", (240,), _ALL_NO_SV, (LENS_WIDE, LENS_LINEAR, LENS_LINEAR_HL)),
    TableRow(RATIO_16_9, "2.7K", (120,), _ALL_SV, (LENS_WIDE, LENS_LINEAR, LENS_LINEAR_HL)),
    TableRow(RATIO_16_9, "2.7K", (60,), _ALL_SV, (LENS_WIDE, LENS_LINEAR, LENS_LINEAR_HL)),
    # 4:3 (no SuperView)
    TableRow(RATIO_4_3, "4K", (60,), _ALL_NO_SV, (LENS_WIDE, LENS_LINEAR, LENS_LINEAR_HL)),
    TableRow(RATIO_4_3, "2.7K", (120,), _ALL_NO_SV, (LENS_WIDE, LENS_LINEAR, LENS_LINEAR_HL)),
)

# Camera enum for each (ratio, resolution). The aspect ratio is encoded in
# the resolution enum on the HERO10, so these keys are unique.
RESOLUTION_ENUM: dict[tuple[str, str], VideoResolution] = {
    (RATIO_16_9, "5.3K"): VideoResolution.NUM_5_3K,
    (RATIO_16_9, "4K"): VideoResolution.NUM_4K,
    (RATIO_16_9, "2.7K"): VideoResolution.NUM_2_7K,
    (RATIO_4_3, "4K"): VideoResolution.NUM_4K_4_3,
    (RATIO_4_3, "2.7K"): VideoResolution.NUM_2_7K_4_3,
}

FPS_ENUM: dict[int, FramesPerSecond] = {
    240: FramesPerSecond.NUM_240_0,
    120: FramesPerSecond.NUM_120_0,
    60: FramesPerSecond.NUM_60_0,
}

LENS_ENUM: dict[str, VideoLens] = {
    LENS_SUPERVIEW: VideoLens.SUPERVIEW,
    LENS_WIDE: VideoLens.WIDE,
    LENS_LINEAR: VideoLens.LINEAR,
    LENS_LINEAR_HL: VideoLens.LINEAR_HORIZON_LEVELING,
    LENS_NARROW: VideoLens.NARROW,
}

DEFAULT_MODE = VideoMode(RATIO_16_9, "2.7K", 60, LENS_WIDE, NO_ZOOM)


def zoom_supported(mode: VideoMode) -> bool:
    """True if the table allows digital zoom with this mode's lens, frame rate and resolution."""
    return any(
        row.ratio == mode.ratio
        and row.resolution == mode.resolution
        and mode.fps in row.fps
        and mode.lens in row.zoom_lenses
        for row in ROWS
    )


def zoom_max(mode: VideoMode) -> int:
    """Largest digital zoom percentage for a mode (0 where the lens has no zoom)."""
    return ZOOM_MAX_PERCENT if zoom_supported(mode) else NO_ZOOM


def build_catalog() -> ModeCatalog:
    """Expand ROWS into every valid HERO10 video mode (zoom 0); zoom is continuous on top."""
    modes: list[VideoMode] = []
    for row in ROWS:
        for fps in row.fps:
            for lens in row.lenses:
                modes.append(VideoMode(row.ratio, row.resolution, fps, lens, NO_ZOOM))
    return ModeCatalog(modes, DEFAULT_MODE, zoom_max=zoom_max)
