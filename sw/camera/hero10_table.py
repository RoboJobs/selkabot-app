"""GoPro HERO10 Black video modes, transcribed from GoPro's official table.

Source: "HERO10 Black: Video Settings And Resolutions"
https://community.gopro.com/s/article/HERO10-Black-Video-Settings-and-Resolutions

Only the rows requested by the user are listed. Each row maps to the camera
enums used by the Open GoPro SDK. This module is data only: it imports the
SDK enums but never talks to a camera.
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

ZOOM_NONE = NO_ZOOM
ZOOM_1_4X = "1.4x"
ZOOM_2X = "2x"

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
        fps: Frame rates offered by this row, e.g. (120, 100).
        lenses: Digital lens options offered by this row.
        zoom: Digital zoom label available on `zoom_lenses`, or None.
        zoom_lenses: Subset of `lenses` on which zoom is allowed.
    """

    ratio: str
    resolution: str
    fps: tuple[int, ...]
    lenses: tuple[str, ...]
    zoom: str | None
    zoom_lenses: tuple[str, ...]


_ALL_NO_SV = (LENS_WIDE, LENS_LINEAR, LENS_LINEAR_HL, LENS_NARROW)
_ALL_SV = (LENS_SUPERVIEW,) + _ALL_NO_SV

ROWS: tuple[TableRow, ...] = (
    # 16:9
    TableRow(RATIO_16_9, "5.3K", (60, 50), _ALL_SV, ZOOM_1_4X, (LENS_WIDE, LENS_LINEAR, LENS_LINEAR_HL)),
    TableRow(RATIO_16_9, "4K", (120, 100), _ALL_NO_SV, ZOOM_1_4X, (LENS_WIDE, LENS_LINEAR_HL)),
    TableRow(RATIO_16_9, "2.7K", (240, 200), _ALL_NO_SV, ZOOM_1_4X, (LENS_WIDE, LENS_LINEAR, LENS_LINEAR_HL)),
    TableRow(RATIO_16_9, "2.7K", (120, 100), _ALL_SV, ZOOM_1_4X, (LENS_WIDE, LENS_LINEAR, LENS_LINEAR_HL)),
    TableRow(RATIO_16_9, "2.7K", (60, 50), _ALL_SV, ZOOM_1_4X, (LENS_WIDE, LENS_LINEAR, LENS_LINEAR_HL)),
    # 4:3 (no SuperView)
    TableRow(RATIO_4_3, "4K", (60, 50), _ALL_NO_SV, ZOOM_2X, (LENS_WIDE, LENS_LINEAR, LENS_LINEAR_HL)),
    TableRow(RATIO_4_3, "2.7K", (120, 100), _ALL_NO_SV, ZOOM_2X, (LENS_WIDE, LENS_LINEAR, LENS_LINEAR_HL)),
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
    200: FramesPerSecond.NUM_200_0,
    120: FramesPerSecond.NUM_120_0,
    100: FramesPerSecond.NUM_100_0,
    60: FramesPerSecond.NUM_60_0,
    50: FramesPerSecond.NUM_50_0,
}

LENS_ENUM: dict[str, VideoLens] = {
    LENS_SUPERVIEW: VideoLens.SUPERVIEW,
    LENS_WIDE: VideoLens.WIDE,
    LENS_LINEAR: VideoLens.LINEAR,
    LENS_LINEAR_HL: VideoLens.LINEAR_HORIZON_LEVELING,
    LENS_NARROW: VideoLens.NARROW,
}

# Digital zoom is sent as a percentage to `gopro/camera/digital_zoom`.
# 0 means no zoom. The values for 1.4x and 2x are NOT known yet; they must be
# calibrated on the camera with sw/camera_probe.py before zoom can be applied.
ZOOM_PERCENT: dict[str, int | None] = {
    ZOOM_NONE: 0,
    ZOOM_1_4X: None,
    ZOOM_2X: None,
}


# Values present in the table that the camera did not accept in testing (PAL frame rates
# on this unit). They stay in the catalog and are disabled in the window until confirmed.
UNCONFIRMED_VALUES: dict[str, frozenset] = {"fps": frozenset({50, 100, 200})}

DEFAULT_MODE = VideoMode(RATIO_16_9, "2.7K", 60, LENS_WIDE, ZOOM_NONE)


def build_catalog() -> ModeCatalog:
    """Expand ROWS into every valid HERO10 video mode.

    A row yields one mode per (fps, lens) pair, and one more mode per
    (fps, zoom lens) pair with the row's zoom factor applied.
    """
    modes: list[VideoMode] = []
    for row in ROWS:
        for fps in row.fps:
            for lens in row.lenses:
                modes.append(VideoMode(row.ratio, row.resolution, fps, lens, ZOOM_NONE))
            for lens in row.zoom_lenses:
                modes.append(VideoMode(row.ratio, row.resolution, fps, lens, row.zoom))
    return ModeCatalog(modes, DEFAULT_MODE)
