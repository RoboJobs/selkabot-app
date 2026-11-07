"""Edit a recorded clip entirely with Shotcut's own engine (MLT/melt): watermarks in the
four corners, a fade in from a still image, a fade out to a still image, and two slow-motion
windows — then render the result with `melt`, no Shotcut GUI or ffmpeg involved.

This writes the same MLT XML shape verified by hand in videos/shotcut/proto_watermark_edit.mlt
(built and checked with the shotcut-mcp tools), generalized to any clip, duration and image set:
- Each watermark image gets its own video track per corner (2 corners each for the always-used
  image and the optional extra one), with an `affine` "Size, Position & Rotate" filter placing
  it flush in that corner, and a `brightness` filter animating its opacity in sync with the
  fade-in/fade-out stills (invisible under the stills, fading in/out with the crossfades).
- The fade stills use any image directly (no pre-rendering step): a `qimage` producer with a
  full-frame `affine` filter (fill, no distort) letterboxes/pillarboxes it to the project's
  aspect ratio, verified against both a portrait-ish JPEG and a wide PNG.
- Slow motion uses MLT's native `timewarp` producer (0.3x, pitch-compensated), not ffmpeg.

Usage:
    uv run python -m sw.shotcut_pipeline <input.mp4> <output.mp4> [--image likegym.png]
        [--fadein-image jdc.jpg] [--fadeout-image selkabot.png]
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import subprocess
from dataclasses import dataclass
from pathlib import Path
from xml.sax.saxutils import quoteattr

logger = logging.getLogger(__name__)

# Always-used watermark, plus its two corners (mirrors sw/watermark.py's WATERMARKS).
ALWAYS_WATERMARK = Path(__file__).resolve().parents[1] / "selkabot.png"
ALWAYS_CORNERS = ("bottom-left", "top-right")
# Corners of the optional, user-chosen image (sw/watermark.py's IMAGE_CORNERS).
IMAGE_CORNERS = ("top-left", "bottom-right")
WATERMARK_SIZE_PCT = 10  # Each watermark's bounding box, as a percentage of the frame

# Slow-motion windows, in seconds from the start of the clip. Clamped to the clip's actual
# duration; a window that no longer has positive length after clamping is dropped.
SLOWDOWN_WINDOWS_S = [(8.0, 12.0), (14.0, 16.0)]
SLOWDOWN_SPEED = 0.3

FADE_STILL_SECONDS = 2.0  # Full length of each fade still, before the crossfade eats into it
FADE_TRANSITION_SECONDS = 1.0  # Crossfade length, split evenly between the still and the clip

MELT = os.environ.get("SHOTCUT_MELT_PATH", "melt")

# x%, y% of the top-left corner of each watermark's bounding box (box is WATERMARK_SIZE_PCT wide).
_CORNER_XY = {
    "top-left": (0, 0),
    "top-right": (100 - WATERMARK_SIZE_PCT, 0),
    "bottom-left": (0, 100 - WATERMARK_SIZE_PCT),
    "bottom-right": (100 - WATERMARK_SIZE_PCT, 100 - WATERMARK_SIZE_PCT),
}


@dataclass
class Probe:
    width: int
    height: int
    fps_num: int
    fps_den: int
    total_frames: int  # Number of frames in the source clip


def probe_video(path: Path) -> Probe:
    """Probe a video's resolution, exact frame rate and frame count."""
    out = subprocess.run(
        [
            "ffprobe",
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=width,height,r_frame_rate",
            "-show_entries",
            "format=duration",
            "-of",
            "json",
            str(path),
        ],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    data = json.loads(out)
    stream = data["streams"][0]
    width, height = int(stream["width"]), int(stream["height"])
    fps_num, fps_den = (int(x) for x in stream["r_frame_rate"].split("/"))
    duration = float(data["format"]["duration"])
    total_frames = round(duration * fps_num / fps_den)
    return Probe(width=width, height=height, fps_num=fps_num, fps_den=fps_den, total_frames=total_frames)


def _prop(name: str, value: object) -> str:
    return f'    <property name={quoteattr(name)}>{value}</property>'


def _fit_filter(filter_id: str, out: int) -> str:
    """A full-frame `affine` filter that letterboxes/pillarboxes an image, preserving aspect."""
    return (
        f'    <filter id={quoteattr(filter_id)} in="0" out="{out}">\n'
        f"{_prop('mlt_service', 'affine')}\n"
        f"{_prop('background', 'color:#00000000')}\n"
        f"{_prop('transition.fill', '1')}\n"
        f"{_prop('transition.distort', '0')}\n"
        f"{_prop('transition.rect', '0=0%/0%:100%x100%')}\n"
        f"    </filter>"
    )


def _watermark_filters(transform_id: str, opacity_id: str, out: int, corner: str, alpha_keyframes: str) -> str:
    x, y = _CORNER_XY[corner]
    rect = f"0={x}%/{y}%:{WATERMARK_SIZE_PCT}%x{WATERMARK_SIZE_PCT}%"
    return (
        f'    <filter id={quoteattr(transform_id)} in="0" out="{out}">\n'
        f"{_prop('mlt_service', 'affine')}\n"
        f"{_prop('background', 'color:#00000000')}\n"
        f"{_prop('transition.fill', '1')}\n"
        f"{_prop('transition.distort', '0')}\n"
        f"{_prop('transition.rect', rect)}\n"
        f"    </filter>\n"
        f'    <filter id={quoteattr(opacity_id)} in="0" out="{out}">\n'
        f"{_prop('mlt_service', 'brightness')}\n"
        f"{_prop('level', '1')}\n"
        f"{_prop('alpha', alpha_keyframes)}\n"
        f"    </filter>"
    )


def _clamped_slowdown_windows(total_frames: int, fps_num: int, fps_den: int, lo: int, hi: int) -> list[tuple[int, int]]:
    """Slow-motion windows as [start, end) source-frame ranges, clamped to [lo, hi)."""
    fps = fps_num / fps_den
    windows = []
    for start_s, end_s in SLOWDOWN_WINDOWS_S:
        start = max(lo, round(start_s * fps))
        end = min(hi, round(end_s * fps))
        if end > start:
            windows.append((start, end))
    return windows


def build_project_xml(
    source: Path,
    probe: Probe,
    watermarks: list[tuple[Path, tuple[str, ...]]],
    fadein_image: Path | None,
    fadeout_image: Path | None,
) -> str:
    """Build the MLT project XML for one edited, watermarked clip."""
    fps_num, fps_den, total = probe.fps_num, probe.fps_den, probe.total_frames
    still_frames = round(FADE_STILL_SECONDS * fps_num / fps_den)
    trans_frames = round(FADE_TRANSITION_SECONDS * fps_num / fps_den)
    has_fadein = fadein_image is not None
    has_fadeout = fadeout_image is not None

    body_lo = trans_frames if has_fadein else 0
    body_hi = total - trans_frames if has_fadeout else total
    windows = _clamped_slowdown_windows(total, fps_num, fps_den, body_lo, body_hi)

    producers: list[str] = []
    v1_entries: list[str] = []  # (producer_ref_xml, final_len) built as we go
    cursor = 0  # Current position on the final (post-edit) timeline

    def add_entry(xml: str, length: int) -> None:
        nonlocal cursor
        v1_entries.append(xml)
        cursor += length

    main_id = "main"
    producers.append(
        f'  <producer id="{main_id}" in="0" out="{total - 1}">\n'
        f"{_prop('length', total)}\n"
        f"{_prop('resource', source)}\n"
        f"{_prop('mlt_service', 'avformat-novalidate')}\n"
        f"  </producer>"
    )

    slow_id = "slowmo"
    if windows:
        slow_out = round(total / SLOWDOWN_SPEED)
        producers.append(
            f'  <producer id="{slow_id}" in="0" out="{slow_out}">\n'
            f"{_prop('length', slow_out + 1)}\n"
            f"{_prop('resource', f'{SLOWDOWN_SPEED}:{source}')}\n"
            f"{_prop('mlt_service', 'timewarp')}\n"
            f"{_prop('warp_resource', source)}\n"
            f"{_prop('warp_speed', SLOWDOWN_SPEED)}\n"
            f"{_prop('warp_pitch', '1')}\n"
            f"  </producer>"
        )

    # Lead fade-in: a still, trimmed to its solo portion, then a crossfade into the clip's start.
    lead_transition_id = "lead_transition"
    if has_fadein:
        fadein_id = "fadein_still"
        producers.append(
            f'  <producer id="{fadein_id}" in="0" out="{still_frames - 1}">\n'
            f"{_prop('length', still_frames)}\n"
            f"{_prop('resource', fadein_image)}\n"
            f"{_prop('mlt_service', 'qimage')}\n"
            f"{_fit_filter('fadein_fit', still_frames - 1)}\n"
            f"  </producer>"
        )
        solo_len = still_frames - trans_frames
        if solo_len > 0:
            add_entry(f'    <entry producer="{fadein_id}" in="0" out="{solo_len - 1}" />', solo_len)
        producers.append(
            f'  <tractor id="{lead_transition_id}" in="0" out="{trans_frames - 1}">\n'
            f'    <track producer="{fadein_id}" in="{solo_len}" out="{still_frames - 1}" />\n'
            f'    <track producer="{main_id}" in="0" out="{trans_frames - 1}" />\n'
            f'    <transition id="lead_luma" out="{trans_frames - 1}">\n'
            f"{_prop('a_track', '0')}\n"
            f"{_prop('b_track', '1')}\n"
            f"{_prop('mlt_service', 'luma')}\n"
            f"    </transition>\n"
            f'    <transition id="lead_mix" out="{trans_frames - 1}">\n'
            f"{_prop('a_track', '0')}\n"
            f"{_prop('b_track', '1')}\n"
            f"{_prop('start', '-2')}\n"
            f"{_prop('accepts_blanks', '1')}\n"
            f"{_prop('mlt_service', 'mix')}\n"
            f"    </transition>\n"
            f"  </tractor>"
        )
        fade_in_start = cursor  # Frame where the watermarks start ramping up
        add_entry(f'    <entry producer="{lead_transition_id}" in="0" out="{trans_frames - 1}" />', trans_frames)
        fade_in_end = cursor - 1  # Frame where the main video takes over at full opacity
    else:
        fade_in_start = fade_in_end = 0  # Watermarks are at full opacity from the first frame

    # Body: the clip's middle, split into normal and slow-motion pieces.
    body_cur = body_lo
    for start, end in windows:
        if start > body_cur:
            add_entry(f'    <entry producer="{main_id}" in="{body_cur}" out="{start - 1}" />', start - body_cur)
        warped_len = round((end - start) / SLOWDOWN_SPEED)
        warped_in = int(start / SLOWDOWN_SPEED)
        add_entry(
            f'    <entry producer="{slow_id}" in="{warped_in}" out="{warped_in + warped_len - 1}" />', warped_len
        )
        body_cur = end
    if body_hi > body_cur:
        add_entry(f'    <entry producer="{main_id}" in="{body_cur}" out="{body_hi - 1}" />', body_hi - body_cur)

    # Trailing fade-out: a crossfade from the clip's end, then the still's solo portion.
    trail_transition_id = "trail_transition"
    if has_fadeout:
        fadeout_id = "fadeout_still"
        producers.append(
            f'  <producer id="{fadeout_id}" in="0" out="{still_frames - 1}">\n'
            f"{_prop('length', still_frames)}\n"
            f"{_prop('resource', fadeout_image)}\n"
            f"{_prop('mlt_service', 'qimage')}\n"
            f"{_fit_filter('fadeout_fit', still_frames - 1)}\n"
            f"  </producer>"
        )
        fade_out_start = cursor  # Frame where the watermarks start ramping down
        producers.append(
            f'  <tractor id="{trail_transition_id}" in="0" out="{trans_frames - 1}">\n'
            f'    <track producer="{main_id}" in="{total - trans_frames}" out="{total - 1}" />\n'
            f'    <track producer="{fadeout_id}" in="0" out="{trans_frames - 1}" />\n'
            f'    <transition id="trail_luma" out="{trans_frames - 1}">\n'
            f"{_prop('a_track', '0')}\n"
            f"{_prop('b_track', '1')}\n"
            f"{_prop('mlt_service', 'luma')}\n"
            f"    </transition>\n"
            f'    <transition id="trail_mix" out="{trans_frames - 1}">\n'
            f"{_prop('a_track', '0')}\n"
            f"{_prop('b_track', '1')}\n"
            f"{_prop('start', '-2')}\n"
            f"{_prop('accepts_blanks', '1')}\n"
            f"{_prop('mlt_service', 'mix')}\n"
            f"    </transition>\n"
            f"  </tractor>"
        )
        add_entry(f'    <entry producer="{trail_transition_id}" in="0" out="{trans_frames - 1}" />', trans_frames)
        fade_out_end = cursor - 1  # Frame where the still takes over alone
        solo_len = still_frames - trans_frames
        if solo_len > 0:
            add_entry(f'    <entry producer="{fadeout_id}" in="{trans_frames}" out="{still_frames - 1}" />', solo_len)
    else:
        fade_out_start = fade_out_end = cursor - 1  # Watermarks stay at full opacity to the end

    total_final = cursor

    # One producer per watermark corner, each with its own position and an opacity ramp that
    # matches the fades: invisible under a still, ramping during its crossfade, opaque otherwise.
    alpha_keyframes = f"0={0 if has_fadein else 1};"
    if has_fadein:
        alpha_keyframes += f"{fade_in_start}=0;{fade_in_end}=1;"
    alpha_keyframes += f"{fade_out_start}=1;"
    if has_fadeout:
        alpha_keyframes += f"{fade_out_end}=0;"
    alpha_keyframes = alpha_keyframes.rstrip(";")

    watermark_tracks: list[str] = []
    for i, (image, corners) in enumerate(watermarks):
        for j, corner in enumerate(corners):
            wm_id = f"wm_{i}_{j}"
            producers.append(
                f'  <producer id="{wm_id}" in="0" out="{total_final - 1}">\n'
                f"{_prop('length', total_final)}\n"
                f"{_prop('resource', image)}\n"
                f"{_prop('mlt_service', 'qimage')}\n"
                f"{_watermark_filters(f'{wm_id}_t', f'{wm_id}_o', total_final - 1, corner, alpha_keyframes)}\n"
                f"  </producer>"
            )
            watermark_tracks.append(
                f'  <playlist id="pl_{wm_id}">\n'
                f'    <entry producer="{wm_id}" in="0" out="{total_final - 1}" />\n'
                f"  </playlist>"
            )

    tracks_xml = "\n".join(f'    <track producer="pl_wm_{i}_{j}" />' for i, (_, c) in enumerate(watermarks) for j in range(len(c)))
    composite_transitions = "\n".join(
        f'    <transition id="composite_{i}_{j}">\n'
        f"{_prop('a_track', '0')}\n"
        f"{_prop('b_track', str(1 + k))}\n"
        f"{_prop('mlt_service', 'qtblend')}\n"
        f"    </transition>"
        for k, (i, j) in enumerate((i, j) for i, (_, c) in enumerate(watermarks) for j in range(len(c)))
    )

    profile = (
        f'<profile description="{probe.width}x{probe.height}" width="{probe.width}" height="{probe.height}" '
        f'progressive="1" sample_aspect_num="1" sample_aspect_den="1" '
        f'display_aspect_num="{probe.width}" display_aspect_den="{probe.height}" '
        f'frame_rate_num="{fps_num}" frame_rate_den="{fps_den}" colorspace="709" />'
    )

    return (
        "<?xml version='1.0' encoding='utf-8'?>\n"
        '<mlt LC_NUMERIC="C" producer="tractor0">\n'
        f"  {profile}\n"
        + "\n".join(producers)
        + "\n"
        + '  <playlist id="playlist_v1">\n'
        + "\n".join(v1_entries)
        + "\n  </playlist>\n"
        + "\n".join(watermark_tracks)
        + "\n"
        + f'  <tractor id="tractor0" in="0" out="{total_final - 1}">\n'
        + '    <track producer="playlist_v1" />\n'
        + tracks_xml
        + "\n"
        + composite_transitions
        + "\n  </tractor>\n"
        "</mlt>\n"
    )


def edit_clip(
    original: Path,
    output: Path,
    watermarks: list[tuple[Path, tuple[str, ...]]] | None = None,
    fadein_image: Path | None = None,
    fadeout_image: Path | None = None,
    keep_project: Path | None = None,
) -> Path:
    """Edit `original` (watermarks, fades, slow motion) and render it to `output` with melt.

    Args:
        original: Input video file.
        output: Output video file. Must differ from `original`.
        watermarks: Pairs of (image, corners); defaults to the always-used watermark alone.
        fadein_image: Still image to fade in from, or None for no opening fade.
        fadeout_image: Still image to fade out to, or None for no closing fade.
        keep_project: If given, the generated .mlt project is also written here (for inspection).

    Returns:
        The output path.
    """
    if watermarks is None:
        watermarks = [(ALWAYS_WATERMARK, ALWAYS_CORNERS)]
    # melt resolves relative resource paths against the project file's own directory, not the
    # caller's cwd, so every path written into the XML must be absolute.
    original = original.resolve()
    watermarks = [(image.resolve(), corners) for image, corners in watermarks]
    fadein_image = fadein_image.resolve() if fadein_image else None
    fadeout_image = fadeout_image.resolve() if fadeout_image else None

    probe = probe_video(original)
    xml = build_project_xml(original, probe, watermarks, fadein_image, fadeout_image)

    project_path = keep_project or output.with_suffix(".generated.mlt")
    project_path.write_text(xml, encoding="utf-8")
    try:
        subprocess.run(
            [
                MELT,
                str(project_path),
                "-consumer",
                f"avformat:{output}",
                "vcodec=libx264",
                "crf=18",
                "preset=medium",
                "acodec=aac",
                "ab=192k",
                "pix_fmt=yuv420p",
                "movflags=+faststart",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
    except subprocess.CalledProcessError as e:
        logger.error("melt failed for %s:\n%s", original, e.stderr)
        raise
    finally:
        if keep_project is None:
            project_path.unlink(missing_ok=True)
    return output


def watermarks_for(image: str | None) -> list[tuple[Path, tuple[str, ...]]]:
    """Return the always-used watermark, plus `image` (a file name) in IMAGE_CORNERS if given."""
    watermarks = [(ALWAYS_WATERMARK, ALWAYS_CORNERS)]
    if image:
        watermarks.append((Path(image), IMAGE_CORNERS))
    return watermarks


def main() -> None:
    parser = argparse.ArgumentParser(description="Edit and watermark a clip entirely with Shotcut's MLT/melt engine.")
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--image", help="Extra watermark image (file name), in addition to selkabot.png")
    parser.add_argument("--fadein-image", type=Path, default=None)
    parser.add_argument("--fadeout-image", type=Path, default=None)
    parser.add_argument("--keep-project", type=Path, default=None, help="Also save the generated .mlt here")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(message)s")
    edit_clip(
        args.input,
        args.output,
        watermarks_for(args.image),
        args.fadein_image,
        args.fadeout_image,
        args.keep_project,
    )
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
