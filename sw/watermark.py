"""Add watermark images to the corners of a video.

Uses ffmpeg. Each watermark is scaled to WATERMARK_WIDTH_PCT of the video width,
keeping its aspect ratio, and the video is re-encoded with the audio copied.
"""

from __future__ import annotations

import json
import subprocess
from pathlib import Path

WATERMARK_WIDTH_PCT = 0.10  # Watermark width as a fraction of the video width
# Always-used watermarks: (image, corners it appears in). Images are relative to the current working directory.
WATERMARKS: list[tuple[Path, tuple[str, ...]]] = [
    (Path("selkabot.png"), ("bottom-left", "top-right")),
]
# Corners of the optional, user-chosen image (the corners jdc.jpg always used).
IMAGE_CORNERS = ("top-left", "bottom-right")
# Overlay x:y for each corner of the video.
CORNERS = {
    "top-left": ("0", "0"),
    "top-right": ("main_w-overlay_w", "0"),
    "bottom-left": ("0", "main_h-overlay_h"),
    "bottom-right": ("main_w-overlay_w", "main_h-overlay_h"),
}


def watermarks_for(image: str | None) -> list[tuple[Path, tuple[str, ...]]]:
    """Return WATERMARKS, plus the file `image` in IMAGE_CORNERS when one is given."""
    if not image:
        return list(WATERMARKS)
    return [*WATERMARKS, (Path(image), IMAGE_CORNERS)]


def video_width(video: Path) -> int:
    """Return the width in pixels of the first video stream."""
    out = subprocess.run(
        [
            "ffprobe",
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=width",
            "-of",
            "json",
            str(video),
        ],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    return int(json.loads(out)["streams"][0]["width"])


def _filter_graph(wm_width: int, watermarks: list[tuple[Path, tuple[str, ...]]]) -> str:
    """Build the ffmpeg filter graph; the final output is labelled [vout]."""
    parts = []
    placements = []  # (copy label, corner) for every corner of every image
    for i, (_, corners) in enumerate(watermarks, start=1):
        labels = [f"w{i}_{j}" for j in range(len(corners))]
        parts.append(f"[{i}:v]scale={wm_width}:-1,format=rgba,split={len(corners)}" + "".join(f"[{s}]" for s in labels))
        placements += list(zip(labels, corners))
    last = "0:v"
    for k, (label, corner) in enumerate(placements):
        x, y = CORNERS[corner]
        out = "vout" if k == len(placements) - 1 else f"v{k}"
        parts.append(f"[{last}][{label}]overlay={x}:{y}[{out}]")
        last = out
    return ";".join(parts)


def add_watermark(
    video: Path,
    output: Path,
    watermarks: list[tuple[Path, tuple[str, ...]]] = WATERMARKS,
    width_pct: float = WATERMARK_WIDTH_PCT,
) -> Path:
    """Write a copy of `video` with each watermark image in its corners.

    Args:
        video: Input video file.
        output: Output video file. Must differ from `video`.
        watermarks: Pairs of (image, corners). Images are PNG or JPEG; PNG with alpha is best.
        width_pct: Watermark width as a fraction of the video width (0.10 = 10%).

    Returns:
        The output path.
    """
    wm_width = round(video_width(video) * width_pct)
    image_args = []
    for image, _ in watermarks:
        image_args += ["-i", str(image)]
    subprocess.run(
        [
            "ffmpeg",
            "-y",
            "-v",
            "error",
            "-i",
            str(video),
            *image_args,
            "-filter_complex",
            _filter_graph(wm_width, watermarks),
            "-map",
            "[vout]",
            "-map",
            "0:a?",
            "-c:v",
            "libx264",
            "-crf",
            "18",
            "-preset",
            "medium",
            "-pix_fmt",
            "yuv420p",
            "-c:a",
            "copy",
            str(output),
        ],
        check=True,
    )
    return output
