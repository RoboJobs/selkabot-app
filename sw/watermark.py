"""Add a watermark image to the bottom-left and top-right corners of a video.

Uses ffmpeg. The watermark is scaled to WATERMARK_WIDTH_PCT of the video width,
keeping its aspect ratio, and the video is re-encoded with the audio copied.
"""

from __future__ import annotations

import json
import subprocess
from pathlib import Path

WATERMARK_WIDTH_PCT = 0.10  # Watermark width as a fraction of the video width
WATERMARK_IMAGE = Path("selkabot.png")  # Relative to the current working directory


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


def add_watermark(
    video: Path,
    output: Path,
    image: Path = WATERMARK_IMAGE,
    width_pct: float = WATERMARK_WIDTH_PCT,
) -> Path:
    """Write a copy of `video` with `image` in the bottom-left and top-right corners.

    Args:
        video: Input video file.
        output: Output video file. Must differ from `video`.
        image: Watermark image (PNG with alpha recommended).
        width_pct: Watermark width as a fraction of the video width (0.10 = 10%).

    Returns:
        The output path.
    """
    wm_width = round(video_width(video) * width_pct)
    filter_graph = (
        f"[1:v]scale={wm_width}:-1,format=rgba,split=2[wm_a][wm_b];"
        "[0:v][wm_a]overlay=0:main_h-overlay_h[tmp];"
        "[tmp][wm_b]overlay=main_w-overlay_w:0"
    )
    subprocess.run(
        [
            "ffmpeg",
            "-y",
            "-v",
            "error",
            "-i",
            str(video),
            "-i",
            str(image),
            "-filter_complex",
            filter_graph,
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
