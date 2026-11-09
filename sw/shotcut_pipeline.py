"""Shotcut (MLT/melt) edit engine: clip a recording, add watermarks, fades and slow motion.

The project is built as MLT XML (the same format Shotcut saves), so it can be opened in
Shotcut afterwards, and rendered with `melt` (Shotcut's own engine) — no Shotcut GUI and
no MCP server are needed at runtime.

Edits, all in source-frame terms:
- Clip: a window of the recording (start, length), default the whole recording.
- Slow motion: each slowdown window is played at `speed`. Smoothing is one of:
    "high": pre-rendered with ffmpeg minterpolate (motion-compensated 200 fps interpolation,
            then slowed down), cached under SMOOTH_CACHE_DIR. Slow to render, best quality.
    "shotcut": MLT's native `timewarp` producer. Instant, lower quality.
- Fades: an optional still image fades in at the start and out at the end (crossfades).
- Watermarks: images in fixed corners, faded in and out with the stills.

`watermarks_for` and `edit_clip` are the entry points; see sw/shotcut_watermark.py for the
standalone script and settings UI.
"""

from __future__ import annotations

import hashlib
import json
import logging
import os
import re
import shutil
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Callable
from xml.sax.saxutils import quoteattr

logger = logging.getLogger(__name__)

# Receives short progress messages, such as "slowdown 1/2: 40%". Used by the window's status line.
ProgressFn = Callable[[str], None]


def _report(progress: ProgressFn | None, message: str) -> None:
    """Send a progress message to the caller, or to the log when there is no caller to send it to."""
    if progress is not None:
        progress(message)
    else:
        logger.info(message)


def _emit(progress: ProgressFn | None, message: str) -> None:
    """Log a message and also send it to the caller (the terminal and the window both see it)."""
    logger.info(message)
    if progress is not None:
        progress(message)


def _run_ffmpeg(cmd: list[str], expected_seconds: float, progress: ProgressFn | None) -> None:
    """Run ffmpeg, reporting its progress every 5% of `expected_seconds` of output."""
    full = [cmd[0], "-progress", "pipe:1", "-nostats", *cmd[1:]]
    proc = subprocess.Popen(full, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    assert proc.stdout is not None and proc.stderr is not None
    shown = -5
    for line in proc.stdout:
        key, _, value = line.strip().partition("=")
        if key in ("out_time_us", "out_time_ms") and value.isdigit():
            pct = min(99, int(int(value) / 1e6 / expected_seconds * 100))
            if pct >= shown + 5:
                shown = pct
                _report(progress, f"{pct}%")
    err = proc.stderr.read()
    rc = proc.wait()
    if rc:
        raise subprocess.CalledProcessError(rc, full, stderr=err)


def _run_melt(cmd: list[str], progress: ProgressFn | None) -> None:
    """Run melt, reporting the percentage it prints every 5%. Raises CalledProcessError on failure."""
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    assert proc.stdout is not None
    fd = proc.stdout.fileno()
    output: list[str] = []
    carry = ""
    shown = -5
    while chunk := os.read(fd, 4096):
        text = chunk.decode("utf-8", "replace")
        output.append(text)
        search = carry + text
        for match in re.finditer(r"percentage:\s*(\d+)", search):
            pct = int(match.group(1))
            if pct >= shown + 5 or (pct == 100 and shown < 100):
                shown = pct
                _report(progress, f"{pct}%")
        carry = search[-24:]
    rc = proc.wait()
    if rc:
        raise subprocess.CalledProcessError(rc, cmd, output="".join(output)[-4000:])

ROOT = Path(__file__).resolve().parents[1]
# Always-used watermark, plus its two corners (mirrors sw/watermark.py's WATERMARKS).
ALWAYS_WATERMARK = ROOT / "selkabot.png"
ALWAYS_CORNERS = ("bottom-left", "top-right")
# Corners of the optional, user-chosen image (sw/watermark.py's IMAGE_CORNERS).
IMAGE_CORNERS = ("top-left", "bottom-right")
WATERMARK_SIZE_PCT = 10  # Each watermark's bounding box, as a percentage of the frame

# Smoothed slow-motion segments are cached here, outside the recording folders (the upload
# sync uploads every .mp4 it finds in a recording folder).
SMOOTH_CACHE_DIR = ROOT / "videos" / "shotcut" / "smoothed"
SMOOTHING_MODES = ("high", "shotcut")
# ffmpeg minterpolate is single-threaded, so each slowdown is split into this many chunks that
# are rendered in parallel. Chunks overlap by CHUNK_CONTEXT_FRAMES source frames on each side.
DEFAULT_CHUNKS = 4
CHUNK_CONTEXT_FRAMES = 2
MIN_CHUNK_SECONDS = 0.5  # Shorter windows get fewer chunks

FADE_STILL_SECONDS = 2.0  # Full length of each fade still, before the crossfade eats into it
FADE_TRANSITION_SECONDS = 1.0  # Crossfade length, split evenly between the still and the clip

# The flatpak build of melt (SHOTCUT_MELT_PATH=shotcut-melt) cannot read files under /tmp, so
# keep projects and renders inside the repository or the home folder.
MELT = os.environ.get("SHOTCUT_MELT_PATH", "melt")
FFMPEG = os.environ.get("FFMPEG_PATH", "ffmpeg")

# x%, y% of the top-left corner of each watermark's bounding box (box is WATERMARK_SIZE_PCT wide).
_CORNER_XY = {
    "top-left": (0, 0),
    "top-right": (100 - WATERMARK_SIZE_PCT, 0),
    "bottom-left": (0, 100 - WATERMARK_SIZE_PCT),
    "bottom-right": (100 - WATERMARK_SIZE_PCT, 100 - WATERMARK_SIZE_PCT),
}


@dataclass(frozen=True)
class EditSettings:
    """Everything that changes from one video to the next. Times are in seconds.

    slowdowns: (start, end) pairs, relative to the start of the clip (not of the recording).
    clip_start: where the clip starts in the recording.
    clip_length: clip duration, or None for the rest of the recording.
    speed: playback speed inside a slowdown window (0.3 = 30%).
    smoothing: "high" (ffmpeg minterpolate, slow) or "shotcut" (MLT timewarp, instant).
    fadein_image / fadeout_image: stills for the opening and closing fades, or None.
    """

    slowdowns: tuple[tuple[float, float], ...] = ((8.0, 12.0), (14.0, 16.0))
    speed: float = 0.3
    clip_start: float = 0.0
    clip_length: float | None = None
    smoothing: str = "high"
    chunks: int = DEFAULT_CHUNKS  # Parallel pieces per slowdown in "high" smoothing
    fadein_image: Path | None = None
    fadeout_image: Path | None = None
    soundtrack: Path | None = None  # Replaces the recording's own sound when set
    soundtrack_start: float = 0.0  # Where in the soundtrack file the output starts, in seconds

    def __post_init__(self) -> None:
        if self.soundtrack_start < 0:
            raise ValueError("soundtrack_start cannot be negative")
        if self.smoothing not in SMOOTHING_MODES:
            raise ValueError(f"smoothing must be one of {SMOOTHING_MODES}, not {self.smoothing!r}")
        if not 0 < self.speed <= 1:
            raise ValueError(f"speed must be in (0, 1], not {self.speed}")
        if self.chunks < 1:
            raise ValueError("chunks must be at least 1")
        if self.clip_start < 0:
            raise ValueError("clip_start cannot be negative")
        if self.clip_length is not None and self.clip_length <= 0:
            raise ValueError("clip_length must be positive, or None for the rest of the recording")
        for start, end in self.slowdowns:
            if not 0 <= start < end:
                raise ValueError(f"slowdown ({start}, {end}) must have 0 <= start < end")


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
    return f"    <property name={quoteattr(name)}>{value}</property>"


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


def _atempo_chain(speed: float) -> str:
    """ffmpeg atempo filters (each limited to 0.5..2) whose product is `speed`; pitch is kept."""
    filters = []
    remaining = speed
    while remaining < 0.5:
        filters.append("atempo=0.5")
        remaining /= 0.5
    filters.append(f"atempo={remaining:.6f}")
    return ",".join(filters)


def _has_audio(path: Path) -> bool:
    out = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "a", "-show_entries", "stream=index", "-of", "csv=p=0", str(path)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    return bool(out.strip())


def _effective_chunks(frames: int, fps: float, chunks: int) -> int:
    """At most `chunks`, and never so many that a chunk is shorter than MIN_CHUNK_SECONDS."""
    return max(1, min(chunks, int(frames / fps / MIN_CHUNK_SECONDS)))


def smooth_segment(
    source: Path,
    probe: Probe,
    start_frame: int,
    end_frame: int,
    speed: float,
    cache_dir: Path = SMOOTH_CACHE_DIR,
    progress: ProgressFn | None = None,
    chunks: int = DEFAULT_CHUNKS,
) -> Path:
    """Return a file holding source frames [start_frame, end_frame) slowed to `speed`, smoothed.

    Motion-compensated interpolation (minterpolate, mci/aobmc, bidirectional, OBMC-based
    variable-size block matching) to an intermediate rate of fps/speed, then slowed to `speed`
    and resampled back to the source frame rate.

    The window is cut into `chunks` pieces that are interpolated in parallel, each with a little
    extra source footage on either side so the seams are interpolated with real neighbours; the
    extra output frames are trimmed off and the pieces are joined. Audio is slowed in one pass
    with atempo (pitch kept). Results are cached by their inputs.
    """
    fps = probe.fps_num / probe.fps_den
    stat = source.stat()
    video_filter = (
        f"minterpolate=fps={round(fps / speed)}:mi_mode=mci:mc_mode=aobmc:me_mode=bidir:vsbmc=1,"
        f"setpts=PTS/{speed},fps={probe.fps_num}/{probe.fps_den}"
    )
    chunks = _effective_chunks(end_frame - start_frame, fps, chunks)
    # "v3": chunked rendering. The key covers everything that changes the output.
    key_parts = ["v3", str(source.resolve()), str(stat.st_size), str(stat.st_mtime_ns), str(start_frame), str(end_frame)]
    key_parts += [str(speed), video_filter, _atempo_chain(speed), f"chunks={chunks}"]
    key = hashlib.sha1("|".join(key_parts).encode()).hexdigest()[:16]
    output = cache_dir / f"smooth_{source.stem}_{start_frame}-{end_frame}_{key}.mp4"
    if output.exists():
        _report(progress, f"reusing cached {output.name}")
        return output

    cache_dir.mkdir(parents=True, exist_ok=True)
    work = cache_dir / f"{output.stem}.parts"
    temp = output.with_name(output.stem + ".tmp.mp4")
    expected_out = (end_frame - start_frame) / fps / speed
    _report(progress, f"ffmpeg started: {end_frame - start_frame} source frames -> {expected_out:.1f}s at {speed}x, {chunks} chunk(s)")
    started = time.monotonic()
    try:
        work.mkdir(parents=True, exist_ok=True)
        parts = _render_chunks(source, probe, start_frame, end_frame, speed, video_filter, chunks, work, progress)
        joined = work / "video.mp4"
        _concat_parts(parts, joined, work)
        if _has_audio(source):
            audio = work / "audio.m4a"
            _render_audio(source, probe, start_frame, end_frame, speed, audio)
            _mux(joined, audio, temp)
        else:
            shutil.copyfile(joined, temp)
    except BaseException:
        temp.unlink(missing_ok=True)
        shutil.rmtree(work, ignore_errors=True)
        raise
    shutil.rmtree(work, ignore_errors=True)
    expected = round((end_frame - start_frame) / speed)
    actual = _video_frames(temp)
    if abs(actual - expected) > 2:
        temp.unlink(missing_ok=True)
        raise RuntimeError(f"smoothed segment has {actual} frames, expected about {expected}")
    os.replace(temp, output)
    _report(progress, f"ffmpeg finished in {time.monotonic() - started:.0f}s")
    return output


def _render_chunks(
    source: Path,
    probe: Probe,
    start_frame: int,
    end_frame: int,
    speed: float,
    video_filter: str,
    chunks: int,
    work: Path,
    progress: ProgressFn | None,
) -> list[Path]:
    """Interpolate the window in `chunks` parallel pieces; returns them in order, trimmed exactly.

    Piece i covers source frames [lo, hi) of the window, read with CHUNK_CONTEXT_FRAMES extra on
    each side. Its output is trimmed to the output frames that belong to [lo, hi). The trim counts
    are taken from the window's own frame grid (round(offset / speed)), so the pieces add up to
    exactly the length of the whole window.
    """
    fps = probe.fps_num / probe.fps_den
    length = end_frame - start_frame
    bounds = [start_frame + round(i * length / chunks) for i in range(chunks + 1)]
    jobs = []
    for i, (lo, hi) in enumerate(zip(bounds, bounds[1:])):
        a = max(start_frame, lo - CHUNK_CONTEXT_FRAMES)
        b = min(end_frame, hi + CHUNK_CONTEXT_FRAMES)
        trim_start = round((lo - start_frame) / speed) - round((a - start_frame) / speed)
        trim_end = round((hi - start_frame) / speed) - round((a - start_frame) / speed)
        jobs.append((i, a, b, trim_start, trim_end))

    def render(job: tuple[int, int, int, int, int]) -> Path:
        i, a, b, trim_start, trim_end = job
        out = work / f"part_{i:02d}.mp4"
        vf = f"{video_filter},trim=start_frame={trim_start}:end_frame={trim_end},setpts=PTS-STARTPTS"
        chunk_seconds = (trim_end - trim_start) / fps
        _run_ffmpeg(
            [
                FFMPEG,
                "-v",
                "error",
                "-y",
                "-ss",
                f"{a / fps:.6f}",
                "-t",
                f"{(b - a) / fps:.6f}",
                "-i",
                str(source),
                "-an",
                "-vf",
                vf,
                "-c:v",
                "libx264",
                "-crf",
                "18",
                "-preset",
                "fast",
                "-pix_fmt",
                "yuv420p",
                str(out),
            ],
            chunk_seconds,
            None if progress is None else (lambda m, i=i: progress(f"chunk {i + 1}/{chunks} {m}")),
        )
        return out

    with ThreadPoolExecutor(max_workers=chunks) as pool:
        return list(pool.map(render, jobs))


def _concat_parts(parts: list[Path], output: Path, work: Path) -> None:
    listing = work / "parts.txt"
    listing.write_text("".join(f"file '{part.resolve()}'\n" for part in parts), encoding="utf-8")
    subprocess.run(
        [FFMPEG, "-v", "error", "-y", "-f", "concat", "-safe", "0", "-i", str(listing), "-c", "copy", str(output)],
        check=True,
        capture_output=True,
        text=True,
    )


def _render_audio(source: Path, probe: Probe, start_frame: int, end_frame: int, speed: float, output: Path) -> None:
    fps = probe.fps_num / probe.fps_den
    subprocess.run(
        [
            FFMPEG,
            "-v",
            "error",
            "-y",
            "-ss",
            f"{start_frame / fps:.6f}",
            "-t",
            f"{(end_frame - start_frame) / fps:.6f}",
            "-i",
            str(source),
            "-vn",
            # Pad to the video's length: the slowed audio can be a few frames short, and the mux
            # must not cut the video down to it.
            "-af",
            f"{_atempo_chain(speed)},apad",
            "-t",
            f"{(end_frame - start_frame) / fps / speed:.6f}",
            "-c:a",
            "aac",
            "-b:a",
            "192k",
            str(output),
        ],
        check=True,
        capture_output=True,
        text=True,
    )


def _mux(video: Path, audio: Path, output: Path) -> None:
    subprocess.run(
        [
            FFMPEG,
            "-v",
            "error",
            "-y",
            "-i",
            str(video),
            "-i",
            str(audio),
            "-map",
            "0:v:0",
            "-map",
            "1:a:0",
            "-c",
            "copy",
            str(output),
        ],
        check=True,
        capture_output=True,
        text=True,
    )


def _clip_window(lo_frame: int, hi_frame: int, fps: float, start_s: float, end_s: float) -> tuple[int, int] | None:
    """[start, end) source frames for a window given in clip-relative seconds, clamped to [lo, hi)."""
    start = max(lo_frame, lo_frame + round(start_s * fps))
    end = min(hi_frame, lo_frame + round(end_s * fps))
    return (start, end) if end > start else None


def _duration_seconds(path: Path) -> float:
    out = subprocess.run(
        ["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", str(path)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    return float(out.strip())


def build_project_xml(
    source: Path,
    probe: Probe,
    watermarks: list[tuple[Path, tuple[str, ...]]],
    fadein_image: Path | None,
    fadeout_image: Path | None,
    clip: tuple[int, int],
    windows: list[tuple[int, int, Path | None]],
    speed: float,
    music: tuple[Path, int, int] | None = None,
) -> tuple[str, int]:
    """Build the MLT project XML for one edited, watermarked clip; also returns its length in frames.

    clip: (first source frame, number of frames) of the clip.
    music: (soundtrack file, first frame of the soundtrack to use, length of the soundtrack in
        frames), or None to keep the recording's own sound. Frames are at the project's frame rate.
    windows: slow-motion windows as (start, end) source frames within the clip, each with the
        pre-rendered smoothed file for it, or None to use MLT's timewarp instead.
    """
    fps_num, fps_den = probe.fps_num, probe.fps_den
    fps = fps_num / fps_den
    still_frames = round(FADE_STILL_SECONDS * fps)
    trans_frames = round(FADE_TRANSITION_SECONDS * fps)
    has_fadein = fadein_image is not None
    has_fadeout = fadeout_image is not None
    clip_lo, clip_n = clip
    clip_hi = clip_lo + clip_n

    producers: list[str] = []
    v1_entries: list[str] = []
    cursor = 0  # Current position on the final (post-edit) timeline

    def add_entry(xml: str, length: int) -> None:
        nonlocal cursor
        v1_entries.append(xml)
        cursor += length

    main_id = "main"
    producers.append(
        f'  <producer id="{main_id}" in="0" out="{probe.total_frames - 1}">\n'
        f"{_prop('length', probe.total_frames)}\n"
        f"{_prop('resource', source)}\n"
        f"{_prop('mlt_service', 'avformat-novalidate')}\n"
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
            f'    <track producer="{main_id}" in="{clip_lo}" out="{clip_lo + trans_frames - 1}" />\n'
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
    body_lo = clip_lo + (trans_frames if has_fadein else 0)
    body_hi = clip_hi - (trans_frames if has_fadeout else 0)
    body_cur = body_lo
    timewarp_needed = any(smoothed is None for _, _, smoothed in windows)
    if timewarp_needed:
        timewarp_out = round(probe.total_frames / speed)
        producers.append(
            f'  <producer id="timewarp" in="0" out="{timewarp_out}">\n'
            f"{_prop('length', timewarp_out + 1)}\n"
            f"{_prop('resource', f'{speed}:{source}')}\n"
            f"{_prop('mlt_service', 'timewarp')}\n"
            f"{_prop('warp_resource', source)}\n"
            f"{_prop('warp_speed', speed)}\n"
            f"{_prop('warp_pitch', '1')}\n"
            f"  </producer>"
        )
    for k, (start, end, smoothed) in enumerate(windows):
        if start > body_cur:
            add_entry(f'    <entry producer="{main_id}" in="{body_cur}" out="{start - 1}" />', start - body_cur)
        if smoothed is None:
            warped_len = round((end - start) / speed)
            warped_in = int(start / speed)
            add_entry(
                f'    <entry producer="timewarp" in="{warped_in}" out="{warped_in + warped_len - 1}" />', warped_len
            )
        else:
            slow_id = f"slow_{k}"
            length = _video_frames(smoothed)
            producers.append(
                f'  <producer id="{slow_id}" in="0" out="{length - 1}">\n'
                f"{_prop('length', length)}\n"
                f"{_prop('resource', smoothed)}\n"
                f"{_prop('mlt_service', 'avformat-novalidate')}\n"
                f"  </producer>"
            )
            add_entry(f'    <entry producer="{slow_id}" in="0" out="{length - 1}" />', length)
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
            f'    <track producer="{main_id}" in="{clip_hi - trans_frames}" out="{clip_hi - 1}" />\n'
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

    corner_pairs = [(i, j) for i, (_, c) in enumerate(watermarks) for j in range(len(c))]
    tracks_xml = "\n".join(f'    <track producer="pl_wm_{i}_{j}" />' for i, j in corner_pairs)
    # With a soundtrack, the recording's own audio is hidden and the soundtrack is an extra
    # audio-only track whose playlist entry is exactly the output's length.
    v1_track = '    <track producer="playlist_v1" />\n'
    music_xml = ""
    music_track = ""
    if music is not None:
        music_path, music_in, music_len = music
        v1_track = '    <track producer="playlist_v1" hide="audio" />\n'
        music_xml = (
            f'  <producer id="music" in="0" out="{music_len - 1}">\n'
            f"{_prop('length', music_len)}\n"
            f"{_prop('resource', music_path)}\n"
            f"{_prop('mlt_service', 'avformat-novalidate')}\n"
            f"  </producer>\n"
            f'  <playlist id="pl_music">\n'
            f'    <entry producer="music" in="{music_in}" out="{music_in + total_final - 1}" />\n'
            f"  </playlist>\n"
        )
        music_track = '    <track producer="pl_music" />\n'
    composite_transitions = "\n".join(
        f'    <transition id="composite_{i}_{j}">\n'
        f"{_prop('a_track', '0')}\n"
        f"{_prop('b_track', str(1 + k))}\n"
        f"{_prop('mlt_service', 'qtblend')}\n"
        f"    </transition>"
        for k, (i, j) in enumerate(corner_pairs)
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
        + music_xml
        + "\n".join(watermark_tracks)
        + "\n"
        + f'  <tractor id="tractor0" in="0" out="{total_final - 1}">\n'
        + v1_track
        + tracks_xml
        + "\n"
        + music_track
        + composite_transitions
        + "\n  </tractor>\n"
        "</mlt>\n",
        total_final,
    )


def _video_frames(path: Path) -> int:
    return probe_video(path).total_frames


def plan_windows(
    probe: Probe, settings: EditSettings, clip: tuple[int, int]
) -> list[tuple[int, int]]:
    """Slow-motion windows as absolute source frames, clamped to the body of the clip."""
    fps = probe.fps_num / probe.fps_den
    clip_lo, clip_n = clip
    trans = round(FADE_TRANSITION_SECONDS * fps)
    body_lo = clip_lo + (trans if settings.fadein_image else 0)
    body_hi = clip_lo + clip_n - (trans if settings.fadeout_image else 0)
    windows = []
    for start_s, end_s in settings.slowdowns:
        window = _clip_window(clip_lo, clip_lo + clip_n, fps, start_s, end_s)
        if window is None:
            continue
        start, end = max(window[0], body_lo), min(window[1], body_hi)
        if end > start:
            windows.append((start, end))
    return windows


def clip_range(probe: Probe, settings: EditSettings) -> tuple[int, int]:
    """(first source frame, number of frames) of the clip described by `settings`."""
    fps = probe.fps_num / probe.fps_den
    first = min(probe.total_frames - 1, round(settings.clip_start * fps))
    available = probe.total_frames - first
    if settings.clip_length is None:
        return first, available
    return first, min(available, round(settings.clip_length * fps))


def edit_clip(
    original: Path,
    output: Path,
    watermarks: list[tuple[Path, tuple[str, ...]]] | None = None,
    fadein_image: Path | None = None,
    fadeout_image: Path | None = None,
    settings: EditSettings | None = None,
    progress: ProgressFn | None = None,
) -> Path:
    """Edit `original` (clip, watermarks, fades, slow motion) and render it to `output` with melt.

    The editable Shotcut project is saved next to `output` as `<output stem>.mlt`.

    Args:
        original: Input video file.
        output: Output video file. Must differ from `original`.
        watermarks: Pairs of (image, corners); defaults to the always-used watermark alone.
        fadein_image: Still image to fade in from, or None for no opening fade.
        fadeout_image: Still image to fade out to, or None for no closing fade.
        settings: Clip, slow-motion and smoothing settings. Its fade images are overridden by
            the explicit fadein_image/fadeout_image arguments when those are given.
        progress: Called with each progress message (also logged). None logs only.

    Returns:
        The output path.
    """
    settings = settings or EditSettings()
    if watermarks is None:
        watermarks = [(ALWAYS_WATERMARK, ALWAYS_CORNERS)]
    fadein_image = fadein_image or settings.fadein_image
    fadeout_image = fadeout_image or settings.fadeout_image
    # melt resolves relative resource paths against the project file's own directory, not the
    # caller's cwd, so every path written into the XML must be absolute.
    original = original.resolve()
    watermarks = [(image.resolve(), corners) for image, corners in watermarks]
    fadein_image = fadein_image.resolve() if fadein_image else None
    fadeout_image = fadeout_image.resolve() if fadeout_image else None

    name = original.name
    started = time.monotonic()

    def say(message: str) -> None:
        _emit(progress, f"{name}: {message}")

    say("probing")
    probe = probe_video(original)
    fps = probe.fps_num / probe.fps_den
    clip = clip_range(probe, settings)
    windows = plan_windows(probe, settings, clip)
    clip_first, clip_frames = clip
    say(
        f"recording is {probe.total_frames / fps:.2f}s; clip is {clip_frames / fps:.2f}s "
        f"long, starting at {clip_first / fps:.2f}s ({probe.width}x{probe.height}, {fps:.3f} fps)"
    )
    for k, (start, end) in enumerate(windows, start=1):
        say(
            f"slowdown {k}/{len(windows)}: clip {(start - clip_first) / fps:.2f}-{(end - clip_first) / fps:.2f}s "
            f"at {settings.speed}x"
        )
    if settings.slowdowns and not windows:
        say("no slowdown falls inside the clip")
    say(
        f"fade-in: {fadein_image.name if fadein_image else 'off'}, "
        f"fade-out: {fadeout_image.name if fadeout_image else 'off'}, "
        f"watermarks: {len(watermarks)} image(s)"
    )

    smoothed: list[Path | None] = [None] * len(windows)
    if settings.smoothing == "high" and windows:
        say(f"smoothing {len(windows)} slowdown(s) with ffmpeg in parallel (high quality, slow)")

        def smooth_one(item: tuple[int, tuple[int, int]]) -> Path:
            k, (start, end) = item
            label = f"slowdown {k}/{len(windows)}"
            return smooth_segment(
                original,
                probe,
                start,
                end,
                settings.speed,
                progress=lambda m: _emit(progress, f"{name}: {label}: {m}"),
                chunks=settings.chunks,
            )

        # The windows are rendered in parallel: each ffmpeg process is single-threaded.
        with ThreadPoolExecutor(max_workers=len(windows)) as pool:
            smoothed = list(pool.map(smooth_one, enumerate(windows, start=1)))
        say("smoothing done")

    music = None
    soundtrack = None
    if settings.soundtrack is not None:
        soundtrack = settings.soundtrack.resolve()
        if not soundtrack.is_file():
            raise FileNotFoundError(f"soundtrack not found: {soundtrack}")
        music = (
            soundtrack,
            round(settings.soundtrack_start * fps),
            round(_duration_seconds(soundtrack) * fps),
        )
    xml, total_frames = build_project_xml(
        original,
        probe,
        watermarks,
        fadein_image,
        fadeout_image,
        clip,
        [(s, e, path) for (s, e), path in zip(windows, smoothed)],
        settings.speed,
        music,
    )
    if music is not None:
        _, music_in, music_len = music
        if music_in + total_frames > music_len:
            say(
                f"warning: soundtrack {soundtrack.name} ends after "
                f"{(music_len - music_in) / fps:.1f}s from the start point; the rest of the video is silent"
            )
        say(f"soundtrack {soundtrack.name} from {settings.soundtrack_start:g}s")

    # The project is kept next to the rendered video (<name>_watermarked.mlt) so it can be
    # opened in Shotcut and edited afterwards. It is never deleted.
    project_path = output.with_suffix(".mlt")
    project_path.write_text(xml, encoding="utf-8")
    # The expected length, so the upload sync can tell a finished render from a truncated one
    # even when the clip was trimmed (and so is shorter than its original).
    output.with_suffix(".edit.json").write_text(
        json.dumps({"expected_seconds": total_frames * probe.fps_den / probe.fps_num}) + "\n", encoding="utf-8"
    )
    # Render to a temporary name and rename only when melt succeeds, so an interrupted render
    # never leaves a truncated file under the final name (the upload sync would pick it up).
    temp_output = output.with_name(output.stem + ".rendering.mp4")
    cmd = [
        MELT,
        str(project_path),
        "-consumer",
        f"avformat:{temp_output}",
        "vcodec=libx264",
        "crf=18",
        "preset=medium",
        "acodec=aac",
        "ab=192k",
        "pix_fmt=yuv420p",
        "movflags=+faststart",
    ]
    say(f"rendering {total_frames} frames with melt")
    try:
        _run_melt(cmd, progress=lambda m: _emit(progress, f"{name}: rendering {m}"))
    except subprocess.CalledProcessError as e:
        logger.error("melt failed for %s:\n%s", original, e.output)
        temp_output.unlink(missing_ok=True)
        raise
    except BaseException:
        temp_output.unlink(missing_ok=True)  # e.g. the app was stopped mid-render
        raise
    os.replace(temp_output, output)
    say(f"wrote {output.name} in {time.monotonic() - started:.0f}s")
    return output


def watermarks_for(image: str | None) -> list[tuple[Path, tuple[str, ...]]]:
    """Return the always-used watermark, plus `image` (a file name) in IMAGE_CORNERS if given."""
    watermarks = [(ALWAYS_WATERMARK, ALWAYS_CORNERS)]
    if image:
        watermarks.append((Path(image), IMAGE_CORNERS))
    return watermarks
