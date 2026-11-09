"""Watermark and edit recordings with Shotcut's engine, as a script or from the livestream app.

Public API (used by sw/livestream.py and the command line):
    add_watermark(video, output, watermarks=None, settings=None) -> Path
    settings_for_recording(fadein_image, fadeout_image) -> EditSettings
    watermarks_for(image) -> list of (image, corners)

Settings (EditSettings in sw/shotcut_pipeline.py) are read from sw/livestream_config.json and
can be overridden per run: from the command line, or in the window (--gui).

Keys in livestream_config.json (all optional):
    "slowdowns":    "8-12, 14-16"   slow-motion windows, seconds from the clip start
    "speed":        0.3             speed inside a slowdown
    "clip_start":   0               where the clip starts in the recording, seconds
    "clip_length":  ""              clip length in seconds; "" = the rest of the recording
    "smoothing":    "high"          "high" (ffmpeg minterpolate) or "shotcut" (MLT timewarp)
    "fadein_image", "fadeout_image", "image": file names in the current directory
    "exposure":     0               brightness of the recording, stops: -3..3 (negative = darker)
    "soundtrack":   ""              audio file that replaces the recording's sound; "" = keep it
    "soundtrack_start": 0           where in the soundtrack file the video starts, seconds

Usage:
    uv run python -m sw.shotcut_watermark VIDEO [OUTPUT] [--slowdowns "8-12,14-16"] ...
    uv run python -m sw.shotcut_watermark --folder videos/likegym20261108
    uv run python -m sw.shotcut_watermark --gui
"""

from __future__ import annotations

import argparse
import json
import logging
import queue
import threading
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from pathlib import Path
from typing import Any

from sw.shotcut_pipeline import DEFAULT_CHUNKS, SMOOTHING_MODES, EditSettings, ProgressFn, edit_clip, watermarks_for

logger = logging.getLogger(__name__)

CONFIG_FILE = Path(__file__).resolve().parent / "livestream_config.json"
DEFAULT_JOBS = 4  # Videos edited at the same time in a batch
ORIGINAL_SUFFIX = "_original.mp4"
WATERMARKED_SUFFIX = "_watermarked.mp4"


def parse_slowdowns(text: str) -> tuple[tuple[float, float], ...]:
    """Parse "8-12, 14-16" into ((8.0, 12.0), (14.0, 16.0)). Empty text means no slowdowns."""
    windows = []
    for part in text.replace(";", ",").split(","):
        part = part.strip()
        if not part:
            continue
        start_text, sep, end_text = part.partition("-")
        if not sep:
            raise ValueError(f"slowdown {part!r} must look like START-END, for example 8-12")
        try:
            window = (float(start_text), float(end_text))
        except ValueError:
            raise ValueError(f"slowdown {part!r} must look like START-END, for example 8-12") from None
        if not 0 <= window[0] < window[1]:
            raise ValueError(f"slowdown {part!r} must have 0 <= START < END")
        windows.append(window)
    return tuple(windows)


def format_slowdowns(windows: tuple[tuple[float, float], ...]) -> str:
    return ", ".join(f"{start:g}-{end:g}" for start, end in windows)


def _optional_float(text: str | float | None, name: str) -> float | None:
    if text is None or (isinstance(text, str) and not text.strip()):
        return None
    try:
        return float(text)
    except (TypeError, ValueError):
        raise ValueError(f"{name} must be a number of seconds, not {text!r}") from None


def settings_from(values: dict[str, Any]) -> EditSettings:
    """Build EditSettings from text or number values (as typed in the textboxes or stored in JSON).

    Raises ValueError with a message naming the bad field.
    """
    speed = _optional_float(values.get("speed", 0.3), "speed")
    clip_start = _optional_float(values.get("clip_start", 0), "clip_start")
    smoothing = str(values.get("smoothing", "high")).strip() or "high"
    if smoothing not in SMOOTHING_MODES:
        raise ValueError(f"smoothing must be one of {', '.join(SMOOTHING_MODES)}, not {smoothing!r}")
    chunks_text = _optional_float(values.get("chunks", DEFAULT_CHUNKS), "chunks")
    if chunks_text is None or chunks_text < 1 or chunks_text != int(chunks_text):
        raise ValueError(f"chunks must be a whole number of at least 1, not {values.get('chunks')!r}")
    exposure = _optional_float(values.get("exposure", 0), "exposure") or 0.0
    soundtrack = str(values.get("soundtrack") or "").strip()
    soundtrack_start = _optional_float(values.get("soundtrack_start", 0), "soundtrack_start") or 0.0
    fadein = str(values.get("fadein_image") or "").strip()
    fadeout = str(values.get("fadeout_image") or "").strip()
    try:
        return EditSettings(
            slowdowns=parse_slowdowns(str(values["slowdowns"])) if values.get("slowdowns") is not None else EditSettings().slowdowns,
            speed=speed if speed is not None else 0.3,
            clip_start=clip_start or 0.0,
            clip_length=_optional_float(values.get("clip_length"), "clip_length"),
            smoothing=smoothing,
            chunks=int(chunks_text),
            fadein_image=Path(fadein) if fadein else None,
            fadeout_image=Path(fadeout) if fadeout else None,
            exposure=exposure,
            soundtrack=Path(soundtrack) if soundtrack else None,
            soundtrack_start=soundtrack_start,
        )
    except ValueError as e:
        raise ValueError(str(e)) from None


def load_config(config_file: Path = CONFIG_FILE) -> dict[str, Any]:
    """Return the stored values, or {} if the file is missing or unreadable."""
    try:
        data = json.loads(config_file.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return {}
    except (OSError, ValueError):
        logger.warning("Ignoring unreadable config file %s", config_file, exc_info=True)
        return {}
    return data if isinstance(data, dict) else {}


def save_config(values: dict[str, Any], config_file: Path = CONFIG_FILE) -> None:
    """Merge `values` into the config file, keeping keys this module does not own (such as folder)."""
    data = load_config(config_file)
    data.update(values)
    try:
        config_file.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    except OSError:
        logger.exception("Could not save the config file %s", config_file)


def settings_for_recording(fadein_image: Path | None, fadeout_image: Path | None) -> EditSettings:
    """Settings for a live recording: the stored config (slowdowns, exposure, soundtrack, ...) plus
    the fade stills of the livestream app. A config value that cannot be used falls back to defaults."""
    try:
        settings = settings_from(load_config())
    except ValueError:
        logger.exception("Ignoring the edit settings in %s; using the defaults", CONFIG_FILE)
        settings = EditSettings()
    return replace(settings, fadein_image=fadein_image, fadeout_image=fadeout_image)


def add_watermark(
    video: Path,
    output: Path,
    watermarks: list[tuple[Path, tuple[str, ...]]] | None = None,
    settings: EditSettings | None = None,
    progress: ProgressFn | None = None,
) -> Path:
    """Write an edited, watermarked copy of `video` to `output`.

    Args:
        video: Input video file.
        output: Output video file. Must differ from `video`.
        watermarks: Pairs of (image, corners). Defaults to the always-used watermark.
        settings: Clip, slowdown, smoothing and fade settings. Defaults to EditSettings().
        progress: Called with each progress message (also logged).

    Returns:
        The output path.
    """
    if watermarks is None:
        # The extra watermark image stored in the config (the always-used selkabot.png is added too).
        watermarks = watermarks_for(str(load_config().get("image") or "").strip() or None)
    return edit_clip(video, output, watermarks, settings=settings, progress=progress)


def watermarked_name(original: Path) -> Path:
    """`X_original.mp4` -> `X_watermarked.mp4`, next to it (the livestream app's naming)."""
    if not original.name.endswith(ORIGINAL_SUFFIX):
        return original.with_name(original.stem + WATERMARKED_SUFFIX)
    return original.with_name(original.name[: -len(ORIGINAL_SUFFIX)] + WATERMARKED_SUFFIX)


def batch(
    folder: Path,
    settings: EditSettings,
    watermarks: list | None = None,
    redo: bool = False,
    jobs: int = DEFAULT_JOBS,
    progress: ProgressFn | None = None,
) -> list[Path]:
    """Edit every `*_original.mp4` in `folder` that has no watermarked version yet (or all, with redo).

    `jobs` videos are rendered at the same time. Smoothing is single-threaded per segment, so
    a few parallel jobs use the machine much better than one at a time.
    """
    todo = []
    for original in sorted(folder.glob(f"*{ORIGINAL_SUFFIX}")):
        output = watermarked_name(original)
        if output.exists() and not redo:
            _emit(progress, f"skipping {original.name}: {output.name} exists")
            continue
        todo.append((original, output))
    if not todo:
        _emit(progress, f"nothing to edit in {folder}")
        return []
    _emit(progress, f"{len(todo)} video(s) to edit, {max(1, jobs)} at a time")

    def edit(pair: tuple[Path, Path]) -> Path:
        original, output = pair
        return add_watermark(original, output, watermarks, settings, progress)

    with ThreadPoolExecutor(max_workers=max(1, jobs)) as pool:
        done = list(pool.map(edit, todo))
    _emit(progress, f"batch finished: {len(done)} video(s) written")
    return done


def _emit(progress: ProgressFn | None, message: str) -> None:
    logger.info(message)
    if progress is not None:
        progress(message)


def _settings_values(args: argparse.Namespace, stored: dict[str, Any]) -> dict[str, Any]:
    """Stored config values, overridden by any flag given on the command line."""
    values = dict(stored)
    for name in (
        "exposure",
        "slowdowns",
        "speed",
        "clip_start",
        "clip_length",
        "smoothing",
        "chunks",
        "fadein_image",
        "fadeout_image",
        "soundtrack",
        "soundtrack_start",
    ):
        value = getattr(args, name)
        if value is not None:
            values[name] = value
    return values


def run_gui(stored: dict[str, Any]) -> None:
    """A small window with one textbox per setting; Run uses the textboxes and saves them."""
    import tkinter as tk
    from tkinter import ttk

    root = tk.Tk()
    root.title("Shotcut watermark")
    frame = ttk.Frame(root, padding=10)
    frame.grid()

    fields = [
        ("Slowdowns (start-end, …)", "slowdowns", format_slowdowns(settings_from(stored).slowdowns)),
        ("Speed in slowdowns", "speed", str(stored.get("speed", 0.3))),
        ("Clip start (s)", "clip_start", str(stored.get("clip_start", 0))),
        ("Clip length (s, empty = rest)", "clip_length", "" if stored.get("clip_length") in (None, "") else str(stored["clip_length"])),
        ("Fade-in image", "fadein_image", str(stored.get("fadein_image") or "")),
        ("Fade-out image", "fadeout_image", str(stored.get("fadeout_image") or "")),
        ("Soundtrack file", "soundtrack", str(stored.get("soundtrack") or "")),
        ("Watermark image (top-left, bottom-right)", "image", str(stored.get("image") or "")),
        ("Soundtrack start (s)", "soundtrack_start", str(stored.get("soundtrack_start", 0))),
        ("Video (single run)", "video", ""),
        ("Folder (batch run)", "folder", str(stored.get("folder") or "")),
    ]
    entries: dict[str, tk.StringVar] = {}
    for row, (label, key, initial) in enumerate(fields):
        ttk.Label(frame, text=label).grid(row=row, column=0, sticky="w", pady=2)
        var = tk.StringVar(value=initial)
        ttk.Entry(frame, textvariable=var, width=40).grid(row=row, column=1, sticky="ew", pady=2)
        entries[key] = var
    smoothing = tk.StringVar(value=str(stored.get("smoothing", "high")))
    ttk.Label(frame, text="Smoothing").grid(row=len(fields), column=0, sticky="w", pady=2)
    ttk.Combobox(frame, textvariable=smoothing, values=list(SMOOTHING_MODES), state="readonly").grid(
        row=len(fields), column=1, sticky="w", pady=2
    )
    chunks = tk.StringVar(value=str(stored.get("chunks", DEFAULT_CHUNKS)))
    ttk.Label(frame, text="Parallel chunks per slowdown").grid(row=len(fields) + 1, column=0, sticky="w", pady=2)
    ttk.Entry(frame, textvariable=chunks, width=6).grid(row=len(fields) + 1, column=1, sticky="w", pady=2)
    exposure = tk.DoubleVar(value=float(stored.get("exposure", 0) or 0))
    ttk.Label(frame, text="Exposure (- darker, + brighter)").grid(row=len(fields) + 2, column=0, sticky="w", pady=2)
    tk.Scale(
        frame,
        variable=exposure,
        from_=-2,
        to=2,
        resolution=0.1,
        orient="horizontal",
        length=240,
        showvalue=True,
    ).grid(row=len(fields) + 2, column=1, sticky="w", pady=2)
    status = tk.StringVar(value="Ready")
    ttk.Label(frame, textvariable=status, wraplength=420).grid(row=len(fields) + 4, column=0, columnspan=2, sticky="w")

    def values() -> dict[str, Any]:
        result: dict[str, Any] = {key: var.get() for key, var in entries.items()}
        result["smoothing"] = smoothing.get()
        result["chunks"] = chunks.get()
        result["exposure"] = round(exposure.get(), 2)
        return result

    updates: queue.Queue[str] = queue.Queue()
    running = threading.Event()

    def show_updates() -> None:
        latest = None
        while True:
            try:
                latest = updates.get_nowait()
            except queue.Empty:
                break
        if latest is not None:
            status.set(latest)

    def poll() -> None:
        show_updates()
        if running.is_set():
            root.after(200, poll)
        else:
            show_updates()  # The final message is queued before `running` is cleared
            button.state(["!disabled"])

    def run() -> None:
        try:
            vals = values()
            settings = settings_from(vals)
        except ValueError as e:
            status.set(str(e))
            return
        video = Path(vals["video"].strip()) if vals["video"].strip() else None
        watermarks = watermarks_for(vals["image"].strip() or None)
        folder = Path(vals["folder"].strip()) if vals["folder"].strip() else None
        if video is None and folder is None:
            status.set("Give a video or a folder")
            return
        save_config({k: v for k, v in vals.items() if k not in ("video", "folder")} | ({"folder": vals["folder"].strip()} if folder else {}))
        button.state(["disabled"])
        running.set()
        status.set("Starting…")
        logger.info("Window: starting %s with %s", video or folder, settings)

        def work() -> None:
            try:
                if video:
                    output = watermarked_name(video)
                    add_watermark(video, output, watermarks, settings, progress=updates.put)
                    message = f"Done: wrote {output}"
                else:
                    done = batch(folder, settings, watermarks, progress=updates.put)
                    message = f"Done: wrote {len(done)} video(s) to {folder}"
            except Exception as e:  # pylint: disable=broad-except
                logger.exception("Render failed")
                message = f"Failed: {e}"
            updates.put(message)
            running.clear()

        threading.Thread(target=work, daemon=True).start()
        poll()

    button = ttk.Button(frame, text="Run", command=run)
    button.grid(row=len(fields) + 3, column=0, columnspan=2, sticky="ew", pady=(8, 0))
    root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(description="Watermark and edit recordings with Shotcut's engine.")
    parser.add_argument("video", nargs="?", type=Path, help="A recording to edit (…_original.mp4)")
    parser.add_argument("output", nargs="?", type=Path, help="Output file (default: …_watermarked.mp4)")
    parser.add_argument("--folder", type=Path, help="Edit every …_original.mp4 in this folder")
    parser.add_argument("--redo", action="store_true", help="With --folder, also redo videos already edited")
    parser.add_argument("--jobs", type=int, default=DEFAULT_JOBS, help=f"With --folder, videos rendered at once (default {DEFAULT_JOBS})")
    parser.add_argument("--gui", action="store_true", help="Open the window with textboxes for the settings")
    parser.add_argument("--slowdowns", help='Slow-motion windows, e.g. "8-12, 14-16" (seconds from clip start)')
    parser.add_argument("--speed", type=float, help="Speed inside slowdowns (default 0.3)")
    parser.add_argument("--clip-start", dest="clip_start", type=float, help="Clip start in the recording (s)")
    parser.add_argument("--clip-length", dest="clip_length", type=float, help="Clip length (s); default: the rest")
    parser.add_argument("--smoothing", choices=SMOOTHING_MODES, help="high (ffmpeg, slow) or shotcut (instant)")
    parser.add_argument("--exposure", type=float, help="Brightness of the recording in stops, -3 to 3 (negative = darker)")
    parser.add_argument("--chunks", type=int, help=f"Parallel chunks per slowdown in high smoothing (default {DEFAULT_CHUNKS})")
    parser.add_argument("--fadein-image", dest="fadein_image", help="Still for the opening fade (file name)")
    parser.add_argument("--fadeout-image", dest="fadeout_image", help="Still for the closing fade (file name)")
    parser.add_argument("--soundtrack", help="Audio file that replaces the recording's sound")
    parser.add_argument("--soundtrack-start", dest="soundtrack_start", type=float, help="Where the video starts in the soundtrack (s)")
    parser.add_argument("--image", help="Extra watermark image (file name), in addition to selkabot.png")
    parser.add_argument("--save", action="store_true", help="Store these settings in the config file")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
    stored = load_config()
    values = _settings_values(args, stored)
    if args.gui:
        run_gui(values)
        return
    try:
        settings = settings_from(values)
    except ValueError as e:
        parser.error(str(e))
    image = args.image if args.image is not None else stored.get("image")
    watermarks = watermarks_for(str(image or "").strip() or None)
    if args.save:
        save_config(values)

    if args.folder:
        done = batch(args.folder, settings, watermarks, args.redo, args.jobs)
        print(f"wrote {len(done)} video(s)")
    elif args.video:
        output = args.output or watermarked_name(args.video)
        add_watermark(args.video, output, watermarks, settings)
        print(f"wrote {output} and {output.with_suffix('.mlt')}")
    else:
        parser.error("give a VIDEO, --folder, or --gui")


if __name__ == "__main__":
    main()
