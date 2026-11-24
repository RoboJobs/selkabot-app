"""GoPro livestream recorder — preview, record and download over USB.

Connects to a GoPro HERO10 over the wired (USB) interface with the Open GoPro SDK, shows
the camera's preview stream in a Tkinter window and lets the user start/stop video
recording. Each recorded clip is downloaded to the local computer as soon as recording
stops, into the folder named in the "Folder" text box (a subfolder of the output
directory, created if missing). The last folder and watermark image used are stored in
sw/livestream_config.json and restored on startup; without that file the folder is
"default" and no extra watermark image is used.

Threading model: - Main thread: Tkinter UI. Only this thread touches widgets. - Asyncio
thread: owns the WiredGoPro client; all camera commands run here. - Post-process thread:
watermarks (and optionally uploads) clips one at a time,   in the order they were
downloaded, so the camera is free to record meanwhile. - Frame thread: reads the UDP
preview stream with OpenCV, keeps latest frame. Results from the asyncio thread reach
the UI through a queue polled by Tk.

Notes:
- The preview stream is a low resolution live view; the recorded file is in
  the selected video mode (2.7K 16:9 at 60 fps, no zoom, Wide lens by default).
  Modes come from sw/camera/hero10_table.py and are chosen with the cascading
  ratio / resolution / fps / lens / zoom rows.
- The mode can only be changed while not recording or downloading. The
  preview stream is restarted around the change. The last accepted mode is
  saved in livestream_config.json.
- The preview is paused while recording. On HERO10 firmware H21.01.01.62.00,
  starting a recording while the USB preview is running kills the preview
  until the camera is power cycled. Stopping the preview first, and starting
  it again once the clip is written, avoids this.
- The SDK's StreamFeature uses BLE to start the preview stream, which is not
  available on WiredGoPro, so the HTTP preview command is used directly.

Usage:     uv run python -m sw.livestream [--output-dir videos] [--port 8554] [--no-
upload]
"""

from __future__ import annotations

import argparse
import asyncio
import concurrent.futures
import dataclasses
import functools
import json
import logging
import math
import os
import queue
import threading
import time
import tkinter as tk
from collections.abc import Callable, Coroutine
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Any

import cv2
import numpy as np
from open_gopro import WiredGoPro
from open_gopro.models import constants, proto
from PIL import Image, ImageTk

from sw import shotcut_watermark
from sw.camera import hero10_table
from sw.camera.gopro_hero10 import HeroBackend
from sw.camera.mode_persistence import load_mode, save_mode
from sw.camera.mode_picker import ModePicker
from sw.camera.modes import AXES, ModeCatalog, VideoMode
from sw.robot import ui_state as us
from sw.robot.motion_runner import MotionRunner, RunnerState
from sw.robot.schema import (
    MotionConfig,
    RobotSettings,
    SchemaError,
    load_motion_config,
    robot_settings_from_dict,
)
from sw.sync_uploads import sync_uploads
from sw.uploader import make_qr, public_url, upload_video

logger = logging.getLogger(__name__)

# Silence ffmpeg's h264 decoder warnings (missing PPS/references) that are
# expected while joining the UDP stream mid-GOP. Read on first VideoCapture.
os.environ.setdefault("OPENCV_FFMPEG_LOGLEVEL", "-8")

DISCRETE_AXES: tuple[str, ...] = AXES[:-1]  # zoom is a slider, not buttons
ZOOM_MAX_PERCENT = 100

MODE_AXIS_TITLES: dict[str, str] = {
    "ratio": "Ratio",
    "resolution": "Resolution",
    "fps": "FPS",
    "lens": "Lens",
    "zoom": "Zoom",
}

DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parents[1] / "videos"
DEFAULT_FOLDER = "default"
DEFAULT_IMAGE = (
    ""  # Optional watermark, a file name in the current directory; "" means none
)
CONFIG_FILE = Path(__file__).resolve().parent / "livestream_config.json"
MOTIONS_DIR = Path(__file__).resolve().parent / "robot" / "motions"
ROBOT_CHECK_TIMEOUT_S = 20.0  # startup check; the SDK can hang on a dead link
ROBOT_REFRESH_TICKS = 30  # re-evaluate "at start" every ~0.5 s of _poll
DISPLAY_WIDTH = 960
KEEP_ALIVE_PERIOD_S = 3.0


@dataclass
class Shared:
    """A downloaded clip after watermarking, upload and QR code creation."""

    video: Path
    url: str | None = None  # None when uploading is disabled
    qr: Path | None = None


@dataclass
class Clip:
    """A downloaded original waiting to be watermarked."""

    original: Path
    watermarked: Path
    qr_file: Path
    key: str  # Bucket key of the watermarked file, e.g. "<folder>/<name>.mp4"
    watermarks: list[tuple[Path, tuple[str, ...]]]
    fadein_image: Path | None
    fadeout_image: Path | None


class PostProcessor:
    """Watermarks, and optionally uploads, clips one at a time on a background thread.

    Clips are handled in the order they were submitted, so recording and downloading can
    continue while earlier clips are still being processed. Results reach the UI through
    `events`, so the callbacks run on the Tk thread.
    """

    def __init__(
        self,
        upload: bool,
        events: queue.Queue[Callable[[], None]],
        on_done: Callable[[Shared], None],
        on_error: Callable[[Clip, Exception], None],
    ) -> None:
        self.upload = upload
        self._events = events
        self._on_done = on_done
        self._on_error = on_error
        self._jobs: queue.Queue[Callable[[], Any] | None] = queue.Queue()
        # Not a daemon thread: the process waits for queued jobs to finish before
        # exiting.
        self._thread = threading.Thread(target=self._run, name="postprocess")
        self._thread.start()

    def submit(self, clip: Clip) -> None:
        self._jobs.put(functools.partial(self._process, clip))

    def submit_job(self, job: Callable[[], Any]) -> None:
        """Run `job` on the worker, after the jobs already queued."""
        self._jobs.put(job)

    def stop(self) -> None:
        """Queue a stop after the jobs already submitted.

        Returns without waiting for them.
        """
        logger.info("Finishing %d queued job(s) before exiting", self._jobs.qsize())
        self._jobs.put(None)

    def _run(self) -> None:
        while (job := self._jobs.get()) is not None:
            try:
                job()
            except Exception:  # pylint: disable=broad-except
                logger.exception("Background job failed")

    def _process(self, clip: Clip) -> None:
        try:
            shotcut_watermark.add_watermark(
                clip.original,
                clip.watermarked,
                clip.watermarks,
                shotcut_watermark.settings_for_recording(
                    clip.fadein_image, clip.fadeout_image
                ),
            )
            shared = Shared(video=clip.watermarked)
            if self.upload:
                # The URL and QR code were made from clip.key when the download finished
                # (see
                # LivestreamApp._on_recording_stopped), so they are not recomputed here.
                upload_video(clip.watermarked, clip.key)
                shared = Shared(
                    video=clip.watermarked, url=public_url(clip.key), qr=clip.qr_file
                )
        except Exception as e:  # pylint: disable=broad-except
            logger.exception("Post-processing %s failed", clip.original)
            self._events.put(functools.partial(self._on_error, clip, e))
        else:
            self._events.put(functools.partial(self._on_done, shared))


class PreviewReader:
    """Reads the GoPro UDP preview stream in a background thread.

    Only the most recent frame is kept so the display never lags behind. The capture is
    reopened if the stream drops (e.g. while it is restarted after a resolution change).
    """

    def __init__(self, port: int) -> None:
        self._url = f"udp://127.0.0.1:{port}?overrun_nonfatal=1&fifo_size=50000000"
        self._frame: Any = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="preview-reader", daemon=True
        )

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def latest_frame(self) -> Any:
        with self._lock:
            frame, self._frame = self._frame, None
        return frame

    def _run(self) -> None:
        while not self._stop.is_set():
            cap = cv2.VideoCapture(self._url, cv2.CAP_FFMPEG)
            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    break
                with self._lock:
                    self._frame = frame
            cap.release()
            if not self._stop.is_set():
                logger.debug("Preview stream read failed; reopening")
                time.sleep(0.5)


class GoProController:
    """Runs all camera operations on a dedicated asyncio event loop thread."""

    def __init__(self, output_dir: Path, port: int, serial: str | None) -> None:
        self.output_dir = output_dir
        self.port = port
        self._serial = serial
        self._gopro: WiredGoPro | None = None
        self._clip_dir = output_dir  # Folder for the clips of the current recording
        self._watermarks = shotcut_watermark.watermarks_for(
            None
        )  # Watermarks of the current recording
        self._fadein_image: Path | None = None
        self._fadeout_image: Path | None = None
        self._media_before: set[str] = set()
        self._recording = False
        self._keep_alive_task: asyncio.Task | None = None
        self._backend: HeroBackend | None = None
        self.catalog: ModeCatalog = hero10_table.build_catalog()
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=self._loop.run_forever, name="gopro-loop", daemon=True
        )
        self._thread.start()

    def submit(self, coro: Coroutine[Any, Any, Any]) -> concurrent.futures.Future:
        """Schedule a coroutine on the camera loop from any thread."""
        return asyncio.run_coroutine_threadsafe(coro, self._loop)

    @property
    def gopro(self) -> WiredGoPro:
        assert self._gopro is not None, "GoPro is not connected"
        return self._gopro

    async def connect(self, mode: VideoMode) -> str:
        """Connect over USB, configure video mode and start the preview stream.

        Returns:     The camera serial number.
        """
        self._gopro = WiredGoPro(self._serial)
        await self._gopro.open(timeout=10, retries=3)
        # A previous session may have left the preview running, which blocks
        # setting changes.
        await self._stop_preview()
        _check(
            await self.gopro.http_command.load_preset_group(
                group=proto.EnumPresetGroup.PRESET_GROUP_ID_VIDEO
            )
        )
        self._backend = HeroBackend(self.gopro, self.catalog)
        await self._backend.apply_mode(mode)
        await self._start_preview()
        self._keep_alive_task = self._loop.create_task(self._keep_alive())
        # get_camera_name is not supported by HERO10 firmware (404)
        return self.gopro.identifier

    async def set_mode(self, mode: VideoMode) -> None:
        """Change the recording mode, restarting the preview around it."""
        await self._stop_preview()
        try:
            assert self._backend is not None, "connect() must run before set_mode()"
            await self._backend.apply_mode(mode)
        finally:
            await self._start_preview()

    async def start_recording(
        self,
        clip_dir: Path,
        image: str | None,
        fadein_image: Path | None,
        fadeout_image: Path | None,
    ) -> None:
        """Pause the preview and start recording (see module notes).

        The clips of this recording are downloaded to `clip_dir`, which is created if
        missing, and edited with Shotcut's engine: watermarked with `image` (a file name
        in the current directory) in addition to the always-used watermark, and faded
        in/out via `fadein_image`/`fadeout_image` (None for no fade on that side).
        """
        clip_dir.mkdir(parents=True, exist_ok=True)
        self._clip_dir = clip_dir
        self._watermarks = shotcut_watermark.watermarks_for(image)
        self._fadein_image = fadein_image
        self._fadeout_image = fadeout_image
        self._media_before = await self._media_filenames()
        await self._stop_preview()
        try:
            _check(
                await self.gopro.http_command.set_shutter(
                    shutter=constants.Toggle.ENABLE
                )
            )
        except Exception:
            await self._start_preview()
            raise
        self._recording = True

    async def stop_recording(self) -> list[str]:
        """Stop recording, resume the preview and return the new media files on the
        camera."""
        _check(
            await self.gopro.http_command.set_shutter(shutter=constants.Toggle.DISABLE)
        )
        self._recording = False
        # Waits internally until the camera has finished encoding the file.
        # Long recordings are split into several chapter files.
        new_files = sorted((await self._media_filenames()) - self._media_before)
        await self._start_preview()
        return new_files

    async def download(
        self, camera_files: list[str], progress: Callable[[str], None]
    ) -> list[Clip]:
        """Download each camera file and return the clips to post-process.

        The original download is kept as `<name>_original.mp4`; the watermarked copy is
        `<name>_watermarked.mp4`, written later by the PostProcessor.
        """
        self._clip_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        clips: list[Clip] = []
        for i, camera_file in enumerate(camera_files, start=1):
            label = f"({i}/{len(camera_files)})"
            base = f"{stamp}_{Path(camera_file).stem}"
            original = self._clip_dir / f"{base}_original.mp4"
            watermarked = self._clip_dir / f"{base}_watermarked.mp4"
            qr_file = self._clip_dir / f"{base}_qr.png"

            progress(f"Downloading {camera_file} {label}…")
            _check(
                await self.gopro.http_command.download_file(
                    camera_file=camera_file, local_file=original
                )
            )
            key = watermarked.relative_to(self.output_dir).as_posix()
            clips.append(
                Clip(
                    original=original,
                    watermarked=watermarked,
                    qr_file=qr_file,
                    key=key,
                    watermarks=self._watermarks,
                    fadein_image=self._fadein_image,
                    fadeout_image=self._fadeout_image,
                )
            )
        return clips

    async def shutdown(self) -> None:
        """Stop any recording, the preview stream and the loop."""
        if self._keep_alive_task is not None:
            self._keep_alive_task.cancel()
        if self._gopro is not None:
            try:
                if self._recording:
                    await self.gopro.http_command.set_shutter(
                        shutter=constants.Toggle.DISABLE
                    )
                await self.gopro.http_command.set_preview_stream(
                    mode=constants.Toggle.DISABLE
                )
                await self.gopro.close()
            except Exception:  # pylint: disable=broad-except
                logger.exception("Error while closing the GoPro connection")
        self._loop.call_soon(self._loop.stop)

    async def _stop_preview(self) -> None:
        # Not checked: the camera returns an error if no stream is running.
        await self.gopro.http_command.set_preview_stream(mode=constants.Toggle.DISABLE)

    async def _start_preview(self) -> None:
        # Callers stop the preview first: starting while it is running fails.
        _check(
            await self.gopro.http_command.set_preview_stream(
                mode=constants.Toggle.ENABLE, port=self.port
            )
        )

    async def _media_filenames(self) -> set[str]:
        return {
            item.filename
            for item in (await self.gopro.http_command.get_media_list()).data.files
        }

    async def _keep_alive(self) -> None:
        while True:
            await asyncio.sleep(KEEP_ALIVE_PERIOD_S)
            # Commands block while encoding, and the camera stays awake while
            # recording anyway.
            if self._recording:
                continue
            try:
                await self.gopro.http_command.set_keep_alive()
            except Exception:  # pylint: disable=broad-except
                logger.warning("Keep alive failed", exc_info=True)


def _check(response: Any) -> None:
    if not response.ok:
        raise RuntimeError(f"{response.identifier} failed: {response.status}")


def folder_error(name: str) -> str | None:
    """Return why `name` cannot be used as a recording folder, or None if it can."""
    if not name:
        return "enter a folder name."
    if name in (".", "..") or Path(name).name != name or "\\" in name:
        return "the folder name must be a single name, without / or \\."
    return None


def image_error(name: str) -> str | None:
    """Return why `name` cannot be used as the watermark image name, or None if the
    format is fine.

    An empty name is fine: it means no extra watermark image.
    """
    if not name:
        return None
    if Path(name).name != name or "\\" in name:
        return "the image name must be a single file name, without / or \\."
    return None


def image_missing(name: str) -> bool:
    """True if the watermark image `name` is set but is not a file in the current
    directory."""
    return bool(name) and not (Path.cwd() / name).is_file()


def _stored(
    data: dict, key: str, default: str, error: Callable[[str], str | None]
) -> str:
    value = data.get(key)
    if isinstance(value, str) and error(value.strip()) is None:
        return value.strip()
    if value is not None:
        logger.warning("Ignoring invalid %r in the config file; using %r", key, default)
    return default


def load_config(config_file: Path) -> tuple[str, str, str, str]:
    """Return (folder, image, fadein_image, fadeout_image) stored in the config file.

    A missing file, or a value that is missing or unusable, falls back to its default.
    fadein_image/fadeout_image name the stills used for the Shotcut edit pipeline's
    opening and closing fades; like `image`, "" means none and there is no UI box for
    them yet, so they are only ever set by editing the config file directly.
    """
    try:
        data = json.loads(config_file.read_text(encoding="utf-8"))
    except FileNotFoundError:
        data = {}
    except (OSError, ValueError):
        logger.warning("Ignoring unreadable config file %s", config_file, exc_info=True)
        data = {}
    if not isinstance(data, dict):
        data = {}
    return (
        _stored(data, "folder", DEFAULT_FOLDER, folder_error),
        _stored(data, "image", DEFAULT_IMAGE, image_error),
        _stored(data, "fadein_image", DEFAULT_IMAGE, image_error),
        _stored(data, "fadeout_image", DEFAULT_IMAGE, image_error),
    )


def save_config(
    config_file: Path, folder: str, image: str, fadein_image: str, fadeout_image: str
) -> None:
    """Store the recording folder, watermark image and fade stills in the config
    file."""
    try:
        # Merge into the existing file so keys this app does not edit (such as the
        # slowdown and
        # clip settings read by sw/shotcut_watermark.py) are kept.
        try:
            config = json.loads(config_file.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            config = {}
        if not isinstance(config, dict):
            config = {}
        config.update(
            {
                "folder": folder,
                "image": image,
                "fadein_image": fadein_image,
                "fadeout_image": fadeout_image,
            }
        )
        config_file.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    except OSError:
        logger.exception("Could not save the config file %s", config_file)


def _read_config(config_file: Path) -> dict:
    try:
        data = json.loads(config_file.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def load_robot_config(config_file: Path) -> tuple[str, Any]:
    """Return (selected motion name or "", raw "robot" block or None)."""
    data = _read_config(config_file)
    motion = data.get("motion", "")
    if not isinstance(motion, str):
        logger.warning("Ignoring invalid 'motion' in the config file")
        motion = ""
    return motion, data.get("robot")


def save_motion(config_file: Path, motion: str) -> None:
    """Store the selected motion ("" for none), keeping every other key."""
    config = _read_config(config_file)
    config["motion"] = motion
    try:
        config_file.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    except OSError:
        logger.exception("Could not save the config file %s", config_file)


def load_motions(motions_dir: Path) -> tuple[dict[str, MotionConfig], dict[str, str]]:
    """Published motions (folders with a motion.json), and the ones whose
    motion.json is unusable with the reason. Folders without one are skipped."""
    configs: dict[str, MotionConfig] = {}
    broken: dict[str, str] = {}
    if not motions_dir.is_dir():
        return configs, broken
    for folder in sorted(motions_dir.iterdir()):
        meta = folder / "motion.json"
        if not (folder.is_dir() and meta.exists()):
            continue
        try:
            config = load_motion_config(meta)
            if config.name != folder.name:
                raise SchemaError(meta, "name", "must equal the folder name")
            config.check_consistent()
        except (SchemaError, OSError, TypeError, KeyError) as e:
            broken[folder.name] = f"invalid motion.json: {e}"
            continue
        configs[config.name] = config
    return configs, broken


class LivestreamApp:
    """Tkinter window with the live preview, resolution selector, folder box and record
    button."""

    def __init__(
        self,
        root: tk.Tk,
        controller: GoProController,
        upload: bool,
        folder: str,
        image: str,
        fadein_image: str,
        fadeout_image: str,
        config_file: Path,
        *,
        motion_runner: MotionRunner | None = None,
        motions: dict[str, MotionConfig] | None = None,
        broken_motions: dict[str, str] | None = None,
        motion: str = "",
        robot_settings: RobotSettings | None = None,
    ) -> None:
        self.root = root
        self.controller = controller
        self.config_file = config_file
        # No UI box for these yet; preserved as-is whenever folder/image are saved.
        self.fadein_image = fadein_image
        self.fadeout_image = fadeout_image
        self.reader = PreviewReader(controller.port)
        self._events: queue.Queue[Callable[[], None]] = queue.Queue()
        self.postprocessor = PostProcessor(
            upload, self._events, self._on_processed, self._on_process_error
        )
        # Check the bucket on startup; uploads it finds missing go through the
        # same queue.
        self.postprocessor.submit_job(
            functools.partial(sync_uploads, controller.output_dir, folder, upload)
        )
        self._recording = False
        self._busy = True  # Connecting
        self._record_started = 0.0
        self._photo: ImageTk.PhotoImage | None = None
        self._last_frame: Any = None
        self._qr_window: tk.Toplevel | None = None
        self._init_robot(motion_runner, motions, broken_motions, motion, robot_settings)

        root.title("GoPro Livestream")
        root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.video = ttk.Label(root, anchor="center")
        self.video.pack(fill="both", expand=True, padx=8, pady=8)
        self._show_message("Connecting to GoPro...")

        self.picker = ModePicker(
            controller.catalog,
            load_mode(config_file, controller.catalog),
        )
        modes_frame = ttk.Frame(root)
        modes_frame.pack(fill="x", padx=8, pady=(0, 4))
        self._build_mode_rows(modes_frame)

        if self.motion_runner is not None:
            motion_frame = ttk.Frame(root)
            motion_frame.pack(fill="x", padx=8, pady=(0, 4))
            self._build_motion_row(motion_frame)

        controls = ttk.Frame(root)
        controls.pack(fill="x", padx=8, pady=(0, 8))

        self.record_button = tk.Button(
            controls, text="Start recording", width=16, command=self._on_record
        )
        self.record_button.pack(side="right")
        if self.motion_runner is not None:
            self.robot_button = tk.Button(
                controls, text="Move to Start", width=14, command=self._on_robot_button
            )
            self.robot_button.pack(side="right", padx=(0, 8))

        # Packed right-to-left, so the label sits left of the box, which sits left
        # of the button.
        self.folder = tk.StringVar(value=folder)
        self.folder_entry = ttk.Entry(controls, textvariable=self.folder, width=16)
        self.folder_entry.pack(side="right", padx=(0, 8))
        self.folder_entry.bind("<Return>", self._on_config_commit)
        self.folder_entry.bind("<FocusOut>", self._on_config_commit)
        ttk.Label(controls, text="Folder:").pack(side="right", padx=(8, 4))

        self.image = tk.StringVar(value=image)
        self._saved_config = (folder, image)
        self.image_entry = ttk.Entry(controls, textvariable=self.image, width=14)
        self.image_entry.pack(side="right", padx=(0, 8))
        self.image_entry.bind("<Return>", self._on_config_commit)
        self.image_entry.bind("<FocusOut>", self._on_config_commit)
        ttk.Label(controls, text="Image:").pack(side="right", padx=(8, 4))

        self.status = tk.StringVar()
        ttk.Label(root, textvariable=self.status, anchor="w").pack(
            fill="x", padx=8, pady=(0, 8)
        )

        self._update_controls()
        self._run(
            controller.connect(self.picker.mode),
            "Connecting to GoPro over USB…",
            self._on_connected,
        )
        if self.motion_runner is not None:
            self._start_robot_check()
        root.after(15, self._poll)

    # Camera operations ------------------------------------------------------

    def _run(
        self,
        coro: Coroutine[Any, Any, Any],
        status: str,
        on_done: Callable[[Any], None] | None = None,
        on_error: Callable[[Exception], None] | None = None,
    ) -> None:
        """Run a camera coroutine; callbacks are invoked on the Tk thread."""
        self._busy = True
        self.status.set(status)
        self._update_controls()

        def done(future: concurrent.futures.Future) -> None:
            def finish() -> None:
                self._busy = False
                try:
                    result = future.result()
                except Exception as e:  # pylint: disable=broad-except
                    logger.exception("GoPro operation failed")
                    self.status.set(f"Error: {e}")
                    if on_error:
                        on_error(e)
                else:
                    if on_done:
                        on_done(result)
                self._update_controls()

            self._events.put(finish)

        self.controller.submit(coro).add_done_callback(done)

    def _on_connected(self, serial: str) -> None:
        # ffmpeg needs several seconds to probe the stream before the first frame.
        self._show_message("Starting preview...")
        self.reader.start()
        self.status.set(
            f"Connected to GoPro {serial}. Ready to record at {self.picker.mode.label}."
        )

    def _build_mode_rows(self, parent: ttk.Frame) -> None:
        """Create one row per discrete cascade axis, then a zoom slider.

        Options that the committed mode rules out are disabled, not hidden.
        Zoom is continuous: the slider moves freely, and the camera is only
        changed when the user lets go of it.
        """
        self._mode_vars: dict[str, tk.StringVar] = {}
        self._mode_buttons: dict[str, list[ttk.Radiobutton]] = {}
        for row, axis in enumerate(DISCRETE_AXES):
            ttk.Label(parent, text=MODE_AXIS_TITLES[axis], width=10, anchor="w").grid(
                row=row, column=0, sticky="w"
            )
            var = tk.StringVar(value=self.picker.selected_text(axis))
            self._mode_vars[axis] = var
            self._mode_buttons[axis] = [
                ttk.Radiobutton(
                    parent,
                    text=opt.text,
                    value=opt.text,
                    variable=var,
                    command=functools.partial(self._on_mode_pick, axis),
                )
                for opt in self.picker.options(axis)
            ]
            for column, button in enumerate(self._mode_buttons[axis], start=1):
                button.grid(row=row, column=column, sticky="w", padx=4)

        zoom_row = len(DISCRETE_AXES)
        ttk.Label(parent, text=MODE_AXIS_TITLES["zoom"], width=10, anchor="w").grid(
            row=zoom_row, column=0, sticky="w"
        )
        self._zoom_var = tk.DoubleVar(value=self.picker.mode.zoom)
        self._zoom_scale = ttk.Scale(
            parent,
            from_=0,
            to=ZOOM_MAX_PERCENT,
            variable=self._zoom_var,
            command=self._on_zoom_drag,
        )
        self._zoom_scale.grid(row=zoom_row, column=1, columnspan=6, sticky="we", padx=4)
        self._zoom_scale.bind("<ButtonRelease-1>", self._on_zoom_release)
        self._zoom_scale.bind("<KeyRelease>", self._on_zoom_release)
        self._zoom_label = ttk.Label(parent, text=f"{self.picker.mode.zoom}%", width=6)
        self._zoom_label.grid(row=zoom_row, column=7, sticky="w")
        self._update_mode_states()

    def _on_mode_pick(self, axis: str) -> None:
        try:
            target = self.picker.preview(axis, self._mode_vars[axis].get())
        except ValueError:
            self._refresh_mode_controls()
            return
        self._apply_pick(target)

    def _on_zoom_drag(self, _value: str = "") -> None:
        """While the slider moves, show the value only.
        The camera is not changed yet."""
        self._zoom_label.config(text=f"{round(self._zoom_var.get())}%")

    def _on_zoom_release(self, _event: Any = None) -> None:
        """The slider was let go: apply the zoom the user chose."""
        try:
            target = self.picker.preview_zoom(round(self._zoom_var.get()))
        except ValueError:
            self._refresh_mode_controls()
            return
        self._apply_pick(target)

    def _apply_pick(self, target: VideoMode) -> None:
        """Send a picked mode to the camera; commit it only if the camera accepts it."""
        if target == self.picker.mode:
            self._refresh_mode_controls()
            return

        def done(_: None) -> None:
            self.picker.commit(target)
            save_mode(self.config_file, target)
            self._refresh_mode_controls()
            self.status.set(f"Mode set to {target.label}.")

        def failed(_: Exception) -> None:
            self._refresh_mode_controls()

        self._run(
            self.controller.set_mode(target),
            f"Switching to {target.label}…",
            done,
            failed,
        )

    def _refresh_mode_controls(self) -> None:
        """Show the committed mode on every axis and enable
        only the options it allows."""
        for axis in DISCRETE_AXES:
            self._mode_vars[axis].set(self.picker.selected_text(axis))
        self._zoom_var.set(self.picker.mode.zoom)
        self._zoom_label.config(text=f"{self.picker.mode.zoom}%")
        self._update_mode_states()

    def _update_mode_states(self) -> None:
        locked = self._busy or self._recording
        for axis in DISCRETE_AXES:
            for opt, button in zip(self.picker.options(axis), self._mode_buttons[axis]):
                button.config(
                    state="normal" if opt.enabled and not locked else "disabled"
                )
        _, zoom_max = self.picker.zoom_range()
        self._zoom_scale.config(
            state="normal" if zoom_max > 0 and not locked else "disabled"
        )

    def _folder_name(self) -> str | None:
        """The folder typed in the box, or None (with the reason in the status line) if
        it is unusable."""
        name = self.folder.get().strip()
        error = folder_error(name)
        if error is not None:
            self.status.set(f"Folder: {error}")
            return None
        return name

    def _image_name(self) -> str | None:
        """The watermark image typed in the box, or None (with the reason in the status
        line) if its name is unusable."""
        name = self.image.get().strip()
        error = image_error(name)
        if error is not None:
            self.status.set(f"Image: {error}")
            return None
        return name

    def _on_config_commit(self, _event: Any = None) -> None:
        """Remember changed folder and image names in the config file, so the next start
        uses them."""
        folder = self._folder_name()
        image = self._image_name()
        if folder is None or image is None:
            return
        if (folder, image) == self._saved_config:
            return
        save_config(
            self.config_file, folder, image, self.fadein_image, self.fadeout_image
        )
        self._saved_config = (folder, image)
        if image_missing(image):
            self.status.set(
                f"Image {image!r} not found in {
                    Path.cwd()
                }. Only selkabot.png will be used."
            )
        else:
            self.status.set(
                f"Recordings will be saved in {self.controller.output_dir / folder}"
            )

    def _fade_image_path(self, name: str) -> Path | None:
        """Resolve a fadein_image/fadeout_image config value to a path, or None to skip
        that fade.

        A missing file is treated the same as an empty name (skip), logged rather than
        shown in the UI, since there is no textbox for these yet to show an error in.
        """
        if not name:
            return None
        if image_missing(name):
            logger.warning(
                "Fade image %r not found in %s; skipping that fade", name, Path.cwd()
            )
            return None
        return Path.cwd() / name

    def _on_record(self) -> None:
        if not self._recording:
            if (
                self._motion is not None
                and self._ui_state is not us.UiState.UNAVAILABLE
            ):
                self._robot_event(us.Ev.RECORD_PRESSED)
                return
            self._start_recording()
        elif self._ui_state in us.MOTION_STATES:
            self._robot_event(us.Ev.STOP_PRESSED)
        else:
            self._stop_recording()

    def _start_recording(
        self,
        on_started: Callable[[], None] | None = None,
        on_failed: Callable[[Exception], None] | None = None,
    ) -> bool:
        """Start the GoPro recording. False (status set) if the folder or
        image name is unusable, in which case nothing was started."""
        name = self._folder_name()
        if name is None:
            return False
        image = self._image_name()
        if image is None:
            return False
        self._on_config_commit()
        clip_dir = self.controller.output_dir / name
        if image_missing(image):
            # The image is not used, so the recording only gets the always-used
            # watermarks.
            status = f"Image {image!r} not found in {
                Path.cwd()
            }; only selkabot.png is used. Starting recording…"
            watermark_image = None
        else:
            status = "Starting recording…"
            watermark_image = image or None
        fadein_path = self._fade_image_path(self.fadein_image)
        fadeout_path = self._fade_image_path(self.fadeout_image)

        def started(result: None) -> None:
            self._on_recording_started(result)
            if on_started:
                on_started()

        self._run(
            self.controller.start_recording(
                clip_dir, watermark_image, fadein_path, fadeout_path
            ),
            status,
            started,
            on_failed,
        )
        return True

    def _stop_recording(self) -> None:
        """Stop the GoPro recording, at most once per recording."""
        if self._stop_recording_requested:
            return
        self._stop_recording_requested = True
        self._run(
            self.controller.stop_recording(),
            "Stopping recording…",
            self._on_recording_stopped,
            self._on_stop_recording_failed,
        )

    def _on_stop_recording_failed(self, _error: Exception) -> None:
        self._stop_recording_requested = False  # let the operator press Stop again
        if self._ui_state is us.UiState.STOPPING:
            self._robot_event(us.Ev.RECORDING_STOPPED)

    def _on_recording_started(self, _: None) -> None:
        self._recording = True
        self._stop_recording_requested = False
        self._record_started = time.monotonic()
        self._show_paused()

    def _on_recording_stopped(self, camera_files: list[str]) -> None:
        self._recording = False
        if self._ui_state is us.UiState.STOPPING:
            self._robot_event(us.Ev.RECORDING_STOPPED)
        if not camera_files:
            self.status.set(
                "Recording stopped, but no new file was found on the camera."
            )
            return

        def progress(message: str) -> None:
            self._events.put(lambda: self.status.set(message))

        def done(clips: list[Clip]) -> None:
            link = ""
            for clip in clips:
                if self.postprocessor.upload:
                    # The link depends only on the bucket key, so the QR code is shown
                    # now, before
                    # the watermarking and upload run. The link is live once the upload
                    # finishes.
                    url = public_url(clip.key)
                    make_qr(url, clip.qr_file)
                    self._open_qr_window(
                        Shared(video=clip.watermarked, url=url, qr=clip.qr_file)
                    )
                    link = f" Link (live after upload): {url}"
                self.postprocessor.submit(clip)
            self.status.set(f"Watermarking {len(clips)} clip(s) in background…{link}")

        self._run(
            self.controller.download(camera_files, progress),
            "Transferring video from camera…",
            done,
        )

    def _on_processed(self, shared: Shared) -> None:
        if shared.url is None:
            self.status.set(f"Saved: {shared.video.name}")
            logger.info("Video saved to %s", shared.video)
            return
        self.status.set(f"Uploaded: {shared.url}")
        logger.info(
            "Video %s available at %s (QR: %s)", shared.video, shared.url, shared.qr
        )

    def _on_process_error(self, clip: Clip, error: Exception) -> None:
        self.status.set(f"Error processing {clip.original.name}: {error}")

    # Robot motions ------------------------------------------------------------
    # The flow is driven by sw/robot/ui_state.py; these methods feed it events
    # and carry out the actions it returns. Robot results arrive on worker
    # threads and are posted to self._events, so widgets are only touched here.

    def _init_robot(
        self,
        runner: MotionRunner | None,
        motions: dict[str, MotionConfig] | None,
        broken: dict[str, str] | None,
        motion: str,
        settings: RobotSettings | None,
    ) -> None:
        self.motion_runner = runner
        self.motions = dict(motions or {})
        self.robot_settings = settings or RobotSettings()
        # Disabled motions and why (startup check, broken motion.json, load).
        self._motion_reasons: dict[str, str] = dict(broken or {})
        known = motion in self.motions or motion in self._motion_reasons
        self._motion: str | None = motion if runner is not None and known else None
        if runner is not None and motion and not known:
            logger.warning("Saved motion %r is not published; using none", motion)
        self._ui_state = (
            us.UiState.CHECKING if runner is not None else us.UiState.UNAVAILABLE
        )
        self._robot_op = 0
        self._timers: list[str] = []
        self._stop_recording_requested = False
        self._poll_ticks = 0
        self.motion_buttons: dict[str | None, tk.Button] = {}
        self.robot_button: tk.Button | None = None
        self.robot_status = tk.StringVar(master=self.root)

    def _build_motion_row(self, parent: ttk.Frame) -> None:
        """One button per motion plus "No motion"; disabled motions stay visible
        and show their reason on mouse-over."""
        ttk.Label(parent, text="Motion", width=10, anchor="w").pack(side="left")
        names: list[str | None] = [None]
        names += sorted(set(self.motions) | set(self._motion_reasons))
        for name in names:
            button = tk.Button(
                parent,
                text=name or "No motion",
                command=functools.partial(self._on_motion_click, name),
            )
            button.pack(side="left", padx=2)
            button.bind("<Enter>", functools.partial(self._on_motion_hover, name))
            self.motion_buttons[name] = button
        ttk.Label(parent, textvariable=self.robot_status, anchor="w").pack(
            side="left", fill="x", expand=True, padx=(8, 0)
        )

    def _motion_ok(self, name: str | None) -> bool:
        return name is None or (
            name in self.motions and name not in self._motion_reasons
        )

    def _can_change_motion(self) -> bool:
        return self._ui_state is us.UiState.IDLE and not self._recording

    def _on_motion_hover(self, name: str | None, _event: Any = None) -> None:
        if name in self._motion_reasons:
            self.robot_status.set(f"'{name}' disabled: {self._motion_reasons[name]}")

    def _on_motion_click(self, name: str | None) -> None:
        if name == self._motion or not self._can_change_motion():
            return
        if not self._motion_ok(name):
            return
        label = name or "No motion"
        if not messagebox.askyesno("Change motion", f"Change the motion to '{label}'?"):
            return
        # The dialog runs a nested event loop: things may have changed meanwhile.
        if not (self._can_change_motion() and self._motion_ok(name)):
            self.robot_status.set("Motion not changed: the robot or camera got busy.")
            return
        self._motion = name
        save_motion(self.config_file, name or "")
        self._robot_event(us.Ev.SELECT_CONFIRMED, name)

    def _on_robot_button(self) -> None:
        """Move to Start, or Stop robot while moving, or Clear fault."""
        state = self._ui_state
        if state is us.UiState.IDLE:
            self._robot_event(us.Ev.MOVE_TO_START)
        elif state is us.UiState.MOVING_TO_START:
            self._robot_event(us.Ev.STOP_PRESSED)
        elif state is us.UiState.FAULT:
            self._robot_event(us.Ev.FAULT_CLEARED)

    def _robot_context(self) -> us.Context:
        runner = self.motion_runner
        at_start, error = False, math.inf
        if runner is not None and self._motion in self.motions:
            at_start, error = runner.at_start(self._motion)
        return us.Context(
            motion=self._motion,
            at_start=at_start,
            start_error_deg=error,
            auto_stop=self.robot_settings.auto_stop_after_motion,
            runner_fault=runner is not None and runner.state is RunnerState.FAULT,
            move_speed_pct=self.robot_settings.move_to_start_speed_pct,
        )

    def _robot_event(self, kind: us.Ev, arg: Any = None) -> None:
        state = self._ui_state
        new, actions = us.transition(state, us.Event(kind, arg), self._robot_context())
        if new is state and not actions:
            logger.debug("Robot UI: %s ignored in %s", kind.value, state.value)
        else:
            logger.info("Robot UI: %s: %s -> %s", kind.value, state.value, new.value)
        self._ui_state = new
        for action in actions:
            self._do_robot_action(action)
        if new is not us.UiState.CLOSED:
            self._update_controls()

    def _do_robot_action(self, action: us.Action) -> None:
        kind, arg = action.kind, action.arg
        runner = self.motion_runner
        assert runner is not None
        ev = self._robot_event
        if kind is us.Act.STATUS:
            self.robot_status.set(arg)
        elif kind is us.Act.LOAD:
            self._run_robot(
                lambda: runner.load(arg),
                lambda _: ev(us.Ev.LOADED),
                lambda e: ev(us.Ev.LOAD_FAILED, e),
            )
        elif kind is us.Act.MOVE_TO_START:
            self._run_robot(
                lambda: runner.move_to_start(arg),
                lambda _: ev(us.Ev.MOVE_DONE),
                lambda e: ev(us.Ev.MOVE_FAILED, e),
            )
        elif kind is us.Act.RUN:
            self._run_robot(
                lambda: runner.run(arg),
                lambda _: ev(us.Ev.MOTION_DONE),
                lambda e: ev(us.Ev.MOTION_FAILED, e),
            )
        elif kind is us.Act.START_RECORDING:
            started = self._start_recording(
                on_started=lambda: ev(us.Ev.RECORDING_STARTED),
                on_failed=lambda e: ev(us.Ev.RECORDING_FAILED, e),
            )
            if not started:  # bad folder/image name; the status says why
                self._events.put(
                    lambda: ev(us.Ev.RECORDING_FAILED, "check the folder and image")
                )
        elif kind is us.Act.SCHEDULE_PRE_ROLL:
            self._schedule(self.robot_settings.pre_roll_s, us.Ev.PRE_ROLL_ELAPSED)
        elif kind is us.Act.SCHEDULE_POST_ROLL:
            self._schedule(self.robot_settings.post_roll_s, us.Ev.POST_ROLL_ELAPSED)
        elif kind is us.Act.CANCEL_TIMERS:
            for handle in self._timers:
                self.root.after_cancel(handle)
            self._timers.clear()
        elif kind is us.Act.STOP_ROBOT:
            self._stop_robot()
        elif kind is us.Act.STOP_RECORDING:
            self._stop_recording()
        elif kind is us.Act.CLEAR_FAULT:
            runner.clear_fault()
        elif kind is us.Act.CLOSE_RUNNER:
            runner.close()
        elif kind is us.Act.MARK_INVALID:
            name, reason = arg
            if name is not None:
                self._motion_reasons[name] = reason
                if self._motion == name:
                    self._motion = None  # in memory only; the config keeps it

    def _stop_robot(self) -> None:
        # stop() never raises but can wait ~1 s for the RPC lock; keep the UI
        # responsive.
        runner = self.motion_runner
        assert runner is not None
        threading.Thread(target=runner.stop, name="robot-stop", daemon=True).start()

    def _schedule(self, seconds: float, kind: us.Ev) -> None:
        handle = ""

        def fire() -> None:
            if handle in self._timers:
                self._timers.remove(handle)
            self._robot_event(kind)

        handle = self.root.after(int(seconds * 1000), fire)
        self._timers.append(handle)

    def _run_robot(
        self,
        start: Callable[[], concurrent.futures.Future],
        on_done: Callable[[Any], None],
        on_error: Callable[[Exception], None],
    ) -> None:
        """Start a MotionRunner job; callbacks run on the Tk thread. A result
        from an operation older than the latest one is ignored."""
        self._robot_op += 1
        op = self._robot_op

        def deliver(future: concurrent.futures.Future | None, error: Exception | None):
            if op != self._robot_op:
                logger.info("Ignoring a stale robot result (op %d)", op)
                return
            if future is not None:
                error = future.exception()
            if error is not None:
                logger.warning("Robot operation failed: %s", error)
                on_error(error)
            else:
                assert future is not None
                on_done(future.result())

        try:
            future = start()
        except Exception as e:  # pylint: disable=broad-except
            self._events.put(functools.partial(deliver, None, e))
            return
        future.add_done_callback(
            lambda f: self._events.put(functools.partial(deliver, f, None))
        )

    def _start_robot_check(self) -> None:
        runner = self.motion_runner
        assert runner is not None
        self.robot_status.set("Checking robot…")
        self._run_robot(
            runner.connect_and_check, self._on_robot_checked, self._on_robot_unreachable
        )
        op = self._robot_op
        self.root.after(
            int(ROBOT_CHECK_TIMEOUT_S * 1000), lambda: self._robot_check_timeout(op)
        )

    def _robot_check_timeout(self, op: int) -> None:
        if self._ui_state is us.UiState.CHECKING and op == self._robot_op:
            self._robot_op += 1  # a late answer is now stale
            self._on_robot_unreachable(TimeoutError("no answer from the robot"))

    def _on_robot_unreachable(self, error: Exception) -> None:
        for name in self.motions:
            self._motion_reasons.setdefault(name, f"robot unreachable: {error}")
        self._motion = None
        self._robot_event(us.Ev.STARTUP_FAILED, error)

    def _on_robot_checked(self, reasons: dict[str, str | None]) -> None:
        for name, reason in reasons.items():
            if reason is not None:
                self._motion_reasons[name] = reason
        saved = self._motion
        if saved is not None and not self._motion_ok(saved):
            self._motion = None  # in memory only; the config keeps the saved value
        self._robot_event(us.Ev.STARTUP_CHECKED)
        if saved is not None and self._motion is None:
            self.robot_status.set(
                f"Saved motion '{saved}' is disabled ({self._motion_reasons[saved]}); "
                "using No motion."
            )
        elif self._motion is not None:
            self._robot_event(us.Ev.SELECT_CONFIRMED, self._motion)

    def _record_allowed_by_robot(self) -> bool:
        if self.motion_runner is None or self._motion is None:
            return True
        return self._ui_state in (
            us.UiState.IDLE,
            us.UiState.UNAVAILABLE,
            us.UiState.FAULT,
            *us.MOTION_STATES,
        )

    def _update_robot_controls(self) -> None:
        if self.motion_runner is None or self._ui_state is us.UiState.CLOSED:
            return
        state = self._ui_state
        can_change = self._can_change_motion()
        for name, button in self.motion_buttons.items():
            active = name == self._motion
            ok = self._motion_ok(name)
            button.config(
                relief="sunken" if active else "raised",
                bg="#1565c0" if active else "#e0e0e0",
                fg="white" if active else "black",
                disabledforeground="#9e9e9e" if ok else "#c62828",
                state="normal" if can_change and ok else "disabled",
            )
        if self.robot_button is None:
            return
        if state is us.UiState.MOVING_TO_START:
            self.robot_button.config(
                text="Stop robot", bg="#c62828", fg="white", state="normal"
            )
        elif state is us.UiState.FAULT:
            self.robot_button.config(
                text="Clear fault", bg="#f9a825", fg="black", state="normal"
            )
        else:
            usable = state is us.UiState.IDLE and self._motion is not None
            usable = usable and not self._recording
            at_start = usable and self._robot_context().at_start
            self.robot_button.config(
                text="Move to Start",
                bg="#ef6c00" if usable and not at_start else "#e0e0e0",
                fg="white" if usable and not at_start else "black",
                state="normal" if usable else "disabled",
            )

    # UI ---------------------------------------------------------------------

    def _update_controls(self) -> None:
        if self._recording:
            self.record_button.config(
                text="Stop recording",
                bg="#c62828",
                fg="white",
                activebackground="#b71c1c",
            )
        else:
            self.record_button.config(
                text="Start recording",
                bg="#2e7d32",
                fg="white",
                activebackground="#1b5e20",
            )
        record_ok = not self._busy and self._record_allowed_by_robot()
        self.record_button.config(state="normal" if record_ok else "disabled")
        self._update_mode_states()
        self._update_robot_controls()
        # The folder and image are fixed for the duration of a recording
        # and its download.
        radio_state = "disabled" if self._busy or self._recording else "normal"
        self.folder_entry.config(state=radio_state)
        self.image_entry.config(state=radio_state)

    def _poll(self) -> None:
        while True:
            try:
                self._events.get_nowait()()
            except queue.Empty:
                break

        if self._recording and not self._busy:
            elapsed = int(time.monotonic() - self._record_started)
            self.status.set(
                f"● Recording {self.picker.mode.label} — \
                {elapsed // 60:02d}:{elapsed % 60:02d}"
            )

        self._poll_ticks += 1
        if self._poll_ticks % ROBOT_REFRESH_TICKS == 0:
            self._update_robot_controls()

        frame = self.reader.latest_frame()
        # Ignore frames still buffered from before the preview was paused.
        if frame is not None and not self._recording:
            height, width = frame.shape[:2]
            scale = DISPLAY_WIDTH / width
            self._last_frame = cv2.resize(frame, (DISPLAY_WIDTH, int(height * scale)))
            self._show_frame(self._last_frame)

        self.root.after(15, self._poll)

    def _show_frame(self, frame: Any) -> None:
        image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        self._photo = ImageTk.PhotoImage(
            image
        )  # Keep a reference or Tk drops the image
        self.video.config(image=self._photo, text="")

    def _open_qr_window(self, item: Shared) -> None:
        """Show the QR code for a clip in its own window, until the user closes it."""
        if self._qr_window is not None and self._qr_window.winfo_exists():
            self._qr_window.destroy()
        window = tk.Toplevel(self.root)
        window.title("QR code")
        image = ImageTk.PhotoImage(Image.open(item.qr).convert("RGB"))
        label = tk.Label(window, image=image, bg="white")
        label.image = image  # Keep a reference or Tk drops the image
        label.pack(padx=16, pady=(16, 8))
        tk.Label(window, text=item.url, wraplength=420, justify="center").pack(
            padx=16, pady=(0, 16)
        )
        self._qr_window = window

    def _show_message(
        self, text: str, color: tuple[int, int, int] = (220, 220, 220)
    ) -> None:
        """Show text over the dimmed last frame, or a black frame if there is none
        yet."""
        if self._last_frame is None:
            frame = np.zeros((DISPLAY_WIDTH * 9 // 16, DISPLAY_WIDTH, 3), np.uint8)
        else:
            frame = (self._last_frame * 0.35).astype(np.uint8)
        font, font_scale, thickness = cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2
        (text_w, text_h), _ = cv2.getTextSize(text, font, font_scale, thickness)
        origin = ((frame.shape[1] - text_w) // 2, (frame.shape[0] + text_h) // 2)
        cv2.putText(
            frame, text, origin, font, font_scale, color, thickness, cv2.LINE_AA
        )
        self._show_frame(frame)

    def _show_paused(self) -> None:
        self._show_message("REC - preview paused while recording", color=(60, 60, 230))

    def _on_close(self) -> None:
        # Stop the robot first: the GoPro shutdown below can take seconds. The
        # shutdown also stops a recording in progress.
        if self.motion_runner is not None:
            self._robot_event(us.Ev.CLOSE)
        self.reader.stop()
        self.record_button.config(state="disabled")
        self.status.set("Closing…")
        self.root.update_idletasks()
        try:
            self.controller.submit(self.controller.shutdown()).result(timeout=15)
        except Exception:  # pylint: disable=broad-except
            logger.exception("Shutdown did not complete cleanly")
        self.postprocessor.stop()
        self.root.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Live preview and record video from a GoPro over USB."
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=(
            "Base folder for downloaded videos; each recording goes into a "
            "subfolder of it"
        ),
    )
    parser.add_argument(
        "--port", type=int, default=8554, help="UDP port for the preview stream"
    )
    parser.add_argument(
        "--serial",
        default=None,
        help="Camera serial number. Auto-discovered via mDNS if not set",
    )
    parser.add_argument(
        "--no-upload",
        action="store_true",
        help="Only download and watermark; skip the upload and QR code",
    )
    parser.add_argument(
        "--robot-ip",
        default=None,
        help="Fairino controller IP (overrides 'robot.ip' in the config file)",
    )
    parser.add_argument(
        "--no-robot",
        action="store_true",
        help="Run without the robot: no motion buttons, recording only",
    )
    parser.add_argument("--verbose", action="store_true", help="Enable debug logging")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    # The SDK logs every HTTP request/response at INFO; keep the console readable.
    logging.getLogger("open_gopro").setLevel(
        logging.DEBUG if args.verbose else logging.WARNING
    )

    folder, image, fadein_image, fadeout_image = load_config(CONFIG_FILE)
    motion, robot_raw = load_robot_config(CONFIG_FILE)
    runner, settings, motions, broken = None, None, {}, {}
    if not args.no_robot:
        try:
            settings = robot_settings_from_dict(robot_raw)
        except (SchemaError, TypeError) as e:
            logger.error("Robot disabled: invalid 'robot' settings: %s", e)
        else:
            if args.robot_ip:
                settings = dataclasses.replace(settings, ip=args.robot_ip)
            motions, broken = load_motions(MOTIONS_DIR)
            runner = MotionRunner.for_robot(settings, motions)
    root = tk.Tk()
    LivestreamApp(
        root,
        GoProController(args.output_dir, args.port, args.serial),
        upload=not args.no_upload,
        folder=folder,
        image=image,
        fadein_image=fadein_image,
        fadeout_image=fadeout_image,
        config_file=CONFIG_FILE,
        motion_runner=runner,
        motions=motions,
        broken_motions=broken,
        motion=motion,
        robot_settings=settings,
    )
    root.mainloop()


if __name__ == "__main__":
    main()
