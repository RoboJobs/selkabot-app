"""
GoPro livestream recorder — preview, record and download over USB.

Connects to a GoPro HERO10 over the wired (USB) interface with the Open GoPro
SDK, shows the camera's preview stream in a Tkinter window and lets the user
start/stop video recording. Each recorded clip is downloaded to the local
computer as soon as recording stops.

Threading model:
- Main thread: Tkinter UI. Only this thread touches widgets.
- Asyncio thread: owns the WiredGoPro client; all camera commands run here.
- Post-process thread: watermarks (and optionally uploads) clips one at a time,
  in the order they were downloaded, so the camera is free to record meanwhile.
- Frame thread: reads the UDP preview stream with OpenCV, keeps latest frame.
Results from the asyncio thread reach the UI through a queue polled by Tk.

Notes:
- The preview stream is a low resolution live view; the recorded file is in
  the selected resolution (2.7K 16:9 by default) at 60 fps.
- Resolution can only be changed while not recording or downloading. The
  preview stream is restarted around the change.
- The preview is paused while recording. On HERO10 firmware H21.01.01.62.00,
  starting a recording while the USB preview is running kills the preview
  until the camera is power cycled. Stopping the preview first, and starting
  it again once the clip is written, avoids this.
- The SDK's StreamFeature uses BLE to start the preview stream, which is not
  available on WiredGoPro, so the HTTP preview command is used directly.

Usage:
    uv run python -m sw.livestream [--output-dir videos] [--port 8554] [--no-upload]
"""

from __future__ import annotations

import argparse
import asyncio
import concurrent.futures
import functools
import logging
import os
import queue
import threading
import time
import tkinter as tk
from collections.abc import Callable, Coroutine
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from tkinter import ttk
from typing import Any

import cv2
import numpy as np
from open_gopro import WiredGoPro
from open_gopro.models import constants, proto
from open_gopro.models.constants import settings
from PIL import Image, ImageTk

from sw.sync_uploads import sync_uploads
from sw.uploader import make_qr, public_url, upload_video
from sw.watermark import add_watermark

logger = logging.getLogger(__name__)

# Silence ffmpeg's h264 decoder warnings (missing PPS/references) that are
# expected while joining the UDP stream mid-GOP. Read on first VideoCapture.
os.environ.setdefault("OPENCV_FFMPEG_LOGLEVEL", "-8")

# Label shown in the UI -> GoPro resolution setting
RESOLUTIONS: dict[str, settings.VideoResolution] = {
    "2.7K 16:9": settings.VideoResolution.NUM_2_7K,
    "2.7K 4:3": settings.VideoResolution.NUM_2_7K_4_3,
    "1080p": settings.VideoResolution.NUM_1080,
}
DEFAULT_RESOLUTION = "2.7K 16:9"
FPS = settings.FramesPerSecond.NUM_60_0

DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parents[1] / "videos"
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


class PostProcessor:
    """Watermarks, and optionally uploads, clips one at a time on a background thread.

    Clips are handled in the order they were submitted, so recording and downloading
    can continue while earlier clips are still being processed. Results reach the
    UI through `events`, so the callbacks run on the Tk thread.
    """

    def __init__(
        self,
        upload: bool,
        events: queue.Queue[Callable[[], None]],
        on_ready: Callable[[Shared], None],
        on_done: Callable[[Shared], None],
        on_error: Callable[[Clip, Exception], None],
    ) -> None:
        self.upload = upload
        self._events = events
        self._on_ready = on_ready
        self._on_done = on_done
        self._on_error = on_error
        self._jobs: queue.Queue[Callable[[], Any] | None] = queue.Queue()
        # Not a daemon thread: the process waits for queued jobs to finish before exiting.
        self._thread = threading.Thread(target=self._run, name="postprocess")
        self._thread.start()

    def submit(self, clip: Clip) -> None:
        self._jobs.put(functools.partial(self._process, clip))

    def submit_job(self, job: Callable[[], Any]) -> None:
        """Run `job` on the worker, after the jobs already queued."""
        self._jobs.put(job)

    def stop(self) -> None:
        """Queue a stop after the jobs already submitted. Returns without waiting for them."""
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
            add_watermark(clip.original, clip.watermarked)
            shared = Shared(video=clip.watermarked)
            if self.upload:
                # The URL is known before the upload, so the QR code can be shown while it runs.
                url = public_url(clip.watermarked)
                make_qr(url, clip.qr_file)
                shared = Shared(video=clip.watermarked, url=url, qr=clip.qr_file)
                self._events.put(functools.partial(self._on_ready, shared))
                upload_video(clip.watermarked)
        except Exception as e:  # pylint: disable=broad-except
            logger.exception("Post-processing %s failed", clip.original)
            self._events.put(functools.partial(self._on_error, clip, e))
        else:
            self._events.put(functools.partial(self._on_done, shared))


class PreviewReader:
    """Reads the GoPro UDP preview stream in a background thread.

    Only the most recent frame is kept so the display never lags behind.
    The capture is reopened if the stream drops (e.g. while it is restarted
    after a resolution change).
    """

    def __init__(self, port: int) -> None:
        self._url = f"udp://127.0.0.1:{port}?overrun_nonfatal=1&fifo_size=50000000"
        self._frame: Any = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="preview-reader", daemon=True)

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
        self._media_before: set[str] = set()
        self._recording = False
        self._keep_alive_task: asyncio.Task | None = None
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, name="gopro-loop", daemon=True)
        self._thread.start()

    def submit(self, coro: Coroutine[Any, Any, Any]) -> concurrent.futures.Future:
        """Schedule a coroutine on the camera loop from any thread."""
        return asyncio.run_coroutine_threadsafe(coro, self._loop)

    @property
    def gopro(self) -> WiredGoPro:
        assert self._gopro is not None, "GoPro is not connected"
        return self._gopro

    async def connect(self, resolution: settings.VideoResolution) -> str:
        """Connect over USB, configure video mode and start the preview stream.

        Returns:
            The camera serial number.
        """
        self._gopro = WiredGoPro(self._serial)
        await self._gopro.open(timeout=10, retries=3)
        # A previous session may have left the preview running, which blocks setting changes.
        await self._stop_preview()
        _check(await self.gopro.http_command.load_preset_group(group=proto.EnumPresetGroup.PRESET_GROUP_ID_VIDEO))
        await self._apply_video_settings(resolution)
        await self._start_preview()
        self._keep_alive_task = self._loop.create_task(self._keep_alive())
        # get_camera_name is not supported by HERO10 firmware (404)
        return self.gopro.identifier

    async def set_resolution(self, resolution: settings.VideoResolution) -> None:
        """Change the recording resolution, restarting the preview around it."""
        await self._stop_preview()
        try:
            await self._apply_video_settings(resolution)
        finally:
            await self._start_preview()

    async def start_recording(self) -> None:
        """Pause the preview and start recording (see module notes)."""
        self._media_before = await self._media_filenames()
        await self._stop_preview()
        try:
            _check(await self.gopro.http_command.set_shutter(shutter=constants.Toggle.ENABLE))
        except Exception:
            await self._start_preview()
            raise
        self._recording = True

    async def stop_recording(self) -> list[str]:
        """Stop recording, resume the preview and return the new media files on the camera."""
        _check(await self.gopro.http_command.set_shutter(shutter=constants.Toggle.DISABLE))
        self._recording = False
        # Waits internally until the camera has finished encoding the file.
        # Long recordings are split into several chapter files.
        new_files = sorted((await self._media_filenames()) - self._media_before)
        await self._start_preview()
        return new_files

    async def download(self, camera_files: list[str], progress: Callable[[str], None]) -> list[Clip]:
        """Download each camera file and return the clips to post-process.

        The original download is kept as `<name>_original.mp4`; the watermarked
        copy is `<name>_watermarked.mp4`, written later by the PostProcessor.
        """
        self.output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        clips: list[Clip] = []
        for i, camera_file in enumerate(camera_files, start=1):
            label = f"({i}/{len(camera_files)})"
            base = f"{stamp}_{Path(camera_file).stem}"
            original = self.output_dir / f"{base}_original.mp4"
            watermarked = self.output_dir / f"{base}_watermarked.mp4"
            qr_file = self.output_dir / f"{base}_qr.png"

            progress(f"Downloading {camera_file} {label}…")
            _check(await self.gopro.http_command.download_file(camera_file=camera_file, local_file=original))
            clips.append(Clip(original=original, watermarked=watermarked, qr_file=qr_file))
        return clips

    async def shutdown(self) -> None:
        """Stop any recording, the preview stream and the loop."""
        if self._keep_alive_task is not None:
            self._keep_alive_task.cancel()
        if self._gopro is not None:
            try:
                if self._recording:
                    await self.gopro.http_command.set_shutter(shutter=constants.Toggle.DISABLE)
                await self.gopro.http_command.set_preview_stream(mode=constants.Toggle.DISABLE)
                await self.gopro.close()
            except Exception:  # pylint: disable=broad-except
                logger.exception("Error while closing the GoPro connection")
        self._loop.call_soon(self._loop.stop)

    async def _apply_video_settings(self, resolution: settings.VideoResolution) -> None:
        # Resolution first: the valid fps values depend on it.
        _check(await self.gopro.http_setting.video_resolution.set(resolution))
        _check(await self.gopro.http_setting.frames_per_second.set(FPS))

    async def _stop_preview(self) -> None:
        # Not checked: the camera returns an error if no stream is running.
        await self.gopro.http_command.set_preview_stream(mode=constants.Toggle.DISABLE)

    async def _start_preview(self) -> None:
        # Callers stop the preview first: starting while it is running fails.
        _check(await self.gopro.http_command.set_preview_stream(mode=constants.Toggle.ENABLE, port=self.port))

    async def _media_filenames(self) -> set[str]:
        return {item.filename for item in (await self.gopro.http_command.get_media_list()).data.files}

    async def _keep_alive(self) -> None:
        while True:
            await asyncio.sleep(KEEP_ALIVE_PERIOD_S)
            # Commands block while encoding, and the camera stays awake while recording anyway.
            if self._recording:
                continue
            try:
                await self.gopro.http_command.set_keep_alive()
            except Exception:  # pylint: disable=broad-except
                logger.warning("Keep alive failed", exc_info=True)


def _check(response: Any) -> None:
    if not response.ok:
        raise RuntimeError(f"{response.identifier} failed: {response.status}")


class LivestreamApp:
    """Tkinter window with the live preview, resolution selector and record button."""

    def __init__(self, root: tk.Tk, controller: GoProController, upload: bool) -> None:
        self.root = root
        self.controller = controller
        self.reader = PreviewReader(controller.port)
        self._events: queue.Queue[Callable[[], None]] = queue.Queue()
        self.postprocessor = PostProcessor(
            upload, self._events, self._on_ready, self._on_processed, self._on_process_error
        )
        # Check the bucket on startup; uploads it finds missing go through the same queue.
        self.postprocessor.submit_job(functools.partial(sync_uploads, controller.output_dir, upload))
        self._recording = False
        self._busy = True  # Connecting
        self._record_started = 0.0
        self._photo: ImageTk.PhotoImage | None = None
        self._last_frame: Any = None
        self._qr_window: tk.Toplevel | None = None

        root.title("GoPro Livestream")
        root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.video = ttk.Label(root, anchor="center")
        self.video.pack(fill="both", expand=True, padx=8, pady=8)
        self._show_message("Connecting to GoPro...")

        controls = ttk.Frame(root)
        controls.pack(fill="x", padx=8, pady=(0, 8))

        ttk.Label(controls, text="Resolution (60 fps):").pack(side="left")
        self.resolution = tk.StringVar(value=DEFAULT_RESOLUTION)
        self._applied_resolution = DEFAULT_RESOLUTION
        self.resolution_buttons = [
            ttk.Radiobutton(
                controls, text=label, value=label, variable=self.resolution, command=self._on_resolution
            )
            for label in RESOLUTIONS
        ]
        for button in self.resolution_buttons:
            button.pack(side="left", padx=4)

        self.record_button = tk.Button(controls, text="Start recording", width=16, command=self._on_record)
        self.record_button.pack(side="right")

        self.status = tk.StringVar()
        ttk.Label(root, textvariable=self.status, anchor="w").pack(fill="x", padx=8, pady=(0, 8))

        self._update_controls()
        self._run(
            controller.connect(RESOLUTIONS[DEFAULT_RESOLUTION]),
            "Connecting to GoPro over USB…",
            self._on_connected,
        )
        root.after(15, self._poll)

    # Camera operations -------------------------------------------------------------------------------

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
        self.status.set(f"Connected to GoPro {serial}. Ready to record at {self._applied_resolution}, 60 fps.")

    def _on_resolution(self) -> None:
        label = self.resolution.get()
        if label == self._applied_resolution:
            return

        def done(_: None) -> None:
            self._applied_resolution = label
            self.status.set(f"Resolution set to {label}, 60 fps.")

        def failed(_: Exception) -> None:
            self.resolution.set(self._applied_resolution)

        self._run(
            self.controller.set_resolution(RESOLUTIONS[label]),
            f"Switching to {label}…",
            done,
            failed,
        )

    def _on_record(self) -> None:
        if not self._recording:
            self._run(self.controller.start_recording(), "Starting recording…", self._on_recording_started)
        else:
            self._run(self.controller.stop_recording(), "Stopping recording…", self._on_recording_stopped)

    def _on_recording_started(self, _: None) -> None:
        self._recording = True
        self._record_started = time.monotonic()
        self._show_paused()

    def _on_recording_stopped(self, camera_files: list[str]) -> None:
        self._recording = False
        if not camera_files:
            self.status.set("Recording stopped, but no new file was found on the camera.")
            return

        def progress(message: str) -> None:
            self._events.put(lambda: self.status.set(message))

        def done(clips: list[Clip]) -> None:
            for clip in clips:
                self.postprocessor.submit(clip)
            self.status.set(f"Watermarking {len(clips)} clip(s) in background…")

        self._run(
            self.controller.download(camera_files, progress),
            "Transferring video from camera…",
            done,
        )

    def _on_ready(self, shared: Shared) -> None:
        """Show the QR code as soon as the upload starts; the link goes live when the upload ends."""
        self.status.set(f"Uploading {shared.video.name}…")
        self._open_qr_window(shared)

    def _on_processed(self, shared: Shared) -> None:
        if shared.url is None:
            self.status.set(f"Saved: {shared.video.name}")
            logger.info("Video saved to %s", shared.video)
            return
        self.status.set(f"Uploaded: {shared.url}")
        logger.info("Video %s available at %s (QR: %s)", shared.video, shared.url, shared.qr)

    def _on_process_error(self, clip: Clip, error: Exception) -> None:
        self.status.set(f"Error processing {clip.original.name}: {error}")

    # UI ----------------------------------------------------------------------------------------------

    def _update_controls(self) -> None:
        if self._recording:
            self.record_button.config(text="Stop recording", bg="#c62828", fg="white", activebackground="#b71c1c")
        else:
            self.record_button.config(text="Start recording", bg="#2e7d32", fg="white", activebackground="#1b5e20")
        self.record_button.config(state="disabled" if self._busy else "normal")
        radio_state = "disabled" if self._busy or self._recording else "normal"
        for button in self.resolution_buttons:
            button.config(state=radio_state)

    def _poll(self) -> None:
        while True:
            try:
                self._events.get_nowait()()
            except queue.Empty:
                break

        if self._recording and not self._busy:
            elapsed = int(time.monotonic() - self._record_started)
            self.status.set(f"● Recording {self._applied_resolution} 60 fps — {elapsed // 60:02d}:{elapsed % 60:02d}")

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
        self._photo = ImageTk.PhotoImage(image)  # Keep a reference or Tk drops the image
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
        tk.Label(window, text=item.url, wraplength=420, justify="center").pack(padx=16, pady=(0, 16))
        self._qr_window = window

    def _show_message(self, text: str, color: tuple[int, int, int] = (220, 220, 220)) -> None:
        """Show text over the dimmed last frame, or a black frame if there is none yet."""
        if self._last_frame is None:
            frame = np.zeros((DISPLAY_WIDTH * 9 // 16, DISPLAY_WIDTH, 3), np.uint8)
        else:
            frame = (self._last_frame * 0.35).astype(np.uint8)
        font, font_scale, thickness = cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2
        (text_w, text_h), _ = cv2.getTextSize(text, font, font_scale, thickness)
        origin = ((frame.shape[1] - text_w) // 2, (frame.shape[0] + text_h) // 2)
        cv2.putText(frame, text, origin, font, font_scale, color, thickness, cv2.LINE_AA)
        self._show_frame(frame)

    def _show_paused(self) -> None:
        self._show_message("REC - preview paused while recording", color=(60, 60, 230))

    def _on_close(self) -> None:
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
    parser = argparse.ArgumentParser(description="Live preview and record video from a GoPro over USB.")
    parser.add_argument(
        "--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR, help="Where downloaded videos are saved"
    )
    parser.add_argument("--port", type=int, default=8554, help="UDP port for the preview stream")
    parser.add_argument(
        "--serial", default=None, help="Camera serial number. Auto-discovered via mDNS if not set"
    )
    parser.add_argument(
        "--no-upload", action="store_true", help="Only download and watermark; skip the upload and QR code"
    )
    parser.add_argument("--verbose", action="store_true", help="Enable debug logging")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    # The SDK logs every HTTP request/response at INFO; keep the console readable.
    logging.getLogger("open_gopro").setLevel(logging.DEBUG if args.verbose else logging.WARNING)

    root = tk.Tk()
    LivestreamApp(root, GoProController(args.output_dir, args.port, args.serial), upload=not args.no_upload)
    root.mainloop()


if __name__ == "__main__":
    main()
