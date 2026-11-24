"""MotionRunner: load, move to start and play published motions (Phase 5).

Used by the livestream app. One daemon worker thread runs jobs in order; each
public method except stop(), at_start() and close() queues a job and returns
a concurrent.futures.Future. Daemon on purpose: an SDK call can block forever
on a dropped network and must not keep the app alive.

Robot access: one FR5 connection; every RPC call holds `_rpc_lock` only for
the call itself. Motions are started with calls that return at once
(MoveJ with blendT=0, MoveTPD) and completion is polled from the SDK's state
cache, which needs no lock. stop() takes the lock for StopMotion; if it
cannot get it within STOP_LOCK_TIMEOUT_S (the worker is stuck inside an RPC,
e.g. if MoveTPD turns out to block until the motion ends), it opens a fresh
connection just to send StopMotion. The Phase 4 robot test tells which case
applies; both are handled.

The app never generates trajectories: it only plays files already published
with `motionctl publish` (motion.json + the controller file).
"""

from __future__ import annotations

import enum
import math
import queue
import threading
import time
from collections.abc import Callable
from concurrent.futures import Future
from typing import Any

from sw.robot.fr5 import FR5, TPD_BLEND_SMOOTH, FairinoError
from sw.robot.schema import MotionConfig, RobotSettings, SchemaError

POSE_TOLERANCE = 0.01  # GetTPDStartPose vs motion.json (float32 precision)
STOP_ATTEMPTS = 3
STOP_RETRY_S = 0.1
STOP_LOCK_TIMEOUT_S = 1.0
# Actual speed = max x global speed x command speed (manual 1.3.9.5). The
# global speed is the one speed control; commands always get 100%.
COMMAND_PCT = 100


class RunnerState(enum.Enum):
    IDLE = "idle"
    BUSY = "busy"
    FAULT = "fault"
    CLOSED = "closed"


class MotionError(RuntimeError):
    """A motion could not be loaded, started or completed."""


class MotionTimeout(MotionError):
    """A motion did not finish before its watchdog deadline."""


class MotionStopped(MotionError):
    """The motion was ended by stop(); not a fault."""


class RunnerClosed(RuntimeError):
    """The runner was closed; no new jobs are accepted."""


class MotionRunner:
    def __init__(
        self,
        connect: Callable[[], FR5],
        settings: RobotSettings,
        motions: dict[str, MotionConfig],
        *,
        poll_s: float = 0.02,
        start_timeout_s: float = 2.0,
        watchdog_margin_s: float = 3.0,
    ) -> None:
        self._connect = connect
        self.settings = settings
        self.motions = dict(motions)
        self._poll_s = poll_s
        self._start_timeout_s = start_timeout_s
        self._watchdog_margin_s = watchdog_margin_s

        self._rpc_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._fr5: FR5 | None = None
        self._state = RunnerState.IDLE
        self.fault_reason: str | None = None
        self.disabled: dict[str, str] = {}
        self._stop_requested = threading.Event()

        self._jobs: queue.Queue = queue.Queue()
        self._worker = threading.Thread(
            target=self._work, name="motion-runner", daemon=True
        )
        self._worker.start()

    # -- state ---------------------------------------------------------------

    @property
    def state(self) -> RunnerState:
        return self._state

    def _set_state(self, state: RunnerState, reason: str | None = None) -> None:
        with self._state_lock:
            if self._state is RunnerState.CLOSED:
                return
            self._state = state
            self.fault_reason = reason if state is RunnerState.FAULT else None

    def clear_fault(self) -> None:
        """Operator acknowledged a fault (UI button): back to IDLE."""
        if self._state is RunnerState.FAULT:
            self._set_state(RunnerState.IDLE)

    # -- worker ----------------------------------------------------------------

    def _work(self) -> None:
        while True:
            item = self._jobs.get()
            if item is None:
                return
            fn, future = item
            if not future.set_running_or_notify_cancel():
                continue
            if self._state is RunnerState.CLOSED:
                future.set_exception(RunnerClosed("runner is closed"))
                continue
            self._set_state(RunnerState.BUSY)
            try:
                result = fn()
            except MotionStopped as exc:
                self._set_state(RunnerState.IDLE)
                future.set_exception(exc)
            except BaseException as exc:  # noqa: BLE001 - surfaced via the future
                self._set_state(RunnerState.FAULT, str(exc))
                future.set_exception(exc)
            else:
                self._set_state(RunnerState.IDLE)
                future.set_result(result)

    def _submit(self, fn: Callable[[], Any], *, motion: bool = True) -> Future:
        if self._state is RunnerState.CLOSED:
            raise RunnerClosed("runner is closed")
        if motion and self._state is RunnerState.FAULT:
            raise MotionError(f"runner is in fault: {self.fault_reason}")
        future: Future = Future()
        self._jobs.put((fn, future))
        return future

    def _call(self, fn: Callable[[FR5], Any]) -> Any:
        """Run one RPC on the main connection under the RPC lock."""
        if self._fr5 is None:
            raise MotionError("not connected: run connect_and_check() first")
        with self._rpc_lock:
            return fn(self._fr5)

    def _config(self, name: str) -> MotionConfig:
        if name not in self.motions:
            raise ValueError(f"unknown motion {name!r}")
        if name in self.disabled:
            raise ValueError(f"motion {name!r} is disabled: {self.disabled[name]}")
        return self.motions[name]

    # -- startup -----------------------------------------------------------

    def connect_and_check(self) -> Future:
        """Connect, then check every motion once. The future's result maps
        each name to None (usable) or the reason it is disabled."""

        def job() -> dict[str, str | None]:
            if self._fr5 is None:
                self._fr5 = self._connect()
            reasons: dict[str, str | None] = {}
            for name, config in self.motions.items():
                reasons[name] = self._check_motion(config)
            self.disabled = {n: r for n, r in reasons.items() if r is not None}
            self._call(lambda r: r.set_speed(self.settings.safe_default_speed_pct))
            return reasons

        return self._submit(job, motion=False)

    def _check_motion(self, config: MotionConfig) -> str | None:
        try:
            config.check_consistent()
        except SchemaError as exc:
            return f"invalid motion.json: {exc.reason}"
        try:
            md5 = self._call(lambda r: r.file_md5(config.controller_file))
        except FairinoError as exc:
            return f"hash query failed (code {exc.code})"
        if md5 is None:
            return "file missing on controller"
        if md5.lower() != config.md5.lower():
            return "md5 mismatch"
        return None

    # -- motions -----------------------------------------------------------

    def load(self, name: str) -> Future:
        """LoadTPD and check the start pose. Never moves the robot."""
        config = self._config(name)

        def job() -> None:
            self._load(config)

        return self._submit(job)

    def _load(self, config: MotionConfig) -> None:
        self._call(lambda r: r.load_tpd(config.tpd_name))
        pose = self._call(lambda r: r.tpd_start_pose(config.tpd_name))
        diff = max(abs(a - b) for a, b in zip(pose, config.start_pose, strict=True))
        if diff > POSE_TOLERANCE:
            self.disabled[config.name] = "start pose differs from motion.json"
            raise MotionError(
                f"{config.name}: the loaded start pose differs from motion.json by "
                f"{diff:.3f} (re-publish the motion)"
            )

    def at_start(self, name: str) -> tuple[bool, float]:
        """(within tolerance, max joint error in deg). Reads the state cache
        only; never blocks on the robot."""
        config = self.motions[name]
        if self._fr5 is None:
            return False, math.inf
        joints = self._fr5.joints()
        err = max(abs(a - b) for a, b in zip(joints, config.start_joints, strict=True))
        return err <= self.settings.start_tolerance_deg, err

    def move_to_start(self, name: str) -> Future:
        """Move to the motion's start joints at move_to_start_speed_pct."""
        config = self._config(name)

        def job() -> None:
            if self.at_start(name)[0]:
                return
            speed = self.settings.move_to_start_speed_pct
            self._stop_requested.clear()
            self._call(lambda r: r.ensure_auto())
            self._call(lambda r: r.set_speed(speed))
            try:
                self._call(
                    lambda r: r.move_j(
                        list(config.start_joints),
                        config.tool_id,
                        COMMAND_PCT,
                        blocking=False,
                    )
                )
                # Generous deadline: 10% speed on a long joint move.
                self._wait_motion(deadline_s=120.0, must_start=False)
            finally:
                self._restore_speed()

        return self._submit(job)

    def run(self, name: str) -> Future:
        """Play the motion at its approved ovl and wait for it to finish."""
        config = self._config(name)

        def job() -> None:
            self._stop_requested.clear()
            self._call(lambda r: r.ensure_auto())
            self._load(config)  # cheap insurance: a MoveJ may have unloaded it
            ovl = config.approved_ovl
            deadline = 1.5 * config.duration_s * 100.0 / ovl + self._watchdog_margin_s
            timed_out = threading.Event()

            def watchdog() -> None:
                timed_out.set()
                self.stop()

            timer = threading.Timer(deadline, watchdog)
            timer.daemon = True
            timer.start()
            try:
                self._call(lambda r: r.set_speed(ovl))  # global = playback speed
                self._call(
                    lambda r: r.move_tpd(config.tpd_name, TPD_BLEND_SMOOTH, COMMAND_PCT)
                )
                self._wait_motion(deadline_s=deadline, timed_out=timed_out)
            finally:
                timer.cancel()
                self._restore_speed()

        return self._submit(job)

    def _wait_motion(
        self,
        *,
        deadline_s: float,
        must_start: bool = True,
        timed_out: threading.Event | None = None,
    ) -> None:
        """Poll the state cache: wait for motion_done to go 0 (started; the
        cache is stale right after the command), then back to 1 (done)."""
        fr5 = self._fr5
        assert fr5 is not None
        start = time.monotonic()
        started = False
        while time.monotonic() - start < self._start_timeout_s:
            if not fr5.motion_done():
                started = True
                break
            time.sleep(self._poll_s)
        if not started:
            if must_start:
                raise MotionError("motion did not start")
            return
        while True:
            if timed_out is not None and timed_out.is_set():
                raise MotionTimeout("motion did not finish before its deadline")
            if self._stop_requested.is_set():
                raise MotionStopped("stopped")
            main, sub = fr5.error_codes()
            if main != 0:
                self.stop()
                raise FairinoError(main, f"robot error (sub code {sub})")
            if fr5.motion_done():
                return
            if time.monotonic() - start > deadline_s:
                self.stop()
                raise MotionTimeout("motion did not finish before its deadline")
            time.sleep(self._poll_s)

    def _restore_speed(self) -> None:
        try:
            self._call(lambda r: r.set_speed(self.settings.safe_default_speed_pct))
        except Exception:  # noqa: BLE001 - best effort
            pass

    # -- stop and close --------------------------------------------------

    def stop(self) -> bool:
        """StopMotion now. Never raises; safe to call repeatedly and from any
        thread. Returns True if a StopMotion call succeeded."""
        self._stop_requested.set()
        if self._fr5 is None:
            return False
        if self._rpc_lock.acquire(timeout=STOP_LOCK_TIMEOUT_S):
            try:
                return self._stop_with(self._fr5)
            finally:
                self._rpc_lock.release()
        # The worker is stuck inside an RPC: use a fresh connection.
        try:
            spare = self._connect()
        except Exception:  # noqa: BLE001 - never raise from stop()
            return False
        return self._stop_with(spare)

    @staticmethod
    def _stop_with(fr5: FR5) -> bool:
        for attempt in range(STOP_ATTEMPTS):
            try:
                fr5.stop()
                return True
            except Exception:  # noqa: BLE001 - retry, then give up quietly
                if attempt < STOP_ATTEMPTS - 1:
                    time.sleep(STOP_RETRY_S)
        return False

    def close(self) -> None:
        """Stop, reject new jobs, end the worker, close the connection.
        Idempotent."""
        if self._state is RunnerState.CLOSED:
            return
        self.stop()
        self._restore_speed()
        with self._state_lock:
            self._state = RunnerState.CLOSED
        while True:  # fail jobs that never started
            try:
                item = self._jobs.get_nowait()
            except queue.Empty:
                break
            if item is not None:
                item[1].set_exception(RunnerClosed("runner is closed"))
        self._jobs.put(None)
        self._worker.join(timeout=2.0)
        if self._fr5 is not None:
            try:
                self._fr5.close()
            except Exception:  # noqa: BLE001 - best effort
                pass
