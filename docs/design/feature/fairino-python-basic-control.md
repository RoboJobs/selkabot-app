# Plan: Camera-level robot motions (Fairino FR5) for livestream.py

## Context
A GoPro is mounted on the FR5 flange. The goal is cinematic camera moves in the style of the "glambot": fast horizontal and vertical sweeps, crane moves and push-ins. Through all of them the camera **never rolls about its optical axis**, so the horizon stays level, while pan and tilt either follow keyframes or track a target point.

Motions are designed as keyframe files, validated offline against the robot's own kinematics, rehearsed on the robot, and then uploaded to the controller under a name. The operator app `sw/livestream.py` selects a published motion with a button, moves the robot to the motion's start pose on request, and runs the motion when recording starts. **The app never generates or modifies a trajectory.** It only loads and runs files that were validated beforehand.

Starting point:
- The Fairino Python SDK is installed from `third_party/fairino_sdk/linux` (see `pyproject.toml`, `[tool.uv.sources]`). That folder is git-ignored. `codegraph.json` in the repo root re-includes `linux/fairino/` and `linux/example/` in the CodeGraph index, so `codegraph_explore` can read the SDK.
- The SDK connection works (`sw/fairino_basic_query.py`). The robot is at `192.168.58.2` over a USB network adapter.
- The SDK manual is `third_party/fairino_sdk/fairino-doc-en-readthedocs-io-en-3.9.1.pdf`. `pdftotext -layout` makes it searchable.

Terminology: a **motion** is one named, validated trajectory, e.g. `sweep_left_fast`. Avoid the word "shot".

## Findings
- **The SDK already gives what is needed; MoveIt2 is not used.** MoveIt2 needs ROS2 Humble on Ubuntu 22.04, which this host doesn't have, and it solves collision-free A-to-B planning, not choreographed camera paths.
- **SDK functions used.** All are in `fairino/Robot.py`, class `RPC`:
  - Kinematics, with no motion: `GetInverseKinRef(type, desc_pos, joint_ref)`, `GetInverseKinHasSolution`, `GetForwardKin(joints)`.
  - Trajectory files: `TrajectoryJUpLoad(local_path)` stores the file at `/fruser/traj/<file>`. Also `TrajectoryJDelete(file)`, `LoadTrajectoryJ(name, ovl, opt=1)`, `MoveTrajectoryJ()`, `GetTrajectoryStartPose(name)`, `GetTrajectoryPointNum()`, `ComputeFileMD5(path)`. Reference flow: `linux/example/TestUploadTrajectoryJ.py`.
  - Motion and state: `MoveJ(joint_pos, tool, user, vel=...)`, `SetSpeed(pct)`, `StopMotion()`, `PauseMotion()`, `GetActualJointPosDegree()`, `GetActualJointSpeedsDegree()`, `GetRobotMotionDone()`, and `robot_state_pkg` (`motion_done`, `main_code`, `sub_code`, `jt_cur_pos`, `tl_cur_pos`, `actual_qd`). The state getters read `robot_state_pkg`, a cache refreshed by the SDK's own state thread. They do not block on the XML-RPC socket.
  - Setup and limits: `SetToolCoord`, `SetLoadWeight`, `SetLoadCoord`, `GetTCPOffset` (active tool), `GetJointSoftLimitDeg`, `GetRobotInstallAngle`, `ResetAllError`, `RobotEnable`, `Mode`.
  - Fallback store: `LuaUpload`, `ProgramLoad`, `ProgramRun`, `ProgramStop`, `GetProgramState`, `GetLuaList`, `LuaDelete`.
- **Not in the SDK** (checked by searching `Robot.py`): there is **no getter for the global speed** (only `SetSpeed`), and there is **no file rename** call for controller files. The design below works around both (§ Speed handling, § Atomic publish).
- **Return values.** Most SDK calls return an `int` error code (0 = OK). Getters return `(code, value)`. Motion calls first return `GetSafetyCode()` when it is non-zero. Calls made while the connection is flagged down return `-4`.
- **Blocking behaviour.** Most SDK calls wrap the XML-RPC call in `while flag: try ... except socket.error: flag = True`, i.e. they **retry socket errors forever**. A call can therefore block indefinitely when the network drops. The connection-up flag (`RPC.is_conect`) appears to be class-level, so a second `RPC` instance in the same process may share it (checked in Q5).
- **SDK pose** is `[x, y, z, rx, ry, rz]` in mm and degrees, for the active tool frame relative to the active user frame. The Euler convention is believed to be fixed-axis XYZ (`R = Rz·Ry·Rx`), but this is **not confirmed** (Q1).
- **Rotation blending.** When the controller is given Cartesian points (`MoveL`, `NewSpline*`), it blends the full rotation between consecutive points, and with `blendR` or the arc-transition spline type it also blends across neighbouring points. This was observed on the real robot during spline testing. The interpolation method is undocumented. Between two level orientations that differ in both pan and tilt, a rotation blend passes through orientations with roll. Feeding the controller only the keyframes is therefore not enough to keep the camera level (§ Architecture, "Level constraint").
- **Limits.** FR5 joint speed is about 180°/s at 100%, and TCP speed roughly 1 m/s. That is slower than a commercial glambot arm, so motions should be designed for "brisk", not "whip".

## Open questions resolved by Phase 0 (hardware spike)
| # | Question | Blocks |
|---|---|---|
| Q1 | Euler convention of `rx, ry, rz`. How to read tool N's coordinates (`GetTCPOffset` reports the active tool). | `pose.py` constants (the code can be written first, with the convention as a parameter); the tool check |
| Q2 | Trajectory file format: joints or poses, columns, header, whether it has timestamps or only a fixed period, allowed periods (TPD supports 2/4/8 ms). Does `TrajectoryJUpLoad` keep the local file's basename as the remote name? | `traj_file.py`, temp-name publishing |
| Q3 | Does `LoadTrajectoryJ(..., opt=1)` hold, linearly interpolate, or spline between samples? Does it preserve the sample timing, or re-time with its own profile? What does `ovl` do (uniform time scaling)? How does it interact with the global `SetSpeed`? Tracking and timing error. | the validity of the timing contract (§ Backend and timing contract) |
| Q4 | Does a `MoveJ` after `LoadTrajectoryJ` keep the loaded trajectory? Does `MoveTrajectoryJ` block? Is `motion_done` reliable at the end of a trajectory? | `MotionRunner.load/run`, completion criterion |
| Q5 | Can a second `Robot.RPC` connection in the same process send `StopMotion()` while the first is blocked in `MoveTrajectoryJ`, without disturbing the first (shared `is_conect`)? Stop latency. | `MotionRunner.stop` |
| Q6 | Measured roll: dense 8 ms pan+tilt trajectory vs a sparse two-keyframe version | confirms the "Level constraint" design |
| Q7 | What does `ComputeFileMD5` return for a missing file, an unreadable path, and a good file? Does it accept `/fruser/traj/<file>`? | remote verification rules |
| Q8 | FR5 kinematics data: soft limits (`GetJointSoftLimitDeg`), wrist/elbow/shoulder singular regions, and the configuration signature to use. | singularity and configuration checks |

## Backend and timing contract
The design depends on the controller replaying our samples with our timing. That is not known until Q2/Q3 are answered, so it is stated here as a contract plus a decision procedure instead of an assumption.

**Contract (target behaviour, tested in Phase 0).** The backend replays the stored samples in order, one per `sample_period_ms`, and the tool follows the planned path:

| Quantity | Initial acceptance (confirmed or tightened in Phase 0) |
|---|---|
| Tracking: planned vs logged pose, resampled at the planned sample times | position ≤ 3 mm; orientation ≤ 0.3° |
| Roll of the logged tool pose | ≤ 0.2° (§ Level constraint) |
| Timing: motion start (first joint moves > 0.05°) to settled, vs `duration_s / (ovl/100)` | within max(2%, 40 ms) |
| Stop: `stop()` call to all joint speeds < 0.5°/s | ≤ 150 ms |
| Repeatability: 5 runs, end-pose spread | ≤ 1 mm |

**How timing is represented.** The `TrajectoryBackend` writer takes `period_ms` and emits whatever Q2 shows the controller needs: a per-row time column if it has one, otherwise a fixed-period file whose period is a header value or implicit. `sample_period_ms` in `design.yaml` must be one of the periods the controller supports (default 8; allowed set confirmed by Q2, `{2, 4, 8}` expected).

**Decision after Phase 0** (the outcome is recorded in "Phase 0 results" and in the `robot.backend` config key):
- **A. `trajectory_file` backend meets the contract.** Use it. This is the expected case.
- **B. Path preserved, timing re-planned by the controller** (it follows the points but applies its own speed profile). Use the file backend in `timing_mode: controller`. Offline speed and acceleration checks become estimates, the rehearsal measures the real peaks, and `approved_ovl` is chosen so the *measured* peaks stay under the limits. Design durations become advisory. This is accepted only if the contract's tracking and roll rows still pass.
- **C. Neither meets the contract.** Use the `lua` backend, which must pass the same table. Streaming joint targets from Python (`ServoJ` at 8 ms) is **not** an acceptable fallback, because host-side timing breaks the requirement that the controller replays a stored, validated file.

**The Lua backend is a different backend, not a drop-in replacement.** It differs in timing and interpolation (the controller owns both between points), in stop semantics (`ProgramStop` + `StopMotion`), and in completion detection (`GetProgramState`). Its samples can't be 8 ms apart, because Lua program size and spline blending limit point density. The spacing is decided in Phase 0 (for example every 20–40 ms). With it, the validator's midpoint roll checks matter more, and the rehearsal limits stay the authority. Only the selected backend is implemented; the other is not built unless chosen. Both implement the `TrajectoryBackend` Protocol (§ Module interfaces).

## Caveats and risks
- **Thread safety of the SDK.** The SDK's XML-RPC proxy is not assumed to be thread-safe. All commands go through one worker thread, and `stop()` uses a dedicated second connection (§ Stop and shutdown behaviour).
- **Controller filename length.** The C API uses `char name[30]`. Names used: `motion_<name>.txt` and `tmp_<name>.txt`, so motion names are `^[a-z0-9_]{1,19}$` (the longest, `motion_<19>.txt`, is 30 characters).
- **Safety** is enforced in layers: controller collision detection with the correct payload; soft joint limits; offline validation; rehearsal ramp (10 → 100%); Move to Start at 10% global speed; the start-pose check before Record; independent stop channel; `StopMotion` on Stop, close, or any error. The physical e-stop remains the primary safety device. Operating rules (nobody in the cell during rehearsal; the app must not be running while publishing) belong in the README section added in Phase 6.
- **Calibration risk.** A mathematically level path can still give a tilted horizon if the camera tool frame is wrong. § Camera tool calibration makes calibration a gated, recorded step.

## Architecture

### Package map and dependency rules
New package `sw/robot/`. Arrows mean "may import".

```
            pure (no SDK, no Tk, no I/O beyond files)          hardware boundary
 ┌────────────────────────────────────────────────────┐   ┌──────────────────────┐
 │ pose.py  ← path.py ← validate.py                   │   │ fr5.py               │
 │    ↑                     ↑                         │   │  RobotClient Protocol│
 │ schema.py (design / validated / motion / camera)   │   │  FR5 (wraps RPC)     │
 │    ↑                                               │   │  FairinoError        │
 │ traj_file.py (writer/reader for the file backend)  │   └──────────▲───────────┘
 │ ui_state.py (pure UI state machine)                │              │
 └───────────────▲────────────────────────────────────┘              │
                 │                                                   │
      backend.py (TrajectoryBackend Protocol + file backend) ────────┤
      motionctl.py (CLI: plan/validate/compile/rehearse/publish) ────┤
      motion_runner.py (MotionRunner, used by livestream) ───────────┘
      sw/livestream.py → motion_runner.py, ui_state.py, schema.py (never fr5 directly)
```

Rules:
- Only `fr5.py` imports `fairino`.
- `pose`, `path`, `validate`, `schema`, `traj_file` and `ui_state` never import `fr5` or `fairino`. They are unit-testable without hardware.
- `validate.py` gets an IK/FK provider through a small Protocol (`Kinematics`). The real one is backed by `FR5`, and tests use a fake.
- `livestream.py` talks only to `MotionRunner`, `ui_state` and the configuration dataclasses.

### Data flow (offline → controller → app)
```
design.yaml ──plan──▶ samples (t, base-frame pose, pan, tilt)       [frames, pose, path]
            ──validate──▶ validated.json (joints via controller IK, checks)  [validate, fr5 IK/FK]
            ──compile──▶ build/motion_<name>.txt                    [backend / traj_file]
            ──rehearse──▶ temp upload run at increasing ovl, evidence summary
            ──publish──▶ controller file + motion.json (md5, start_joints, rehearsal summary, …)
livestream.py start ─▶ MotionRunner.connect_and_check(all motion.json)  (once)
button+confirm ─▶ load(name) │ Move to Start ─▶ move_to_start(name) @10%
Record ─▶ at_start(name)? ─▶ GoPro start ─▶ pre_roll ─▶ run(name) ─▶ done ─▶ post_roll ─▶ GoPro stop
```

### Frames and transforms
- **B, robot base frame.** Its z axis is assumed vertical (up). `validate` reads `GetRobotInstallAngle` and fails unless it is (0, 0) within 0.1°, and records the value. All planning, validation and the trajectory file work in B.
- **T, camera tool frame** (`tool_id`). Origin at the optical centre; **z = optical axis (forward), x = image right, y = image down.** A pose `[x, y, z, rx, ry, rz]` sent to the controller is the pose of T in B (work-object frame 0). The controller's work-object (user) frames are **not used**.
- **A, authoring frame.** Keyframe positions, `look_at`/`target` points and the workspace box are written in A, defined by the optional `anchor: {pos: [x, y, z], yaw_deg}`: A has its z axis parallel to B's z, translated by `pos` and rotated by `yaw_deg` about z. The default is identity (A = B). This lets a motion be authored relative to the subject's mark.
- **One conversion, then everything in B.** `design_to_base` converts once: `p_B = Rz(yaw)·p_A + pos` for every keyframe position and every target; `pan_B = pan_A + yaw` in angles mode. All later maths (sampling, `look_at` directions, IK, validation) runs in B, so the camera position and the target are always in the same frame before any direction is computed. The workspace box stays axis-aligned in A: the check transforms each sample position B→A and compares it with the box.
- The level constraint is invariant under this transform (a translation plus a rotation about the vertical), so authoring in A cannot break it.

### Orientation model
Orientation is **never stored or interpolated as a rotation.** It is built from two angles per sample, `(pan, tilt)`:
- **pan** (deg): heading of the optical axis about +Z, 0 = looking along +X, positive counter-clockwise seen from above.
- **tilt** (deg): elevation of the optical axis, positive = looking up.
- `R(pan, tilt) = Rz(pan) · R_level0 · Rx(tilt)`, with `R_level0` columns `x_cam = (0,−1,0)`, `y_cam = (0,0,−1)`, `z_cam = (1,0,0)`, and `Rx(tilt)` a rotation about the camera's own x axis. Because it rotates about x_cam, the image-right axis stays `Rz(pan)·(0,−1,0)`, which is horizontal, so the constraint `x_cam · ẑ_B = 0` (a zero **dot** product; a zero cross product would mean parallel axes) holds **exactly** for every (pan, tilt).

The design chooses one `orientation.mode`:
1. **`angles`:** every keyframe gives `pan` and `tilt`, and both are interpolated over time like any other channel (§ Path and timing). Explicit `pan` values are taken **literally**: 170 → −170 is a −340° turn, and a designer wanting a 20° turn writes 170 → 190. `check_pan_turn` rejects a step above 180° between adjacent keyframes unless `allow_long_pan: true`, and limits the cumulative pan to `max_cumulative_pan_deg` (default 360).
2. **`target`:** orientation is **derived at every sample** from the target and the sampled camera position, both in B: `d = target_B − p_cam_B`, `pan = atan2(d_y, d_x)`, `tilt = atan2(d_z, hypot(d_x, d_y))`, so the optical axis points at the target. Pan is unwrapped continuously along the samples (the first sample is wrapped to (−180°, 180°]), so a path that circles the target gives a smooth, multi-turn pan. The target comes from exactly one of:
   - **fixed target:** a design-level `orientation.target`, one point for the whole motion;
   - **target trajectory:** a `target` on **every** keyframe, interpolated over time with the same scheme as the position (§ Path and timing), so a target can move (e.g. follow a subject walking along a line).

   `pan`/`tilt` are not keyframed in target mode, and mixing modes within a design is rejected in v1.

**Degenerate and singular `look_at` cases** (target mode; `check_target_geometry` runs at every sample and every midpoint, and a failure names the sample index and time):
- `|d| ≥ min_target_distance_mm` (default 100). A zero-length or tiny vector is rejected.
- `hypot(d_x, d_y) ≥ min_horizontal_mm` (default 50). Near-vertical targets, where pan is undefined or unstable, are rejected.
- `|tilt| ≤ max_tilt_deg` (default 80, hard maximum 85). The same limit applies in angles mode.
- `|Δpan|` between adjacent samples ≤ `max_pan_rate_deg_s · period` (default 120°/s, 0.96° per 8 ms sample). This catches pan discontinuities (consecutive targets or a path passing close to the vertical through the target) as well as fast pans.

### Path and timing
Each channel is a **piecewise quintic Hermite** function of time, one polynomial per keyframe segment. The channels are `x, y, z` plus `pan, tilt` in angles mode, or `x, y, z` plus the target's `x, y, z` in target mode.
- At each keyframe a channel has a value, a velocity and an acceleration. **Acceleration is 0 at every keyframe**, so acceleration is continuous (jerk is not).
- **Velocity.** The first and last keyframes are at rest (velocity 0). Interior keyframes get a PCHIP-style velocity from the neighbouring segment slopes (zero if the slopes change sign, otherwise a weighted harmonic mean), computed with `scipy.interpolate.PchipInterpolator`'s derivative at the knots. A keyframe can override it with `vel: {x: .., pan: ..}` (mm/s or deg/s, any subset of channels), or set `stop: true` to bring all channels to rest there.
- **Why local, not a global spline.** A global interpolating spline rings and overshoots between keyframes, and editing one keyframe changes the whole path. The piecewise form is local (a keyframe only affects its two adjacent segments), has continuous velocity and acceleration, and works with **two or more keyframes** (two keyframes give a single minimum-jerk quintic). There is no six-point minimum.
- Segments must be at least `min_segment_s` long (default 0.05 s).

**Sampling rule** (exact). `p = sample_period_ms`, `T` = last keyframe time. Let `N = ceil(round(T·1000/p, 6))`, so an exact multiple of the period (including floating-point noise such as 1.2/0.008 = 150.00000000000003) does not gain an extra sample. Sample times are `t_i = i·p/1000` for `i = 0..N`: **`N + 1` samples, uniformly spaced.** For `t_i > T` (only the last sample, and only when T is not a multiple of p) the path is **clamped to the final keyframe**; because the last keyframe is at rest, holding it is exact, and the file keeps its uniform period. The last sample always equals the final keyframe pose. `duration_s = N·p/1000` (executed, ≥ T); `design_duration_s = T` is kept too. Example: T = 2.0, p = 8 → 251 samples; T = 2.003 → N = 251, 252 samples, the last at 2.008 s held.

**Spline behaviour checks** (in addition to the speed and acceleration limits applied to finite differences of every sample):
- `check_path_overshoot`: in every segment, each channel's sampled range must stay within the segment's endpoint values extended by `max(0.5 mm or 0.1°, overshoot_pct% of the segment span)` (default 2%). This catches overshoot that is still inside the workspace box.
- Velocity and acceleration peaks come from the finite differences of the 8 ms samples (dense enough that they bound the real peaks), and are reported per channel.
- Keyframe `vel` overrides are the designer's tool for fixing overshoot or shaping speed.

### Level constraint and what is verified
By construction every planned orientation has zero roll. Verification has three tiers that do different jobs:
1. **Offline validator (heuristic for the controller).** `check_roll` verifies `x_cam · ẑ_B` at every sample (from FK of the solved joints) and at every interval midpoint, under both SLERP and linear-Euler interpolation of the orientation (and linear interpolation of the position, which `check_pointing` also uses). **These midpoint checks are heuristics.** The controller's interpolation is undocumented, so two assumed methods can't bound its real behaviour. Limit: 0.05°.
2. **Rehearsal (authoritative for the robot).** `rehearse` logs the controller-reported tool pose (`tl_cur_pos`) at each state update and measures the roll of the real motion. Limit: ±0.2°. This is the acceptance criterion; the offline check can't replace it.
3. **Calibration (authoritative for the camera).** `tl_cur_pos` comes from the controller's own kinematics, so it cannot see a wrongly measured camera tool frame or mount. § Camera tool calibration covers that with a physical horizon test.

**Why 0.05° offline and ±0.2° at rehearsal.** The plan is exact, so the offline number only has to absorb numerical error (IK/FK round trip, Euler conversion, rounding). Anything above 0.05° means a bug or sampling that is too coarse, so the limit is tight. The rehearsal limit has to absorb real hardware effects: controller blending, servo lag, joint encoder quantization and kinematic calibration error. The two limits answer different questions on purpose.

### Camera tool calibration
The level constraint holds in the camera tool frame T. If T is wrong, a mathematically level motion can still give a tilted horizon or a camera that points off-target. The calibration lives in `sw/robot/camera_tool.yaml` (committed, edited by hand after calibrating):
```yaml
tool_id: 1
t_coord: [x, y, z, rx, ry, rz]    # flange → optical centre (mm, deg); z fwd, x image right, y image down
payload: {mass_kg: 0.25, cog_mm: [x, y, z]}   # SetLoadWeight / SetLoadCoord
calibration:
  approved: false                 # set true only after the procedure below passes
  date: null
  horizon_residual_deg: null      # ≤ 0.2 required
  pointing_residual_deg: null     # ≤ 0.5 required
  optical_centre_error_mm: null   # ≤ 3 required
  notes: ""
```
Procedure (`sw/robot/spike/calibrate_camera.py` guides the operator; thresholds are initial values, confirmed in Phase 0):
1. **Initial values.** `t_coord` from CAD or caliper measurements (lens centre relative to the flange centre, mounting rotation); payload from a scale and a centre-of-gravity estimate. Apply with `SetToolCoord`, `SetLoadWeight`, `SetLoadCoord`.
2. **Axis check.** Jog in the tool frame: +z must move the camera along its viewing direction (the image zooms into a target), +x must move the image content left (the camera moves right), and +y must move it up (the camera moves down).
3. **Optical axis and optical centre.** Put a crosshair marker at a known point in B. From at least three camera positions (≥ 0.5 m away, different pan/tilt), command `look_at` the marker, record a frame, and measure the marker's offset from the image centre (pixels × field of view per pixel). Pointing error ≤ 0.5°. A consistent offset that varies with distance indicates an optical-centre offset error, to be corrected in `t_coord` (≤ 3 mm).
4. **Image axes and horizon.** Film a physical horizontal reference (a laser line level or a levelled string) with the commanded orientation level at pans {−45°, 0°, 45°} and tilts {−20°, 0°, 20°}. Fit the line in each frame: its slope is the roll residual, ≤ 0.2°. Correct the roll component (`rx` of `t_coord`, about the optical axis) and repeat.
5. **Approval.** Record the residuals and date, and set `approved: true`.

Enforcement: `validate` refuses unless `approved` is true, compares the controller's tool coordinate with `camera_tool.yaml` (within 0.1 mm / 0.05°), and records `camera_tool_sha256` in `validated.json`. `publish` refuses an unapproved file. The startup check compares the controller's tool with `camera_tool.yaml` again. Recalibrating changes the hash, so every validation and publication made with the old calibration becomes stale and is flagged until it is re-validated and re-rehearsed.

### Threading and the motion runner
- **Worker.** `MotionRunner` owns **one daemon `threading.Thread`** that consumes a `queue.Queue` of jobs; each public method (except `stop()`/`close()`) enqueues a job and returns a `concurrent.futures.Future`. A daemon thread is used on purpose, not a `ThreadPoolExecutor`: SDK calls retry socket errors forever (§ Findings), so a call can block indefinitely, and such a thread must not keep the application from exiting.
- **State.** `RunnerState` is one of `IDLE`, `BUSY`, `STOPPING`, `FAULT(reason)` (the robot reported an error or a timeout and was confirmed stationary), `UNKNOWN(reason)` (a stop could not be confirmed, or the worker is unresponsive), `CLOSED`.
- **Polling uses cached state.** Completion and position polling read `robot_state_pkg` (non-blocking cache), never an RPC call, so the polling loop can't block on the socket.
- **Completion criterion** (decided by Q4): `motion_done` when it proves reliable; otherwise, the joints within 0.5° of the final sample's joints and `actual_qd` below 0.5°/s for 100 ms.
- **Watchdog.** `run()` starts a `threading.Timer` (separate from the worker, which may be blocked) with `deadline = duration_s / (ovl/100) + max(2 s, 25% of that)`. On expiry it calls `stop()` and fails the run's future with `MotionTimeout`.
- **Speed handling.** `SetSpeed` is global and persistent, and the SDK has **no getter**, so the previous value can't be saved and restored. Instead: the runner sets the speed explicitly before every motion command (`move_to_start`: `move_to_start_speed_pct`, 10; `run`: the motion's `approved_ovl`; `rehearse`: the rehearsal step) and restores **`safe_default_speed_pct`** (config, default 10) in a `finally` after completion, error, stop, timeout and at `close()`. It also sets it in `prepare()` at startup, which covers a previous process that was killed. This is implemented as `FR5.speed(pct)`, a context manager. Restoring is best-effort and logged. How `ovl` in `LoadTrajectoryJ` interacts with the global speed is measured in Q3.
- **UI hand-off.** `LivestreamApp` gets `_run_robot(future, status, on_done, on_error)` that mirrors the existing `_run`: its done-callback posts the finish closure onto the existing `self._events` queue, which `_poll` drains on the Tk thread. **Tk widgets are never touched from the robot worker.** Each robot operation carries an increasing `op_id`; a callback whose `op_id` is stale is ignored.

### Stop and shutdown behaviour
- **Independent stop channel.** `connect_and_check` opens a second `FR5` connection used only for `stop()`. **If it can't be opened, every motion is disabled** with the reason "no independent stop channel" (fail closed; plain recording still works). Q5 verifies that a second connection works in the same process and does not disturb the first.
- **`stop()` never raises, is idempotent, and returns a `StopResult(confirmed, detail)`.** Sequence:
  1. `StopMotion()` on the stop channel, up to 3 attempts 100 ms apart.
  2. If all attempts return errors, one `PauseMotion()`.
  3. Confirm: poll `actual_qd` until every joint is below 0.5°/s for 200 ms, within `stop_confirm_timeout_s` (default 2.0).
  4. Confirmed → the runner enters `FAULT(reason)` if the stop followed an error, or `IDLE` otherwise. **Not confirmed** (all calls failed, or the timeout expired) → `UNKNOWN(reason)`, logged.
  A second `stop()` while `STOPPING` returns the pending result.
- **UNKNOWN outcome.** All motion controls (motion buttons, Move to Start, motion runs) are disabled and the status bar shows "Robot state unknown: use the e-stop or teach pendant, then restart the app". Plain recording (no motion) remains available. There is no automatic recovery from `UNKNOWN`; the app restarts.
- **Worker blocked inside an SDK call.** The watchdog (above) calls `stop()` through the stop channel regardless of the worker. If the run's future doesn't resolve after the stop, the state becomes `UNKNOWN`, the blocked daemon thread is abandoned and further jobs are rejected.
- **`close()`** is idempotent: `stop()`; state → `CLOSED` (new jobs rejected with `RunnerClosed`); signal the worker and join it for at most 2 s; call `CloseRPC` on both connections (best effort); fail the pending futures with `RunnerClosed`; restore the safe default speed (best effort). If the worker is still blocked it is abandoned and logged.
- **`motion_done` never changes.** Covered by the watchdog: the run times out, `stop()` runs, and the outcome is `FAULT("motion timeout")` or `UNKNOWN` depending on whether the stop was confirmed.

### Controller-file verification
Two different hashes are involved and are never mixed up:
- **local MD5:** `hashlib.md5` over the bytes of the generated file on disk, computed at `compile`.
- **remote MD5:** `ComputeFileMD5("/fruser/traj/<file>")`, computed **by the controller** on the file it stores.

`motion.json.md5` holds the local MD5 of the file as published, and `publish` stores it only after the remote MD5 has matched it. The startup check compares the live remote MD5 with it. Every outcome other than a match **disables the motion (fail closed)**, with a distinct reason:

| Outcome | Reason shown |
|---|---|
| the controller has no such file (SDK error or empty result; exact form from Q7) | `file missing on controller` |
| the SDK returns an error code for the hash call | `hash query failed (code N)`; retried twice before giving up (transport errors only) |
| empty or non-32-hex-character hash | `controller returned an invalid hash` |
| hashes differ (compared case-insensitively) | `md5 mismatch` (never retried) |

### Atomic publishing
The SDK has no rename call, so the publish sequence verifies a temporary upload before it touches the live file and writes the metadata last, atomically:
1. **Preconditions:** a fresh `validated.json` (design and camera-tool hashes current), `camera_tool.yaml` approved, rehearsal evidence at ≥ the requested ovl, and the configured backend.
2. **Temporary upload:** upload the build file as `tmp_<name>.txt`. Its remote MD5 must equal the local MD5; otherwise delete the temporary file and abort. Nothing live has changed.
3. **Set aside old metadata:** rename `motion.json` → `motion.json.prev` (skipped if there is none).
4. **Final upload:** upload `motion_<name>.txt` and verify the remote MD5. On failure delete the file and abort, leaving `.prev` in place.
5. **Write metadata:** write `motion.json.tmp`, then `os.replace` it onto `motion.json` (atomic).
6. **Clean up:** delete the temporary controller file and `.prev`.

Failure semantics: before step 4 nothing visible changed. Between steps 3 and 5 the app's startup check fails closed (no metadata or an md5 mismatch means the motion is disabled), so an interrupted publish can't produce a runnable half-published motion. **Recovery = re-run `publish`** (it is idempotent). `list` reports orphans: `tmp_*` files and controller files without `motion.json`. Step 4 does overwrite the previously published file, so a motion is briefly unavailable during publishing; the app must not be running then (README rule).

### UI state machine
`sw/robot/ui_state.py` holds a pure `UiState` enum and `transition(state, event) -> (new_state, actions)`. The Tk app feeds events in and executes the returned actions, so the logic is testable without Tk.

States: `UNAVAILABLE` (no robot: `--no-robot` or startup failure), `CHECKING`, `IDLE`, `LOADING`, `MOVING_TO_START`, `STARTING_RECORDING`, `PRE_ROLL`, `RUNNING`, `POST_ROLL`, `STOPPING`, `FAULT`, `UNKNOWN`.

| From | Event | To | Actions |
|---|---|---|---|
| `CHECKING` | `startup_checked` | `IDLE` | enable passing motions, reset an invalid saved selection |
| `IDLE` | `select_confirmed(name)` | `LOADING` | `load(name)` |
| `LOADING` | `loaded` / `load_failed` | `IDLE` / `IDLE` | status; on failure mark the motion invalid for the session |
| `IDLE` | `move_to_start` | `MOVING_TO_START` | `move_to_start(name)` |
| `MOVING_TO_START` | `move_done` / `move_failed` | `IDLE` / `FAULT` | status |
| `IDLE` | `record_pressed` (motion, at start) | `STARTING_RECORDING` | start the GoPro |
| `IDLE` | `record_pressed` (motion, not at start) | `IDLE` | status "not at start (max error X.X°)"; nothing starts |
| `IDLE` | `record_pressed` (no motion) | (existing flow) | unchanged |
| `STARTING_RECORDING` | `recording_started` | `PRE_ROLL` | schedule `pre_roll_elapsed` (keep the `after` handle) |
| `STARTING_RECORDING` | `recording_failed` | `IDLE` | status; the robot never starts |
| `PRE_ROLL` | `pre_roll_elapsed` | `RUNNING` | `run(name)` |
| `RUNNING` | `motion_done` | `POST_ROLL` | schedule `post_roll_elapsed` if `auto_stop_after_motion` |
| `RUNNING` | `motion_failed` | `STOPPING` | `runner.stop()`, stop recording, show the error |
| `POST_ROLL` | `post_roll_elapsed` | `STOPPING` | stop recording |
| `PRE_ROLL`, `RUNNING`, `POST_ROLL` | `stop_pressed` | `STOPPING` | cancel pending `after` handles, `runner.stop()`, stop recording (once) |
| any non-`IDLE` state | `close` | (closing) | cancel `after` handles, `runner.stop()`, stop recording if recording, then `runner.close()` and destroy |
| `STOPPING` | `stop_confirmed` + `recording_stopped` | `IDLE` or `FAULT` | existing download flow |
| `STOPPING` | `stop_unconfirmed` | `UNKNOWN` | banner |
| any | events not listed for that state | unchanged | ignored, logged |

Races:
- **Close during pre-roll:** the `after` handle is cancelled, so `run()` never starts; `runner.stop()` and the recording stop still run.
- **Motion finishes as Stop is pressed:** `STOPPING` ignores `motion_done`/`motion_failed`, and a `_stop_requested` guard makes the recording stop exactly once.
- **Recording starts but the motion fails, or the GoPro starts and the robot's `run()` fails:** both go to `STOPPING` (stop the robot, stop the recording, show the error). The clip is kept and processed as usual.
- **GoPro start fails:** the robot never starts, because `run()` is only requested after `recording_started`.
- **Selection changes while an operation is pending:** the motion buttons and Move to Start are enabled only in `IDLE`, and events in any other state are ignored by the table. `askyesno` runs a nested event loop, so after "Yes" the handler re-checks that the state is still `IDLE` and the motion is still enabled; otherwise it discards the choice with a status message.
- **Late results:** stale `op_id` callbacks are ignored.

### Disabled-motion behaviour
- All motion buttons stay **visible**. A motion that failed the startup check is **disabled**, drawn in a distinct style, and shows its reason in the status bar on mouse-over.
- If the saved `"motion"` is invalid or missing, the selection resets to "No motion" **in memory** with a status message. The config file keeps the saved value until the user picks another, so a transient outage doesn't lose it.
- If the robot (or its stop channel) is unreachable, every motion is disabled with "robot unreachable" and plain recording works.
- **No MD5 re-check after startup** (the hash is verified once, as required). Cheap load-time guards remain: after `LoadTrajectoryJ`, `load()` checks `GetTrajectoryPointNum() == num_points` (and the start pose, if Q2 shows that comparison is meaningful). A `load()` or `run()` failure marks the motion **invalid for the rest of the session**: its button is disabled with "invalid at load/run: …" and the runner state is checked.

### Error handling contract
- `FR5._call(name, *args)` invokes the SDK method. A non-zero error code raises `FairinoError(code, name)`. For `(code, value)` returns it returns `value`.
- `FairinoError.__str__` includes the call name and the code.
- `MotionRunner` turns exceptions into Future exceptions. The UI shows `f"Robot error: {e}"` in the status bar.
- `motionctl` exits with code 1 and a one-line message on any `FairinoError` or validation failure, and prints the full report for validation failures.

## Data schemas
All files are UTF-8. Units are mm and degrees unless stated otherwise. Every schema has a matching frozen dataclass in `schema.py` with `from_dict`/`to_dict`. Loaders raise `SchemaError(path, field, reason)` on missing or wrong fields.

### `sw/robot/motions/<name>/design.yaml` (hand-written, committed)
Angles mode:
```yaml
name: sweep_left_fast          # must equal folder name; ^[a-z0-9_]{1,19}$
tool_id: 1                     # camera tool frame on the controller
sample_period_ms: 8            # one of the controller's supported periods (Q2)
anchor: {pos: [0, 0, 0], yaw_deg: 0}     # optional authoring frame A; default = base frame
seed_joints: [0, -90, 90, -90, -90, 0]   # IK seed for the first sample
orientation: {mode: angles}
limits:                        # all optional; defaults shown
  max_tcp_speed_mm_s: 800
  max_joint_speed_deg_s: 120   # scalar or list of 6
  max_joint_acc_deg_s2: 600
  max_joint_step_deg: 2.0      # per-sample jump => wrist flip / singularity
  max_wrist_step_deg: 1.0      # J4-J6 per sample
  max_cumulative_pan_deg: 360
  allow_long_pan: false
  overshoot_pct: 2
  min_wrist_margin_deg: 15     # |J5| from its singular position
  max_roll_deg: 0.05
  max_pointing_error_deg: 0.1
workspace:                     # box in the authoring frame A
  min: [200, -700, 150]
  max: [900,  700, 1100]
keyframes:                     # t strictly increasing, first t == 0, at least 2
  - {t: 0.0, pos: [600, -400, 450], pan: 20,  tilt: -5}
  - {t: 1.2, pos: [600,  400, 450], pan: -20, tilt: -5, vel: {y: 400}}   # optional velocity override
  - {t: 2.0, pos: [650,  400, 700], pan: -20, tilt: 5}
```
Target mode, fixed target (or a `target` on every keyframe instead, for a target trajectory):
```yaml
orientation:
  mode: target
  target: [0, 0, 300]          # fixed point, authoring frame A
  min_target_distance_mm: 100
  min_horizontal_mm: 50
  max_tilt_deg: 80
  max_pan_rate_deg_s: 120
keyframes:
  - {t: 0.0, pos: [600, -400, 450]}
  - {t: 2.0, pos: [600,  400, 450]}
```
Schema rules: at least 2 keyframes; the first and last are at rest (no non-zero `vel`, `stop` allowed there); in `angles` mode every keyframe has `pan` **and** `tilt` and none has `target`; in `target` mode exactly one of the design-level `target` or a `target` on every keyframe, and no keyframe has `pan`/`tilt`; `|tilt| ≤ 85`; `sample_period_ms` is in the allowed set; segments ≥ `min_segment_s`; `vel` keys are valid channels for the mode.

### `sw/robot/camera_tool.yaml` (hand-edited, committed)
See § Camera tool calibration.

### `sw/robot/motions/<name>/validated.json` (generated by `validate`; **git-ignored**)
- `name`, `design_sha256`, `camera_tool_sha256`, `created` (ISO 8601), `backend`, `sample_period_ms`, `tool_id`, `install_angle`, `tcp_offset[6]` (read from the controller).
- `num_points` (= `N + 1`), `duration_s`, `design_duration_s`.
- `samples`: `{t: [N+1], pose: [N+1][6], pan: [N+1], tilt: [N+1], joints: [N+1][6]}` (pose in B).
- `peaks`: `{tcp_speed_mm_s, joint_speed_deg_s[6], joint_acc_deg_s2[6], max_joint_step_deg, max_roll_deg, max_pointing_error_deg, max_fk_pos_err_mm, max_fk_rot_err_deg, min_wrist_margin_deg}`.
- `checks`: a list of `{name, passed, detail}`. The file is written only when every check passes. Otherwise the report is printed and nothing is written.

### `sw/robot/motions/<name>/motion.json` (generated by `publish`; read by the app; **committed**)
```json
{
  "name": "sweep_left_fast",
  "backend": "trajectory_file",
  "controller_file": "/fruser/traj/motion_sweep_left_fast.txt",
  "md5": "…",
  "num_points": 251,
  "sample_period_ms": 8,
  "duration_s": 2.0,
  "design_duration_s": 2.0,
  "start_joints": [..6..],
  "start_pose": [..6..],
  "tool_id": 1,
  "tcp_offset": [..6..],
  "approved_ovl": 100,
  "design_sha256": "…", "camera_tool_sha256": "…", "validated_sha256": "…",
  "rehearsal": {"ovl": 100, "max_roll_deg": 0.12, "max_pos_err_mm": 1.8, "date": "2026-11-24"},
  "published_at": "2026-11-24T10:00:00"
}
```
Cross-field rules, enforced by `MotionConfig.check_consistent()`: `controller_file == "/fruser/traj/motion_<name>.txt"`; `duration_s == (num_points − 1) · sample_period_ms / 1000`; `1 ≤ approved_ovl ≤ rehearsal.ovl ≤ 100`; `len(start_joints) == len(tcp_offset) == 6`; `backend` is a known backend.

### Which files are committed
- **Committed:** `design.yaml` (authored), `motion.json` (small published metadata; carries the rehearsal summary so the approval evidence travels with it), `camera_tool.yaml`, `codegraph.json`.
- **Git-ignored (reproducible or machine-local):** `validated.json`, `sw/robot/build/`, rehearsal logs under `sw/robot/logs/` and `motions/*/rehearsal*.{csv,json}`, `motions/*/*.prev`, `motions/*/*.tmp`.
- `motionctl verify NAME` regenerates the trajectory from `design.yaml` and `camera_tool.yaml` and compares its MD5 with `motion.json`. Joint solutions come from the controller's IK, so a mismatch with an unchanged design hash is reported as a **warning** (controller kinematics or SDK changed), not as an error.

### `sw/livestream_config.json` (new keys, merged by the existing `save_config` pattern)
- `"motion"`: the selected motion name, or `""` for no motion.
- `"robot"`: `{"ip": "192.168.58.2", "backend": "trajectory_file", "start_tolerance_deg": 0.5, "move_to_start_speed_pct": 10, "safe_default_speed_pct": 10, "stop_confirm_timeout_s": 2.0, "pre_roll_s": 0.5, "post_roll_s": 1.0, "auto_stop_after_motion": true}`.

## Module interfaces
Signatures are binding for the implementer. Private helpers are free.

- **`frames.py`**: `design_to_base(design) -> Design` (applies `anchor` to positions, targets, `pan`), `base_to_authoring(points, anchor) -> ndarray` (used by the workspace check).
- **`pose.py`**
  - `level_rotation(pan_deg, tilt_deg) -> np.ndarray(3,3)`
  - `look_at_angles(pos, target) -> tuple[pan, tilt]` (raises `DegenerateTarget` for a zero vector; the validator reports the geometry limits)
  - `rotation_to_sdk_euler(R) -> (rx, ry, rz)` and `sdk_euler_to_rotation(rx, ry, rz) -> R`. The convention is the module constant `SDK_EULER = "xyz"`, set by Q1.
  - `roll_error_deg(R) -> float`: the angle of x_cam out of the horizontal plane.
  - `pointing_error_deg(R, pos, target) -> float`
  - `unwrap_deg(seq) -> list` (continuous unwrap used in target mode)
- **`path.py`**
  - `Sample(t, pos, pan, tilt, target)` (all in B; `target` is `None` in angles mode)
  - `hermite_quintic(...)` and `keyframe_velocities(times, values, overrides) -> list` (PCHIP defaults, zero ends)
  - `build_path(design: Design) -> Callable[[float], Sample]`
  - `sample_count(T, period_ms) -> int`, the rule `N + 1` above
  - `sample_path(design) -> list[Sample]`, uniform spacing, last sample clamped to the final keyframe
  - `to_sdk_pose(sample) -> list[6]`
- **`validate.py`**
  - `class Kinematics(Protocol)`: `ik(pose6, seed6) -> list[6] | None`, `fk(joints6) -> list[6]`, `install_angle() -> tuple`, `joint_soft_limits() -> list[(lo, hi)]`, `tool_offset(tool_id) -> list[6]`.
  - `validate(design, camera_tool, samples, kin) -> ValidationReport`, holding `passed: bool`, `checks`, `peaks` and `joints`.
  - Checks, each a separate function so it can be tested alone: `check_camera_tool`, `check_install_angle`, `check_target_geometry` (target mode: distance, horizontal distance, tilt, pan rate), `check_pan_turn`, `check_path_overshoot`, `check_ik`, `check_config_consistency` (the configuration signature, from Q8, is the same at adjacent samples), `check_joint_steps`, `check_wrist` (J4–J6 step and speed), `check_joint_limits_and_winding` (soft limits from the controller, and the total travel of each joint ≤ 360°), `check_singularity` (J5 margin, elbow and shoulder margins, and a manipulability measure from FK finite differences on every 10th sample; thresholds from Q8), `check_joint_speed_acc` (finite differences), `check_tcp_speed`, `check_workspace` (in A), `check_roll` (samples and midpoints, SLERP and linear-Euler, heuristic), `check_pointing` (every design in target mode; samples and midpoints, ≤ `max_pointing_error_deg`), `check_fk_roundtrip` (< 0.1 mm, < 0.05°).
- **`traj_file.py`**: `write(validated: Validated, path: Path, period_ms: int) -> None` and `read(path) -> list[list[float]]`, for round-trip tests. The format follows Q2.
- **`backend.py`**: `class TrajectoryBackend(Protocol)` with `compile(validated, out_dir) -> Path`, `upload(fr5, local_path, remote_name)`, `remote_path(name) -> str`, `load(fr5, config)`, `start(fr5, config)`, `is_done(fr5, config) -> bool`, `stop(fr5)`; and `TrajectoryFileBackend`. A `LuaBackend` exists only if Phase 0 selects it.
- **`schema.py`**: `Design`, `Keyframe`, `CameraTool`, `Validated`, `MotionConfig`, `RobotSettings`, plus `load_design(path)`, `load_camera_tool(path)`, `load_validated(path)`, `load_motion_configs(motions_dir) -> dict[str, MotionConfig]` (skips folders without `motion.json`), `sha256_file(path)`, `MotionConfig.check_consistent()`.
- **`fr5.py`**
  - `class FairinoError(RuntimeError)`, carrying `code` and `call`.
  - `class RobotClient(Protocol)`: the exact SDK methods listed in Findings, used by `FR5`, so fakes can implement it.
  - `class FR5`, which takes `client: RobotClient`:
    - `connect(ip) -> FR5` (classmethod)
    - `ik`, `fk`, `install_angle`, `joint_soft_limits`, `tool_offset` (satisfying `Kinematics`)
    - `joints() -> list[6]`, `joint_speeds() -> list[6]`, `tcp_pose()`, `error_codes() -> (main, sub)`, `motion_done() -> bool`
    - `move_j(joints, tool, speed_pct)`
    - `speed(pct)`: context manager that sets the speed and restores `safe_default_speed_pct` in `finally`
    - `upload_traj(local_path)`, `delete_traj(file)`, `file_md5(remote_path) -> str`
    - `load_traj(remote_path, ovl)`, `run_traj()`, `stop()`
    - `prepare()` (reset errors, enable, auto mode, safe default speed)
- **`ui_state.py`**: `UiState`, `Event`, `Action` enums/dataclasses and `transition(state, event, context) -> (UiState, list[Action])` (the table above).
- **`motion_runner.py`**
  - `class MotionRunner`, which takes `connect: Callable[[], FR5]` (so tests can inject fakes), `backend: TrajectoryBackend` and `settings: RobotSettings`. Methods:
    - `connect_and_check(configs) -> Future[dict[str, str | None]]`: maps each name to None if OK, or to the reason it is disabled.
    - `load(name) -> Future[None]`
    - `move_to_start(name) -> Future[None]`
    - `at_start(name) -> tuple[bool, float]`: synchronous, reads the state cache, and returns (ok, max_err_deg).
    - `run(name) -> Future[None]`
    - `stop() -> StopResult`
    - `close() -> None`
    - `state -> RunnerState`
- **`motionctl.py`**: run as `python -m sw.robot.motionctl <cmd> NAME [--ip IP]`, with these commands:
  - `plan NAME [--preview]`
  - `validate NAME`
  - `compile NAME`
  - `rehearse NAME --ovl 10,25,50,100`
  - `publish NAME --ovl N`
  - `verify NAME`
  - `list`
  - `remove NAME`
  - `new NAME --template sweep|crane|orbit|push_in`, which writes a starter `design.yaml` from `templates.py`.

## Phases
**Gate rule for every phase:** run `uv run pytest sw/tests/ -q` (the full suite). Every test passes, with no new skips or failures, and `uv run ruff check sw/ && uv run ruff format --check sw/` is clean (line length 88, rules E/F/I). Each phase ends in its own commit on `feature/fairino-python-basic-control`. **No unit test may require the robot, the GoPro, or the network.** Hardware-only scripts live outside `sw/tests/` or are marked `@pytest.mark.hardware` and skipped by default (register the marker in `pyproject.toml`).

### Phase 0: Hardware spike and calibration (needs the operator at the robot)
- **Work:** write `sw/robot/spike/` scripts. Each prints its findings and doesn't need tests.
  - Q1 `euler_probe.py`: jog to 3–4 poses and print `GetActualTCPPose` next to the scipy conversion of `GetForwardKin`, for each Euler candidate; find how to read tool N's coordinates.
  - Q2/Q3 `traj_probe.py`: record an 8 ms TPD trajectory, download it, write a small file in the same format, upload (including the basename check), `LoadTrajectoryJ`/`MoveTrajectoryJ` at 10%, and log tracking, timing, and `ovl` behaviour against the contract table.
  - Q4/Q5: the same script tests a `MoveJ` after the load, whether `MoveTrajectoryJ` blocks, `motion_done` at the end, and `StopMotion` from a second connection (with stop latency and the shared `is_conect` check).
  - Q6 `roll_probe.py`: a dense pan+tilt trajectory vs a sparse two-point `MoveL`, logging roll from `tl_cur_pos`.
  - Q7 `md5_probe.py`: `ComputeFileMD5` on a good, a missing and an unreadable path.
  - Q8 `kinematics_probe.py`: soft limits, the singular regions of the FR5, the configuration signature, and manipulability values along sample paths, to set the thresholds.
  - `calibrate_camera.py`: the procedure in § Camera tool calibration; produces the approved `camera_tool.yaml`.
- **Done when:** the answers to Q1–Q8, the backend decision (A, B or C), the confirmed acceptance numbers and an approved `camera_tool.yaml` are written into a "Phase 0 results" section appended to this doc, including a sample trajectory file excerpt.
- Phases 1–3 can be developed in parallel with Phase 0, because they don't depend on its answers beyond constants.

### Phase 1: Pure geometry and schemas (`frames.py`, `pose.py`, `path.py`, `schema.py`, `templates.py`)
- **Work:** as specified in Module interfaces. Add `scipy` and `pyyaml` to `pyproject.toml` dependencies.
- **Tests:**
  - `sw/tests/test_robot_pose.py`:
    - for 1,000 random (pan, tilt) with |tilt| ≤ 80: `R` is orthonormal with det = 1, `|x_cam · ẑ| < 1e-12`, the optical axis heading equals pan and its elevation equals tilt;
    - `look_at_angles` for known targets (ahead, left, above-left, below) and `DegenerateTarget` for a zero vector;
    - `pointing_error_deg` is 0 for `R(look_at_angles(...))`;
    - SDK Euler round-trip for random rotations;
    - `roll_error_deg` equals the injected roll for a rotated frame;
    - `unwrap_deg` across ±180 and across multiple turns.
  - `sw/tests/test_robot_frames.py`: `design_to_base` with a yaw anchor: positions, targets and `pan` convert correctly (`pan_B = pan_A + yaw`); the level property holds in B for any yaw; the workspace box check in A (a rotated design inside its box passes, a sample just outside fails); identity anchor is a no-op.
  - `sw/tests/test_robot_path.py`:
    - the path passes through every keyframe (pos ±1e-6 mm, pan/tilt ±1e-6°);
    - 2 keyframes and 3 keyframes both build (a single minimum-jerk segment for 2);
    - velocity and acceleration are zero at t = 0 and t = T; acceleration is 0 at every keyframe;
    - **locality:** changing one keyframe changes the path only in its two adjacent segments;
    - PCHIP default velocities: zero at a local extremum; no overshoot for monotone keyframes;
    - `vel` override is honoured at the keyframe; `stop: true` gives zero velocity there;
    - **sampling rule:** T = 2.0, p = 8 → 251 samples; T = 1.2, p = 8 → 151 (no float-noise extra sample); T = 2.003, p = 8 → 252 samples, last at 2.008 equal to the final keyframe pose; spacing is uniform; the first sample is at t = 0;
    - explicit pan values are literal (170 → −170 is a −340° turn);
    - target mode: a fixed target gives `optical axis ∥ (target − position)` at every sample (pointing error < 1e-9°); a per-keyframe target trajectory interpolates the target over time; pan is continuous around a target the path circles (multi-turn unwrap);
    - every sample has `roll_error_deg < 1e-9` in both modes.
  - `sw/tests/test_robot_schema.py`:
    - a valid angles design and a valid target design load;
    - each invalid case raises `SchemaError` with the field name: bad name regex, name ≠ folder, non-increasing t, first t ≠ 0, fewer than 2 keyframes, `pan`/`tilt` missing in angles mode, `pan` present in target mode, mixed modes, both design-level and keyframe targets, |tilt| > 85, non-zero velocity at the first or last keyframe, `sample_period_ms` not in the allowed set, segment shorter than `min_segment_s`, wrong vector length, an unknown `vel` channel;
    - `MotionConfig` round-trips and `check_consistent()` rejects each cross-field violation (controller filename, duration vs points × period, `approved_ovl` out of range or above the rehearsal ovl, vector lengths, unknown backend);
    - `load_motion_configs` skips folders without `motion.json`;
    - `CameraTool` loading and the `approved` flag.
  - `templates`: each template produces a design that loads and samples.
- **Done when:** the gate passes.

### Phase 2: Validator (`validate.py`)
- **Work:** as specified. Peaks are computed with finite differences on the sample times.
- **Tests** (`sw/tests/test_robot_validate.py`, with a `FakeKinematics` whose `ik` is a deterministic analytic stand-in and whose failures can be injected):
  - a well-formed angles design and a well-formed target design pass all checks;
  - IK returning None at one sample fails `check_ik` and names the sample index and time;
  - an injected 30° jump on J4 fails `check_joint_steps` and `check_wrist`;
  - an injected configuration flip (J5 changes sign) between adjacent samples fails `check_config_consistency`;
  - a path crossing the wrist margin (|J5| < `min_wrist_margin_deg`) fails `check_singularity`;
  - a joint beyond the controller's soft limit fails; a joint travelling more than 360° fails the winding check;
  - a too-fast design (shrink keyframe times) fails speed/acc, and the peaks are reported;
  - a point outside the box, and a keyframe `vel` override that overshoots a segment inside the box, fail `check_workspace` and `check_path_overshoot` respectively;
  - `check_pan_turn`: |Δpan| > 180° fails unless `allow_long_pan`; cumulative pan above the limit fails;
  - **target geometry:** a target coinciding with the camera position (zero vector), a target within `min_target_distance_mm`, a target directly above the path (horizontal distance < `min_horizontal_mm`), |tilt| above the limit, and a path passing close to the vertical through the target (pan rate above the limit) each fail `check_target_geometry` with the sample index;
  - `check_pointing` passes for a correct target design and fails when the fake IK/FK introduces a 0.5° pointing offset;
  - coarse sampling (e.g. a period of 500 ms with a large simultaneous pan and tilt change) fails the midpoint `check_roll` and `check_pointing` while dense sampling passes, which proves the checks are meaningful;
  - an FK offset injected into the fake fails `check_fk_roundtrip`;
  - `check_camera_tool`: unapproved calibration, and a controller tool offset that differs from `camera_tool.yaml`, both fail;
  - `check_install_angle`: a non-zero install angle fails;
  - the report serializes to the `validated.json` schema.
- **Done when:** the gate passes.

### Phase 3: Robot boundary (`fr5.py`) with fakes
- **Work:** `FR5` wraps the SDK through `RobotClient`. Add `sw/tests/fakes/fake_robot.py` with a `FakeRobotClient` that:
  - records every call;
  - keeps joint state, which `MoveJ` sets instantly;
  - fakes `robot_state_pkg` (`motion_done`, `jt_cur_pos`, `actual_qd`, `main_code`);
  - stores uploaded files in memory and computes `ComputeFileMD5` with `hashlib.md5` (and can return an empty hash, an error code, or a wrong hash on demand);
  - can inject error codes per method;
  - can make `MoveTrajectoryJ` take N polls to complete, never complete, or block on an event;
  - can fail `connect` for a second connection.
- **Tests** (`sw/tests/test_robot_fr5.py`):
  - every wrapper calls the right SDK method with the right argument types and order;
  - a non-zero code raises `FairinoError` with the call name;
  - `(code, value)` unpacking;
  - `move_j` runs inside `speed(pct)`;
  - **the `speed(pct)` context manager restores `safe_default_speed_pct` on normal exit and on an exception, and tolerates a failing restore;**
  - `file_md5` returns the controller MD5 string, and the empty-hash and error cases surface as distinct exceptions.
- **Done when:** the gate passes. The module imports `fairino` lazily, inside `connect`, so tests don't need the SDK's native library.

### Phase 4: Backend, trajectory file and CLI (`backend.py`, `traj_file.py`, `motionctl.py`); needs Phase 0 results
- **Work:**
  - `traj_file.write/read` and `TrajectoryFileBackend` per the Q2/Q3 results (or `LuaBackend` if Phase 0 chose it).
  - `motionctl` commands per Module interfaces:
    - `compile` refuses when `validated.json` is missing, or when its `design_sha256` or `camera_tool_sha256` doesn't match the current files.
    - `rehearse` uploads as `tmp_<name>.txt`, runs each requested ovl after an operator confirmation prompt (`input()`), logs `jt_cur_pos`/`tl_cur_pos` at the state-update rate to `sw/robot/logs/rehearsal/<name>/`, prints tracking, timing and roll statistics against the contract table, writes a rehearsal summary, and deletes the temporary file afterwards.
    - `publish --ovl N` follows § Atomic publishing and requires a rehearsal at ≥ N.
    - `verify` regenerates and compares, as described.
  - Update `.gitignore` per § Which files are committed.
- **Tests:**
  - `sw/tests/test_robot_traj_file.py`: write → read round-trip with exact numeric precision; the header and line count match the sample count; the format matches the Phase 0 excerpt (golden string).
  - `sw/tests/test_robot_motionctl.py`, with `FakeRobotClient` and a `tmp_path` motions dir:
    - `validate` writes `validated.json` only on pass;
    - **stale and mismatched metadata** (each expects a refusal with a named reason): `design_sha256`, `camera_tool_sha256`, `validated_sha256`, tool id, TCP offset, sample period, duration, `num_points`, start joints (vs the first validated sample), controller filename, and `approved_ovl` above the rehearsed ovl;
    - `publish` writes `motion.json` with the correct md5, start_joints (= first sample's joints), duration and rehearsal summary;
    - **atomic publish with failure injection at each step:** a bad temporary upload leaves nothing changed; a bad final upload leaves the old metadata as `.prev` and no new `motion.json`; a failure writing the metadata leaves no half-written file; re-running `publish` after each failure succeeds;
    - `remove` deletes the controller file and `motion.json`;
    - `list` reports a controller file without a `motion.json`, a `tmp_*` orphan, and the reverse;
    - `verify` warns (not fails) on an md5 difference with an unchanged design hash;
    - exit code 1 with a message on `FairinoError`.
- **Done when:** the gate passes, and `plan`/`validate`/`compile`/`rehearse`/`publish` complete on the real robot for one template motion in each orientation mode (manual, operator present).

### Phase 5: `MotionRunner` (`motion_runner.py`); needs Phase 0 results
- **Work:** as specified in Threading, Stop and shutdown, and Controller-file verification.
  - `connect_and_check` runs once and checks, per motion: the remote MD5 per the verification table; the tool offset against `camera_tool.yaml` and `motion.json`; `MotionConfig.check_consistent()`. It also opens and tests the stop channel.
  - `load` calls the backend's load, then verifies `GetTrajectoryPointNum() == num_points`.
  - `move_to_start` runs `move_j(start_joints, tool, move_to_start_speed_pct)` inside `speed()`.
  - `at_start` compares `jt_cur_pos` with `start_joints` (max absolute difference ≤ `start_tolerance_deg`).
  - `run` re-loads first if Q4 says a `MoveJ` invalidates the loaded trajectory, then starts the motion at the approved ovl inside `speed()`, with the watchdog.
- **Tests** (`sw/tests/test_motion_runner.py`, with `FakeRobotClient`):
  - **verification outcomes**, each disabling the motion with its own reason: OK, file missing, SDK error on the hash call (retried twice, then disabled), empty hash, non-hex hash, mismatching hash (never retried), tool mismatch, and an inconsistent `motion.json`;
  - **no runtime method calls `ComputeFileMD5` or uploads anything** after `connect_and_check` (the check happens once only);
  - `load` never moves the robot (no MoveJ or MoveTrajectoryJ calls), and a point-count mismatch fails the load and marks the motion invalid for the session;
  - `move_to_start` sets speed 10 before `MoveJ`, with the motion's start joints, and restores the safe default afterwards;
  - `at_start` is true within tolerance, false just outside it, and reports the max error;
  - `run` resolves when the fake's motion completes, and raises `FairinoError` when an error code appears mid-motion; speed is restored after completion, error, stop and timeout;
  - **stop channel:** a second connection that can't be opened disables every motion with "no independent stop channel"; `stop()` while `run` is in progress calls `StopMotion` on the second connection immediately (not queued behind the worker);
  - **stop failures:** `StopMotion` returning errors is retried 3 times, then `PauseMotion` is tried, then the state is `UNKNOWN`; a stop that is confirmed (joint speeds reach 0) gives `FAULT`/`IDLE`; a stop that isn't confirmed within the timeout gives `UNKNOWN`;
  - **blocked worker:** a fake that blocks inside `MoveTrajectoryJ` forever: the watchdog fires at the deadline, `stop()` runs, the future fails with `MotionTimeout`, and the runner does not hang the test process (daemon thread);
  - **`motion_done` never set:** the run times out the same way;
  - **`close()`** during a pending run: stops the robot, rejects new jobs, joins within the timeout, and is idempotent;
  - `stop()` is idempotent and never raises;
  - the methods for a motion that failed the check raise `ValueError`.
- **Done when:** the gate passes.

### Phase 6: Livestream integration (`sw/livestream.py`, `sw/robot/ui_state.py`)
- **Work:**
  - `ui_state.py` as in § UI state machine.
  - `main()`: `--robot-ip` (overrides config) and `--no-robot`. Build a `MotionRunner` unless `--no-robot` is given, and pass it to `LivestreamApp(..., motion_runner: MotionRunner | None, motions: dict[str, MotionConfig])`.
  - **Startup:** `connect_and_check` runs via `_run_robot`. The results enable or disable each motion button per § Disabled-motion behaviour. If the saved `"motion"` is OK, `load()` it.
  - **Motion row** (`_build_motion_row`): one `tk.Button` per motion plus "No motion". The active one is highlighted (sunken relief and colour).
    - Click → `messagebox.askyesno("Change motion", f"Change the motion to '{name}'?")`.
    - Yes (and the state is still `IDLE`) → save `"motion"` and call `load(name)`; the status shows "Motion '{name}' loaded. Press Move to Start." **The robot does not move.**
    - No, or clicking the active motion → nothing happens.
  - **"Move to Start" button**, packed next to "Start recording" in the `controls` frame:
    - Click → `move_to_start(name)`; the status shows "Moving to start of '{name}' (10%)…".
    - Disabled for "No motion" and in every state except `IDLE`.
    - Highlighted (orange) when `at_start` is false, re-evaluated in `_update_controls` and after each robot operation.
  - **Record** and the rest of the flow follow the transition table. The first step of Record with a motion selected is `at_start(name)`; with "No motion", Record is unchanged.
  - **Stop**, `_on_close`, a GoPro start/stop error, or a robot error: per the table and races. `_on_close` also cancels pending `after` handles and calls `motion_runner.close()`. Read the existing `GoProController.shutdown` first and make sure a recording in progress is stopped on close.
  - `_update_controls` derives every button state from the `UiState`.
- **Tests:**
  - `sw/tests/test_robot_ui_state.py` (no Tk): **table-driven over every (state, event) pair**: the listed transitions return the expected new state and actions; every unlisted pair leaves the state unchanged and returns no actions.
  - Extend `sw/tests/test_livestream_ui.py`, following its existing `root`/`app` fixture and `FakeController` patterns, plus a `FakeMotionRunner` that records calls and returns pre-resolved Futures (monkeypatch `messagebox.askyesno`):
    - with `--no-robot` (runner None), no motion row and no Move to Start button are built, and the existing tests pass unchanged;
    - the buttons reflect the startup check: failed motions are visible but disabled, with the reason in the status bar on mouse-over; an invalid saved motion resets to "No motion" in memory while the config file keeps the saved value; an unreachable robot disables every motion;
    - confirming the dialog calls `load` and saves the config, and **does not call** `move_to_start` or `run`; declining changes nothing; clicking the active motion doesn't open the dialog; **a state change while the dialog is open discards the choice**;
    - Move to Start calls `move_to_start` with the selected name;
    - Record with `at_start` false doesn't call `controller.start_recording` and sets the status message;
    - Record with `at_start` true starts recording, and `run` is called only after the recording-started callback plus pre-roll (drive `root.after` with `root.update()` or a monkeypatched `after`);
    - motion completion triggers stop recording when `auto_stop_after_motion` is set;
    - **races:** close during pre-roll cancels the `after` handle so `run` is never called, and stops the robot and the recording; a `motion_done` arriving after Stop is ignored and the recording stops exactly once; a motion failure while recording stops the robot and the recording and keeps the clip; a failing `run()` after a successful GoPro start does the same; a failing GoPro start never calls `run`; a stale `op_id` result is ignored;
    - Stop during a motion calls `runner.stop()` before `stop_recording`;
    - `UNKNOWN` runner state disables the motion controls and shows the banner, and plain recording still works;
    - `_on_close` calls `stop()` and `close()`.
- **Docs:** add a "Robot motions" section to `README.md` covering the workflow (`new` → `plan` → `validate` → `compile` → `rehearse` → `publish`), camera calibration, operating safety rules (the app must not run while publishing; nobody in the cell during rehearsal), and the config keys.
- **Done when:** the gate passes, and the end-to-end check in Verification succeeds on hardware.

## Critical files
- **New:**
  - `sw/robot/__init__.py`, `frames.py`, `pose.py`, `path.py`, `schema.py`, `templates.py`, `validate.py`, `fr5.py`, `traj_file.py`, `backend.py`, `ui_state.py`, `motion_runner.py`, `motionctl.py`, `spike/*.py`
  - `sw/robot/camera_tool.yaml`
  - `sw/robot/motions/<name>/{design.yaml,motion.json}` (generated and local files are git-ignored)
  - `sw/tests/fakes/fake_robot.py`
  - `sw/tests/test_robot_{pose,frames,path,schema,validate,fr5,traj_file,motionctl,ui_state}.py`, `sw/tests/test_motion_runner.py`
- **Modified:** `sw/livestream.py`, `sw/tests/test_livestream_ui.py`, `pyproject.toml` (`scipy`, `pyyaml`, `hardware` pytest marker), `.gitignore` (generated and local robot files), `README.md`.
- **Already added:** `codegraph.json` (index scope for the SDK).
- **Reference (read, don't modify):**
  - `third_party/fairino_sdk/linux/fairino/Robot.py`
  - `third_party/fairino_sdk/linux/example/TestUploadTrajectoryJ.py` and `servo.py`
  - `sw/fairino_basic_query.py`
  - the existing `_run`/`_events`/`_poll` pattern and `save_config` in `sw/livestream.py`

## Verification
1. **Automated:** `uv run pytest sw/tests/ -q` and ruff are green after every phase (gate rule).
2. **Calibration:** run `calibrate_camera.py`; the horizon residual is ≤ 0.2°, the pointing residual ≤ 0.5° and the optical-centre error ≤ 3 mm, and `camera_tool.yaml` is approved.
3. **Offline:** `motionctl new demo --template sweep`, then `plan demo --preview` and `validate demo`. Review the peak joint speeds, `max_roll_deg` and the wrist/singularity margins in the report. Repeat with a target-mode design (fixed target, then a per-keyframe target trajectory) and review `max_pointing_error_deg`.
4. **Robot, nobody in the cell, e-stop at hand:** `compile demo`, then `rehearse demo --ovl 10,25,50,100`. At every level the measured roll stays within ±0.2°, tracking within the contract (3 mm / 0.3°) and timing within max(2%, 40 ms). Then `publish demo --ovl 100`, and `verify demo`.
5. **End to end in `livestream.py`:**
   - The startup check enables `demo`.
   - Select it and confirm: the motion loads and the robot does not move.
   - Press Record: it is refused with the "not at start" message.
   - Press Move to Start: the robot moves at 10%.
   - Press Record: the check passes, the GoPro records, the motion runs after the pre-roll, and recording auto-stops.
   - Check the clip for a level horizon (and, for the target design, that the subject stays centred).
   - Press Record again without Move to Start: it is refused.
   - Repeat with Stop pressed mid-motion: the robot halts immediately and the recording stops.
   - Close the app during a motion: the robot halts and the recording stops.
6. **Failure handling:**
   - Modify `motion.json`'s md5 and restart: `demo` is disabled with "md5 mismatch".
   - Delete the controller file and restart: "file missing on controller".
   - Unplug the robot's network during a motion: the watchdog stops it (or the app reports UNKNOWN) and the app still exits cleanly.
   - Interrupt a `publish` between steps 3 and 5: the app disables the motion, and re-running `publish` fixes it.
