# Plan: Camera-level robot motions (Fairino FR5) for livestream.py

## Read this first (rules for whoever implements this)
1. **Motions are played with `MoveTPD`.** We generate a TPD file (the same format the teach pendant records), upload it, load it with `LoadTPD`, and play it with `MoveTPD`. This route is verified end to end on the real robot (§ History). **Do not use `LoadTrajectoryJ`/`MoveTrajectoryJ`, Lua programs, or `ServoJ` streaming.** Questions about the TrajectoryJ file format are closed: they do not matter.
2. **Upload files only with `FR5.upload_file`** (§ Controller file transfer). Never call `TrajectoryJUpLoad`, `LuaUpload` or `PointTableUpLoad`: they fail with -1 because of a race in the SDK.
3. **Work the phases in order** (§ Phases). Each phase lists its tasks, its tests and when it is done. The first goal is Phase 4's milestone: **a motion generated from a `design.yaml` plays on the real robot.**
4. **Do not work on anything in § Deferred**, even if it looks related: camera calibration, roll measurement on the robot, stop latency, tracking/timing accuracy, singularity analysis, an independent stop channel, rehearsal ramps, atomic publishing. The user will ask for those later.
5. If a hardware fact you need is not in this document, **stop and ask the operator**. Do not start an investigation. Never move the robot without the operator's explicit YES (the scripts prompt for it).
6. **All automated/manual testing against hardware runs on the simulator, `192.168.50.2`, by default** (§ Simulator vs real robot). Use `--ip 192.168.50.2` (the `motionctl` default) for `pose`, `plan`, `validate` (IK/FK), and for exercising `move_j`/motion calls where the sim accepts them. **File upload and `MoveTPD` playback do not work on the sim** (confirmed; § History) — `upload`, `play` and `publish` need the real robot at `192.168.58.2`, run by the operator with YES confirmation. If an automated step needs a file on the controller and the upload call fails, **stop and tell the operator the local file path**; the operator uploads it manually through the pendant/web UI and confirms before you continue. Never silently retry a failing upload against the real robot without being asked.
7. Known-good reference code: `sw/fairino_basic_query.py` (load + play a TPD), `sw/robot/spike/file_port_probe.py` (`upload_file`), `sw/robot/spike/tpd_upload_check.py` (upload + MD5 + `LoadTPD` + start pose check). Copy their call sequences.

## Context
A GoPro is mounted on the FR5 flange. The goal is cinematic camera moves in the style of the "glambot": horizontal and vertical sweeps, crane moves and push-ins. Through all of them the camera **never rolls about its optical axis**, so the horizon stays level, while pan and tilt either follow keyframes or track a target point.

Motions are designed as keyframe files (`design.yaml`), sampled in Python, converted to joints with the controller's own inverse kinematics, written as a TPD file, uploaded to the controller and played there. The operator app `sw/livestream.py` selects a published motion with a button, moves the robot to the motion's start on request, and plays the motion when recording starts. **The app never generates or modifies a trajectory.** It only loads and plays files that were published beforehand.

Starting point:
- The Fairino Python SDK is installed from `third_party/fairino_sdk/linux` (see `pyproject.toml`, `[tool.uv.sources]`). That folder is git-ignored. `codegraph.json` re-includes `linux/fairino/` and `linux/example/` in the CodeGraph index.
- Robot: `192.168.58.2` over a USB network adapter. Simulator: `192.168.50.2` (§ Simulator vs real robot — the default target for development and tests; it does **not** store uploaded files or play `MoveTPD` trajectories).
- Controller software v3.9.1, SDK V2.2.1.
- Manuals: SDK manual `third_party/fairino_sdk/fairino-doc-en-readthedocs-io-en-3.9.1.pdf`; protocol manual `third_party/fairino_sdk/Collaborative Robot Controller Communication Command Protocol User Manual.pdf` (file transfer, controller error codes in Table 10-1). `pdftotext -layout` makes them searchable.

Terminology: a **motion** is one named trajectory, e.g. `sweep_left_fast`. Avoid the word "shot".

## Simulator vs real robot
**Default target for all development, scripts and manual test runs: the simulator, `192.168.50.2`.** Only the operator, with an explicit YES, runs anything against the real robot, `192.168.58.2`.

| Call family | On the simulator | On the real robot |
|---|---|---|
| `GetInverseKinRef`, `GetForwardKin`, `GetJointSoftLimitDeg`, `GetRobotInstallAngle`, `GetActualTCPNum` | Works; use for `pose`, `plan`, `validate` | Works |
| `MoveJ` and other motion commands | Accepted, but a second connection's `StopMotion` did not reliably interrupt a running `MoveJ` (§ History) — treat stop as unverified on the sim | Works; needs operator YES |
| `FR5.upload_file` (`FileUpload` + port 20010) | **Fails.** The sim does not open the data port the same way; every attempt returned -1 even with the retrying protocol (§ History) | Works (verified end to end) |
| `LoadTPD`, `GetTPDStartPose`, `MoveTPD` | **Does not play trajectories.** `LoadTPD` may return 0 but the controller does not store or run files the same way | Works (verified end to end) |
| `ServoJ` (joint-space servo streaming) | Expected to work like any other motion command (same row as `MoveJ`) — this is how a generated TPD file is previewed on the sim, since upload/`MoveTPD` are unavailable there (§ Simulator playback via ServoJ) | Not used; `MoveTPD` is the production path |

Consequences for implementation and testing:
- Every `motionctl` command defaults to `--ip 192.168.50.2`. Only `upload`, `play` and `publish` (and the Phase 4/5 robot acceptance steps) require `--ip 192.168.58.2`, and those three commands are run **only by the operator**, never unattended.
- When `upload` fails against the real robot (any `FileTransferError`), **do not retry automatically and do not debug the controller.** Print the local build file's path (`sw/robot/build/motion_<name>.txt`) and ask the operator to upload it manually (teach pendant or web UI trajectory import, which is known to work — § History), then continue once they confirm it's on the controller.
- Unit tests never touch either IP; they use `FakeRobotClient` (§ Phase 2). "Manual test on the sim" means a human or the implementer running a `motionctl` command by hand against `192.168.50.2`, not an automated test.

## Decisions (binding)
| # | Decision | Why |
|---|---|---|
| D1 | Backend = **TPD**: `LoadTPD` → `GetTPDStartPose` → move to start → `MoveTPD`. Config value `backend: "tpd"`. | Verified on the real robot. TrajectoryJ rejected every file and has undocumented format questions. |
| D2 | Upload with `FR5.upload_file(local, file_type=20)`. The file lands at `/fruser/traj/<basename>`. Verify with `ComputeFileMD5`. | The SDK upload calls race the controller. Type 20 accepts large files; type 0 (Lua) rejects files of ~1 MB. |
| D3 | Controller file `/fruser/traj/motion_<name>.txt`; TPD name **`traj/motion_<name>`** (no extension, no leading slash) for `LoadTPD`, `GetTPDStartPose`, `MoveTPD`. | `LoadTPD` looks in `/fruser/`; `traj/<stem>` is verified. An absolute path returns 8. |
| D4 | `sample_period_ms` = **8** for every motion (the default). | High accuracy is not needed, and 8 ms files are 4× smaller and need 4× fewer IK calls. The pendant's recording was 2 ms; 8 ms is confirmed by the Phase 4 robot test, with a fallback procedure in § TPD file format. |
| D5 | `tool_id` = **0** (flange) for every motion until a camera tool is configured. The camera frame is treated as the flange frame. | Camera calibration is deferred. Tool 0 needs no setup. |
| D6 | Joint values in the file come from the controller's IK (`GetInverseKinRef`, seeded with the previous sample). The pose columns are the planned pose. | File joints and poses must agree. We don't know which of the two `MoveTPD` uses, so both must be right. |
| D7 | Keep motions **≤ 20 s** for now. | The verified recording is 23 s (11520 points at 2 ms, 1.3 MB). |
| D8 | Safety minimum: operator YES prompt before any motion; move to start at 10% speed; `StopMotion` on Ctrl-C, Stop, close or error; controller collision detection stays on. **The physical e-stop is the primary safety device.** | — |
| D9 | The sim preview (`motionctl simplay`) streams a compiled TPD file's joints with `ServoJ`, one row per call, timed by the file's own period. **Never `ServoJT` (torque control).** | `ServoJ` is a direct joint-position stream, the closest thing to "replay this file" the sim can do, since `upload`/`MoveTPD` don't work there (§ Simulator vs real robot). `ServoJT` has no position reference; using it to track a trajectory needs a PD controller we have not built and do not need for visualization. |

## TPD file format (binding)
Verified by playing `sw/tracktestpc.txt` (a copy of a pendant recording) from Python. Generated files must match it exactly, field for field.

```
<N>,1,<period_ms>,<tool_id>,0,0
<j1>,<j2>,<j3>,<j4>,<j5>,<j6>,<x>,<y>,<z>,<rx>,<ry>,<rz>,<period_ms>,0,17
... N data rows in total ...
```
- Line 1 (header): `N` = number of data rows (must equal the row count); then `1`; then the sample period in ms; then the tool number; then `0,0`. With D4 and D5 this is **`N,1,8,0,0,0`**. (The pendant's 2 ms recording has `N,1,2,0,0,0`.)
- Each data row has 15 comma-separated values: 6 joint angles (deg), the TCP pose `x, y, z` (mm) and `rx, ry, rz` (deg, the SDK's Euler convention), then three values. Write **`<period_ms>,0,17`**, i.e. `8,0,17` at 8 ms. The 2 ms recording has `2,0,17` on every row, and a third-party generator wrote `5,0,17` together with a 5 ms period in its header, so the 13th value is taken to be the period.
- Numbers: fixed point with 4 decimals (`f"{v:.4f}"`), as in the recording. No spaces. Lines end with `\n`. No trailing blank line is required.
- Rows are uniformly spaced in time, one per `period_ms`. The first row is the start pose, the last row is the end pose (at rest).
- **8 ms is not yet verified on the robot.** The Phase 4 test measures the playback time. If an 8 ms file is rejected by `LoadTPD`, or plays at the wrong speed (for example 4× too fast, which would mean the controller assumes 2 ms per row), try in this order and record the result in § History: (a) keep the 8 ms header but write `2,0,17` in every row; (b) generate at 2 ms (`sample_period_ms: 2`), which is the verified format. Do not investigate further than that; tell the operator.
- Example, first two lines of the verified (2 ms) recording:
  ```
  11520,1,2,0,0,0
  91.2883,-121.5264,129.4355,-98.2465,-89.9802,0.0237,135.9624,-268.4651,361.4536,-179.6626,0.0199,-178.7353,2,0,17
  ```

## Simulator playback via ServoJ (D9)
Previewing a compiled TPD file end to end (upload, `LoadTPD`, `MoveTPD`) only works on the real robot (§ Simulator vs real robot). To see a generated motion move on the sim before ever touching the real robot, `motionctl simplay` reads the compiled file's joint columns and streams them with `ServoJ`, one row per call, which is a real SDK motion primitive (not a visualization hack) and works on the sim the same as `MoveJ` does.

- `ServoJ(joint_pos, axis_pos, cmdT=period_s)` in `fairino/Robot.py:1581`. `axis_pos` is the external-axis position, always `[0,0,0,0]` for us (no external axes). `cmdT` defaults to 0.008s; the manual's "recommended range [0.001~0.0016]" is for the controller-side command cycle and is not what paces playback — **the SDK's own example (`example/servo.py`) paces calls from the client with `time.sleep(cmdT)` between `ServoJ` calls**, which is what `simplay` does: `cmdT` is passed through unchanged so the controller's internal interpolation matches the inter-call spacing.
- Sequence: `ServoMoveStart()`; for each row, `ServoJ(row_joints, [0,0,0,0], cmdT=period_s)`, then sleep `period_s` (minus the call's own elapsed time, clamped to ≥ 0, so drift doesn't accumulate); `ServoMoveEnd()` in a `finally`, even on an error or Ctrl-C.
- **Never `ServoJT`** (joint torque control, D9). It has no position reference of its own; replaying a position trajectory with it needs a PD controller we have not built, which is a different and riskier project, not a visualization tool.
- This is a **preview only**. It does not exercise `LoadTPD`/`MoveTPD`, the controller's own TPD interpolation, or the upload path, so it cannot stand in for the Phase 4 real-robot acceptance test. Its purpose is to let the implementer and the operator see a generated motion move, and sanity-check the level horizon and timing, before spending real-robot time on it.
- `FR5.stream_joints(rows, period_s, ovl=100)` (new in `fr5.py`) wraps the sequence above. `motionctl simplay NAME [--ovl N] [--ip IP]` (default sim) reads `sw/robot/build/motion_<name>.txt` (or compiles it first if missing) and calls it; `--ovl` scales the inter-row sleep (not `cmdT`, which stays at the file's period) the same way `ovl` scales `MoveTPD`, so a sim preview at `--ovl 50` takes twice as long as the planned duration.

## Robot call reference
All are methods of `fairino.Robot.RPC`. Only `sw/robot/fr5.py` may call them (§ Package map).

| Job | Call | Notes |
|---|---|---|
| Connect / disconnect | `Robot.RPC(ip)`, `CloseRPC()` | `RPC()` prints the proxy; harmless. |
| IK (no motion) | `GetInverseKinRef(0, pose6, ref_joints6)` → `(err, joints6)` | Type 0 = absolute pose in the base frame. Uses the **active tool**. |
| FK (no motion) | `GetForwardKin(joints6)` → `(err, pose6)` | Uses the active tool. |
| Active tool | `GetActualTCPNum()` → `(err, tool_id)` or `robot_state_pkg.tool` | Must equal the design's `tool_id` during validate. |
| Soft limits | `GetJointSoftLimitDeg()` → `(err, [j1min, j1max, …, j6min, j6max])` | Real robot: J1 ±175, J2 −265…85, J3 ±160, J4 −265…85, J5 ±175, J6 ±175. |
| Install angle | `GetRobotInstallAngle()` → `(err, [yangle, zangle])` | Must be (0, 0). |
| Upload | `FR5.upload_file` (our protocol, § Controller file transfer) | Never the SDK upload calls. |
| Remote MD5 | `ComputeFileMD5("/fruser/traj/<file>")` → `(0, md5)`; `(4, None)` = not found | |
| Delete | `TrajectoryJDelete("<file>")` | A plain RPC; safe. |
| Load TPD | `LoadTPD("traj/motion_<name>")` → 0 | 8 = "tpd file does not exist". |
| Start pose | `GetTPDStartPose("traj/motion_<name>")` → `(0, pose6)` | Float32 precision: compare with tolerance 0.01. |
| Auto mode | `Mode(0)`, then **sleep 1 s**, then check `robot_state_pkg.robot_mode == 0` | `MoveTPD` does nothing in manual mode. The state cache is stale right after `Mode`. |
| Speed | `SetSpeed(pct)` | Global and persistent; there is no getter. |
| Move to start | `MoveJ(start_joints, tool_id, 0, vel=pct)` | Blocking by default (`blendT=-1`). |
| Play | `MoveTPD("traj/motion_<name>", 0, ovl)` | blend 0 = no smoothing; `ovl` = speed scale 0–100. |
| Sim preview (D9) | `ServoMoveStart()`; repeated `ServoJ(joints6, [0,0,0,0], cmdT=period_s)`; `ServoMoveEnd()` | Client paces calls with `time.sleep(period_s)`, as the SDK's own example does. Not used on the real robot. |
| Stop | `StopMotion()` | |
| State (cache, never blocks) | `robot_state_pkg`: `jt_cur_pos`, `tl_cur_pos`, `motion_done`, `mc_queue_len`, `robot_mode`, `main_code`, `sub_code`, `tool` | Refreshed by the SDK's state thread (port 20004). |

SDK behaviour to remember:
- Most calls return an `int` (0 = OK); getters return `(code, value)`. Calls made while the connection is down return -4.
- Most calls **retry socket errors forever** (`while flag: try … except socket.error`), so a call can block indefinitely if the network drops.
- Positive codes come from the controller (protocol manual Table 10-1: 3 = wrong parameter count, 4 = parameter value abnormal, 8 = tpd file does not exist, 14 = instruction failed). Negative codes come from the SDK (`RobotError`: -1 other, -4 RPC down, -7 local upload file not found).

## Controller file transfer (binding)
**Never call** `TrajectoryJUpLoad`, `LuaUpload`, `PointTableUpLoad` or any SDK method built on `__FileUpLoad`. They send the `FileUpload` request and then make **one** immediate connect to port 20010. The controller opens 20010 about 50 ms after the request, so that connect is refused and the call returns -1. Retrying the SDK call does not help.

**`FR5.upload_file(local_path, file_type=20) -> str`** does the protocol itself (protocol manual §5.1.50; reference: `upload_file` in `sw/robot/spike/file_port_probe.py`):
1. `file_type` must be **20** (stored at `/fruser/traj/<basename>`) or 0. Anything else raises `ValueError` before any call: the other types are firmware and upgrade packages.
2. XML-RPC `FileUpload(file_type, basename)` on the SDK's proxy (`rpc.robot.FileUpload`). Non-zero → `FileTransferError(stage="request", code=rtn)`.
3. Connect to `<ip>:20010`, retrying every 50 ms for up to 5 s. No connection → `FileTransferError(stage="connect")`.
4. Once connected, **always send the whole file.** Connecting and closing without sending leaves an **empty file** on the controller. Framing: `"/f/b" + f"{len(data) + 50:10d}" + md5_hex`, then the bytes, then `"/b/f"`.
5. Sleep 0.5 s, read the reply. It must start with `b"SUCCESS"`; otherwise `FileTransferError(stage="transfer")`.
6. `ComputeFileMD5("/fruser/traj/<basename>")` must return code 0 and the local MD5 (case-insensitive); otherwise `FileTransferError(stage="verify")`. Return the remote path.

Ports 20010/20011 are **closed while idle by design**. Never probe them as a health check.

## Architecture

### Package map and dependency rules
Package `sw/robot/`. Arrows mean "may import".
```
            pure (no SDK, no Tk; file I/O only)                hardware boundary
 ┌──────────────────────────────────────────────────┐   ┌──────────────────────┐
 │ pose.py ← path.py ← validate.py                  │   │ fr5.py               │
 │   ↑                    ↑                         │   │  RobotClient Protocol│
 │ schema.py   frames.py  tpd_file.py               │   │  FR5, FairinoError,  │
 │ ui_state.py                                      │   │  FileTransferError   │
 └───────────────▲──────────────────────────────────┘   └──────────▲───────────┘
                 │                                                 │
      motionctl.py (CLI: new/pose/plan/validate/compile/upload/play/publish/list)
      motion_runner.py (MotionRunner, used by livestream)
      sw/livestream.py → motion_runner.py, ui_state.py, schema.py (never fr5 directly)
```
Rules:
- Only `fr5.py` imports `fairino`, and it imports it lazily inside `FR5.connect`, so tests never need the SDK.
- `pose`, `path`, `frames`, `schema`, `validate`, `tpd_file`, `ui_state` never import `fr5` or `fairino`.
- `validate.py` gets IK/FK through the `Kinematics` Protocol. `FR5` satisfies it; tests use a fake.
- There is no backend abstraction. TPD is the only backend.

### Data flow
```
design.yaml ──plan──▶ samples (t, pose in base frame, pan, tilt)        [frames, path, pose]
            ──validate──▶ validated.json (joints from controller IK, checks) [validate + FR5 IK]
            ──compile──▶ sw/robot/build/motion_<name>.txt (TPD file)       [tpd_file]
            ──upload──▶ /fruser/traj/motion_<name>.txt, MD5 ok, LoadTPD ok   [FR5]
            ──play──▶ operator test on the robot (YES prompt)              [FR5]
            ──publish──▶ motion.json (md5, start joints, approved ovl)     [schema]
livestream start ─▶ MotionRunner.connect_and_check(all motion.json)  (MD5 of each file)
button+confirm ─▶ load(name) │ Move to Start ─▶ move_to_start(name) @10%
Record ─▶ at_start(name)? ─▶ GoPro start ─▶ pre_roll ─▶ run(name) ─▶ done ─▶ post_roll ─▶ GoPro stop
```

### Frames and transforms (implemented in Phase 1)
- **B, robot base frame.** z up. All planning, validation and the TPD file work in B. `validate` checks that the install angle is (0, 0).
- **T, camera tool frame** = the controller tool `tool_id` (D5: tool 0, the flange). **z = optical axis (forward), x = image right, y = image down.** A pose `[x, y, z, rx, ry, rz]` is the pose of T in B. Work-object (user) frames are not used (user 0).
- **A, authoring frame**, defined by the optional `anchor: {pos: [x, y, z], yaw_deg}`: z parallel to B's z, translated by `pos` and rotated by `yaw_deg`. Default identity. Lets a motion be authored relative to the subject's mark.
- `design_to_base` converts once: `p_B = Rz(yaw)·p_A + pos` for positions and targets; `pan_B = pan_A + yaw`. Everything after runs in B. The workspace box stays axis-aligned in A.

### Orientation model (implemented in Phase 1)
Orientation is **never stored or interpolated as a rotation.** It is built from two angles per sample:
- **pan** (deg): heading of the optical axis about +Z, 0 = looking along +X, positive counter-clockwise from above.
- **tilt** (deg): elevation of the optical axis, positive = up.
- `R(pan, tilt) = Rz(pan) · R_level0 · Rx(tilt)`, with `R_level0` columns `x_cam = (0,−1,0)`, `y_cam = (0,0,−1)`, `z_cam = (1,0,0)`. The image-right axis stays horizontal for every (pan, tilt), so roll is exactly zero by construction.

`orientation.mode`:
1. **`angles`:** every keyframe has `pan` and `tilt`, interpolated like any channel. Pan values are literal: 170 → −170 is a −340° turn.
2. **`target`:** orientation derived at every sample: `d = target − p_cam`, `pan = atan2(d_y, d_x)`, `tilt = atan2(d_z, hypot(d_x, d_y))`, pan unwrapped continuously. The target is either a design-level fixed `orientation.target`, or a `target` on every keyframe (a moving target, interpolated like the position).

Target-mode geometry limits (checked at every sample): `|d| ≥ min_target_distance_mm` (100), `hypot(d_x, d_y) ≥ min_horizontal_mm` (50), `|tilt| ≤ max_tilt_deg` (80, hard max 85), pan rate ≤ `max_pan_rate_deg_s` (120).

### Path and timing (implemented in Phase 1)
- Each channel (`x, y, z` plus `pan, tilt`, or plus the target's `x, y, z`) is a **piecewise quintic Hermite** in time, one polynomial per keyframe segment. Acceleration is 0 at every keyframe. First and last keyframes are at rest. Interior velocities are PCHIP-style; a keyframe may override them with `vel: {x: …, pan: …}` or set `stop: true`. Two or more keyframes. Segments ≥ `min_segment_s` (0.05 s).
- **Sampling rule:** `p = sample_period_ms`, `T` = last keyframe time, `N = ceil(round(T·1000/p, 6))`. Samples at `t_i = i·p/1000` for `i = 0..N` (**N + 1 samples**). A sample past `T` is clamped to the final keyframe. `duration_s = N·p/1000`. Example at 8 ms: T = 4.0 s → 501 samples.
- `to_sdk_pose(sample)` gives `[x, y, z, rx, ry, rz]` for the TPD file and for IK.

### Level constraint: what is checked now
The plan is level by construction, and the Phase 1 tests prove it (roll < 1e-9° at every sample). On the robot, the operator checks the horizon visually during the Phase 4 test. Measuring roll on the robot and calibrating the camera frame are deferred.

### Motion runner (Phase 5)
- **Worker.** `MotionRunner` owns one daemon `threading.Thread` that runs jobs from a `queue.Queue`; each public method (except `stop`, `at_start`, `close`) enqueues a job and returns a `concurrent.futures.Future`. Daemon on purpose: SDK calls can block forever and must not keep the app alive.
- **One connection, one lock.** All RPC calls go through one `FR5` and a `threading.Lock` held only for the duration of each RPC call. Polling reads `robot_state_pkg` (the cache) without the lock. `stop()` takes the lock briefly and calls `StopMotion()`. (If the Phase 4 test shows that `MoveTPD` blocks until the motion ends, `stop()` uses a second `FR5` connection instead. Record which one in § History.)
- **State:** `IDLE`, `BUSY`, `FAULT(reason)`, `CLOSED`.
- **Completion of `run`:** after `MoveTPD` returns 0, wait up to 2 s for `motion_done == 0` (the motion started; the cache is stale at first), then wait for `motion_done == 1`. Never started → `MotionError("motion did not start")`. A non-zero `main_code` while waiting → `FairinoError`.
- **Watchdog:** a `threading.Timer` with `deadline = 1.5 · duration_s · 100 / ovl + 3 s`. On expiry: `stop()` and fail the future with `MotionTimeout`.
- **Speed:** set before every motion (`move_to_start`: `move_to_start_speed_pct`, default 10) and reset to `safe_default_speed_pct` (default 10) in a `finally`. `MoveTPD`'s own `ovl` sets the playback speed.
- **Auto mode:** before any motion, `FR5.ensure_auto()` (`Mode(0)`, sleep 1 s, check `robot_mode == 0`, else `FairinoError`).
- **UI hand-off:** `LivestreamApp._run_robot(future, status, on_done, on_error)` mirrors the existing `_run`: the done-callback posts onto `self._events`, which `_poll` drains on the Tk thread. Tk widgets are never touched from the worker. Each robot operation carries an increasing `op_id`; stale callbacks are ignored.

### Stop and shutdown (Phase 5)
- `stop()` never raises and is idempotent: `StopMotion()` up to 3 attempts 100 ms apart; returns `True` if one returned 0. State → `IDLE`, or `FAULT` if the stop followed an error. No confirmation polling (deferred).
- `close()` is idempotent: `stop()`, state `CLOSED`, reject new jobs with `RunnerClosed`, join the worker for at most 2 s, `CloseRPC()` (best effort), reset speed (best effort).

### UI state machine (Phase 6)
`sw/robot/ui_state.py` holds a pure `UiState` enum and `transition(state, event, context) -> (new_state, actions)`. The Tk app feeds events in and executes the returned actions.

States: `UNAVAILABLE` (no robot: `--no-robot` or startup failure), `CHECKING`, `IDLE`, `LOADING`, `MOVING_TO_START`, `STARTING_RECORDING`, `PRE_ROLL`, `RUNNING`, `POST_ROLL`, `STOPPING`, `FAULT`.

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
| any non-`IDLE` state | `close` | (closing) | cancel `after` handles, `runner.stop()`, stop recording if recording, `runner.close()`, destroy |
| `STOPPING` | `recording_stopped` | `IDLE`, or `FAULT` if the runner is in `FAULT` | existing download flow |
| `FAULT` | `fault_cleared` (operator button) | `IDLE` | `runner.clear_fault()` |
| any | events not listed for that state | unchanged | ignored, logged |

Races: close during pre-roll cancels the `after` handle so `run()` never starts; `STOPPING` ignores late `motion_done`/`motion_failed`; a `_stop_requested` guard stops the recording exactly once; a failed GoPro start never starts the robot; after `askyesno` returns, re-check that the state is still `IDLE` before acting; stale `op_id` results are ignored.

Disabled motions: buttons stay visible; a motion that failed the startup check is disabled and shows its reason on mouse-over. An invalid saved `"motion"` resets to "No motion" in memory only. Robot unreachable → all motions disabled, plain recording works.

### Error handling contract
- `FR5._call(name, *args)` invokes the SDK method. A non-zero code raises `FairinoError(code, call)`. For `(code, value)` returns it returns `value`.
- `FileTransferError(stage, code)` for uploads (stages `request`, `connect`, `transfer`, `verify`).
- `MotionRunner` turns exceptions into Future exceptions. The UI shows `f"Robot error: {e}"`.
- `motionctl` exits with code 1 and a one-line message on any error, and prints the full report for validation failures.

## Data schemas
UTF-8. Units mm and degrees. Each schema has a frozen dataclass in `schema.py`. Loaders raise `SchemaError(path, field, reason)`.

### `sw/robot/motions/<name>/design.yaml` (hand-written, committed)
```yaml
name: sweep_left_fast          # must equal folder name; ^[a-z0-9_]{1,19}$
tool_id: 0                     # D5: flange
sample_period_ms: 8            # D4
anchor: {pos: [0, 0, 0], yaw_deg: 0}     # optional authoring frame A
seed_joints: [0, -90, 90, -90, -90, 0]   # IK seed for the first sample: use the joints
                                         # `motionctl pose` prints at the start position
orientation: {mode: angles}
limits:                        # all optional; defaults shown
  max_tcp_speed_mm_s: 800
  max_joint_speed_deg_s: 120   # scalar or list of 6
  max_joint_acc_deg_s2: 600
  max_joint_step_deg: 2.0      # per-sample jump => IK branch flip
  max_cumulative_pan_deg: 360
  allow_long_pan: false
  overshoot_pct: 2
workspace:                     # box in the authoring frame A
  min: [200, -700, 150]
  max: [900,  700, 1100]
keyframes:                     # t strictly increasing, first t == 0, at least 2
  - {t: 0.0, pos: [600, -400, 450], pan: 20,  tilt: -5}
  - {t: 1.2, pos: [600,  400, 450], pan: -20, tilt: -5, vel: {y: 400}}
  - {t: 2.0, pos: [650,  400, 700], pan: -20, tilt: 5}
```
Target mode: `orientation: {mode: target, target: [0, 0, 300]}` (plus the optional geometry limits) and keyframes with `pos` only, or a `target` on every keyframe. Schema rules (already enforced by `schema.py`): ≥ 2 keyframes; first and last at rest; angles mode needs `pan` and `tilt` on every keyframe and no `target`; target mode has exactly one target source and no `pan`/`tilt`; `|tilt| ≤ 85`; `sample_period_ms ∈ {2, 4, 8}`; segments ≥ `min_segment_s`; `vel` keys valid for the mode. Limits keys that still exist in `schema.py` but are not listed above (`max_wrist_step_deg`, `min_wrist_margin_deg`, `max_roll_deg`, `max_pointing_error_deg`) are accepted and ignored for now.

### `sw/robot/motions/<name>/validated.json` (written by `validate`; git-ignored)
- `name`, `design_sha256`, `created` (ISO 8601), `sample_period_ms`, `tool_id`, `install_angle`.
- `num_points` (N + 1), `duration_s`, `design_duration_s`.
- `samples`: `{t: [...], pose: [[6]...], pan: [...], tilt: [...], joints: [[6]...]}`.
- `peaks`: `{tcp_speed_mm_s, joint_speed_deg_s[6], joint_acc_deg_s2[6], max_joint_step_deg, max_fk_pos_err_mm, max_fk_rot_err_deg}`.
- `checks`: list of `{name, passed, detail}`. Written only when every check passes.

### `sw/robot/motions/<name>/motion.json` (written by `publish`; read by the app; committed)
```json
{
  "name": "sweep_left_fast",
  "backend": "tpd",
  "controller_file": "/fruser/traj/motion_sweep_left_fast.txt",
  "tpd_name": "traj/motion_sweep_left_fast",
  "md5": "…",
  "num_points": 251,
  "sample_period_ms": 8,
  "duration_s": 2.0,
  "design_duration_s": 2.0,
  "start_joints": [..6..],
  "start_pose": [..6..],
  "tool_id": 0,
  "approved_ovl": 50,
  "design_sha256": "…",
  "published_at": "2026-11-24T10:00:00"
}
```
`MotionConfig.check_consistent()`: `backend == "tpd"`; `controller_file == "/fruser/traj/motion_<name>.txt"`; `tpd_name == "traj/motion_<name>"`; `num_points ≥ 2`; `duration_s == (num_points − 1) · sample_period_ms / 1000`; `1 ≤ approved_ovl ≤ 100`; `start_joints` and `start_pose` have 6 values.

### Which files are committed
- Committed: `design.yaml`, `motion.json`.
- Git-ignored: `validated.json`, `sw/robot/build/`, `sw/robot/logs/`.

### `sw/livestream_config.json` (new keys)
- `"motion"`: selected motion name, or `""`.
- `"robot"`: `{"ip": "192.168.58.2", "backend": "tpd", "start_tolerance_deg": 0.5, "move_to_start_speed_pct": 10, "safe_default_speed_pct": 10, "pre_roll_s": 0.5, "post_roll_s": 1.0, "auto_stop_after_motion": true}`.

## Module interfaces
Signatures are binding. Private helpers are free.

- **`frames.py`, `pose.py`, `path.py`**: done in Phase 1 (`design_to_base`, `base_to_authoring`, `within_workspace`; `level_rotation`, `look_at_angles`, `roll_error_deg`, `pointing_error_deg`, `rotation_to_sdk_euler`, `sdk_euler_to_rotation`, `unwrap_deg`, `SDK_EULER = "xyz"`; `Sample`, `sample_count`, `sample_times`, `build_trajectory`, `sample_path`, `to_sdk_pose`).
- **`schema.py`**: `Design`, `Keyframe`, `MotionConfig`, `RobotSettings`, `load_design`, `load_motion_config`, `load_motion_configs`, `sha256_file`, `motion_config_to_dict`. (`CameraTool` stays but is unused until calibration.)
- **`tpd_file.py`** (Phase 4):
  - `write(path, joints: list[list[float]], poses: list[list[float]], period_ms: int, tool_id: int) -> None` (format above; raises `ValueError` on length mismatch or rows ≠ 6 values).
  - `read(path) -> TpdFile` with `header: tuple`, `joints`, `poses` (for tests and checks).
  - `md5(path) -> str`.
- **`validate.py`** (Phase 3):
  - `class Kinematics(Protocol)`: `ik(pose6, ref6) -> list[float] | None`, `fk(joints6) -> list[float]`, `active_tool() -> int`, `install_angle() -> tuple[float, float]`, `joint_soft_limits() -> list[tuple[float, float]]`.
  - `validate(design, samples, kin) -> ValidationReport` (`passed`, `checks`, `peaks`, `joints`).
  - Checks, each its own function: `check_active_tool`, `check_install_angle`, `check_target_geometry`, `check_pan_turn`, `check_path_overshoot`, `check_workspace`, `check_tcp_speed`, `check_ik`, `check_joint_steps`, `check_joint_limits`, `check_joint_speed_acc`, `check_fk_roundtrip` (every 10th sample: < 0.1 mm, < 0.05°).
  - `report_to_validated(report, design, samples) -> dict` (the `validated.json` content).
- **`fr5.py`** (Phase 2):
  - `class FairinoError(RuntimeError)` (`code`, `call`); `class FileTransferError(RuntimeError)` (`stage`, `code`).
  - `class RobotClient(Protocol)`: the SDK methods in § Robot call reference plus `robot` (the XML-RPC proxy) and `robot_state_pkg`.
  - `class FR5(client, ip)`:
    - `connect(ip) -> FR5` (classmethod; lazy `from fairino import Robot`), `close()`
    - `ik`, `fk`, `active_tool`, `install_angle`, `joint_soft_limits` (satisfies `Kinematics`)
    - `joints()`, `tcp_pose()`, `motion_done() -> bool`, `error_codes() -> (main, sub)`, `robot_mode() -> int` (all from the cache)
    - `ensure_auto()`, `set_speed(pct)`, `speed(pct)` (context manager restoring `safe_default_pct` in `finally`)
    - `move_j(joints, tool_id, vel_pct)` (blocking)
    - `upload_file(local_path, file_type=20) -> str`, `file_md5(remote_path) -> str | None` (None = not found), `delete_traj(filename)`
    - `load_tpd(name)`, `tpd_start_pose(name) -> list[float]`, `move_tpd(name, blend, ovl)`, `stop()`
    - `stream_joints(rows: list[list[float]], period_s: float, ovl: int = 100)` (D9, sim preview): `ServoMoveStart()`; for each row, `ServoJ(row, [0,0,0,0], cmdT=period_s)` then sleep `period_s * 100/ovl` (clamped to ≥ 0 after subtracting the call's own elapsed time); `ServoMoveEnd()` in `finally`. Never `ServoJT`.
- **`motionctl.py`** (Phase 4, `publish`/`list` in Phase 5): `uv run python -m sw.robot.motionctl <cmd> …`:
  - `new NAME --template sweep|crane|orbit|push_in|first_test`
  - All commands take `--ip IP`, default `192.168.50.2` (the simulator; § Simulator vs real robot). `upload`, `play` and `publish` need `--ip 192.168.58.2` and operator confirmation; they refuse to run against the default sim IP since they need the real file/motion services.
  - `pose [--ip IP]` (default sim): print current joints, tool, TCP pose, and the pan/tilt/roll of that pose (no motion). Used to author keyframes and seed joints.
  - `plan NAME`: sample count, duration, peak TCP speed (no robot).
  - `validate NAME [--ip IP]` (default sim): IK on the controller (no motion), writes `validated.json`.
  - `compile NAME`: writes `sw/robot/build/motion_<name>.txt`; refuses if `validated.json` is missing or its `design_sha256` is stale.
  - `simplay NAME [--ovl N] [--ip IP]` (default sim; § Simulator playback via ServoJ, D9): reads the compiled file (compiles first if missing), moves to the start pose with `move_j` at `move_to_start_speed_pct`, then `stream_joints`. Preview only; never the production path. Not restricted to the sim IP (harmless on the real robot too, but there is no reason to run it there instead of `play`).
  - `upload NAME --ip 192.168.58.2`: `upload_file`, `LoadTPD`, `GetTPDStartPose` equals the first row's pose (tolerance 0.01). No motion. **Real robot only; refuses the sim IP.** On `FileTransferError`, print the local build path and stop — do not retry; ask the operator to upload manually.
  - `play NAME --ovl N --ip 192.168.58.2`: the robot test (see Phase 4). **Real robot only; refuses the sim IP** (it does not play TPD trajectories). Requires operator YES.
  - `publish NAME --ovl N --ip 192.168.58.2`: **real robot only**, operator-run. `list` (no `--ip`, reads only local `motion.json` files).
- **`motion_runner.py`** (Phase 5): `MotionRunner(connect: Callable[[], FR5], settings: RobotSettings, motions: dict[str, MotionConfig])` with `connect_and_check() -> Future[dict[str, str | None]]`, `load(name)`, `move_to_start(name)`, `at_start(name) -> tuple[bool, float]`, `run(name)`, `stop() -> bool`, `clear_fault()`, `close()`, `state`.
- **`ui_state.py`** (Phase 6): `UiState`, `Event`, `Action`, `transition(state, event, context)`.

## Phases
**Gate rule for every phase:** `uv run pytest sw/tests/ -q` passes with no new failures or skips, and the files you changed pass `uv run ruff check <files>` and `uv run ruff format --check <files>` (the repo has old ruff errors elsewhere; do not fix those). The pre-commit hook runs the same checks on staged files. Each phase ends in its own commit on `feature/fairino-python-basic-control`, made only when the user asks. **No unit test may need the robot, the GoPro or the network.** Hardware steps are run by the operator from `motionctl`, never from `pytest`.

### Phase 0: hardware spike — closed for the core features
Answered and binding: the TPD route (D1–D3), the file transfer protocol, the TPD file format, the IK/FK, soft limits and install angle calls, the auto-mode rule, the Euler convention constant `SDK_EULER = "xyz"` (simulator-verified; the Phase 4 robot test checks it visually). Everything else that Phase 0 originally listed is in § Deferred.

### Phase 1: pure geometry and schemas — done (commit `36d0914`)

### Phase 1b: align the Phase 1 code with the TPD decision
Small, mechanical. Tasks:
- `schema.py`: `BACKENDS = ("tpd",)`; `RobotSettings.backend` default `"tpd"`; remove `stop_confirm_timeout_s` from `RobotSettings`.
- `MotionConfig` / `MOTION_FILE_KEYS`: add `tpd_name`; remove `tcp_offset`, `camera_tool_sha256`, `validated_sha256`, `rehearsal`; `check_consistent()` as in § `motion.json`.
- `templates.py`: every template uses `sample_period_ms: 8` (already the case) and `tool_id: 0`. Add `first_test`: angles mode, pan 0, tilt 0, two keyframes 4 s apart, the second 100 mm above the first (`[500, 0, 400]` → `[500, 0, 500]` as placeholders; the operator replaces them using `motionctl pose`).
- `.gitignore`: add `sw/robot/motions/*/validated.json`, `sw/robot/build/`, `sw/robot/logs/`.
- Tests: update `sw/tests/test_robot_schema.py` for the new `MotionConfig` and backend; add a test that every template loads, samples, and has period 8 and tool 0.
- Done when: the gate passes.

### Phase 2: robot boundary (`fr5.py`) with fakes
- Work: `FR5` as in § Module interfaces. Port `upload_file` from `sw/robot/spike/file_port_probe.py`. `connect` imports `fairino` lazily.
- Fake: `sw/tests/fakes/fake_robot.py` with `FakeRobotClient`: records calls; `GetInverseKinRef` returns a deterministic function of the pose (or `(err, None)` on demand); `GetForwardKin` inverts it; a fake `robot_state_pkg` (`jt_cur_pos`, `tl_cur_pos`, `motion_done`, `mc_queue_len`, `robot_mode`, `main_code`, `sub_code`, `tool`); in-memory files for `ComputeFileMD5`; injectable error codes per method; **`TrajectoryJUpLoad`, `LuaUpload`, `PointTableUpLoad` raise `AssertionError` if called.**
- Tests (`sw/tests/test_robot_fr5.py`):
  - each wrapper calls the right SDK method with the right arguments; non-zero codes raise `FairinoError` with the call name; `(code, value)` unpacking;
  - `speed(pct)` restores the safe default on normal exit and on an exception, and tolerates a failing restore;
  - `ensure_auto` calls `Mode(0)`, waits (monkeypatch `time.sleep`), and raises if `robot_mode` stays 1;
  - `upload_file` with a local TCP listener on an ephemeral port standing in for 20010 (make the port a constructor argument): listener starts 100 ms after `FileUpload` → success; never starts → `stage="connect"`; `FileUpload` non-zero → `stage="request"` and no connect; reply not `SUCCESS` → `stage="transfer"`; MD5 differs → `stage="verify"`; the listener receives exactly the framing; type 5 → `ValueError` before any call;
  - `file_md5` returns `None` for code 4.
- Done when: the gate passes.

### Phase 3: validator (`validate.py`)
- Work: as in § Module interfaces. IK runs sample by sample: the first with `seed_joints` as reference, each next one with the previous solution. At 8 ms a 4 s motion is 501 IK calls (about a second or two over XML-RPC); print progress every 500 samples.
- Defaults: joint limit margin 2° inside the soft limits; FK round trip on every 10th sample.
- Tests (`sw/tests/test_robot_validate.py`, with a `FakeKinematics`): a good angles design and a good target design pass; IK `None` at one sample fails `check_ik` naming the index and time; a 30° jump on one joint fails `check_joint_steps`; a joint beyond its soft limit fails; a too-fast design fails speed/acc with peaks reported; a point outside the box fails `check_workspace`; an overshooting `vel` override fails `check_path_overshoot`; pan turn and target geometry failures; wrong active tool and non-zero install angle fail; an FK offset fails `check_fk_roundtrip`; the report serialises to the `validated.json` schema.
- Done when: the gate passes.

### Phase 4: TPD writer, `motionctl`, first robot test ← **milestone**
- Work: `tpd_file.py`; `motionctl` commands `new`, `pose`, `plan`, `validate`, `compile`, `simplay`, `upload`, `play`; `FR5.stream_joints` (D9).
- Use `simplay` on the simulator to see a generated motion move and sanity-check timing and the level horizon **before** spending real-robot time (recommended, not required): `motionctl new`, `validate`, `compile`, then `simplay NAME --ovl 25`.
- `play NAME --ovl N` sequence (copy `sw/fairino_basic_query.py`):
  1. Check the local build file's MD5 equals the controller's (`file_md5`); if not, tell the operator to run `upload`.
  2. `load_tpd(tpd_name)`; `tpd_start_pose` equals the first row's pose (0.01).
  3. Print the current joints and the start joints. Ask `Type YES to continue:`. Anything else aborts.
  4. `ensure_auto()`; `speed(10)`: `move_j(start_joints, tool_id, 10)`.
  5. `Press Enter to play:`. Then `load_tpd` again (cheap insurance), `move_tpd(tpd_name, 0, N)`. Print how long `move_tpd` took to return.
  6. Wait for completion as in § Motion runner, with the watchdog; print the elapsed time vs `duration_s · 100 / N`.
  7. Ctrl-C at any point → `stop()`. Always reset speed and `close()` in `finally`.
- Tests: `sw/tests/test_robot_tpd_file.py`: write → read round trip; header `N,1,8,0,0,0` and rows ending `8,0,17` for period 8 and tool 0; header `N,1,2,0,0,0` and `2,0,17` for period 2; 4 decimals; **golden test: writing the verified recording's first data row (§ TPD file format example; embed the line in the test, do not read `sw/tracktestpc.txt`) with N = 1 reproduces `1,1,2,0,0,0` and that row byte for byte**; length mismatch raises. `sw/tests/test_robot_motionctl.py` with `FakeRobotClient` and `tmp_path`: `new` writes a loadable design; `validate` writes `validated.json` only on pass; `compile` refuses a stale `design_sha256`; `upload` calls `upload_file` then `LoadTPD("traj/motion_<name>")`; `play` aborts without YES and never calls `MoveJ`/`MoveTPD` then; `play` with YES calls `Mode(0)`, `MoveJ(start_joints, …)`, `MoveTPD("traj/motion_<name>", 0, N)` in that order; Ctrl-C (raise `KeyboardInterrupt` from the fake) calls `StopMotion`; `simplay` calls `move_j` to the start pose then `ServoMoveStart`, one `ServoJ` per compiled row (joints matching the file, `axis_pos == [0,0,0,0]`), `ServoMoveEnd`, and never calls `upload_file`, `LoadTPD` or `MoveTPD`; a `ServoJ` error mid-stream still calls `ServoMoveEnd` (via `finally`); `simplay` compiles the file itself when the build is missing. `sw/tests/test_robot_fr5.py` adds: `stream_joints` calls `ServoMoveStart` once, one `ServoJ` per row with `axisPos=[0,0,0,0]` and the row's own `cmdT`, then `ServoMoveEnd`; `ServoMoveEnd` still runs when a row's `ServoJ` raises `FairinoError`; the real-time pacing is not asserted (it uses `time.sleep`, monkeypatched in the test to avoid a slow suite).
- **Operator acceptance on the real robot** (cell clear, e-stop in hand):
  1. Jog the robot with the pendant to a pose where the flange points forward and level. `motionctl pose` → copy the position, pan, tilt into `first_test`'s first keyframe and the joints into `seed_joints`; second keyframe = 100 mm higher.
  2. `motionctl validate first_test`, `compile first_test`, `upload first_test`.
  3. `motionctl play first_test --ovl 25`: the robot moves to the start, then rises 100 mm in about 16 s (4 s at 25%) without turning the flange. **Time it.** About 4 s instead of 16 s means the controller ignored the 8 ms period: apply the fallback in § TPD file format. (This first test is small and slow on purpose, so a 4× speed error stays safe.)
  4. Repeat with `sweep` adapted to the cell, at `--ovl 25` then `--ovl 100`.
  5. Record in § History: did it play at 8 ms (and which fallback, if any); `move_tpd` return time (decides the stop design in Phase 5); measured vs expected duration; whether the camera stayed visually level.
- Done when: the gate passes and step 3 succeeds on the robot.

### Phase 5: `publish` and `MotionRunner`
- Work: `motionctl publish NAME --ovl N`: requires the build file and `validated.json` to be current; uploads; verifies MD5; `LoadTPD` + start pose check; writes `motion.json` via a temporary file and `os.replace`. `motionctl list`: published motions and whether the controller file's MD5 matches. `MotionRunner` as in § Motion runner and § Stop and shutdown. `connect_and_check` per motion: `check_consistent()`, controller MD5 equals `motion.json.md5` (reasons: `file missing on controller`, `md5 mismatch`, `hash query failed (code N)`).
- Tests (`sw/tests/test_motion_runner.py`, `FakeRobotClient`): each startup reason disables only that motion; `load` never moves the robot and checks the start pose; `move_to_start` sets speed 10, calls `MoveJ` with the start joints, restores speed; `at_start` true inside the tolerance, false just outside, returns the max error; `run` resolves when the fake's `motion_done` goes 0 → 1, raises when it never goes 0, raises on a non-zero `main_code`; the watchdog fails a run whose `motion_done` never returns to 1 and calls `StopMotion`; `stop()` never raises and is idempotent; `close()` during a run stops and rejects new jobs; a blocked fake RPC does not hang the test process.
- Done when: the gate passes and `motionctl publish first_test --ovl 25` succeeds on the robot.

### Phase 6: livestream integration
- Work: `ui_state.py` (§ UI state machine). `main()`: `--robot-ip`, `--no-robot`. Build a `MotionRunner` unless `--no-robot`; pass it and the loaded `motion.json` configs to `LivestreamApp`.
  - Startup: `connect_and_check` via `_run_robot`; enable/disable buttons; if the saved motion is OK, `load()` it.
  - Motion row: one button per motion plus "No motion"; the active one highlighted. Click → `askyesno("Change motion", …)`; Yes (state still `IDLE`) → save `"motion"` and `load(name)`. The robot does not move.
  - "Move to Start" button next to "Start recording": `move_to_start(name)`; enabled only in `IDLE` with a motion selected; orange when `at_start` is false.
  - Record with a motion: `at_start` first; then the transition table. Stop, close and errors per the table. `_on_close` cancels `after` handles and calls `runner.close()`; make sure a recording in progress is stopped (read `GoProController.shutdown` first).
- Tests: `sw/tests/test_robot_ui_state.py`, table-driven over every (state, event) pair. Extend `sw/tests/test_livestream_ui.py` with a `FakeMotionRunner` returning pre-resolved futures (monkeypatch `messagebox.askyesno`): `--no-robot` builds no motion row and old tests pass; confirm → `load` + config saved, never `move_to_start`/`run`; decline → nothing; Move to Start calls `move_to_start`; Record not at start → no recording; Record at start → `run` only after recording started + pre-roll; motion done → recording stops when `auto_stop_after_motion`; close during pre-roll → `run` never called; Stop during a motion → `runner.stop()` before `stop_recording`; stale `op_id` ignored.
- Docs: "Robot motions" section in `README.md`: workflow `new → pose → validate → compile → upload → play → publish`, safety rules (nobody in the cell; e-stop in hand; the app must not run while publishing), config keys.
- Done when: the gate passes and § Verification step 3 succeeds on the robot.

## Deferred (do not start; the user will ask)
| Item | Notes for later |
|---|---|
| Camera tool calibration (`camera_tool.yaml`, tool ≠ 0, payload) | `CameraTool` exists in `schema.py`. Old procedure: `git show efdd2df:docs/design/feature/fairino-python-basic-control.md`, § Camera tool calibration. |
| Measuring roll, tracking and timing on the robot | Would log `tl_cur_pos` during `play`. |
| Stop latency, independent stop channel, stop confirmation | A second `RPC` connection's `StopMotion` did not interrupt a `MoveJ` on the simulator. |
| Singularity thresholds, manipulability, IK configuration signature, wrist margin | `check_joint_steps` and the speed limits catch the practical failures for now. |
| Re-confirming the Euler convention numerically on the real robot | Visual check in Phase 4 is enough for now. |
| Other periods (2, 4 ms) except as the 8 ms fallback; motions > 20 s | — |
| Rehearsal ramps, approval evidence, atomic publish with temporary files, `verify` | — |
| `MoveTPD` blend = 1, `ovl` time-scaling accuracy | — |
| Anything with `LoadTrajectoryJ`/`MoveTrajectoryJ`, Lua programs, `ServoJ` | **Not planned at all.** |

## Critical files
- New: `sw/robot/tpd_file.py`, `validate.py`, `fr5.py`, `ui_state.py`, `motion_runner.py`, `motionctl.py`; `sw/robot/motions/<name>/{design.yaml,motion.json}`; `sw/tests/fakes/fake_robot.py`; `sw/tests/test_robot_{fr5,validate,tpd_file,motionctl,ui_state}.py`, `sw/tests/test_motion_runner.py`.
- Modified: `sw/robot/schema.py`, `sw/robot/templates.py`, `sw/tests/test_robot_schema.py`, `sw/livestream.py`, `sw/tests/test_livestream_ui.py`, `.gitignore`, `README.md`.
- Reference (read, don't modify): `sw/fairino_basic_query.py`, `sw/robot/spike/file_port_probe.py`, `sw/robot/spike/tpd_upload_check.py`, `third_party/fairino_sdk/linux/fairino/Robot.py`. `sw/tracktestpc.txt` is a local, untracked copy of a pendant recording (1.3 MB); tests must not depend on it.

## Verification
1. Automated: the gate after every phase.
2. Robot, Phase 4: `first_test` (8 ms) plays at `--ovl 25` in about 16 s; the flange rises without turning; the camera stays visually level.
3. End to end in `livestream.py` (Phase 6): the startup check enables `first_test`; selecting it loads it without moving; Record before Move to Start is refused; Move to Start moves at 10%; Record starts the GoPro, the motion runs after the pre-roll, recording auto-stops; Stop mid-motion halts the robot and stops recording; closing the app during a motion halts the robot.
4. Failure handling: edit `motion.json`'s md5 → the motion is disabled with "md5 mismatch"; delete the controller file → "file missing on controller".

## Open problem: smoothing a detour keyframe's velocity kink (handed off, 2026-11-24)

**Status: blocked. Not a bug in validate.py or path.py as such; the design below needs a smoother middle keyframe, and the current mechanism (PCHIP default velocities) cannot produce one without either a manual `vel` override (untried) or a different detour point. Handed to a more capable model to solve; this section is written to be self-contained.**

### The motion in question
`sw/robot/motions/camtest/design.yaml` (current content, reproduced in full since it will keep changing):
```yaml
name: camtest
tool_id: 1
sample_period_ms: 8
seed_joints: [89.718, -90.297, 0.077, 29.659, -88.781, 45.769]
orientation: {mode: angles}
workspace: {min: [-50, -900, -650], max: [250, -300, 800]}
keyframes:
  - {t: 0.0,  pos: [95.94, -403.331, 673.525],  pan: -91.165, tilt: -29.453}
  - {t: 9.0,  pos: [99.0,  -580.0,   450.0],    pan: -90.8,   tilt: -20.0}
  - {t: 18.0, pos: [102,   -758.094, -508.421], pan: -90.0,   tilt: 20.0, stop: true}
```
This is a real cinematic move the user specified from jogged poses on the real robot's tool 1 (the camera tool, defined below). Tool 1's `t_coord` is `[-282, -282, -40, 90, 0, -45]` (mm, deg; flange→camera, SDK xyz Euler), applied on both the simulator and (the user's intent) the real robot. The user confirmed roll is ~0-0.8° at both the start and end keyframes when read back from the controller, i.e. tool 1 is correctly oriented (z = optical axis, x = image-right, y = image-down, per `pose.py`).

### Why the middle keyframe exists
A direct two-keyframe path from the start to the end position+orientation fails `check_ik`: at samples around t≈0.4-0.45 of the (then 2s) design, `GetInverseKinRef` returns error 112 (no solution near the seeded reference) with **J3 (elbow) pinned at 151-156°**, against a soft limit of **160°** (`GetJointSoftLimitDeg` on the real robot/sim: `[-175,175, -265,85, -160,160, -265,85, -175,175, -175,175]`, i.e. J3 ∈ [-160, 160]). This was confirmed by a fine-grained sweep (41 points) of the straight-line interpolated path seeded continuously from the previous solution: IK succeeds up to about t=0.40 (J3≈152°), fails for t∈[0.45,0.55] (no solution from either direction's seed), and succeeds again from about t=0.60 onward but having jumped to a visibly different configuration branch (J2 from ≈-140° to ≈-20°, J4 from ≈-92° to ≈-226°). **The straight path asks the elbow to bend past its physical limit; there is no continuous IK solution for it.**

A middle keyframe at `[99.0, -580.0, 450.0]`, pan -90.8°, tilt -20.0° was found by scanning z and tilt at the gap's x,y and picking a point with comfortable J3 margin (J3≈97° there, found via `GetInverseKinRef` with seed `[89.718,-90.297,0.077,29.659,-88.781,45.769]`). Both resulting legs (start→mid, mid→end) were verified to be fully IK-reachable with a continuously seeded sweep (21 points each). With this keyframe in `design.yaml`, **`check_ik` now passes end to end** (confirmed via `motionctl validate camtest --ip 192.168.50.2`, real simulator, 2251 samples at 18s). The kinematic/reachability problem is solved.

### The remaining problem: joint 2 acceleration
With the IK-reachable 3-keyframe path, `check_joint_speed_acc` still fails on **joint 2 (shoulder)**, and stretching the total duration does not converge:

| Total duration | J2 peak speed (deg/s) | J2 peak accel (deg/s²) | Limit |
|---|---|---|---|
| 2s (2 keyframes, pre-detour) | n/a (IK failed) | n/a | speed 120, accel 600 |
| 2s (3 keyframes) | not reached (other checks failed first) | not reached | |
| 4s | (not isolated) | | |
| 5s | 280.8 (fails) | not reached (speed failed first) | |
| 12s | passes (not re-measured exactly) | 1321.8 (fails) | accel 600 |
| 18s | passes | 1085.9 (fails) | accel 600 |

Going from 12s to 18s (a 1.5x stretch, which should cut acceleration by 1.5²≈2.25x under pure time-scaling) only cut peak acceleration from 1321.8 to 1085.9 (a 1.22x reduction) — **much less than the 1/T² relationship that holds for a smooth path**. This is the diagnostic signal: the acceleration peak is **not** simply "the path is too fast for its duration", it is a **velocity-direction discontinuity (a kink) at the middle keyframe**, which pure time-scaling cannot fully remove, because a kink's acceleration spike (ideally a Dirac-like impulse in the idealized zero-width limit) shrinks much more slowly than 1/T² as duration increases.

### Why there is a kink: the mechanism in path.py
From `sw/robot/path.py` (`Channel`, `keyframe_velocities`, `Trajectory.__init__`): each channel (x, y, z, pan, tilt here, since this design uses `orientation.mode: angles`) is built as a **piecewise quintic Hermite** spline, one polynomial per segment, continuous in position and velocity at each keyframe, with **acceleration forced to zero at every keyframe** (`_quintic_coeffs` enforces `p''(0)=0, p''(1)=0` in each segment's local parameter). Velocities at interior keyframes come from `keyframe_velocities`: for more than 2 keyframes, `scipy.interpolate.PchipInterpolator(times, values)(times, nu=1)` -- PCHIP's own derivative estimate, which is **shape-preserving**: if the channel is not monotonic through the middle keyframe (i.e. the channel reverses direction, flattens, or changes curvature sign there), PCHIP assigns **velocity ≈ 0** at that keyframe for that channel. The two straight-ish legs before and after the kept-at-rest middle point then each have to decelerate to ~0 and reaccelerate, producing a path that is numerically C1 (continuous velocity, by construction) but has a very sharp *curvature* change -- a velocity vector that swings through a large angle in a short arc length, which is what drives joint 2's acceleration up disproportionately as the two legs' direction change becomes sharper relative to their (otherwise generous) timing budget.

**This has not been fully diagnosed per-channel.** It has not been checked which of x, y, z, pan, tilt is/are non-monotonic through the middle keyframe and therefore pinned to near-zero PCHIP velocity there (position channels are the most likely culprit, given the physical detour; check `path.Trajectory` directly, or reconstruct with `path.sample_path(design)` and inspect consecutive sample derivatives numerically). It has also not been checked whether joint 2's acceleration spike is a direct image of a Cartesian-space kink, or an IK-amplification of a smaller Cartesian kink (IK Jacobians can amplify velocity/acceleration non-linearly near where J3 is still close to its limit, which it still might be at the middle keyframe even with the chosen margin).

### What has NOT been tried (candidate fixes, unevaluated)
1. **A keyframe-level `vel` override at the middle keyframe** (`Keyframe.vel`, a dict `{channel: value}`, already supported by the schema -- see `schema.py`'s `_check_keyframes` and `ANGLE_VEL_CHANNELS = ("x","y","z","pan","tilt")`). Setting an explicit, non-zero "through velocity" for the channels that are currently pinned to ~0 by PCHIP (likely one or more of x, y, z) should let the quintic segments blend through the kink with continuous curvature-compatible velocity instead of nearly stopping and restarting. **The correct through-velocity has not been computed.** A principled choice would be a one-sided or central finite-difference estimate of the chord slope using the neighbouring keyframes and an assumed/target speed, or a value tangent to a smoothed (e.g. circular-arc) detour rather than the two straight chords.
2. **Moving the detour point to reduce the kink angle directly**, e.g. placing it to make the start→mid and mid→end chords more nearly collinear (smaller turn angle), at the cost of less J3 margin (the current point was chosen purely to maximize J3 margin, with no consideration of path smoothness). A 2D/3D search over candidate detour points, jointly optimizing for (a) IK margin from J3's 160° limit and (b) the turn angle / implied curvature at the detour, has not been done.
3. **A 4th (or more) keyframe scheme**: e.g. two closely-spaced interior keyframes that let the PCHIP/quintic machinery approximate a smooth arc rather than a single sharp corner, instead of fighting a single corner with velocity overrides.
4. **Checking whether `allow_long_pan`-style per-channel speed/acc *limits* are actually the right ones for this path**, i.e. whether 120deg/s / 600deg/s² (`Limits.max_joint_speed_deg_s`, `max_joint_acc_deg_s2`, defaults in `schema.py`) are themselves appropriate for this real motion, or whether the user would accept raising them in `design.yaml`'s `limits:` block for this specific camera and payload (not yet asked).
5. **Revisiting the IK seeding/branch choice at the detour itself**: the found middle-keyframe joint solution (`seed = [89.718,-90.297,0.077,29.659,-88.781,45.769]` was reused as the single seed for the detour-point search; a different configuration branch at the detour point might have a smaller J2 jump relative to both neighbours even if J3's margin is less generous.

### Constraints that any fix must satisfy
- **`check_ik` must keep passing** (do not reopen the elbow-limit problem; any new middle keyframe or additional keyframes must stay clear of J3 > ~155° along the whole path, checked by a dense IK sweep as above, not just at the keyframes themselves, since the failure was in the *interpolated path*, not at any single keyframe).
- **Zero roll is exact by construction** in `angles` mode (pan/tilt-only orientation, per `pose.py`'s `level_rotation`) and must stay that way; do not introduce explicit rotation interpolation.
- **D7: motions should stay ≤20s** (not a hard code-enforced limit currently, but a project guideline, now under pressure -- 18s is already at this ceiling and still fails). A fix that requires pushing well past 20s to pass `check_joint_speed_acc` should be treated as not solving the actual problem.
- **Tool frame, workspace box, and the two user-specified endpoint keyframes (position and orientation) are fixed** -- they come from the user's physical jogging and tool-1 configuration and should not be altered. Only the middle keyframe (position, orientation, timing, and optionally its `vel` override) and/or the number/placement of interior keyframes are free to change.
- **Validation must be run against real IK** (the simulator at `192.168.50.2` or the real robot), not just the offline `plan` command, because `check_ik`/`check_joint_limits`/`check_joint_speed_acc` all depend on the controller's actual inverse kinematics, which the project's own fakes only approximate (see `sw/tests/fakes/fake_kinematics.py`; it is NOT accurate enough to debug this specific elbow-limit problem, which is a real behaviour of the physical/simulated kinematic chain, and was only found by querying the real simulator directly).
- **Default limits**: `max_tcp_speed_mm_s=800`, `max_joint_speed_deg_s=120`, `max_joint_acc_deg_s2=600`, `max_joint_step_deg=2.0`, all overridable per-design in `design.yaml`'s `limits:` block (`schema.py`'s `Limits` dataclass) but not yet discussed with the user as something to change.

### Reproducing the problem
```
# from the repo root, simulator must be reachable at 192.168.50.2
uv run python -m sw.robot.motionctl plan camtest        # offline peak-speed estimate, no robot
uv run python -m sw.robot.motionctl validate camtest --ip 192.168.50.2   # full check, real IK calls
```
(`sw/robot/motions/camtest/design.yaml` must exist with the content above; `rm -f sw/robot/motions/camtest/validated.json` before re-running `validate` if iterating, since a stale file is otherwise just overwritten anyway -- not load-bearing, just a habit from earlier debugging in this session.)

To inspect per-sample joint values/speeds directly against the real simulator (used throughout this investigation):
```python
from fairino import Robot
from sw.robot import schema, path as path_module
from sw.robot.path import to_sdk_pose

design = schema.load_design("sw/robot/motions/camtest/design.yaml")
samples = path_module.sample_path(design)
r = Robot.RPC("192.168.50.2")
ref = list(design.seed_joints)
joints = []
for s in samples:
    err, j = r.GetInverseKinRef(0, to_sdk_pose(s), ref)
    joints.append(j)       # None on failure; check err first in real use
    ref = j
r.CloseRPC()
# then: import numpy as np; arr = np.array(joints); speeds = np.abs(np.diff(arr, axis=0)) / (design.sample_period_ms/1000)
```

## History (facts, not instructions)
Full earlier text of this document: `git show efdd2df:docs/design/feature/fairino-python-basic-control.md`.

- **Phase 1 (commit `36d0914`).** Pure geometry and schemas; 87 robot tests. Changes from the original plan: roll is measured about the optical axis against `u = normalise(z_cam × z_base)`; pointing error uses `atan2(|cross|, dot)`.
- **Phase 0 on the simulator (commit `1526e5a`).** Euler convention `"xyz"` (extrinsic) from a J1 rotation test. The simulator does not store uploaded files or play trajectories. A second connection's `StopMotion` did not interrupt a running `MoveJ`.
- **TrajectoryJ attempts (abandoned).** `LoadTrajectoryJ` returned 4 ("parameter value abnormal") for every layout tried, including the pendant's own recording; `GetTrajectoryStartPose` read the TPD header as a point. Dropped in favour of TPD (D1).
- **TPD playback (commit `bcbcbee`).** `LoadTPD("tracktest1")` + `MoveTPD` replays a pendant recording from Python. `MoveTPD` needs auto mode; read `robot_mode` only after a 1 s wait.
- **Upload race (commit `efdd2df`).** Ports 20010/20011 open only during a transfer, about 50 ms after the `FileUpload` request; the SDK's single immediate connect is refused (-1). Our retrying upload works; a connect without sending leaves an empty file. Type 0 uploads are rejected by `LuaUpLoadUpdate` for large files ("LUA FILE SIZE IS TOO LARGE!").
- **End-to-end TPD route on the real robot (2026-11-24).** `sw/tracktestpc.txt` uploaded as type 20 → `/fruser/traj/tracktestpc.txt` (MD5 `442c28842962b0e814d6ff9aa151a9f8` matches) → `LoadTPD("traj/tracktestpc")` = 0 → `GetTPDStartPose` = first row's pose within 0.000013 → `MoveTPD("traj/tracktestpc", 0, 50)` played the track (`sw/fairino_basic_query.py`, `MOVE_OVL` 50, `PLAY_OVL` 50).
- Pendant recordings live at `/fruser/<name>.txt` and load as `LoadTPD("<name>")`.
- Phase 4 robot test results: *(to be filled in: played at 8 ms yes/no and fallback used, `move_tpd` return time, measured vs expected duration, visual level check)*.
