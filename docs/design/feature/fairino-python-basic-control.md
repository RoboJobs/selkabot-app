# Plan: Camera-level robot motions (Fairino FR5) for livestream.py

## Context
A GoPro is mounted on the FR5 flange. The goal is cinematic camera moves in the style of the "glambot": fast horizontal and vertical sweeps, crane moves and push-ins. Through all of them the camera **never rolls about its optical axis**, so the horizon stays level, while pan and tilt can follow keyframes.

Motions are designed as keyframe files, validated offline against the robot's own kinematics, rehearsed, and then uploaded to the controller under a name. The operator app `sw/livestream.py` selects a published motion with a button, moves the robot to the motion's start pose on request, and runs the motion when recording starts. **The app never generates or modifies a trajectory.** It only loads and runs files that were validated beforehand.

Starting point:
- The Fairino Python SDK is installed from `third_party/fairino_sdk/linux` (see `pyproject.toml`, `[tool.uv.sources]`). That folder is git-ignored, so read the SDK on disk.
- The SDK connection works (`sw/fairino_basic_query.py`). The robot is at `192.168.58.2` over a USB network adapter.
- The SDK manual is `third_party/fairino_sdk/fairino-doc-en-readthedocs-io-en-3.9.1.pdf`. `pdftotext -layout` makes it searchable.

Terminology: a **motion** is one named, validated trajectory, e.g. `sweep_left_fast`. Avoid the word "shot".

## Findings
- **The SDK already gives what is needed; MoveIt2 is not used.** MoveIt2 needs ROS2 Humble on Ubuntu 22.04, which this host doesn't have, and it solves collision-free A-to-B planning, not choreographed camera paths.
- **SDK functions used.** All are in `fairino/Robot.py`, class `RPC`:
  - Kinematics, with no motion: `GetInverseKinRef(type, desc_pos, joint_ref)`, `GetInverseKinHasSolution`, `GetForwardKin(joints)`.
  - Trajectory files: `TrajectoryJUpLoad(local_path)` stores the file at `/fruser/traj/<file>`. Also `TrajectoryJDelete(file)`, `LoadTrajectoryJ(name, ovl, opt=1)`, `MoveTrajectoryJ()`, `GetTrajectoryStartPose(name)`, `GetTrajectoryPointNum()`, `ComputeFileMD5(path)`. Reference flow: `linux/example/TestUploadTrajectoryJ.py`.
  - Motion and state: `MoveJ(joint_pos, tool, user, vel=...)`, `SetSpeed(pct)` (global speed), `StopMotion()`, `GetActualJointPosDegree()`, `GetRobotMotionDone()`, and `robot_state_pkg` (`motion_done`, `main_code`, `sub_code`, `jt_cur_pos`, `tl_cur_pos`). `robot_state_pkg` is refreshed by the SDK's own 8 ms state thread.
  - Setup: `SetToolCoord`, `SetWObjCoord`, `SetLoadWeight`, `SetLoadCoord`, `GetTCPOffset`, `ResetAllError`, `RobotEnable`, `Mode`.
  - Fallback store: `LuaUpload`, `ProgramLoad`, `ProgramRun`, `ProgramStop`, `GetLuaList`, `LuaDelete`.
- **Return values.** Most SDK calls return an `int` error code (0 = OK). Getters return `(code, value)`. Motion calls first return `GetSafetyCode()` when it is non-zero.
- **SDK pose** is `[x, y, z, rx, ry, rz]` in mm and degrees, for the active tool frame relative to the active user frame. The Euler convention is believed to be fixed-axis XYZ (`R = Rz·Ry·Rx`), but this is **not confirmed** (Step 0).
- **Rotation blending.** When the controller is given Cartesian points (`MoveL`, `NewSpline*`), it blends the full rotation between consecutive points, and with `blendR` or the arc-transition spline type it also blends across neighbouring points. This was observed on the real robot during spline testing. The interpolation method is undocumented. Between two level orientations that differ in both pan and tilt, a rotation blend passes through orientations with roll. Feeding the controller only the keyframes is therefore not enough to keep the camera level (§ Architecture, "Level constraint").
- **Limits.** FR5 joint speed is about 180°/s at 100%, and TCP speed roughly 1 m/s. That is slower than a commercial glambot arm, so motions should be designed for "brisk", not "whip".

## Open questions resolved by Step 0 (hardware spike)
| # | Question | Blocks |
|---|---|---|
| Q1 | Euler convention of `rx, ry, rz`, and the camera tool-frame axes | `pose.py` constants (the code can be written first, with the convention as a parameter) |
| Q2 | Column format of `/fruser/traj/*.txt`: joints or poses, period, header | `traj_file.py` |
| Q3 | Does `LoadTrajectoryJ(..., opt=1)` replay the points as exact samples or as spline control points? What is the tracking error? | the validity of the dense-sampling approach |
| Q4 | Does a `MoveJ` after `LoadTrajectoryJ` keep the loaded trajectory? Does `MoveTrajectoryJ` block? | `MotionRunner.load/run` |
| Q5 | Can `StopMotion()` be sent from a second `Robot.RPC` connection while the first is blocked in `MoveTrajectoryJ`? | `MotionRunner.stop` |
| Q6 | Measured roll: dense 8 ms pan+tilt trajectory vs a sparse two-keyframe version | confirms the "Level constraint" design |

If Q2 or Q3 fail, i.e. the trajectory file can't be made to replay the samples faithfully, switch to the Lua fallback (§ Risks). Only `traj_file.py` → `lua_file.py`, and the load/run calls in `MotionRunner`, change.

## Caveats and risks
- **Lua fallback.** A generated `/fruser/motion_<name>.lua` holds the same dense samples as explicit-joint points: `NewSplineStart(1)` ("given points are path points"), or `MoveL` with `blendR` ≤ half the point spacing. It is run with `ProgramLoad` + `ProgramRun` and stopped with `ProgramStop`. Timing is then partly decided by the controller, and the rotation-blending caveat applies. The validator's midpoint roll checks and the rehearsal measurements still guard it.
- **Thread safety of the SDK.** The SDK's XML-RPC proxy is not assumed to be thread-safe. All calls go through one worker thread, and `stop()` uses a dedicated second connection (Q5). If Q5 fails, `run()` must be non-blocking: start the motion, then poll `motion_done`, so the worker stays free for `StopMotion`.
- **Controller filename length.** The C API uses `char name[30]`, so `motion_<name>.txt` must stay ≤ 30 characters. Motion names are therefore `^[a-z0-9_]{1,19}$`.
- **Spline overshoot.** Quintic splines can overshoot between keyframes. The validator's workspace-box check catches this, and the designer then adds keyframes.
- **Safety** is enforced in layers:
  - controller collision detection, with the correct payload;
  - soft joint limits;
  - offline validation;
  - rehearsal ramp (10 → 100%);
  - Move to Start at 10% global speed;
  - the start-pose check before Record;
  - `StopMotion` on Stop, close, or any error.

  The physical e-stop remains the primary safety device. Operating rules (nobody in the cell during rehearsal) belong in the README section added in Phase 6.

## Architecture

### Package map and dependency rules
New package `sw/robot/`. Arrows mean "may import".

```
            pure (no SDK, no Tk, no I/O beyond files)          hardware boundary
 ┌────────────────────────────────────────────────────┐   ┌──────────────────────┐
 │ pose.py  ← path.py ← validate.py                   │   │ fr5.py               │
 │    ↑                     ↑                         │   │  RobotClient Protocol│
 │ schema.py (design / validated / motion dataclasses)│   │  FR5 (wraps RPC)     │
 │    ↑                                               │   │  FairinoError        │
 │ traj_file.py (writer for /fruser/traj format)      │   └──────────▲───────────┘
 └───────────────▲────────────────────────────────────┘              │
                 │                                                   │
      motionctl.py (CLI: plan/validate/compile/rehearse/publish) ────┤
      motion_runner.py (MotionRunner, used by livestream) ───────────┘
      sw/livestream.py → motion_runner.py, schema.py (never fr5 directly)
```

Rules:
- Only `fr5.py` imports `fairino`.
- `pose`, `path`, `validate`, `schema` and `traj_file` never import `fr5` or `fairino`. They are unit-testable without hardware.
- `validate.py` gets an IK/FK provider through a small Protocol (`Kinematics`). The real one is backed by `FR5`, and tests use a fake. This keeps validation pure and lets it use the controller's own kinematics.
- `livestream.py` talks only to `MotionRunner` and the motion configuration dataclasses.

### Data flow (offline → controller → app)
```
design.yaml ──plan──▶ samples (t, pose, pan, tilt)         [pose.py, path.py]
            ──validate──▶ validated.json (+ joints via controller IK, checks)  [validate.py, fr5 IK/FK]
            ──compile──▶ build/motion_<name>.txt           [traj_file.py]
            ──rehearse──▶ runs a temp upload at increasing ovl, logs, sets approved_ovl
            ──publish──▶ /fruser/traj/motion_<name>.txt on controller + motion.json (md5, start_joints, …)
livestream.py start ─▶ MotionRunner.connect_and_check(all motion.json)  (once)
button+confirm ─▶ load(name) │ Move to Start ─▶ move_to_start(name) @10%
Record ─▶ at_start(name)? ─▶ GoPro start ─▶ pre_roll ─▶ run(name) ─▶ done ─▶ post_roll ─▶ GoPro stop
```

### Level constraint (core design)
Frames:
- **Base frame:** z points up.
- **Camera tool frame** (`SetToolCoord`, origin at the optical centre): **z = optical axis (forward), x = image right, y = image down.**

**Constraint:** the image-x axis stays horizontal, `x_cam · ẑ_base = 0`. This is a zero dot product. A zero cross product would mean parallel axes.

**Parameterization.** Orientation is never stored or interpolated as a rotation. It is built from two angles:
- **pan** (deg): heading of the optical axis about base +Z, with 0 = looking along base +X and positive = counter-clockwise seen from above.
- **tilt** (deg): elevation of the optical axis, positive = looking up, limited to |tilt| ≤ 80° to avoid the vertical singularity.
- `R(pan, tilt) = Rz(pan) · R_level0 · Rx(tilt)`, where:
  - `R_level0` has columns `x_cam = (0,−1,0)`, `y_cam = (0,0,−1)`, `z_cam = (1,0,0)`;
  - `Rx(tilt)` is a rotation about the camera's own x axis. Because it rotates about x_cam, x_cam stays equal to `Rz(pan)·(0,−1,0)`, which is horizontal, so the constraint holds **exactly** for every (pan, tilt).
- `look_at` keyframes are converted with `d = target − pos`, `pan = atan2(d_y, d_x)`, `tilt = atan2(d_z, hypot(d_x, d_y))`.

**Path and timing.** Every keyframe has a time `t`. `x(t), y(t), z(t), pan(t), tilt(t)` are each a **quintic interpolating spline** through the keyframes, with zero velocity and zero acceleration at the first and last keyframe (min-jerk-like start and stop). Use `scipy.interpolate.make_interp_spline(k=5, bc_type=...)`. Pan is unwrapped before fitting.

**Sampling.** The path is sampled at the controller trajectory period, `sample_period_ms`, default 8 (Q2/Q3 may change it). At 8 ms consecutive samples differ by a fraction of a degree; a 60°/s pan is 0.48° per sample. Whatever rotation blending the controller applies between such close neighbours, the roll it can add is second order in the per-sample change, about 0.001°.

**Joints.** Joints are solved per sample by `GetInverseKinRef`, seeded from the previous sample (the first sample is seeded from `seed_joints` in the design). Joints are what gets uploaded, so the controller performs no IK and makes no configuration choice.

**Verified, not assumed.** The validator checks roll at every sample (via FK of the solved joints) and at every interval midpoint, under both SLERP and linear-Euler interpolation. Rehearsal measures the actual roll from `tl_cur_pos`.

### Threading and UI integration
- `MotionRunner` owns a single-thread `concurrent.futures.ThreadPoolExecutor(max_workers=1)`. All its public methods except `stop()` submit work there and return a `Future`.
- `stop()` sends `StopMotion()` on a second `FR5` connection, opened in `connect_and_check`, so it is never queued behind a running motion (Q5). It is idempotent and never raises; failures are only logged.
- `LivestreamApp` gets `_run_robot(future, status, on_done, on_error)`, which mirrors the existing `_run`. It sets the busy state and status, and its done-callback posts the finish closure onto the existing `self._events` queue, which `_poll` drains on the Tk thread. **Tk widgets are never touched from the robot worker.**
- Motion completion: `run()` returns a Future that resolves when the motion finishes (`motion_done == 1` after the start, with no error code), or raises `FairinoError` on a robot error. Polling happens every 50 ms on the worker thread.

### Error handling contract
- `FR5._call(name, *args)` invokes the SDK method. A non-zero error code raises `FairinoError(code, name)`. For `(code, value)` returns it returns `value`.
- `FairinoError.__str__` includes the call name and the code.
- `MotionRunner` translates exceptions into Future exceptions. The UI shows `f"Robot error: {e}"` in the status bar and calls `stop()` (harmless if already stopped).
- `motionctl` exits with code 1 and a one-line message on any `FairinoError` or validation failure, and prints the full report for validation failures.

## Data schemas
All files are UTF-8. Units are mm and degrees unless stated otherwise. Every schema has a matching frozen dataclass in `schema.py` with `from_dict`/`to_dict`. Loaders raise `SchemaError(path, field, reason)` on missing or wrong fields.

### `sw/robot/motions/<name>/design.yaml` (hand-written)
```yaml
name: sweep_left_fast          # must equal folder name; ^[a-z0-9_]{1,19}$
tool_id: 1                     # camera tool frame on the controller
user_id: 0                     # 0 = base; or the "set" work-object frame
sample_period_ms: 8
seed_joints: [0, -90, 90, -90, -90, 0]   # IK seed for the first keyframe
limits:
  max_tcp_speed_mm_s: 800
  max_joint_speed_deg_s: 120
  max_joint_acc_deg_s2: 600
  max_joint_step_deg: 2.0      # per-sample jump => wrist flip / singularity
workspace:                     # box in the user frame
  min: [200, -700, 150]
  max: [900,  700, 1100]
keyframes:                     # t strictly increasing, first t == 0, at least 2
  - {t: 0.0, pos: [600, -400, 450], pan: 20,  tilt: -5}
  - {t: 1.2, pos: [600,  400, 450], pan: -20, tilt: -5}
  - {t: 2.0, pos: [650,  400, 700], look_at: [0, 0, 300]}
```
Each keyframe has exactly one of: (`pan` **and** `tilt`) or `look_at`.

### `sw/robot/motions/<name>/validated.json` (generated by `validate`)
- `name`, `design_sha256`, `created` (ISO 8601), `sample_period_ms`, `tool_id`, `user_id`, `tcp_offset[6]` (read from the controller).
- `samples`: `{t: [N], pose: [N][6], pan: [N], tilt: [N], joints: [N][6]}`.
- `peaks`: `{tcp_speed_mm_s, joint_speed_deg_s[6], joint_acc_deg_s2[6], max_joint_step_deg, max_roll_deg, max_fk_pos_err_mm, max_fk_rot_err_deg}`.
- `checks`: a list of `{name, passed, detail}`. The file is written only when every check passes. Otherwise the report is printed and nothing is written.

### `sw/robot/motions/<name>/motion.json` (generated by `publish`; read by the app)
```json
{
  "name": "sweep_left_fast",
  "controller_file": "/fruser/traj/motion_sweep_left_fast.txt",
  "md5": "…",
  "start_joints": [..6..],
  "start_pose": [..6..],
  "tool_id": 1, "user_id": 0,
  "tcp_offset": [..6..],
  "duration_s": 2.0,
  "approved_ovl": 100,
  "design_sha256": "…", "validated_sha256": "…",
  "published_at": "2026-11-24T10:00:00"
}
```

### `sw/livestream_config.json` (new keys, merged by the existing `save_config` pattern)
- `"motion"`: the selected motion name, or `""` for no motion.
- `"robot"`: `{"ip": "192.168.58.2", "start_tolerance_deg": 0.5, "move_to_start_speed_pct": 10, "pre_roll_s": 0.5, "post_roll_s": 1.0, "auto_stop_after_motion": true}`.

## Module interfaces
Signatures are binding for the implementer. Private helpers are free.

- **`pose.py`**
  - `level_rotation(pan_deg, tilt_deg) -> np.ndarray(3,3)`
  - `look_at_angles(pos, target) -> tuple[pan, tilt]`
  - `rotation_to_sdk_euler(R) -> (rx, ry, rz)` and `sdk_euler_to_rotation(rx, ry, rz) -> R`. The convention is the module constant `SDK_EULER = "xyz"`, set by Q1.
  - `roll_error_deg(R) -> float`: the angle of x_cam out of the horizontal plane.
  - `unwrap_deg(seq) -> list`
- **`path.py`**
  - `Sample(t, pos, pan, tilt)`
  - `build_path(design: Design) -> Callable[[float], Sample]`
  - `sample_path(design) -> list[Sample]`, sampled at `sample_period_ms`, always including t = 0 and t = T.
  - `to_sdk_pose(sample) -> list[6]`
- **`validate.py`**
  - `class Kinematics(Protocol)`: `ik(pose6, seed6) -> list[6] | None`, `fk(joints6) -> list[6]`.
  - `validate(design, samples, kin) -> ValidationReport`, holding `passed: bool`, `checks`, `peaks` and `joints`.
  - Checks, each a separate function so it can be tested alone:
    - `check_ik`
    - `check_joint_steps`
    - `check_joint_speed_acc` (finite differences)
    - `check_tcp_speed`
    - `check_workspace`
    - `check_roll` (at samples, and at SLERP and linear-Euler midpoints, < 0.05°)
    - `check_fk_roundtrip` (< 0.1 mm, < 0.05°)
- **`traj_file.py`**: `write(validated: Validated, path: Path) -> None` and `read(path) -> list[list[float]]`, for round-trip tests. The format follows Q2.
- **`schema.py`**: `Design`, `Keyframe`, `Validated`, `MotionConfig`, plus `load_design(path)`, `load_validated(path)`, `load_motion_configs(motions_dir) -> dict[str, MotionConfig]` (skips folders without `motion.json`), `sha256_file(path)`.
- **`fr5.py`**
  - `class FairinoError(RuntimeError)`, carrying `code` and `call`.
  - `class RobotClient(Protocol)`: the exact SDK methods listed in Findings, used by `FR5`, so fakes can implement it.
  - `class FR5`, which takes `client: RobotClient`:
    - `connect(ip) -> FR5` (classmethod)
    - `ik`, `fk` (satisfying `Kinematics`)
    - `joints() -> list[6]`, `tcp_pose()`, `tcp_offset()`, `error_codes() -> (main, sub)`, `motion_done() -> bool`
    - `move_j(joints, tool, user, speed_pct)`
    - `upload_traj(local_path)`, `delete_traj(file)`, `file_md5(remote_path) -> str`
    - `load_traj(remote_path, ovl)`, `run_traj()`, `stop()`
    - `prepare()` (reset errors, enable, auto mode)
- **`motion_runner.py`**
  - `class MotionRunner`, which takes `connect: Callable[[], FR5]` (so tests can inject fakes) and `settings: RobotSettings`. Methods:
    - `connect_and_check(configs) -> Future[dict[str, str | None]]`: maps each name to None if OK, or to the reason it is disabled.
    - `load(name) -> Future[None]`
    - `move_to_start(name) -> Future[None]`
    - `at_start(name) -> tuple[bool, float]`: synchronous, reads `robot_state_pkg` through FR5, and returns (ok, max_err_deg).
    - `run(name) -> Future[None]`
    - `stop() -> None`
    - `close() -> None`
- **`motionctl.py`**: run as `python -m sw.robot.motionctl <cmd> NAME [--ip IP]`, with these commands:
  - `plan NAME [--preview]`
  - `validate NAME`
  - `compile NAME`
  - `rehearse NAME --ovl 10,25,50,100`
  - `publish NAME --ovl N`
  - `list`
  - `remove NAME`
  - `new NAME --template sweep|crane|orbit|push_in`, which writes a starter `design.yaml` from `templates.py`.

## Phases
**Gate rule for every phase:** run `uv run pytest sw/tests/ -q` (the full suite). Every test passes, with no new skips or failures, and `uv run ruff check sw/ && uv run ruff format --check sw/` is clean (line length 88, rules E/F/I). Each phase ends in its own commit on `feature/fairino-python-basic-control`. **No unit test may require the robot, the GoPro, or the network.** Hardware-only scripts live outside `sw/tests/` or are marked `@pytest.mark.hardware` and skipped by default (register the marker in `pyproject.toml`).

### Phase 0: Hardware spike (Step 0; needs the operator at the robot)
- **Work:** write `sw/robot/spike/` scripts. Each prints its findings and doesn't need tests.
  - Q1 `euler_probe.py`: jog to 3–4 poses and print `GetActualTCPPose` next to the scipy conversion of `GetForwardKin`, for each Euler candidate.
  - Q2/Q3 `traj_probe.py`: record an 8 ms TPD trajectory, download it, write a small file in the same format, upload, `LoadTrajectoryJ`/`MoveTrajectoryJ` at 10%, and log tracking error.
  - Q4/Q5: the same script tests a `MoveJ` after the load, whether `MoveTrajectoryJ` blocks, and `StopMotion` from a second connection.
  - Q6 `roll_probe.py`: a dense pan+tilt trajectory vs a sparse two-point `MoveL`, logging roll from `tl_cur_pos`.
- **Done when:** the answers to Q1–Q6 are written into a "Step 0 results" section appended to this doc, including a sample trajectory file excerpt. If Q2/Q3 fail, the decision to use the Lua fallback is recorded there.
- Phases 1–3 can be developed in parallel with Phase 0, because they don't depend on its answers beyond constants.

### Phase 1: Pure geometry and schemas (`pose.py`, `path.py`, `schema.py`, `templates.py`)
- **Work:** as specified in Module interfaces. Add `scipy` and `pyyaml` to `pyproject.toml` dependencies.
- **Tests:**
  - `sw/tests/test_robot_pose.py`:
    - for 1,000 random (pan, tilt) with |tilt| ≤ 80: `R` is orthonormal with det = 1, `|x_cam · ẑ| < 1e-12`, the optical axis heading equals pan and its elevation equals tilt;
    - `look_at_angles` for known targets (ahead, left, above);
    - SDK Euler round-trip for random rotations;
    - `roll_error_deg` equals the injected roll for a rotated frame;
    - `unwrap_deg` across ±180.
  - `sw/tests/test_robot_path.py`:
    - the path passes through every keyframe (pos ±1e-6 mm, pan/tilt ±1e-6°);
    - velocity and acceleration are zero at t = 0 and t = T;
    - the sample count is `floor(T/period) + 1`, and it includes T;
    - pan unwrapping (keyframes 170° → −170° turn 20°, not 340°);
    - `look_at` keyframes are converted;
    - every sample has `roll_error_deg < 1e-9`.
  - `sw/tests/test_robot_schema.py`:
    - a valid design loads;
    - each invalid case raises `SchemaError` with the field name: bad name regex, name ≠ folder, non-increasing t, first t ≠ 0, a keyframe with both or neither of pan/tilt and look_at, |tilt| > 80, wrong vector length;
    - `motion.json` round-trips;
    - `load_motion_configs` skips folders without `motion.json`.
  - `templates`: each template produces a design that loads and samples.
- **Done when:** the gate passes.

### Phase 2: Validator (`validate.py`)
- **Work:** as specified. Peaks are computed with finite differences on the sample times.
- **Tests** (`sw/tests/test_robot_validate.py`, with a `FakeKinematics` whose `ik` is a deterministic analytic stand-in and whose failures can be injected):
  - a well-formed design passes all checks;
  - IK returning None at one sample fails `check_ik` and names the sample index and time;
  - an injected 30° jump on J4 fails `check_joint_steps`;
  - a too-fast design (shrink keyframe times) fails speed/acc, and the peaks are reported;
  - a point outside the box, including spline overshoot between keyframes, fails `check_workspace`;
  - coarse sampling (e.g. a period of 500 ms with a large simultaneous pan and tilt change) fails the midpoint `check_roll` while dense sampling passes, which proves the check is meaningful;
  - an FK offset injected into the fake fails `check_fk_roundtrip`;
  - the report serializes to the `validated.json` schema.
- **Done when:** the gate passes.

### Phase 3: Robot boundary (`fr5.py`) with fakes
- **Work:** `FR5` wraps the SDK through `RobotClient`. Add `sw/tests/fakes/fake_robot.py` with a `FakeRobotClient` that:
  - records every call;
  - keeps joint state, which `MoveJ` sets instantly;
  - fakes `robot_state_pkg` (`motion_done`, `jt_cur_pos`, `main_code`);
  - stores uploaded files in memory and computes `ComputeFileMD5` with `hashlib.md5`;
  - can inject error codes per method;
  - can make `MoveTrajectoryJ` take N polls to complete.
- **Tests** (`sw/tests/test_robot_fr5.py`):
  - every wrapper calls the right SDK method with the right argument types and order;
  - a non-zero code raises `FairinoError` with the call name;
  - `(code, value)` unpacking;
  - `move_j` sets the global speed before moving;
  - `file_md5` returns the controller MD5 string.
- **Done when:** the gate passes. The module imports `fairino` lazily, inside `connect`, so tests don't need the SDK's native library.

### Phase 4: Trajectory file and CLI (`traj_file.py`, `motionctl.py`); needs Q1–Q4
- **Work:**
  - `traj_file.write/read` per the Q2 format.
  - `motionctl` commands per Module interfaces:
    - `compile` refuses when `validated.json` is missing, or when its `design_sha256` doesn't match the current `design.yaml`.
    - `rehearse` uploads as `motion_tmp_<name>.txt`, runs each requested ovl after an operator confirmation prompt (`input()`), logs `jt_cur_pos`/`tl_cur_pos` at 8 ms to `motions/<name>/rehearsal_<ovl>.csv`, prints tracking and roll statistics, and deletes the temporary file afterwards.
    - `publish --ovl N` requires that a rehearsal at ≥ N exists, then uploads, compares the MD5 and writes `motion.json`.
  - Add `sw/robot/build/` to `.gitignore`. Commit the `motions/*/` files.
- **Tests:**
  - `sw/tests/test_robot_traj_file.py`: write → read round-trip with exact numeric precision; the header and line count match the sample count; the format matches the Step 0 excerpt (golden string).
  - `sw/tests/test_robot_motionctl.py`, with `FakeRobotClient` and `tmp_path` motions dir:
    - `validate` writes `validated.json` only on pass;
    - `compile` refuses stale designs;
    - `publish` writes `motion.json` with the correct md5, start_joints (= first sample's joints) and duration, and refuses an ovl that wasn't rehearsed;
    - `remove` deletes the controller file and `motion.json`;
    - `list` reports a controller file without a `motion.json`, and the reverse;
    - exit code 1 with a message on `FairinoError`.
- **Done when:** the gate passes, and `plan`/`validate`/`compile`/`rehearse`/`publish` complete on the real robot for one template motion (manual, operator present).

### Phase 5: `MotionRunner` (`motion_runner.py`); needs Q4/Q5
- **Work:** as specified in Threading and Module interfaces.
  - `connect_and_check` runs once and checks, per motion: the controller file exists and its MD5 equals `motion.json.md5`; the `tool_id`/`tcp_offset` match the controller (`GetTCPOffset` for that tool, within 0.1 mm / 0.05°). It returns reasons such as `"file missing on controller"`, `"md5 mismatch"` or `"tool offset differs"`.
  - `load` calls `LoadTrajectoryJ(controller_file, approved_ovl, 1)`.
  - `move_to_start` sets `SetSpeed(move_to_start_speed_pct)` and then `MoveJ(start_joints, tool, user)`.
  - `at_start` compares `jt_cur_pos` with `start_joints` (max absolute difference ≤ `start_tolerance_deg`).
  - `run` re-calls `load` first if Q4 says a `MoveJ` invalidates it, then `SetSpeed(approved_ovl)` + `MoveTrajectoryJ`, and waits for `motion_done` while checking error codes.
- **Tests** (`sw/tests/test_motion_runner.py`, with `FakeRobotClient`):
  - `connect_and_check` reports OK, a missing file, an md5 mismatch and a tool mismatch;
  - **no runtime method calls `ComputeFileMD5` or uploads anything** (the check happens once only);
  - `load` never moves the robot (no MoveJ or MoveTrajectoryJ calls);
  - `move_to_start` sets speed 10 before `MoveJ`, with the motion's start joints;
  - `at_start` is true within tolerance, false just outside it, and reports the max error;
  - `run` resolves when the fake's motion completes, and raises `FairinoError` when an error code appears mid-motion;
  - `stop()` while `run` is in progress calls `StopMotion` on the second connection immediately (not queued) and makes `run` finish;
  - `stop()` is idempotent and swallows errors;
  - the methods for a motion that failed the check raise `ValueError`.
- **Done when:** the gate passes.

### Phase 6: Livestream integration (`sw/livestream.py`)
- **Work:**
  - `main()`: `--robot-ip` (overrides config) and `--no-robot`. Build a `MotionRunner` unless `--no-robot` is given, and pass it to `LivestreamApp(..., motion_runner: MotionRunner | None, motions: dict[str, MotionConfig])`.
  - **Startup:** `connect_and_check` runs via `_run_robot`. The results enable or disable each motion button; disabled buttons get the reason in the status bar when clicked. If the saved `"motion"` is OK, `load()` it.
  - **Motion row** (`_build_motion_row`): one `tk.Button` per motion plus "No motion". The active one is highlighted (sunken relief and colour).
    - Click → `messagebox.askyesno("Change motion", f"Change the motion to '{name}'?")`.
    - Yes → save `"motion"` and call `load(name)`; the status shows "Motion '{name}' loaded. Press Move to Start." **The robot does not move.**
    - No, or clicking the active motion → nothing happens.
  - **"Move to Start" button**, packed next to "Start recording" in the `controls` frame:
    - Click → `move_to_start(name)`; the status shows "Moving to start of '{name}' (10%)…".
    - Disabled for "No motion", while busy, and while recording.
    - Highlighted (orange) when `at_start` is false, re-evaluated in `_update_controls` and after each robot operation.
  - **Record** (`_on_record`, not recording, motion selected): the first step is `at_start(name)`.
    - If false, return with the status "Robot is not at the start of '{name}' (max error X.X°). Press Move to Start."
    - Otherwise, proceed as today. In `_on_recording_started`, schedule `root.after(pre_roll_s*1000, start motion)` → `run(name)` via `_run_robot`.
    - When it finishes, and `auto_stop_after_motion` is set, `root.after(post_roll_s*1000, self._on_record)` stops the recording through the existing path.
    - With "No motion", Record is unchanged.
  - **Stop**, `_on_close`, a GoPro start/stop error, or a robot error: call `motion_runner.stop()` first. `_on_close` also calls `motion_runner.close()`.
  - `_update_controls`: the motion buttons and Move to Start are disabled while busy, recording or moving, and failed motions stay disabled.
- **Tests** (extend `sw/tests/test_livestream_ui.py`, following its existing `root`/`app` fixture and `FakeController` patterns, plus a `FakeMotionRunner` that records calls and returns pre-resolved Futures; monkeypatch `messagebox.askyesno`):
  - with `--no-robot` (runner None), no motion row and no Move to Start button are built, and the existing tests pass unchanged;
  - the buttons reflect the startup check (a failed motion is disabled);
  - confirming the dialog calls `load` and saves the config, and **does not call** `move_to_start` or `run`;
  - declining changes nothing; clicking the active motion doesn't open the dialog;
  - Move to Start calls `move_to_start` with the selected name;
  - Record with `at_start` false doesn't call `controller.start_recording` and sets the status message;
  - Record with `at_start` true starts recording, and `run` is called only after the recording-started callback plus pre-roll (drive `root.after` with `root.update()` or a monkeypatched `after`);
  - motion completion triggers stop recording when `auto_stop_after_motion` is set;
  - Stop during a motion calls `runner.stop()` before `stop_recording`;
  - `_on_close` calls `stop()` and `close()`;
  - Record and the motion buttons are disabled while the robot is moving.
- **Docs:** add a "Robot motions" section to `README.md` covering the workflow (`new` → `plan` → `validate` → `compile` → `rehearse` → `publish`), operating safety rules, and the config keys.
- **Done when:** the gate passes, and the end-to-end check in Verification succeeds on hardware.

## Critical files
- **New:**
  - `sw/robot/__init__.py`, `pose.py`, `path.py`, `schema.py`, `templates.py`, `validate.py`, `fr5.py`, `traj_file.py`, `motion_runner.py`, `motionctl.py`, `spike/*.py`
  - `sw/robot/motions/<name>/{design.yaml,validated.json,motion.json,rehearsal_*.csv}`
  - `sw/tests/fakes/fake_robot.py`
  - `sw/tests/test_robot_{pose,path,schema,validate,fr5,traj_file,motionctl}.py`, `sw/tests/test_motion_runner.py`
- **Modified:** `sw/livestream.py`, `sw/tests/test_livestream_ui.py`, `pyproject.toml` (`scipy`, `pyyaml`, `hardware` pytest marker), `.gitignore` (`sw/robot/build/`), `README.md`.
- **Reference (read, don't modify):**
  - `third_party/fairino_sdk/linux/fairino/Robot.py`
  - `third_party/fairino_sdk/linux/example/TestUploadTrajectoryJ.py` and `servo.py`
  - `sw/fairino_basic_query.py`
  - the existing `_run`/`_events`/`_poll` pattern and `save_config` in `sw/livestream.py`

## Verification
1. **Automated:** `uv run pytest sw/tests/ -q` and ruff are green after every phase (gate rule).
2. **Offline:** `motionctl new demo --template sweep`, then `plan demo --preview` and `validate demo`. Review the peak joint speeds and `max_roll_deg` in the report.
3. **Robot, nobody in the cell, e-stop at hand:** `compile demo`, then `rehearse demo --ovl 10,25,50,100`. Measured roll must stay within ±0.2° and position tracking within 3 mm at every level. Then `publish demo --ovl 100`.
4. **End to end in `livestream.py`:**
   - The startup check enables `demo`.
   - Select it and confirm: the motion loads and the robot does not move.
   - Press Record: it is refused with the "not at start" message.
   - Press Move to Start: the robot moves at 10%.
   - Press Record: the check passes, the GoPro records, the motion runs after the pre-roll, and recording auto-stops.
   - Check the clip for a level horizon.
   - Press Record again without Move to Start: it is refused.
   - Repeat with Stop pressed mid-motion: the robot halts immediately and the recording stops.
5. **Tamper test:** modify `motion.json`'s md5 and restart the app. `demo` shows as disabled with "md5 mismatch".
