# Portrait camera motions (wrist-rolled, same camera tool)

Branch: `feature/fairino-control-2`. Builds on `docs/design/feature/fairino-python-basic-control.md` (read it first). Video-pipeline changes come later and are out of scope.

## Context
Today every motion is landscape: in the camera tool frame (tool 1: z = optical axis, x = image right, y = image down), image-right stays horizontal and image-down points down, so **roll = 0**. Portrait motions need the camera rolled 90° about its optical axis for the whole motion: image-x points (projected) downward and image-y points left. The horizon constraint still applies: the roll is held at a constant 90° instead of 0°.

How portrait is reached physically: **the camera stays tool 1, fixed to the flange, and the wrist turns it.** The operator does this by rotating J4 and J5 about 90° from a landscape pose. That is a particular maneuver, not a general rule: the solved portrait configuration may use all three wrist joints and depends on the arm configuration and the mount. The camera's pose relative to the base changes; relative to the flange it does not. **No new tool frame on the controller.**

## The constraint, precisely
With `roll_error_deg` (`sw/robot/pose.py:71`), the signed angle from the level reference `u = normalise(z_cam × ẑ)` to `x_cam` about `z_cam`:
- landscape: roll = 0 (x_cam = u, horizontal; y_cam points downward);
- **portrait: roll = +90°.** The invariant is constant roll +90°: **image-y is horizontal (pointing left)**, and image-x points along the projected downward direction. At tilt 0 that is exactly world −Z (`x=(0,0,−1)`, `y=(0,1,0)`, `z=(1,0,0)` at pan 0, checked numerically: roll +90.000); at non-zero tilt it is "down" projected onto the image plane, not −Z. Roll −90 would be portrait upside down, so the sign matters and is checked.

Pan and tilt keep their meaning: they fix the **viewing direction**, not the framing. Because the camera is offset about 400 mm from the flange, rolling it with the wrist moves the optical centre and changes the composition; tools that re-level report that displacement (`LevelResult.moved_mm` already exists).

## Key risk: the SDK Euler singularity
A portrait camera looking horizontally has SDK Euler angles `[0, 90, 0]`: **ry = ±90°, the singular point of the fixed-XYZ convention.** This holds for every pan at tilt ≈ 0, and ry stays within about |tilt| of 90°. Near it rx and rz are individually ill-defined and can jump between adjacent samples; only the rotation is well defined. Landscape poses have ry ≈ 0, which is why this never came up. This is a *representation* problem, separate from the physical wrist singularity (J5 = 0/180). Where it matters:

| Place | Effect | Fix |
|---|---|---|
| Our maths (`camera_angles`, `roll_error_deg`, `level.solve_wrist`, `check_roll`) | None: they convert Euler → matrix (`sdk_euler_to_rotation`), which is well defined everywhere | — |
| `validate.check_fk_roundtrip` (`_angle_diff_deg` per Euler angle) and `motionctl._check_same_kinematics` (per-column 0.05°) | False failures: equal rotations with different Euler triples | Compare **rotations**: new `pose.rotation_angle_deg(Ra, Rb)` (angle of `Raᵀ·Rb`) |
| Cartesian mode: TPD pose columns are today the **planned** poses from our Euler conversion (`path.to_sdk_pose` → `rotation_to_sdk_euler`; scipy picks rz = 0 at the singularity) | They may not match the controller's own representation of the same rotation, and may jump between rows | **Validation** stores the controller's FK of every solved sample as the artifact's pose columns (joint mode already does this, D6). See "Artifact boundary" |
| Controller playback of rows whose rx/rz jump | Unknown: could give "TPD command point error" or an unexpected wrist turn | **Phase 0 H2/H3** (below) |

## Artifact boundary (binding)
**Validation produces the complete, checked artifact; compilation and playback only consume it.** `compile_motion` (`motionctl.py:368`) keeps serializing `validated.json` unchanged and never calls the controller.
- **Cartesian mode, in `validate`:**
  1. Solve joints (IK) for every sample from the planned poses.
  2. FK every solved sample.
  3. Compare FK against the **planned** pose: position, plus rotation by `rotation_angle_deg`. This is `check_fk_roundtrip`, now on every sample, not every 10th, since the FK is computed anyway.
  4. Store the accepted **FK poses** in `validated.json` as the pose columns.

  The planned poses stay the comparison reference; the round trip is never "FK vs stored FK".
- **Joint mode:** unchanged (it already stores FK poses).
- A tool change between validation and compilation therefore cannot silently change the artifact. `validated.json` records `tool_id` (already) and `format` (new), and `compile`/`publish` refuse a mismatch with the design.

## Design (binding)
- **D11 (new): `format: landscape | portrait`** in `design.yaml` (default `landscape`), mapped in `schema.py` to `CAMERA_ROLL_DEG = {"landscape": 0.0, "portrait": 90.0}`. `Design.camera_roll_deg` is derived from it. The tool stays the design's `tool_id` (1).
- **Metadata round trip.** `format` is stored in `validated.json` and published into `motion.json` **from the validated artifact** (not from the design). In `motion.json` it's optional (missing = landscape, so published files stay valid) and not in `MOTION_FILE_KEYS`. So `motion_config_to_dict` (`schema.py:541`, which only serializes `MOTION_FILE_KEYS`) gets an explicit addition that writes `format`. `check_consistent()` rejects an unknown format.
- **`peaks.max_roll_deg` changes meaning:** it is now the maximum **deviation from the requested roll**, not the maximum absolute roll. Documented in the doc and in the field's comment; for landscape the two are the same.
- **`pose.py`:**
  - `level_rotation(pan, tilt, roll=0.0) = Rz(pan)·R_level0·Rx(tilt)·Rz(roll)`; the last factor rolls about the camera's own optical axis;
  - new `roll_deviation_deg(R, target_roll)` = wrapped `roll_error_deg(R) − target`, signed and wrapped to (−180, 180];
  - new `rotation_angle_deg(Ra, Rb)`.

  `camera_angles` is unchanged (it returns the absolute roll).
- **`level.py`:** `solve_wrist(fk, joints, pan, tilt, wrist_guess=None, roll=0.0)`. Its residual's first term becomes `_wrap(r − roll)`. `level_keyframe(..., roll=0.0)` likewise. The solver's contract is unchanged: it raises `LevelError` only when the orientation residual can't be met. A singular wrist can satisfy the residual exactly, so **rejecting singular results stays the validator's job** (`check_wrist_singularity`), not the solver's.
- **`validate.py`:**
  - `check_roll(poses, limit, target_roll)` measures `roll_deviation_deg`; failure messages name the target and the sample;
  - `level_joint_path` passes `design.camera_roll_deg` to `solve_wrist`;
  - `check_keyframe_ik` builds keyframe poses with the roll;
  - the cartesian path uses `path.to_sdk_pose(sample, roll_deg)` (default 0, other callers unchanged), checks roll against the target, and follows the artifact boundary above;
  - `check_fk_roundtrip` compares rotations;
  - `check_wrist_singularity` is unchanged and is the gate for singular wrists.
- **`motionctl`:**
  - `new --format portrait` writes `format: portrait`;
  - `capture` levels to the design's roll and prints the optical-centre displacement (`moved_mm`);
  - `pose` prints pan, tilt and the roll relative to a design's format when one is given (`pose NAME`), otherwise both "landscape roll" and "portrait roll";
  - `_check_same_kinematics` compares rotations;
  - `compile` is unchanged (it copies `validated.json`);
  - `publish` writes `format` from `validated.json`.
- **`motionctl derive NEW --from NAME --format portrait --wrist J4OFF J5OFF`** makes an **unvalidated draft** from a joint-mode design. The J4/J5 offsets are an operator-supplied **starting guess** (as on the pendant), not a kinematic rule:
  1. Resolve each source keyframe's joints with **the same rules as validation** (`check_keyframe_ik`: `joints` as given; pose keyframes via IK seeded by `seed`, which is an IK hint, not necessarily the solution).
  2. Add the J4/J5 offsets as the wrist guess, then run `level_keyframe(..., roll=90)` from it. It may move all three wrist joints; J1–J3 stay.
  3. FK the result, convert the position and pan **back into the source authoring frame** (the anchor; reuse the conversion `capture` already does) so the anchor isn't applied twice, and write `pos`/`pan`/`tilt` + `seed`.
  4. Keep `t`, `stop`, `ramp_fraction`, `limits`, `workspace` and `anchor`; set `format: portrait`; set `seed_joints` to the first derived configuration.
  5. Print per keyframe the optical-centre displacement and the J5 singularity margin.

  The draft must then pass `validate` (full trajectory) and `fit` like any design. It's **allowed to fail** validation, for example when the moved camera path leaves the original workspace box.
- **App:** nothing changes in `MotionRunner` (same tool, same playback). The livestream motion buttons mark portrait motions (from `motion.json` `format`), for example a suffix "(portrait)".
- **Out of scope, still a prerequisite:** issue #7 (protecting against the operator changing the active tool) stays a separate fix. Keeping the same tool avoids a new tool frame but doesn't solve it.

## Risks to check early
- **Wrist singularity.** A level landscape pose has J5 near −90 (`up_down_horizontal` starts at J5 = −88.8). A portrait solution may land near J5 = 0/±180, which `check_wrist_singularity` rejects. Phase 0 H1 measures it on a real portrait pose. If it's near-singular, keyframes need a different arm configuration (re-capture): a design finding, not a code change.
- **Reach and composition.** Re-leveling moves the optical centre (about 400 mm mount offset); workspace boxes may need widening and framing will differ. `derive` and `capture` report the displacement.

## Phases
**Gate as before:** `uv run pytest sw/tests/ -q` green, ruff clean on changed files. Hardware steps are run by the operator; read-only kinematics calls are allowed.

### Phase 0: portrait facts on the robot (operator)
- **H1 (read-only):** jog to a portrait pose by turning J4 and J5 about 90° from a landscape keyframe. Read joints and pose (`sw/read_tool1_state.py`). Record:
  - J5 and its singularity margin;
  - the SDK Euler angles (expect ry ≈ ±90);
  - the GoPro preview is portrait the right way up.
- **H2 (pendant evidence):** record with the pendant a short portrait move that **crosses tilt 0 while pan changes**. Check offline that the recorded file **actually contains the rx/rz discontinuity** (the parser `tpd_file.read` plus a small script). Play it from Python with production smoothing (`MoveTPD` blend 1).
- **H3 (generated file, the real test):** after Phase 1–2 code exists, generate a short portrait cartesian and a joint-mode motion that cross tilt 0 with changing pan, through the FK-column pipeline. `play` them at low then full `--ovl` with blend 1.
- **Acceptance for H2/H3:** no controller error, **and** the camera orientation is physically continuous, with no unexpected wrist or joint turn (watch J4/J6 in the joint log or on the pendant, and the GoPro preview). Passing validates the tested cases, not every near-singular trajectory. Record all of it in § History; if H2 or H3 fails, stop and redesign the pose-column handling.

### Phase 1: pure maths, schema, validator (fakes only)
- `pose.py`, `level.py`, `schema.py` (`format`, `CAMERA_ROLL_DEG`, `MotionConfig.format` + serializer), and the `validate.py` changes, including the cartesian artifact boundary and `format` in `validated.json`.

### Phase 2: CLI
- `new --format`, `capture` with roll and displacement, `pose NAME`, `_check_same_kinematics` by rotation, `publish` writing `format` from `validated.json`, compile/publish refusing a tool or format mismatch, `derive`.

### Phase 3: app label
- Portrait suffix on the motion button.

### Phase 4: first portrait motion (operator, after H3 passes)
- `derive up_down_portrait --from up_down_horizontal --format portrait --wrist <J4> <J5>` (offsets from H1), or capture keyframes directly. Then `validate`/`fit`/`compile` with `--ip 192.168.58.2`. Widen the workspace or re-capture if validation rejects the moved path.
- `play` at increasing `--ovl`, then `publish`. In the GoPro preview the image is portrait the right way up and the horizon stays level through the motion.
- In the app: a landscape and a portrait motion both play.

## Unit tests (binding; no robot, no network)
Fixtures:
- **`WristFK` fake** (`sw/tests/fakes/fake_kinematics.py`, or extend `FakeKinematics`): a small analytic arm (J1–J3 set the wrist centre, J4–J6 a ZYZ-like wrist times a camera mount offset) whose `fk` returns SDK Euler via `rotation_to_sdk_euler` and whose `ik` inverts it. The fake exercises the contracts (roll targets, singularity, Euler representation); **it does not prove the real mount supports any particular J4/J5 maneuver.** Reuse `test_robot_level.py`'s helpers (`_joints_for`, `stuck_fk`) where possible.
- **Representation fake:** an FK wrapper that returns a *different Euler triple for the same rotation* near ry = 90 (for example by re-encoding through scipy with rx/rz redistributed), to test rotation-based comparisons.

**`test_robot_pose.py`**
- `level_rotation(0, 0, 90)` columns: x = (0,0,−1), y = (0,1,0), z = (1,0,0).
- For 1,000 random (pan, |tilt| ≤ 80) with roll 90:
  - `roll_error_deg == 90` (±1e-9);
  - image-y is horizontal (`y_cam · ẑ == 0`) and image-x points downward-projected (`x_cam · ẑ < 0`);
  - the optical axis equals the roll-0 one;
  - the matrix is orthonormal with det = 1.
- `level_rotation(p, t, 0)` equals the old two-argument result (regression).
- `roll_deviation_deg`: 0 at target; roll 179 vs target −179 → **−2°** (signed, wrapped); roll −179 vs target 179 → +2°.
- `rotation_angle_deg`: identity 0; Rz(37°) → 37; symmetric; a portrait rotation → Euler → rotation round trip gives < 1e-9 even when scipy warns gimbal lock (warning suppressed in the test).
- `camera_angles` of portrait SDK poses near ry = 90 (tilt 0, ±5, ±30) returns the right pan/tilt and roll +90.

**`test_robot_level.py`**
- `solve_wrist(..., roll=90)` reaches roll 90 ± 1e-3 with the requested pan/tilt, keeping J1–J3 exactly; it works from a landscape guess and from an offset guess.
- `roll=0` from a portrait guess returns to landscape.
- Wrapping: target 90 from a pose at roll −270 counts as solved; a result at roll −90 does not satisfy target 90.
- `level_keyframe(..., roll=90)` keeps the viewing direction (pan/tilt within 1e-3), reports `roll_before`, and reports a non-zero `moved_mm` with an offset mount.
- **No new solver contract:** near J5 ≈ 0 the solver may succeed; the test asserts only that *if* it returns, the residual is met, and that `check_wrist_singularity` rejects the result (see validate tests).

**`test_robot_schema.py`**
- `format` defaults to landscape; `portrait` → `camera_roll_deg == 90`; an unknown value raises `SchemaError(field="format")`; accepted in both interpolation modes.
- `MotionConfig`:
  - without `format` it loads as landscape;
  - with `format: portrait` it **round-trips through `motion_config_to_dict`** (the serializer addition);
  - an unknown format fails `check_consistent()`.
- The committed `up_down_horizontal/motion.json` still loads (regression).

**`test_robot_validate.py`**
- `check_roll(poses, limit, target_roll=90)` passes for portrait poses and fails for landscape poses, naming target and sample; with target 0, the existing tests are unchanged.
- **Portrait joint-mode design on `WristFK`** passes `_validate_joint` end to end, tested on the actual requirements:
  - roll within `max_roll_deg` of 90 at every sample;
  - J1–J3 equal the joint blend;
  - joints within soft limits;
  - no joint step above `max_joint_step_deg` (continuity);
  - J5 margin above the limit;
  - `peaks.max_roll_deg` is the deviation from 90.

  (No test that J4/J5 differ by about 90° from landscape.)
- **Portrait cartesian design:**
  - `validated.json` pose columns equal FK of the solved joints;
  - the round trip compares FK against the *planned* poses (a 0.5° injected FK rotation error fails; a different Euler triple for the same rotation passes);
  - every sample has roll 90 by FK.
- A portrait design whose wrist solution lands within the J5 margin is rejected by `check_wrist_singularity`.
- `validated.json` contains `format` and `tool_id`.

**`test_robot_path.py`**
- `to_sdk_pose(sample, roll_deg=90)` has roll 90; the default is unchanged for every existing caller.

**`test_robot_motionctl.py`**
- `compile` writes exactly the pose columns stored in `validated.json` and makes **no controller calls** (FakeRobotClient call log empty for compile).
- `compile`/`publish` refuse when `validated.json`'s `format` or `tool_id` doesn't match the design.
- `publish` writes `format` from `validated.json` into `motion.json`, and it survives reload.
- `new --format portrait` writes `format: portrait`.
- `capture` on a portrait design levels to roll 90 and prints the displacement; a landscape design is unchanged (regression).
- `pose NAME` prints roll relative to the design format.
- `_check_same_kinematics` accepts equal rotations with different Euler triples, and refuses 2 mm or 0.2° off.
- `derive`:
  - resolves source joints like validation (a `joints` keyframe as given; a pose keyframe via IK from its seed);
  - applies the `--wrist` guess and levels to roll 90 with J1–J3 unchanged;
  - with a **non-identity anchor**, the derived `pos`/`pan` are in the authoring frame (re-loading and converting to base gives the FK pose, the anchor is applied once);
  - keeps `t`, `stop`, `ramp_fraction`, limits, workspace and anchor; sets `format: portrait` and `seed_joints` = the first derived configuration;
  - refuses cartesian sources, an existing target name, and a missing `--wrist`;
  - the output is a draft: **not** required to validate.

**`test_motion_runner.py`**
- A portrait `motion.json` loads, checks and runs exactly like a landscape one (same calls: `MoveJ` with tool 1, `MoveTPD` with blend 1).

**`test_livestream_robot.py`**
- A portrait motion's button shows the portrait suffix; a landscape one doesn't.

## Critical files
- Modify:
  - `sw/robot/pose.py`, `level.py`, `schema.py` (`format`, serializer), `validate.py` (roll target, cartesian FK artifact, `format` in validated), `path.py` (`to_sdk_pose` roll argument), `motionctl.py` (new/capture/pose/derive/publish/`_check_same_kinematics`, mismatch refusals);
  - `sw/livestream.py` (label);
  - `sw/tests/fakes/fake_kinematics.py`;
  - the matching tests.
- Unchanged: `fr5.py`, `motion_runner.py`, `joint_path.py`, `tpd_file.py`, the `compile_motion` serialization, the controller's tool table.
- New: this doc. The main doc `fairino-python-basic-control.md` gets D11, the artifact-boundary rule (D6 updated: cartesian pose columns now FK too), and a pointer to the new doc.

## Verification
1. Gate green after every phase.
2. Phase 0: H1 recorded; H2 shows the discontinuity is present in the recording and plays continuously; H3 (generated files, blend 1) plays continuously with no unexpected turns. H3 passing is a precondition for Phase 4.
3. Phase 4: the portrait motion validates (roll within `max_roll_deg` of +90° at every sample), plays without "TPD command point error", and the GoPro preview shows portrait, right way up, level horizon. Landscape motions validate and play unchanged.

## History (facts, not instructions)
- **Phase 1 done (2026-11-25).** `pose` (roll in `level_rotation`, `roll_deviation_deg`, `rotation_angle_deg`, `pose_rotation_diff_deg`; the scipy gimbal-lock warning silenced), `level` (`roll` in `solve_wrist`/`level_keyframe`), `schema` (`format`, `CAMERA_ROLL_DEG`, `MotionConfig.format` + serializer), `path.to_sdk_pose(sample, roll_deg)`, `validate` (roll target in IK/keyframes/wrist solve/`check_roll`; `check_fk_roundtrip` on every sample by rotation and returning FK poses; cartesian pose columns = FK; `format` in `validated.json`). Test fixture `sw/tests/fakes/wrist_kinematics.py` (`WristFK`: spherical wrist, mount offset, a mount that makes portrait singular, alternate Euler triples). Phase 0 (operator) still pending.
- **Phases 2 and 3 done (2026-11-25).** `motionctl`: `new --format`; `capture` levels to the design's roll and prints displacement and J5 margin; `pose [NAME]` (roll error vs the design's format, or both landscape and portrait errors); `derive NEW --from NAME --format F --wrist J4OFF J5OFF` (unvalidated draft; source joints resolved like validation; anchor converted back; `t`/`stop`/`ramp_fraction`/limits/workspace/anchor kept; `seed_joints` = first derived keyframe; pans unwrapped); `compile`/`publish` refuse a `validated.json` whose format or tool differs from the design; `publish` writes `format` from `validated.json`; `list` shows the format; `_check_same_kinematics` compares rotations. App: portrait motions are labelled "NAME (portrait)". `MotionRunner` unchanged (test: a portrait motion runs with the same calls). Remaining: Phase 0 H1/H2 and H3, then Phase 4 (operator).
- **First portrait motions, on the simulator (2026-11-25).** `up_down_portrait` (1.5 s, J2/J4 swing): the portrait wrist sits at J5 = 0 for the whole move, where the per-sample wrist solve breaks down, but with J5 = 0 exactly the joint blend holds roll at 90.000°. New design option `level_wrist: false` (joint mode only): J4–J6 follow the joint blend; `check_roll` still verifies roll on FK at every sample; the singularity check is skipped. `br2tl_portrait` (1.5 s): the leveled path crosses J5 = 0 (closest 0.47°) with calm J4/J6 (57 and 24 deg/s), so the design sets `limits.min_wrist_margin_deg: 0`; speed/acceleration/step checks still apply. Both still need `validate`/`compile` with `--ip 192.168.58.2` before playing on the real robot. `capture` now warns about joint-mode keyframes that did not come from capture (a template leftover caused the first br2tl_portrait failure).
