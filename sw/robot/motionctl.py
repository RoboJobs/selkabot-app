"""CLI for designing, validating, compiling and running robot motions.

Usage: `uv run python -m sw.robot.motionctl <cmd> ...`. See `--help` on each
command. Defaults to the simulator (192.168.50.2); `upload`, `play`,
`publish` and `list` default to the real robot (192.168.58.2), and `play` is
meant to be run by the operator with the cell clear and the e-stop in hand.
Motions are played as controller Lua programs (ServoJ over the joint rows);
MoveTPD is not used. See the design doc's "Read this first"
and "Simulator vs real robot" sections before changing this file.
"""

from __future__ import annotations

import argparse
import copy
import math
import sys
import time
from pathlib import Path

import yaml

from sw.robot import lua_program, schema, templates, tpd_file, validate
from sw.robot import path as path_module
from sw.robot.fr5 import FR5, FairinoError, FileTransferError
from sw.robot.frames import base_to_authoring
from sw.robot.level import LevelError, level_keyframe
from sw.robot.pose import (
    camera_angles,
    pose_rotation_diff_deg,
    unwrap_deg,
    wrap_deg,
)

SIM_IP = "192.168.50.2"
ROBOT_IP = "192.168.58.2"
MOTIONS_DIR = Path(__file__).resolve().parent / "motions"
BUILD_DIR = Path(__file__).resolve().parent / "build"
MOVE_TO_START_SPEED_PCT = 25
# The controller's actual speed is max x global speed (SetSpeed) x the
# command's own speed (MoveJ vel) -- manual 1.3.9.5. We use the
# global speed as the one speed control and give commands 100%.
COMMAND_PCT = 100


class CommandError(Exception):
    """A motionctl command failed; the CLI prints this and exits 1."""


def _motion_dir(name: str) -> Path:
    return MOTIONS_DIR / name


def _design_path(name: str) -> Path:
    return _motion_dir(name) / "design.yaml"


def _validated_path(name: str) -> Path:
    return _motion_dir(name) / "validated.json"


def _motion_json_path(name: str) -> Path:
    return _motion_dir(name) / "motion.json"


def _build_path(name: str) -> Path:
    return BUILD_DIR / f"motion_{name}.txt"


def _lua_build_path(name: str) -> Path:
    return BUILD_DIR / f"motion_{name}.lua"


def load_design(name: str) -> schema.Design:
    path = _design_path(name)
    if not path.exists():
        raise CommandError(f"no design.yaml for {name!r} (run 'new' first): {path}")
    return schema.load_design(path)


# -- new ----------------------------------------------------------------


def cmd_new(args: argparse.Namespace) -> None:
    folder = _motion_dir(args.name)
    if folder.exists():
        raise CommandError(f"{folder} already exists")
    folder.mkdir(parents=True)
    design = templates.new_design(args.template, args.name)
    if args.tool is not None:
        design["tool_id"] = args.tool
    if args.format:
        design = {"name": design.pop("name"), "format": args.format, **design}
    if args.interpolation:
        design = {"name": design.pop("name"), "interpolation": args.interpolation,
                  **design}  # fmt: skip
    (folder / "design.yaml").write_text(yaml.safe_dump(design, sort_keys=False))
    print(f"Wrote {folder / 'design.yaml'} from template '{args.template}'.")
    print("Edit its keyframes and workspace before validating.")


# -- pose -----------------------------------------------------------------


def _r(values, digits=4):
    return [round(float(v), digits) for v in values]


def _design_roll(raw: dict) -> float:
    """The roll a raw design.yaml asks for (its 'format'; default landscape)."""
    fmt = raw.get("format", schema.DEFAULT_FORMAT)
    if fmt not in schema.CAMERA_ROLL_DEG:
        raise CommandError(
            f"format {fmt!r} must be one of {tuple(schema.CAMERA_ROLL_DEG)}"
        )
    return schema.CAMERA_ROLL_DEG[fmt]


def cmd_pose(args: argparse.Namespace) -> None:
    """Show the robot's current camera pose and what leveling would change.
    With a design NAME, roll is shown relative to that design's format."""
    target = None
    if args.name:
        load_design(args.name)  # a clear error if it does not exist
        target = _design_roll(yaml.safe_load(_design_path(args.name).read_text()))
    robot = FR5.connect(args.ip)
    try:
        joints = robot.joints()
        tool = robot.active_tool()
        pose = robot.fk(joints)
        try:
            leveled = level_keyframe(robot.fk, joints, roll=target or 0.0)
        except LevelError as exc:
            leveled = exc
    finally:
        robot.close()
    pan, tilt, roll = camera_angles(*pose[3:])
    print("Joints (deg):", _r(joints))
    print(
        f"Active tool: {tool}"
        + ("  (tool 0 is the flange, not the camera)" if tool == 0 else "")
    )
    print("Pose [x,y,z,rx,ry,rz] (mm, deg):", _r(pose))
    print(f"Camera: pan {pan:.3f}, tilt {tilt:.3f}, roll {roll:.3f}")
    if target is None:
        print(
            f"  landscape roll error {wrap_deg(roll):.3f}deg, portrait roll error "
            f"{wrap_deg(roll - 90.0):.3f}deg"
        )
    else:
        print(
            f"  {args.name} wants roll {target:g}: off by "
            f"{wrap_deg(roll - target):.3f}deg"
        )
    if isinstance(leveled, LevelError):
        print(f"Cannot level here: {leveled}")
    else:
        print(
            f"Leveled by the wrist to roll {target or 0.0:g}: joints "
            f"{_r(leveled.joints)}, camera moves {leveled.moved_mm:.1f}mm, "
            f"J5 margin {_j5_margin(leveled.joints):.1f}deg"
        )
    print("'motionctl capture NAME --t T' writes this pose (leveled) as a keyframe.")


def _j5_margin(joints: list[float]) -> float:
    """Distance (deg) of J5 from the wrist singularity at 0/180."""
    return math.degrees(math.asin(min(1.0, abs(math.sin(math.radians(joints[4]))))))


# -- capture ---------------------------------------------------------------


def cmd_capture(args: argparse.Namespace) -> None:
    """Write the robot's current pose (or --joints) as a leveled keyframe.

    Joints are read, not the pendant's pose readout: they do not depend on the
    active tool, and FK with the design's tool gives the exact camera pose.
    """
    path = _design_path(args.name)
    if not path.exists():
        raise CommandError(f"no design.yaml for {args.name!r} (run 'new' first)")
    if args.t < 0:
        raise CommandError("--t must be >= 0")
    raw = yaml.safe_load(path.read_text())
    roll = _design_roll(raw)
    robot = FR5.connect(args.ip)
    try:
        tool = robot.active_tool()
        if tool != raw.get("tool_id"):
            raise CommandError(
                f"the controller's active tool is {tool} but the design uses "
                f"{raw.get('tool_id')}; apply the camera tool on the pendant first"
            )
        joints = list(args.joints) if args.joints else robot.joints()
        if args.no_level:
            pose = robot.fk(joints)
            roll_before = wrap_deg(camera_angles(*pose[3:])[2] - roll)
            moved = 0.0
        else:
            try:
                result = level_keyframe(robot.fk, joints, roll=roll)
            except LevelError as exc:
                raise CommandError(f"cannot level this keyframe: {exc}") from exc
            joints, pose = result.joints, result.pose
            roll_before = wrap_deg(result.roll_before - roll)
            moved = result.moved_mm
    finally:
        robot.close()

    pan, tilt, _roll = camera_angles(*pose[3:])
    anchor_raw = raw.get("anchor") or {}
    anchor_obj = schema.Anchor(
        tuple(anchor_raw.get("pos", (0.0, 0.0, 0.0))),
        float(anchor_raw.get("yaw_deg", 0.0)),
    )
    pos_a = base_to_authoring(pose[:3], anchor_obj)
    kf = {
        "t": float(args.t),
        "pos": _r(pos_a, 3),
        "pan": round(pan - anchor_obj.yaw_deg, 4),
        "tilt": round(tilt, 4),
    }
    if raw.get("interpolation") == "joint":
        kf["seed"] = _r(joints)
    keyframes = [] if args.reset else list(raw.get("keyframes") or [])
    keyframes = [k for k in keyframes if abs(float(k["t"]) - args.t) > 1e-9]
    keyframes.append(kf)
    keyframes.sort(key=lambda k: float(k["t"]))
    raw["keyframes"] = keyframes
    if args.reset or args.t == 0:
        raw["seed_joints"] = _r(joints)
    path.write_text(yaml.safe_dump(raw, sort_keys=False))
    print(f"Keyframe t={args.t}: pos {kf['pos']}, pan {kf['pan']}, tilt {kf['tilt']}")
    if args.no_level:
        print(
            f"Not leveled (roll {roll_before:.3f}deg is dropped when the keyframe "
            "is solved; near full reach that can make it unreachable)."
        )
    else:
        print(
            f"Leveled by the wrist to roll {roll:g}: removed {roll_before:.3f}deg "
            f"of roll error, camera moved {moved:.1f}mm. Joints {_r(joints)}, "
            f"J5 margin {_j5_margin(joints):.1f}deg"
        )
    print(f"{len(keyframes)} keyframe(s) in {path}")
    if raw.get("interpolation") == "joint":
        foreign = [k["t"] for k in keyframes if "seed" not in k and "joints" not in k]
        if foreign:
            print(
                "Warning: keyframe(s) at t="
                + ", ".join(f"{float(t):g}" for t in foreign)
                + " did not come from capture (left from the template?). Remove "
                "them from design.yaml, or capture the first keyframe with --reset."
            )


# -- derive ----------------------------------------------------------------


def cmd_derive(args: argparse.Namespace) -> None:
    """Make an unvalidated draft in another format from a joint-mode design.

    Each source keyframe's joints are resolved like validation does; the
    --wrist J4/J5 offsets are added (the operator's maneuver, a starting
    guess), and the wrist re-levels the camera to the new format's roll
    keeping where it then points (like 'capture'). Read-only robot calls.
    """
    if _motion_dir(args.name).exists():
        raise CommandError(f"{_motion_dir(args.name)} already exists")
    source = load_design(args.source)
    if source.interpolation != "joint":
        raise CommandError(
            f"{args.source} uses interpolation '{source.interpolation}'; derive "
            "needs a joint-mode design (its keyframes resolve to joints)"
        )
    raw = yaml.safe_load(_design_path(args.source).read_text())
    roll = schema.CAMERA_ROLL_DEG[args.format]
    robot = FR5.connect(args.ip)
    try:
        tool = robot.active_tool()
        if tool != source.tool_id:
            raise CommandError(
                f"the controller's active tool is {tool} but {args.source} uses "
                f"{source.tool_id}"
            )
        result, kf_joints = validate.check_keyframe_ik(source, robot)
        if not result.passed:
            raise CommandError(
                f"cannot resolve {args.source}'s keyframes: {result.detail}"
            )
        derived = []
        for i, (kf_raw, joints) in enumerate(
            zip(raw["keyframes"], kf_joints, strict=True)
        ):
            moved = [*joints[:3], joints[3] + args.wrist[0], joints[4] + args.wrist[1],
                     joints[5]]  # fmt: skip
            try:
                lv = level_keyframe(robot.fk, moved, roll=roll)
            except LevelError as exc:
                raise CommandError(f"keyframe {i}: cannot level: {exc}") from exc
            shift = math.dist(robot.fk(joints)[:3], lv.pose[:3])
            derived.append((kf_raw, lv, shift))
    finally:
        robot.close()

    pans = unwrap_deg([camera_angles(*lv.pose[3:])[0] for _, lv, _ in derived])
    keyframes = []
    for (kf_raw, lv, shift), pan in zip(derived, pans, strict=True):
        tilt = camera_angles(*lv.pose[3:])[1]
        kf = {
            "t": float(kf_raw["t"]),
            "pos": _r(base_to_authoring(lv.pose[:3], source.anchor), 3),
            "pan": round(pan - source.anchor.yaw_deg, 4),
            "tilt": round(tilt, 4),
            "seed": _r(lv.joints),
        }
        if kf_raw.get("stop"):
            kf["stop"] = True
        keyframes.append(kf)
        print(
            f"t={kf['t']}: camera moved {shift:.1f}mm, pan {kf['pan']}, tilt "
            f"{kf['tilt']}, J5 margin {_j5_margin(lv.joints):.1f}deg"
        )
    out = copy.deepcopy(raw)
    out.pop("format", None)
    out = {"name": args.name, "format": args.format,
           **{k: v for k, v in out.items() if k != "name"}}  # fmt: skip
    out["keyframes"] = keyframes
    out["seed_joints"] = keyframes[0]["seed"]
    folder = _motion_dir(args.name)
    folder.mkdir(parents=True)
    (folder / "design.yaml").write_text(yaml.safe_dump(out, sort_keys=False))
    print(
        f"Wrote draft {folder / 'design.yaml'} ({args.format}). It is not "
        f"validated: run 'validate {args.name}' (widen the workspace or "
        "re-capture keyframes if the moved camera path fails)."
    )


# -- plan -----------------------------------------------------------------


def cmd_plan(args: argparse.Namespace) -> None:
    design = load_design(args.name)
    if design.interpolation == "joint":
        count = path_module.sample_count(
            design.keyframes[-1].t, design.sample_period_ms
        )
        print(f"Design: {design.name}")
        print(f"Interpolation: joint, tool_id: {design.tool_id}")
        print(
            f"Keyframes: {len(design.keyframes)}, samples: {count} at "
            f"{design.sample_period_ms}ms, duration {design.keyframes[-1].t:.3f}s"
        )
        print("Camera path and speeds come from the robot's FK: run 'validate'.")
        return
    samples = path_module.sample_path(design)
    dt = design.sample_period_ms / 1000.0
    import numpy as np

    positions = np.array([s.pos for s in samples])
    peak_speed = 0.0
    if len(positions) >= 2:
        diffs = np.diff(positions, axis=0)
        peak_speed = float(np.linalg.norm(diffs, axis=1).max() / dt)
    duration = (len(samples) - 1) * dt
    print(f"Design: {design.name}")
    print(f"Mode: {design.orientation.mode}, tool_id: {design.tool_id}")
    print(f"Samples: {len(samples)} at {design.sample_period_ms}ms")
    print(f"Duration: {duration:.3f}s (design: {design.keyframes[-1].t:.3f}s)")
    print(f"Peak TCP speed: {peak_speed:.1f}mm/s")


# -- validate --------------------------------------------------------------


def _run_validation(name: str, ip: str):
    """Validate a design against the robot's kinematics (no motion)."""
    design = load_design(name)
    samples = path_module.sample_path(design) if design.interpolation != "joint" else []
    robot = FR5.connect(ip)
    try:
        report = validate.validate(design, samples, robot)
        install_angle = robot.install_angle()
    finally:
        robot.close()
    return design, samples, report, install_angle


# Checks that only depend on how fast the path is run, which `fit` can fix.
TIMING_CHECKS = {"check_tcp_speed", "check_joint_steps", "check_joint_speed_acc"}


def _print_report(design, report) -> None:
    for check in report.checks:
        status = "OK" if check.passed else "FAIL"
        detail = f": {check.detail}" if check.detail else ""
        print(f"[{status}] {check.name}{detail}")
    if report.joints:
        pk = report.peaks
        duration = design.keyframes[-1].t
        k = validate.min_time_scale(design, pk)
        print(
            f"Peaks: TCP {pk.tcp_speed_mm_s:.0f}mm/s, joint speed "
            f"{max(pk.joint_speed_deg_s):.1f}deg/s, joint acc "
            f"{max(pk.joint_acc_deg_s2):.1f}deg/s^2"
        )
        print(
            f"Fastest duration within the limits: about {duration * k:.2f}s "
            f"(now {duration:.2f}s). 'motionctl fit {design.name} --write' rescales."
        )


def cmd_validate(args: argparse.Namespace) -> None:
    design, samples, report, install_angle = _run_validation(args.name, args.ip)
    _print_report(design, report)

    if not report.passed:
        raise CommandError(f"validation failed for {args.name!r}")

    data = validate.report_to_validated(report, design, samples)
    data["design_sha256"] = schema.sha256_file(_design_path(args.name))
    data["install_angle"] = list(install_angle)
    _motion_dir(args.name).mkdir(parents=True, exist_ok=True)
    _validated_path(args.name).write_text(_dumps(data))
    print(f"Wrote {_validated_path(args.name)}")


def _dumps(data: dict) -> str:
    import json

    return json.dumps(data, indent=2)


# -- fit -------------------------------------------------------------------


def cmd_fit(args: argparse.Namespace) -> None:
    """Find the uniform time scale that fits the limits; optionally apply it."""
    design, _samples, report, _angle = _run_validation(args.name, args.ip)
    blocking = [
        c for c in report.checks if not c.passed and c.name not in TIMING_CHECKS
    ]
    if blocking or not report.joints:
        _print_report(design, report)
        raise CommandError(
            "cannot fit the timing: fix the non-timing failures first "
            f"({', '.join(c.name for c in blocking) or 'no joint solution'})"
        )
    k = validate.min_time_scale(design, report.peaks) * args.margin
    old = design.keyframes[-1].t
    new = math.ceil(old * k * 100.0) / 100.0  # round up: never below the fit
    print(
        f"Current duration {old:.2f}s; fitted duration {new:.2f}s "
        f"(scale {k:.3f}, margin {args.margin})."
    )
    if not args.write:
        print("Re-run with --write to rescale the keyframe times in design.yaml.")
        return
    path = _design_path(args.name)
    raw = yaml.safe_load(path.read_text())
    scale = new / old
    for kf in raw["keyframes"]:
        kf["t"] = round(float(kf["t"]) * scale, 4)
        for channel, value in (kf.get("vel") or {}).items():
            kf["vel"][channel] = value / scale  # velocities scale as 1/time
    path.write_text(yaml.safe_dump(raw, sort_keys=False))
    print(f"Rewrote {path}. Run 'validate {args.name}' again to confirm.")


def _load_validated(name: str) -> dict:
    import json

    path = _validated_path(name)
    if not path.exists():
        raise CommandError(f"no validated.json for {name!r} (run 'validate' first)")
    return json.loads(path.read_text())


def _check_validated_current(name: str, validated: dict) -> None:
    current = schema.sha256_file(_design_path(name))
    if validated.get("design_sha256") != current:
        raise CommandError(
            f"validated.json is stale for {name!r} (design.yaml changed); "
            "re-run 'validate'"
        )
    design = load_design(name)
    fmt = validated.get("format", schema.DEFAULT_FORMAT)
    if fmt != design.format or validated.get("tool_id") != design.tool_id:
        raise CommandError(
            f"validated.json ({fmt}, tool {validated.get('tool_id')}) does not "
            f"match design.yaml ({design.format}, tool {design.tool_id}); "
            "re-run 'validate'"
        )


# -- compile -----------------------------------------------------------


def compile_motion(name: str) -> Path:
    """Write the TPD build file. Raises CommandError if validated.json is
    missing or stale. Returns the build file's path."""
    validated = _load_validated(name)
    _check_validated_current(name, validated)
    samples = validated["samples"]
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    out = _build_path(name)
    tpd_file.write(
        out,
        joints=samples["joints"],
        poses=samples["pose"],
        period_ms=validated["sample_period_ms"],
        tool_id=validated["tool_id"],
    )
    return out


def cmd_compile(args: argparse.Namespace) -> None:
    out = compile_motion(args.name)
    print(f"Wrote {out}")


def _ensure_compiled(name: str) -> Path:
    out = _build_path(name)
    if not out.exists():
        print(f"{out} missing; compiling first.")
        out = compile_motion(name)
    return out


# -- simplay (D9: simulator preview via ServoJ, not the production path) ----


def cmd_simplay(args: argparse.Namespace) -> None:
    out = _ensure_compiled(args.name)
    validated = _load_validated(args.name)
    tpd = tpd_file.read(out)
    start_joints = tpd.joints[0]
    period_s = validated["sample_period_ms"] / 1000.0

    robot = FR5.connect(args.ip)
    try:
        print(f"Moving to start pose at {MOVE_TO_START_SPEED_PCT}% speed...")
        with robot.speed(MOVE_TO_START_SPEED_PCT):
            robot.move_j(start_joints, validated["tool_id"], COMMAND_PCT)
        print(
            f"Streaming {len(tpd.joints)} rows at {period_s * 1000:.0f}ms "
            f"(ovl {args.ovl}; preview only, not the production path)..."
        )
        robot.stream_joints(tpd.joints, period_s, ovl=args.ovl)
        print("Done.")
    except KeyboardInterrupt:
        print("\nInterrupted; stopping.")
        robot.stop()
    finally:
        robot.close()


# -- controller programs (real robot only) ----------------------------------


def _refuse_sim(command: str, ip: str) -> None:
    if ip == SIM_IP:
        raise CommandError(
            f"{command} needs the real robot (--ip {ROBOT_IP}); the simulator "
            "does not store uploaded programs"
        )


LUA_UPLOAD_TYPE = 0  # FileUpload type for Lua programs (stored in /fruser/)


KINEMATICS_TOLERANCE_MM = 1.0
KINEMATICS_TOLERANCE_DEG = 0.05


def _check_same_kinematics(robot: FR5, out: Path, rows: int = 12) -> None:
    """The file's pose columns must be this robot's FK of its joints.

    A file computed with another controller's kinematics (the simulator's arm
    model differs from the real FR5 by ~28 mm) has joints and poses that
    disagree on this robot: the camera would not be where the design put it
    (with MoveTPD the controller rejected such files with "TPD command point
    error", main 1 sub 10). Read-only: FK calls only.
    """
    tpd = tpd_file.read(out)
    step = max(1, len(tpd.joints) // rows)
    for i in list(range(0, len(tpd.joints), step)) + [len(tpd.joints) - 1]:
        fk = robot.fk(tpd.joints[i])
        dpos = math.dist(fk[:3], tpd.poses[i][:3])
        # As rotations: near ry = 90 (portrait) Euler triples are not unique.
        drot = pose_rotation_diff_deg(fk, tpd.poses[i])
        if dpos > KINEMATICS_TOLERANCE_MM or drot > KINEMATICS_TOLERANCE_DEG:
            raise CommandError(
                f"{out.name} row {i}: this robot's FK of the joints is {dpos:.1f}mm "
                f"/ {drot:.3f}deg from the file's pose. The file was computed with "
                f"another controller's kinematics (the simulator?). Re-run capture, "
                f"validate and compile with --ip {robot.ip}"
            )


def _put_program(
    robot: FR5, name: str, ovl: int, pace: str = "queue"
) -> tuple[str, str, list[list[float]]]:
    """Check the compiled file matches this robot's kinematics, write the Lua
    program at `ovl` and upload it. Returns the controller path, the
    controller's MD5 of the registered program and the rows it plays."""
    out = _ensure_compiled(name)
    _check_same_kinematics(robot, out)
    lua, rows = write_lua(name, ovl, pace)
    period_ms = _load_validated(name)["sample_period_ms"]
    print(
        f"Wrote {lua} ({len(rows)} rows, ovl {ovl}, pace {pace}, about "
        f"{(len(rows) - 1) * period_ms / 1000.0:.2f}s)."
    )
    try:
        remote = robot.upload_file(str(lua), file_type=LUA_UPLOAD_TYPE)
    except FileTransferError as exc:
        raise CommandError(
            f"upload of {lua} failed at stage '{exc.stage}' ({exc.detail!r})"
        ) from exc
    md5 = robot.file_md5(remote)
    if md5 is None:
        raise CommandError(f"{remote} not found on the controller after upload")
    print(f"Uploaded to {remote} (controller MD5 {md5}).")
    return remote, md5, rows


def _check_ovl(ovl: int) -> None:
    if not 1 <= ovl <= 100:
        raise CommandError("--ovl must be 1..100")


def cmd_upload(args: argparse.Namespace) -> None:
    _refuse_sim("upload", args.ip)
    _check_ovl(args.ovl)
    robot = FR5.connect(args.ip)
    try:
        _put_program(robot, args.name, args.ovl, args.pace)
    finally:
        robot.close()


def write_lua(name: str, ovl: int, pace: str) -> tuple[Path, list[list[float]]]:
    """Write the Lua program for the compiled file at `ovl`. Returns the path
    and the rows it plays (one per sample period)."""
    out = _ensure_compiled(name)
    validated = _load_validated(name)
    rows = lua_program.resample(tpd_file.read(out).joints, ovl)
    lua = _lua_build_path(name)
    lua.write_text(lua_program.render(rows, validated["sample_period_ms"], name, pace))
    return lua, rows


# -- publish / list (real robot only for publish) ---------------------------


def cmd_publish(args: argparse.Namespace) -> None:
    """Recompile from the current validated.json, upload the program at the
    approved speed, and write motion.json (what the livestream app reads)."""
    import datetime
    import os

    _refuse_sim("publish", args.ip)
    _check_ovl(args.ovl)
    compile_motion(args.name)  # refuses a missing or stale validated.json
    validated = _load_validated(args.name)
    robot = FR5.connect(args.ip)
    try:
        remote, md5, rows = _put_program(robot, args.name, args.ovl)
    finally:
        robot.close()

    period_ms = validated["sample_period_ms"]
    raw = {
        "name": args.name,
        "backend": "lua",
        "controller_file": remote,
        "md5": md5,
        "num_points": len(rows),
        "sample_period_ms": period_ms,
        "duration_s": (len(rows) - 1) * period_ms / 1000.0,
        "design_duration_s": validated["design_duration_s"],
        "start_joints": rows[0],
        "end_joints": rows[-1],
        "start_pose": validated["samples"]["pose"][0],
        "tool_id": validated["tool_id"],
        "approved_ovl": args.ovl,
        "design_sha256": validated["design_sha256"],
        "published_at": datetime.datetime.now().isoformat(timespec="seconds"),
        "format": validated.get("format", schema.DEFAULT_FORMAT),
    }
    schema.motion_config_from_dict(raw, "motion.json")  # check_consistent()
    path = _motion_json_path(args.name)
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(_dumps(raw))
    os.replace(tmp, path)
    print(f"Published {args.name} at ovl {args.ovl}: wrote {path}")


def cmd_list(args: argparse.Namespace) -> None:
    """Published motions; with --check, also whether the controller file's
    MD5 still matches motion.json."""
    if not MOTIONS_DIR.exists():
        print("No motions.")
        return
    configs = schema.load_motion_configs(MOTIONS_DIR)
    drafts = sorted(
        d.name
        for d in MOTIONS_DIR.iterdir()
        if d.is_dir() and d.name not in configs and (d / "design.yaml").exists()
    )
    robot = FR5.connect(args.ip) if args.check and configs else None
    try:
        for name, config in configs.items():
            line = (
                f"{name}: {config.duration_s:.2f}s, ovl {config.approved_ovl}, "
                f"tool {config.tool_id}, {config.format}, "
                f"published {config.published_at}"
            )
            if robot is not None:
                remote = robot.file_md5(config.controller_file)
                if remote is None:
                    line += "  [file missing on controller]"
                elif remote.lower() != config.md5.lower():
                    line += "  [md5 mismatch]"
                else:
                    line += "  [controller file OK]"
            print(line)
    finally:
        if robot is not None:
            robot.close()
    for name in drafts:
        print(f"{name}: not published")


# -- play (real robot only; operator confirmation required) -----------------


def cmd_play(args: argparse.Namespace) -> None:
    """Upload the program at --ovl, move to the start and run it."""
    _refuse_sim("play", args.ip)
    _check_ovl(args.ovl)
    out = _ensure_compiled(args.name)
    validated = _load_validated(args.name)
    period_ms = validated["sample_period_ms"]

    robot = FR5.connect(args.ip)
    try:
        remote, _md5, rows = _put_program(robot, args.name, args.ovl, args.pace)
        expected = (len(rows) - 1) * period_ms / 1000.0
        print("Current joints:", [round(v, 2) for v in robot.joints()])
        print("Start joints:  ", [round(v, 2) for v in rows[0]])
        print("\nThe robot will MOVE to the start pose, then run the program.")
        print("Cell must be clear and the e-stop in hand.")
        if input("Type YES to continue: ").strip() != "YES":
            print("Aborted.")
            return

        robot.ensure_auto()
        with robot.speed(MOVE_TO_START_SPEED_PCT):
            robot.move_j(rows[0], validated["tool_id"], COMMAND_PCT)
        input("Press Enter to play: ")

        start = time.monotonic()
        robot.run_program(remote)
        outcome = _wait_for_program(robot, rows[-1], expected * 1.5 + 3.0)
        elapsed = time.monotonic() - start
        print(f"{outcome}. Elapsed: {elapsed:.2f}s (expected about {expected:.2f}s)")
        if not outcome.startswith("Program finished"):
            print(_progress(robot, out, period_ms, elapsed, rows))
    except KeyboardInterrupt:
        print("\nInterrupted; stopping.")
        _stop_program(robot)
    finally:
        robot.close()


def _progress(
    robot: FR5,
    out: Path,
    period_ms: int,
    elapsed_s: float,
    rows: list[list[float]] | None = None,
) -> str:
    """Where in the file (or in `rows`, the rows actually played) the robot
    stopped: the nearest row by joint distance. Its planned time over the
    elapsed time is the effective playback speed."""
    joints = robot.joints()
    if rows is None:
        rows = tpd_file.read(out).joints
    i = min(range(len(rows)), key=lambda k: math.dist(rows[k], joints))
    planned = i * period_ms / 1000.0
    rate = 100.0 * planned / elapsed_s if elapsed_s > 0 else 0.0
    return (
        f"Stopped near row {i} of {len(rows)} (planned t={planned:.2f}s): "
        f"effective playback speed about {rate:.1f}% of planned"
    )


def _stop_program(robot: FR5) -> None:
    """ProgramStop, then StopMotion; each is tried even if the other fails."""
    for stop in (robot.stop_program, robot.stop):
        try:
            stop()
        except FairinoError as exc:
            print(f"{exc} while stopping")


PROGRAM_RUNNING = 2
AT_END_DEG = 0.5  # max joint error to the last row to count as arrived
STILL_DEG = 0.01  # max joint change between two polls to count as standing
SETTLE_S = 0.3  # how long the arm must stand still


def _wait_for_program(
    robot: FR5, end_joints: list[float], deadline_s: float, poll_s: float = 0.02
) -> str:
    """Wait until the arm has moved, reached `end_joints` and stands still,
    watching the robot's error code; stop the program on an error or at the
    deadline. The program state does not decide completion (with WaitMs
    between ServoJ calls it toggled 1/2 every ~0.1 s; a motion may pause);
    it is only used to report how long the program itself ran.
    Returns a one-line outcome."""
    t0 = time.monotonic()
    prev = robot.joints()
    moved = False
    still_since: float | None = None
    ran: list[float | None] = [None, None]  # program state: first/last running

    def timing() -> str:
        if ran[0] is None or ran[1] is None:
            return ""
        return f" (program ran {ran[1] - ran[0]:.2f}s, from t={ran[0]:.2f}s)"

    while (now := time.monotonic()) - t0 < deadline_s:
        main, sub = robot.error_codes()
        if main:
            _stop_program(robot)
            return f"Robot error during the program: main {main}, sub {sub}"
        if robot.program_state() == PROGRAM_RUNNING:
            if ran[0] is None:
                ran[0] = now - t0
            ran[1] = now - t0
        joints = robot.joints()
        step = max(abs(a - b) for a, b in zip(joints, prev, strict=True))
        prev = joints
        if step >= STILL_DEG:
            moved = True
            still_since = None
        elif still_since is None:
            still_since = now
        settled = still_since is not None and now - still_since >= SETTLE_S
        at_end = (
            max(abs(a - b) for a, b in zip(joints, end_joints, strict=True))
            <= AT_END_DEG
        )
        if settled and moved and at_end:
            return "Program finished" + timing()
        time.sleep(poll_s)
    _stop_program(robot)
    return f"Program still running after {deadline_s:.1f}s; stopped it"


# -- argparse wiring ---------------------------------------------------------


def _add_ip(p: argparse.ArgumentParser, default: str = SIM_IP) -> None:
    p.add_argument("--ip", default=default)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="motionctl")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("new")
    p.add_argument("name")
    p.add_argument("--template", required=True, choices=sorted(templates.TEMPLATES))
    p.add_argument("--interpolation", choices=schema.INTERPOLATIONS)
    p.add_argument("--tool", type=int, help="tool_id (the camera is tool 1)")
    p.add_argument("--format", choices=sorted(schema.CAMERA_ROLL_DEG))
    p.set_defaults(func=cmd_new)

    p = sub.add_parser("derive", help="draft another format from a joint design")
    p.add_argument("name", help="the new motion")
    p.add_argument("--from", dest="source", required=True, help="source motion")
    p.add_argument("--format", required=True, choices=sorted(schema.CAMERA_ROLL_DEG))
    p.add_argument(
        "--wrist",
        type=float,
        nargs=2,
        required=True,
        metavar=("J4OFF", "J5OFF"),
        help="J4/J5 offsets (deg) applied before re-leveling, as on the pendant",
    )
    _add_ip(p)
    p.set_defaults(func=cmd_derive)

    p = sub.add_parser("capture")
    p.add_argument("name")
    p.add_argument("--t", type=float, required=True, help="keyframe time (s)")
    p.add_argument(
        "--joints", type=float, nargs=6, help="use these instead of the robot's"
    )
    p.add_argument("--reset", action="store_true", help="drop existing keyframes")
    p.add_argument("--no-level", action="store_true")
    _add_ip(p)
    p.set_defaults(func=cmd_capture)

    p = sub.add_parser("pose")
    p.add_argument("name", nargs="?", help="show roll relative to this design")
    _add_ip(p)
    p.set_defaults(func=cmd_pose)

    p = sub.add_parser("plan")
    p.add_argument("name")
    p.set_defaults(func=cmd_plan)

    p = sub.add_parser("validate")
    p.add_argument("name")
    _add_ip(p)
    p.set_defaults(func=cmd_validate)

    p = sub.add_parser("fit")
    p.add_argument("name")
    p.add_argument("--margin", type=float, default=1.1)
    p.add_argument("--write", action="store_true")
    _add_ip(p)
    p.set_defaults(func=cmd_fit)

    p = sub.add_parser("compile")
    p.add_argument("name")
    p.set_defaults(func=cmd_compile)

    p = sub.add_parser("simplay")
    p.add_argument("name")
    p.add_argument("--ovl", type=int, default=100)
    _add_ip(p)
    p.set_defaults(func=cmd_simplay)

    def add_pace(p: argparse.ArgumentParser) -> None:
        p.add_argument(
            "--pace",
            choices=lua_program.PACES,
            default="queue",
            help="queue: ServoJ paces itself (measured); wait: also WaitMs(period)",
        )

    p = sub.add_parser("upload", help="upload the motion's Lua program")
    p.add_argument("name")
    p.add_argument("--ovl", type=int, default=100, help="playback speed, 1..100")
    add_pace(p)
    _add_ip(p, default=ROBOT_IP)
    p.set_defaults(func=cmd_upload)

    p = sub.add_parser("publish")
    p.add_argument("name")
    p.add_argument("--ovl", type=int, required=True, help="approved speed, 1..100")
    _add_ip(p, default=ROBOT_IP)
    p.set_defaults(func=cmd_publish)

    p = sub.add_parser("list")
    p.add_argument("--check", action="store_true", help="check controller MD5s")
    _add_ip(p, default=ROBOT_IP)
    p.set_defaults(func=cmd_list)

    for cmd in ("play", "luaplay"):  # luaplay: the name used while testing
        p = sub.add_parser(cmd, help="upload the Lua program, move to start, run it")
        p.add_argument("name")
        p.add_argument("--ovl", type=int, default=50, help="playback speed, 1..100")
        add_pace(p)
        _add_ip(p, default=ROBOT_IP)
        p.set_defaults(func=cmd_play)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        args.func(args)
    except (CommandError, FairinoError, FileTransferError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
