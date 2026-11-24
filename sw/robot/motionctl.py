"""CLI for designing, validating, compiling and running robot motions.

Usage: `uv run python -m sw.robot.motionctl <cmd> ...`. See `--help` on each
command. Defaults to the simulator (192.168.50.2); `upload` and `play` need
--ip 192.168.58.2 explicitly and are meant to be run by the operator with the
cell clear and the e-stop in hand. See the design doc's "Read this first"
and "Simulator vs real robot" sections before changing this file.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import yaml

from sw.robot import path as path_module
from sw.robot import schema, templates, tpd_file, validate
from sw.robot.fr5 import FR5, FairinoError, FileTransferError
from sw.robot.frames import base_to_authoring
from sw.robot.level import LevelError, level_keyframe
from sw.robot.pose import camera_angles

SIM_IP = "192.168.50.2"
ROBOT_IP = "192.168.58.2"
MOTIONS_DIR = Path(__file__).resolve().parent / "motions"
BUILD_DIR = Path(__file__).resolve().parent / "build"
POSE_TOLERANCE = 0.01
MOVE_TO_START_SPEED_PCT = 10


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


def _controller_file(name: str) -> str:
    return f"/fruser/traj/motion_{name}.txt"


def _tpd_name(name: str) -> str:
    return f"traj/motion_{name}"


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
    if args.interpolation:
        design = {"name": design.pop("name"), "interpolation": args.interpolation,
                  **design}  # fmt: skip
    (folder / "design.yaml").write_text(yaml.safe_dump(design, sort_keys=False))
    print(f"Wrote {folder / 'design.yaml'} from template '{args.template}'.")
    print("Edit its keyframes and workspace before validating.")


# -- pose -----------------------------------------------------------------


def _r(values, digits=4):
    return [round(float(v), digits) for v in values]


def cmd_pose(args: argparse.Namespace) -> None:
    """Show the robot's current camera pose and what leveling would change."""
    robot = FR5.connect(args.ip)
    try:
        joints = robot.joints()
        tool = robot.active_tool()
        pose = robot.fk(joints)
        try:
            leveled = level_keyframe(robot.fk, joints)
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
    print(f"Camera: pan {pan:.3f}, tilt {tilt:.3f}, roll {roll:.3f} (0 = horizontal)")
    if isinstance(leveled, LevelError):
        print(f"Cannot level here: {leveled}")
    else:
        print(
            f"Leveled by the wrist: joints {_r(leveled.joints)}, camera moves "
            f"{leveled.moved_mm:.1f}mm"
        )
    print("'motionctl capture NAME --t T' writes this pose (leveled) as a keyframe.")


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
            roll_before = camera_angles(*pose[3:])[2]
            moved = 0.0
        else:
            try:
                result = level_keyframe(robot.fk, joints)
            except LevelError as exc:
                raise CommandError(f"cannot level this keyframe: {exc}") from exc
            joints, pose = result.joints, result.pose
            roll_before, moved = result.roll_before, result.moved_mm
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
            f"Leveled by the wrist: removed {roll_before:.3f}deg of roll, camera "
            f"moved {moved:.1f}mm. Joints {_r(joints)}"
        )
    print(f"{len(keyframes)} keyframe(s) in {path}")


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
            robot.move_j(start_joints, validated["tool_id"], MOVE_TO_START_SPEED_PCT)
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


# -- upload (real robot only) --------------------------------------------


def cmd_upload(args: argparse.Namespace) -> None:
    if args.ip == SIM_IP:
        raise CommandError(
            "upload needs the real robot (--ip 192.168.58.2); the simulator "
            "does not store uploaded files"
        )
    out = _ensure_compiled(args.name)
    robot = FR5.connect(args.ip)
    try:
        try:
            remote = robot.upload_file(str(out), file_type=20)
        except FileTransferError as exc:
            raise CommandError(
                f"upload failed at stage '{exc.stage}' ({exc.detail!r}). "
                f"Ask the operator to upload {out} to the controller manually "
                "(teach pendant or web UI trajectory import), then re-run "
                "this check."
            ) from exc
        print(f"Uploaded to {remote}")

        tpd_name = _tpd_name(args.name)
        robot.load_tpd(tpd_name)
        start_pose = robot.tpd_start_pose(tpd_name)
        expected = tpd_file.read(out).poses[0]
        diff = max(abs(a - b) for a, b in zip(start_pose, expected, strict=True))
        print(f"LoadTPD({tpd_name!r}) OK. Start pose matches file (diff {diff:.6f}).")
        if diff > POSE_TOLERANCE:
            raise CommandError(
                f"start pose differs from the file by {diff}, more than "
                f"the tolerance ({POSE_TOLERANCE})"
            )
    finally:
        robot.close()


# -- play (real robot only; operator confirmation required) -----------------


def cmd_play(args: argparse.Namespace) -> None:
    if args.ip == SIM_IP:
        raise CommandError(
            "play needs the real robot (--ip 192.168.58.2); MoveTPD does not "
            "play trajectories on the simulator"
        )
    out = _ensure_compiled(args.name)
    local_md5 = tpd_file.md5(out)
    remote_path = _controller_file(args.name)
    tpd_name = _tpd_name(args.name)

    robot = FR5.connect(args.ip)
    try:
        remote_md5 = robot.file_md5(remote_path)
        if remote_md5 is None or remote_md5.lower() != local_md5.lower():
            raise CommandError(
                f"controller copy of {remote_path} does not match the "
                f"compiled file; run 'upload {args.name}' first"
            )

        robot.load_tpd(tpd_name)
        start_pose = robot.tpd_start_pose(tpd_name)
        expected = tpd_file.read(out).poses[0]
        diff = max(abs(a - b) for a, b in zip(start_pose, expected, strict=True))
        if diff > POSE_TOLERANCE:
            raise CommandError(
                f"GetTPDStartPose differs from the file by {diff}; run 'upload' again"
            )

        validated = _load_validated(args.name)
        start_joints = validated["samples"]["joints"][0]
        print("Current joints:", [round(v, 2) for v in robot.joints()])
        print("Start joints:  ", [round(v, 2) for v in start_joints])
        print("\nThe robot will MOVE to the start pose, then play the motion.")
        print("Cell must be clear and the e-stop in hand.")
        if input("Type YES to continue: ").strip() != "YES":
            print("Aborted.")
            return

        robot.ensure_auto()
        with robot.speed(MOVE_TO_START_SPEED_PCT):
            robot.move_j(start_joints, validated["tool_id"], MOVE_TO_START_SPEED_PCT)
        input("Press Enter to play: ")

        robot.load_tpd(tpd_name)  # cheap insurance against a stale load
        start = time.monotonic()
        robot.move_tpd(tpd_name, 0, args.ovl)
        print(f"move_tpd returned after {time.monotonic() - start:.3f}s")

        expected_duration = validated["duration_s"] * 100.0 / args.ovl
        deadline = time.monotonic() + expected_duration * 1.5 + 3.0
        while time.monotonic() < deadline:
            if robot.motion_done():
                break
            time.sleep(0.1)
        else:
            print("Warning: motion_done not confirmed within the deadline.")
        elapsed = time.monotonic() - start
        print(f"Elapsed: {elapsed:.2f}s (expected about {expected_duration:.2f}s)")
    except KeyboardInterrupt:
        print("\nInterrupted; stopping.")
        robot.stop()
    finally:
        robot.close()


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
    p.set_defaults(func=cmd_new)

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

    p = sub.add_parser("upload")
    p.add_argument("name")
    _add_ip(p, default=ROBOT_IP)
    p.set_defaults(func=cmd_upload)

    p = sub.add_parser("play")
    p.add_argument("name")
    p.add_argument("--ovl", type=int, default=50)
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
