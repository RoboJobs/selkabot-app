"""CLI for designing, validating, compiling and running robot motions.

Usage: `uv run python -m sw.robot.motionctl <cmd> ...`. See `--help` on each
command. Defaults to the simulator (192.168.50.2); `upload` and `play` need
--ip 192.168.58.2 explicitly and are meant to be run by the operator with the
cell clear and the e-stop in hand. See the design doc's "Read this first"
and "Simulator vs real robot" sections before changing this file.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import yaml

from sw.robot import path as path_module
from sw.robot import schema, templates, tpd_file, validate
from sw.robot.fr5 import FR5, FairinoError, FileTransferError

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
    (folder / "design.yaml").write_text(yaml.safe_dump(design, sort_keys=False))
    print(f"Wrote {folder / 'design.yaml'} from template '{args.template}'.")
    print("Edit its keyframes and workspace before validating.")


# -- pose -----------------------------------------------------------------


def cmd_pose(args: argparse.Namespace) -> None:
    from sw.robot.pose import level_rotation, roll_error_deg, sdk_euler_to_rotation

    robot = FR5.connect(args.ip)
    try:
        joints = robot.joints()
        pose = robot.tcp_pose()
        tool = robot.active_tool()
    finally:
        robot.close()
    print("Joints (deg):", [round(v, 4) for v in joints])
    print("Active tool:", tool)
    print("TCP pose [x,y,z,rx,ry,rz] (mm, deg):", [round(v, 4) for v in pose])
    rotation = sdk_euler_to_rotation(*pose[3:])
    roll = roll_error_deg(rotation)
    # pan/tilt of the current pose, assuming it is level: invert level_rotation
    # is not available directly, so report roll and let the operator read
    # pan/tilt off the pose's own rz/combination if they need it precisely.
    print(f"Roll relative to level: {roll:.3f}deg (0 if level)")
    print()
    print("Copy the position into a keyframe's 'pos', the joints into")
    print("'seed_joints', and read pan/tilt from the pendant or by trial.")
    _ = level_rotation  # kept for future use; silence unused-import lint


# -- plan -----------------------------------------------------------------


def cmd_plan(args: argparse.Namespace) -> None:
    design = load_design(args.name)
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


def cmd_validate(args: argparse.Namespace) -> None:
    design = load_design(args.name)
    samples = path_module.sample_path(design)
    robot = FR5.connect(args.ip)
    try:
        report = validate.validate(design, samples, robot)
        install_angle = robot.install_angle()
    finally:
        robot.close()

    for check in report.checks:
        status = "OK" if check.passed else "FAIL"
        detail = f": {check.detail}" if check.detail else ""
        print(f"[{status}] {check.name}{detail}")

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
    p.set_defaults(func=cmd_new)

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
