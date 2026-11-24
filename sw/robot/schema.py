"""File formats for robot motions, with strict validation.

Every loader raises SchemaError(path, field, reason) so a bad file is reported
with the field that caused it. Units: mm and degrees unless stated otherwise.
"""

from __future__ import annotations

import hashlib
import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

NAME_RE = re.compile(r"^[a-z0-9_]{1,19}$")
ALLOWED_PERIODS_MS = (2, 4, 8)
MAX_ABS_TILT_DEG = 85.0
BACKENDS = ("tpd",)
INTERPOLATIONS = ("cartesian", "joint")
ANGLE_VEL_CHANNELS = ("x", "y", "z", "pan", "tilt")
TARGET_VEL_CHANNELS = ("x", "y", "z", "tx", "ty", "tz")


class SchemaError(ValueError):
    """A file does not match its schema. Carries the file, field and reason."""

    def __init__(self, path: object, field_name: str, reason: str) -> None:
        self.path = str(path)
        self.field = field_name
        self.reason = reason
        super().__init__(f"{self.path}: {field_name}: {reason}")


def sha256_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _vec(raw: Any, n: int, path: object, field_name: str) -> tuple[float, ...]:
    if not isinstance(raw, (list, tuple)) or len(raw) != n:
        raise SchemaError(path, field_name, f"expected {n} numbers")
    try:
        return tuple(float(v) for v in raw)
    except (TypeError, ValueError):
        raise SchemaError(path, field_name, "all entries must be numbers") from None


def _number(raw: Any, path: object, field_name: str) -> float:
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise SchemaError(path, field_name, "must be a number")
    return float(raw)


def _check_keys(raw: dict, allowed: set[str], path: object, where: str) -> None:
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise SchemaError(path, where, f"unknown key(s): {', '.join(unknown)}")


@dataclass(frozen=True)
class Keyframe:
    t: float
    pos: tuple[float, float, float] | None  # None only for a joint-mode `joints` kf
    pan: float | None = None
    tilt: float | None = None
    target: tuple[float, float, float] | None = None
    vel: dict[str, float] = field(default_factory=dict)
    stop: bool = False
    joints: tuple[float, ...] | None = None  # joint mode: exact joint target
    seed: tuple[float, ...] | None = None  # joint mode: IK seed for a pose keyframe


@dataclass(frozen=True)
class Anchor:
    """Authoring frame A relative to base B: A's origin is pos, rotated yaw_deg."""

    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw_deg: float = 0.0


@dataclass(frozen=True)
class Orientation:
    mode: str  # "angles" or "target"
    target: tuple[float, float, float] | None = None  # fixed target, mode "target"
    min_target_distance_mm: float = 100.0
    min_horizontal_mm: float = 50.0
    max_tilt_deg: float = 80.0
    max_pan_rate_deg_s: float = 120.0
    max_cumulative_pan_deg: float = 360.0
    allow_long_pan: bool = False


@dataclass(frozen=True)
class Limits:
    max_tcp_speed_mm_s: float = 800.0
    max_joint_speed_deg_s: float = 120.0
    max_joint_acc_deg_s2: float = 600.0
    max_joint_step_deg: float = 2.0
    max_wrist_step_deg: float = 1.0
    min_wrist_margin_deg: float = 15.0  # cartesian mode: |J5| away from 0/180
    min_elbow_margin_deg: float = 5.0  # cartesian mode: |J3| away from 0 (straight)
    overshoot_pct: float = 2.0
    max_roll_deg: float = 0.5  # joint mode only; cartesian mode is level exactly
    max_pointing_error_deg: float = 0.1
    min_segment_s: float = 0.05


LIMIT_KEYS = set(Limits.__dataclass_fields__)


@dataclass(frozen=True)
class Design:
    name: str
    tool_id: int
    sample_period_ms: int
    seed_joints: tuple[float, ...]
    orientation: Orientation
    keyframes: tuple[Keyframe, ...]
    workspace_min: tuple[float, float, float]
    workspace_max: tuple[float, float, float]
    anchor: Anchor = Anchor()
    user_id: int = 0
    limits: Limits = Limits()
    interpolation: str = "cartesian"  # "cartesian" or "joint"


def _keyframe(
    raw: Any, index: int, path: object, mode: str, interpolation: str = "cartesian"
) -> Keyframe:
    where = f"keyframes[{index}]"
    if not isinstance(raw, dict):
        raise SchemaError(path, where, "must be a mapping")
    allowed = {"t", "pos", "pan", "tilt", "target", "vel", "stop", "joints", "seed"}
    _check_keys(raw, allowed, path, where)
    if interpolation != "joint":
        for key in ("joints", "seed"):
            if key in raw:
                raise SchemaError(
                    path, f"{where}.{key}", "only allowed with interpolation: joint"
                )
    elif "vel" in raw:
        raise SchemaError(path, f"{where}.vel", "not allowed with interpolation: joint")
    stop = raw.get("stop", False)
    if not isinstance(stop, bool):
        raise SchemaError(path, f"{where}.stop", "must be true or false")
    if interpolation == "joint" and "joints" in raw:
        extra = sorted({"pos", "pan", "tilt", "target", "seed"} & set(raw))
        if extra:
            raise SchemaError(
                path, where, f"a 'joints' keyframe cannot also set {', '.join(extra)}"
            )
        if "t" not in raw:
            raise SchemaError(path, where, "needs 't'")
        t = _number(raw["t"], path, f"{where}.t")
        joints = _vec(raw["joints"], 6, path, f"{where}.joints")
        return Keyframe(t, None, stop=stop, joints=joints)
    if "t" not in raw or "pos" not in raw:
        raise SchemaError(path, where, "needs 't' and 'pos'")
    t = _number(raw["t"], path, f"{where}.t")
    pos = _vec(raw["pos"], 3, path, f"{where}.pos")
    pan = tilt = target = None
    if mode == "angles":
        if "target" in raw:
            raise SchemaError(path, f"{where}.target", "not allowed in angles mode")
        if "pan" not in raw or "tilt" not in raw:
            raise SchemaError(path, where, "angles mode needs 'pan' and 'tilt'")
        pan = _number(raw["pan"], path, f"{where}.pan")
        tilt = _number(raw["tilt"], path, f"{where}.tilt")
        if abs(tilt) > MAX_ABS_TILT_DEG:
            raise SchemaError(
                path, f"{where}.tilt", f"|tilt| must be <= {MAX_ABS_TILT_DEG}"
            )
    else:
        if "pan" in raw or "tilt" in raw:
            raise SchemaError(path, where, "pan/tilt not allowed in target mode")
        if "target" in raw:
            target = _vec(raw["target"], 3, path, f"{where}.target")
    vel_raw = raw.get("vel", {}) or {}
    if not isinstance(vel_raw, dict):
        raise SchemaError(path, f"{where}.vel", "must be a mapping of channel: value")
    vel = {str(k): _number(v, path, f"{where}.vel.{k}") for k, v in vel_raw.items()}
    seed = _vec(raw["seed"], 6, path, f"{where}.seed") if "seed" in raw else None
    return Keyframe(t, pos, pan, tilt, target, vel, stop, seed=seed)


def _orientation(raw: Any, path: object) -> Orientation:
    if not isinstance(raw, dict) or "mode" not in raw:
        raise SchemaError(path, "orientation", "needs a 'mode' key")
    mode = raw["mode"]
    if mode not in ("angles", "target"):
        raise SchemaError(path, "orientation.mode", "must be 'angles' or 'target'")
    allowed = {
        "mode", "target", "min_target_distance_mm", "min_horizontal_mm",
        "max_tilt_deg", "max_pan_rate_deg_s", "max_cumulative_pan_deg",
        "allow_long_pan",
    }  # fmt: skip
    _check_keys(raw, allowed, path, "orientation")
    values: dict[str, Any] = {"mode": mode}
    if "target" in raw:
        if mode != "target":
            raise SchemaError(path, "orientation.target", "only valid in target mode")
        values["target"] = _vec(raw["target"], 3, path, "orientation.target")
    for key in allowed - {"mode", "target", "allow_long_pan"}:
        if key in raw:
            values[key] = _number(raw[key], path, f"orientation.{key}")
    if "allow_long_pan" in raw:
        values["allow_long_pan"] = bool(raw["allow_long_pan"])
    if "max_tilt_deg" in values and values["max_tilt_deg"] > MAX_ABS_TILT_DEG:
        raise SchemaError(
            path, "orientation.max_tilt_deg", f"must be <= {MAX_ABS_TILT_DEG}"
        )
    return Orientation(**values)


def _limits(raw: Any, path: object) -> Limits:
    if raw is None:
        return Limits()
    if not isinstance(raw, dict):
        raise SchemaError(path, "limits", "must be a mapping")
    _check_keys(raw, LIMIT_KEYS, path, "limits")
    return Limits(**{k: _number(v, path, f"limits.{k}") for k, v in raw.items()})


def _check_keyframes(
    keyframes: tuple[Keyframe, ...],
    orient: Orientation,
    design_target: bool,
    limits: Limits,
    path: object,
) -> None:
    if len(keyframes) < 2:
        raise SchemaError(path, "keyframes", "at least 2 keyframes are needed")
    if keyframes[0].t != 0.0:
        raise SchemaError(path, "keyframes[0].t", "the first keyframe must be at t = 0")
    for i in range(1, len(keyframes)):
        dt = keyframes[i].t - keyframes[i - 1].t
        if dt <= 0:
            raise SchemaError(path, f"keyframes[{i}].t", "times must strictly increase")
        if dt < limits.min_segment_s:
            raise SchemaError(
                path,
                f"keyframes[{i}].t",
                f"segment shorter than min_segment_s ({limits.min_segment_s})",
            )
    per_keyframe_target = [kf.target is not None for kf in keyframes]
    if orient.mode == "target":
        if design_target and any(per_keyframe_target):
            raise SchemaError(
                path,
                "orientation.target",
                "use a design-level target or per-keyframe targets, not both",
            )
        if not design_target and not all(per_keyframe_target):
            raise SchemaError(
                path,
                "keyframes",
                "target mode needs a design-level target or a target on every keyframe",
            )
    allowed_vel = set(
        ANGLE_VEL_CHANNELS
        if orient.mode == "angles"
        else TARGET_VEL_CHANNELS
        if all(per_keyframe_target) and not design_target
        else ("x", "y", "z")
    )
    for i, kf in enumerate(keyframes):
        bad = sorted(set(kf.vel) - allowed_vel)
        if bad:
            raise SchemaError(
                path,
                f"keyframes[{i}].vel",
                f"channels not valid in this mode: {', '.join(bad)}",
            )
        if i in (0, len(keyframes) - 1) and any(v != 0 for v in kf.vel.values()):
            raise SchemaError(
                path, f"keyframes[{i}].vel", "first and last keyframes must be at rest"
            )


def design_from_dict(
    raw: Any, path: object = "<dict>", folder_name: str | None = None
) -> Design:
    if not isinstance(raw, dict):
        raise SchemaError(path, "<root>", "must be a mapping")
    allowed = {
        "name",
        "tool_id",
        "user_id",
        "interpolation",
        "sample_period_ms",
        "anchor",
        "seed_joints",
        "orientation",
        "limits",
        "workspace",
        "keyframes",
    }
    _check_keys(raw, allowed, path, "<root>")
    name = raw.get("name")
    if not isinstance(name, str) or not NAME_RE.match(name):
        raise SchemaError(path, "name", "must match ^[a-z0-9_]{1,19}$")
    if folder_name is not None and name != folder_name:
        raise SchemaError(path, "name", f"must equal the folder name '{folder_name}'")
    tool_id = raw.get("tool_id")
    if (
        not isinstance(tool_id, int)
        or isinstance(tool_id, bool)
        or not 0 <= tool_id <= 14
    ):
        raise SchemaError(path, "tool_id", "must be an integer 0..14")
    user_id = raw.get("user_id", 0)
    if (
        not isinstance(user_id, int)
        or isinstance(user_id, bool)
        or not 0 <= user_id <= 14
    ):
        raise SchemaError(path, "user_id", "must be an integer 0..14")
    period = raw.get("sample_period_ms")
    if period not in ALLOWED_PERIODS_MS:
        raise SchemaError(
            path, "sample_period_ms", f"must be one of {ALLOWED_PERIODS_MS}"
        )
    seed = _vec(raw.get("seed_joints"), 6, path, "seed_joints")
    anchor_raw = raw.get("anchor") or {}
    if not isinstance(anchor_raw, dict):
        raise SchemaError(path, "anchor", "must be a mapping")
    anchor = Anchor(
        _vec(anchor_raw.get("pos", [0, 0, 0]), 3, path, "anchor.pos"),
        _number(anchor_raw.get("yaw_deg", 0.0), path, "anchor.yaw_deg"),
    )
    orientation = _orientation(raw.get("orientation"), path)
    interpolation = raw.get("interpolation", "cartesian")
    if interpolation not in INTERPOLATIONS:
        raise SchemaError(path, "interpolation", f"must be one of {INTERPOLATIONS}")
    if interpolation == "joint" and orientation.mode != "angles":
        raise SchemaError(
            path,
            "interpolation",
            "joint interpolation needs orientation mode 'angles' (target mode must "
            "re-aim the camera at every sample, which only cartesian mode does)",
        )
    limits = _limits(raw.get("limits"), path)
    workspace = raw.get("workspace")
    if not isinstance(workspace, dict):
        raise SchemaError(path, "workspace", "needs 'min' and 'max'")
    ws_min = _vec(workspace.get("min"), 3, path, "workspace.min")
    ws_max = _vec(workspace.get("max"), 3, path, "workspace.max")
    if any(lo > hi for lo, hi in zip(ws_min, ws_max, strict=True)):
        raise SchemaError(path, "workspace", "min must not exceed max on any axis")
    kf_raw = raw.get("keyframes")
    if not isinstance(kf_raw, list):
        raise SchemaError(path, "keyframes", "must be a list")
    keyframes = tuple(
        _keyframe(kf, i, path, orientation.mode, interpolation)
        for i, kf in enumerate(kf_raw)
    )
    _check_keyframes(
        keyframes, orientation, orientation.target is not None, limits, path
    )
    return Design(
        name,
        tool_id,
        int(period),
        seed,
        orientation,
        keyframes,
        ws_min,
        ws_max,
        anchor,
        user_id,
        limits,
        interpolation,
    )


def load_design(path: Path) -> Design:
    path = Path(path)
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise SchemaError(path, "<yaml>", f"not valid YAML: {exc}") from None
    return design_from_dict(raw, path, folder_name=path.parent.name)


@dataclass(frozen=True)
class CameraTool:
    tool_id: int
    t_coord: tuple[float, ...]
    mass_kg: float
    cog_mm: tuple[float, float, float]
    approved: bool
    date: str | None
    horizon_residual_deg: float | None
    pointing_residual_deg: float | None
    optical_centre_error_mm: float | None

    def require_approved(self) -> None:
        if not self.approved:
            raise SchemaError(
                "camera_tool.yaml",
                "calibration.approved",
                "camera tool calibration is not approved",
            )


def camera_tool_from_dict(raw: Any, path: object = "<dict>") -> CameraTool:
    if not isinstance(raw, dict):
        raise SchemaError(path, "<root>", "must be a mapping")
    tool_id = raw.get("tool_id")
    if not isinstance(tool_id, int) or isinstance(tool_id, bool):
        raise SchemaError(path, "tool_id", "must be an integer")
    payload = raw.get("payload") or {}
    cal = raw.get("calibration") or {}
    if not isinstance(payload, dict) or not isinstance(cal, dict):
        raise SchemaError(path, "payload/calibration", "must be mappings")
    approved = cal.get("approved", False)
    if not isinstance(approved, bool):
        raise SchemaError(path, "calibration.approved", "must be true or false")

    def opt(key: str) -> float | None:
        value = cal.get(key)
        return None if value is None else _number(value, path, f"calibration.{key}")

    return CameraTool(
        tool_id=tool_id,
        t_coord=_vec(raw.get("t_coord"), 6, path, "t_coord"),
        mass_kg=_number(payload.get("mass_kg"), path, "payload.mass_kg"),
        cog_mm=_vec(payload.get("cog_mm"), 3, path, "payload.cog_mm"),
        approved=approved,
        date=cal.get("date"),
        horizon_residual_deg=opt("horizon_residual_deg"),
        pointing_residual_deg=opt("pointing_residual_deg"),
        optical_centre_error_mm=opt("optical_centre_error_mm"),
    )


def load_camera_tool(path: Path) -> CameraTool:
    path = Path(path)
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise SchemaError(path, "<yaml>", f"not valid YAML: {exc}") from None
    return camera_tool_from_dict(raw, path)


MOTION_FILE_KEYS = (
    "name", "backend", "controller_file", "tpd_name", "md5", "num_points",
    "sample_period_ms", "duration_s", "design_duration_s", "start_joints",
    "start_pose", "tool_id", "approved_ovl", "design_sha256", "published_at",
)  # fmt: skip


@dataclass(frozen=True)
class MotionConfig:
    """The published metadata of one motion; the only file the app reads."""

    name: str
    backend: str
    controller_file: str
    tpd_name: str
    md5: str
    num_points: int
    sample_period_ms: int
    duration_s: float
    design_duration_s: float
    start_joints: tuple[float, ...]
    start_pose: tuple[float, ...]
    tool_id: int
    approved_ovl: int
    design_sha256: str
    published_at: str

    def check_consistent(self, path: object = "motion.json") -> None:
        """Reject cross-field inconsistencies. Raises SchemaError."""
        if self.backend not in BACKENDS:
            raise SchemaError(path, "backend", f"must be one of {BACKENDS}")
        expected = f"/fruser/traj/motion_{self.name}.txt"
        if self.controller_file != expected:
            raise SchemaError(path, "controller_file", f"must be {expected}")
        expected_tpd = f"traj/motion_{self.name}"
        if self.tpd_name != expected_tpd:
            raise SchemaError(path, "tpd_name", f"must be {expected_tpd}")
        if self.num_points < 2:
            raise SchemaError(path, "num_points", "needs at least 2 points")
        expected_duration = (self.num_points - 1) * self.sample_period_ms / 1000.0
        if abs(self.duration_s - expected_duration) > 1e-6:
            raise SchemaError(
                path,
                "duration_s",
                f"must equal (num_points - 1) * period = {expected_duration}",
            )
        if not 1 <= self.approved_ovl <= 100:
            raise SchemaError(
                path, "approved_ovl", "must satisfy 1 <= approved_ovl <= 100"
            )
        if len(self.start_joints) != 6:
            raise SchemaError(path, "start_joints", "expected 6 values")
        if len(self.start_pose) != 6:
            raise SchemaError(path, "start_pose", "expected 6 values")


def motion_config_from_dict(raw: Any, path: object = "<dict>") -> MotionConfig:
    if not isinstance(raw, dict):
        raise SchemaError(path, "<root>", "must be a mapping")
    missing = [k for k in MOTION_FILE_KEYS if k not in raw]
    if missing:
        raise SchemaError(path, "<root>", f"missing key(s): {', '.join(missing)}")
    config = MotionConfig(
        name=str(raw["name"]),
        backend=str(raw["backend"]),
        controller_file=str(raw["controller_file"]),
        tpd_name=str(raw["tpd_name"]),
        md5=str(raw["md5"]),
        num_points=int(raw["num_points"]),
        sample_period_ms=int(raw["sample_period_ms"]),
        duration_s=float(raw["duration_s"]),
        design_duration_s=float(raw["design_duration_s"]),
        start_joints=_vec(raw["start_joints"], 6, path, "start_joints"),
        start_pose=_vec(raw["start_pose"], 6, path, "start_pose"),
        tool_id=int(raw["tool_id"]),
        approved_ovl=int(raw["approved_ovl"]),
        design_sha256=str(raw["design_sha256"]),
        published_at=str(raw["published_at"]),
    )
    config.check_consistent(path)
    return config


def motion_config_to_dict(config: MotionConfig) -> dict[str, Any]:
    out = {k: getattr(config, k) for k in MOTION_FILE_KEYS}
    for key in ("start_joints", "start_pose"):
        out[key] = list(out[key])
    return out


def load_motion_config(path: Path) -> MotionConfig:
    path = Path(path)
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise SchemaError(path, "<json>", f"not valid JSON: {exc}") from None
    return motion_config_from_dict(raw, path)


def load_motion_configs(motions_dir: Path) -> dict[str, MotionConfig]:
    """Load every motions/<name>/motion.json. Folders without one are skipped."""
    configs: dict[str, MotionConfig] = {}
    for folder in sorted(Path(motions_dir).iterdir()):
        meta = folder / "motion.json"
        if folder.is_dir() and meta.exists():
            config = load_motion_config(meta)
            if config.name != folder.name:
                raise SchemaError(
                    meta, "name", f"must equal the folder name '{folder.name}'"
                )
            configs[config.name] = config
    return configs


@dataclass(frozen=True)
class RobotSettings:
    ip: str = "192.168.58.2"
    backend: str = "tpd"
    start_tolerance_deg: float = 0.5
    move_to_start_speed_pct: int = 10
    safe_default_speed_pct: int = 10
    pre_roll_s: float = 0.5
    post_roll_s: float = 1.0
    auto_stop_after_motion: bool = True

    def __post_init__(self) -> None:
        if self.backend not in BACKENDS:
            raise ValueError(f"backend must be one of {BACKENDS}")
        for key in ("move_to_start_speed_pct", "safe_default_speed_pct"):
            if not 1 <= getattr(self, key) <= 100:
                raise ValueError(f"{key} must be 1..100")


def robot_settings_from_dict(raw: Any) -> RobotSettings:
    """Build settings from the 'robot' block of livestream_config.json."""
    if raw is None:
        return RobotSettings()
    if not isinstance(raw, dict):
        raise SchemaError("livestream_config.json", "robot", "must be a mapping")
    allowed = set(RobotSettings.__dataclass_fields__)
    _check_keys(raw, allowed, "livestream_config.json", "robot")
    try:
        return RobotSettings(**raw)
    except ValueError as exc:
        raise SchemaError("livestream_config.json", "robot", str(exc)) from None
