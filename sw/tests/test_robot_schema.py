"""Schema validation for robot motion files (no hardware)."""

import copy
import json

import pytest

from sw.robot import schema, templates

ANGLES = {
    "name": "demo",
    "tool_id": 1,
    "sample_period_ms": 8,
    "seed_joints": [0, -90, 90, -90, -90, 0],
    "orientation": {"mode": "angles"},
    "workspace": {"min": [0, -1000, 0], "max": [1000, 1000, 1000]},
    "keyframes": [
        {"t": 0.0, "pos": [600, -400, 450], "pan": 20, "tilt": -5},
        {"t": 1.2, "pos": [600, 400, 450], "pan": -20, "tilt": -5},
        {"t": 2.0, "pos": [650, 400, 700], "pan": -20, "tilt": 5},
    ],
}

TARGET = {
    "name": "demo",
    "tool_id": 1,
    "sample_period_ms": 8,
    "seed_joints": [0, -90, 90, -90, -90, 0],
    "orientation": {"mode": "target", "target": [0, 0, 300]},
    "workspace": {"min": [0, -1000, 0], "max": [1000, 1000, 1000]},
    "keyframes": [
        {"t": 0.0, "pos": [600, -400, 450]},
        {"t": 2.0, "pos": [600, 400, 450]},
    ],
}


def _mutate(base, fn):
    raw = copy.deepcopy(base)
    fn(raw)
    return raw


def test_valid_angles_design_loads():
    d = schema.design_from_dict(ANGLES, "demo", folder_name="demo")
    assert d.orientation.mode == "angles"
    assert len(d.keyframes) == 3


def test_valid_target_design_loads():
    d = schema.design_from_dict(TARGET, "demo", folder_name="demo")
    assert d.orientation.target == (0.0, 0.0, 300.0)


@pytest.mark.parametrize(
    "mutate, field",
    [
        (lambda r: r.update(name="Bad-Name"), "name"),
        (lambda r: r.update(name="x" * 20), "name"),
        (lambda r: r.update(tool_id=15), "tool_id"),
        (lambda r: r.update(tool_id=-1), "tool_id"),
        (lambda r: r.update(sample_period_ms=5), "sample_period_ms"),
        (lambda r: r.update(seed_joints=[0, 0, 0]), "seed_joints"),
        (
            lambda r: r["keyframes"].__setitem__(1, {**r["keyframes"][1], "t": 0.0}),
            "keyframes[1].t",
        ),
        (
            lambda r: r["keyframes"].__setitem__(0, {**r["keyframes"][0], "t": 0.5}),
            "keyframes[0].t",
        ),
        (lambda r: r.update(keyframes=r["keyframes"][:1]), "keyframes"),
        (lambda r: r["keyframes"][1].pop("pan"), "keyframes[1]"),
        (lambda r: r["keyframes"][1].update(tilt=86), "keyframes[1].tilt"),
        (lambda r: r["keyframes"][1].update(target=[0, 0, 0]), "keyframes[1].target"),
        (lambda r: r["keyframes"][1].update(pos=[1, 2]), "keyframes[1].pos"),
        (lambda r: r["keyframes"][0].update(vel={"x": 5}), "keyframes[0].vel"),
        (lambda r: r["keyframes"][2].update(vel={"pan": 5}), "keyframes[2].vel"),
        (lambda r: r["keyframes"][1].update(vel={"tx": 5}), "keyframes[1].vel"),
        (lambda r: r["keyframes"][1].update(t=0.01), "keyframes[1].t"),
        (
            lambda r: r.update(workspace={"min": [5, 0, 0], "max": [1, 1, 1]}),
            "workspace",
        ),
        (lambda r: r.update(bogus=1), "<root>"),
        (lambda r: r["keyframes"][1].update(surprise=1), "keyframes[1]"),
    ],
)
def test_invalid_angles_designs_name_the_field(mutate, field):
    raw = _mutate(ANGLES, mutate)
    with pytest.raises(schema.SchemaError) as exc:
        schema.design_from_dict(raw, "demo", folder_name="demo")
    assert exc.value.field.startswith(field.split("[")[0]) or field in exc.value.field


def test_name_must_equal_folder_name():
    with pytest.raises(schema.SchemaError, match="folder"):
        schema.design_from_dict(ANGLES, "demo", folder_name="other")


def test_target_mode_rejects_pan_and_tilt():
    raw = _mutate(TARGET, lambda r: r["keyframes"][0].update(pan=10))
    with pytest.raises(schema.SchemaError, match="pan/tilt"):
        schema.design_from_dict(raw, "demo")


def test_angles_mode_rejects_a_design_level_target():
    raw = _mutate(
        ANGLES, lambda r: r.update(orientation={"mode": "angles", "target": [0, 0, 1]})
    )
    with pytest.raises(schema.SchemaError, match="orientation.target"):
        schema.design_from_dict(raw, "demo")


def test_target_mode_rejects_both_design_and_keyframe_targets():
    raw = _mutate(TARGET, lambda r: r["keyframes"][0].update(target=[1, 1, 1]))
    with pytest.raises(schema.SchemaError, match="not both"):
        schema.design_from_dict(raw, "demo")


def test_target_mode_needs_a_target_somewhere():
    raw = _mutate(TARGET, lambda r: r["orientation"].pop("target"))
    with pytest.raises(schema.SchemaError, match="target"):
        schema.design_from_dict(raw, "demo")


def test_per_keyframe_targets_without_design_target_load():
    raw = _mutate(
        TARGET,
        lambda r: (
            r["orientation"].pop("target"),
            r["keyframes"][0].update(target=[0, 0, 300]),
            r["keyframes"][1].update(target=[0, 200, 300]),
        ),
    )
    d = schema.design_from_dict(raw, "demo")
    assert d.keyframes[1].target == (0.0, 200.0, 300.0)


def test_segment_shorter_than_minimum_is_rejected():
    raw = _mutate(ANGLES, lambda r: r["keyframes"][1].update(t=0.02))
    with pytest.raises(schema.SchemaError, match="min_segment_s"):
        schema.design_from_dict(raw, "demo")


def test_load_design_reads_yaml_and_checks_folder(tmp_path):
    import yaml

    folder = tmp_path / "demo"
    folder.mkdir()
    (folder / "design.yaml").write_text(yaml.safe_dump(ANGLES))
    d = schema.load_design(folder / "design.yaml")
    assert d.name == "demo"


def test_load_design_rejects_invalid_yaml(tmp_path):
    folder = tmp_path / "demo"
    folder.mkdir()
    (folder / "design.yaml").write_text("name: [unclosed")
    with pytest.raises(schema.SchemaError, match="YAML"):
        schema.load_design(folder / "design.yaml")


def _motion_dict(**overrides):
    base = {
        "name": "demo",
        "backend": "tpd",
        "controller_file": "/fruser/traj/motion_demo.txt",
        "tpd_name": "traj/motion_demo",
        "md5": "0" * 32,
        "num_points": 251,
        "sample_period_ms": 8,
        "duration_s": 2.0,
        "design_duration_s": 2.0,
        "start_joints": [0, -90, 90, -90, -90, 0],
        "start_pose": [600, -400, 450, 180, 0, 90],
        "tool_id": 0,
        "approved_ovl": 50,
        "design_sha256": "a" * 64,
        "published_at": "2026-11-24T10:00:00",
    }
    base.update(overrides)
    return base


def test_motion_config_round_trips_through_json(tmp_path):
    raw = _motion_dict()
    path = tmp_path / "motion.json"
    path.write_text(json.dumps(raw))
    config = schema.load_motion_config(path)
    assert schema.motion_config_to_dict(config) == raw


@pytest.mark.parametrize(
    "override, field",
    [
        ({"controller_file": "/fruser/traj/other.txt"}, "controller_file"),
        ({"tpd_name": "traj/other"}, "tpd_name"),
        ({"duration_s": 1.5}, "duration_s"),
        ({"approved_ovl": 0}, "approved_ovl"),
        ({"approved_ovl": 101}, "approved_ovl"),
        ({"start_joints": [0, 0]}, "start_joints"),
        ({"backend": "servo"}, "backend"),
    ],
)
def test_motion_config_cross_field_violations(tmp_path, override, field):
    path = tmp_path / "motion.json"
    path.write_text(json.dumps(_motion_dict(**override)))
    with pytest.raises(schema.SchemaError) as exc:
        schema.load_motion_config(path)
    assert exc.value.field == field


def test_motion_config_missing_key_is_reported(tmp_path):
    raw = _motion_dict()
    raw.pop("md5")
    path = tmp_path / "motion.json"
    path.write_text(json.dumps(raw))
    with pytest.raises(schema.SchemaError, match="md5"):
        schema.load_motion_config(path)


def test_load_motion_configs_skips_folders_without_metadata(tmp_path):
    good = tmp_path / "demo"
    good.mkdir()
    (good / "motion.json").write_text(json.dumps(_motion_dict()))
    (tmp_path / "draft").mkdir()  # no motion.json yet
    configs = schema.load_motion_configs(tmp_path)
    assert list(configs) == ["demo"]


def test_camera_tool_requires_approval():
    raw = {
        "tool_id": 1,
        "t_coord": [0, 0, 60, 0, 0, 0],
        "payload": {"mass_kg": 0.25, "cog_mm": [0, 0, 30]},
        "calibration": {"approved": False},
    }
    tool = schema.camera_tool_from_dict(raw)
    with pytest.raises(schema.SchemaError, match="not approved"):
        tool.require_approved()
    raw["calibration"]["approved"] = True
    schema.camera_tool_from_dict(raw).require_approved()


def test_robot_settings_defaults_and_validation():
    defaults = schema.robot_settings_from_dict(None)
    assert defaults.move_to_start_speed_pct == 25
    assert defaults.backend == "tpd"
    with pytest.raises(schema.SchemaError):
        schema.robot_settings_from_dict({"move_to_start_speed_pct": 200})
    with pytest.raises(schema.SchemaError):
        schema.robot_settings_from_dict({"backend": "servo"})
    with pytest.raises(schema.SchemaError):
        schema.robot_settings_from_dict({"unknown_key": 1})


@pytest.mark.parametrize("name", list(templates.TEMPLATES))
def test_every_template_loads_and_samples(name):
    from sw.robot import path

    raw = templates.new_design(name, "demo")
    design = schema.design_from_dict(raw, "demo", folder_name="demo")
    assert len(path.sample_path(design)) > 1
    assert design.sample_period_ms == 8
    assert design.tool_id == 0


def test_unknown_template_is_rejected():
    with pytest.raises(ValueError, match="unknown template"):
        templates.new_design("nope", "demo")


# -- interpolation: joint ------------------------------------------------------

JOINT = {
    "name": "demo",
    "tool_id": 1,
    "interpolation": "joint",
    "sample_period_ms": 8,
    "seed_joints": [90, -90, 0, 30, -90, 45],
    "orientation": {"mode": "angles"},
    "workspace": {"min": [-1000, -1500, -700], "max": [1000, 0, 1000]},
    "keyframes": [
        {"t": 0.0, "pos": [96, -403, 673], "pan": -91, "tilt": -29},
        {"t": 5.0, "joints": [90, 71, 0, -181, -90, 45]},
    ],
}


def test_interpolation_defaults_to_cartesian():
    assert schema.design_from_dict(ANGLES, "demo").interpolation == "cartesian"


def test_joint_design_with_pose_and_joints_keyframes_loads():
    d = schema.design_from_dict(JOINT, "demo", folder_name="demo")
    assert d.interpolation == "joint"
    assert d.keyframes[0].pos == (96.0, -403.0, 673.0)
    assert d.keyframes[1].pos is None
    assert d.keyframes[1].joints == (90.0, 71.0, 0.0, -181.0, -90.0, 45.0)


def test_joint_design_pose_keyframe_accepts_a_seed():
    raw = copy.deepcopy(JOINT)
    raw["keyframes"][1] = {
        "t": 5.0,
        "pos": [102, -758, -508],
        "pan": -90,
        "tilt": 20,
        "seed": [90, 71, 0, -181, -90, 45],
    }
    d = schema.design_from_dict(raw, "demo")
    assert d.keyframes[1].seed == (90.0, 71.0, 0.0, -181.0, -90.0, 45.0)


@pytest.mark.parametrize(
    "mutate, field",
    [
        (lambda r: r.update(interpolation="spline"), "interpolation"),
        (
            lambda r: r.update(orientation={"mode": "target", "target": [0, 0, 0]}),
            "interpolation",
        ),
        (lambda r: r["keyframes"][1].update(pos=[1, 2, 3]), "keyframes[1]"),
        (lambda r: r["keyframes"][0].update(vel={"x": 1}), "keyframes[0].vel"),
        (lambda r: r["keyframes"][1].update(joints=[1, 2, 3]), "keyframes[1].joints"),
    ],
)
def test_invalid_joint_designs_raise_with_field(mutate, field):
    with pytest.raises(schema.SchemaError) as exc:
        schema.design_from_dict(_mutate(JOINT, mutate), "demo")
    assert exc.value.field == field


@pytest.mark.parametrize("key", ["joints", "seed"])
def test_joints_and_seed_are_rejected_in_cartesian_mode(key):
    raw = copy.deepcopy(ANGLES)
    raw["keyframes"][1][key] = [0, 0, 0, 0, 0, 0]
    with pytest.raises(schema.SchemaError) as exc:
        schema.design_from_dict(raw, "demo")
    assert exc.value.field == f"keyframes[1].{key}"
