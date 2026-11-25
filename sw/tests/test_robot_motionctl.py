"""Tests for the motionctl CLI, using FakeRobotClient and tmp_path. No hardware."""

from __future__ import annotations

import copy
import json

import pytest

from sw.robot import fr5, motionctl, schema, tpd_file
from sw.robot.pose import (
    camera_angles,
    level_rotation,
    rotation_to_sdk_euler,
)
from sw.tests.fakes.fake_kinematics import _joints_to_pose, _pose_to_joints
from sw.tests.fakes.fake_robot import FakeRobotClient
from sw.tests.fakes.wrist_kinematics import WristFK


@pytest.fixture(autouse=True)
def _redirect_dirs(tmp_path, monkeypatch):
    """Every test gets its own motions/ and build/ directories."""
    motions = tmp_path / "motions"
    build = tmp_path / "build"
    monkeypatch.setattr(motionctl, "MOTIONS_DIR", motions)
    monkeypatch.setattr(motionctl, "BUILD_DIR", build)
    return motions, build


@pytest.fixture
def client():
    return FakeRobotClient()


@pytest.fixture
def connect(client, monkeypatch):
    """Replace FR5.connect with one that returns an FR5 wrapping `client`."""

    def fake_connect(ip):
        return fr5.FR5(client, ip)

    monkeypatch.setattr(motionctl.FR5, "connect", staticmethod(fake_connect))
    return client


def _run(args):
    return motionctl.main(args)


def _seed_ik_fk(client):
    """A trivial but invertible IK/FK so validate() can run end to end."""

    def ik(type, desc_pos, joint_pos_ref):
        return 0, _pose_to_joints(desc_pos)

    def fk(joint_pos):
        return 0, _joints_to_pose(joint_pos)

    client.GetInverseKinRef = ik
    client.GetForwardKin = fk


# -- new --------------------------------------------------------------------


def test_new_writes_a_loadable_design(_redirect_dirs):
    motions, _build = _redirect_dirs
    _run(["new", "demo", "--template", "first_test"])
    design = schema.load_design(motions / "demo" / "design.yaml")
    assert design.name == "demo"
    assert design.tool_id == 0
    assert design.sample_period_ms == 8


def test_new_refuses_an_existing_folder(_redirect_dirs):
    _run(["new", "demo", "--template", "first_test"])
    assert _run(["new", "demo", "--template", "first_test"]) == 1


# -- validate --------------------------------------------------------------


def test_validate_writes_validated_json_only_on_pass(connect, _redirect_dirs):
    motions, _build = _redirect_dirs
    _seed_ik_fk(connect)
    _run(["new", "demo", "--template", "first_test"])
    code = _run(["validate", "demo"])
    assert code == 0
    data = json.loads((motions / "demo" / "validated.json").read_text())
    assert data["name"] == "demo"
    assert all(c["passed"] for c in data["checks"])
    assert data["design_sha256"]


def test_validate_fails_and_writes_nothing_on_ik_failure(connect, _redirect_dirs):
    motions, _build = _redirect_dirs
    connect.GetInverseKinRef = lambda type, desc_pos, joint_pos_ref: (4, None)
    _run(["new", "demo", "--template", "first_test"])
    code = _run(["validate", "demo"])
    assert code == 1
    assert not (motions / "demo" / "validated.json").exists()


# -- compile ----------------------------------------------------------------


def test_compile_writes_the_build_file(connect, _redirect_dirs):
    _redirect_dirs_, build = _redirect_dirs
    _seed_ik_fk(connect)
    _run(["new", "demo", "--template", "first_test"])
    _run(["validate", "demo"])
    assert _run(["compile", "demo"]) == 0
    out = build / "motion_demo.txt"
    assert out.exists()
    tpd = tpd_file.read(out)
    assert tpd.period_ms == 8
    assert tpd.tool_id == 0


def test_compile_refuses_a_stale_design_sha(connect, _redirect_dirs):
    motions, _build = _redirect_dirs
    _seed_ik_fk(connect)
    _run(["new", "demo", "--template", "first_test"])
    _run(["validate", "demo"])
    # Touch the design after validating.
    design_path = motions / "demo" / "design.yaml"
    design_path.write_text(design_path.read_text() + "\n# changed\n")
    assert _run(["compile", "demo"]) == 1


def test_compile_refuses_without_validated_json(_redirect_dirs):
    _run(["new", "demo", "--template", "first_test"])
    assert _run(["compile", "demo"]) == 1


# -- simplay (D9) ------------------------------------------------------------


def test_simplay_streams_compiled_rows_and_never_touches_tpd_calls(
    connect, _redirect_dirs
):
    _seed_ik_fk(connect)
    _run(["new", "demo", "--template", "first_test"])
    _run(["validate", "demo"])
    _run(["compile", "demo"])

    code = _run(["simplay", "demo", "--ovl", "10000"])  # fast, so the test is quick
    assert code == 0
    assert connect.servo_move_started
    assert connect.servo_move_ended
    assert len(connect.servo_j_calls) > 0
    for _joints, axis_pos, _cmdt in connect.servo_j_calls:
        assert axis_pos == [0.0, 0.0, 0.0, 0.0]
    assert connect.file_upload_calls == []
    assert connect.tpd_loaded is None
    assert connect.move_tpd_calls == []


def test_simplay_compiles_automatically_if_missing(connect, _redirect_dirs):
    _redirect_dirs_, build = _redirect_dirs
    _seed_ik_fk(connect)
    _run(["new", "demo", "--template", "first_test"])
    _run(["validate", "demo"])
    assert not (build / "motion_demo.txt").exists()

    code = _run(["simplay", "demo", "--ovl", "10000"])
    assert code == 0
    assert (build / "motion_demo.txt").exists()


# -- upload -------------------------------------------------------------


def test_upload_refuses_the_sim_ip(connect, _redirect_dirs):
    _seed_ik_fk(connect)
    _run(["new", "demo", "--template", "first_test"])
    _run(["validate", "demo"])
    _run(["compile", "demo"])
    code = _run(["upload", "demo", "--ip", motionctl.SIM_IP])
    assert code == 1


def test_upload_calls_upload_file_then_load_tpd(connect, _redirect_dirs, monkeypatch):
    """The real upload wire protocol (FileUpload + port 20010 + MD5 verify) is
    covered by sw/tests/test_robot_fr5.py against a real local listener.
    Here we only need the call *sequence*: upload_file, then LoadTPD with the
    matching start pose, so FR5.upload_file is stubbed directly."""
    _redirect_dirs_, build = _redirect_dirs
    _seed_ik_fk(connect)
    _run(["new", "demo", "--template", "first_test"])
    _run(["validate", "demo"])
    _run(["compile", "demo"])

    out = build / "motion_demo.txt"
    tpd = tpd_file.read(out)
    connect.tpd_start_poses["traj/motion_demo"] = tpd.poses[0]

    calls = []

    def fake_upload_file(self, local_path, file_type=20):
        calls.append((file_type, local_path))
        connect.file_upload_calls.append((file_type, "motion_demo.txt"))
        return "/fruser/traj/motion_demo.txt"

    monkeypatch.setattr(fr5.FR5, "upload_file", fake_upload_file)

    code = _run(["upload", "demo", "--ip", motionctl.ROBOT_IP])
    assert code == 0
    assert connect.file_upload_calls == [(20, "motion_demo.txt")]
    assert connect.tpd_loaded == "traj/motion_demo"
    assert calls == [(20, str(out))]


# -- play ---------------------------------------------------------------


def test_play_refuses_the_sim_ip(connect, _redirect_dirs):
    _seed_ik_fk(connect)
    _run(["new", "demo", "--template", "first_test"])
    _run(["validate", "demo"])
    _run(["compile", "demo"])
    code = _run(["play", "demo", "--ip", motionctl.SIM_IP])
    assert code == 1


def test_play_without_matching_remote_md5_refuses(connect, _redirect_dirs):
    _seed_ik_fk(connect)
    _run(["new", "demo", "--template", "first_test"])
    _run(["validate", "demo"])
    _run(["compile", "demo"])
    # No file registered in connect.files, so file_md5 returns None.
    code = _run(["play", "demo", "--ip", motionctl.ROBOT_IP])
    assert code == 1
    assert connect.move_j_calls == []
    assert connect.move_tpd_calls == []


def test_play_aborts_without_yes_and_never_moves(connect, _redirect_dirs, monkeypatch):
    _redirect_dirs_, build = _redirect_dirs
    _seed_ik_fk(connect)
    _run(["new", "demo", "--template", "first_test"])
    _run(["validate", "demo"])
    _run(["compile", "demo"])
    out = build / "motion_demo.txt"
    data = out.read_bytes()
    connect.files["/fruser/traj/motion_demo.txt"] = data
    tpd = tpd_file.read(out)
    connect.tpd_start_poses["traj/motion_demo"] = tpd.poses[0]

    monkeypatch.setattr("builtins.input", lambda prompt="": "no")
    code = _run(["play", "demo", "--ip", motionctl.ROBOT_IP])
    assert code == 0  # aborting is not an error
    assert connect.move_j_calls == []
    assert connect.move_tpd_calls == []


def test_play_with_yes_calls_mode_movej_movetpd_in_order(
    connect, _redirect_dirs, monkeypatch
):
    _redirect_dirs_, build = _redirect_dirs
    _seed_ik_fk(connect)
    _run(["new", "demo", "--template", "first_test"])
    _run(["validate", "demo"])
    _run(["compile", "demo"])
    out = build / "motion_demo.txt"
    connect.files["/fruser/traj/motion_demo.txt"] = out.read_bytes()
    tpd = tpd_file.read(out)
    connect.tpd_start_poses["traj/motion_demo"] = tpd.poses[0]
    connect.robot_state_pkg.motion_done = 1  # completes immediately

    inputs = iter(["YES", ""])
    monkeypatch.setattr("builtins.input", lambda prompt="": next(inputs))
    monkeypatch.setattr(motionctl.time, "sleep", lambda s: None)

    code = _run(["play", "demo", "--ip", motionctl.ROBOT_IP, "--ovl", "50"])
    assert code == 0

    names = [c[0] for c in connect.calls]
    assert "Mode" in names
    assert "MoveJ" in names
    assert "MoveTPD" in names
    assert names.index("Mode") < names.index("MoveJ") < names.index("MoveTPD")
    # global speed = the playback speed, MoveTPD at 100% (speeds multiply)
    assert connect.move_tpd_calls == [("traj/motion_demo", 1, 100)]  # smoothed
    assert 50 in connect.speeds
    assert connect.speeds[-1] == 10  # safe default restored afterwards


def test_play_ctrl_c_calls_stop_motion(connect, _redirect_dirs, monkeypatch):
    _redirect_dirs_, build = _redirect_dirs
    _seed_ik_fk(connect)
    _run(["new", "demo", "--template", "first_test"])
    _run(["validate", "demo"])
    _run(["compile", "demo"])
    out = build / "motion_demo.txt"
    connect.files["/fruser/traj/motion_demo.txt"] = out.read_bytes()
    tpd = tpd_file.read(out)
    connect.tpd_start_poses["traj/motion_demo"] = tpd.poses[0]

    def raise_interrupt(prompt=""):
        raise KeyboardInterrupt()

    monkeypatch.setattr("builtins.input", raise_interrupt)
    code = _run(["play", "demo", "--ip", motionctl.ROBOT_IP])
    assert code == 0
    assert "StopMotion" in [c[0] for c in connect.calls]


# -- interpolation: joint, and fit ---------------------------------------------

JOINT_DESIGN = {
    "name": "crane",
    "tool_id": 0,
    "interpolation": "joint",
    "sample_period_ms": 8,
    "seed_joints": [0, -90, 90, -90, -90, 0],
    "orientation": {"mode": "angles"},
    "workspace": {"min": [0, -1000, 0], "max": [1000, 1000, 1000]},
    "keyframes": [
        {"t": 0.0, "pos": [600, 0, 300], "pan": 0, "tilt": 0},
        {"t": 4.0, "pos": [600, 0, 800], "pan": 0, "tilt": 0},
    ],
}


def _write_design(motions, raw):
    import yaml

    folder = motions / raw["name"]
    folder.mkdir(parents=True)
    (folder / "design.yaml").write_text(yaml.safe_dump(raw, sort_keys=False))
    return folder / "design.yaml"


def test_joint_design_validates_and_compiles_fk_poses(connect, _redirect_dirs):
    motions, build = _redirect_dirs
    _seed_ik_fk(connect)
    _write_design(motions, JOINT_DESIGN)
    assert _run(["validate", "crane"]) == 0
    data = json.loads((motions / "crane" / "validated.json").read_text())
    assert data["interpolation"] == "joint"
    assert _run(["compile", "crane"]) == 0
    tpd = tpd_file.read(build / "motion_crane.txt")
    assert tpd.num_points == data["num_points"] == 501
    for got, want in zip(tpd.joints, data["samples"]["joints"], strict=True):
        assert got == pytest.approx(want, abs=1e-4)
    # pose columns are FK of the joints (here the fake's exact inverse)
    assert tpd.poses[0] == pytest.approx(_joints_to_pose(tpd.joints[0]), abs=1e-3)


def test_fit_write_rescales_keyframe_times_to_the_limits(connect, _redirect_dirs):
    import yaml

    motions, _build = _redirect_dirs
    _seed_ik_fk(connect)
    raw = dict(JOINT_DESIGN, limits={"max_tcp_speed_mm_s": 100.0})
    path = _write_design(motions, raw)
    assert _run(["validate", "crane"]) == 1  # too fast for 100 mm/s
    assert _run(["fit", "crane", "--write", "--margin", "1.0"]) == 0
    t_end = yaml.safe_load(path.read_text())["keyframes"][-1]["t"]
    assert t_end > 4.0
    assert _run(["validate", "crane"]) == 0


def test_fit_refuses_when_a_non_timing_check_fails(connect, _redirect_dirs):
    motions, _build = _redirect_dirs
    _seed_ik_fk(connect)
    raw = dict(JOINT_DESIGN, tool_id=3)  # active tool is 0: check_active_tool fails
    _write_design(motions, raw)
    assert _run(["fit", "crane", "--write"]) == 1


# -- new --tool/--interpolation, capture ----------------------------------------


def test_new_sets_tool_and_interpolation(_redirect_dirs):
    motions, _build = _redirect_dirs
    _run(
        [
            "new",
            "demo",
            "--template",
            "first_test",
            "--tool",
            "1",
            "--interpolation",
            "joint",
        ]
    )
    d = schema.load_design(motions / "demo" / "design.yaml")
    assert (d.tool_id, d.interpolation) == (1, "joint")


def _capture_setup(connect, motions, interpolation="joint"):
    import yaml

    _seed_ik_fk(connect)
    args = ["new", "demo", "--template", "first_test", "--interpolation", interpolation]
    _run(args)
    path = motions / "demo" / "design.yaml"
    return path, yaml


def test_capture_reset_writes_one_leveled_keyframe_and_the_seed(
    connect, _redirect_dirs
):
    motions, _build = _redirect_dirs
    path, yaml = _capture_setup(connect, motions)
    joints = _pose_to_joints([600.0, 0.0, 300.0, -90.0, 0.0, -90.0])
    connect.robot_state_pkg.jt_cur_pos = joints
    assert _run(["capture", "demo", "--t", "0", "--reset"]) == 0
    raw = yaml.safe_load(path.read_text())
    assert len(raw["keyframes"]) == 1
    kf = raw["keyframes"][0]
    assert kf["t"] == 0.0 and kf["pos"] == pytest.approx([600, 0, 300], abs=1e-3)
    assert "seed" in kf
    assert raw["seed_joints"] == pytest.approx(kf["seed"])


def test_capture_appends_and_replaces_by_time_then_validates(connect, _redirect_dirs):
    motions, _build = _redirect_dirs
    path, yaml = _capture_setup(connect, motions)
    level_start = _pose_to_joints([600.0, 0.0, 300.0, -90.0, 0.0, -90.0])
    level_end = _pose_to_joints([600.0, 0.0, 800.0, -90.0, 0.0, -90.0])
    _run(["capture", "demo", "--t", "0", "--reset", "--joints", *map(str, level_start)])
    _run(["capture", "demo", "--t", "4", "--joints", *map(str, level_end)])
    _run(["capture", "demo", "--t", "4", "--joints", *map(str, level_end)])  # replace
    raw = yaml.safe_load(path.read_text())
    assert [k["t"] for k in raw["keyframes"]] == [0.0, 4.0]
    assert _run(["validate", "demo"]) == 0


def test_capture_levels_a_rolled_pose(connect, _redirect_dirs):
    from scipy.spatial.transform import Rotation

    from sw.robot.pose import camera_angles, level_rotation, rotation_to_sdk_euler

    motions, _build = _redirect_dirs
    path, yaml = _capture_setup(connect, motions)
    rot = level_rotation(10, -5) @ Rotation.from_euler("z", 8, degrees=True).as_matrix()
    joints = _pose_to_joints([600.0, 0.0, 300.0, *rotation_to_sdk_euler(rot)])
    assert _run(["capture", "demo", "--t", "0", "--reset", "--joints",
                 *map(str, joints)]) == 0  # fmt: skip
    kf = yaml.safe_load(path.read_text())["keyframes"][0]
    assert (kf["pan"], kf["tilt"]) == pytest.approx((10, -5), abs=1e-3)
    seeded_pose = _joints_to_pose(kf["seed"])
    assert abs(camera_angles(*seeded_pose[3:])[2]) < 1e-3  # the seed is level


def test_capture_refuses_when_the_active_tool_differs(connect, _redirect_dirs):
    motions, _build = _redirect_dirs
    _capture_setup(connect, motions)
    connect.robot_state_pkg.tool = 1  # design uses tool 0
    assert _run(["capture", "demo", "--t", "0", "--reset"]) == 1


def test_capture_in_cartesian_mode_writes_no_seed(connect, _redirect_dirs):
    motions, _build = _redirect_dirs
    path, yaml = _capture_setup(connect, motions, interpolation="cartesian")
    connect.robot_state_pkg.jt_cur_pos = _pose_to_joints([600.0, 0, 300, -90, 0, -90])
    assert _run(["capture", "demo", "--t", "0", "--reset"]) == 0
    kf = yaml.safe_load(path.read_text())["keyframes"][0]
    assert "seed" not in kf


# -- publish and list --------------------------------------------------------------


def _published_setup(connect, motions, monkeypatch):
    """A validated joint design plus an upload stub that 'stores' the file."""
    _seed_ik_fk(connect)
    _write_design(motions, JOINT_DESIGN)
    assert _run(["validate", "crane"]) == 0

    def fake_upload(self, local_path, file_type=20):
        from pathlib import Path

        data = Path(local_path).read_bytes()
        connect.files[f"/fruser/traj/{Path(local_path).name}"] = data
        connect.tpd_start_poses["traj/motion_crane"] = tpd_file.read(local_path).poses[
            0
        ]
        return f"/fruser/traj/{Path(local_path).name}"

    monkeypatch.setattr(fr5.FR5, "upload_file", fake_upload)


def test_publish_writes_a_consistent_motion_json(connect, _redirect_dirs, monkeypatch):
    motions, build = _redirect_dirs
    _published_setup(connect, motions, monkeypatch)
    assert _run(["publish", "crane", "--ovl", "40", "--ip", motionctl.ROBOT_IP]) == 0
    config = schema.load_motion_config(motions / "crane" / "motion.json")
    validated = json.loads((motions / "crane" / "validated.json").read_text())
    assert config.md5 == tpd_file.md5(build / "motion_crane.txt")
    assert config.approved_ovl == 40
    assert list(config.start_joints) == validated["samples"]["joints"][0]
    assert config.tpd_name == "traj/motion_crane"
    assert not (motions / "crane" / "motion.json.tmp").exists()


def test_publish_refuses_the_sim_ip_and_a_stale_design(
    connect, _redirect_dirs, monkeypatch
):
    motions, _build = _redirect_dirs
    _published_setup(connect, motions, monkeypatch)
    assert _run(["publish", "crane", "--ovl", "40", "--ip", motionctl.SIM_IP]) == 1
    design = motions / "crane" / "design.yaml"
    design.write_text(design.read_text() + "\n# edited\n")
    assert _run(["publish", "crane", "--ovl", "40", "--ip", motionctl.ROBOT_IP]) == 1
    assert not (motions / "crane" / "motion.json").exists()


def test_publish_skip_upload_checks_a_manual_upload_by_md5(
    connect, _redirect_dirs, monkeypatch
):
    motions, build = _redirect_dirs
    _published_setup(connect, motions, monkeypatch)
    args = [
        "publish",
        "crane",
        "--ovl",
        "40",
        "--skip-upload",
        "--ip",
        motionctl.ROBOT_IP,
    ]
    assert _run(args) == 1  # nothing on the controller yet
    motionctl.compile_motion("crane")
    out = build / "motion_crane.txt"
    connect.files["/fruser/traj/motion_crane.txt"] = out.read_bytes()
    connect.tpd_start_poses["traj/motion_crane"] = tpd_file.read(out).poses[0]
    assert _run(args) == 0
    assert connect.file_upload_calls == []


def test_list_shows_published_and_draft_motions(
    connect, _redirect_dirs, monkeypatch, capsys
):
    motions, _build = _redirect_dirs
    _published_setup(connect, motions, monkeypatch)
    _run(["publish", "crane", "--ovl", "40", "--ip", motionctl.ROBOT_IP])
    _run(["new", "draft", "--template", "first_test"])
    capsys.readouterr()
    assert _run(["list", "--check", "--ip", motionctl.ROBOT_IP]) == 0
    out = capsys.readouterr().out
    assert "crane:" in out and "[controller file OK]" in out
    assert "draft: not published" in out


# -- kinematics guard and play's completion wait ------------------------------------


def _compiled_crane(connect, motions):
    _seed_ik_fk(connect)
    _write_design(motions, JOINT_DESIGN)
    assert _run(["validate", "crane"]) == 0
    assert _run(["compile", "crane"]) == 0


def test_upload_refuses_a_file_from_other_kinematics(connect, _redirect_dirs, capsys):
    motions, _build = _redirect_dirs
    _compiled_crane(connect, motions)

    def shifted_fk(joint_pos):  # this "robot" puts the camera 28 mm elsewhere
        pose = _joints_to_pose(joint_pos)
        return 0, [pose[0] + 28.0, *pose[1:]]

    connect.GetForwardKin = shifted_fk
    capsys.readouterr()
    assert _run(["upload", "crane", "--ip", motionctl.ROBOT_IP]) == 1
    assert "another controller's kinematics" in capsys.readouterr().err
    assert connect.file_upload_calls == []


def _ready_to_play(connect, motions, build, monkeypatch):
    _compiled_crane(connect, motions)
    out = build / "motion_crane.txt"
    connect.files["/fruser/traj/motion_crane.txt"] = out.read_bytes()
    connect.tpd_start_poses["traj/motion_crane"] = tpd_file.read(out).poses[0]
    connect.robot_state_pkg.robot_mode = 0
    inputs = iter(["YES", ""])
    monkeypatch.setattr("builtins.input", lambda prompt="": next(inputs))
    monkeypatch.setattr("sw.robot.fr5.time.sleep", lambda s: None)


def test_play_waits_for_the_motion_to_start_and_finish(
    connect, _redirect_dirs, monkeypatch, capsys
):
    motions, build = _redirect_dirs
    _ready_to_play(connect, motions, build, monkeypatch)
    capsys.readouterr()
    assert _run(["play", "crane", "--ip", motionctl.ROBOT_IP, "--ovl", "100"]) == 0
    assert "Motion finished" in capsys.readouterr().out


def test_play_reports_and_stops_on_a_robot_error(
    connect, _redirect_dirs, monkeypatch, capsys
):
    motions, build = _redirect_dirs
    _ready_to_play(connect, motions, build, monkeypatch)
    connect.move_tpd_script = [0]
    real_move_tpd = connect.MoveTPD

    def failing_move_tpd(name, blend, ovl):  # the controller faults once playing
        rtn = real_move_tpd(name, blend, ovl)
        connect.robot_state_pkg.main_code, connect.robot_state_pkg.sub_code = 1, 10
        return rtn

    connect.MoveTPD = failing_move_tpd
    capsys.readouterr()
    _run(["play", "crane", "--ip", motionctl.ROBOT_IP, "--ovl", "100"])
    out = capsys.readouterr().out
    assert "main 1, sub 10" in out
    assert "StopMotion" in [c[0] for c in connect.calls]


def test_play_no_smooth_passes_blend_0_to_move_tpd(
    connect, _redirect_dirs, monkeypatch
):
    motions, build = _redirect_dirs
    _ready_to_play(connect, motions, build, monkeypatch)
    args = ["play", "crane", "--ip", motionctl.ROBOT_IP, "--ovl", "100", "--no-smooth"]
    assert _run(args) == 0
    assert connect.move_tpd_calls == [("traj/motion_crane", 0, 100)]


# -- portrait (D11) -----------------------------------------------------------------

WRIST_SEED = [0.0, 0.0, 0.0, 0.0, -90.0, 0.0]

PORTRAIT_DESIGN = {
    "name": "port",
    "format": "portrait",
    "tool_id": 1,
    "interpolation": "joint",
    "sample_period_ms": 8,
    "seed_joints": WRIST_SEED,
    "orientation": {"mode": "angles"},
    "workspace": {"min": [-1000, -1500, -1000], "max": [1500, 1500, 1500]},
    "keyframes": [
        {"t": 0.0, "pos": [600, -400, 450], "pan": 20, "tilt": -5},
        {"t": 3.0, "pos": [600, 400, 650], "pan": -20, "tilt": 10},
    ],
}


def _seed_wrist(client, kin=None):
    """Make the fake controller use WristFK kinematics, tool 1 active."""
    kin = kin or WristFK()
    client.GetInverseKinRef = lambda type, desc_pos, ref: (0, kin.ik(desc_pos, ref))
    client.GetForwardKin = lambda joints: (0, kin.fk(joints))
    client.robot_state_pkg.tool = 1
    return kin


def _wrist_joints(kin, pan, tilt, roll, pos=(600.0, 0.0, 400.0)):
    return kin.ik([*pos, *rotation_to_sdk_euler(level_rotation(pan, tilt, roll))],
                  WRIST_SEED)  # fmt: skip


def _roll_of(kin, joints):
    return camera_angles(*kin.fk(joints)[3:])[2]


def test_new_with_format_portrait(_redirect_dirs):
    motions, _build = _redirect_dirs
    _run(["new", "p", "--template", "first_test", "--tool", "1", "--format",
          "portrait"])  # fmt: skip
    assert schema.load_design(motions / "p" / "design.yaml").format == "portrait"


def test_capture_on_a_portrait_design_levels_to_roll_90(
    connect, _redirect_dirs, capsys
):
    import yaml

    motions, _build = _redirect_dirs
    kin = _seed_wrist(connect)
    _run(["new", "p", "--template", "first_test", "--tool", "1", "--interpolation",
          "joint", "--format", "portrait"])  # fmt: skip
    connect.robot_state_pkg.jt_cur_pos = _wrist_joints(kin, 10, -5, 80.0)
    assert _run(["capture", "p", "--t", "0", "--reset"]) == 0
    kf = yaml.safe_load((motions / "p" / "design.yaml").read_text())["keyframes"][0]
    assert _roll_of(kin, kf["seed"]) == pytest.approx(90.0, abs=1e-3)
    assert (kf["pan"], kf["tilt"]) == pytest.approx((10, -5), abs=1e-3)
    assert "roll 90" in capsys.readouterr().out


def test_capture_on_a_landscape_design_still_levels_to_0(connect, _redirect_dirs):
    import yaml

    motions, _build = _redirect_dirs
    kin = _seed_wrist(connect)
    _run(["new", "l", "--template", "first_test", "--tool", "1", "--interpolation",
          "joint"])  # fmt: skip
    connect.robot_state_pkg.jt_cur_pos = _wrist_joints(kin, 10, -5, 6.0)
    assert _run(["capture", "l", "--t", "0", "--reset"]) == 0
    kf = yaml.safe_load((motions / "l" / "design.yaml").read_text())["keyframes"][0]
    assert _roll_of(kin, kf["seed"]) == pytest.approx(0.0, abs=1e-3)


def test_pose_with_a_design_shows_roll_relative_to_its_format(
    connect, _redirect_dirs, capsys
):
    motions, _build = _redirect_dirs
    kin = _seed_wrist(connect)
    _write_design(motions, PORTRAIT_DESIGN)
    connect.robot_state_pkg.jt_cur_pos = _wrist_joints(kin, 0, 0, 87.0)
    assert _run(["pose", "port"]) == 0
    out = capsys.readouterr().out
    assert "port wants roll 90: off by -3.000deg" in out
    assert _run(["pose"]) == 0
    out = capsys.readouterr().out
    assert "landscape roll error 87.000deg" in out
    assert "portrait roll error -3.000deg" in out


def _validated_portrait(connect, motions, kin=None):
    kin = _seed_wrist(connect, kin)
    _write_design(motions, PORTRAIT_DESIGN)
    assert _run(["validate", "port"]) == 0
    return kin


def test_compile_copies_validated_poses_without_controller_calls(
    connect, _redirect_dirs
):
    motions, build = _redirect_dirs
    _validated_portrait(connect, motions)
    validated = json.loads((motions / "port" / "validated.json").read_text())
    assert validated["format"] == "portrait"
    calls_before = len(connect.calls)
    assert _run(["compile", "port"]) == 0
    assert len(connect.calls) == calls_before  # compile never talks to the robot
    tpd = tpd_file.read(build / "motion_port.txt")
    for got, want in zip(tpd.poses, validated["samples"]["pose"], strict=True):
        assert got == pytest.approx(want, abs=1e-4)


def test_compile_and_publish_refuse_a_format_mismatch(
    connect, _redirect_dirs, monkeypatch
):
    motions, _build = _redirect_dirs
    _validated_portrait(connect, motions)
    path = motions / "port" / "validated.json"
    data = json.loads(path.read_text())
    data["format"] = "landscape"
    path.write_text(json.dumps(data))
    assert _run(["compile", "port"]) == 1
    assert _run(["publish", "port", "--ovl", "50", "--ip", motionctl.ROBOT_IP]) == 1
    data["format"], data["tool_id"] = "portrait", 2
    path.write_text(json.dumps(data))
    assert _run(["compile", "port"]) == 1


def test_publish_writes_format_from_validated_json(
    connect, _redirect_dirs, monkeypatch
):
    from pathlib import Path

    motions, _build = _redirect_dirs
    _validated_portrait(connect, motions)

    def fake_upload(self, local_path, file_type=20):
        connect.files[f"/fruser/traj/{Path(local_path).name}"] = Path(
            local_path
        ).read_bytes()
        connect.tpd_start_poses["traj/motion_port"] = tpd_file.read(local_path).poses[0]
        return f"/fruser/traj/{Path(local_path).name}"

    monkeypatch.setattr(fr5.FR5, "upload_file", fake_upload)
    assert _run(["publish", "port", "--ovl", "60", "--ip", motionctl.ROBOT_IP]) == 0
    config = schema.load_motion_config(motions / "port" / "motion.json")
    assert config.format == "portrait"
    assert config.tool_id == 1


def test_kinematics_guard_compares_rotations(connect, _redirect_dirs):
    motions, build = _redirect_dirs
    _validated_portrait(connect, motions)
    motionctl.compile_motion("port")
    out = build / "motion_port.txt"
    robot = fr5.FR5(connect, motionctl.ROBOT_IP)
    # The same rotations written as other Euler triples: accepted.
    _seed_wrist(connect, WristFK(alternate_euler=True))
    tpd = tpd_file.read(out)
    assert connect.GetForwardKin(tpd.joints[0])[1][3:] != pytest.approx(
        tpd.poses[0][3:], abs=1e-3
    )
    motionctl._check_same_kinematics(robot, out)
    # 0.2 deg of real rotation error, or 2 mm: refused.
    _seed_wrist(connect, WristFK(fk_rot_offset_deg=0.2))
    with pytest.raises(motionctl.CommandError):
        motionctl._check_same_kinematics(robot, out)
    _seed_wrist(connect, WristFK(offset_mm=(102.0, 0.0, 150.0)))
    with pytest.raises(motionctl.CommandError):
        motionctl._check_same_kinematics(robot, out)


# -- derive --------------------------------------------------------------------------

LANDSCAPE_SOURCE = {
    **PORTRAIT_DESIGN,
    "name": "land",
    "format": "landscape",
    "ramp_fraction": 0.2,
    "anchor": {"pos": [100, 50, 0], "yaw_deg": 30},
    "limits": {"max_tcp_speed_mm_s": 1200},
    "keyframes": [
        {"t": 0.0, "pos": [500, -400, 450], "pan": -10, "tilt": -5},
        {"t": 2.0, "pos": [500, 0, 500], "pan": -20, "tilt": 0, "stop": True},
        {"t": 4.0, "pos": [500, 400, 650], "pan": -40, "tilt": 10},
    ],
}


def _derive(args):
    return _run(["derive", "drv", "--from", "land", "--format", "portrait", *args])


def test_derive_applies_the_wrist_guess_and_levels_to_portrait(connect, _redirect_dirs):
    import yaml

    from sw.robot import frames
    from sw.robot import validate as validate_module

    motions, _build = _redirect_dirs
    kin = _seed_wrist(connect)
    _write_design(motions, LANDSCAPE_SOURCE)
    assert _derive(["--wrist", "10", "0"]) == 0
    raw = yaml.safe_load((motions / "drv" / "design.yaml").read_text())
    source = schema.load_design(motions / "land" / "design.yaml")
    _, src_joints = validate_module.check_keyframe_ik(source, kin)
    derived = schema.load_design(motions / "drv" / "design.yaml")
    base = frames.design_to_base(derived)

    assert raw["format"] == "portrait"
    for key in ("ramp_fraction", "limits", "workspace", "anchor", "tool_id"):
        assert raw[key] == LANDSCAPE_SOURCE[key]
    assert [k["t"] for k in raw["keyframes"]] == [0.0, 2.0, 4.0]
    assert [k.get("stop", False) for k in raw["keyframes"]] == [False, True, False]
    assert raw["seed_joints"] == raw["keyframes"][0]["seed"]
    for kf, kf_base, src in zip(raw["keyframes"], base.keyframes, src_joints,
                                strict=True):  # fmt: skip
        assert _roll_of(kin, kf["seed"]) == pytest.approx(90.0, abs=1e-3)
        assert kf["seed"][:3] == pytest.approx(src[:3], abs=1e-4)  # J1-J3 kept
        fk = kin.fk(kf["seed"])
        # the anchor is applied once: authoring pos -> base equals the FK pose
        assert kf_base.pos == pytest.approx(fk[:3], abs=1e-2)
        # J4 +10 turned the view 10deg (J4 is about base z in WristFK)
        src_pan = camera_angles(*kin.fk(src)[3:])[0]
        assert camera_angles(*fk[3:])[0] == pytest.approx(src_pan + 10, abs=1e-2)


def test_derive_resolves_joints_keyframes_as_given(connect, _redirect_dirs):
    import yaml

    motions, _build = _redirect_dirs
    kin = _seed_wrist(connect)
    raw = copy.deepcopy(LANDSCAPE_SOURCE)
    joints = _wrist_joints(kin, -40, 10, 0.0, pos=(500, 400, 650))
    raw["keyframes"][2] = {"t": 4.0, "joints": joints}
    _write_design(motions, raw)
    assert _derive(["--wrist", "0", "0"]) == 0
    kf = yaml.safe_load((motions / "drv" / "design.yaml").read_text())["keyframes"][2]
    assert kf["seed"][:3] == pytest.approx(joints[:3], abs=1e-4)
    assert _roll_of(kin, kf["seed"]) == pytest.approx(90.0, abs=1e-3)


def test_derive_refusals(connect, _redirect_dirs):
    motions, _build = _redirect_dirs
    _seed_wrist(connect)
    cart = {**LANDSCAPE_SOURCE, "interpolation": "cartesian"}
    cart.pop("ramp_fraction")
    _write_design(motions, cart)
    assert _derive(["--wrist", "0", "0"]) == 1  # cartesian source
    assert not (motions / "drv").exists()
    _write_design(motions, {**PORTRAIT_DESIGN, "name": "drv"})
    import shutil

    shutil.rmtree(motions / "land")
    _write_design(motions, LANDSCAPE_SOURCE)
    assert _derive(["--wrist", "0", "0"]) == 1  # target already exists
    with pytest.raises(SystemExit):
        _derive([])  # --wrist is required


def test_derive_output_is_a_draft_that_may_fail_validation(connect, _redirect_dirs):
    motions, _build = _redirect_dirs
    _seed_wrist(connect)
    raw = {**LANDSCAPE_SOURCE, "workspace": {"min": [0, -500, 0], "max": [1, 1, 1]}}
    _write_design(motions, raw)
    assert _derive(["--wrist", "0", "0"]) == 0
    assert (motions / "drv" / "design.yaml").exists()
    assert _run(["validate", "drv"]) == 1  # the original box rejects the path


def test_capture_warns_about_keyframes_left_from_the_template(
    connect, _redirect_dirs, capsys
):
    motions, _build = _redirect_dirs
    _capture_setup(connect, motions)  # first_test template: keyframes at 0 and 4
    connect.robot_state_pkg.jt_cur_pos = _pose_to_joints([600.0, 0, 300, -90, 0, -90])
    assert _run(["capture", "demo", "--t", "0"]) == 0  # no --reset
    out = capsys.readouterr().out
    assert "keyframe(s) at t=4 did not come from capture" in out
    assert _run(["capture", "demo", "--t", "0", "--reset"]) == 0
    assert "did not come from capture" not in capsys.readouterr().out


# -- luaplay ------------------------------------------------------------------


def _luaplay_ready(connect, monkeypatch):
    """A compiled 'demo' and an FR5.upload_file stub that records the upload."""
    _seed_ik_fk(connect)
    _run(["new", "demo", "--template", "first_test"])
    _run(["validate", "demo"])
    _run(["compile", "demo"])
    uploads = []

    def fake_upload_file(self, local_path, file_type=20):
        uploads.append((file_type, local_path))
        return "/fruser/motion_demo.lua"

    monkeypatch.setattr(fr5.FR5, "upload_file", fake_upload_file)
    return uploads


def _script_program_motion(connect, _redirect_dirs, *, state=None):
    """Make ProgramRun move the fake arm from the start joints to the end
    joints (20 readings), then stand still there. `state` overrides the
    program-state script."""
    motions, _build = _redirect_dirs
    joints = json.loads((motions / "demo" / "validated.json").read_text())
    start, end = joints["samples"]["joints"][0], joints["samples"]["joints"][-1]
    path = [
        [a + (b - a) * k / 20 for a, b in zip(start, end, strict=True)]
        for k in range(21)
    ]
    real_run = connect.ProgramRun

    def run():
        code = real_run()
        connect.robot_state_pkg.joints_script = [list(start)] + path
        if state is not None:
            connect.robot_state_pkg.program_script = list(state)
        return code

    connect.ProgramRun = run


def test_luaplay_refuses_the_sim_ip(connect, _redirect_dirs):
    assert _run(["luaplay", "demo", "--ip", motionctl.SIM_IP]) == 1


def test_luaplay_uploads_the_program_and_aborts_without_yes(
    connect, _redirect_dirs, monkeypatch
):
    _motions, build = _redirect_dirs
    uploads = _luaplay_ready(connect, monkeypatch)
    monkeypatch.setattr("builtins.input", lambda prompt="": "no")
    assert _run(["luaplay", "demo", "--ovl", "100"]) == 0
    lua = build / "motion_demo.lua"
    assert uploads == [(motionctl.LUA_UPLOAD_TYPE, str(lua))]
    rows = tpd_file.read(build / "motion_demo.txt").joints
    assert lua.read_text().count("\n{") == len(rows)
    assert connect.move_j_calls == []
    assert connect.program_runs == 0


def test_luaplay_with_yes_moves_to_start_then_runs_the_program(
    connect, _redirect_dirs, monkeypatch
):
    _luaplay_ready(connect, monkeypatch)
    _script_program_motion(connect, _redirect_dirs)
    monkeypatch.setattr("builtins.input", lambda prompt="": "YES")
    assert _run(["luaplay", "demo", "--ovl", "50"]) == 0
    names = [c[0] for c in connect.calls]
    assert names.index("Mode") < names.index("MoveJ") < names.index("ProgramLoad")
    assert names.index("ProgramLoad") < names.index("ProgramRun")
    assert connect.programs_loaded == ["/fruser/motion_demo.lua"]
    assert connect.program_runs == 1
    assert connect.program_stops == 0
    assert connect.move_tpd_calls == []
    assert connect.servo_j_calls == []  # nothing streamed from Python


def test_luaplay_stops_the_program_on_a_robot_error(
    connect, _redirect_dirs, monkeypatch
):
    _luaplay_ready(connect, monkeypatch)
    monkeypatch.setattr("builtins.input", lambda prompt="": "YES")
    connect.program_run_script = [2]  # never finishes by itself

    real_run = connect.ProgramRun

    def run_then_fault():
        code = real_run()
        connect.robot_state_pkg.main_code = 1
        connect.robot_state_pkg.sub_code = 24
        return code

    connect.ProgramRun = run_then_fault
    assert _run(["luaplay", "demo", "--ovl", "100"]) == 0
    assert connect.program_stops == 1
    assert ("StopMotion", (), {}) in connect.calls


def test_luaplay_ctrl_c_stops_program_and_motion(connect, _redirect_dirs, monkeypatch):
    _luaplay_ready(connect, monkeypatch)
    answers = iter(["YES"])

    def fake_input(prompt=""):
        try:
            return next(answers)
        except StopIteration:
            raise KeyboardInterrupt from None

    monkeypatch.setattr("builtins.input", fake_input)
    assert _run(["luaplay", "demo"]) == 0
    assert connect.program_runs == 0
    assert connect.program_stops == 1
    assert ("StopMotion", (), {}) in connect.calls


def test_luaplay_ignores_the_toggling_program_state(
    connect, _redirect_dirs, monkeypatch, capsys
):
    """Real robot: the program state toggles 1/2 while the program runs."""
    _luaplay_ready(connect, monkeypatch)
    _script_program_motion(connect, _redirect_dirs, state=[2, 1, 2, 1, 2, 1])
    monkeypatch.setattr("builtins.input", lambda prompt="": "YES")
    assert _run(["luaplay", "demo", "--ovl", "100"]) == 0
    out = capsys.readouterr().out
    assert "Program finished" in out
    assert connect.program_stops == 0


def test_wait_for_program_stops_a_program_that_never_reaches_the_end(client):
    robot = fr5.FR5(client, motionctl.ROBOT_IP)
    client.robot_state_pkg.joints_script = [[0.0] * 6, [1.0] + [0.0] * 5]
    outcome = motionctl._wait_for_program(robot, [5.0] + [0.0] * 5, deadline_s=0.5)
    assert outcome.startswith("Program still running")
    assert client.program_stops == 1
    assert ("StopMotion", (), {}) in client.calls


def test_wait_for_program_stops_on_a_robot_error(client):
    robot = fr5.FR5(client, motionctl.ROBOT_IP)
    client.robot_state_pkg.main_code, client.robot_state_pkg.sub_code = 1, 24
    outcome = motionctl._wait_for_program(robot, [0.0] * 6, deadline_s=5.0)
    assert outcome == "Robot error during the program: main 1, sub 24"
    assert client.program_stops == 1


def test_luaplay_reports_how_long_the_program_ran(
    connect, _redirect_dirs, monkeypatch, capsys
):
    _luaplay_ready(connect, monkeypatch)
    _script_program_motion(connect, _redirect_dirs, state=[1, 2, 2, 2, 2, 1])
    monkeypatch.setattr("builtins.input", lambda prompt="": "YES")
    assert _run(["luaplay", "demo", "--ovl", "100"]) == 0
    assert "(program ran " in capsys.readouterr().out
