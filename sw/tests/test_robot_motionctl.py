"""Tests for the motionctl CLI, using FakeRobotClient and tmp_path. No hardware."""

from __future__ import annotations

import json

import pytest

from sw.robot import fr5, motionctl, schema, tpd_file
from sw.tests.fakes.fake_kinematics import _joints_to_pose, _pose_to_joints
from sw.tests.fakes.fake_robot import FakeRobotClient


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
    assert connect.move_tpd_calls == [("traj/motion_demo", 0, 100)]
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
