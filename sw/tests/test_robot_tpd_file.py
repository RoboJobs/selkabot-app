"""Tests for the TPD file writer/reader. No hardware."""

from __future__ import annotations

import pytest

from sw.robot import tpd_file

JOINTS = [
    [91.2883, -121.5264, 129.4355, -98.2465, -89.9802, 0.0237],
    [91.0, -121.0, 129.0, -98.0, -89.0, 0.0],
]
POSES = [
    [135.9624, -268.4651, 361.4536, -179.6626, 0.0199, -178.7353],
    [136.0, -268.0, 361.0, -179.0, 0.0, -178.0],
]

# Verified: first data row of a real pendant recording (2ms period, tool 0).
GOLDEN_JOINTS = [91.2883, -121.5264, 129.4355, -98.2465, -89.9802, 0.0237]
GOLDEN_POSE = [135.9624, -268.4651, 361.4536, -179.6626, 0.0199, -178.7353]
GOLDEN_HEADER = "1,1,2,0,0,0"
GOLDEN_ROW = (
    "91.2883,-121.5264,129.4355,-98.2465,-89.9802,0.0237,"
    "135.9624,-268.4651,361.4536,-179.6626,0.0199,-178.7353,2,0,17"
)


def test_write_then_read_round_trips(tmp_path):
    path = tmp_path / "motion_demo.txt"
    tpd_file.write(path, JOINTS, POSES, period_ms=8, tool_id=0)
    result = tpd_file.read(path)
    assert result.num_points == 2
    assert result.period_ms == 8
    assert result.tool_id == 0
    for actual, expected in zip(result.joints, JOINTS, strict=True):
        assert actual == pytest.approx(expected)
    for actual, expected in zip(result.poses, POSES, strict=True):
        assert actual == pytest.approx(expected)


def test_header_and_row_ending_at_period_8(tmp_path):
    path = tmp_path / "motion_demo.txt"
    tpd_file.write(path, JOINTS, POSES, period_ms=8, tool_id=0)
    lines = path.read_text().splitlines()
    assert lines[0] == "2,1,8,0,0,0"
    assert lines[1].endswith(",8,0,17")
    assert lines[2].endswith(",8,0,17")


def test_header_and_row_ending_at_period_2(tmp_path):
    path = tmp_path / "motion_demo.txt"
    tpd_file.write(path, JOINTS, POSES, period_ms=2, tool_id=0)
    lines = path.read_text().splitlines()
    assert lines[0] == "2,1,2,0,0,0"
    assert lines[1].endswith(",2,0,17")


def test_numbers_use_4_decimals(tmp_path):
    path = tmp_path / "motion_demo.txt"
    tpd_file.write(path, [[1.0] * 6], [[2.0] * 6], period_ms=8, tool_id=0)
    line = path.read_text().splitlines()[1]
    assert "1.0000" in line
    assert "2.0000" in line


def test_golden_row_matches_a_real_recording(tmp_path):
    """Writing the verified recording's own first row reproduces it exactly."""
    path = tmp_path / "golden.txt"
    tpd_file.write(path, [GOLDEN_JOINTS], [GOLDEN_POSE], period_ms=2, tool_id=0)
    lines = path.read_text().splitlines()
    assert lines[0] == GOLDEN_HEADER
    assert lines[1] == GOLDEN_ROW


def test_length_mismatch_raises(tmp_path):
    path = tmp_path / "x.txt"
    with pytest.raises(ValueError, match="must match"):
        tpd_file.write(path, JOINTS, POSES[:1], period_ms=8, tool_id=0)


def test_wrong_row_width_raises(tmp_path):
    path = tmp_path / "x.txt"
    with pytest.raises(ValueError, match="expected 6"):
        tpd_file.write(path, [[1.0, 2.0, 3.0]], POSES[:1], period_ms=8, tool_id=0)


def test_read_rejects_wrong_row_width(tmp_path):
    path = tmp_path / "x.txt"
    path.write_text("1,1,8,0,0,0\n1,2,3,4,5,6,7,8,9,10,11,12,8,0\n")  # 14 values
    with pytest.raises(ValueError, match="expected 15"):
        tpd_file.read(path)


def test_read_rejects_empty_file(tmp_path):
    path = tmp_path / "x.txt"
    path.write_text("")
    with pytest.raises(ValueError, match="empty"):
        tpd_file.read(path)


def test_md5_matches_file_bytes(tmp_path):
    path = tmp_path / "x.txt"
    tpd_file.write(path, JOINTS, POSES, period_ms=8, tool_id=0)
    import hashlib

    expected = hashlib.md5(path.read_bytes()).hexdigest()
    assert tpd_file.md5(path) == expected
