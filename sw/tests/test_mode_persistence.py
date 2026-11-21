"""Tests for saving and loading the video mode in the livestream config file."""

import json
import logging

import pytest

from sw import livestream as l
from sw.camera import hero10_table as table
from sw.camera.mode_persistence import MODE_KEY, load_mode, mode_from_dict, mode_to_dict, save_mode
from sw.camera.modes import VideoMode

CATALOG = table.build_catalog()
WIDE_4K = VideoMode("16:9", "4K", 120, "Wide", "1.4x")


def test_round_trip_restores_the_saved_mode(tmp_path):
    path = tmp_path / "config.json"
    assert save_mode(path, WIDE_4K)
    assert load_mode(path, CATALOG) == WIDE_4K


def test_missing_file_gives_the_default(tmp_path):
    assert load_mode(tmp_path / "absent.json", CATALOG) == CATALOG.default


def test_missing_key_gives_the_default(tmp_path):
    path = tmp_path / "config.json"
    path.write_text(json.dumps({"folder": "x"}), encoding="utf-8")
    assert load_mode(path, CATALOG) == CATALOG.default


def test_unparseable_file_gives_the_default(tmp_path):
    path = tmp_path / "config.json"
    path.write_text("{not json", encoding="utf-8")
    assert load_mode(path, CATALOG) == CATALOG.default


@pytest.mark.parametrize(
    "saved",
    [
        {"ratio": "16:9", "resolution": "5.3K", "fps": 240, "lens": "Wide", "zoom": "none"},  # not in table
        {"ratio": "16:9", "resolution": "4K", "fps": "120", "lens": "Wide", "zoom": "none"},  # wrong type
        {"ratio": "16:9", "resolution": "4K"},  # missing axes
        "16:9 4K",  # not an object
    ],
)
def test_unusable_saved_mode_gives_the_default_and_logs(tmp_path, caplog, saved):
    path = tmp_path / "config.json"
    path.write_text(json.dumps({MODE_KEY: saved}), encoding="utf-8")
    with caplog.at_level(logging.WARNING):
        assert load_mode(path, CATALOG) == CATALOG.default
    assert any(record.levelno == logging.WARNING for record in caplog.records)


def test_save_keeps_other_keys_used_by_the_watermark_tool(tmp_path):
    path = tmp_path / "config.json"
    original = {"folder": "clips", "speed": "0.3", "flash_min_gap": 0.08}
    path.write_text(json.dumps(original), encoding="utf-8")
    save_mode(path, WIDE_4K)
    saved = json.loads(path.read_text(encoding="utf-8"))
    assert {k: saved[k] for k in original} == original
    assert saved[MODE_KEY] == mode_to_dict(WIDE_4K)


def test_save_creates_the_file_when_missing(tmp_path):
    path = tmp_path / "config.json"
    assert save_mode(path, WIDE_4K)
    assert json.loads(path.read_text(encoding="utf-8")) == {MODE_KEY: mode_to_dict(WIDE_4K)}


def test_save_does_not_overwrite_an_unparseable_file(tmp_path):
    path = tmp_path / "config.json"
    path.write_text("{not json", encoding="utf-8")
    assert save_mode(path, WIDE_4K) is False
    assert path.read_text(encoding="utf-8") == "{not json"


def test_livestream_folder_and_image_are_preserved_when_mode_is_saved(tmp_path):
    path = tmp_path / "config.json"
    l.save_config(path, "event", "logo.png", "", "")
    save_mode(path, WIDE_4K)
    assert l.load_config(path) == ("event", "logo.png", "", "")
    assert load_mode(path, CATALOG) == WIDE_4K


def test_saving_livestream_settings_keeps_the_saved_mode(tmp_path):
    path = tmp_path / "config.json"
    save_mode(path, WIDE_4K)
    l.save_config(path, "event", "", "", "")
    assert load_mode(path, CATALOG) == WIDE_4K


def test_mode_from_dict_accepts_only_catalog_modes():
    assert mode_from_dict(mode_to_dict(WIDE_4K), CATALOG) == WIDE_4K
    assert mode_from_dict(None, CATALOG) is None
