import argparse
import json
from pathlib import Path

import pytest

from sw import shotcut_watermark as w
from sw.shotcut_pipeline import EditSettings


@pytest.mark.parametrize("text, expected", [("", ()), ("1-2; 3 - 4", ((1.0, 2.0), (3.0, 4.0)))])
def test_parse_slowdowns(text, expected):
    assert w.parse_slowdowns(text) == expected


@pytest.mark.parametrize("text", ["3", "a-b", "2-2", "-1-2"])
def test_parse_slowdowns_rejects_bad_ranges(text):
    with pytest.raises(ValueError):
        w.parse_slowdowns(text)


def test_format_and_optional_float():
    assert w.format_slowdowns(((1, 2.5),)) == "1-2.5"
    assert w._optional_float(" ", "x") is None
    assert w._optional_float(2, "x") == 2
    with pytest.raises(ValueError, match="x"):
        w._optional_float("no", "x")


def test_settings_from_defaults_and_custom_paths():
    assert w.settings_from({}) == EditSettings()
    s = w.settings_from({"slowdowns": "1-2", "speed": "0.5", "chunks": "2", "fadein_image": "in.png", "fadeout_image": "out.png"})
    assert s.slowdowns == ((1.0, 2.0),) and s.fadein_image == Path("in.png")
    with pytest.raises(ValueError, match="smoothing"):
        w.settings_from({"smoothing": "bad"})
    with pytest.raises(ValueError, match="chunks"):
        w.settings_from({"chunks": "1.5"})


def test_config_load_save_merges_and_tolerates_errors(tmp_path):
    cfg = tmp_path / "config.json"
    assert w.load_config(cfg) == {}
    cfg.write_text("not json")
    assert w.load_config(cfg) == {}
    cfg.write_text(json.dumps({"folder": "keep", "speed": 0.2}))
    w.save_config({"speed": 0.4}, cfg)
    assert json.loads(cfg.read_text()) == {"folder": "keep", "speed": 0.4}
    cfg.write_text("[]")
    assert w.load_config(cfg) == {}
    monkey = None


@pytest.mark.parametrize("original, expected", [("x_original.mp4", "x_watermarked.mp4"), ("x.mp4", "x_watermarked.mp4")])
def test_watermarked_name(original, expected):
    assert w.watermarked_name(Path(original)).name == expected


def test_add_watermark_delegates(monkeypatch, tmp_path):
    called = {}
    monkeypatch.setattr(w, "edit_clip", lambda *args, **kwargs: called.update(args=args, kwargs=kwargs) or args[1])
    monkeypatch.setattr(w, "load_config", lambda *a, **k: {})  # No extra watermark image in the config
    out = w.add_watermark(tmp_path / "in.mp4", tmp_path / "out.mp4")
    assert out.name == "out.mp4" and called["args"][2] == w.watermarks_for(None)


def test_batch_skips_existing_and_supports_redo(monkeypatch, tmp_path):
    first = tmp_path / "a_original.mp4"; first.touch()
    existing = tmp_path / "a_watermarked.mp4"; existing.touch()
    second = tmp_path / "b_original.mp4"; second.touch()
    calls = []
    monkeypatch.setattr(w, "add_watermark", lambda *args: calls.append(args) or args[1])
    assert w.batch(tmp_path, EditSettings(), jobs=1) == [second.with_name("b_watermarked.mp4")]
    assert len(calls) == 1
    w.batch(tmp_path, EditSettings(), redo=True, jobs=2)
    assert len(calls) == 3


def test_settings_values_only_overrides_explicit_flags():
    args = argparse.Namespace(**{name: None for name in ("exposure", "slowdowns", "speed", "clip_start", "clip_length", "smoothing", "chunks", "fadein_image", "fadeout_image", "soundtrack", "soundtrack_start")})
    args.speed = 0.7
    assert w._settings_values(args, {"speed": 0.2, "chunks": 3}) == {"speed": 0.7, "chunks": 3}


def test_save_config_write_failure_is_logged(monkeypatch, tmp_path):
    monkeypatch.setattr(Path, "write_text", lambda *a, **k: (_ for _ in ()).throw(OSError("nope")))
    w.save_config({}, tmp_path / "config.json")
