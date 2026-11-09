import json
import queue
from pathlib import Path

import pytest

from sw import livestream as l


@pytest.mark.parametrize("name", ["", ".", "..", "a/b", "a\\b"])
def test_folder_error(name):
    assert l.folder_error(name)


def test_validation_helpers_and_config(tmp_path):
    assert l.folder_error("good") is None
    assert l.image_error("") is None
    assert l.image_error("dir/x.png")
    cfg = tmp_path / "config.json"
    cfg.write_text(json.dumps({"folder": "clips", "image": "logo.png", "extra": 1}))
    assert l.load_config(cfg)[:2] == ("clips", "logo.png")
    l.save_config(cfg, "new", "", "in.png", "out.png")
    data = json.loads(cfg.read_text())
    assert data["extra"] == 1 and data["folder"] == "new"
    cfg.write_text("not json")
    assert l.load_config(cfg) == ("default", "", "", "")
    cfg.write_text(json.dumps({"folder": "../bad", "image": "dir/x"}))
    assert l.load_config(cfg) == ("default", "", "", "")


def test_check_and_postprocessor(monkeypatch):
    class Response: ok = False; identifier = "x"; status = "bad"
    with pytest.raises(RuntimeError, match="x failed"): l._check(Response())
    events = queue.Queue(); done = []; errors = []
    monkeypatch.setattr(l, "edit_clip", lambda *a, **k: None)
    pp = l.PostProcessor(False, events, done.append, lambda c, e: errors.append(e))
    clip = l.Clip(Path("a"), Path("b"), Path("q"), "k", [], None, None)
    pp.submit(clip); pp.stop(); pp._thread.join(2)
    callback = events.get(timeout=1); callback()
    assert done[0].video == clip.watermarked and not errors


def test_postprocessor_error_and_upload(monkeypatch):
    events = queue.Queue(); done = []; errors = []
    monkeypatch.setattr(l, "edit_clip", lambda *a, **k: (_ for _ in ()).throw(ValueError("bad edit")))
    pp = l.PostProcessor(False, events, done.append, lambda c, e: errors.append((c, e)))
    clip = l.Clip(Path("a"), Path("b"), Path("q"), "k", [], None, None)
    pp.submit(clip); pp.stop(); pp._thread.join(2)
    events.get(timeout=1)()
    assert isinstance(errors[0][1], ValueError)


def test_controller_property_and_check_success(tmp_path):
    l._check(type("Response", (), {"ok": True})())
    controller = object.__new__(l.GoProController)
    controller._gopro = "camera"
    assert controller.gopro == "camera"
    controller._gopro = None
    with pytest.raises(AssertionError): controller.gopro


def test_preview_reader_latest_frame(monkeypatch):
    reader = l.PreviewReader(8554)
    reader._frame = "frame"
    assert reader.latest_frame() == "frame"
    assert reader.latest_frame() is None
