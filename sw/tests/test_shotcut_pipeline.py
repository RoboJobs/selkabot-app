import subprocess
from pathlib import Path

import pytest

from sw import shotcut_pipeline as p


def test_edit_settings_validation_and_helpers():
    with pytest.raises(ValueError): p.EditSettings(speed=0)
    with pytest.raises(ValueError): p.EditSettings(smoothing="x")
    assert p._atempo_chain(0.25) == "atempo=0.5,atempo=0.500000"
    assert p._effective_chunks(10, 30, 4) == 1
    assert p._clip_window(10, 100, 10, 2, 5) == (30, 60)
    assert p._clip_window(10, 20, 10, 2, 5) is None


def test_probe_and_deterministic_planning():
    probe = p.Probe(1920, 1080, 30, 1, 900)
    settings = p.EditSettings(slowdowns=((1, 2),), clip_start=2, clip_length=5)
    assert p.clip_range(probe, settings) == (60, 150)
    assert p.plan_windows(probe, p.EditSettings(slowdowns=((1, 2),)), (0, 300)) == [(30, 60)]
    assert p.watermarks_for(None) == [(p.ALWAYS_WATERMARK, p.ALWAYS_CORNERS)]
    assert p.watermarks_for("logo.png")[1] == (Path("logo.png"), p.IMAGE_CORNERS)
    assert 'name="x"' in p._prop("x", "a&b")
    assert "affine" in p._fit_filter("f", 2)
    assert "bottom-right" not in p._watermark_filters("a", "b", 2, "top-left", "0=0", 10.0)


def test_build_project_xml_contains_fades_watermarks_and_slowdowns(tmp_path, monkeypatch):
    probe = p.Probe(100, 50, 10, 1, 100)
    smooth = tmp_path / "smooth.mp4"; smooth.touch()
    monkeypatch.setattr(p, "_video_frames", lambda _: 20)
    monkeypatch.setattr(p, "_watermark_box_height", lambda image, probe: 10.0)  # No real logo.png here
    xml, frames = p.build_project_xml(Path("source.mp4"), probe, p.watermarks_for("logo.png"), Path("in.png"), Path("out.png"), (0, 50), [(10, 20, smooth)], 0.5)
    assert frames > 0
    for fragment in ("fadein_still", "fadeout_still", "slow_0", "logo.png", "qtblend"):
        assert fragment in xml


def test_run_ffmpeg_reports_and_propagates(monkeypatch):
    class Proc:
        stdout = iter(["out_time_us=5000000\n"])
        stderr = type("E", (), {"read": lambda self: ""})()
        def wait(self): return 0
    monkeypatch.setattr(p.subprocess, "Popen", lambda *a, **k: Proc())
    messages = []
    p._run_ffmpeg(["ffmpeg", "-i", "x"], 5, messages.append)
    assert messages == ["99%"]


def test_run_melt_failure(monkeypatch):
    class Out:
        def fileno(self): return 0
    class Proc:
        stdout = Out()
        def wait(self): return 2
    monkeypatch.setattr(p.subprocess, "Popen", lambda *a, **k: Proc())
    monkeypatch.setattr(p.os, "read", lambda fd, n: b"")
    with pytest.raises(subprocess.CalledProcessError): p._run_melt(["melt"], None)


def test_smooth_segment_cache_hit(tmp_path, monkeypatch):
    source = tmp_path / "source.mp4"; source.write_bytes(b"x")
    probe = p.Probe(10, 10, 10, 1, 100)
    monkeypatch.setattr(p, "_render_chunks", lambda *a, **k: [])
    monkeypatch.setattr(p, "_concat_parts", lambda parts, output, work: output.write_bytes(b"rendered"))
    monkeypatch.setattr(p, "_has_audio", lambda _: False)
    monkeypatch.setattr(p, "_video_frames", lambda _: 20)
    first = p.smooth_segment(source, probe, 0, 10, 0.5, tmp_path)
    assert first.exists()
    messages = []
    assert p.smooth_segment(source, probe, 0, 10, 0.5, tmp_path, messages.append) == first
    assert any("reusing" in m for m in messages)
