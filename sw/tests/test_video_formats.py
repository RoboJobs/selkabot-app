"""Regression tests: the edit pipeline handles every new camera format.

The probes describe the real frame sizes and frame rates from the HERO10
table (sw/camera/hero10_table.py). No video or ffmpeg is needed; the
pipeline's planning and XML code is checked with exact numbers.
"""

from pathlib import Path

import pytest
from PIL import Image

from sw import shotcut_pipeline as p

# (name, probe) for one real mode per ratio, resolution and frame rate family.
FORMATS = [
    ("16:9 5.3K 60", p.Probe(5312, 2988, 60, 1, 600)),
    ("16:9 4K 120", p.Probe(3840, 2160, 120, 1, 1200)),
    ("16:9 2.7K 240", p.Probe(2704, 1520, 240, 1, 2400)),
    ("4:3 4K 60", p.Probe(4000, 3000, 60, 1, 600)),
    ("4:3 2.7K 120", p.Probe(2704, 2028, 120, 1, 1200)),
]
IDS = [name for name, _ in FORMATS]
BASE_FPS = 30  # reference frame rate for the scaling checks


@pytest.mark.parametrize("name, probe", FORMATS, ids=IDS)
def test_clip_range_scales_with_frame_rate(name, probe):
    settings = p.EditSettings(slowdowns=((1, 2),), clip_start=2, clip_length=5)
    base = p.clip_range(p.Probe(1920, 1080, BASE_FPS, 1, 900), settings)
    fps = probe.fps_num / probe.fps_den
    start, end = p.clip_range(probe, settings)
    # The same seconds map to proportionally more frames, within one frame of rounding.
    assert abs(start - base[0] * fps / BASE_FPS) <= 1
    assert abs(end - base[1] * fps / BASE_FPS) <= 1


@pytest.mark.parametrize("name, probe", FORMATS, ids=IDS)
def test_slowdown_windows_are_inside_the_clip_for_every_format(name, probe):
    settings = p.EditSettings(slowdowns=((1, 2),), clip_start=2, clip_length=5)
    clip = p.clip_range(probe, settings)
    for start, end in p.plan_windows(probe, settings, clip):
        assert clip[0] <= start < end <= clip[1]


@pytest.mark.parametrize("name, probe", FORMATS, ids=IDS)
def test_flash_events_are_deterministic_and_inside_the_video(name, probe):
    fps = probe.fps_num / probe.fps_den
    events = p.flash_events(((1.0, 3.0),), fps, probe.total_frames, seed=7)
    assert events == p.flash_events(((1.0, 3.0),), fps, probe.total_frames, seed=7)
    assert events, "a 2 second window should flash at least once"
    assert all(0 <= a < b <= probe.total_frames for a, b, _ in events)


def test_chunk_count_depends_on_duration_not_frame_rate():
    # Chunking splits by seconds, so 10 s of footage gets the same chunks at any fps.
    counts = {
        fps: p._effective_chunks(fps * 10, fps, 4) for fps in (30, 60, 100, 120, 240)
    }
    assert len(set(counts.values())) == 1, counts


def test_portrait_output_profile_swaps_social_dimensions():
    portrait = p.Probe(2160, 3840, 120, 1, 1200)
    landscape = p.Probe(3840, 2160, 120, 1, 1200)
    assert portrait.portrait
    assert p.output_dimensions(portrait, 1920, 1080) == (1080, 1920)
    assert p.output_dimensions(landscape, 1920, 1080) == (1920, 1080)


def test_probe_video_applies_container_rotation(monkeypatch, tmp_path):
    class Result:
        stdout = (
            '{"streams":[{"width":3840,"height":2160,'
            '"r_frame_rate":"120/1","tags":{"rotate":"90"}}],'
            '"format":{"duration":"1.0"}}'
        )

    monkeypatch.setattr(p.subprocess, "run", lambda *args, **kwargs: Result())
    probe = p.probe_video(tmp_path / "portrait.mp4")
    assert (probe.width, probe.height, probe.rotation) == (2160, 3840, 90)


@pytest.mark.parametrize("name, probe", FORMATS, ids=IDS)
def test_project_profile_matches_the_source_format(name, probe, tmp_path, monkeypatch):
    monkeypatch.setattr(p, "_video_frames", lambda _: probe.total_frames)
    monkeypatch.setattr(p, "_watermark_box_height", lambda image, probe: 10.0)
    xml, _ = p.build_project_xml(
        Path("source.mp4"), probe, p.watermarks_for(None), None, None, (0, 50), [], 0.5
    )
    assert f'width="{probe.width}" height="{probe.height}"' in xml
    assert f'frame_rate_num="{probe.fps_num}" frame_rate_den="{probe.fps_den}"' in xml
    assert (
        f'display_aspect_num="{probe.width}" display_aspect_den="{probe.height}"' in xml
    )


@pytest.mark.parametrize("name, probe", FORMATS, ids=IDS)
def test_watermark_box_follows_the_frame_and_logo_shape(
    name, probe, tmp_path, monkeypatch
):
    logo = tmp_path / "logo.png"
    Image.new("RGBA", (200, 100)).save(logo)  # 2:1 logo
    monkeypatch.setattr(p, "_image_size", lambda path: (200, 100))
    expected = p.WATERMARK_SIZE_PCT * (probe.width / probe.height) * 0.5
    assert p._watermark_box_height(logo, probe) == pytest.approx(expected)


def test_slowdown_interpolation_rate_scales_with_the_source(tmp_path, monkeypatch):
    """The minterpolate rate is fps/speed, so a 240 fps source interpolates faster."""

    class Stop(Exception):
        pass

    captured: list[str] = []

    def fake_run(cmd, expected_seconds, progress):
        captured.append(" ".join(cmd))
        raise Stop

    source = tmp_path / "source.mp4"
    source.touch()
    monkeypatch.setattr(p, "_run_ffmpeg", fake_run)
    monkeypatch.setattr(p, "SMOOTH_CACHE_DIR", tmp_path / "cache")
    probe = p.Probe(2704, 1520, 240, 1, 2400)
    with pytest.raises(Stop):
        p.smooth_segment(source, probe, 0, 240, 0.3, cache_dir=tmp_path / "cache")
    assert "minterpolate=fps=800:" in captured[0]
    assert "setpts=PTS/0.3,fps=240/1" in captured[0]
