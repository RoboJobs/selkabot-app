# Plan: Cascading recording-mode selector (GoPro HERO10) in livestream.py

## Context
`sw/livestream.py` offers 3 hardcoded radio buttons (`RESOLUTIONS`, fixed `FPS=60`, lines 74-81) applied by `GoProController._apply_video_settings` (line ~355). The user wants a cascading menu: **ratio -> resolution -> fps -> digital lens -> zoom**. All options are always visible, but only compatible ones are enabled. Default: 16:9, 2.7K, 60 fps, Wide, no zoom. The design must allow other camera models or brands later. The selection is persisted in `sw/livestream_config.json`. 1080p is dropped.

## Findings
- Tkinter + ttk UI. The UI runs on the Tk thread and the camera on an asyncio thread via `_run()`. Existing change pattern: `_on_resolution` (597-614) applies the change, then reverts the UI on failure.
- Open GoPro SDK (`open_gopro>=0.22`): `http_setting.video_resolution` (2), `frames_per_second` (3), `video_lens` (121). Zoom is a **command**, `http_command.set_digital_zoom(percent=int)`, not a setting.
- On HERO10 the aspect ratio is encoded in the resolution enum (e.g. `NUM_4K_4_3=18`, `NUM_2_7K_4_3=6`), not `video_aspect_ratio` (108). 5.3K 16:9 is `NUM_5_3K=100`.
- `VideoLens`: WIDE=0, NARROW=2, SUPERVIEW=3, LINEAR=4, LINEAR_HORIZON_LEVELING=8.
- `shotcut_pipeline` / `shotcut_watermark` are probe-driven (ffprobe w/h/fps), so ratio and fps changes work. Only concern: `minterpolate` at `smoothing=high` is much slower for 120-240 fps sources (rate = fps/speed).
- Existing tests cover only pure helpers (`sw/tests/test_livestream.py`). The Tk app and camera coroutines are untested, so new logic must be pure and testable. CI (`.github/workflows/tests.yml`) lists `--cov=` per module, so a new module needs a flag.

## Caveats and risks
1. **Compatibility table:** now sourced from the official GoPro page (see *Verified HERO10 catalog* below), so it is no longer a guess. An earlier web-search claim that 5.3K SuperView does not exist was **wrong**; the official table lists it. The probe script (Phase 0) still checks that the camera accepts every entry.
2. **Zoom percent mapping is unknown.** `set_digital_zoom` takes 0-100. The 1.4x and 2x percents are constants (`ZoomOption.percent`), to be calibrated with the probe script (set, then read `gopro/camera/state`).
3. **Setting order and failure:** apply resolution -> fps -> lens -> zoom. Changing resolution may silently alter fps or lens on the camera. After applying, read back the state and verify. On a mismatch, revert the UI and show an error.
4. **Zoom reset:** the camera may reset zoom after a preset reload or resolution change, so always reapply zoom last (including 0 for none).
5. **Preview:** the UDP preview must stop and restart around the change, as `set_resolution` does today. Preview aspect already follows the frame, but `_show_message` assumes 16:9.
6. **Cascade ambiguity:** picking a ratio may invalidate the current resolution (and so on down). Rule: keep the current value if still valid, otherwise pick the first valid one in a fixed preference order (default-closest). Never leave an invalid state.
7. **fps numbering:** user lists "120/100" and "240/200". Treat these as PAL/NTSC variants, 100 and 200 being the PAL rates. Both are shown as selectable fps options (this is a CLI choice, not a region setting).
8. **Persisted config:** it is validated on load (unknown or invalid mode falls back to the default with a warning). `save_config` signature and the `load_config` 4-tuple are used in `main()` and in tests, so extend them without breaking callers. `shotcut_watermark` shares the JSON file, so merge-on-save must keep unknown keys.
9. **Downstream cost:** high fps plus `smoothing=high` makes slowdowns slow. Do not change the pipeline; log or document a warning only.
10. `livestream_config.json` is git-tracked and already modified locally. Do not commit user-specific values from tests.

## Verified HERO10 catalog (source: GoPro HERO10 Black Video Settings and Resolutions)
Zoom applies only to the lenses listed. The page's notes say 1.4x for resolutions 2.7K and above, and 2x for resolutions below 2.7K (16:9) or for all 4:3 modes.

16:9 (all requested rows):

| Resolution | FPS | Lenses | Zoom lenses | Zoom |
|---|---|---|---|---|
| 5.3K | 60/50 | SuperView, Wide, Linear, Linear+HL, Narrow | Wide, Linear, Linear+HL | 1.4x |
| 4K | 120/100 | Wide, Linear, Linear+HL, Narrow | Wide, Linear+HL | 1.4x |
| 2.7K | 240/200 | Wide, Linear, Linear+HL, Narrow | Wide, Linear, Linear+HL | 1.4x |
| 2.7K | 120/100 | SuperView, Wide, Linear, Linear+HL, Narrow | Wide, Linear, Linear+HL | 1.4x |
| 2.7K | 60/50 | SuperView, Wide, Linear, Linear+HL, Narrow | Wide, Linear, Linear+HL | 1.4x |

4:3 (requested rows):

| Resolution | FPS | Lenses | Zoom lenses | Zoom |
|---|---|---|---|---|
| 4K | 60/50 | Wide, Linear, Linear+HL, Narrow | Wide, Linear, Linear+HL | 2x |
| 2.7K | 120/100 | Wide, Linear, Linear+HL, Narrow | Wide, Linear, Linear+HL | 2x |

Consequences of the official table:
- **2x zoom never applies to the requested 16:9 modes.** All of them are 2.7K or above, so only 1.4x exists there. 2x would only appear for 1080p, which is dropped. The zoom axis is therefore None/1.4x for 16:9 and None/2x for 4:3.
- Narrow never allows zoom, and 4K 120/100 also excludes Linear from zoom.
- The 4:3 rows have no SuperView. 4:3 2.7K 60/50 also exists on the camera but was not requested.
- Not requested and not added: 5.3K and 4K at 30/25 and 24 fps, 1080p, 5K 4:3, Looping mode.

## Architecture
New package `sw/camera/` (camera-agnostic core + per-model data and adapters):

- `modes.py` (pure, no hardware or Tk):
  - `VideoMode(ratio, resolution, fps, lens, zoom)`, a frozen dataclass of neutral string/number keys (e.g. `"16:9"`, `"2.7K"`, `60`, `"wide"`, `"none"|"1.4x"|"2x"`).
  - `ModeCatalog(modes: frozenset[VideoMode])` with `options(field, partial_selection) -> list[Option(value, enabled)]` (every value of the field is returned; `enabled` means compatible given the earlier choices), `resolve(selection) -> VideoMode` (cascade correction), `default`, `is_valid`.
  - Ordered axes: `AXES = ("ratio","resolution","fps","lens","zoom")`.
- `base.py`: `CameraBackend` Protocol: `async connect(mode)`, `async apply_mode(mode)`, `catalog: ModeCatalog`, plus the existing start/stop/download pieces over time. `livestream.py` depends only on this.
- `gopro_hero10.py`: the HERO10 catalog as a compact data table (rows of ratio, resolution, fps list, lens list, zoom list) and the mapping VideoMode -> SDK enums (`VideoResolution`, `FramesPerSecond`, `VideoLens`, zoom percent). The only module that imports `open_gopro` setting enums for modes.
- `sw/livestream.py`: `GoProController` implements the backend through the HERO10 module, with `apply_mode` replacing `set_resolution` / `_apply_video_settings`. The UI replaces the radio row with five labelled radio groups (or comboboxes) built from `catalog.options()`. On any change, recompute all five axes, disable the incompatible entries, and apply through `_run` (revert on failure). The status text uses `mode.label` instead of "60 fps".
- Adding a new model or brand later means adding one data module that implements the Protocol. No UI change.

## Deliverable location
`master` is the current branch, so implementation step 0 is: create a feature branch (proposed `gopro-video-modes`), then write this plan to **`docs/design/gopro-video-modes.md`** (named after that branch). Plan mode only allows editing this plan file, so the copy happens right after approval.

## Phases
**Gate rule for every phase:** run `uv run pytest sw/tests/ -q` (full suite, not just the new tests). Every test must pass, with no new skips or failures, before the next phase starts. If anything fails, fix it in the same phase. Each phase ends in its own commit.

### Phase 0 - Data and probe
- **Work:**
  - Fetch the GoPro HERO10 settings page and build the table (ratio, resolution, fps, lenses, zoom) for the requested modes only.
  - Write `sw/camera_probe.py`, a hardware-only script that sets each entry, reads the state back and reports accept/reject. It also finds the zoom percent for 1.4x and 2x.
- Fetch the GoPro HERO10 settings page and build the table for the requested modes only. Write `sw/camera_probe.py`, a hardware-only script that tries each catalog entry and reports accept/reject and the zoom percents.
- Tests (`sw/tests/test_camera_probe.py`, no hardware): report formatting, handling of rejected entries, and the module imports without a camera.
- Gate: full suite green. **User checkpoint:** run the probe on the camera, then correct the catalog (5.3K SuperView, zoom with Narrow, 1.4x/2x percents).

### Phase 1 - Core mode model (`sw/camera/modes.py`)
- **Work:**
  - Create `sw/camera/modes.py` (pure Python, no SDK, no Tk).
  - `VideoMode` frozen dataclass with fields ratio, resolution, fps, lens, zoom.
  - `ModeCatalog` with `options(axis, selection)`, `resolve(selection)` and `default`.
  - Axes ordered ratio, resolution, fps, lens, zoom. Resolve keeps a still-valid value, otherwise takes the first valid one.
- Tests (`sw/tests/test_modes.py`):
  - `VideoMode` is immutable and hashable, with a readable label.
  - `options()` returns all values of each axis, with `enabled` only for compatible ones, at every cascade level (ratio, resolution, fps, lens, zoom).
  - `resolve()` keeps a still-valid value and otherwise picks the first valid one in preference order.
  - Changing the ratio re-cascades all downstream axes.
  - The default (16:9, 2.7K, 60, Wide, no zoom) is valid.
  - Consistency over the whole catalog: every mode is reachable through the cascade, `resolve` never returns an invalid mode, and an empty or unknown selection falls back to the default.
  - Edge cases: 4:3 offers no SuperView and only 2x zoom, Narrow offers no zoom, and the 16:9 catalog matches the requested list.
- Gate: full suite green, including the unchanged existing tests.

### Phase 2 - HERO10 adapter (`sw/camera/base.py`, `gopro_hero10.py`)
- **Work:**
  - `base.py`: `CameraBackend` Protocol (`catalog`, `connect(mode)`, `apply_mode(mode)`).
  - `gopro_hero10.py`: the catalog as a data table, the mapping from `VideoMode` to the SDK enums and zoom percent, and `apply_mode`. It sets resolution, fps, lens, then zoom last, reads the state back and raises on a mismatch.
- Tests (`sw/tests/test_gopro_hero10.py`, fake gopro client):
  - Every catalog mode maps to a valid `VideoResolution`, `FramesPerSecond` and `VideoLens`, with no duplicate mappings.
  - `apply_mode` calls the settings in the order resolution, fps, lens, zoom, and always sets zoom last (including 0 for none).
  - A failed response raises `RuntimeError` and stops the sequence.
  - Read-back verification flags a camera that changed fps or lens silently.
  - The catalog satisfies the `CameraBackend` Protocol (static and runtime check).
- Gate: full suite green.

### Phase 3 - Controller integration (`GoProController`)
- **Work:**
  - In `sw/livestream.py`, replace `RESOLUTIONS`, the `FPS` constant, `set_resolution` and `_apply_video_settings` with `apply_mode` on the backend.
  - `connect` takes a `VideoMode`.
  - Keep the preview stop/start around the change and leave the recording flow untouched.
- Tests (extend `sw/tests/test_livestream.py`, fake gopro):
  - `connect(mode)` applies the mode after loading the video preset group.
  - `apply_mode` stops the preview, applies, and restarts the preview in `finally`, on success and on failure.
  - The recording flow (`start_recording` stops the preview, `stop_recording`, `download`) behaves as before.
  - Regression: the existing `_check`, `PostProcessor` and `PreviewReader` tests still pass untouched.
- Gate: full suite green.

### Phase 4 - UI
- **Work:**
  - Replace the resolution radio row (livestream.py 516-526) with five labelled groups, one per axis, built from `catalog.options()`.
  - Replace `_on_resolution` (597-614) with a handler that re-resolves the selection, refreshes the enabled state of all groups, and applies through `_run` (revert on failure).
  - Add the new widgets to `_update_controls` and replace the hardcoded "60 fps" strings with the mode label.
- Tests (`sw/tests/test_livestream_ui.py`; the cascade logic is already covered by Phase 1, so these test only the thin wiring, using a helper that computes widget states without a display, and skipped if Tk is unavailable):
  - The startup state matches the default.
  - Choosing a ratio disables the incompatible resolutions, fps, lens and zoom entries.
  - All five axes are locked while busy or recording (`_update_controls`).
  - A failed apply reverts every selection to the last applied mode.
  - The status text follows the selection, with no hardcoded "60 fps".
- Gate: full suite green. Manual check with the camera: step through the cascade.

### Phase 5 - Persistence
- **Work:**
  - Store the mode as `video_mode` in `livestream_config.json`.
  - Extend `load_config` and `save_config` in livestream.py (448-469), keeping merge-on-save and validating on load.
  - Restore the saved mode at startup and save it after a successful apply.
- Tests (extend `test_livestream.py`, using `tmp_path`):
  - Round trip of the `video_mode` key.
  - A missing, invalid or unknown mode falls back to the default and logs a warning.
  - **Regression:** `load_config` and `save_config` keep their existing behavior, and unknown keys (watermark / flash settings) are preserved.
  - `shotcut_watermark.load_config` and `save_config` still read and write the shared file without losing `video_mode`.
- Gate: full suite green.

### Phase 6 - Regression sweep and wrap-up
- **Work:**
  - Fix anything the new fps and ratio fixtures expose in `shotcut_pipeline.py` and `shotcut_watermark.py`.
  - Add `--cov=sw.camera` to `.github/workflows/tests.yml`.
  - Update the livestream.py docstring and the README.
- Tests (extend `test_shotcut_pipeline.py` and `test_shotcut_watermark.py`): ffprobe fixtures for 4:3 and 5.3K 16:9, and 120/240 fps (watermark box size, MLT profile width, height and rate, `clip_range`, flash event timing, chunk count with fps-aware slowdown).
- Add `--cov=sw.camera` to `.github/workflows/tests.yml`. Update the livestream.py docstring and the README.
- Gate: full CI command passes locally (`uv run pytest --cov=sw.camera --cov=sw.livestream --cov=sw.shotcut_watermark --cov=sw.shotcut_pipeline --cov-report=term-missing sw/tests/`), and the hardware end-to-end check below is done.

## Critical files
`sw/livestream.py` (74-81, 246-269, 355-358, 513-526, 597-614, 735-746, 414-469), `sw/shotcut_watermark.py` (config helpers, 139-169), `.github/workflows/tests.yml`, `sw/tests/test_livestream.py`. New: `sw/camera/{modes,base,gopro_hero10}.py`, `sw/camera_probe.py`, `sw/tests/test_modes.py`, `sw/tests/test_gopro_hero10.py`.

## Verification
- `uv run pytest --cov=sw.camera --cov=sw.livestream --cov=sw.shotcut_watermark --cov=sw.shotcut_pipeline sw/tests/`
- With the camera connected: `uv run python sw/camera_probe.py`, then `uv run python -m sw.livestream`. Step through the cascades (e.g. 16:9, 4K, 120, Narrow, check zoom disabled), record a clip per representative mode, and confirm the ffprobe width, height and fps match the selection and the watermark/edit pipeline completes.
