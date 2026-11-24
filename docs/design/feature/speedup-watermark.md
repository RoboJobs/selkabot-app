# Fast social-video postprocessing

The postprocessor now defaults to a social delivery profile of 1920×1080 at 30 fps.
The original high-fps recording is retained for slow-motion extraction, while normal
footage is cached as a scaled 30-fps proxy before Shotcut/MLT composition.

## Slow motion

The default `native` mode retimes captured frames instead of using `minterpolate`.
For a 120-fps source and 30-fps output, 0.25× can keep every captured frame; faster
slowdowns sample the required subset of source frames. Native mode clamps requests below
the source/output limit, so 0.2× becomes 0.25× rather than inventing frames.

The existing `high` interpolation mode remains available explicitly for projects that
need synthetic frames. `shotcut` remains available as the legacy MLT timewarp mode.
Slow segments are cached with the source signature, output profile, mode, and effective
speed in the cache key.

## Effects and controls

The watermark GUI persists output fps/size and exposes independent checkboxes for:

- camera flash effect;
- image fade effects, controlling both fade-in and fade-out stills.

Disabling a checkbox does not remove the configured windows or image paths. The effect
is simply omitted from the generated project and can be re-enabled later.

Flash keyframes are generated at the output fps, reducing unnecessary 120-fps effect
work. Final output remains atomic and the original download is preserved.

## Validation

Unit tests cover native speed limits, output profiles, disabled flashes, persisted
effect toggles, and configuration parsing. A real FFmpeg smoke test verifies that a
120-fps one-second segment clamped to 0.25× produces 120 frames at 30 fps.
