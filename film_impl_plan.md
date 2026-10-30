# FILM Frame Interpolation Integration Plan

*Updated after codex review feedback.*

## Goal

Add slow-motion frame interpolation to `video_processing.py` using Google's FILM model.
Generate N interpolated frames between consecutive real frames within a configurable
region of the video (defined by start/end percentage of total frames). The slow-motion
effect is achieved by keeping the original FPS and increasing frame count, which
lengthens the video duration in the interpolated region.

## Architecture

### 1. New module: `sw/frame_interpolator.py`

Create a reusable `FrameInterpolator` class that wraps the FILM model logic from `film_test.py`.

**Class: `FrameInterpolator`**

- **Constructor (`__init__`)**:
  - `n_interpolated_frames: int = 4` — number of interpolated frames to generate between each pair.
  - `align: int = 64` — padding alignment for the FILM model.
  - Loads the FILM model from TFHub (`https://tfhub.dev/google/film/1`). Same approach as
    the existing `Interpolator` class in `film_test.py`. TFHub caches the model locally
    after first download.

- **Method: `interpolate(frame1: np.ndarray, frame2: np.ndarray) -> list[np.ndarray]`**:
  - Takes two BGR uint8 frames (as produced by OpenCV's `VideoCapture.read()`).
  - Converts them to float32 RGB in [0, 1] range (FILM model expects this).
  - **Batched inference**: generates all N intermediate frames in a single model call.
    Repeats `x0` and `x1` N times along the batch dimension and passes all `t` values
    `[k / (N+1) for k in 1..N]` together. This reduces Python overhead and improves
    accelerator utilization vs. N separate calls.
  - Applies `_pad_to_align` before inference and crops back after.
  - Converts each result back to BGR uint8 format.
  - Returns the list of N interpolated frames (does NOT include the original input frames).

- **Internal helper: `_pad_to_align`**:
  - Reused from `film_test.py` for handling non-aligned image dimensions.

### 2. Modifications to `sw/video_processing.py`

**New global parameters:**

```python
INTERPOLATION_START = 0.1   # Start interpolation after 10% of frames
INTERPOLATION_END = 0.9     # Stop interpolation at 90% of frames
INTERPOLATION_N = 4         # Number of interpolated frames between each pair
```

**Changes to `enqueue_videos` function:**

The `enqueue_videos` function reads frames sequentially and puts them onto a queue.
This is the natural place to inject interpolated frames — ordering is naturally preserved.

- Import `FrameInterpolator` from `frame_interpolator`.
- Instantiate a `FrameInterpolator(n_interpolated_frames=INTERPOLATION_N)` at the start.
- Track the previous **post-zoom** frame (interpolation happens on already-processed frames,
  not raw decoded frames, so spatial transforms are smooth).
- **Pair-based boundary check**: for pair `(i-1, i)`, interpolate only when BOTH `i-1 >= start_idx`
  AND `i <= end_idx`, where `start_idx = int(total_frames * INTERPOLATION_START)` and
  `end_idx = int(total_frames * INTERPOLATION_END)`.
- If within the interpolation region, call `interpolator.interpolate(prev_frame, current_frame)`
  and enqueue each interpolated frame before enqueuing the current real frame.
- Use a separate `output_idx` counter that increments for both real and interpolated frames.

**Queue contract change:**

The queue tuple changes from `(video_name, frame_idx, frame)` to include an `is_interpolated`
boolean flag: `(video_name, output_idx, frame, is_interpolated)`.

**Changes to `inference_loop`:**

- Unpack the new 4-element tuple: `(video_name, output_idx, frame, is_interpolated)`.
- If `is_interpolated` is `True`, skip YOLO inference and pass empty masks `[]`.
- If `is_interpolated` is `False`, run YOLO as before.
- Forward `(video_name, output_idx, frame, masks)` to downstream queues (same as current output format).

**Changes to `display_loop` and `video_writer_loop`:**

- No changes needed — they already receive `(video_name, idx, frame, masks)` and work
  with the frame directly. Empty masks for interpolated frames are handled correctly since
  `draw_person_masks` currently returns `[]` anyway (masks are disabled).

**FPS handling:**

- The output video keeps the same FPS as the writer's default (30.0). Since interpolated
  frames increase the frame count, the interpolated region plays in slow motion (longer duration).
  This is the intended behavior for the slow-motion effect.

## Data Flow

```
enqueue_videos (reads real frames, generates interpolated frames in-between)
    |
    v
input_queue: (video_name, output_idx, frame, is_interpolated)
    |
    v
inference_loop (runs YOLO on real frames; skips YOLO on interpolated, passes empty masks)
    |
    v
display_queue / writer_queue: (video_name, output_idx, frame, masks)
    |
    v
display_loop / video_writer_loop (unchanged)
```

## Edge Cases

1. **First frame of video**: No previous frame exists, so no interpolation — just enqueue normally.
2. **Pair-based boundaries**: Interpolate pair `(i-1, i)` only when both `i-1` and `i` are
   within `[start_idx, end_idx]`. Otherwise emit only the real frame.
3. **Very short videos**: If `total_frames * (INTERPOLATION_END - INTERPOLATION_START)` < 2,
   skip interpolation entirely.
4. **Frame size consistency**: Interpolation uses post-zoom frames, which are all resized back
   to original dimensions, so sizes are consistent.
5. **Memory**: FILM model + TensorFlow loaded once and reused across all frame pairs.

## Performance Notes

- **Batched inference** reduces the N separate model calls to 1 call per frame pair.
- FILM inference in `enqueue_videos` makes the producer the slowest stage. This is acceptable
  because the queue back-pressure naturally throttles the pipeline. The queue size may need
  to be increased for better throughput.
- GPU contention between TensorFlow (FILM) and PyTorch (YOLO) is possible if both use the
  same GPU. On CPU-only setups this is not an issue.
