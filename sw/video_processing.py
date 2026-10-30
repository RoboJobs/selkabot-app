import argparse
import asyncio
import multiprocessing as mp
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np
from ultralytics import YOLO

from frame_interpolator import FrameInterpolator

ZOOMED_FRAME_RATIO = 0.7
ZOOM_RATIO = 0.6
ZOOM_TRANSITION_RATIO = 0.05

INTERPOLATION_START = 0.1
INTERPOLATION_END = 0.9
INTERPOLATION_N = 4


def iter_video_paths(input_dir: Path) -> list[Path]:
    """Return a sorted list of .mp4 files in the input directory."""
    return sorted(input_dir.glob("*.mp4"))


def compute_zoom_frame_indices(
    total_frames: int,
    zoomed_frame_ratio: float,
    zoom_ratio: float,
    zoom_transition_ratio: float,
) -> dict[int, float]:
    """Compute the frame indices to zoom and their per-frame zoom ratios."""
    if total_frames <= 0 or zoomed_frame_ratio <= 0:
        return {}

    zoomed_count = int(total_frames * zoomed_frame_ratio)
    zoomed_count = max(0, min(total_frames, zoomed_count))
    if zoomed_count == 0:
        return {}

    leading = (total_frames - zoomed_count) // 2
    zoomed_indices = list(range(leading, leading + zoomed_count))

    transition_count = int(total_frames * zoom_transition_ratio)
    transition_count = max(0, min(zoomed_count, transition_count))

    zoom_map: dict[int, float] = {}
    if transition_count == 0:
        for idx in zoomed_indices:
            zoom_map[idx] = zoom_ratio
        return zoom_map

    for offset, frame_idx in enumerate(zoomed_indices):
        if offset < transition_count:
            step = offset / (transition_count -
                             1) if transition_count > 1 else 1.0
            ratio = 1.0 + (zoom_ratio - 1.0) * step
        elif offset >= zoomed_count - transition_count:
            tail_offset = offset - (zoomed_count - transition_count)
            step = tail_offset / (transition_count -
                                  1) if transition_count > 1 else 1.0
            ratio = zoom_ratio + (1.0 - zoom_ratio) * step
        else:
            ratio = zoom_ratio
        zoom_map[frame_idx] = ratio

    return zoom_map


def crop_and_resize(
    frame: np.ndarray,
    crop_size: tuple[int, int],
    start_pixel: tuple[int, int],
) -> np.ndarray:
    """Crop the frame using the provided ROI and resize to original size."""
    start_row, start_col = start_pixel
    crop_h, crop_w = crop_size
    end_row = start_row + crop_h
    end_col = start_col + crop_w
    cropped = frame[start_row:end_row, start_col:end_col]
    return cv2.resize(cropped, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_LINEAR)


def enqueue_videos(
    video_paths: Iterable[Path],
    output_queue: mp.Queue,
) -> None:
    """Read frames from videos, apply zoom effect and interpolation, and enqueue them."""
    interpolator = FrameInterpolator(n_interpolated_frames=INTERPOLATION_N)

    for video_path in video_paths:
        cap = cv2.VideoCapture(str(video_path))
        if not cap.isOpened():
            print(f"Failed to open video: {video_path}")
            continue

        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 0
        zoomed_map = compute_zoom_frame_indices(
            total_frames,
            ZOOMED_FRAME_RATIO,
            ZOOM_RATIO,
            ZOOM_TRANSITION_RATIO,
        )

        interp_start_idx = int(total_frames * INTERPOLATION_START)
        interp_end_idx = int(total_frames * INTERPOLATION_END)

        frame_idx = 0
        output_idx = 0
        prev_frame: np.ndarray | None = None

        while True:
            ok, frame = cap.read()
            if not ok:
                break
            height, width = frame.shape[:2]
            print("Processing: ", frame_idx)

            zoom_value = zoomed_map.get(frame_idx)
            if zoom_value is not None and zoom_value < 1.0:
                crop_w = int(zoom_value * width)
                crop_h = int(zoom_value * height)
                crop_w = max(1, min(width, crop_w))
                crop_h = max(1, min(height, crop_h))
                start_row = 0
                start_col = max(0, (width - crop_w) // 2)
                frame = crop_and_resize(
                    frame, (crop_h, crop_w), (start_row, start_col))

            # Pair-based interpolation: interpolate (prev, current) when both
            # fall within the interpolation window.
            if (
                prev_frame is not None
                and (frame_idx - 1) >= interp_start_idx
                and frame_idx <= interp_end_idx
            ):
                interp_frames = interpolator.interpolate(prev_frame, frame)
                for interp_frame in interp_frames:
                    output_queue.put(
                        (video_path.name, output_idx, interp_frame, True)
                    )
                    output_idx += 1

            output_queue.put((video_path.name, output_idx, frame, False))
            output_idx += 1
            prev_frame = frame
            frame_idx += 1

        cap.release()

    output_queue.put(None)


def resolve_person_class_id(names) -> int:
    """Return the class id for the 'person' class from a YOLO names mapping."""
    if isinstance(names, dict):
        for class_id, name in names.items():
            if name == "person":
                return int(class_id)
    else:
        for class_id, name in enumerate(names):
            if name == "person":
                return class_id
    raise ValueError("The provided model does not include a 'person' class.")


def extract_person_masks(result, person_class_id: int) -> list[np.ndarray]:
    """Extract person class masks from a single Ultralytics result."""
    if result.masks is None or result.boxes is None:
        return []

    classes = result.boxes.cls.cpu().numpy().astype(int)
    person_indices = np.where(classes == person_class_id)[0]
    if person_indices.size == 0:
        return []

    mask_tensor = result.masks.data[person_indices]
    masks = mask_tensor.cpu().numpy()
    return [mask > 0.5 for mask in masks]


def draw_person_masks(
    frame: np.ndarray,
    masks: list[np.ndarray],
    color: tuple[int, int, int] = (0, 255, 0),
    alpha: float = 0.5,
) -> np.ndarray:
    """Overlay segmentation masks onto the frame."""
    annotated = frame.copy()
    frame_h, frame_w = annotated.shape[:2]
    for mask in masks:
        if mask.shape[:2] != (frame_h, frame_w):
            mask = cv2.resize(
                mask.astype(np.uint8),
                (frame_w, frame_h),
                interpolation=cv2.INTER_NEAREST,
            ).astype(bool)
        overlay = annotated.copy()
        overlay[mask] = color
        annotated = cv2.addWeighted(overlay, alpha, annotated, 1 - alpha, 0)
    return annotated


async def inference_loop(
    input_queue: mp.Queue,
    output_queues: list[mp.Queue],
    model_path: str,
) -> None:
    """Run YOLO inference on frames and forward person masks."""
    model = YOLO(model_path)
    person_class_id = resolve_person_class_id(model.names)

    while True:
        item = await asyncio.to_thread(input_queue.get)
        if item is None:
            # Signal termination to all downstream consumers (display, writer)
            for q in output_queues:
                await asyncio.to_thread(q.put, None)
            break

        video_name, output_idx, frame, is_interpolated = item

        if is_interpolated:
            masks: list[np.ndarray] = []
        else:
            results = model.predict(frame, verbose=False)
            result = results[0]
            masks = extract_person_masks(result, person_class_id)

        # Fan out: each consumer gets its own copy of the result
        for q in output_queues:
            await asyncio.to_thread(q.put, (video_name, output_idx, frame, masks))


async def display_loop(output_queue: mp.Queue, window_name: str) -> None:
    """Draw person masks and display frames in a window."""
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    while True:
        item = await asyncio.to_thread(output_queue.get)
        if item is None:
            break

        _, _, frame, masks = item
        annotated = draw_person_masks(frame, masks)
        cv2.imshow(window_name, annotated)
        cv2.waitKey(1)

    cv2.destroyAllWindows()


async def video_writer_loop(
    writer_queue: mp.Queue,
    input_dir: Path,
    fps: float = 30.0,
) -> None:
    """Write annotated frames to mp4 video files."""
    current_video_name: str | None = None
    writer: cv2.VideoWriter | None = None

    while True:
        item = await asyncio.to_thread(writer_queue.get)
        if item is None:
            break

        video_name, _, frame, masks = item
        annotated = frame # draw_person_masks(frame, masks)

        # New video detected: close previous writer and open a new one
        if video_name != current_video_name:
            if writer is not None:
                writer.release()
            current_video_name = video_name
            stem = Path(video_name).stem
            output_path = input_dir / f"{stem}_post.mp4"
            h, w = annotated.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(str(output_path), fourcc, fps, (w, h))
            print(f"Writing output video: {output_path}")

        writer.write(annotated)

    # Release the last writer when the queue is drained
    if writer is not None:
        writer.release()


def run_inference_process(
    input_queue: mp.Queue,
    output_queues: list[mp.Queue],
    model_path: str,
) -> None:
    """Entry point for the inference process."""
    asyncio.run(inference_loop(input_queue, output_queues, model_path))


def run_display_process(output_queue: mp.Queue, window_name: str) -> None:
    """Entry point for the display process."""
    asyncio.run(display_loop(output_queue, window_name))


def run_video_writer_process(
    writer_queue: mp.Queue,
    input_dir: Path,
    fps: float = 30.0,
) -> None:
    """Entry point for the video writer process."""
    asyncio.run(video_writer_loop(writer_queue, input_dir, fps))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run YOLO person segmentation on .mp4 videos and display results."
    )
    parser.add_argument(
        "--input_dir",
        type=Path,
        help="Directory containing .mp4 files.",
    )
    parser.add_argument(
        "--model",
        default="yolov8n-seg.pt",
        help="Ultralytics segmentation model path or name.",
    )
    parser.add_argument(
        "--window-name",
        default="Person Segmentation",
        help="OpenCV window name.",
    )
    parser.add_argument(
        "--queue-size",
        type=int,
        default=4,
        help="Max number of frames buffered between processes.",
    )
    args = parser.parse_args()

    if not args.input_dir.exists():
        raise SystemExit(f"Input directory does not exist: {args.input_dir}")

    video_paths = iter_video_paths(args.input_dir)
    if not video_paths:
        print(f"No .mp4 files found in {args.input_dir}")
        return

    mp.set_start_method("spawn", force=True)

    # Pipeline: enqueue_videos -> inference -> [display, writer]
    input_queue = mp.Queue(maxsize=args.queue_size)
    display_queue = mp.Queue(maxsize=args.queue_size)
    writer_queue = mp.Queue(maxsize=args.queue_size)

    # Inference fans out to both display and video writer queues
    inference_process = mp.Process(
        target=run_inference_process,
        args=(input_queue, [display_queue, writer_queue], args.model),
        daemon=True,
    )
    display_process = mp.Process(
        target=run_display_process,
        args=(display_queue, args.window_name),
        daemon=True,
    )
    video_writer_process = mp.Process(
        target=run_video_writer_process,
        args=(writer_queue, args.input_dir),
        daemon=True,
    )

    inference_process.start()
    display_process.start()
    video_writer_process.start()

    enqueue_videos(video_paths, input_queue)

    inference_process.join()
    display_process.join()
    video_writer_process.join()


if __name__ == "__main__":
    main()
