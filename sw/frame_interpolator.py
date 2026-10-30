import numpy as np
import tensorflow as tf
import tensorflow_hub as hub

_UINT8_MAX_F = float(np.iinfo(np.uint8).max)


def _pad_to_align(x, align):
    """Pad image batch so width and height are divisible by *align*.

    Returns the padded tensor and a dict of crop kwargs to undo the padding.
    """
    assert np.ndim(x) == 4
    assert align > 0

    height, width = x.shape[-3:-1]
    height_to_pad = (align - height % align) if height % align != 0 else 0
    width_to_pad = (align - width % align) if width % align != 0 else 0

    bbox_to_pad = {
        "offset_height": height_to_pad // 2,
        "offset_width": width_to_pad // 2,
        "target_height": height + height_to_pad,
        "target_width": width + width_to_pad,
    }
    padded_x = tf.image.pad_to_bounding_box(x, **bbox_to_pad)
    bbox_to_crop = {
        "offset_height": height_to_pad // 2,
        "offset_width": width_to_pad // 2,
        "target_height": height,
        "target_width": width,
    }
    return padded_x, bbox_to_crop


class FrameInterpolator:
    """Generate interpolated frames between two images using the FILM model.

    Accepts BGR uint8 frames (OpenCV convention) and returns BGR uint8 frames.
    """

    def __init__(self, n_interpolated_frames: int = 4, align: int = 64) -> None:
        self._n = n_interpolated_frames
        self._align = align
        self._model = hub.load("https://tfhub.dev/google/film/1")

    def interpolate(
        self, frame1: np.ndarray, frame2: np.ndarray
    ) -> list[np.ndarray]:
        """Return *n* interpolated frames between *frame1* and *frame2*.

        Both inputs are BGR uint8 arrays of shape (H, W, 3).
        Returns a list of *n* BGR uint8 arrays (excluding the two inputs).
        """
        # Convert BGR uint8 -> RGB float32 [0, 1]
        img1 = frame1[:, :, ::-1].astype(np.float32) / _UINT8_MAX_F
        img2 = frame2[:, :, ::-1].astype(np.float32) / _UINT8_MAX_F

        n = self._n
        x0 = np.expand_dims(img1, 0)  # (1, H, W, 3)
        x1 = np.expand_dims(img2, 0)  # (1, H, W, 3)

        # Pad to alignment once (same for all time steps)
        if self._align is not None:
            x0, bbox_to_crop = _pad_to_align(x0, self._align)
            x1, _ = _pad_to_align(x1, self._align)

        # Run one inference per intermediate time step to avoid GPU OOM
        out: list[np.ndarray] = []
        for k in range(1, n + 1):
            t = np.array([k / (n + 1)], dtype=np.float32)  # (1,)
            inputs = {"x0": x0, "x1": x1, "time": t[..., np.newaxis]}
            result = self._model(inputs, training=False)
            image = result["image"]

            if self._align is not None:
                image = tf.image.crop_to_bounding_box(image, **bbox_to_crop)

            rgb = np.clip(
                image[0].numpy() * _UINT8_MAX_F, 0, _UINT8_MAX_F
            ).astype(np.uint8)
            out.append(rgb[:, :, ::-1])
        return out
