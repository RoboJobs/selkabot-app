"""Store the selected video mode in the livestream config file.

The mode lives under the "video_mode" key of livestream_config.json, which
sw/livestream.py and sw/shotcut_watermark.py also read. Saving merges into the
file so other keys are kept. A file that cannot be parsed is never overwritten,
so the user's settings are not lost.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any

from sw.camera.modes import AXES, ModeCatalog, VideoMode

logger = logging.getLogger(__name__)

MODE_KEY = "video_mode"


def mode_to_dict(mode: VideoMode) -> dict[str, Any]:
    return {axis: getattr(mode, axis) for axis in AXES}


def mode_from_dict(data: Any, catalog: ModeCatalog) -> VideoMode | None:
    """Return the mode in `data`, or None if it is malformed or not in the catalog."""
    if not isinstance(data, dict):
        return None
    try:
        mode = VideoMode(**{axis: data[axis] for axis in AXES})
    except (KeyError, TypeError):
        return None
    return mode if catalog.is_valid(mode) else None


def load_mode(config_file: Path, catalog: ModeCatalog) -> VideoMode:
    """Return the saved mode, or the catalog default if it is missing or unusable."""
    try:
        data = json.loads(config_file.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return catalog.default
    except (OSError, ValueError):
        logger.warning("Ignoring unreadable config file %s", config_file, exc_info=True)
        return catalog.default
    if not isinstance(data, dict) or MODE_KEY not in data:
        return catalog.default
    mode = mode_from_dict(data[MODE_KEY], catalog)
    if mode is None:
        logger.warning(
            "Ignoring saved video mode %r in %s", data[MODE_KEY], config_file
        )
        return catalog.default
    return mode


def save_mode(config_file: Path, mode: VideoMode) -> bool:
    """Store `mode` in the config file, keeping other keys.

    Returns False if nothing was written.
    """
    try:
        config = json.loads(config_file.read_text(encoding="utf-8"))
    except FileNotFoundError:
        config = {}
    except (OSError, ValueError):
        logger.warning(
            "Not saving video mode: %s is unreadable", config_file, exc_info=True
        )
        return False
    if not isinstance(config, dict):
        logger.warning("Not saving video mode: %s is not a JSON object", config_file)
        return False
    config[MODE_KEY] = mode_to_dict(mode)
    try:
        config_file.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    except OSError:
        logger.warning("Could not save video mode to %s", config_file, exc_info=True)
        return False
    return True
