"""Camera-agnostic video mode model with cascading option rules.

A video mode is a combination of five axes, always chosen in this order:

    ratio -> resolution -> fps -> lens -> zoom

Each discrete axis value is either enabled or disabled given the choices made
on the earlier axes. All values of an axis are always listed so a UI can show
them, but only enabled values may be selected.

Zoom is continuous: an integer percentage from 0 to `zoom_max(mode)`. The
catalog therefore lists only zoom 0 modes, and the maximum comes from a
function supplied by the camera adapter (for example "no zoom with SuperView").
Nothing here knows about any camera; per-camera data lives in the adapter
modules (for example sw/camera/hero10_table.py).
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass, replace
from typing import Any

AXES: tuple[str, ...] = ("ratio", "resolution", "fps", "lens", "zoom")

NO_ZOOM = 0  # digital zoom percentage meaning "no zoom"


@dataclass(frozen=True)
class VideoMode:
    """A fully specified video mode. `zoom` is a percentage, 0 meaning no zoom."""

    ratio: str
    resolution: str
    fps: int
    lens: str
    zoom: int = NO_ZOOM

    @property
    def label(self) -> str:
        """Short human readable name, e.g. "16:9 2.7K 60fps Wide zoom 42%"."""
        text = f"{self.ratio} {self.resolution} {self.fps}fps {self.lens}"
        if self.zoom != NO_ZOOM:
            text += f" zoom {self.zoom}%"
        return text


@dataclass(frozen=True)
class Option:
    """One value of a discrete axis and whether it can be selected right now."""

    value: Any
    enabled: bool


class ModeCatalog:
    """The set of valid video modes a camera supports, with cascade rules.

    Args:
        modes: Every valid mode with zoom 0. Duplicates are ignored; order is
            kept and decides the order values are shown and the fallback preference.
        default: The mode to start from. Must be one of `modes`.
        zoom_max: Largest zoom percentage allowed for a mode (given with zoom 0).
            Defaults to no zoom at all.
    """

    def __init__(
        self,
        modes: Iterable[VideoMode],
        default: VideoMode,
        zoom_max: Callable[[VideoMode], int] | None = None,
    ) -> None:
        self._modes: tuple[VideoMode, ...] = tuple(dict.fromkeys(modes))
        if not self._modes:
            raise ValueError("catalog needs at least one mode")
        if any(m.zoom != NO_ZOOM for m in self._modes):
            raise ValueError("catalog modes must have zoom 0; zoom is continuous")
        if default not in self._modes:
            raise ValueError(f"default {default.label} is not in the catalog")
        self._set = frozenset(self._modes)
        self._default = default
        self._zoom_max = zoom_max if zoom_max is not None else (lambda mode: NO_ZOOM)
        self._values: dict[str, tuple[Any, ...]] = {
            axis: tuple(dict.fromkeys(getattr(m, axis) for m in self._modes))
            for axis in AXES
        }

    @property
    def default(self) -> VideoMode:
        return self._default

    @property
    def modes(self) -> tuple[VideoMode, ...]:
        return self._modes

    def zoom_max(self, mode: VideoMode) -> int:
        """Largest zoom percentage the camera allows with `mode`'s other settings."""
        return self._zoom_max(replace(mode, zoom=NO_ZOOM))

    def values(self, axis: str) -> tuple[Any, ...]:
        """Every discrete value of the axis in the catalog, in display order."""
        return self._values[_check_axis(axis)]

    def is_valid(self, mode: VideoMode) -> bool:
        if not isinstance(mode.zoom, int) or isinstance(mode.zoom, bool):
            return False
        base = replace(mode, zoom=NO_ZOOM)
        return base in self._set and NO_ZOOM <= mode.zoom <= self.zoom_max(base)

    def options(self, axis: str, selection: Mapping[str, Any]) -> list[Option]:
        """List the values of a discrete axis, enabled if compatible with `selection`.

        Only the axes before `axis` in the cascade order are looked at. Missing
        entries in `selection` are treated as unconstrained.
        """
        enabled = set(self._enabled_values(axis, selection))
        return [Option(v, v in enabled) for v in self.values(axis)]

    def select(self, current: VideoMode, axis: str, value: Any) -> VideoMode:
        """Return the mode after the user picks `value` on `axis`.

        Discrete axes: downstream axes keep their current value when it is still
        compatible, otherwise they take the first enabled value in display order.
        Zoom: `value` is a percentage that must be within the allowed range.

        Raises:
            ValueError: if `value` is not allowed given the choices in `current`.
        """
        _check_axis(axis)
        chosen = _as_dict(current)
        if axis == "zoom":
            base = replace(current, zoom=NO_ZOOM)
            if not isinstance(value, int) or not NO_ZOOM <= value <= self.zoom_max(
                base
            ):
                raise ValueError(f"zoom {value!r} is not allowed with {base.label}")
            chosen["zoom"] = value
            return VideoMode(**chosen)
        if value not in self._enabled_values(axis, chosen):
            raise ValueError(f"{axis}={value!r} is not compatible with {current.label}")
        chosen[axis] = value
        return self.resolve(VideoMode(**chosen))

    def resolve(self, mode: VideoMode) -> VideoMode:
        """Return the nearest valid mode to `mode`, walking the cascade in order.

        Each discrete axis keeps the requested value if it is enabled given the
        axes already settled, otherwise it takes the first enabled value. Zoom
        keeps the requested percentage, clamped to the allowed range.
        """
        chosen: dict[str, Any] = {}
        requested = _as_dict(mode)
        for axis in AXES[:-1]:
            allowed = self._enabled_values(axis, chosen)
            chosen[axis] = requested[axis] if requested[axis] in allowed else allowed[0]
        base = VideoMode(**chosen, zoom=NO_ZOOM)
        chosen["zoom"] = min(max(int(requested["zoom"]), NO_ZOOM), self.zoom_max(base))
        return VideoMode(**chosen)

    def _enabled_values(
        self, axis: str, selection: Mapping[str, Any]
    ) -> tuple[Any, ...]:
        """Values of `axis` in a mode matching every earlier axis in `selection`."""
        prefix = AXES[: AXES.index(_check_axis(axis))]
        matching = [
            m
            for m in self._modes
            if all(getattr(m, a) == selection[a] for a in prefix if a in selection)
        ]
        present = {getattr(m, axis) for m in matching}
        return tuple(v for v in self.values(axis) if v in present)


def _check_axis(axis: str) -> str:
    if axis not in AXES:
        raise ValueError(f"unknown axis {axis!r}; expected one of {AXES}")
    return axis


def _as_dict(mode: VideoMode) -> dict[str, Any]:
    return {axis: getattr(mode, axis) for axis in AXES}
