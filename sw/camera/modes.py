"""Camera-agnostic video mode model with cascading option rules.

A video mode is a combination of five axes, always chosen in this order:

    ratio -> resolution -> fps -> lens -> zoom

Each axis value is either enabled or disabled given the choices made on the
earlier axes. All values of an axis are always listed so a UI can show them,
but only enabled values may be selected. Nothing here knows about any camera;
per-camera data lives in the adapter modules (for example
sw/camera/hero10_table.py).
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from typing import Any

AXES: tuple[str, ...] = ("ratio", "resolution", "fps", "lens", "zoom")

NO_ZOOM = "none"


@dataclass(frozen=True)
class VideoMode:
    """A fully specified video mode."""

    ratio: str
    resolution: str
    fps: int
    lens: str
    zoom: str = NO_ZOOM

    @property
    def label(self) -> str:
        """Short human readable name, e.g. "16:9 2.7K 60fps Wide 1.4x"."""
        text = f"{self.ratio} {self.resolution} {self.fps}fps {self.lens}"
        if self.zoom != NO_ZOOM:
            text += f" {self.zoom}"
        return text


@dataclass(frozen=True)
class Option:
    """One value of an axis and whether it can be selected right now."""

    value: Any
    enabled: bool


class ModeCatalog:
    """The set of valid video modes a camera supports, with cascade rules.

    Args:
        modes: Every valid mode. Duplicates are ignored; order is kept and
            decides the order values are shown and the fallback preference.
        default: The mode to start from. Must be one of `modes`.
    """

    def __init__(self, modes: Iterable[VideoMode], default: VideoMode) -> None:
        self._modes: tuple[VideoMode, ...] = tuple(dict.fromkeys(modes))
        if not self._modes:
            raise ValueError("catalog needs at least one mode")
        if default not in self._modes:
            raise ValueError(f"default {default.label} is not in the catalog")
        self._default = default
        self._values: dict[str, tuple[Any, ...]] = {
            axis: tuple(dict.fromkeys(getattr(m, axis) for m in self._modes)) for axis in AXES
        }

    @property
    def default(self) -> VideoMode:
        return self._default

    @property
    def modes(self) -> tuple[VideoMode, ...]:
        return self._modes

    def values(self, axis: str) -> tuple[Any, ...]:
        """Every value the axis takes anywhere in the catalog, in display order."""
        return self._values[_check_axis(axis)]

    def is_valid(self, mode: VideoMode) -> bool:
        return mode in self._modes

    def options(self, axis: str, selection: Mapping[str, Any]) -> list[Option]:
        """List the values of `axis`, enabled only if compatible with `selection`.

        Only the axes before `axis` in the cascade order are looked at. Missing
        entries in `selection` are treated as unconstrained.
        """
        enabled = set(self._enabled_values(axis, selection))
        return [Option(v, v in enabled) for v in self.values(axis)]

    def select(self, current: VideoMode, axis: str, value: Any) -> VideoMode:
        """Return the mode after the user picks `value` on `axis`.

        Downstream axes keep their current value when it is still compatible.
        Otherwise they take the first enabled value in display order.

        Raises:
            ValueError: if `value` is not enabled given the earlier choices in
                `current`.
        """
        _check_axis(axis)
        if value not in self._enabled_values(axis, _as_dict(current)):
            raise ValueError(f"{axis}={value!r} is not compatible with {current.label}")
        chosen = _as_dict(current)
        chosen[axis] = value
        return self.resolve(VideoMode(**chosen))

    def resolve(self, mode: VideoMode) -> VideoMode:
        """Return the nearest valid mode to `mode`, walking the cascade in order.

        Each axis keeps the requested value if it is enabled given the axes
        already settled. Otherwise it takes the first enabled value. The result
        is always a valid catalog mode.
        """
        chosen: dict[str, Any] = {}
        requested = _as_dict(mode)
        for axis in AXES:
            allowed = self._enabled_values(axis, chosen)
            chosen[axis] = requested[axis] if requested[axis] in allowed else allowed[0]
        return VideoMode(**chosen)

    def _enabled_values(self, axis: str, selection: Mapping[str, Any]) -> tuple[Any, ...]:
        """Values of `axis` that appear in a mode matching every earlier axis in `selection`."""
        prefix = AXES[: AXES.index(_check_axis(axis))]
        matching = [
            m for m in self._modes if all(getattr(m, a) == selection[a] for a in prefix if a in selection)
        ]
        present = {getattr(m, axis) for m in matching}
        return tuple(v for v in self.values(axis) if v in present)


def _check_axis(axis: str) -> str:
    if axis not in AXES:
        raise ValueError(f"unknown axis {axis!r}; expected one of {AXES}")
    return axis


def _as_dict(mode: VideoMode) -> dict[str, Any]:
    return {axis: getattr(mode, axis) for axis in AXES}
