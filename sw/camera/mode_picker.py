"""What the UI shows for the cascading video mode selector.

ModePicker holds the mode the camera is set to ("committed") and answers the
questions a UI needs: which discrete values to list and which are enabled, the
zoom range, and which mode a pick would produce. It has no Tk or camera
dependency. The UI calls commit() only after the camera has accepted the new mode.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from sw.camera.modes import AXES, NO_ZOOM, ModeCatalog, VideoMode


@dataclass(frozen=True)
class PickerOption:
    """One button in the selector: the text shown, the value it stands for, and whether it can be pressed."""

    text: str
    value: Any
    enabled: bool


class ModePicker:
    """Tracks the committed mode and turns user picks into candidate modes.

    Args:
        catalog: The camera's mode catalog.
        mode: Starting mode. Defaults to the catalog default.
        unavailable: Values per discrete axis that are in the catalog but must not be offered.
            A saved mode using one of them falls back to the default.
    """

    def __init__(
        self,
        catalog: ModeCatalog,
        mode: VideoMode | None = None,
        unavailable: Mapping[str, frozenset] | None = None,
    ) -> None:
        self._catalog = catalog
        self._unavailable: dict[str, frozenset] = dict(unavailable or {})
        self._committed = mode if mode is not None else catalog.default
        if not catalog.is_valid(self._committed):
            raise ValueError(f"{self._committed.label} is not in the catalog")
        if self._is_unavailable(self._committed):
            # A saved mode that the window may no longer offer falls back to the default.
            self._committed = catalog.default

    @property
    def mode(self) -> VideoMode:
        """The mode the camera is known to be in."""
        return self._committed

    @property
    def axes(self) -> tuple[str, ...]:
        return AXES

    def selected_text(self, axis: str) -> str:
        """Text of the committed value on a discrete axis, as shown on its button."""
        return str(getattr(self._committed, axis))

    def options(self, axis: str) -> list[PickerOption]:
        """Every discrete value of `axis`, enabled only where the committed mode allows it."""
        selection = {a: getattr(self._committed, a) for a in AXES}
        return [
            PickerOption(str(o.value), o.value, o.enabled and o.value not in self._unavailable.get(axis, frozenset()))
            for o in self._catalog.options(axis, selection)
        ]

    def zoom_range(self) -> tuple[int, int]:
        """Lowest and highest zoom percentage the committed mode allows."""
        return NO_ZOOM, self._catalog.zoom_max(self._committed)

    def preview(self, axis: str, text: str) -> VideoMode:
        """The mode that picking `text` on a discrete axis would produce, without committing it.

        Raises:
            ValueError: `text` is not a value of `axis`, or it is not enabled.
        """
        value = self._value_for(axis, text)
        if value in self._unavailable.get(axis, frozenset()):
            raise ValueError(f"{text} {axis} is not available on this camera yet")
        return self._catalog.select(self._committed, axis, value)

    def preview_zoom(self, percent: int) -> VideoMode:
        """The mode that setting zoom to `percent` would produce, without committing it.

        Raises:
            ValueError: `percent` is outside the range the committed mode allows.
        """
        return self._catalog.select(self._committed, "zoom", percent)

    def commit(self, mode: VideoMode) -> None:
        """Record that the camera is now in `mode`."""
        if not self._catalog.is_valid(mode):
            raise ValueError(f"{mode.label} is not in the catalog")
        self._committed = mode

    def _is_unavailable(self, mode: VideoMode) -> bool:
        return any(getattr(mode, axis) in values for axis, values in self._unavailable.items())

    def _value_for(self, axis: str, text: str) -> Any:
        if axis == "zoom":
            raise ValueError("zoom is a percentage; use preview_zoom")
        for value in self._catalog.values(axis):
            if str(value) == text:
                return value
        raise ValueError(f"{text!r} is not a {axis} value")
