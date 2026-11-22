"""Interface that every camera adapter implements.

The livestream controller depends only on this protocol, so a different
camera brand or model needs one new adapter module and no UI changes.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from sw.camera.modes import ModeCatalog, VideoMode


@runtime_checkable
class CameraBackend(Protocol):
    """A camera that can list and apply video modes."""

    @property
    def catalog(self) -> ModeCatalog:
        """Every mode this camera supports, plus its default."""
        ...

    async def apply_mode(self, mode: VideoMode) -> None:
        """Configure the camera for `mode`, or raise if it cannot be applied.

        Raises:
            ValueError: `mode` is not in `catalog`.
            RuntimeError: the camera rejected a setting or did not keep it.
        """
        ...
