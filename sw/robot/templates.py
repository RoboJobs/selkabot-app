"""Starter designs for `motionctl new`. Each returns a dict that loads as a Design.

Values are placeholders for a subject 400 mm ahead of the robot base; edit the
keyframes and the workspace box for the real set before validating.
"""

from __future__ import annotations

from typing import Any


def _workspace() -> dict[str, list[float]]:
    return {"min": [200, -900, 100], "max": [950, 900, 1100]}


def sweep() -> dict[str, Any]:
    """Horizontal sweep, camera pans while the body moves sideways."""
    return {
        "tool_id": 1,
        "sample_period_ms": 8,
        "seed_joints": [0, -90, 90, -90, -90, 0],
        "orientation": {"mode": "angles"},
        "workspace": _workspace(),
        "keyframes": [
            {"t": 0.0, "pos": [600, -400, 450], "pan": 20, "tilt": -5},
            {"t": 1.2, "pos": [600, 400, 450], "pan": -20, "tilt": -5},
            {"t": 2.0, "pos": [600, 400, 450], "pan": -20, "tilt": -5, "stop": True},
        ],
    }


def crane() -> dict[str, Any]:
    """Vertical crane, camera rises while it keeps a fixed heading."""
    return {
        "tool_id": 1,
        "sample_period_ms": 8,
        "seed_joints": [0, -90, 90, -90, -90, 0],
        "orientation": {"mode": "angles"},
        "workspace": _workspace(),
        "keyframes": [
            {"t": 0.0, "pos": [600, 0, 300], "pan": 0, "tilt": 0},
            {"t": 2.0, "pos": [600, 0, 800], "pan": 0, "tilt": 0},
        ],
    }


def orbit() -> dict[str, Any]:
    """Arc around a fixed subject point, the camera keeps it in frame."""
    return {
        "tool_id": 1,
        "sample_period_ms": 8,
        "seed_joints": [0, -90, 90, -90, -90, 0],
        "orientation": {"mode": "target", "target": [400, 0, 300]},
        "workspace": _workspace(),
        "keyframes": [
            {"t": 0.0, "pos": [600, -300, 500]},
            {"t": 2.0, "pos": [600, 300, 500]},
        ],
    }


def push_in() -> dict[str, Any]:
    """Slow approach toward the subject, the camera stays on it."""
    return {
        "tool_id": 1,
        "sample_period_ms": 8,
        "seed_joints": [0, -90, 90, -90, -90, 0],
        "orientation": {"mode": "target", "target": [400, 0, 300]},
        "workspace": _workspace(),
        "keyframes": [
            {"t": 0.0, "pos": [700, 0, 400]},
            {"t": 2.5, "pos": [520, 0, 360]},
        ],
    }


TEMPLATES = {"sweep": sweep, "crane": crane, "orbit": orbit, "push_in": push_in}


def new_design(template: str, name: str) -> dict[str, Any]:
    """Return a complete design dict named `name` built from a template."""
    if template not in TEMPLATES:
        raise ValueError(
            f"unknown template {template!r}; choose from {sorted(TEMPLATES)}"
        )
    design = {"name": name, **TEMPLATES[template]()}
    return design
