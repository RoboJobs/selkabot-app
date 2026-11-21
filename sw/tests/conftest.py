"""Pytest configuration for PWMController tests."""

import pytest


def _is_raspberry_pi_5() -> bool:
    """Return True only when running on a Raspberry Pi 5."""
    try:
        with open("/proc/device-tree/model", "r") as f:
            model = f.read()
        return "Raspberry Pi 5" in model
    except OSError:
        return False


def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "hardware: mark tests requiring Raspberry Pi 5 hardware "
        "(auto-skipped otherwise)",
    )


def pytest_collection_modifyitems(config, items):
    if _is_raspberry_pi_5():
        return  # hardware available — run all tests
    skip_hw = pytest.mark.skip(reason="Requires Raspberry Pi 5 hardware")
    for item in items:
        if "hardware" in item.keywords:
            item.add_marker(skip_hw)
