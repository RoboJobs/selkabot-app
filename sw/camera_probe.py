"""Hardware probe for the HERO10 video mode table.

Run with the camera connected over USB:

    uv run python -m sw.camera_probe [--serial SERIAL]

For every (ratio, resolution, fps, lens) combination in the table it sets the
camera, reads the state back and reports whether the camera accepted it.
Rejected combinations must be removed from the table in
sw/camera/hero10_table.py.

The pure helpers (case expansion, state comparison, report formatting) do not
touch hardware and are unit tested.
"""

from __future__ import annotations

import argparse
import asyncio
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from typing import Any

from sw.camera import hero10_table as table
from sw.camera.gopro_hero10 import state_mismatches
from sw.camera.modes import VideoMode


@dataclass(frozen=True)
class ProbeCase:
    """One combination to send to the camera."""

    ratio: str
    resolution: str
    fps: int
    lens: str

    @property
    def label(self) -> str:
        return f"{self.ratio} {self.resolution} {self.fps}fps {self.lens}"


@dataclass(frozen=True)
class ProbeResult:
    """Outcome of applying one case."""

    case: ProbeCase
    accepted: bool
    mismatches: tuple[str, ...] = ()
    error: str = ""


def probe_cases(rows: Iterable[table.TableRow] = table.ROWS) -> list[ProbeCase]:
    """Expand table rows into every (ratio, resolution, fps, lens) combination."""
    seen: set[ProbeCase] = set()
    cases: list[ProbeCase] = []
    for row in rows:
        for fps in row.fps:
            for lens in row.lenses:
                case = ProbeCase(row.ratio, row.resolution, fps, lens)
                if case not in seen:
                    seen.add(case)
                    cases.append(case)
    return cases


def compare_state(expected: ProbeCase, state: Mapping[Any, Any]) -> tuple[str, ...]:
    """Return human readable mismatches between the requested case and camera state."""
    return state_mismatches(VideoMode(expected.ratio, expected.resolution, expected.fps, expected.lens), state)


def format_report(results: Iterable[ProbeResult]) -> str:
    """Render probe results as a plain text table, one case per line."""
    lines: list[str] = []
    accepted = rejected = 0
    for result in results:
        if result.accepted:
            accepted += 1
            lines.append(f"OK    {result.case.label}")
        else:
            rejected += 1
            detail = result.error or "; ".join(result.mismatches)
            lines.append(f"FAIL  {result.case.label}  ({detail})")
    lines.append(f"{accepted} accepted, {rejected} rejected")
    return "\n".join(lines)


async def _apply_case(gopro: Any, case: ProbeCase) -> ProbeResult:
    """Send one case to the camera and read the state back."""
    try:
        for response in (
            await gopro.http_setting.video_resolution.set(table.RESOLUTION_ENUM[(case.ratio, case.resolution)]),
            await gopro.http_setting.frames_per_second.set(table.FPS_ENUM[case.fps]),
            await gopro.http_setting.video_lens.set(table.LENS_ENUM[case.lens]),
        ):
            if not response.ok:
                return ProbeResult(case, False, error=f"camera returned {response.status}")
        state = (await gopro.http_command.get_camera_state()).data
    except Exception as exc:  # the probe must keep going through the whole table
        return ProbeResult(case, False, error=str(exc))
    mismatches = compare_state(case, state)
    return ProbeResult(case, not mismatches, mismatches)


async def _run(serial: str | None) -> int:
    from open_gopro import WiredGoPro

    gopro = WiredGoPro(serial)
    await gopro.open()
    try:
        results = [await _apply_case(gopro, case) for case in probe_cases()]
    finally:
        await gopro.close()
    print(format_report(results))
    return 0 if all(r.accepted for r in results) else 1


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--serial", default=None, help="Camera serial number (auto-discovered if omitted)")
    args = parser.parse_args()
    raise SystemExit(asyncio.run(_run(args.serial)))


if __name__ == "__main__":
    main()
