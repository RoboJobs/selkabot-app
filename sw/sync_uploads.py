"""Check which local videos are in the R2 bucket and upload the ones that are not.

Runs on startup of the livestream app, and can also run on its own:
    uv run python -m sw.sync_uploads [--report-only]

Steps:
1. Abort incomplete multipart uploads, but only where the local video exists.
   Uploads with no local video are kept, so their partial data is not thrown away.
2. List the bucket once. A local video counts as uploaded only when the bucket has
   an object with the same file name and the same size.
3. Upload the videos that are missing, or whose size differs (a partial object).

For now only plain `<name>.mp4` files are synced. The `_original` files are never
uploaded, and `_watermarked` files came after these recordings, so they are not
handled yet.
"""

from __future__ import annotations

import argparse
import logging
from dataclasses import dataclass, field
from pathlib import Path

from sw.uploader import abort_upload, incomplete_uploads, list_objects, upload_video

logger = logging.getLogger(__name__)

DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parents[1] / "videos"
EXCLUDED_SUFFIXES = ("_original.mp4", "_watermarked.mp4")


@dataclass
class Summary:
    """What the sync found and did. Lists hold file names."""

    stale: list[str] = field(default_factory=list)  # Incomplete uploads that have a local video
    aborted: list[str] = field(default_factory=list)
    present: list[str] = field(default_factory=list)  # In the bucket with the same size
    uploaded: list[str] = field(default_factory=list)
    pending: list[str] = field(default_factory=list)  # Missing or wrong size, not uploaded (report only)
    failed: dict[str, str] = field(default_factory=dict)  # Name -> error

    def log(self) -> None:
        logger.info("Upload sync summary")
        logger.info("  Incomplete uploads with a local video: %d", len(self.stale))
        logger.info("  Aborted: %d %s", len(self.aborted), self.aborted)
        logger.info("  Already in bucket (name and size match): %d", len(self.present))
        logger.info("  Uploaded now: %d %s", len(self.uploaded), self.uploaded)
        logger.info("  Pending (not uploaded): %d %s", len(self.pending), self.pending)
        logger.info("  Failed: %d %s", len(self.failed), self.failed)


def sync_uploads(output_dir: Path, upload: bool = True) -> Summary:
    """Reconcile the local videos with the bucket. With upload=False, only report."""
    summary = Summary()
    # Recursive: the livestream app saves each recording into a subfolder of output_dir.
    local = {
        path.name: path
        for path in sorted(output_dir.rglob("*.mp4"))
        if not path.name.endswith(EXCLUDED_SUFFIXES)
    }

    for key, upload_id in incomplete_uploads():
        if key not in local:
            logger.info("Keeping incomplete upload of %s: no local file", key)
            continue
        summary.stale.append(key)
        if upload:
            abort_upload(key, upload_id)
            summary.aborted.append(key)

    remote = list_objects()
    for name, path in local.items():
        if remote.get(name) == path.stat().st_size:
            summary.present.append(name)
        elif not upload:
            summary.pending.append(name)
        else:
            try:
                upload_video(path)
                summary.uploaded.append(name)
            except Exception as e:  # pylint: disable=broad-except
                logger.exception("Upload of %s failed", name)
                summary.failed[name] = str(e)

    summary.log()
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="Upload local videos that are missing from the R2 bucket.")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR, help="Folder with the videos")
    parser.add_argument("--report-only", action="store_true", help="Only report; do not abort or upload")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(message)s")
    sync_uploads(args.output_dir, upload=not args.report_only)


if __name__ == "__main__":
    main()
