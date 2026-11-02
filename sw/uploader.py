"""Upload videos to Cloudflare R2 and create QR codes that link to them.

Credentials and settings are read from the repo-root `.env` file:
    R2_ACCOUNT_ID, R2_ACCESS_KEY_ID, R2_SECRET_ACCESS_KEY,
    R2_BUCKET, R2_PUBLIC_BASE_URL (e.g. https://videos.selkabot.com)
"""

from __future__ import annotations

import os
from pathlib import Path
from urllib.parse import quote

import boto3
import qrcode
from dotenv import load_dotenv

ENV_FILE = Path(__file__).resolve().parents[1] / ".env"


def _client():
    load_dotenv(ENV_FILE)
    return boto3.client(
        "s3",
        endpoint_url=f"https://{os.environ['R2_ACCOUNT_ID']}.r2.cloudflarestorage.com",
        aws_access_key_id=os.environ["R2_ACCESS_KEY_ID"],
        aws_secret_access_key=os.environ["R2_SECRET_ACCESS_KEY"],
        region_name="auto",
    )


def public_url(local_file: Path) -> str:
    """Return the public URL that `upload_video` will give `local_file`. Known before uploading."""
    load_dotenv(ENV_FILE)
    return f"{os.environ['R2_PUBLIC_BASE_URL'].rstrip('/')}/{quote(local_file.name)}"


def upload_video(local_file: Path) -> None:
    """Upload a file to the R2 bucket, under the key that `public_url` gives it."""
    load_dotenv(ENV_FILE)
    _client().upload_file(
        str(local_file),
        os.environ["R2_BUCKET"],
        local_file.name,
        ExtraArgs={"ContentType": "video/mp4", "ContentDisposition": "attachment"},
    )


def list_objects() -> dict[str, int]:
    """Return {key: size in bytes} for every object in the bucket."""
    load_dotenv(ENV_FILE)
    sizes: dict[str, int] = {}
    pages = _client().get_paginator("list_objects_v2").paginate(Bucket=os.environ["R2_BUCKET"])
    for page in pages:
        for obj in page.get("Contents", []):
            sizes[obj["Key"]] = obj["Size"]
    return sizes


def incomplete_uploads() -> list[tuple[str, str]]:
    """Return (key, upload id) for every multipart upload that was started but never completed."""
    load_dotenv(ENV_FILE)
    uploads: list[tuple[str, str]] = []
    pages = _client().get_paginator("list_multipart_uploads").paginate(Bucket=os.environ["R2_BUCKET"])
    for page in pages:
        uploads += [(u["Key"], u["UploadId"]) for u in page.get("Uploads", [])]
    return uploads


def abort_upload(key: str, upload_id: str) -> None:
    """Discard an incomplete multipart upload and the parts it has stored."""
    load_dotenv(ENV_FILE)
    _client().abort_multipart_upload(Bucket=os.environ["R2_BUCKET"], Key=key, UploadId=upload_id)


def make_qr(url: str, output: Path) -> Path:
    """Write a PNG QR code that encodes `url`."""
    qrcode.make(url).save(output)
    return output
