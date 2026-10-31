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


def upload_video(local_file: Path) -> str:
    """Upload a file to the R2 bucket and return its public URL."""
    load_dotenv(ENV_FILE)
    key = local_file.name
    _client().upload_file(
        str(local_file),
        os.environ["R2_BUCKET"],
        key,
        ExtraArgs={"ContentType": "video/mp4", "ContentDisposition": "attachment"},
    )
    return f"{os.environ['R2_PUBLIC_BASE_URL'].rstrip('/')}/{quote(key)}"


def make_qr(url: str, output: Path) -> Path:
    """Write a PNG QR code that encodes `url`."""
    qrcode.make(url).save(output)
    return output
