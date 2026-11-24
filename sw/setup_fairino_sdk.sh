#!/usr/bin/env bash
# Download the Fairino Python SDK into third_party/fairino_sdk (git-ignored) and
# make it an installable uv project. pyproject.toml points at
# third_party/fairino_sdk/linux, so run this before `uv sync`, locally (via
# sw/setup.sh) and in CI. Safe to re-run.
set -euo pipefail
cd "$(dirname "$0")/.."

# Download Fairino Python SDK for controller sw version 3.9.1 (only if not already extracted,
# so re-runs do not overwrite the files added below)
FAIRINO_SDK_URL="https://github.com/FAIR-INNOVATION/fairino-python-sdk/archive/refs/tags/v2.2.1_robot3.9.1.tar.gz"
SDK_DIR="third_party/fairino_sdk"
if [ ! -f "$SDK_DIR/linux/fairino/Robot.py" ]; then
    SDK_ARCHIVE="$(mktemp --suffix=.tar.gz)"
    mkdir -p "$SDK_DIR"
    curl -fsSL -o "$SDK_ARCHIVE" "$FAIRINO_SDK_URL"
    # Strip the archive's top-level folder so the contents land directly in $SDK_DIR
    tar -xzf "$SDK_ARCHIVE" -C "$SDK_DIR" --strip-components=1
    rm -f "$SDK_ARCHIVE"
fi

# Make the extracted SDK an installable uv project: uv sync compiles Robot.py into fairino.Robot
cat > "$SDK_DIR/linux/pyproject.toml" <<'EOF'
[build-system]
requires = ["setuptools>=68", "cython>=3.0"]
build-backend = "setuptools.build_meta"

[project]
name = "fairino"
version = "2.2.1"
requires-python = ">=3.12,<3.13"

[tool.setuptools]
packages = ["fairino"]
EOF
cat > "$SDK_DIR/linux/setup.py" <<'EOF'
from Cython.Build import cythonize
from setuptools import Extension, setup

setup(
    ext_modules=cythonize(
        [Extension("fairino.Robot", ["fairino/Robot.py"])],
        compiler_directives={"language_level": "3"},
        quiet=True,
    ),
)
EOF
touch "$SDK_DIR/linux/fairino/__init__.py"
