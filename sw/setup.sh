# # Install uv
# curl -LsSf https://astral.sh/uv/install.sh | sh

# # Install CUDA12.6
# wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-ubuntu2404.pin
# sudo mv cuda-ubuntu2404.pin /etc/apt/preferences.d/cuda-repository-pin-600
# wget https://developer.download.nvidia.com/compute/cuda/12.6.0/local_installers/cuda-repo-ubuntu2404-12-6-local_12.6.0-560.28.03-1_amd64.deb
# sudo dpkg -i cuda-repo-ubuntu2404-12-6-local_12.6.0-560.28.03-1_amd64.deb
# sudo cp /var/cuda-repo-ubuntu2404-12-6-local/cuda-26DFA2FB-keyring.gpg /usr/share/keyrings/
# sudo apt-get update
# sudo apt-get -y install cuda-toolkit-12-6

# # Install cuDNN9.20.0
# wget https://developer.download.nvidia.com/compute/cudnn/9.20.0/local_installers/cudnn-local-repo-ubuntu2404-9.20.0_1.0-1_amd64.deb
# sudo dpkg -i cudnn-local-repo-ubuntu2404-9.20.0_1.0-1_amd64.deb
# sudo cp /var/cudnn-local-repo-ubuntu2404-9.20.0/cudnn-*-keyring.gpg /usr/share/keyrings/
# sudo apt-get update
# sudo apt-get -y install cudnn

# Download Fairino Python SDK for controller sw version 3.9.1 (only if not already extracted,
# so re-runs do not overwrite the files added below)
FAIRINO_SDK_URL="https://github.com/FAIR-INNOVATION/fairino-python-sdk/archive/refs/tags/v2.2.1_robot3.9.1.tar.gz"
SDK_DIR="third_party/fairino_sdk"
if [ ! -f "$SDK_DIR/linux/fairino/Robot.py" ]; then
    SDK_ARCHIVE="$(mktemp --suffix=.tar.gz)"
    mkdir -p "$SDK_DIR"
    curl -L -o "$SDK_ARCHIVE" "$FAIRINO_SDK_URL"
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

# Install the SDK and the rest of the project into .venv
uv sync

# Codegraph
# curl -fsSL https://raw.githubusercontent.com/colbymchenry/codegraph/main/install.sh | sh
# source ~/.bashrc
# codegraph install
# codegraph init

