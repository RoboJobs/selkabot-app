# photobot

## Installation

```bash
sudo apt install flatpak
flatpak install flathub org.shotcutapp.Shotcut  # Install Shotcut from flatpak
export SHOTCUT_PATH=$HOME/.local/bin/shotcut-flatpak # Setup environment variables for the mcp server
export SHOTCUT_MELT_PATH=$HOME/.local/bin/shotcut-melt
git clone git@github.com:juanmed/photobot.git
git config core.hooksPath .githooks   # Enable repository quality hooks
git submodule update --init --recursive shotcut-mcp   # fetch the shotcut-mcp submodule source
python3 shotcut-mcp/scripts/shotcut_mcp_server.py --check   # verify the server starts
claude mcp add --transport stdio shotcut -- python3 shotcut-mcp/scripts/shotcut_mcp_server.py   # register the server for this project only
claude mcp list   # confirm the shotcut server is registered
```

## Development checks

Run pre-commit hooks manually using `ruff` and unit tests using `pytest`:

```bash
uv run ruff format --check path/to/file.py
uv run pytest --cov=sw.livestream --cov=sw.shotcut_watermark --cov=sw.shotcut_pipeline --cov-report=term-missing sw/tests/
```
