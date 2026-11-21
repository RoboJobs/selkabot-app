# photobot

## Installation

```bash
sudo apt install flatpak
flatpak install flathub org.shotcutapp.Shotcut  # Install Shotcut from flatpak
export SHOTCUT_PATH=$HOME/.local/bin/shotcut-flatpak # Setup environment variables for the mcp server
export SHOTCUT_MELT_PATH=$HOME/.local/bin/shotcut-melt
git clone git@github.com:juanmed/photobot.git
git submodule update --init --recursive shotcut-mcp   # fetch the shotcut-mcp submodule source
python3 shotcut-mcp/scripts/shotcut_mcp_server.py --check   # verify the server starts
claude mcp add --transport stdio shotcut -- python3 shotcut-mcp/scripts/shotcut_mcp_server.py   # register the server for this project only
claude mcp list   # confirm the shotcut server is registered
```

## GoPro livestream video modes

`sw/livestream.py` records with a mode chosen from five rows: ratio, resolution, fps, lens and zoom. Options that the current mode rules out are disabled. The valid modes for the HERO10 come from GoPro's settings table in `sw/camera/hero10_table.py`. The selected mode is saved in `sw/livestream_config.json`.

Zoom percentages for 1.4x and 2x are not known yet. Calibrate them with the camera connected over USB:

    uv run python -m sw.camera_probe --calibrate-zoom 50

Look at the displayed zoom factor and record the percentage in `ZOOM_PERCENT` in `hero10_table.py`. To check that every table mode is accepted by the camera, run `uv run python -m sw.camera_probe` with no arguments.

