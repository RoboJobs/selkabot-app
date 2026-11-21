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

`sw/livestream.py` records with a mode chosen from five rows: ratio, resolution, fps, lens and zoom. Options that the current mode rules out are disabled. Zoom is a slider from 0 to 100%, enabled only on lenses that support digital zoom; the camera is changed when the slider is released. The valid modes for the HERO10 come from GoPro's settings table in `sw/camera/hero10_table.py`. The selected mode is saved in `sw/livestream_config.json`.

To check that every table mode is accepted by the camera, run `uv run python -m sw.camera_probe` with the camera connected over USB.
