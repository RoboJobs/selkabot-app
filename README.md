# photobot

## Installation

```bash
# Shotcut
sudo apt install flatpak
flatpak install flathub org.shotcutapp.Shotcut  # Install Shotcut from flatpak
export SHOTCUT_PATH=$HOME/.local/bin/shotcut-flatpak # Setup environment variables for the mcp server
export SHOTCUT_MELT_PATH=$HOME/.local/bin/shotcut-melt

# Clone repo and install 
git clone git@github.com:juanmed/photobot.git
git config core.hooksPath .githooks   # Enable repository quality hooks
git submodule update --init --recursive shotcut-mcp   # fetch the shotcut-mcp submodule source
uv sync
python3 shotcut-mcp/scripts/shotcut_mcp_server.py --check   # verify the server starts
claude mcp add --transport stdio shotcut -- python3 shotcut-mcp/scripts/shotcut_mcp_server.py   # register the server for this project only
claude mcp list   # confirm the shotcut server is registered

# Codegraph 
```

## Development checks

Run pre-commit hooks manually using `ruff` and unit tests using `pytest`:

```bash
uv run ruff format --check path/to/file.py
uv run pytest --cov=sw.livestream --cov=sw.shotcut_watermark --cov=sw.shotcut_pipeline --cov-report=term-missing sw/tests/
```

## Robot motions

A GoPro on the Fairino FR5 runs pre-made camera moves (the horizon stays
level) while `sw/livestream.py` records. The app never creates or edits a
trajectory: it only plays motions that were published beforehand. The full
design is in `docs/design/feature/fairino-python-basic-control.md`.

### Making a motion (`motionctl`)

```bash
M="uv run python -m sw.robot.motionctl"
$M new NAME --template first_test --tool 1 --interpolation joint
$M capture NAME --t 0 --reset --ip 192.168.58.2   # jog to each keyframe, then capture
$M capture NAME --t 2 --ip 192.168.58.2
$M validate NAME --ip 192.168.58.2                # kinematics only; nothing moves
$M fit NAME --write                               # fastest timing within the limits
$M compile NAME --ip 192.168.58.2
$M simplay NAME                                   # optional preview on the simulator
$M play NAME --ovl 25 --ip 192.168.58.2           # operator test on the robot (asks YES)
$M publish NAME --ovl 100 --ip 192.168.58.2       # upload + write motion.json
$M list --check --ip 192.168.58.2
```

Capture, validate and compile against the robot that will play the file
(`--ip 192.168.58.2`). The simulator's arm model is about 28 mm off the real
robot's, and the controller rejects files computed on it.

### In the app

```bash
uv run python -m sw.livestream               # with the robot (IP from the config)
uv run python -m sw.livestream --no-robot    # recording only
```

1. At startup the app checks every published motion on the controller (its
   MD5 hash). A motion that fails stays visible but disabled; hover over it to
   see why.
2. Click a motion and confirm: it is loaded. **The robot does not move.**
3. **Move to Start** (orange while the robot is not at the start pose) moves
   the robot at 25 % speed. While it moves, the same button reads **Stop robot**.
4. **Start recording** with a motion selected checks that the robot is at the
   start pose, starts the GoPro, waits `pre_roll_s`, plays the motion, and stops
   recording `post_roll_s` after it ends. **Stop recording** during a motion
   stops the robot first. Closing the window stops the robot.
5. After a robot error, the button reads **Clear fault**. With "No motion"
   selected, recording works as before.

### Safety rules

- Nobody in the cell while the robot moves; keep the e-stop in hand. The
  physical e-stop is the primary safety device.
- Do not run the app while publishing motions.
- Re-run `publish` after changing a motion: the app refuses a controller file
  whose MD5 differs from `motion.json`.

### Config keys (`sw/livestream_config.json`)

- `"motion"`: the selected motion, or `""` for none.
- `"robot"` (all optional): `{"ip": "192.168.58.2", "start_tolerance_deg": 0.5,
  "move_to_start_speed_pct": 25, "safe_default_speed_pct": 10,
  "pre_roll_s": 0.5, "post_roll_s": 1.0, "auto_stop_after_motion": true}`.
