import argparse
import json
import random
import subprocess
from pathlib import Path

ROOT = Path("/home/juan/Documents/photobot")
VIDEOS = ROOT / "videos"
OUT = VIDEOS / "compilation"
SEED = 777
W, H = 1920, 1080
FPS = "60000/1001"
OPENING_LEN = 4.0
SEQ_START = 4.0  # the prep and MOI clips start at this second of their own clip
PREP_LEN = 0.7
MOI_LEN = 4.0
MONTAGE_START = SEQ_START + 3.0
MONTAGE_SLICE = 1.5
MONTAGE_TAIL = 3.0
MONTAGE_COUNT = 3
STAGE2_SLOTS = [1.0, 1.0, 1.0, 2.0]
STAGE2_START = SEQ_START + 7.0
STAGE3_START = SEQ_START + 12.0
STAGE3_SLOT = 0.3
STAGE3_COUNT = 6
FINAL_FADE = 3.0
VSCALE = (
    f"scale={W}:{H}:force_original_aspect_ratio=decrease,pad={W}:{H}:(ow-iw)/2:(oh-ih)/2,"
    f"setsar=1,fps={FPS},format=yuv420p"
)
AFORMAT = "aformat=sample_rates=48000:channel_layouts=stereo"

# Every clip starts at 0 s. These are the lengths each role needs from its clip.
NEED = {
    "opening": OPENING_LEN,
    "moi_x": SEQ_START + MOI_LEN,
    "prep": SEQ_START + PREP_LEN,
    "montage": MONTAGE_START + MONTAGE_SLICE,
    "montage_last": MONTAGE_START + MONTAGE_SLICE + MONTAGE_TAIL,
    "stage2": STAGE2_START + sum(STAGE2_SLOTS),
    "stage3": STAGE3_START + STAGE3_SLOT * STAGE3_COUNT,
    "final": STAGE3_START + STAGE3_SLOT * (STAGE3_COUNT - 1) + FINAL_FADE,
}


def parse_args():
    p = argparse.ArgumentParser(description="Build the MOI compilation from one folder of *_watermarked.mp4 clips.")
    p.add_argument("--folder", required=True,
                   help="folder with the clips; a relative path is taken from the repo root")
    p.add_argument("--skip-list", default=str(Path(__file__).with_name("moi_skip_list.txt")),
                   help="text file with one clip per line to leave out (names or URLs, # comments allowed)")
    return p.parse_args()


def load_skip_list(path):
    """Return the set of clip file names listed in the skip file. Missing file means nothing skipped."""
    if not Path(path).is_file():
        return set()
    names = set()
    for line in Path(path).read_text().splitlines():
        entry = line.split("#", 1)[0].strip()
        if entry:
            names.add(entry.rstrip("/").rsplit("/", 1)[-1])
    return names


def probe(path):
    out = subprocess.run(
        ["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", str(path)],
        capture_output=True, text=True, check=True,
    ).stdout
    return float(out)


def take(pool, need, role):
    """Remove and return the first clip in pool that is at least `need` seconds long."""
    for name in pool:
        if durations[name] >= need:
            pool.remove(name)
            return name
    raise SystemExit(f"no clip left for {role}: needs {need:.2f}s")


args = parse_args()
folder = Path(args.folder)
if not folder.is_absolute():
    folder = ROOT / folder
if not folder.is_dir():
    raise SystemExit(f"not a folder: {folder}")

skip = load_skip_list(args.skip_list)
clips = sorted(
    p.name for p in folder.glob("*_watermarked.mp4")
    if not p.name.endswith("_watermarked_watermarked.mp4") and p.name not in skip
)
print(f"skipping {len(skip & {p.name for p in folder.glob('*.mp4')})} clips from {args.skip_list}")
if not clips:
    raise SystemExit(f"no *_watermarked.mp4 clips left in {folder} after the skip list")
durations = {name: probe(folder / name) for name in clips}

rng = random.Random(SEED)
order = clips.copy()
rng.shuffle(order)

# Opening: one random clip for its first 4 s, then three prep clips, the MOI clip, montage, stage 2, stage 3 and final.
opening_name = take(order, NEED["opening"], "opening")
x_name = take(order, NEED["moi_x"], "moi_x")
prep = [take(order, NEED["prep"], "prep") for _ in range(3)]
montage = []
for i in range(MONTAGE_COUNT):
    need = NEED["montage_last"] if i == MONTAGE_COUNT - 1 else NEED["montage"]
    montage.append(take(order, need, f"montage {i + 1}"))

# Stage 2: three random clips, each showing seconds 7 to 10 of its own clip, the last one to 12.
elig2 = [n for n in order if durations[n] >= NEED["stage2"]]
if len(elig2) < 4:
    raise SystemExit(f"need at least 4 clips for stage 2, have {len(elig2)}")
stage2 = rng.sample(elig2, 3)

# Stage 3: six random clips, each shown for 0.3 s, starting at second 12.
free3 = [n for n in order if n not in stage2]
v6_pool = [n for n in free3 if durations[n] >= NEED["final"]]
if not v6_pool:
    raise SystemExit("no clip long enough for the final shot")
v6 = rng.choice(v6_pool)
others_pool = [n for n in free3 if n != v6 and durations[n] >= NEED["stage3"]]
if len(others_pool) < STAGE3_COUNT - 1:
    raise SystemExit(f"need {STAGE3_COUNT - 1} clips for stage 3, have {len(others_pool)}")
stage3 = rng.sample(others_pool, STAGE3_COUNT - 1) + [v6]

# Fourth stage 2 clip, drawn from its own RNG.
extra_pool = [n for n in elig2 if n not in set(stage2) | set(stage3)]
stage2.append(random.Random(SEED + 1).choice(extra_pool))

final_ss = round(STAGE3_START + STAGE3_SLOT * (STAGE3_COUNT - 1), 3)
final_len = round(durations[v6] - final_ss, 3)
assert final_len >= FINAL_FADE, "the final clip must run at least as long as the selkabot fade"

# Clips in playback order. The watermarked clips already carry the watermark image.
inputs = []
filters = []
segments = []
t = 0.0

body = [("opening", opening_name, 0.0, OPENING_LEN)]
for name in prep:
    body.append(("prep", name, SEQ_START, PREP_LEN))
body.append(("moi_x", x_name, SEQ_START, MOI_LEN))
for i, name in enumerate(montage):
    dur = MONTAGE_SLICE + (MONTAGE_TAIL if i == MONTAGE_COUNT - 1 else 0)
    body.append(("montage", name, MONTAGE_START, dur))
for i, name in enumerate(stage2):
    body.append(("stage2", name, STAGE2_START + sum(STAGE2_SLOTS[:i]), STAGE2_SLOTS[i]))
for k, name in enumerate(stage3[:-1]):
    body.append(("stage3", name, STAGE3_START + STAGE3_SLOT * k, STAGE3_SLOT))
body.append(("final", v6, final_ss, final_len))

pieces = []
for n, (kind, name, ss, dur) in enumerate(body):
    inputs += ["-ss", str(ss), "-t", str(dur), "-i", str(folder / name)]
    filters.append(f"[{n}:v]{VSCALE},setpts=PTS-STARTPTS[v{n}]")
    filters.append(f"[{n}:a]{AFORMAT},asetpts=PTS-STARTPTS[a{n}]")
    pieces.append(f"[v{n}][a{n}]")
    segments.append({"kind": kind, "src": name, "ss": ss, "dur": dur, "timeline": round(t, 3)})
    t += dur

# Selkabot: the final clip dissolves into selkabot.png over its last 3 s.
last = len(body) - 1
sel = len(body)
inputs += ["-loop", "1", "-t", str(FINAL_FADE), "-framerate", FPS, "-i", str(ROOT / "selkabot.png")]
filters += [
    f"[{sel}:v]{VSCALE},setpts=PTS-STARTPTS[vs]",
    f"anullsrc=r=48000:cl=stereo,atrim=0:{FINAL_FADE},asetpts=PTS-STARTPTS[as]",
    f"[v{last}][vs]xfade=transition=fade:duration={FINAL_FADE}:offset={round(final_len - FINAL_FADE, 3)}[vX]",
    f"[a{last}][as]acrossfade=d={FINAL_FADE}[aX]",
]
pieces[-1] = "[vX][aX]"
filters.append("".join(pieces) + f"concat=n={len(pieces)}:v=1:a=1[vout][aout]")
total = round(t, 3)

OUT.mkdir(parents=True, exist_ok=True)
name_stem = folder.name
plan_file = OUT / f"{name_stem}_moi_plan.json"
out_file = OUT / f"{name_stem}_moi_compilation.mp4"
plan_file.write_text(json.dumps({"seed": SEED, "folder": str(folder), "total": total, "segments": segments}, indent=2))
print(f"{len(clips)} clips in {folder}")
print(f"planned total: {total:.2f}s, {len(segments)} segments")
for s in segments:
    print(f"{s['timeline']:6.2f}  {s['kind']:15s} {s['src']:50s} ss={s['ss']:>8} dur={s['dur']}")

cmd = (
    ["ffmpeg", "-y", "-hide_banner", "-loglevel", "error"]
    + inputs
    + ["-filter_complex", ";".join(filters), "-map", "[vout]", "-map", "[aout]",
       "-c:v", "libx264", "-preset", "medium", "-crf", "20",
       "-c:a", "aac", "-b:a", "192k", "-movflags", "+faststart", str(out_file)]
)
subprocess.run(cmd, check=True)
print("wrote", out_file)
