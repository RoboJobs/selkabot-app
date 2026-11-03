import csv
import json
import random
import subprocess
from pathlib import Path

ROOT = Path("/home/juan/Documents/photobot")
VIDEOS = ROOT / "videos"
OUT = VIDEOS / "compilation"
SCRATCH = Path("/tmp/claude-1000/-home-juan-Documents-photobot/c257327b-97b9-4911-9080-de3a288fd84a/scratchpad")
SEED = 7
W, H = 1920, 1080
FPS = "60000/1001"
STILL_LEN = 3.0
FADE_IN_LEN = 1.0
DISSOLVE_LEN = 0.7
PREP_LEN = 0.7
X_PREP_LEN = 0.5
MOI_LEN = 4.0
MONTAGE_START = 3.0
MONTAGE_SLICE = 1.5
MONTAGE_TAIL = 3.0
MONTAGE_COUNT = 3
STAGE2_SLOTS = [1.0, 1.0, 1.0, 2.0]
STAGE2_START = 7.0
STAGE3_START = 12.0
STAGE3_SLOT = 0.3
STAGE3_COUNT = 6
FINAL_FADE = 3.0
VSCALE = (
    f"scale={W}:{H}:force_original_aspect_ratio=decrease,pad={W}:{H}:(ow-iw)/2:(oh-ih)/2,"
    f"setsar=1,fps={FPS},format=yuv420p"
)
AFORMAT = "aformat=sample_rates=48000:channel_layouts=stereo"


def probe(path):
    out = subprocess.run(
        ["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", str(path)],
        capture_output=True, text=True, check=True,
    ).stdout
    return float(out)


rows = []
with open(ROOT / "startimes.csv") as f:
    for name, start in csv.reader(f):
        h, m, s = start.split(":")
        rows.append((name, int(h) * 3600 + int(m) * 60 + float(s)))
durations = {name: probe(VIDEOS / name) for name, _ in rows}

rng = random.Random(SEED)
candidates = [(n, t) for n, t in rows if t >= PREP_LEN]
rng.shuffle(candidates)

# Same draws as v4, so the opening, prep and MOI clips stay the same.
x_name, x_start = candidates.pop(0)
prep = [candidates.pop(0) for _ in range(3)]
montage = [c for c in candidates if c[0] != x_name][:MONTAGE_COUNT]

a_name, a_start = prep[0]
assert a_start > STILL_LEN, "the dissolve video must start after the still's 3 s"
a_ss = rng.uniform(0, a_start - DISSOLVE_LEN)
prep_bc = []
for name, start in prep[1:]:
    prep_bc.append((name, round(rng.uniform(0, start - PREP_LEN), 3)))

used = {x_name, *[n for n, _ in prep], *[n for n, _ in montage]}
free = [(n, t) for n, t in rows if n not in used]

# Stage 2: three random videos, each showing its own MOI seconds 7 to 10, the last one to 12.
elig2 = [(n, t) for n, t in free if durations[n] >= t + STAGE2_START + sum(STAGE2_SLOTS)]
stage2 = rng.sample(elig2, 3)

# Stage 3: six random videos, each shown for 0.3 s, starting at MOI second 12.
free3 = [(n, t) for n, t in free if n not in {s[0] for s in stage2}]
end3 = STAGE3_START + STAGE3_SLOT * STAGE3_COUNT
v6_pool = [(n, t) for n, t in free3 if durations[n] >= t + end3 + FINAL_FADE]
v6 = rng.choice(v6_pool)
others_pool = [(n, t) for n, t in free3 if n != v6[0] and durations[n] >= t + end3]
stage3 = rng.sample(others_pool, STAGE3_COUNT - 1) + [v6]

# Fourth stage 2 clip, drawn from its own RNG so the picks above stay the same.
extra_pool = [c for c in elig2 if c[0] not in {s[0] for s in stage2} | {s[0] for s in stage3}]
stage2.append(random.Random(SEED + 1).choice(extra_pool))

final_ss = round(v6[1] + STAGE3_START + STAGE3_SLOT * (STAGE3_COUNT - 1), 3)
final_len = round(durations[v6[0]] - final_ss, 3)
assert final_len >= FINAL_FADE, "the final clip must run at least as long as the selkabot fade"

# Opening: JDC still fades in from black, then dissolves into the first prep video.
inputs = [
    "-loop", "1", "-t", str(STILL_LEN + DISSOLVE_LEN), "-framerate", FPS, "-i", str(ROOT / "jdc.jpg"),
    "-ss", str(round(a_ss, 3)), "-t", str(DISSOLVE_LEN), "-i", str(VIDEOS / a_name),
]
filters = [
    f"[0:v]{VSCALE},fade=t=in:st=0:d={FADE_IN_LEN}:color=black,setpts=PTS-STARTPTS[s]",
    f"[1:v]{VSCALE},setpts=PTS-STARTPTS[av]",
    f"anullsrc=r=48000:cl=stereo,atrim=0:{STILL_LEN + DISSOLVE_LEN},asetpts=PTS-STARTPTS[sa]",
    f"[1:a]{AFORMAT},asetpts=PTS-STARTPTS[aa]",
    f"[s][av]xfade=transition=fade:duration={DISSOLVE_LEN}:offset={STILL_LEN}[v0]",
    f"[sa][aa]acrossfade=d={DISSOLVE_LEN}[a0]",
]
segments = [{"kind": "still+dissolve", "src": "jdc.jpg -> " + a_name, "ss": round(a_ss, 3),
             "dur": STILL_LEN + DISSOLVE_LEN, "timeline": 0.0}]
t = STILL_LEN + DISSOLVE_LEN

# Body clips after the opening, in playback order.
body = []
for name, ss in prep_bc:
    body.append(("prep", name, ss, PREP_LEN))
body.append(("moi_x", x_name, round(x_start - X_PREP_LEN, 3), X_PREP_LEN + MOI_LEN))
for i, (name, start) in enumerate(montage):
    dur = MONTAGE_SLICE + (MONTAGE_TAIL if i == MONTAGE_COUNT - 1 else 0)
    body.append(("montage", name, round(start + MONTAGE_START, 3), dur))
for i, (name, start) in enumerate(stage2):
    body.append(("stage2", name, round(start + STAGE2_START + sum(STAGE2_SLOTS[:i]), 3), STAGE2_SLOTS[i]))
for k, (name, start) in enumerate(stage3[:-1]):
    body.append(("stage3", name, round(start + STAGE3_START + STAGE3_SLOT * k, 3), STAGE3_SLOT))
body.append(("final", v6[0], final_ss, final_len))

pieces = ["[v0][a0]"]
for n, (kind, name, ss, dur) in enumerate(body, start=2):
    inputs += ["-ss", str(ss), "-t", str(dur), "-i", str(VIDEOS / name)]
    filters.append(f"[{n}:v]{VSCALE},setpts=PTS-STARTPTS[v{n}]")
    filters.append(f"[{n}:a]{AFORMAT},asetpts=PTS-STARTPTS[a{n}]")
    pieces.append(f"[v{n}][a{n}]")
    segments.append({"kind": kind, "src": name, "ss": ss, "dur": dur, "timeline": round(t, 3)})
    t += dur

# Selkabot: the final clip dissolves into selkabot.png over its last 3 s.
last = len(body) + 1
sel = last + 1
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
(SCRATCH / "moi_plan_v6.json").write_text(json.dumps({"seed": SEED, "total": total, "segments": segments}, indent=2))
print(f"planned total: {total:.2f}s, {len(segments)} segments")
for s in segments:
    print(f"{s['timeline']:6.2f}  {s['kind']:15s} {s['src']:50s} ss={s['ss']:>8} dur={s['dur']}")

out_file = OUT / "test_moi_compilation_v6.mp4"
cmd = (
    ["ffmpeg", "-y", "-hide_banner", "-loglevel", "error"]
    + inputs
    + ["-filter_complex", ";".join(filters), "-map", "[vout]", "-map", "[aout]",
       "-c:v", "libx264", "-preset", "medium", "-crf", "20",
       "-c:a", "aac", "-b:a", "192k", "-movflags", "+faststart", str(out_file)]
)
subprocess.run(cmd, check=True)
print("wrote", out_file)
