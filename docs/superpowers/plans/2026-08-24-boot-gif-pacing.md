# Boot GIF Pacing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace all five public NeXT boot GIFs with contextually accelerated 15 fps versions and a three-second final hold.

**Architecture:** The lab encoder lowers its documented and tested minimum/default hold from five seconds to three. One-off FFmpeg filter graphs retime manually reviewed intervals from the lossless capture sources into new FFV1 intermediates; the existing two-pass encoder then produces and verifies the GIFs. The lab receives the canonical copies first, after which QEMU receives byte-identical assets at the existing README URLs.

**Tech Stack:** Python 3 standard library and unittest, Pillow, FFmpeg/FFprobe, Git, Markdown.

---

### Task 1: Change the lab GIF hold contract to three seconds

**Files:**
- Modify: `/home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-lab/tests/test_boot_gif.py`
- Modify: `/home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-lab/nextcube_lab/boot_gif.py`
- Modify: `/home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-lab/README.md`

- [ ] **Step 1: Write the failing three-second contract tests**

Change `test_encode_filter_uses_exact_default_filter` to call `encode_filter()` and expect:

```python
self.assertEqual(
    encode_filter(),
    "fps=15,tpad=stop_mode=clone:stop_duration=3.000",
)
```

Change invalid hold inputs from `4.99` and `4.0` to `2.99` and `2.0`. Change the real encoder assertion to:

```python
self.assertGreaterEqual(report.identical_tail_seconds, 3.0)
```

Change the mocked no-shortfall durations from `4920`/`4930` and expected `5.0` to `2920`/`2930` and expected `3.0`. Change the exact generated FFmpeg chain to end in:

```text
tpad=stop_mode=clone:stop_duration=3.000
```

- [ ] **Step 2: Run the focused tests and verify RED**

Run:

```sh
python3 -B -m unittest \
  tests.test_boot_gif.GifEncodingApiTests.test_encode_filter_uses_exact_default_filter \
  tests.test_boot_gif.GifEncodingApiTests.test_encode_filter_rejects_strict_invalid_arguments \
  tests.test_boot_gif.GifEncodingApiTests.test_verify_accepts_repeated_identical_tail_frames \
  tests.test_boot_gif.GifEncodingApiTests.test_verify_tail_hold_has_no_shortfall_tolerance \
  tests.test_boot_gif.GifEncodingApiTests.test_encode_builds_both_required_ffmpeg_passes
```

Expected: failures still report or generate the five-second contract.

- [ ] **Step 3: Implement the three-second default and minimum**

In `nextcube_lab/boot_gif.py`, add one constant near the GIF validation helpers:

```python
_GIF_MINIMUM_HOLD_SECONDS = 3.0
```

Use it in `_gif_hold()` and update its message to `at least 3 seconds`. Change the default `end_hold_seconds` argument to `3.0` in `encode_filter()`, `encode_gif()`, and `verify_gif()`. Change both CLI `--end-hold` defaults to `3.0`.

In `README.md`, change `adds at least five seconds` to `adds at least three seconds`.

- [ ] **Step 4: Run focused and complete lab tests**

Run:

```sh
python3 -B -m unittest tests.test_boot_gif
python3 -B -m unittest discover -s tests
git diff --check
```

Expected: 926 tests pass with one existing opt-in skip, and `git diff --check` exits zero.

- [ ] **Step 5: Commit the hold contract**

```sh
git add README.md nextcube_lab/boot_gif.py tests/test_boot_gif.py
git commit -m "docs: shorten boot GIF endpoint holds"
```

### Task 2: Produce the five contextually retimed FFV1 sources

**Files:**
- Read: `/tmp/nextcube-final-captures/netbsd-tailtrim.mkv`
- Read: `/tmp/nextcube-final-captures/nextstep.mkv`
- Read: `/tmp/nextcube-final-captures/plan9-tcp-clean.mkv`
- Read: `/tmp/nextcube-final-captures/plan9-il-clean.mkv`
- Create outside Git: `/tmp/nextcube-final-captures/retimed-*.mkv`

- [ ] **Step 1: Create the full NetBSD retime**

```sh
ffmpeg -n -i /tmp/nextcube-final-captures/netbsd-tailtrim.mkv \
  -filter_complex '[0:v]trim=start=0:end=30,setpts=(PTS-STARTPTS)/4[v0];[0:v]trim=start=30:end=130,setpts=(PTS-STARTPTS)/2[v1];[0:v]trim=start=130:end=155,setpts=PTS-STARTPTS[v2];[v0][v1][v2]concat=n=3:v=1:a=0,fps=15[v]' \
  -map '[v]' -an -c:v ffv1 -level 3 -g 1 \
  /tmp/nextcube-final-captures/retimed-netbsd-full.mkv
```

Expected: approximately 82.5 seconds before the encoder's final hold.

- [ ] **Step 2: Create the network-only NetBSD retime**

```sh
ffmpeg -n -i /tmp/nextcube-final-captures/netbsd-tailtrim.mkv \
  -filter_complex '[0:v]trim=start=80:end=130,setpts=(PTS-STARTPTS)/2[v0];[0:v]trim=start=130:end=155,setpts=PTS-STARTPTS[v1];[v0][v1]concat=n=2:v=1:a=0,fps=15[v]' \
  -map '[v]' -an -c:v ffv1 -level 3 -g 1 \
  /tmp/nextcube-final-captures/retimed-netbsd-network.mkv
```

Expected: approximately 50 seconds before the final hold.

- [ ] **Step 3: Create the NeXTSTEP retime**

```sh
ffmpeg -n -i /tmp/nextcube-final-captures/nextstep.mkv \
  -filter_complex '[0:v]trim=start=0:end=15,setpts=(PTS-STARTPTS)/4[v0];[0:v]trim=start=15:end=30,setpts=(PTS-STARTPTS)/2[v1];[0:v]trim=start=30:end=75,setpts=(PTS-STARTPTS)/4[v2];[0:v]trim=start=75:end=86,setpts=PTS-STARTPTS[v3];[v0][v1][v2][v3]concat=n=4:v=1:a=0,fps=15[v]' \
  -map '[v]' -an -c:v ffv1 -level 3 -g 1 \
  /tmp/nextcube-final-captures/retimed-nextstep.mkv
```

Expected: approximately 33.5 seconds before the final hold.

- [ ] **Step 4: Create the Plan 9 TCP retime**

```sh
ffmpeg -n -i /tmp/nextcube-final-captures/plan9-tcp-clean.mkv \
  -filter_complex '[0:v]trim=start=0:end=60,setpts=(PTS-STARTPTS)/4[v0];[0:v]trim=start=60:end=130,setpts=(PTS-STARTPTS)/2[v1];[0:v]trim=start=130:end=151,setpts=PTS-STARTPTS[v2];[v0][v1][v2]concat=n=3:v=1:a=0,fps=15[v]' \
  -map '[v]' -an -c:v ffv1 -level 3 -g 1 \
  /tmp/nextcube-final-captures/retimed-plan9-tcp.mkv
```

Expected: approximately 71 seconds before the final hold.

- [ ] **Step 5: Create the Plan 9 IL retime**

```sh
ffmpeg -n -i /tmp/nextcube-final-captures/plan9-il-clean.mkv \
  -filter_complex '[0:v]trim=start=0:end=45,setpts=(PTS-STARTPTS)/4[v0];[0:v]trim=start=45:end=55,setpts=(PTS-STARTPTS)/2[v1];[0:v]trim=start=55:end=90,setpts=(PTS-STARTPTS)/4[v2];[0:v]trim=start=90:end=100,setpts=PTS-STARTPTS[v3];[0:v]trim=start=100:end=135,setpts=(PTS-STARTPTS)/4[v4];[0:v]trim=start=135:end=145,setpts=PTS-STARTPTS[v5];[0:v]trim=start=145:end=170,setpts=(PTS-STARTPTS)/4[v6];[0:v]trim=start=170:end=180,setpts=PTS-STARTPTS[v7];[0:v]trim=start=180:end=240,setpts=(PTS-STARTPTS)/4[v8];[0:v]trim=start=240:end=264,setpts=PTS-STARTPTS[v9];[v0][v1][v2][v3][v4][v5][v6][v7][v8][v9]concat=n=10:v=1:a=0,fps=15[v]' \
  -map '[v]' -an -c:v ffv1 -level 3 -g 1 \
  /tmp/nextcube-final-captures/retimed-plan9-il.mkv
```

Expected: approximately 109 seconds before the final hold.

- [ ] **Step 6: Verify retimed source cadence, dimensions, and duration**

Run `ffprobe -v error -count_frames -select_streams v:0 -show_entries stream=avg_frame_rate,nb_read_frames,width,height -show_entries format=duration -of json` on every `retimed-*.mkv`.

Expected: `avg_frame_rate` is `15/1`; NeXTSTEP and Plan 9 are 1134×891; NetBSD is 1128×885; durations are within one 15 fps frame of the values above.

### Task 3: Encode, inspect, and commit the lab GIFs

**Files:**
- Modify: `/home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-lab/docs/boot/netbsd-full.gif`
- Modify: `/home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-lab/docs/boot/netbsd-network.gif`
- Modify: `/home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-lab/docs/boot/nextstep.gif`
- Modify: `/home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-lab/docs/boot/plan9-tcp.gif`
- Modify: `/home/blanham/projects/NeXT/lab/.worktrees/netbsd-nfs-lab/docs/boot/plan9-il.gif`

- [ ] **Step 1: Encode to fresh temporary GIF paths**

```sh
python3 -B scripts/boot_gif.py encode \
  --source /tmp/nextcube-final-captures/retimed-netbsd-full.mkv \
  --timeline /tmp/nextcube-final-captures/netbsd-full.mkv.timeline.json \
  --end-hold 3 \
  --out /tmp/nextcube-final-captures/faster-netbsd-full.gif

python3 -B scripts/boot_gif.py encode \
  --source /tmp/nextcube-final-captures/retimed-netbsd-network.mkv \
  --timeline /tmp/nextcube-final-captures/netbsd-full.mkv.timeline.json \
  --end-hold 3 \
  --out /tmp/nextcube-final-captures/faster-netbsd-network.gif

python3 -B scripts/boot_gif.py encode \
  --source /tmp/nextcube-final-captures/retimed-nextstep.mkv \
  --timeline /tmp/nextcube-final-captures/nextstep.mkv.timeline.json \
  --end-hold 3 \
  --out /tmp/nextcube-final-captures/faster-nextstep.gif

python3 -B scripts/boot_gif.py encode \
  --source /tmp/nextcube-final-captures/retimed-plan9-tcp.mkv \
  --timeline /tmp/nextcube-final-captures/plan9-tcp-4.mkv.timeline.json \
  --end-hold 3 \
  --out /tmp/nextcube-final-captures/faster-plan9-tcp.gif

python3 -B scripts/boot_gif.py encode \
  --source /tmp/nextcube-final-captures/retimed-plan9-il.mkv \
  --timeline /tmp/nextcube-final-captures/plan9-il.mkv.timeline.json \
  --end-hold 3 \
  --out /tmp/nextcube-final-captures/faster-plan9-il.gif
```

- [ ] **Step 2: Inspect beginning, interaction, and final frames**

Decode contact sheets from each candidate at one-second intervals. Confirm changing visible output within the first five playback seconds, readable prompt/login transitions, a decorated QEMU frame with no X-root spill, and the same final shell or desktop.

- [ ] **Step 3: Verify every candidate**

```sh
for gif in /tmp/nextcube-final-captures/faster-*.gif; do
  python3 -B scripts/boot_gif.py verify --gif "$gif" --end-hold 3
done
```

Expected: five passes at 15 fps, loop zero, at least 3.0 seconds of identical tail, and less than 20 MiB each.

- [ ] **Step 4: Replace and re-verify the lab assets**

Copy each candidate over its same-named `docs/boot/` destination, then run the same verifier on all five repository paths. Run `git diff --check` and `git status --short`.

- [ ] **Step 5: Commit the retimed lab assets**

```sh
git add docs/boot/netbsd-full.gif docs/boot/netbsd-network.gif \
  docs/boot/nextstep.gif docs/boot/plan9-tcp.gif docs/boot/plan9-il.gif
git commit -m "docs: accelerate guest boot recordings"
```

### Task 4: Merge the lab change and publish QEMU's replacements

**Files:**
- Modify: `/home/blanham/projects/NeXT/qemu/docs/boot/netbsd-full.gif`
- Modify: `/home/blanham/projects/NeXT/qemu/docs/boot/netbsd-network.gif`
- Modify: `/home/blanham/projects/NeXT/qemu/docs/boot/nextstep.gif`
- Modify: `/home/blanham/projects/NeXT/qemu/docs/boot/plan9-tcp.gif`
- Modify: `/home/blanham/projects/NeXT/qemu/docs/boot/plan9-il.gif`

- [ ] **Step 1: Merge the lab feature branch locally**

In `/home/blanham/projects/NeXT/lab`, merge `feature/netbsd-nfs` into `metachicken` with a non-fast-forward merge. Preserve all pre-existing untracked results and tools.

- [ ] **Step 2: Verify the merged lab result**

Run:

```sh
python3 -B -m unittest discover -s tests
for gif in docs/boot/*.gif; do
  python3 -B scripts/boot_gif.py verify --gif "$gif" --end-hold 3
done
git diff --check
```

Expected: 926 tests pass with one existing opt-in skip and all five GIFs pass.

- [ ] **Step 3: Copy byte-identical assets into QEMU**

Copy the five merged lab GIFs over the same five paths in `/home/blanham/projects/NeXT/qemu/docs/boot/`. Confirm each pair with `cmp` and `sha256sum`.

- [ ] **Step 4: Commit and push QEMU**

```sh
git add docs/boot/netbsd-full.gif docs/boot/netbsd-network.gif \
  docs/boot/nextstep.gif docs/boot/plan9-tcp.gif docs/boot/plan9-il.gif
git commit -m "docs: accelerate NeXT guest boot GIFs"
git push github metachicken
```

- [ ] **Step 5: Verify public publication**

Confirm `git ls-remote github refs/heads/metachicken` equals local `HEAD`, and confirm each existing `raw.githubusercontent.com/blanham/qemu-NeXT/metachicken/docs/boot/*.gif` URL returns HTTP 200 with `Content-Type: image/gif` and the expected byte count.
