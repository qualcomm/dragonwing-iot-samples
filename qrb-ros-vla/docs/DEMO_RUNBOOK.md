# Demo runbook — Pi0.5 flexibility lab

Presenter-facing operating instructions for the live demo: one frozen VLA on the Hexagon NPUs of a
Qualcomm Dragonwing IQ-9075 EVK, driving a simulated Panda arm from a browser.

Every number here was measured on this board. Talk narrative and slide-level framing live in
[`IQ_9075_ROBOTICS_TALK.md`](IQ_9075_ROBOTICS_TALK.md); the method behind the measurements is in the
[README](../README.md) and [`DESIGN.md`](DESIGN.md).

---

## 1. Before the talk (once, ~5 min)

### 1.1 Pre-flight the board

All five must pass. Run from the repository root:

```bash
cd /home/ubuntu/QRB-ROS-VLA
ls /dev/fastrpc-cdsp /dev/dma_heap/system
cat /sys/class/remoteproc/remoteproc*/state | sort -u
du -sh artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075
ls artifacts/paligemma_tokenizer.model artifacts/libero_ep0.npz
ls ros2_ws/install/setup.bash /home/ubuntu/libero/libero-env.sh
```

Expected output:

```text
/dev/dma_heap/system
/dev/fastrpc-cdsp
running
2.9G	artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075
artifacts/libero_ep0.npz  artifacts/paligemma_tokenizer.model
ros2_ws/install/setup.bash  /home/ubuntu/libero/libero-env.sh
```

`running` must be the **only** line from the remoteproc states — a stopped Hexagon means the model
will silently fall back to CPU or fail to load.

### 1.2 Know your URL

Find the board's address and write it down:

```bash
ip -4 -o addr show scope global | awk '{print $2, $4}'
```

At time of writing that is `end0 192.168.1.166/24`, so the demo URL is **`http://192.168.1.166:8080`**.
DHCP moves; re-check on the day. Load the page on the presenting laptop *before* the room fills.

### 1.3 Rehearse once, end to end

Cold start to first arm motion is about 90 s. Do not discover that live.

---

## 2. Launch (~80 s before you speak)

```bash
cd /home/ubuntu/QRB-ROS-VLA && scripts/start-libero-web-demo.sh
```

This opens a tmux session with two panes. **Wait for both readiness markers** — starting early looks
like a hang:

| Pane | Marker | Takes | Why |
|---|---|---|---|
| left | `pi0.5 bundle loaded; waiting for 2 camera streams and a task` | ~45 s | maps 2.9 GB of context binaries across both Hexagons |
| right | `flexibility lab on http://0.0.0.0:8080 - 41 scenes, 9 scripted gestures` | ~35 s | LIBERO + MuJoCo + scene catalog |

Then open the URL. You should see the arm, a black bowl, a plate, and two small camera tiles.

**Project the browser from the laptop, not the terminal.**

### 2.1 Smoke test before you talk

Type `pick up the black bowl`. Confirm:

- the arm moves within ~2 s,
- the telemetry block turns green: `NPU confirmed: libQnnHtp.so`,
- chunk latency reads roughly 1100 ms.

Then press **Reset everything** so you open on a clean stage.

### 2.2 Options

```bash
PORT=8090 scripts/start-libero-web-demo.sh                         # different port
WEB_ARGS='--scene libero_goal:3' scripts/start-libero-web-demo.sh  # open on another world
WEB_ARGS='--no-realtime' scripts/start-libero-web-demo.sh          # ignore the 10 Hz pacing
```

---

## 3. The seven-minute run

| # | Do | Say |
|---|---|---|
| 1 | Sandbox world, type `put the black bowl on the plate` | 3B VLA, both Hexagons, ~1.1 s per 50-step chunk, nothing leaves the board |
| 2 | Point at the two small camera tiles | that is all the model sees — 256 px, two cameras; the big view is for you, not for it |
| 3 | Switch to `goal #3`, click the **green** chip | same weights, new world — I changed the simulator, not the model |
| 4 | Type an **amber** chip, e.g. `pick up the wine bottle` | a sentence it was never demonstrated on, in a world it knows |
| 5 | **A/B probe** on two object nouns | 3–8× the noise floor: proof the words matter, measured live, not a vibe |
| 6 | **A/B probe** `wave hi` vs `do nothing at all` | ~1×: identical. This checkpoint is LIBERO-tuned, and here is the edge |
| 7 | Click the scripted **wave hi** gesture | so this one is choreography, and I am telling you rather than letting you assume |

**Step 6 into step 7 is the point of the whole demo.** Do not skip it. Stating your own failure mode is
what converts the skeptic in the front row into an ally; a demo that hides it does not survive their
question.

### 3.1 What the chip colours mean

- **green** — the instruction this world ships with, i.e. the phrasing the policy was demonstrated on.
- **amber** — this world's objects in wording the policy was never shown. In-distribution skill,
  out-of-distribution sentence. This is what "flexible" honestly means here.
- **blue (scripted lane)** — hand-written trajectory, no model in the loop.

### 3.2 The A/B probe, in one breath

It freezes one observation and asks for three chunks: prompt A, prompt B, then A again. The third is
the repeat-noise floor. A pair counts as separated only above **2×** that floor. Object-noun swaps
measure 3–8×; non-manipulation prompts measure 0.9–1.8×. Takes ~3.5 s.

It resets to the scene start pose by default, and should stay that way on stage: measured mid-grasp
with the gripper already closing on the bowl, the same pair drops to 0.9× — the visual context has
already decided what happens next. Untick the box only if someone asks that exact question.

---

## 4. Recovery — every control is safe to press mid-motion

| Control | Does | Cost |
|---|---|---|
| **Stop** | freezes the current lane, leaves the scene untouched | instant |
| **Reset scene** | arm and objects back to this world's start pose | < 1 s |
| **Reset everything** | rebuilds the startup world, blanks every panel | ~2.7 s |

Use **Stop** when the arm is doing something embarrassing and you want to talk over it. Use **Reset
everything** between audience volunteers.

Interrupting an A/B probe mid-sequence is honoured in ~1.1 s (the chunk already in flight) rather than
the ~3.3 s all three would take. Last click always wins, and a lane you cancelled never repaints the
screen afterwards.

**Never restart the process to recover** — that costs 80 s. All three controls keep the model loaded.

---

## 5. If something goes wrong

| Symptom | Cause | Fix |
|---|---|---|
| Log says `backend: waiting`, nothing moves | left pane not ready yet | wait for `bundle loaded` |
| `no action chunk returned; is the VLA node up?` | VLA node died | check the left pane; restart that pane only |
| Page loads but the video is black | opened before the simulator finished | reload the page |
| Arm swept an object onto the floor | expected in free play; there is no auto-recovery by design | **Reset everything** |
| Chunk latency ≫ 1.2 s | thermal throttling, or two VLA nodes running | `ros2 node list` must show exactly one `/qrb_ros_vla` |
| Scene picker empty | LIBERO not on the path | source `/home/ubuntu/libero/libero-env.sh` |
| Browser cannot reach the board | laptop hopped networks | re-check `ip -4 -o addr show scope global`; prefer wired |
| Note reads `simulator re-posed…` | the simulator refused a step and the scene self-healed | nothing; carry on, or **Reset everything** |

**You cannot run the demo "too long."** The simulator is built with `ignore_done=True`, so the episode
never ends: no horizon cut-off, no goal predicate, no surprise reset mid-sentence. Verified at **10,744
continuous steps** in one episode (~18× robosuite's default 1000-step horizon), after which the VLA lane
still returned chunks at 1114 ms and a full reset still worked. If the simulator ever does refuse a step,
the scene re-poses itself, the lane drops to idle, and the note explains it — the process does not die.

**Biggest non-technical risk:** this serves over your LAN. A projector or venue network switching the
laptop to a captive portal kills the page for reasons that have nothing to do with the NPU. Keep the
board IP written down, and prefer a wired link.

---

## 6. Two things not to say

- **Do not say it "understands" a gesture.** Measured: `wave hi` and `do nothing at all` produce
  action chunks that are indistinguishable at the noise floor. Say that *object references* work and
  that gestures are scripted.
- **Do not quote a latency you have not seen on screen that day.** It is live in the panel. Read that
  number.

---

## 7. Appendix — driving it without a browser

Every control is a plain HTTP endpoint, so a dead browser is not a dead demo. From the board:

```bash
BASE=http://127.0.0.1:8080
curl -s $BASE/api/state | python3 -m json.tool | head -20
curl -s -X POST $BASE/api/task    -H 'Content-Type: application/json' -d '{"task":"pick up the black bowl"}'
curl -s -X POST $BASE/api/gesture -H 'Content-Type: application/json' -d '{"name":"wave_hi"}'
curl -s -X POST $BASE/api/scene   -H 'Content-Type: application/json' -d '{"scene_id":"libero_goal:3"}'
curl -s -X POST $BASE/api/probe   -H 'Content-Type: application/json' -d '{"a":"pick up the black bowl","b":"pick up the plate"}'
curl -s -X POST $BASE/api/stop
curl -s -X POST $BASE/api/reset   -H 'Content-Type: application/json' -d '{"all":true}'
curl -s $BASE/api/catalog | python3 -c 'import json,sys; d=json.load(sys.stdin); print(len(d["scenes"]), "scenes;", [g["name"] for g in d["gestures"]])'
```

Live video streams are `$BASE/mjpeg?kind=presenter`, `?kind=model0`, `?kind=model1`. State pushes over
Server-Sent Events at `$BASE/stream`.

## 8. Appendix — the offline measurement

The number quoted in step 5 is reproducible without the browser, with the VLA node already running:

```bash
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
source /home/ubuntu/libero/libero-env.sh
${LIBERO_VENV_PYTHON:-python3} bench/prompt_probe.py --compare \
  --prompts "pick up the black bowl" "pick up the plate" "wave hi" "do nothing at all"
```

Raw output lands in `bench/prompt-probe/`. Expect a repeat-noise floor near 0.12, an object swap near
0.52, and `wave hi` vs `do nothing at all` near the floor.
