# QRB ROS VLA - Pi0.5 on Qualcomm Dragonwing

Running [Pi0.5](https://aihub.qualcomm.com/models/pi05), a 3B-parameter vision-language-action
model, entirely on the dual Hexagon NPUs of a **Qualcomm Dragonwing IQ-9075 EVK**, inside a ROS 2
Jazzy node.

## Measured on the board

| | |
|---|---|
| Action chunk (50 steps × 7 DoF) | **1111 ms**, 0.90 chunks/s |
| Speed vs. real time | **4.5×** (LIBERO is 10 Hz, so 50 steps = 5 s of motion) |
| Numerical correctness | **45/45 output tensors bitwise identical** to Qualcomm's `qnn-net-run` |
| Accuracy vs. human demonstration | **normalized MAE 0.149** (15% of the dataset's action std) |
| CPU fallback | none — 100% NPU, on `libQnnHtp.so` |

Full method, breakdowns, and limitations: **[docs/DESIGN.md](docs/DESIGN.md)**.

## What you need

- A **Qualcomm Dragonwing IQ-9075 EVK** running Ubuntu 24.04 Server or newer.
- The board and your laptop on the same network.
- The `ubuntu` account on the board with `sudo` access.
- Internet access during setup. Expect about 3 GB of model/demo downloads plus Ubuntu/ROS packages.
- A Qualcomm AI Hub account/API token. The setup fetches the already-compiled Pi0.5 QNN context
  bundle; it does **not** run a live compile job and does not need a browser on the board.

For the most conservative clean-board setup, run Qualcomm's generic [Install Required Software Packages](https://dragonwingdocs.qualcomm.com/Ubuntu/devices/iq9075-evk/Install_required_software_packages) flow before this demo. It installs the broader board multimedia/AI baseline and may run `apt upgrade` and reboot. `scripts/iq9-one-shot-demo.sh` still installs the narrower ROS/QNN/LIBERO packages this repository directly uses, so rerunning it after the Qualcomm setup is expected.

The repository is source-only. Generated files under `artifacts/` are fetched or created on the board
and are intentionally ignored by git.

## First-time path

If you are new to the board, do these steps from top to bottom. Commands marked `LAPTOP` run on your
laptop. Commands marked `DEVICE` run inside the SSH session on the IQ-9075.

### 1. Get onto the board

Find the board IP from your router, DHCP table, serial console, or a monitor attached during setup.
On the board itself, this prints the IP addresses:

```bash
# DEVICE
hostname -I
```

From your laptop, SSH into the board. Replace `192.0.2.10` with the board IP you found:

```bash
# LAPTOP
export DEV_IP=192.0.2.10
ssh ubuntu@$DEV_IP
```

If this is a fresh image, change the default password before doing anything else:

```bash
# DEVICE
passwd
```

### 2. Install Qualcomm's baseline packages

Follow Qualcomm's [Install Required Software Packages](https://dragonwingdocs.qualcomm.com/Ubuntu/devices/iq9075-evk/Install_required_software_packages) page on the board before continuing. That script installs the generic IQ-9075 AI/multimedia baseline and may reboot the board.

After the reboot, reconnect from your laptop:

```bash
# LAPTOP
ssh ubuntu@$DEV_IP
```

Then continue in the SSH session.

### 3. Confirm this is the right target

```bash
# DEVICE
cat /etc/os-release | sed -n '1,4p'
uname -m
uname -r
nproc
ls /dev/fastrpc-cdsp /dev/dma_heap/system
df -h /
```

Expected output:

```text
Ubuntu 24.04 or newer
aarch64
a kernel name ending in -qcom
8
/dev/fastrpc-cdsp and /dev/dma_heap/system exist
at least 30G free on /
```

If `/dev/fastrpc-cdsp` exists but is not readable by `ubuntu`, add the user to the device group, then
log out and back in:

```bash
# DEVICE
sudo usermod -aG fastrpc $(id -un)
exit
```

### 4. Clone the source

```bash
# DEVICE
sudo apt-get update
sudo apt-get install -y git
git clone https://github.com/qualcomm/dragonwing-iot-samples.git ~/dragonwing-iot-samples
cd ~/dragonwing-iot-samples/qrb-ros-vla
```

### 5. Configure Qualcomm AI Hub access
The model fetch step uses `qai-hub-models fetch Pi0.5`. If you already have a pre-fetched bundle, copy
it to `artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075/` and skip authentication.

Otherwise, create/sign into a Qualcomm AI Hub account on your laptop, copy your API token, and keep it
ready. The one-shot setup installs `qai-hub-models`; if the Pi0.5 fetch fails, run the command printed
by the script to authenticate, then rerun the one-shot setup. A successful artifact fetch leaves these
files on the board:

```text
artifacts/paligemma_tokenizer.model
artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075/action_expert.bin
artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075/backbone.bin
artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075/token_emb.bin
artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075/vision_encoder.bin
artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075/metadata.json
artifacts/libero_ep0.npz
artifacts/manifest.sha256
```

### 6. Run the setup and demo

```bash
# DEVICE
cd ~/QRB-ROS-VLA
scripts/iq9-one-shot-demo.sh
```

This installs ROS 2 Jazzy, QRB ROS packages, QAIRT, Python helper tools, the tokenizer, the Pi0.5 QNN
context bundle, the LIBERO replay episode, the ROS workspace, and LIBERO web-demo dependencies. It then
starts the web demo in tmux. On a clean board, expect this to take a while; most time is package/model
download and the ROS workspace build.

If you want setup without launching the demo:

```bash
# DEVICE
scripts/iq9-one-shot-demo.sh --setup-only
```

### 7. Open the browser UI

The board is headless. Open the UI from your laptop browser, not on the board:

```text
http://BOARD_IP:8080
```

Use the same IP address you used for SSH.

If setup is already complete and you only need to reopen the demo:

```bash
# DEVICE
cd ~/QRB-ROS-VLA
scripts/start-libero-web-demo.sh
```

Expected proof that the NPU path is live appears in the ROS tmux pane:

```text
backend libQnnHtp.so
pi0.5 bundle loaded; waiting for 2 camera streams and a task
```

Submitting a prompt in the UI should produce `/qrb_ros_vla/stats` with `backend: libQnnHtp.so` and a
nonzero `chunks_per_second`.

### 8. If setup fails

Search the exact error text in [`workshop/TROUBLESHOOTING.md`](workshop/TROUBLESHOOTING.md). Common
first-run failures:

| Error text | Usual fix |
|---|---|
| `NO_PUBKEY` during `apt update` | Import the missing key shown in the error, then rerun setup |
| `Conflicting values set for option Trusted` | Delete the duplicate `qcom-ppa` source list entry; the stock image already has it |
| `Permission denied` opening `/dev/fastrpc-cdsp` | `sudo usermod -aG fastrpc $(id -un)`, then log out and back in |
| `Pi0.5 bundle fetch failed` | Authenticate Qualcomm AI Hub or copy a verified bundle into `artifacts/` |
| Browser cannot open port 8080 | Confirm the laptop and board are on the same network and the tmux web pane is still running |

## Quick commands after setup

Headless replay smoke test, no browser required:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

QNN_HTP_BURST=1 ros2 run qrb_ros_vla vla_node --ros-args \
  -p bundle_dir:=$PWD/artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075 \
  -p tokenizer_model:=$PWD/artifacts/paligemma_tokenizer.model \
  -p action_dof:=7 &

python3 demo/libero_replay.py --steps 15 --stride 12 --horizon 20
```

This feeds the NPU the exact observations a human demonstrator saw and scores the predicted actions
against what they actually did. No robot, no simulator, no display required.

### The live demo: Pi0.5 flexibility lab

An arm in simulation you drive from a browser, with no fixed instruction list. LIBERO supplies
physics, objects, and camera frames; ROS 2 carries images, state, and your prompt; Pi0.5 returns
action chunks from the Hexagon NPUs; the simulator executes them. Nothing resets on a benchmark
goal — free play has no goal, so the scene stays where you left it until you reset it.


Manual path, two terminals — each needs the same three `source` lines:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source /home/ubuntu/libero/libero-env.sh

ros2 launch qrb_ros_vla vla.launch.py                                       # terminal A
${LIBERO_VENV_PYTHON:-python3} demo/libero_web_demo.py --host 0.0.0.0 --port 8080   # terminal B
```

Open `http://BOARD_IP:8080`, using the same IP address you used for SSH. Three lanes, each labelled in the UI with what produced the motion:

| Lane | What runs | Use it to show |
|---|---|---|
| **VLA** | your text → `/qrb_ros_vla/task` → Pi0.5 on the NPUs → action chunk → `env.step` | one frozen policy taking arbitrary wording, replanning every 1.1 s against live frames |
| **A/B probe** | one frozen observation, prompts A, B, then A again | that the *words* are doing work — with a number, not a vibe |
| **scripted** | hand-written trajectories, `demo/scripted_gestures.py` | gestures the VLA provably cannot do (wave, nod, bow), honestly badged as choreography |

**41 worlds, one model.** The picker lists every installed LIBERO world (`libero_spatial`,
`libero_object`, `libero_goal`, `libero_10`) plus local sandboxes in `demo/scenes/`. Switching a world
reloads only the simulator — weights are never touched. Each world advertises the objects it actually
contains, its demonstrated instruction (green chip), and phrasings it was never shown (amber chips).

**Three ways out of a mess**, since long free play does knock objects off tables:

| Control | Does | Keeps |
|---|---|---|
| **Stop** | halts the current lane mid-chunk | the scene exactly where it stands |
| **Reset scene** | arm and objects back to this world's start pose, counters zeroed | the loaded world |
| **Reset everything** | rebuilds the startup world from scratch and clears task, counters, A/B result, telemetry, and action readout | nothing — it is the "get me back to a known stage" button |

`Reset everything` rebuilds the environment rather than re-posing it, so a world wedged into a state
`env.reset()` cannot undo (a drawer torn off, an object under the table) does not survive it. It costs
about 2.7 s and never reloads the model.

**Every control is safe to press mid-flight — last click wins.** This is the part that needed the most
care, because inference runs on the ROS executor thread while the simulator thread is blocked waiting
for a chunk. Each command bumps an epoch and aborts the in-flight request, so an action chunk or a stats
message that lands after you pressed reset is dropped instead of repainting a panel you just cleared. A
cancelled request also wakes the waiter immediately: interrupting an A/B probe mid-sequence is honoured
in ~1.1 s (the current chunk) instead of the ~3.3 s the full three would take. Superseded lanes write
nothing on their way out, and only the most recent queued request survives — a probe or gesture clicked
just before a reset does not replay onto the fresh stage afterwards. Verified for reset during: a VLA
chunk executing, a scripted gesture mid-loop, and an A/B probe waiting on the NPU.

**What the model sees vs. what you see.** The policy always receives exactly its trained views,
`agentview` + `robot0_eye_in_hand` at 256px. The large presenter view is a separate `frontview` render
the model never sees; the trained camera crops the arm out of frame when it lifts.

Action execution is paced to LIBERO's 10 Hz control rate. `--no-realtime` runs as fast as possible.
That paces execution, not planning: the sim waits for each NPU chunk, then plays it at 10 Hz.

#### How flexible is it, exactly?

Measured, not asserted — `bench/prompt_probe.py`, raw output in `bench/prompt-probe/`:

| Prompt pair, same frozen observation | Difference | vs. noise floor |
|---|---|---|
| same prompt twice (repeat-noise floor) | 0.12–0.13 | 1.0× by definition |
| "pick up the black bowl" vs "pick up the **plate**" | **0.52–0.53** | **3.7–4.3× — the wording changed the plan** |
| "wave hi" vs "do nothing at all" | 0.13–0.17 | 1.0–1.4× — under the bar, i.e. no effect |

Differences are mean per-DoF |Δaction| in units of the dataset's action std; a pair counts as separated
only above 2× the floor (`CLEAR_FACTOR` in `demo/language_probe.py`). Ranges are across repeat runs of
the same command. Live in the lab the same test reads 3.4× (sandbox) and 7.8× (`goal #3`) for an
object-noun swap, and 0.9–1.8× for the non-manipulation pair — under the bar in every world tried.

The reason: the deployed bundle is `lerobot/pi05_libero`, fine-tuned on LIBERO, whose instruction
vocabulary is entirely pick/place/open/close/turn-on. It discriminates **object references** sharply and
collapses non-manipulation prompts onto the same generic reach at the salient object. That boundary is
the most interesting thing to show an audience, which is why the A/B probe is a first-class button and
why "wave hi" lives in the scripted lane instead of being passed off as policy output.

The probe resets to the scene start pose by default. The result genuinely depends on the pose: measured
mid-grasp, with the gripper already closing on the bowl, the same pair drops to 0.9× the floor — the
visual context has already decided what happens next. Untick the box in the UI to probe in place.

```bash
# offline version of the same measurement, with the VLA node already running
${LIBERO_VENV_PYTHON:-python3} bench/prompt_probe.py --compare \
  --prompts "pick up the black bowl" "pick up the plate" "wave hi" "do nothing at all"
```

**Running this in front of people?** Step-by-step pre-flight, launch markers, a timed seven-minute run
order, recovery, and a browser-free fallback: **[`docs/DEMO_RUNBOOK.md`](docs/DEMO_RUNBOOK.md)**.
Narrative and framing for the IQ-9075 robotics pitch: [`docs/IQ_9075_ROBOTICS_TALK.md`](docs/IQ_9075_ROBOTICS_TALK.md).

## Three findings that shaped the design

1. **The CDSP cannot map all of Pi0.5 at once.** The four context binaries hold 2875 MiB of weights;
   one Hexagon tops out near 1600 MiB and fails with `err 1002`. Worse, upstream reports success
   anyway and hands back a broken graph handle.
2. **QNN exposes both NPUs, and they are not equally fast.** Device 0 is ~25–30% quicker on every
   component. Splitting the four contexts across both by *cost* (not size) removes context paging
   entirely: 4421 ms → **1111 ms** per chunk.
3. **Pi0.5 has no state tensor.** Proprioception is discretized into 256 bins and spliced into the
   *language prompt*, so tokens must be rebuilt every control step — which rules out a precomputed
   phrasebook and puts a SentencePiece tokenizer in the node.

## Layout

```
ros2_ws/src/qrb_ros_vla/       # Pi05Runner (NPU orchestration), tokenizer, ROS 2 node, bench tool
ros2_ws/src/qrb_ros_vla_msgs/  # ActionChunk, InferenceStats
ros2_ws/src/qrb_inference_manager/  # vendored upstream 2.2.0 + dual-NPU device selection
bench/                         # verification harness, device probe, prompt_probe.py, raw measurements
demo/                          # libero_web_demo.py (flexibility lab) + web/, scenes/, scripted_gestures.py,
                               #   scene_catalog.py, language_probe.py, libero_replay.py, synthetic_publisher.py
scripts/                       # install / fetch / codegen
docs/DESIGN.md                 # every number, with method and limitations
```
