# IQ-9075 for robotics: VLA now, hybrid RL + VLA next

> Talk thesis: **You can run the VLA for most things, but I am looking for you all for a hybrid robotics RL policy + VLA.**

In this repo, **IQ-9075** means the Qualcomm Dragonwing IQ-9075 EVK / QCS9075 board used by the QRB ROS VLA project.

**Operating the demo is a separate document.** Pre-flight, launch markers, the timed run order,
recovery, and a browser-free fallback live in [`DEMO_RUNBOOK.md`](DEMO_RUNBOOK.md). This page is the
narrative; that one is what you hold while presenting.

## One-minute explanation

Robotics is not just one neural network. A useful robot has to:

1. read sensors,
2. understand a human goal,
3. choose an action,
4. react when physics disagrees,
5. keep doing that locally, safely, and fast enough.

IQ-9075 is a good fit because it lets those pieces run on the robot-side computer instead of depending on a cloud loop.

## What we can already show live

The current project runs **Pi0.5**, a 3B-parameter vision-language-action model, inside a ROS 2 node on IQ-9075.

Measured results from this repository:

The closed-loop success number is documented in `docs/DESIGN.md` §10 and audited by
`bench/libero-closed-loop-pooled.json`, which pools 42/50 from
`bench/libero-closed-loop-sweep.jsonl` with 44/50 from
`bench/libero-closed-loop-inits20.jsonl`.

| Result | Why students should care |
|---|---|
| 50-step, 7-DoF action chunks in about **1.11–1.15 s** | At 10 Hz, that chunk represents 5 seconds of robot motion. The model plans faster than the horizon it emits. |
| Backend is **`libQnnHtp.so`** | The model is running on the Hexagon NPU, not silently falling back to CPU. |
| **45/45 tensors bitwise identical** to Qualcomm `qnn-net-run` | The ROS runner matches the reference output exactly. This is a correctness claim, not a vibe. |
| LIBERO replay normalized MAE about **0.149** | Predicted actions track human demonstration data at a measurable scale. |
| Closed-loop LIBERO result: **86/100 successes** | The policy can act, observe consequences, and continue, not just score a fixed recording. |

## The live demo story

The talk demo is intentionally ROS-backed:

```text
browser command
  -> ROS /qrb_ros_vla/task
LIBERO simulated cameras
  -> ROS /vla/camera0/image_raw and /vla/camera1/image_raw
LIBERO robot state
  -> ROS /qrb_ros_vla/state
VLA action chunk + timing
  <- ROS /qrb_ros_vla/action_chunk and /qrb_ros_vla/stats
LIBERO simulator steps with those actions
```

No physical cameras are required for the talk. The separate LIBERO project in `/home/ubuntu/libero` provides the simulation environment, assets, initial states, and MuJoCo/robosuite physics. ROS is still the transport.

The lab starts idle: the simulator loads and renders, but nothing is published to ROS until you type a prompt, run the A/B probe, or fire a scripted gesture. Free play has no goal — the benchmark success predicate is never evaluated, so the scene only resets when you reset it.

Three lanes, and the UI always says which one produced the motion:

1. **VLA.** Typed text goes verbatim to `/qrb_ros_vla/task`; Pi0.5 replans on the NPUs every ~1.1 s against live frames. 41 worlds are switchable live from the picker and the weights never change — that is the flexibility claim, and it is the one to make out loud.
2. **A/B language probe.** Freezes one observation and asks for three chunks: prompt A, prompt B, then A again as a repeat-noise floor. This is the answer to "is it really reading my words, or just reaching at the nearest object?" — measured on stage.
3. **Scripted.** Hand-written trajectories for wave/nod/bow. Say plainly that these are choreography, not policy output, and why: see below.

**The honest limit, and how to present it as a strength.** The deployed bundle is `lerobot/pi05_libero`, fine-tuned on LIBERO's pick/place/open/close/turn-on vocabulary. Measured with `bench/prompt_probe.py`, from the same frozen observation, in units of the dataset's action std: swapping the object noun ("black bowl" → "plate") moves the plan **0.52**, versus a **0.13** repeat-noise floor — 3.7×, so the language is doing real work. But "wave hi" versus "do nothing at all" differ by **0.13** — exactly the noise floor. Non-manipulation prompts collapse onto the same generic reach.

So do not let the room believe the arm waves because it understood a greeting. Show the A/B result, then show the boundary, then fire the scripted gesture and name it as scripted. A demo that states its own failure mode survives the skeptical question from the front row; one that does not, does not.



Fast path: start both panes in tmux with one command:

```bash
scripts/start-libero-web-demo.sh
```

Manual path:

Run order:

Terminal A keeps the VLA node alive:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source /home/ubuntu/libero/libero-env.sh
ros2 launch qrb_ros_vla vla.launch.py
```

Terminal B serves the browser UI and runs the LIBERO loop:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
source /home/ubuntu/libero/libero-env.sh
${LIBERO_VENV_PYTHON:-python3} demo/libero_web_demo.py --host 0.0.0.0 --port 8080
```

The simulator steps returned action chunks at 10 Hz by default, matching the LIBERO control rate, so the visible rollout is paced like robot time instead of sprinting through the chunk.
This paces the visible action execution, not the VLA planning wait: the sim waits for each NPU action chunk, then plays that chunk at 10 Hz.


Open:

```text
http://<board-ip>:8080
```

What to point at on screen:

- typed text becomes a ROS task message,
- the two small frames are exactly what the policy sees; the big one is a presenter camera it never sees,
- LIBERO supplies simulated camera frames and robot state,
- the VLA returns a chunk of future motion,
- `libQnnHtp.so` proves the NPU path is live,
- the simulator steps with the returned actions,
- the lane badge names what produced the motion.

Suggested seven-minute running order:

| Time | Do | Say |
|---|---|---|
| 0:00 | Load `sandbox / sandbox table`, type "put the black bowl on the plate" | one 3B VLA, both Hexagons, 1.1 s per 50-step chunk, nothing in the cloud |
| 1:30 | Switch to `goal #3`, run its green chip | same weights, different world — I changed the simulator, not the model |
| 3:00 | Type an amber chip's wording, e.g. "pick up the wine bottle" | a sentence it was never demonstrated on, in a world it knows |
| 4:00 | Run the A/B probe on two object nouns | here is the proof the words matter: measured live at 3.4–7.8× the noise floor depending on the world |
| 5:00 | Probe "wave hi" vs "do nothing at all" | and here is the edge — 0.9–1.8×, under the 2× bar, because this checkpoint is LIBERO-tuned |
| 6:00 | Fire the scripted wave | so this one is choreography, and I am telling you that rather than letting you assume |

Both A/B numbers above are live readings from this lab (sandbox and `goal #3`); the 2× bar is
`CLEAR_FACTOR` in `demo/language_probe.py`. Object-noun swaps clear it comfortably, non-manipulation
prompts do not. Run the probe twice in the room if someone doubts the floor is real.

**If it goes wrong on stage.** `Stop` freezes the current lane and leaves the scene alone. `Reset scene`
re-poses the arm and objects in the world you are in. `Reset everything` rebuilds the startup world from
scratch and blanks every panel — use it between audience volunteers, or any time the arm has swept
something onto the floor and you want a clean slate without touching the terminal. None of the three
restarts the model, so recovery costs seconds, not the ~70 s of a full relaunch.

## Why QRB ROS matters on IQ-9075

This demo is not just "a model happened to run on a board." The useful story is that IQ-9075 gives the
robotics stack a local data plane: sensors, ROS 2, inference, and control all stay on the robot-side
computer.

### 1. QRB ROS keeps accelerator access inside ROS

`qrb_inference_manager` is the bridge from a ROS node to QNN / Hexagon. That matters because the VLA is
not a Python notebook calling a cloud endpoint; it is a ROS 2 component that publishes and subscribes to
normal topics while its heavy graphs run on `libQnnHtp.so`. In this repo we also had to use the upstream
2.x path rather than the apt 1.1.1 library, because Pi0.5 needs int32 language-token inputs, multiple
input tensors, HTP burst / DCVS setup, and the newer DMA-buf API surface.

The practical message for the room: QRB ROS lets a robotics team keep the software shape they already
understand — ROS topics, launch files, messages, stats — while still reaching the Qualcomm accelerator
path directly.

### 2. Zero-copy is the next latency knob, not a buzzword

Today's measured chunk latency is already useful: about 1.1 s for a 50-step action chunk. The remaining
obvious waste is CPU-side tensor movement between model stages. `backbone` alone emits 36 KV tensors,
about **34 MB**, that currently get read out and repacked before the action expert consumes them.

That is exactly the kind of copy QRB ROS is meant to avoid. `qrb_ros_transport` passes DMA-buf file
descriptors, and upstream `qrb_inference_manager` exposes `inference_execute_dmabuf()` for `.bin` QNN
models. We do **not** claim the live demo already uses that path; the honest claim is better: the
profile tells us where the next ~hundreds-of-ms class improvement lives, and the QRB ROS stack has the
right mechanism for it. The pre-flight check looks for `/dev/dma_heap/system` because that is the heap
zero-copy transport needs.

### 3. DDS tuning is part of robotics correctness

DDS is not just plumbing. A bad DDS setup can make a robot look smart or broken for the wrong reason.
The rules this project uses are deliberately boring:

- **Isolate graphs.** In a room full of boards, every seat gets a persisted `ROS_DOMAIN_ID`; otherwise
  30 boards on domain 0 merge into one ROS graph and someone can accidentally echo another person's
  action chunk. For workshop runs we also set `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.
- **Use sensor QoS for images.** Camera subscriptions use `rclcpp::SensorDataQoS()`: latest data wins,
  because a VLA acting on stale frames is worse than a VLA running at a lower rate.
- **Do not assume cross-topic ordering.** DDS preserves ordering per topic, not across topics. The
  replay and simulator publish robot state before images, so the node cannot fire on frame *k* while
  the prompt still encodes state *k−1*.
- **Drop, do not queue, during inference.** A 1.1 s model call is too long for an executor callback;
  inference runs on a worker, and frames that arrive mid-inference are discarded instead of building a
  stale backlog.

These are not glamour optimizations. They are the difference between a reproducible robotics system and
a demo whose behavior changes because discovery, queueing, or topic timing changed.

## Why IQ-9075 is a good robotics platform

### 1. Local AI matters

Robots cannot assume perfect Wi-Fi. Local inference avoids cloud latency spikes and keeps camera data on the device. IQ-9075 has the NPU path needed for a large VLA while leaving CPU headroom for ROS, simulation, control, logging, and safety code.

### 2. ROS makes the model inspectable

Students do not need to treat the VLA as magic. They can inspect topics, messages, and latency:

```bash
ros2 topic list
ros2 topic echo /qrb_ros_vla/stats
ros2 interface show qrb_ros_vla_msgs/msg/ActionChunk
```

That is the bridge from AI demo to robotics engineering.

### 3. Chunked actions match robot control structure

The VLA emits many future actions at once. That is useful because a robot can plan at a slower semantic rate while a lower-level controller handles smooth execution between replans.

### 4. The board exposes real systems tradeoffs

Students can measure what changes when they move work between CPU, NPU, simulator, and ROS. That is more valuable than a black-box cloud demo because robotics failures are often systems failures.

## Is the VLA a general-purpose robot brain?

No. A VLA is a strong **semantic manipulation policy**, not a complete robot operating system.

What it is good at:

- reading camera observations plus a natural-language task,
- using visual context to choose manipulation actions,
- producing a short horizon of robot motion,
- replanning after it sees the consequences of its own actions.

What it is not, by itself:

- a world builder that invents missing objects or fixtures,
- a task planner that decomposes arbitrary long-horizon goals into named skills,
- a reflex controller for contact, slips, stalls, or recovery,
- a safety system for real hardware,
- a universal command interpreter for gestures like "say hi" unless that behavior exists in its training distribution or in a separate skill.

That distinction matters for the demo. "Pick up the cream cheese" is a VLA-shaped request if cream cheese is visible in the loaded scene. "Say hi" is better handled by a scripted wave skill. "Open drawer, then put the bowl in it" needs either cumulative scene state plus a policy that can do both steps, or a small agent/router that dispatches each step to the right skill.

The more honest architecture is therefore:

```text
Human command
    |
    v
Command router / agent
    |-- scripted skills: wave, reset, open gripper, close gripper
    |-- scene checks: which objects and fixtures exist?
    `-- VLA policy: visually grounded manipulation actions
```

This is still exciting: the VLA supplies language-conditioned manipulation, and IQ-9075 runs it locally. The missing layer is the agentic robotics wrapper that decides when to call the VLA, when to call a scripted or RL skill, and when to refuse because the scene or hardware cannot support the request.

## The honest limitation

A VLA is powerful, but it should not be the only control policy for every robot. Contact-rich manipulation, recovery, fast feedback, and hardware-specific constraints still need specialized control.

That is why the research direction is **hybrid robotics RL policy + VLA**.

## The hybrid policy pitch

```text
Human command
    |
    v
VLA: language understanding, visual context, semantic plan, action proposal
    |
    v
RL policy / controller: contact handling, fast feedback, recovery, safety, hardware adaptation
    |
    v
Robot actuators
    |
    v
Sensors feed the next replan
```

The VLA is the semantic layer: "what is the task and what should happen next?"

The RL policy is the embodied layer: "how do I make that work on this robot, with this gripper, with this friction, right now?"

## Ask for the room

The VLA is already running. The open opportunity is the hybrid layer.

Useful student projects:

- train an RL residual policy that corrects VLA action chunks,
- build a safety filter around VLA actions,
- compare VLA-only vs RL-only vs hybrid in LIBERO,
- add ROS tools that visualize why a policy chose an action,
- tune the bridge from action chunks to real robot controllers,
- measure latency, thermals, and NPU/CPU utilization during closed-loop control.

Close with this:

> We can run the VLA for most things today. What I want help with is making the robot robust: VLA for language and visual intent, RL for the fast embodied behavior that makes it work outside the demo.
