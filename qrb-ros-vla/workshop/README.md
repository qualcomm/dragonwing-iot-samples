# Workshop — Get a Vision-Language-Action model running on your IQ-9075

**Total runtime: 2 hours** for the required path (Modules 0–5), **2 h 30 m** if the room reaches all
four `[STRETCH]` modules. For 10–50 attendees, each on their own Qualcomm Dragonwing IQ-9075 EVK.
Needs **1 instructor + 1 helper per 12 attendees**.

By the end of the first 25 minutes you will have a 3-billion-parameter robot foundation model running
on your own board, generating motion from a sentence you type. The rest of the time is spent seeing
how little work QRB ROS asks of you to get there — and how much of it is just ROS 2 that you already
know.

> **Pre-work is mandatory and blocks follow-along.** See [PREWORK.md](PREWORK.md), deadline 7 days
> out. It downloads 2.9 GB of model weights; doing that live for 30 people takes over five hours.

## What you are running

**Pi0.5** is a vision-language-action model: it takes camera images plus a plain-English instruction
and outputs robot joint motion. It runs entirely on your board's Hexagon NPUs — no cloud, no GPU.

You will not be writing a neural network. You will be wiring one up, and the point of the workshop is
that this turns out to be ordinary ROS 2 work.

## Modules

| № | Title | Min | Cumulative | Required? |
| --: | --- | --: | --: | --- |
| 0 | Preflight | 10 | 0:10 | required |
| 1 | **Get it running** | 15 | 0:25 | required |
| 2 | It's just ROS 2 | 20 | 0:45 | required |
| 3 | Drive it with real robot data | 20 | 1:05 | required |
| — | *break* | 10 | 1:15 | — |
| 4 | Make it 3× faster with one parameter | 15 | 1:30 | required |
| 5 | The whole API is three calls | 15 | 1:45 | required |
| 6 | Tune it for your robot | 15 | 2:00 | `[STRETCH]` |
| 7 | Prove it is numerically correct | 15 | 2:15 | `[STRETCH]` |
| 8 | Let it drive a robot and see if it succeeds | 15 | 2:30 | `[STRETCH]` |

**Every module ends at a checkpoint (`CP-N`) you can verify, and has a one-command escape hatch:**
`./workshop/scripts/fastforward.sh N`. Nobody sits watching someone else debug.

Set up your terminal once:

```
DEVICE
cd ~/QRB-ROS-VLA
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
```

---

## Module 0 — Preflight

**Time budget:** 10 min
**Goal:** confirm every board in the room is ready before anything is taught.
**You will end with:** an all-green preflight.
**Prerequisite state:** pre-work complete.
**Escape hatch:** `./workshop/scripts/fastforward.sh 0`

```
DEVICE
./workshop/scripts/preflight.sh
```

**Expected output:** 22 `[PASS]` lines, ending:

```
PREFLIGHT v1 9075 OK 22/22 2f369c domain=64
```

Any `[FAIL]` tells you the exact command to fix it. Raise a hand — **a helper comes to you.**

**Checkpoint CP-0:** `./workshop/scripts/check.sh 0` → `CP-0 PASS: board passes preflight`

---

## Module 1 — Get it running

**Time budget:** 15 min
**Goal:** a VLA generating robot motion on your board, as fast as possible.
**You will end with:** action chunks streaming on a ROS topic.
**Prerequisite state:** CP-0.
**Escape hatch:** `./workshop/scripts/fastforward.sh 1`

One command. **Terminal A:**

```
DEVICE
ros2 launch qrb_ros_vla vla.launch.py
```

It takes about 20 seconds to map 2.9 GB of model weights onto the NPUs. **Expected output:**

```
[vla_node-1] [INFO] ... tokenizer: .../paligemma_tokenizer.model (live prompt build, state folded in)
[vla_node-1] [INFO] ... task set: 'pick up the black bowl and place it on the plate' (13 real tokens...)
[vla_node-1] [INFO] ... loading ... - inference starts once ready
```

It is now waiting for camera images. Give it some. **Terminal B:**

```
DEVICE
cd ~/QRB-ROS-VLA
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
python3 demo/synthetic_publisher.py --rate 1.0
```

**Expected output** in Terminal B, once per second:

```
chunk #1 task='pick up the black bowl and place it on the plate' 50x7dof first=[-0.033 -0.045  1.102 ...] range=[-1.830, 2.118]
  libQnnHtp.so: vision=194.8 token_emb=23.5 backbone=506.0 expert=375.4 pack=6.0 total=1105.9 ms (0.90 chunk/s)
```

**That is a 3B-parameter vision-language-action model running on your board.** Each line is 50
future robot timesteps across 7 degrees of freedom, produced in ~1.1 seconds.

`libQnnHtp.so` in that output is the proof it ran on the NPU — that is the Hexagon backend. There is
no CPU fallback here.

**Leave both terminals running.** You will use them for the next two modules.

**Checkpoint CP-1:** in a **third** terminal:

```
DEVICE
cd ~/QRB-ROS-VLA && ./workshop/scripts/check.sh 1
```

```
CP-1 PASS: node is publishing action chunks
```

---

## Module 2 — It's just ROS 2

**Time budget:** 20 min
**Goal:** see that the VLA is an ordinary ROS 2 node you can inspect and drive with tools you know.
**You will end with:** the model responding to an instruction you typed.
**Prerequisite state:** CP-1, both terminals running.
**Escape hatch:** `./workshop/scripts/fastforward.sh 2`

Nothing about this node is special. Use your normal tools — **Terminal C:**

```
DEVICE
ros2 topic list | grep qrb_ros_vla
ros2 node info /qrb_ros_vla
ros2 topic hz /qrb_ros_vla/action_chunk
```

Look at the message types. They are plain ROS 2 interfaces:

```
DEVICE
ros2 interface show qrb_ros_vla_msgs/msg/ActionChunk
```

Now watch the latency breakdown the node publishes for every single chunk:

```
DEVICE
ros2 topic echo /qrb_ros_vla/stats --once
```

**Every performance number in this workshop comes from that topic.** Nothing is estimated.

### Now tell the robot to do something else

The instruction is just a `std_msgs/String`:

```
DEVICE
ros2 topic pub --once /qrb_ros_vla/task std_msgs/String "{data: 'put the red block in the drawer'}"
```

**Expected output** in Terminal A:

```
task set: 'put the red block in the drawer' (14 real tokens, 8 state dims)
```

and in Terminal B the `task=` field on each chunk changes, with different action values.

<details>
<summary>Where did the robot's joint positions go? There is no state topic in the model.</summary>

There is a `~/state` topic, but Pi0.5 has **no state input tensor**. It discretizes joint values into
256 buckets and splices them into the *language prompt*:

```
Task: put the red block in the drawer, State: 140 102 166 128 192 12 160 128;
Action:
```

Which is why the token count changed when state arrived. It also means the prompt is rebuilt every
control step, so the tokenizer has to live in the node — it is not something you can precompute.

</details>

**Checkpoint CP-2:**

```
DEVICE
./workshop/scripts/check.sh 2
```

```
CP-2 PASS: node accepted a new task and is still publishing chunks
```

---

## Module 3 — Drive it with real robot data

**Time budget:** 20 min
**Goal:** find out whether the actions are any good, using a real human demonstration.
**You will end with:** a measured accuracy score.
**Prerequisite state:** CP-2, node running in Terminal A.
**Escape hatch:** `./workshop/scripts/fastforward.sh 3`

Synthetic images prove the pipeline runs. They do not tell you the model is *working*. So replay a
real robot demonstration from the LIBERO dataset — the data Pi0.5 was trained against — and compare
what the model predicts against what the human actually did.

Stop the synthetic publisher in **Terminal B** (`Ctrl-C`), then:

```
DEVICE
python3 demo/libero_replay.py --steps 8 --stride 20 --horizon 10
```

**Expected output:**

```
frame    0 | MAE 0.0188 | normalized MAE 0.105 | grip pred -0.99 truth -1.00
...
    dim        MAE    MAE/std   action std
     dx     0.0305      0.091        0.336
   grip     0.0125      0.013        0.999
    ALL     0.0248      0.123
```

Read the **`MAE/std`** column — the error as a fraction of how much actions naturally vary in this
dataset. **0.12 means the error is about 12% of the natural spread.** Predicting the dataset average
would score 1.0, so the model is genuinely tracking the demonstrator.

Look at `grip`: 0.013. The gripper is effectively binary (open/closed, ±1), and the model gets it
right every step.

> **Be precise about what this is.** The model is fed the human's observations and scored on one
> episode. That is *action accuracy*, not a task success rate — the model never sees the consequences
> of its own actions. It is the honest version of "does this work?". Module 8 closes that gap by
> putting a simulator in the loop, if you get that far.

**Checkpoint CP-3:**

```
DEVICE
./workshop/scripts/check.sh 3
```

```
CP-3 PASS: normalized MAE 0.123 against the human demonstration
```

---

## *Break — 10 minutes*

---

## Module 4 — Make it 3× faster with one parameter

**Time budget:** 15 min
**Goal:** use both of the board's NPUs instead of one.
**You will end with:** the same pipeline, roughly 3× faster.
**Prerequisite state:** CP-3.
**Escape hatch:** `./workshop/scripts/fastforward.sh 4`

Your IQ-9075 has **two** Hexagon NPUs. Confirm the runtime can see both:

```
DEVICE
g++ -std=c++17 -O2 -I/usr/include/QNN bench/qnn_device_probe.cpp -ldl -o /tmp/qnn_device_probe
/tmp/qnn_device_probe
```

**Expected output:**

```
hardware devices   : 2
  device[0] id=0 type=ON_CHIP numCores=1
  device[1] id=1 type=ON_CHIP numCores=1
```

Pi0.5 is four model components totalling 2875 MiB of weights, and that does not fit in one NPU's
mapping budget — so on a single NPU the pipeline has to keep swapping weights in and out. Measure
that:

```
DEVICE
QNN_HTP_BURST=1 ros2_ws/install/qrb_ros_vla/lib/qrb_ros_vla/pi05_bench \
  --bundle artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075 \
  --iters 3 --warmup 1 --resident "vision_encoder" --devices "0,0,0,0" \
  --json /tmp/cp4-single.json
```

**Expected output** — note `ctx create/free`, which is pure weight-swapping overhead:

```
  ctx create/free      2258.48   ...
  TOTAL / chunk        3277.16   ...
```

Now spread the four components across both NPUs so all of them stay loaded:

```
DEVICE
QNN_HTP_BURST=1 ros2_ws/install/qrb_ros_vla/lib/qrb_ros_vla/pi05_bench \
  --bundle artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075 \
  --iters 5 --warmup 2 --devices "1,1,0,0" \
  --json /tmp/cp4-latency.json
```

**Expected output:**

```
  ctx create/free         0.00      0.00 ...
  TOTAL / chunk        1111.xx   ...
  chunks/s: 0.90
```

**Weight-swapping is now exactly zero, and the pipeline is ~3× faster.** In the node this is just a
launch argument — no rebuild, no code:

```
DEVICE
ros2 launch qrb_ros_vla vla.launch.py htp_device_ids:="[1,1,0,0]"
```

<details>
<summary>Why <code>1,1,0,0</code> and not <code>0,0,1,1</code>? Try both.</summary>

The two NPUs are not equally fast — device 0 runs every component 25–30% quicker. So you put the
*expensive* components (`backbone`, `action_expert`) on device 0, not the *big* ones. Cost, not size.

| Split (vision, token_emb, backbone, expert) | ms/chunk |
| --- | --: |
| `0,0,1,1` — balanced by size | 1323 |
| **`1,1,0,0` — balanced by cost** | **1111** |

The defaults in the launch file already do the right thing; this is just why.

</details>

At 10 Hz robot control, a 50-step chunk is **5 seconds of motion generated in 1.1 seconds**.

**Checkpoint CP-4:**

```
DEVICE
./workshop/scripts/check.sh 4
```

```
CP-4 PASS: dual-NPU action chunk in 1113 ms with zero context paging
```

---

## Module 5 — The whole API is three calls

**Time budget:** 15 min
**Goal:** see exactly how little code QRB ROS needs to put a model on the NPU, so you can do it with your own.
**You will end with:** your own compiled program running a model on the Hexagon.
**Prerequisite state:** CP-4.
**Escape hatch:** `./workshop/scripts/fastforward.sh 5`

Everything you have run today sits on `qrb_inference_manager`, and its entire API is three calls:

```cpp
QrbInferenceManager mgr(model_path, "libQnnHtp.so");   // 1. load onto the NPU
mgr.inference_execute(input_bytes);                    // 2. run it
auto outputs = mgr.get_output_tensors();               // 3. read results
```

No session setup, no delegate registration, no graph building, no device management. Read the whole
example — it is about 40 lines including comments:

```
DEVICE
less workshop/examples/minimal_npu.cpp
```

Build it yourself. One `g++` line, no CMake:

```
DEVICE
g++ -std=c++17 -O2 \
  -I$PWD/ros2_ws/install/qrb_inference_manager/include \
  workshop/examples/minimal_npu.cpp \
  -L$PWD/ros2_ws/install/qrb_inference_manager/lib -lqrb_inference_manager \
  -Wl,-rpath,$PWD/ros2_ws/install/qrb_inference_manager/lib \
  -o /tmp/minimal_npu
```

Run it:

```
DEVICE
QNN_HTP_BURST=1 /tmp/minimal_npu \
  artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075/vision_encoder.bin
```

**Expected output:**

```
got 1 output tensor(s):
  img_embed    bytes=2097152   shape=[1, 256, 2048]
               first 4 values: 6.9219 -0.7668 -0.6043 -0.7043

That was a 3B-parameter model's vision encoder, on the NPU, in 3 API calls.
```

### Doing this with your own model

Change one string to move that model to the CPU — `"libQnnHtp.so"` → `"libQnnCpu.so"`. That is the
whole difference between NPU and CPU execution.

To use your own model, export it from [Qualcomm AI Hub](https://aihub.qualcomm.com) as a
`.tflite`, `.so`, or `.bin`, and point the same three calls at it. If your model has one input and
one output, `qrb_ros_nn_inference` is a ready-made ROS 2 node that does even this much for you:

```
DEVICE
ros2 run qrb_ros_nn_inference qrb_ros_nn_inference --ros-args \
  -p model_path:=/path/to/your_model.bin -p backend_option:=htp
```

Pi0.5 needed a custom node only because it is four chained models with 41 inputs on one stage. A
normal single-model pipeline does not.

**Checkpoint CP-5:**

```
DEVICE
./workshop/scripts/check.sh 5
```

```
CP-5 PASS: minimal_npu built and ran a model on the NPU
```

---

## Module 6 `[STRETCH]` — Tune it for your robot

**Time budget:** 15 min
**Goal:** trade accuracy for speed, and see how the knobs behave.
**You will end with:** a measured speed/accuracy curve.
**Prerequisite state:** CP-4.
**Escape hatch:** `./workshop/scripts/fastforward.sh 6`

Pi0.5 refines its action chunk over 10 denoising steps. That is a tunable — fewer steps is faster:

```
DEVICE
for S in 4 6 10; do
  echo "--- denoise_steps=$S ---"
  QNN_HTP_BURST=1 ros2_ws/install/qrb_ros_vla/lib/qrb_ros_vla/pi05_bench \
    --bundle artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075 \
    --iters 3 --warmup 1 --steps $S | grep -E "action_expert|TOTAL|chunks/s"
done
```

The action expert is ~37 ms per step, so each step you remove is ~37 ms off the chunk.

**Whether accuracy survives is an empirical question, not a guess.** Restart the node with fewer
steps and re-run Module 3's scoring to find out:

```
DEVICE
ros2 launch qrb_ros_vla vla.launch.py denoise_steps:=5
```

then in Terminal B: `python3 demo/libero_replay.py --steps 8 --stride 20 --horizon 10`

Compare the `ALL / MAE/std` figure against the 0.123 you measured with 10 steps. Report what you
find — this is a real experiment and the answer is not written down anywhere.

**Checkpoint CP-6:** `./workshop/scripts/check.sh 6`

---

## Module 7 `[STRETCH]` — Prove it is numerically correct

**Time budget:** 15 min
**Goal:** confirm the pipeline is not just fast but right.
**You will end with:** bitwise agreement with Qualcomm's reference runner.
**Prerequisite state:** CP-4.
**Escape hatch:** `./workshop/scripts/fastforward.sh 7`

Fast and wrong is easy to build. `qnn-net-run` ships with QAIRT and is the reference implementation,
so replay every stage's inputs through it and diff the outputs (~3 min):

```
DEVICE
rm -rf /tmp/cp7dump
QNN_HTP_BURST=1 ros2_ws/install/qrb_ros_vla/lib/qrb_ros_vla/pi05_bench \
  --bundle artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075 \
  --iters 1 --warmup 0 --dump-dir /tmp/cp7dump
python3 bench/verify_against_qnn_net_run.py --dump-dir /tmp/cp7dump | tee /tmp/cp7-verify.log
```

**Expected output:**

```
PASS vision_encoder: all 1 output tensors bitwise identical to qnn-net-run
PASS token_emb: all 7 output tensors bitwise identical to qnn-net-run
PASS backbone: all 36 output tensors bitwise identical to qnn-net-run
PASS action_expert: all 1 output tensors bitwise identical to qnn-net-run
VERIFIED: ...
```

45 of 45 tensors, still passing with components split across two NPUs.

**Checkpoint CP-7:** `./workshop/scripts/check.sh 7`

---

## Module 8 `[STRETCH]` — Let it drive a robot and see if it succeeds

**Time budget:** 15 min
**Goal:** stop scoring predictions and find out whether the policy completes a task.
**You will end with:** a simulated Franka arm finishing a LIBERO task, driven by your board's NPUs.
**Prerequisite state:** CP-1 (the node running).
**Escape hatch:** `./workshop/scripts/fastforward.sh 8`

Module 3 compared the model's predictions against a human's. That is a real measurement, but it has
a hole in it: the human's actions decided what happened next, so the policy never faced the
consequences of its own. A wrong policy and a slightly-wrong-but-plausible policy score about the
same.

Closing the loop removes the human. The simulator renders what the arm can see, the policy decides,
the simulator executes that decision, and the task's own goal predicate says whether it worked.

<details>
<summary>Why this runs on the board at all, given Module 3 said Gazebo could not render</summary>

Gazebo's Ogre2 renderer demands OpenGL 3.3 **core**, and this board's Adreno driver exposes only
OpenGL ES. LIBERO uses MuJoCo, which is satisfied by the OpenGL 4.5 **compatibility** profile that
Mesa's software rasterizer advertises here. Same board, same driver, opposite outcome — the
difference is one word in a requirement.

Rendering is therefore on the CPU: 340 ms for both camera views. That is why the loop renders only
when the policy replans, which is exactly what a 50-step action chunk buys you.
</details>

```
DEVICE
source ~/libero/libero-env.sh
python3 demo/libero_closed_loop.py \
  --suite libero_10 --task-index 0 --episodes 2 \
  --replan-horizon 10 --max-steps 520 | tee /tmp/cp8-closedloop.log
```

**Expected output:** each episode ends `SUCCESS`, in roughly 250–300 steps and about 50 s.

```
suite libero_10 task 0: 'put both the alphabet soup and the tomato sauce in the basket'
[1/2] init 0: SUCCESS steps=285 chunks=29   51.1s  lat=1130.94
[2/2] init 1: SUCCESS steps=283 chunks=29   50.6s  lat=1124.59

success 2/2 = 100.0%  (95% CI 34.2%-100.0%, Wilson, on 2 episodes)
backends seen: ['libQnnHtp.so']  stale chunks: 0
```

Three things in that output matter more than the success count:

* **`backends seen: ['libQnnHtp.so']`** — proof the NPU ran, attached to every single control step
  rather than asserted once at startup. A CPU fallback shows a different backend and is far slower.
* **`stale chunks: 0`** — every action chunk was matched to the observation it was computed from, by
  the timestamp the node echoes back. A non-zero count means the loop lost synchronization and the
  arm was acting on stale plans.
* **The confidence interval** — two episodes cannot distinguish a good policy from a lucky one. The
  interval is the honest width, and it is wide on purpose.

> **A broken observation path does not error.** Send the camera images the wrong way up, or the state
> vector in the wrong order, and the loop still runs, still renders, and still reports a rate — of
> zero. If your episodes all fail, suspect the plumbing before the policy.

**Checkpoint CP-8:** `./workshop/scripts/check.sh 8`

---

## What you did

In two hours, on a single embedded board:

- Ran a **3-billion-parameter vision-language-action model** entirely on-device
- Drove it from a plain `std_msgs/String` instruction and standard ROS 2 tooling
- Measured it against a real human demonstration: **~12% of the natural action spread**
- Made it **3× faster** with one launch argument
- Put a model on the NPU yourself in **three API calls**

The thing worth taking away: apart from `htp_device_ids`, nothing here was Qualcomm-specific from a
ROS developer's point of view. Standard messages, standard topics, standard launch files, standard
`ros2` CLI. The NPU is a backend string.

## Where to go next

| | |
| --- | --- |
| Your own model | Export from [AI Hub](https://aihub.qualcomm.com), point `qrb_ros_nn_inference` at it |
| Faster still | ~223 ms/chunk is CPU-side tensor copying; [`qrb_ros_transport`](https://github.com/qualcomm-qrb-ros/qrb_ros_transport) gives zero-copy DMA-buf |
| Full detail | [`docs/DESIGN.md`](../docs/DESIGN.md) — every measurement, method, and limitation |
| A real robot | Pi0.5's actions target a 7-DoF Franka; a different arm needs its own trained policy |

## Something broken?

[TROUBLESHOOTING.md](TROUBLESHOOTING.md) — symptoms are verbatim and greppable.
