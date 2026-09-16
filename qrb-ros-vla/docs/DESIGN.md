# Pi0.5 on the Dragonwing IQ-9075: what we built and what we measured

Every number on this page was measured on a real Qualcomm Dragonwing IQ-9075 EVK and is
reproducible from `bench/`. Numbers taken from Qualcomm AI Hub's own published profiling are
labelled as such and never mixed with ours.

- **Board:** Qualcomm Dragonwing IQ-9075 EVK / QCS9075 (`qcom,sa8775p`), 8× Cortex-A78C, dual Hexagon
- **OS:** Ubuntu 24.04.4 LTS Server, aarch64, kernel `6.8.0-1080-qcom`, headless
- **ROS 2:** Jazzy (`ros-jazzy-ros-base`), installed by `scripts/install-ros2-jazzy.sh`
- **Runtime:** QAIRT 2.46.0 (`qairt-libs`/`qairt-tools` from `ppa:ubuntu-qcom-iot/qcom-ppa`)

## Headline result

Pi0.5 — a 3B-parameter vision-language-action model — runs entirely on the IQ-9075's Hexagon NPUs
inside a ROS 2 node, producing a **50-step action chunk in 1.11–1.15 s** with **zero CPU fallback**
and **bitwise-identical outputs** to Qualcomm's own reference runner.

LIBERO runs at 10 Hz, so a 50-step chunk is 5 s of motion — **~4.4× faster than real time**. The
range is not measurement sloppiness: run-to-run spread on this board is ~30 ms and the published
1111 ms figure does not reproduce, so §6a leads with the sustained measurement — **1128.1 ms averaged
over 3153 chunks and 90 minutes of closed-loop operation** — rather than the best bench run. Replaying real human demonstrations through it, the NPU's predicted actions track the
demonstrator to within **14–15% of the dataset's action standard deviation** (§7).

And it does not just predict plausible actions — it completes tasks. Driving LIBERO's simulator
closed-loop on the same board, with the policy's own actions determining what happens next,
**86 of 100 episodes across all ten LIBERO-10 tasks succeed (86%, 95% CI 77.9–91.5%)** (§10). A null
policy scores 0/24 on the same harness, so the predicate is measuring the model rather than the
plumbing.

| Pipeline configuration | ms / chunk | chunks/s | notes |
|---|---:|---:|---|
| One NPU, every context paged in/out | 4421 | 0.23 | clean, no mapping errors |
| One NPU, best safe residency (`vision_encoder`) | 3277 | 0.31 | 2258 ms of it is context paging |
| **Two NPUs, all four contexts resident** | **1111** | **0.90** | **zero context paging** |

`bench/pi05-latency-dual-npu.json`, `bench/pi05-latency-rotate-all.json`.

## 1. Getting the model — no browser, no account

Pi0.5's AI Hub page lists **Dragonwing IQ-9075 EVK as its `default_device`**, and prebuilt QNN
context binaries are published for `qualcomm-qcs9075`. They download **without an AI Hub API token**,
which removes the per-user cloud compile job that normally gates AI Hub models:

```bash
qai-hub-models fetch Pi0.5 --runtime qnn_context_binary --precision mixed --chipset qualcomm-qcs9075
```

2.55 GB zipped → 2.9 GB of context binaries. This matters a lot for a workshop: model acquisition is
a plain HTTPS download that can be mirrored offline.

| Component | On-disk | MiB | Calls per chunk |
|---|---:|---:|---:|
| `token_emb.bin` | 1 055 449 088 | 1006.6 | 1 |
| `backbone.bin` | 979 836 928 | 934.4 | 1 |
| `vision_encoder.bin` | 540 233 728 | 515.2 | 3 (one per camera slot) |
| `action_expert.bin` | 439 205 888 | 418.9 | 10 (flow-matching denoise steps) |
| **total** | | **2875.1** | |

Quantization is mixed: w4a16 backbone, w8a16 vision encoder and action expert.

**QAIRT 2.46.0 executes binaries built with 2.45.0** — the bundle records
`SDK build=v2.45.0.260326154327, SoC model ID=77` and runs unmodified.

## 2. The pipeline

```
3 × vision_encoder → token_emb → backbone (prefill, 18-layer KV cache) → action_expert × 10
```

`num_inference_steps: 10` and `chunk_size: 50` come from `lerobot/pi05_libero`'s public
`config.json`, which also confirms 2 real cameras + 1 `empty_cameras` slot = the 3 the graph expects.
The dataset's `meta/info.json` gives `robot_type: panda`, `fps: 10`, `state[8]`, `actions[7]`.

### Tensor contract (from the context binaries, not the docs)

`src_len = 256 × 3 cameras + 200 tokens = 968`.

| Component | Inputs | Outputs |
|---|---|---|
| `vision_encoder` | `image` [1,3,224,224] f32, RGB, [-1,1] | `img_embed` [1,256,2048] |
| `token_emb` | `lang_tokens` [1,200] **int32**, `img_embed1..3`, `lang_mask` [1,200] f32 | `prefix_emb` [1,968,2048], `prefix_att_2d`, `prefix_sin/cos`, `suffix_sin/cos`, `full_att_4d` |
| `backbone` | `prefix_att_2d_masks`, `hidden_state`, `rope_emb_cos`, `rope_emb_sin` | `k_cache_l0..17`, `v_cache_l0..17`, each [1,968,1,256] |
| `action_expert` | `x_t` [1,50,32], `time_step` [1], `key_cache_l*`, `value_cache_l*`, `rope_emb_cos/sin`, `full_att_4d` | `action_emb` [1,50,32] |

### Three traps in wiring it together

`QrbInferenceManager::inference_execute()` takes **one flat byte buffer** and slices it across graph
inputs in compiled graph order. Get the order wrong and you get plausible garbage with no error.

1. **KV cache order is lexicographic, not numeric.** `backbone` *emits*
   `k_cache_l0, l1, l2 … l17`, but `action_expert` *consumes*
   `key_cache_l0, l1, l10, l11 … l17, l2, l3 … l9`. Feeding them in producer order silently
   scrambles 12 of 18 layers.
2. **Names change between producer and consumer**, and cos/sin swap position:
   `prefix_emb`→`hidden_state`, `prefix_att_2d`→`prefix_att_2d_masks`,
   `suffix_sin/cos`→`rope_emb_sin/cos`.
3. **Robot state is not a tensor anywhere.** Pi0.5 discretizes state into 256 bins and splices it
   into the *language prompt* (§4). Nothing in the graph takes proprioception directly.

Graph order is generated from the binaries themselves into
`ros2_ws/src/qrb_ros_vla/include/qrb_ros_vla/pi05_graph_order.hpp` by `scripts/gen_graph_order.py`,
and `static_assert`ed against the shapes the code assumes.

## 3. Correctness proof

`bench/verify_against_qnn_net_run.py` re-splits every stage's flat input buffer, replays it through
Qualcomm's stock `qnn-net-run`, and diffs the outputs against ours.

```
PASS vision_encoder   ( 1 output tensor )  bitwise identical
PASS token_emb        ( 7 output tensors)  bitwise identical
PASS backbone         (36 output tensors)  bitwise identical
PASS action_expert    ( 1 output tensor )  bitwise identical
VERIFIED: Pi05Runner's graph-order packing reproduces qnn-net-run exactly.
```

45/45 tensors, and it still passes with components pinned to different NPUs.

> **`qnn-net-run` needs `--use_native_input_files`.** By default it parses every input file as
> float32 and casts to the graph dtype. For `token_emb`'s int32 `lang_tokens` that turns real token
> ids into zeros — i.e. all-padding — and the tool then "disagrees" with a runner that is actually
> correct. This cost real debugging time; the only differing rows were exactly the 24 real language
> tokens, and their values equalled the padding embedding, which is what gave it away.

## 4. Tokenization: state lives in the prompt

Reproduced exactly from openpi's `PaligemmaTokenizer` (the `state is not None` branch):

```python
cleaned = prompt.strip().replace("_", " ").replace("\n", " ")
discretized = np.digitize(state, np.linspace(-1, 1, 257)[:-1]) - 1
full = f"Task: {cleaned}, State: {' '.join(map(str, discretized))};\nAction: "
tokens = tokenizer.encode(full, add_bos=True)   # right-pad with 0 to 200, parallel mask
```

Consequences:

- The prompt changes **every control step**, so a precomputed phrasebook cannot be correct for
  closed-loop use. Tokenization runs in-process in C++ (`Pi05Tokenizer`, ~0.1 ms) via
  `libsentencepiece-dev`. The offline phrasebook remains only as a fallback.
- State must already be normalized to ≈[-1,1] with the policy's training statistics before it
  reaches the node. `Pi05Tokenizer::DiscretizeState` reduces `np.digitize` to
  `clamp(floor((x+1)*128), 0, 255)`.

**The tokenizer is not gated.** `google/paligemma-3b-pt-224` returns HTTP 401 without an accepted
licence, but the identical SentencePiece model is served unauthenticated from the `big_vision`
bucket, which is where openpi fetches it too. `scripts/fetch-tokenizer.sh` pins
`sha256:8986bb4f…168fc6` (4 264 023 bytes, vocab 257 152, bos=2, pad=0).

## 5. The two hardware findings that decide the architecture

### 5a. The CDSP has a weight-mapping ceiling well below the model size

Mapping all four contexts (2875 MiB) into one CDSP fails:

```
fastrpc memory map for fd: 39 with length: 434110464 failed with error: 0x1
SharedMemoryMod failed to Map Buffer to SMMU for domain 0
Failed to map weights buffer to device!  →  err 1002
```

This is **DSP address space, not host RAM** — it fails with ~34 GB free, as root, and each binary
loads fine alone. Measured on device 0:

| Peak concurrently-mapped weights | Result |
|---:|---|
| 1523 MiB | clean |
| 1941 MiB | `Failed to map buffer` (recovers, but untrustworthy) |
| 2875 MiB (all four) | hard failure |

It is **not a pure volume limit**: repeated map/unmap fragments the address space, so identical
totals succeed or fail depending on history. Two-component residency sets summing to 2360 MiB failed
even though a 2456 MiB peak had succeeded earlier in a fresh process. `Pi05Runner` therefore
enforces a deliberately conservative **1600 MiB per-device soft ceiling** and reports a clear error
instead of letting QNN fail deep inside graph init.

> **Upstream reports this failure but still returns success.** A context whose weights fail to map
> still logs `Initialize Qnn graph from binary file successfully`, and the resulting handle produces
> a garbage-length output vector. `Pi05Runner::RunComponent` cross-checks output arity against the
> committed graph header to turn that into a real error.

### 5b. QNN exposes both Hexagon NPUs, and they are not equally fast

`bench/qnn_device_probe.cpp` asks the backend directly:

```
hardware devices   : 2
  device[0] id=0 type=ON_CHIP numCores=1
  device[1] id=1 type=ON_CHIP numCores=1
```

Each device has its own mapping budget, so splitting the four contexts across both lets **all of
them stay resident** and removes context paging entirely. Upstream `qrb_inference_manager` hardcodes
`deviceCreate(nullptr, nullptr, …)` (device 0); we added a `device_id` parameter that scopes the
device handle with a single-entry `QnnDevice_PlatformInfo_t`.

Then the surprise — **device 0 is consistently ~25–30% faster than device 1**:

| Component | on device 0 | on device 1 |
|---|---:|---:|
| `backbone` (×1) | 509 ms | 656 ms |
| `action_expert` (×10) | 379 ms | 483 ms |
| `vision_encoder` (×3) | 140 ms | 194 ms |

So the split is chosen by *cost*, not by size: the expensive pair goes on device 0.

| Split (vision, token_emb, backbone, expert) | ms / chunk |
|---|---:|
| `0,0,1,1` — size-balanced | 1323 |
| `1,0,1,0` | 1258 |
| **`1,1,0,0` — cost-balanced (default)** | **1111** |

## 6. Measured latency breakdown (default config)

12 iterations after 3 warmup, `QNN_HTP_BURST=1`, all contexts resident.

| Stage | Calls | mean ms | p95 ms | Device |
|---|---:|---:|---:|---|
| `vision_encoder` | 3 | 193.25 | 194.07 | 1 |
| `token_emb` | 1 | 22.48 | 23.18 | 1 |
| `backbone` | 1 | 509.32 | 510.39 | 0 |
| `action_expert` | 10 | 379.26 | 382.17 | 0 |
| host packing | — | 6.24 | 6.58 | CPU |
| context create/free | — | 0.00 | 0.00 | — |
| **total per chunk** | | **1110.73** | **1113.49** | |

### 6a. The same benchmark under confirmed exclusive access is *slower*

§11 asked for this to be re-measured with nothing else on the NPU, because the run above overlapped
with other work on a shared board. Re-run on an idle board — loadavg 0.04, no other process holding
either `fastrpc` node, 20 iterations after 5 warmup, thermals 47.4 °C rising to 49.6 °C:

| Stage | published (12 iters) | exclusive (20 iters) | Δ |
|---|---:|---:|---:|
| `vision_encoder` | 193.25 | 196.19 | +1.5% |
| `token_emb` | 22.48 | **28.96** | **+28.8%** |
| `backbone` | 509.32 | 523.27 | +2.7% |
| `action_expert` | 379.26 | 396.57 | +4.6% |
| host packing | 6.24 | 6.38 | +2.2% |
| **total per chunk** | **1110.73** | **1151.54** | **+3.7%** |

**This is the opposite of the expected result and we are not going to explain it away.** Removing
contention should have tightened the number; instead every stage got slower.

The first hypothesis was address-space fragmentation: §5a records that *repeated map/unmap fragments
the DSP address space*, observed there as mapping **failures**, and the same effect degrading the
performance of successfully-mapped contexts would look like this. The board had 25 hours uptime and
several thousand map/unmap cycles behind it.

**That hypothesis does not survive its own test.** Fragmentation from repeated mapping would
accumulate, so latency should climb across consecutive runs. Five fresh-process runs, minutes apart on
an otherwise idle board (`bench/pi05-latency-drift.csv`):

| Run | total ms | `token_emb` ms | zone-0 temp |
|---|---:|---:|---:|
| 1 | **1120.5** | 23.90 | 47.4 °C |
| 2 | 1147.9 | 27.02 | 49.2 °C |
| 3 | 1127.6 | 26.60 | 49.9 °C |
| 4 | **1150.8** | 28.56 | 50.3 °C |
| 5 | 1146.5 | 26.82 | 50.6 °C |

Mean **1138.7 ms, stdev 13.6 ms, range 1120.5–1150.8** — a 30 ms spread within minutes, and *not
monotonic*: run 3 was faster than run 2. Correlation with run index is +0.63, but correlation with
die temperature is **+0.71**, and the coolest run was the fastest. Temperature rose monotonically
47.4 → 50.6 °C across the sequence, so run index and temperature are confounded and n=5 cannot
separate them. What can be said is that a progressive fragmentation effect is not in evidence, and
thermal state is the simpler explanation that fits at least as well.

A cooldown test narrows it further without needing a reboot. Uptime only ever increases, so if
temperature were the driver, returning the die to its idle baseline should restore the fast figure.
The board sheds heat in about 15 s; re-measured at 46.6 °C — *cooler* than the fastest earlier run,
with an hour more uptime behind it:

| | total ms | `token_emb` ms | temp |
|---|---:|---:|---:|
| drift run 1 | 1120.5 | **23.90** | 47.4 °C |
| drift run 5 | 1146.5 | 26.82 | 50.6 °C |
| **after cooldown** | **1130.0** | **26.77** | **46.6 °C** |

Cooling recovered only part of the gap, and `token_emb` — the stage that regressed most — did not
recover *at all*: 26.77 ms cooled against 26.82 ms warm, versus 23.90 ms earlier the same session at
the same temperature. So temperature does not fully explain it either, and neither hypothesis is
cleanly supported.

Across all eight runs today: **total mean 1139.3 ms, stdev 12.8, range 1120.5–1151.5**, and
`token_emb` swinging **21%** (23.90–28.96 ms) with no clean relationship to temperature or run order.
The honest conclusion is that this board has roughly 30 ms of intrinsic run-to-run variance at the
chunk level that we cannot attribute, and that a 12-iteration benchmark cannot resolve it.

Three things follow, and they matter more than either hypothesis:

1. **Run-to-run spread on this board is ~30 ms**, wider than the 1111–1139 previously recorded. Any
   figure from 12 or 20 iterations carries that much noise.
2. **The published 1110.73 ms does not reproduce.** It sits below *all five* fresh runs. It was not
   wrong when taken, but it is the optimistic tail of the distribution, not the centre.
3. **The most defensible figure is the sustained one.** The closed-loop sweeps (§10) averaged
   **1128.1 ms over 3153 chunks and 90 minutes** — a sample two orders of magnitude larger than any
   bench run, on a thermally settled board, with a 44 ms spread. That is what a user would actually
   experience.

`bench/pi05-latency-dual-npu.json`, `bench/pi05-latency-exclusive.json`,
`bench/pi05-latency-drift.csv`. The reboot-then-measure experiment that would isolate uptime from
temperature has still not been run.

**Proof the NPU ran, not the CPU.** Four independent signals: AI Hub reports 100% NPU layer
placement for all four components (3835/3835, 2473/2473, 1120/1120, 34/34); the loaded backend is
`libQnnHtp.so`, published on every `InferenceStats` message; outputs are bitwise identical to
`qnn-net-run` on the HTP backend; and the whole pipeline is ~1.1 s where a CPU path would be
far slower.

### How this compares with AI Hub's published numbers

AI Hub profiles each component **in isolation**; we run the whole chain with host-side marshalling
between stages, so ours are necessarily higher. Both are per-chunk totals for the same call counts.

| Component | AI Hub published (single call × calls) | Ours, in-pipeline |
|---|---:|---:|
| `vision_encoder` | 40.68 × 3 = 122.0 ms | 193.25 ms (device 1) |
| `token_emb` | 4.24 × 1 = 4.2 ms | 22.48 ms |
| `backbone` | 397.27 × 1 = 397.3 ms | 509.32 ms |
| `action_expert` | 36.43 × 10 = 364.3 ms | 379.26 ms |
| **total** | **887.8 ms** | **1110.73 ms** |

The gap is dominated by moving tensors between stages on the CPU: `backbone` alone emits 36 KV
tensors totalling ~34 MB that must be read out and repacked into the expert's input buffer. That is
the obvious target for `qrb_ros_transport` zero-copy DMA-buf work — upstream `qrb_inference_manager`
2.x already has an `inference_execute_dmabuf()` path we do not yet use.

## 7. Does it actually predict the right actions?

Latency and bitwise fidelity say the model runs correctly. They say nothing about whether its
*outputs are sensible*. `demo/libero_replay.py` closes that gap without needing a robot: it replays a
real LIBERO episode — the same dataset Pi0.5 was calibrated on — feeding the NPU exactly the
observations a human demonstrator saw, then compares the predicted action chunk against what the
demonstrator actually did.

Both state and action use **MEAN_STD** normalization, as declared by `lerobot/pi05_libero`'s
`policy_preprocessor.json` (`{VISUAL: IDENTITY, STATE: MEAN_STD, ACTION: MEAN_STD}`), with mean/std
from the dataset's own `meta/stats.json`.

Episode 0, task *"put the white mug on the left plate and put the yellow and white mug on the right
plate"*, 15 replanning steps at stride 12, scoring a 20-step horizon:

| dim | MAE | MAE / action std | action std |
|---|---:|---:|---:|
| `dx` | 0.0400 | 0.119 | 0.336 |
| `dy` | 0.0594 | 0.157 | 0.378 |
| `dz` | 0.0666 | 0.150 | 0.445 |
| `droll` | 0.0107 | 0.273 | 0.039 |
| `dpitch` | 0.0125 | 0.197 | 0.063 |
| `dyaw` | 0.0088 | 0.113 | 0.078 |
| `grip` | 0.0358 | 0.036 | 0.999 |
| **all** | **0.0334** | **0.149** | |

`MAE / action std` is the scale-free number to read: **0.149 means the error is ~15% of the natural
spread of actions in this dataset**, i.e. far better than predicting the dataset mean (which scores
1.0 by construction). The gripper — the one dimension where being wrong is unambiguous, since it is
effectively binary at ±1 — is matched to **0.036**, and every logged step had the correct sign.
A shorter 10-step horizon scores slightly better (0.132), consistent with prediction quality decaying
mildly across the chunk.

At stride 12 (1.2 s of motion consumed per replan) and 1.12 s per chunk, this configuration runs
**faster than real time** end to end, on the board, with nothing precomputed.

Raw output: `bench/libero-replay-accuracy.txt`. Reproduce with `scripts/fetch-libero-episode.sh`
then `demo/libero_replay.py`.

### 7a. A publish-ordering bug was hiding some of the accuracy

The numbers above were measured with a latent race in the demo. `VlaNode::Ready()`
(`vla_node.cpp:366`) gates inference on two things only — 200 language tokens and one fresh frame per
camera. **Robot state is not part of the trigger.** And `Pi05Tokenizer::BuildPrompt`
(`pi05_tokenizer.cpp:57-61`) falls back to the Pi0-style bare-task prompt when state is empty,
instead of the Pi0.5 `Task: …, State: …;\nAction:` form this export was calibrated on.

`demo/libero_replay.py` published **images before state** on every step. DDS guarantees per-topic
ordering but not cross-topic ordering, so the node could fire on frame *k* while `tokens_` still
encoded state *k−1* — a silently stale prompt, with no log line and no error.

Publishing state first, then task, then images (with a 50 ms gap, free against 1.1 s of inference)
and verifying the echoed `header.stamp` on every returned chunk:

| | as published | after the fix, run 1 | run 2 |
|---|---:|---:|---:|
| MAE / action std, all dims | 0.149 | **0.143** | **0.145** |
| gripper | 0.036 | **0.022** | **0.022** |

Raw output: `bench/libero-replay-accuracy-stateorder.txt`. The gripper improvement is far outside the
0.002 run-to-run spread. **We did not A/B against the old code**, so this is consistent with the race
having been real rather than proof of it; the honest claim is that the ordering is now correct and the
number is now 0.143–0.145. Zero chunks were discarded for stamp mismatch in either run.

**What this is not:** open-loop teacher-forced comparison against one episode is not a task success
rate. §10 is.

## 8. Toolchain findings worth knowing

The apt package `ros-jazzy-qrb-ros-nn-inference` / `ros-jazzy-qrb-inference-manager` is **1.1.1**;
upstream `main` is **2.2.0**. For this model the deb is unusable and building from source is
mandatory:

| Capability | deb 1.1.1 | upstream `main` |
|---|---|---|
| int32 tensor inputs (`lang_tokens`) | **rejected** | supported |
| Multiple input tensors | partial | supported (2.1.1) |
| HTP performance / DCVS init | absent | `QNN_HTP_BURST=1` → locked TURBO (2.2.0) |
| DMA-buf zero-copy for `.bin` models | absent | `inference_execute_dmabuf()` (2.0.0) |

> **`OutputTensor` is an ABI break.** Upstream added four dmabuf fields
> (`output_dmabuf_fd/size/offset/ptr`), changing `sizeof(OutputTensor)`. Code compiled against the
> deb headers but linked against the rebuilt library reads `std::vector<OutputTensor>` at the wrong
> stride and crashes with `std::bad_array_new_length`. Any dependent package must be **cleanly
> rebuilt** against the overlay headers, not just relinked.

Smaller ones:

- `ppa:ubuntu-qcom-iot/qcom-ppa` is already on the stock image; re-adding it is a fatal
  `Conflicting values set for option Trusted`. `ppa:ubuntu-qcom-iot/qirp` is absent but installs
  cleanly and **imported its signing key without `NO_PUBKEY`** — contrary to what we expected.
- `packages.ros.org` resolves to `ftp.osuosl.org`, whose TLS certificate covers only `*.osuosl.org`,
  so `https://` fails. The official docs page uses `http://` anyway; GPG signature verification is
  what actually protects the packages.
- `/dev/fastrpc-cdsp` and `/dev/fastrpc-cdsp1` are `fastrpc:fastrpc` `crw-rw-r--`. Inference works
  as a normal user; no root needed.
- Enabling burst mode on the *second* device can fail with `Failed to set powerConfig … 0x32cb`;
  upstream warns and continues at default performance.

## 9. ROS 2 integration

```
sensor_msgs/Image × N  ─┐
std_msgs/String   ~/task ├─► qrb_ros_vla ──► qrb_ros_vla_msgs/ActionChunk    ~/action_chunk
std_msgs/Float32MultiArray ~/state       └─► qrb_ros_vla_msgs/InferenceStats ~/stats
std_msgs/Int32MultiArray ~/lang_tokens (raw override)
```

Inference takes ~1.1 s, far too long for an executor thread, so it runs on a dedicated worker.
Frames arriving mid-inference are **dropped, not queued** — a VLA acting on stale observations is
worse than one acting at a lower rate. Every chunk ships an `InferenceStats` message so no
performance claim is ever detached from a live measurement.

Verified end to end with `demo/synthetic_publisher.py`: 87 consecutive chunks, 50×7 DoF each, steady
0.90 chunks/s on `libQnnHtp.so`.

### Gazebo does not render headless on this board
`qrb_ros_simulation` was the intended showcase surface and `gz-harmonic` **does** install on arm64
(`scripts/install-gazebo-sim.sh`), shipping a real manipulator (`rml_63_gripper_arm`), an AMR, Orbbec
camera xacros, and office/warehouse worlds. But camera sensors need a GL context, and on Ubuntu
Server on the IQ-9075 the render engine will not start:

```
libEGL warning: failed to open /dev/dri/renderD128: Permission denied
OGRE EXCEPTION(3:RenderingAPIException): eglChooseConfig for device ... EGL_EXT_platform_device
OGRE EXCEPTION(3:RenderingAPIException): OpenGL 3.3 is not supported
```

Two independent problems. The permission error is fixable — the user must be in the `render` group
for `/dev/dri/renderD128`. The second is not: **Ogre2 wants desktop OpenGL 3.3, and the Adreno driver
here exposes OpenGL ES** (`libEGL_adreno.so.1`, "OpenGL ES Shader Compiler"). Falling back to
`ogre` (v1) aborts, and forcing Mesa llvmpipe (`LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe`)
segfaults inside `Ogre2RenderEngine::LoadImpl`.

So *Gazebo's* sim path on-device is limited to physics-only worlds with no camera sensors — useless
for a VLA, whose entire input is images.

### But MuJoCo does render headless here, and the reason matters

The obvious conclusion — "this board cannot render, so the simulator must live on a laptop" — is
wrong, and we nearly shipped it. **LIBERO's renderer works on this board.** The distinction is
narrow and entirely explains the difference:

| | Gazebo / Ogre2 | LIBERO / MuJoCo |
|---|---|---|
| Requires | OpenGL 3.3 **core** | OpenGL 3.3, satisfied by a **compatibility** context |
| Gets from Mesa llvmpipe | crashes in `Ogre2RenderEngine::LoadImpl` | OpenGL **4.5 compatibility**, works |
| Result | no camera sensors | 256×256 offscreen RGB, both cameras |

Mesa's software rasterizer advertises a 4.5 *compatibility* profile. Ogre2 demands *core* and dies;
MuJoCo's classic renderer is happy with compatibility. Two env vars and one apt package are the
whole story:

```bash
sudo apt-get install -y libosmesa6 libosmesa6-dev
export MUJOCO_GL=osmesa
```

`MUJOCO_GL=egl` also works if `__EGL_VENDOR_LIBRARY_FILENAMES` is pointed at Mesa's
`50_mesa.json`, at the same speed — the vendor filter is required because the default glvnd order
picks `libEGL_adreno.so.1`, which returns zero configs for `EGL_OPENGL_BIT`. There is no hardware
path: the Adreno 663 is reachable only as OpenGL ES, `/dev/dri/renderD128` is the `msm_dpu` display
controller rather than the GPU so Mesa's `freedreno` cannot bind it, and `zink` is refused by both
Vulkan ICDs. Rendering here is software, and that is the end of the road.

**Consequence: everything runs on the board.** No laptop, no cross-host DDS, no bridge. §10 is the
closed loop this unlocked, and the workshop stays laptop-free.

## 10. Closed-loop task success

§7 proves the policy predicts sensible actions. It cannot prove the policy *completes tasks*, because
the demonstrator's actions — not the policy's — determine what happens next. `demo/libero_closed_loop.py`
removes that crutch: the LIBERO simulator runs on the board, the policy's own actions drive it, and the
benchmark's own success predicate decides the outcome.

```
sim.render() ×2 ─rot180─► /vla/camera{0,1}/image_raw ─┐
8-D proprio ─MEAN_STD──► ~/state                      ├─► qrb_ros_vla (NPU, ~1.13 s)
task string ───────────► ~/task                       ┘            │
                                                                    ▼
env.step() × H ◄─un-normalize─ ~/action_chunk ◄──────────── 50 × 7 actions
check_success() ──── repeat until success or max_steps
```

### 10a. Rendering happens only when the policy replans

A 2-camera 256×256 observation costs **340 ms** on the software rasterizer; a physics step costs
**31 ms**. Rendering every step turns a 220-step episode into 84.8 s; rendering once per replan turns
it into 8.7–15.3 s. That is not a shortcut — it is exactly what action chunking buys, since the policy
only needs an observation when it plans.

The consequence is worth stating plainly, because it inverts the expectation: **on this board the
simulator is the expensive half, not the 3B VLA.** The NPU produces 50 actions in 1.13 s; llvmpipe
needs 340 ms to draw one pair of frames. Per action executed, the sim dominates.

### 10b. The conventions, resolved by measurement rather than documentation

Four conventions silently destroy a VLA's accuracy while everything still appears to run. Each was
settled against the recorded dataset, which is on disk, so none of them is a matter of opinion.
Harnesses: `bench/verify_libero_contract.py`, `bench/verify_libero_action_units.py`.

| Convention | Verdict | Evidence |
|---|---|---|
| 8-D state layout | `eef_pos(3) + rotvec(3) + gripper_qpos(2)` | max abs err **0.0057** vs dataset `state[0]` |
| Camera orientation | **rot180**, both cameras | agentview NCC **+0.820** vs −0.045 identity |
| Gripper sign | **−1 open, +1 closed** | qpos 0.021→0.039 on −1, →0.001 on +1 |
| Steps per action | **exactly one** | 8.65 mm vs 217 mm position error at t=50 |
| Settling | ~10 idle steps, gripper open | err 0.0209 → 0.0057 |

`rot180` independently matches the `img[::-1,::-1]` that openvla/openpi apply — corroboration, not
coincidence. It applies equally to `obs["…_image"]` and to raw `sim.render()`, which is worth checking
rather than assuming since they are different code paths.

> **The axis-angle branch is a trap, and the obvious fix is the wrong one.** LIBERO's home pose points
> the gripper straight down, putting the rotation angle at almost exactly π — the discontinuity of the
> axis-angle representation. Pinning the branch by the sign of the quaternion's scalar part is the
> natural move and it fails here: a rollout flips from +3.14 to −3.14 partway through, a jump of 2π,
> while representing the *same* physical orientation. Because state is spliced into the language prompt
> as 256 discretized bins (§4), that reads to the policy as the wrist having spun 360° between two
> consecutive control steps. The recorded dataset never wraps — over all 200 frames its first rotvec
> component stays within [+2.95, +3.20] and the largest step-to-step change is 0.021 — so the correct
> rule is **continuity against the previous state**, not a fixed sign test.

### 10c. Verifying the harness before trusting the number

A wrong bridge and a bad policy are indistinguishable from a success rate alone, so correctness was
established in rungs that each fail loudly:

1. **Units.** Feed the *dataset's own* recorded actions into the sim from the matching initial state.
   End-effector position tracks the recording to **8.2–10.4 mm across all 200 frames, flat** — and
   flatness is the actual proof, since wrong units or cadence produce monotonic divergence (the
   2-steps-per-action variant diverges to 217 mm by t=50). The residual is the initial-state offset,
   not drift.
2. **Observation path.** Reproduce §7's open-loop MAE through the same code: 0.143/0.145 (§7a).
3. **The success predicate itself.** A high success rate is only evidence about the *policy* if a
   policy-shaped hole in the same harness scores zero. `bench/verify_libero_null_policy.py` runs the
   identical episode loop with Pi0.5 replaced by zeros (hold still) and by uniform random actions in
   the controller's own `[-1, 1]` space:

   | null policy | successes |
   |---|---:|
   | zeros | **0/12** |
   | random | **0/12** |
   | goal predicate already true at reset | **0/24** |

   Both must be zero, and are — across four tasks × 3 initial states × 520 steps. Had either
   scored, the closed-loop rate would have been measuring the harness rather than the model, and
   would have looked identical from the outside. Raw output: `bench/libero-null-policy.json`.
4. **Closed loop.** Only then, a success rate.

### 10d. Result

**All ten tasks of LIBERO-10, ten initial states each — 100 closed-loop episodes, 86 successes:**

| | value |
|---|---|
| **success rate** | **86/100 = 86.0%**, 95% CI **77.9–91.5%** (Wilson) |
| replan horizon | 10 actions of each 50-action chunk |
| max episode length | 520 steps |
| chunks executed | 3153 |
| NPU latency | 1128.1 ms mean, range 1104.7–1148.5 across all 100 episodes |
| backend, every chunk | `libQnnHtp.so` |
| chunks discarded on stamp mismatch | **0** |
| inference retries | **0** |
| total wall clock | 90 min |

The ten initial states are two separate bands of five — LIBERO's indices 0–4 and 20–24 — run as
independent sweeps precisely because a single band cannot tell you whether the states you happened to
pick were unrepresentatively easy. They were not:

| Band | Rate | 95% CI |
|---|---|---|
| initial states 0–4 | 42/50 = 84.0% | 71.5–91.7% |
| initial states 20–24 | 44/50 = 88.0% | 76.2–94.4% |

The intervals overlap comfortably, so the two bands are consistent and pooling them is legitimate.
`bench/analyze_libero_sweep.py` does the pooling, and refuses to do it if the bands disagree on suite,
replan horizon, step cap, library versions, image orientation or steps-per-action — and warns loudly if
they cover different task sets, because an incomplete band that happens to be missing the hardest tasks
reports a flatteringly high rate and drags the pooled figure up with it.

Per task, pooled over both bands:

| # | Pooled | 0–4 | 20–24 | Task |
|---|---|---|---|---|
| 0 | 10/10 | 5/5 | 5/5 | put both the alphabet soup and the tomato sauce in the basket |
| 1 | 10/10 | 5/5 | 5/5 | put both the cream cheese box and the butter in the basket |
| 2 | 10/10 | 5/5 | 5/5 | turn on the stove and put the moka pot on it |
| 3 | 8/10 | 5/5 | 3/5 | put the black bowl in the bottom drawer of the cabinet and close it |
| 4 | 9/10 | 5/5 | 4/5 | put the white mug on the left plate and the yellow and white mug on the right |
| 5 | 10/10 | 5/5 | 5/5 | pick up the book and place it in the back compartment of the caddy |
| 6 | 8/10 | 3/5 | 5/5 | put the white mug on the plate and the chocolate pudding right of the plate |
| 7 | 10/10 | 5/5 | 5/5 | put both the alphabet soup and the cream cheese box in the basket |
| 8 | 7/10 | 2/5 | 5/5 | put both moka pots on the stove |
| 9 | **4/10** | 2/5 | 2/5 | put the yellow and white mug in the microwave and close it |

Half the tasks are solved 10/10. The interesting entries are the disagreements: task 8 scored 2/5 in
one band and 5/5 in the other, which is a reminder of how little a five-episode sample resolves. Task 9
is the only task low in *both* bands — closing the microwave after placing the mug is genuinely the
hardest thing in this suite for this policy.

**Every one of the 14 failures ran to exactly 520 steps.** None diverged, thrashed, or produced
nonsense — they ran out of step budget mid-task. That is a meaningful distinction: the failure mode is
"too slow for the cap" rather than "wrong", and it means the 520-step ceiling is itself part of the
result. A larger cap would likely convert some of them, which is exactly why the cap is reported
alongside the rate.

Latency held at 1128.1 ms mean with a 44 ms spread across **90 minutes** of continuous load, so **no
thermal degradation was observed** — though this is still not the sustained-load soak test §11 asks for,
and a sweep is a duty cycle rather than a pinned load.

The cost model predicted from measured parts holds: for the 285-step episode, 29 × 1.126 s inference
+ 29 × 0.34 s render + 285 × 0.031 s physics = 51.3 s against **51.1 s** observed.

**What this is and is not.** It is 10 initial states per task, not LIBERO's full 50, so it is a
*reduced* protocol — 86% with a roughly ±7-point interval, not a benchmark figure. Two bands agreeing
raises confidence that the sampled states are not pathological, but twenty percent of the available
states is still a sample. It is also one suite of four. A like-for-like comparison against published pi0-class results would need
their replan horizon and step cap to match ours, and we have not verified those, so no comparison is
drawn here.

Every episode is one JSON object in `bench/libero-closed-loop-sweep.jsonl` carrying suite, task
index, initial-state index, replan horizon, step cap, mujoco/robosuite/numpy versions, the per-chunk
backend and the latency distribution — so the number above is auditable rather than merely asserted.
Reproduce with `scripts/run-libero-sweep.sh libero_10 5` and `INIT_START=20
scripts/run-libero-sweep.sh libero_10 5`, then pool with `bench/analyze_libero_sweep.py`.

**Proof the NPU ran** is per-control-step here rather than asserted once: `InferenceStats.backend`
accompanies every chunk and the harness records the set of backends observed. A CPU fallback would
show a different backend and a latency an order of magnitude worse.

**Not yet measured:** the `cv_bridge` + `ResizeNormalizeChw` cost, which runs on the node's executor
thread and is excluded from `InferenceStats` — so closed-loop step time exceeds the 1111 ms of §6 by
an amount we have not quantified. Sustained-load thermal behaviour is also unmeasured; a long sweep
pins eight A78C cores on software rasterization while the NPU runs the policy, which is precisely the
contention regime §11 flags.

## 11. Honest limitations

- **The task success rate is on a reduced protocol, not the standard benchmark.** §10 closes the
  loop and the policy does now see the consequences of its own actions, but a full LIBERO evaluation
  is 4 suites × 10 tasks × 50 episodes and is not affordable here. Any rate we publish states its
  episode count and confidence interval, and is never presented as a suite-level benchmark number.
- **Action semantics are embodiment-specific.** The published export is calibrated on LIBERO (Franka
  Panda, 7-DoF). Its output is *not* valid joint commands for a different arm — including the RML-63
  in `qrb_ros_simulation`. §12 quantifies how far off one concrete candidate arm is.
- **The closed loop is not run-to-run deterministic.** The same task, the same initial state and the
  same code produced 285, 286 and 289 steps on three runs — the flow-matching action expert samples
  its initial noise and we do not seed it. Outcomes were identical (all three succeeded), so this
  perturbs trajectories rather than success, but it means **no closed-loop number here is exactly
  reproducible** and any two conditions must be compared statistically rather than by equality. It is
  also why §12's harness gate is an interval-overlap test and not a bit-exactness test.
- **State normalization statistics are external.** The node expects pre-normalized state; the demos
  use the dataset's `meta/stats.json`. A different robot needs its own statistics.
- **Gazebo cannot render on the board** (§9), so no on-device Gazebo camera demo. LIBERO/MuJoCo
  can (§9), which is what the closed-loop demo uses — but only through a software rasterizer, so
  rendering is the expensive half of the loop (§10).
- **No power or thermal measurements**, and no sustained-load soak test. All numbers are from short
  runs of 12–15 chunks on a thermally unstressed board.
- **The NPUs are an exclusive resource, and the dev board is shared.** Two processes that each map the
  full 2.9 GB bundle will contend for the same CDSP mapping budget: the second one hits the `err 1002`
  path from §5a, and both see inflated latency. Any measurement here is only trustworthy if nothing
  else was on the NPU at the time. Some runs during development overlapped with other work on this
  board (observed spread: 1111–1139 ms/chunk for the same configuration), so **the headline numbers
  should be re-measured under confirmed exclusive access before publication.** `bench/pi05/measure.py`
  samples loadavg and thermals around every run for exactly this reason; the simpler `pi05_bench` does
  not.
- Latency was measured with **synthetic and replayed images**, never a live camera; `qrb_ros_transport`
  zero-copy DMA-buf is not yet wired, so a real camera path may differ.
- The `precompiled_qnn_onnx` variant is published for QCS9075 and untested here.
- The `~1600 MiB` per-device mapping ceiling is empirical and conservative, not a documented limit.

## 12. Driving a real arm: what the hardware would have to be

§11 says the export's action semantics are embodiment-specific. This section makes that concrete for
one candidate: a $329 VUPN2355 6-DOF desktop arm (6× TD-8125MG PWM servos, no encoders) driven over
I²C through a PCA9685. The board side is easy — `/dev/i2c-18/19/20` are live and JLS1 pins 8/10 are
I²C by default. The policy side is the question, and it is cheaper to answer in simulation than by
buying into it.

**Method.** Reproduce each of the arm's physical limitations inside LIBERO, one at a time, and score
each against the measured 86/100 baseline of §10. This is only interpretable because the null policy
scores 0/24, so the 86% is signal rather than a harness artifact. `bench/arm_embodiment.py` holds the
model; the ablation runs through the same `demo/libero_closed_loop.py` used for the headline number,
behind a default-off `--ablate` flag, so with the flag absent the published path is unchanged.

**The instrument is checked before the results are believed.** A quantizer that silently passed
sub-quantum motion through, or an identity mode that was not an identity, would yield a plausible
table that measured nothing. `bench/verify_arm_embodiment.py` asserts all twelve properties the modes
claim (12/12) and the `none` condition then scored **18/20 = 90%** on the board, overlapping the
baseline's 77.9–91.5% interval — so the ablation harness demonstrably leaves the loop alone.

### 12a. The arm cannot reach, and that alone settles it

Reach must be measured from the robot's base, not the world origin — and in LIBERO the base position
is **scene-dependent**, taking three distinct values across `libero_10` (x = −0.51, −0.66 or −0.75).
So it is read from the live sim per task rather than assumed; assuming one value is exactly how an
earlier pass of this analysis got a 604 mm requirement wrong by 200 mm. The Panda itself has 855 mm of
reach and the scenes are laid out for that. Screening every task against object and fixture positions
(`bench/verify_arm_workspace.py`):

| Metric, across all 10 tasks | Value |
|---|---|
| Radial reach required from the base | **604–751 mm** |
| Nearest required point | 276–450 mm |
| Tasks whose *nearest* point is already beyond a 300 mm reach | **6 of 10** |
| Tasks that fit | **0 of 10** |
| Linear scale factor by which the scene exceeds the arm | **2.01× – 2.50×** |

This is geometry, not policy quality: no fine-tune, calibration or IK makes a 300 mm arm touch
something 700 mm away. It is also robust to the fact that 300 mm is currently an *assumed* reach —
even taking the product page's entire 370 mm fully-extended bounding box as though every millimetre
were usable reach, the scenes are still **1.63× – 2.03×** too large. **LIBERO on this arm is not a
tuning problem, it is the wrong embodiment class**, and nothing in 12b–12f changes that.

### 12b. Slew rate is a non-issue — 31× margin

Worth stating because it is the intuitive worry and it is wrong. From the recorded episode
(`bench/analyze_arm_envelope.py`), the policy's most aggressive single step asks for 10.7 mm of
translation and 1.20° of rotation at 10 Hz. A 0.16 s/60° servo delivers 37.5° per 100 ms tick:

| | Needed (worst case) | Available | Margin |
|---|---|---|---|
| Joint travel per control tick | 1.20° | 37.5° | **31.2×** |

Hobby servos are far faster than this policy needs. Nothing in the control path has to be optimised
for speed — which the 22% NPU duty cycle (1.11 s of inference per 5.0 s of commanded motion) already
suggested from the other direction.

### 12c. PWM quantization looks fatal on paper and costs nothing in the loop

This is the one place where a plausible analytical argument gave the wrong answer, so both halves are
recorded.

**The prediction.** The PCA9685's 12 bits span one PWM **period**, not the servo's 500–2500 µs pulse
band, so a *faster* frame rate yields a *finer* angular quantum — counter-intuitive but arithmetically
plain. Against what the policy commands per step (median 7.30 mm, 0.54°):

| Frame rate | Angular quantum | Linear quantum at 300 mm | Counts per commanded rotation step |
|---|---|---|---|
| **50 Hz** | 0.4395° | 2.30 mm | **1.23** |
| 100 Hz | 0.2197° | 1.15 mm | 2.45 |
| 200 Hz | 0.1099° | 0.58 mm | 4.91 |
| 330 Hz | 0.0666° | 0.35 mm | 8.09 |

At 50 Hz the median commanded rotation is barely one count, which reads as though the orientation half
of every action would be discretized into nothing. The obvious recommendation was "run at ≥200 Hz."

**The measurement says otherwise.** Applying the 50 Hz quantum to the absolute commanded pose in the
closed loop, with the pessimistic max-reach radius:

| Condition | k/n | Rate | 95% CI | vs baseline |
|---|---|---|---|---|
| baseline (§10) | 86/100 | 86.0% | 77.9–91.5% | — |
| `quant` @ 50 Hz | **43/50** | **86.0%** | 74–93% | **+0.0 pts** |

Per task it is indistinguishable from baseline except on task 9, which was already the weakest
(4/10 at baseline, 1/5 here). Mean residual between requested and realized pose: 1.107 mm.

**Why the paper argument failed.** It measured open-loop fidelity, and this loop is not open. The
policy replans every 10 steps from a fresh observation, so quantization error is corrected rather than
accumulated; the OSC impedance controller (kp = 150) chases a target rather than integrating a velocity,
so a stuttering delta stream still yields smooth motion; and sub-quantum requests accumulate in the
pose residual until they cross a count instead of vanishing. **Counts-per-commanded-step is not a valid
predictor of closed-loop success**, and the earlier "≥200 Hz" recommendation is withdrawn — 50 Hz is
adequate, which is also the easier thing to wire.

<Note>
This condition was measured twice. The first run reported 10/50 = 20% and was **retracted**: the
quantizer was differencing *absolute* rotation vectors, which is invalid near LIBERO's `|rotvec| ≈ π`
home pose. The tell was that an 8× finer quantum barely reduced the injected error (1.54 → 1.51 ×
signal) while reversing the commanded rotation direction on 27% of steps. `bench/retracted/` keeps the
data and the diagnosis; `bench/verify_arm_embodiment.py` now covers the rotation path it missed.
</Note>

### 12d. The policy does not appear to need joint feedback

The expensive worry about a PWM servo arm is that it has no encoders: it knows what it *asked* for and
nothing about what happened. So the cheapest decisive test is to remove proprioception entirely —
publish the dataset mean as the state on every step, which normalizes to all zeros, telling the policy
nothing — and see what it costs. Any real feedback scheme, dead reckoning included, supplies strictly
more information than that, so this brackets the whole question from the pessimistic side.

| Condition | State the policy receives | k/n | Rate | 95% CI (Wilson) | vs baseline |
|---|---|---|---|---|---|
| baseline (§10) | true 8-D pose | 86/100 | 86.0% | 77.9–91.5% | — |
| `none` (gate) | true 8-D pose | 18/20 | 90.0% | 70–97% | +4.0 pts |
| `state-const` | **the dataset mean** | **44/50** | **88.0%** | 76–94% | **+2.0 pts** |

**It costs nothing measurable.** The intervals overlap almost completely; the point estimate is
marginally *higher*, which at these sample sizes is noise, not an improvement. On `libero_10` this
export is effectively vision-dominated, which makes sense — the wrist camera sees the gripper, so the
pose is largely recoverable from pixels.

Two consequences:

- **A feedback-less arm is viable on this axis**, which removes the main objection to cheap servo
  hardware. Encoders are not what stands between this policy and a real arm.
- **The dead-reckoning ablation was dropped as provably uninformative.** Dead reckoning carries more
  information than a constant, so its result is bracketed between 86% and 88% and measuring it would
  consume 45 minutes of exclusive NPU time to learn nothing. Recorded here rather than silently
  skipped.

**This result deserves suspicion, not celebration.** "Removing an input changes nothing" has an
innocent reading and a worrying one, and they are not distinguished by this experiment:

- *Innocent:* the policy genuinely treats state as redundant given two camera views.
- *Worrying:* the state channel contributes nothing because something upstream is wrong, in which case
  §4's tokenization and §7's accuracy work deserve re-examination.

What the experiment does establish is that this is not a *structural* break. §4 splices state into the
prompt as 256 discretized bins, and the harness publishes state before images precisely because a
missing state produces a different prompt shape — but here a valid, in-distribution mean vector was
published, so the prompt structure was identical to baseline and only the values differed. The model
is value-insensitive, not un-wired.

Distinguishing the two readings needs one more condition that has **not** been run: publish an
*adversarial* state — a pose wildly inconsistent with the images — and see whether success degrades. If
even that changes nothing, the state path is inert and it is a correctness question about the published
pipeline, not a fact about hobby arms. That is the highest-value follow-up here.

### 12e. The missing 5th-vs-6th DOF is the expensive one

"6 DOF" on these arms counts the gripper, leaving 5 positioning joints: base yaw, then a chain of
pitches in the vertical plane that yaw defines, then a roll about the tool axis. The consequence is
that **tool azimuth is not independently commandable** — it is whatever base yaw already had to be to
put the tip where it is — while the policy emits full 6-DOF pose deltas. The ablation removes the
world-z component of each commanded rotation delta and changes nothing else:

| Condition | k/n | Rate | 95% CI | vs baseline | Intervals disjoint? |
|---|---|---|---|---|---|
| baseline (§10) | 86/100 | 86.0% | 77.9–91.5% | — | — |
| `dof5` | **25/50** | **50.0%** | 37–63% | **−36.0 pts** | **yes** |

This is the only condition tested that is *statistically separated* from the baseline, and it is not
close. Per task it separates almost binarily:

| Task | Baseline | `dof5` | What the task needs |
|---|---|---|---|
| 0 put both soup and tomato sauce in the basket | 10/10 | **5/5** | drop into an open basket |
| 1 put cream cheese and butter in the basket | 10/10 | **5/5** | drop into an open basket |
| 7 put soup and cream cheese in the basket | 10/10 | **5/5** | drop into an open basket |
| 6 put mug on plate, pudding right of plate | 8/10 | **5/5** | place on a flat plate |
| 4 put two mugs on left/right plates | 9/10 | 4/5 | place on a flat plate |
| 2 turn on the stove and put the moka pot on it | 10/10 | **0/5** | align with a burner and a handle |
| 5 place the book in the caddy's back compartment | 10/10 | **1/5** | insert into a slot |
| 3 put the bowl in the bottom drawer and close it | 8/10 | **0/5** | align with a drawer face |
| 8 put both moka pots on the stove | 7/10 | **0/5** | align with burners |
| 9 put the mug in the microwave and close it | 4/10 | **0/5** | align with a door opening |

The split is not random, and that is the strongest evidence the ablation is measuring what it claims:
**every task that survives places an object into an open-topped, orientation-tolerant target, and every
task that collapses requires aligning with a specific opening** — a burner, a drawer face, a caddy
compartment, a microwave door. Losing tool yaw is exactly the failure a lost base-coupled azimuth
predicts. Task 2 going from 10/10 to 0/5 is the cleanest single data point; task 9 is the weakest, since
its baseline was already only 4/10.

Read this as a *lower bound on the damage*: the projection is first-order (it zeroes the world-z
axis-angle component rather than projecting the full pose onto the reachable manifold), and a real
5-joint arm also carries the coupling between azimuth and position that this does not model. **A 5-DOF
arm is not adequate for half of this suite**, and unlike reach that is a property of the joint count
rather than the scale.

### 12f. What would have to be true

Everything measured, in one table:

| Limitation reproduced | k/n | Rate | 95% CI | vs baseline | Separated? |
|---|---|---|---|---|---|
| — baseline (§10) | 86/100 | 86.0% | 77.9–91.5% | — | — |
| `none` — harness gate | 18/20 | 90.0% | 70–97% | +4.0 | no |
| `state-const` — no joint feedback at all | 44/50 | 88.0% | 76–94% | +2.0 | no |
| `quant` — 50 Hz PWM command quantum | 43/50 | 86.0% | 74–93% | +0.0 | no |
| `dof5` — 5 positioning joints, not 6 | **25/50** | **50.0%** | 37–63% | **−36.0** | **yes** |
| `composite` — all four at once | **25/50** | **50.0%** | 37–63% | **−36.0** | **yes** |

**The degradations do not compound.** `composite` enables slew limiting, the 5-DOF projection, 50 Hz
quantization and dead-reckoned state simultaneously, and lands on exactly `dof5`'s 25/50 — the same
result on 8 of 10 tasks, with the two differences swapping a single episode each in opposite directions
(task 4 gains one, task 6 loses one), which is what §11's non-determinism produces. So the joint count
is not merely the largest term, it is the *only* term: everything else is free even in combination.

Two incidental confirmations from that run: the slew limiter engaged on **0 of ~13 000 steps**, making
12b's 31× margin an observation rather than an estimate, and the mean quantizer residual was 1.074 mm,
consistent with the 2.30 mm quantum being absorbed rather than accumulated.

**Only two things about this arm actually matter, and neither is the one you would guess:**

- **Reach ≥ ~800 mm**, or a spatially rescaled task set of your own — and rescaling changes the action
  distribution the policy was trained on, so it is not free. This is the fatal one (12a).
- **6 true positioning joints, not 5.** −36 points, the only statistically separated condition: a
  5-joint arm loses every task that needs to align with a specific opening (12e). Count the servos
  before buying — "6 DOF" nearly always means 5 + a gripper.

Three things that look like blockers and are not:

- **Joint feedback — not required.** Removing proprioception entirely costs nothing measurable (12d).
- **PWM frame rate — 50 Hz is fine.** The resolution argument said otherwise and the measurement
  overruled it (12c).
- **Servo speed — 31× more than needed** (12b).

So: **reach ≫ joint count ≫ everything else.** The two objections most likely to be raised against a
$329 servo arm — that it has no encoders and that its PWM is coarse — are precisely the two that cost
nothing here. The two that decide it are geometric.

**Caveats.**

- The arm geometry in `bench/arm-spec-measured.json` is still `status: assumed` (datasheet and
  product-page figures). Its `m1_checklist` lists the physical measurements that would replace them,
  and every artifact stamps the spec's status and SHA so an assumed-geometry number cannot be mistaken
  for a measured one. Only the reach conclusion has been shown robust to the assumption; the
  quantization result used the *pessimistic* max-reach radius, which cuts the safe way.
- The 5-DOF projection is first-order — it zeroes the world-z rotation component rather than projecting
  the full pose onto the reachable manifold, and it does not model the coupling between azimuth and
  position that a real 5-joint arm has. Treat −36 points as a lower bound on the damage.
- **An instrument check is only as good as its coverage.** `bench/verify_arm_embodiment.py` passed
  12/12 while the rotation quantizer was badly wrong, because every quantization check drove
  translation only. One 50-episode condition was measured, published internally as a −66 point result,
  and retracted. It now covers the rotation path and asserts 17/17. `bench/retracted/` keeps the bad
  data and the diagnosis rather than deleting them.
- Everything here is `libero_10` only, at 5 episodes per task per condition, and inherits §11's
  non-determinism: conditions are comparable statistically, never by equality.

## 13. Reproducing

```bash
scripts/install-ros2-jazzy.sh          # ROS 2 Jazzy
scripts/install-qrb-ros.sh             # qirp PPA + QRB ROS packages
sudo apt-get install -y qairt-libs qairt-tools qairt-headers libsentencepiece-dev
scripts/fetch-tokenizer.sh             # PaliGemma tokenizer (checksum pinned)

cd artifacts && qai-hub-models fetch Pi0.5 --runtime qnn_context_binary \
    --precision mixed --chipset qualcomm-qcs9075 && cd ..

cd ros2_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && cd ..
source ros2_ws/install/setup.bash

QNN_HTP_BURST=1 ros2_ws/install/qrb_ros_vla/lib/qrb_ros_vla/pi05_bench \
  --bundle artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075 \
  --iters 12 --warmup 3 --json bench/pi05-latency-dual-npu.json

# correctness: dump stage tensors, replay through qnn-net-run, diff
QNN_HTP_BURST=1 ros2_ws/install/qrb_ros_vla/lib/qrb_ros_vla/pi05_bench \
  --bundle artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075 \
  --iters 1 --warmup 0 --dump-dir /tmp/pi05dump
python3 bench/verify_against_qnn_net_run.py --dump-dir /tmp/pi05dump

# the demo: replay a real human demonstration through the NPU and score it
scripts/fetch-libero-episode.sh
QNN_HTP_BURST=1 ros2 run qrb_ros_vla vla_node --ros-args \
  -p bundle_dir:=$PWD/artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075 \
  -p tokenizer_model:=$PWD/artifacts/paligemma_tokenizer.model -p action_dof:=7 &
python3 demo/libero_replay.py --steps 15 --stride 12 --horizon 20
```
