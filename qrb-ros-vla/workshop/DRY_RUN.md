# Dry run log

A dry run is only useful if it is **stopwatch-timed on a freshly flashed board within 48 hours of
delivery**, driven from `PREWORK.md` alone. Record each one here so timing drift stays visible across
revisions.

## Template — copy for each run

```
Date:            YYYY-MM-DD
Run by:
Board:           IQ-9075 EVK, freshly flashed? yes/no
Ubuntu / kernel:
Network:         wired / wifi / offline-USB
Attendee sim:    followed PREWORK.md from scratch? yes/no
```

| № | Module | Budget | Actual | Notes |
| --: | --- | --: | --: | --- |
| 0 | Preflight | 10 | | |
| 1 | Get it running | 15 | | |
| 2 | It's just ROS 2 | 20 | | |
| 3 | Drive it with real robot data | 20 | | |
| — | break | 10 | | |
| 4 | Make it 3× faster | 15 | | |
| 5 | The whole API is three calls | 15 | | |
| 6 | Tune it `[STRETCH]` | 15 | | |
| 7 | Numerically correct `[STRETCH]` | 15 | | |
| 8 | Closed-loop simulator `[STRETCH]` | 15 | | |
| | **total** | **150** | | |

### Escape hatches — each must be run from a genuinely broken state

Breaking it for real matters: running `fastforward.sh 1` on an already-built, already-running board
proves nothing.

| Hatch | How it was broken first | Worked? | Time |
| --- | --- | --- | --: |
| `fastforward.sh 0` | — | | |
| `fastforward.sh 1` | `rm -rf ros2_ws/build ros2_ws/install; pkill -f vla_node` | | |
| `fastforward.sh 2` | `pkill -f vla_node` | | |
| `fastforward.sh 3` | `rm /tmp/cp3-replay.log; pkill -f vla_node` | | |
| `fastforward.sh 4` | `rm /tmp/cp4-latency.json` | | |
| `fastforward.sh 5` | `rm /tmp/minimal_npu /tmp/cp5-minimal.log` | | |
| `fastforward.sh 6` | `rm /tmp/cp6-steps.log` | | |
| `fastforward.sh 7` | `rm /tmp/cp7-verify.log` | | |
| `fastforward.sh 8` | node stopped (as ff_4-7 leave it), `rm /tmp/cp8-closedloop.log` | **yes** | ~10 min |

### Offline test

- [ ] Whole workshop completed with the network cable **unplugged**
- [ ] `preflight.sh` passes offline
- [ ] USB-stick artifact copy path tested

### Preflight failure-hint test

Deliberately break each of these and confirm the printed `[FAIL]` hint is correct and
copy-pasteable:

- [ ] Remove a `.bin` file → checksum failure names the right file
- [ ] `deluser $(id -un) fastrpc` → CDSP readability hint works
- [ ] Remove `ROS_DOMAIN_ID` from `~/.bashrc` → the suggested command fixes it
- [ ] Move `artifacts/paligemma_tokenizer.model` aside → tokenizer hint works

### Multi-board test (cannot be caught on one board)

- [ ] Two boards running simultaneously **do not** see each other's topics
- [ ] `ros2 topic echo /qrb_ros_vla/action_chunk` on board A shows only board A's chunks

### Outcome

```
Total wall time:
Modules cut:
Checkpoints that failed:
Symptoms to add to TROUBLESHOOTING.md:
Runtime table updated in README.md?   yes/no
```

---

## Run 1 — component verification, original module design (2026-07-29)

Validated that every checkpoint and escape hatch executed on real hardware. **Superseded:** this ran
against the first module design, which was restructured after review (see Run 2) because it led with
hardware internals rather than with getting a VLA working.

All nine checkpoints of the old numbering passed. Findings carried forward:

- `pyarrow` has no arm64 apt package, so reading LIBERO's parquet live would have required a forbidden
  `pip install`. Fixed with `scripts/prepare-libero-npz.py`, which converts to a numpy-only `.npz`
  (29.3 MB) during pre-work.
- `preflight.sh` reported 21/22 because `ROS_DOMAIN_ID` was not persisted. The printed hint was
  executed verbatim and produced 22/22, confirming the hint is correct.

## Run 2 — component verification, current module design (2026-07-29)

Not a full timed attendee-path run. This confirmed the **restructured** modules, checkpoints, and
escape hatches all execute and pass on real hardware.

```
Date:            2026-07-29
Board:           IQ-9075 EVK (dev board, not freshly flashed)
Ubuntu / kernel: 24.04.4 / 6.8.0-1080-qcom
Network:         wired
```

| Checkpoint | Result | Measured |
| --- | --- | --- |
| CP-0 | PASS | preflight 22/22 in 2.6 s |
| CP-1 | PASS | node publishing action chunks, `chunk_size: 50` |
| CP-2 | PASS | task changed live over ROS to *"put the red block in the drawer"*, chunks continued |
| CP-3 | PASS | normalized MAE **0.123** vs the human demonstration |
| CP-4 | PASS | dual-NPU chunk **1139 ms**, zero context paging |
| CP-5 | PASS | `minimal_npu` built and ran `vision_encoder`, `img_embed` = 2 097 152 bytes |
| CP-6 | PASS | 3 denoise-step settings measured |
| CP-7 | PASS | 4/4 components, 45/45 tensors bitwise identical to `qnn-net-run` |
| CP-8 | PASS | closed loop completed the task; 1/1 episode, 252 steps, 26 chunks, `libQnnHtp.so`, 0 stale chunks |

**Also verified in this run:**

- `ros2 launch qrb_ros_vla vla.launch.py` works with no arguments; `htp_device_ids` resolves to
  `[1, 1, 0, 0]` and the tokenizer loads from the default path.
- `workshop/examples/minimal_npu.cpp` compiles with a single `g++` line and runs a 515 MiB model
  component on the NPU.
- `fastforward.sh 8` run cumulatively **from a node-down state**: ff_1 through ff_7 passed, ff_8
  detected the stopped node and restarted it, and CP-8 reported
  `2/2 closed-loop episodes completed the task on libQnnHtp.so` (275 and 292 steps, 0 stale chunks).
  This is the case that matters, because ff_4-7 deliberately stop the node to free the NPUs while
  Module 8 needs it running -- and the log-grep idiom the earlier hatches use would have wrongly
  concluded the node was already up, leaving every rollout to time out.
- **Two bugs surfaced only by running the hatch a second time, from a different directory.** Both are
  fixed and the hatch was then re-run from `/tmp` end to end: ff_1-ff_8 all ran, 2/2 episodes
  succeeded (286 and 285 steps), `CP-8 PASS`, zero abort markers.
  1. *Working directory.* `fastforward.sh` never `cd`s to the repo root and `ff_8` relied on a
     relative `--stats-npz` default, so the hatch only worked from the repo root. Artifact defaults
     in the five affected scripts now resolve against the repository; an explicitly passed path is
     still relative to the cwd.
  2. *Intermittent abort after success.* The harness spun rclpy on a daemon thread and destroyed the
     node while that thread was still inside the executor, which aborted the process with
     `terminate called without an active exception` on roughly one run in two -- **after** the
     episodes had already succeeded. Under `set -o pipefail` that failed the pipeline, `set -e`
     killed the hatch, and CP-8 never ran. Shutdown is now ordered
     `executor.shutdown()` -> `join()` -> `destroy_node()`.
     Worth remembering as a class: a hatch that does its work and then dies before verifying looks
     like a checkpoint bug, not a teardown bug.

**Found and fixed during this run:**

- `minimal_npu.cpp` failed to link with `undefined reference to QrbInferenceManager::...` when
  `-I/opt/ros/jazzy/include` preceded the overlay include — it compiled against the older apt header.
  The documented build line now puts the overlay include first, and the symptom is in
  TROUBLESHOOTING.md under Module 5.

**Still outstanding before delivery:**

- [ ] Timed run on a **freshly flashed** board, driven from `PREWORK.md` alone
- [ ] Escape hatches re-run from genuinely broken states (above they ran from a working board)
- [ ] Full offline / cable-unplugged run
- [ ] Two-board `ROS_DOMAIN_ID` isolation test
- [ ] Per-module wall-clock timings with a stopwatch, including the talking
