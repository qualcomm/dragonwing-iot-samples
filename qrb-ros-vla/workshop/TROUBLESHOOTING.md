# Troubleshooting

Symptoms are **verbatim and greppable** — search this file for the text you see on screen.

`CP` is the checkpoint the symptom belongs to. If a fix does not work within two minutes, run the
escape hatch and move on; do not let one board hold up the room.

## Setup and pre-work

| Symptom (verbatim) | CP | Root cause | Fix | Escape hatch |
| --- | --- | --- | --- | --- |
| Serial console shows garbage or nothing | 0 | Wrong serial device — the board exposes **four**; the Linux console is the **second** | `sudo screen /dev/ttyUSB1 115200` | Use SSH instead |
| No `/dev/ttyUSB*` at all on Windows | 0 | FTDI VCP driver not installed | Install the FTDI VCP driver manually | Use SSH instead |
| Serial prompt exists but never shows `ubuntu login:` | 0 | That is the SAIL console, not the main domain console | Try the next `/dev/ttyUSB*` up | Use SSH instead |
| `Permission denied (publickey,password)` on ssh | 0 | Default `ubuntu` password never changed | Log in on serial, run `passwd` | — |
| `E: Conflicting values set for option Trusted` | 0 | `qcom-ppa` added twice; it is **already on the stock image** | `ls /etc/apt/sources.list.d/` then delete the duplicate, `sudo apt-get update` | — |
| `NO_PUBKEY <hex>` during `apt update` | 0 | Repository signing key missing | `sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys <hex>` | — |
| `SSL: no alternative certificate subject name matches target host name 'packages.ros.org'` | 0 | It resolves to `ftp.osuosl.org`, whose cert covers only `*.osuosl.org` | None needed — `install-ros2-jazzy.sh` falls back to `http://`; GPG still verifies packages | — |
| `[FAIL] CDSP readable by <user>` | 0 | Not in the `fastrpc` group | `sudo usermod -aG fastrpc $(id -un)`, then log out and back in | — |
| `[FAIL] model artifacts match manifest.sha256` | 0 | Truncated or partial download | Re-run pre-work step 4, or copy from the USB stick | Copy from USB |
| `[FAIL] free disk >= 30 GB` | 0 | Disk full | `sudo apt-get clean`, delete old builds | — |
| `[FAIL] all remoteproc cores running` | 0 | A DSP firmware core did not come up | Reboot (~24 s) | Reboot, then reflash if it persists |
| `[FAIL] ROS_DOMAIN_ID persisted` | 0 | Seat id not yet in `~/.bashrc` | Run the command the `[FAIL]` line prints, then open a new shell | — |

## Module 1 — getting it running

| Symptom (verbatim) | CP | Root cause | Fix | Escape hatch |
| --- | --- | --- | --- | --- |
| `colcon: command not found` | 1 | Build tools missing | `sudo apt-get install -y python3-colcon-common-extensions` | — |
| `Could not find a package configuration file provided by "qrb_inference_manager"` | 1 | Overlay not sourced, or built out of order | `source /opt/ros/jazzy/setup.bash`, then `colcon build` from `ros2_ws/` | `./workshop/scripts/fastforward.sh 1` |
| `fatal error: sentencepiece_processor.h: No such file` | 1 | Dev package missing | `sudo apt-get install -y libsentencepiece-dev` | — |
| `terminate called ... 'std::bad_array_new_length'` | 1 | **ABI mismatch** — compiled against apt headers, linked against the vendored 2.2.0 library | `rm -rf ros2_ws/build ros2_ws/install`, then rebuild cleanly | `./workshop/scripts/fastforward.sh 1` |
| `Input tensor 0 data type is not supported!` | 1 | Running against apt `qrb_inference_manager` 1.1.1, which rejects int32 | Rebuild so the vendored source is used | `./workshop/scripts/fastforward.sh 1` |
| `parameter 'bundle_dir' is required` | 1 | Node started by hand without parameters | Use `ros2 launch qrb_ros_vla vla.launch.py` instead | `./workshop/scripts/fastforward.sh 1` |
| `cannot load SentencePiece model` | 1 | Tokenizer missing | `./scripts/fetch-tokenizer.sh` | Copy from USB |
| `/qrb_ros_vla/action_chunk` not listed | 1 | Node not running, died, or still loading (~20 s) | Check Terminal A for errors | `./workshop/scripts/fastforward.sh 1` |
| Topic exists but no chunks ever arrive | 1 | Nothing is publishing camera images | Start `demo/synthetic_publisher.py` in Terminal B | `./workshop/scripts/fastforward.sh 1` |
| You see **other people's** topics in `ros2 topic list` | 1 | `ROS_DOMAIN_ID` collision — 30 boards on domain 0 merge into one graph | `export ROS_DOMAIN_ID=$(cat ~/.ros_domain_id)` and `export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` in **every** terminal | Re-run preflight, which assigns one |
| `Failed to map buffer of size ...` / `err 1002` at startup | 1 | Too many model components on one NPU | Use the launch file's defaults (`htp_device_ids:="[1,1,0,0]"`) | `./workshop/scripts/fastforward.sh 4` |
| `graph returned N output tensors, header declares M` | 1 | A component silently failed to map and returned a broken handle | Split across both NPUs — see Module 4 | `./workshop/scripts/fastforward.sh 4` |

## Module 2 — ROS 2 introspection

| Symptom (verbatim) | CP | Root cause | Fix | Escape hatch |
| --- | --- | --- | --- | --- |
| `ros2 topic pub` runs but the task never changes | 2 | Wrong topic name, or a domain mismatch between terminals | Topic is `/qrb_ros_vla/task`; check `ROS_DOMAIN_ID` matches | `./workshop/scripts/fastforward.sh 2` |
| `task set: '...' (N real tokens, 0 state dims)` | 2 | Expected before any state arrives; state dims appear once `~/state` is published | None — the synthetic publisher and replay both publish state | — |
| `~/state received but no 'tokenizer_model' is set` | 2 | Node started without a tokenizer, so state cannot be encoded | Restart via the launch file, which sets it | `./workshop/scripts/fastforward.sh 1` |

## Module 3 — real robot data

| Symptom (verbatim) | CP | Root cause | Fix | Escape hatch |
| --- | --- | --- | --- | --- |
| `ModuleNotFoundError: No module named 'pyarrow'` | 3 | Reading the parquet live; there is no arm64 apt package | Use the `.npz`: `--episode artifacts/libero_ep0.npz` (pre-work step 5 builds it) | `./workshop/scripts/fastforward.sh 3` |
| `ModuleNotFoundError: No module named 'PIL'` | 3 | Image decoder missing | `sudo apt-get install -y python3-pil` | — |
| `missing artifacts/libero_ep0.npz` | 3 | Pre-work step 5 not done | `./scripts/fetch-libero-episode.sh /tmp/libero && python3 scripts/prepare-libero-npz.py --max-frames 200` | Copy from USB |
| `no chunks received - is the VLA node running?` | 3 | Node down, or the synthetic publisher is competing for the camera topics | Stop `synthetic_publisher.py`; confirm CP-1 | `./workshop/scripts/fastforward.sh 3` |
| Replay prints `no ALL row` | 3 | Replay exited early | Read the log above the summary | `./workshop/scripts/fastforward.sh 3` |
| `normalized MAE` much worse than ~0.15 | 3 | State normalization or a wrong episode/stats pairing | Confirm the `.npz` was built from the same dataset's `stats.json` | `./workshop/scripts/fastforward.sh 3` |

## Module 4 — dual-NPU speedup

| Symptom (verbatim) | CP | Root cause | Fix | Escape hatch |
| --- | --- | --- | --- | --- |
| Probe prints `hardware devices : 1` | 4 | Only one NPU visible to QNN | Reboot and re-run | Board can still do Modules 1–3 and 5; it just will not show the speedup |
| `[FAIL] /dev/fastrpc-cdsp1 present` | 4 | Second NPU not enumerated | Reboot | As above |
| `dlopen(libQnnHtp.so) failed` | 4 | QAIRT not installed | `sudo apt-get install -y qairt-libs qairt-tools qairt-headers` | — |
| `ctx create/free` still large with `--devices "1,1,0,0"` | 4 | A component is not resident | Confirm `--resident` lists all four (the default) | `./workshop/scripts/fastforward.sh 4` |
| Dual-NPU run is no faster | 4 | The node is still running and holding both NPUs | Stop the node before benchmarking | `./workshop/scripts/fastforward.sh 4` |
| `Failed to set powerConfig with error 0x32cb` | 4 | Burst mode rejected on device 1 | Benign — it warns and continues at default performance | — |
| Chunk latency > 2000 ms on the dual-NPU run | 4 | Thermal throttling, or another heavy process | `cat /sys/class/thermal/thermal_zone*/temp`; wait 60 s and retry | Reboot |

## Module 5 — the three-call API

| Symptom (verbatim) | CP | Root cause | Fix | Escape hatch |
| --- | --- | --- | --- | --- |
| `undefined reference to ... QrbInferenceManager::QrbInferenceManager(...)` | 5 | Include order — you compiled against the older apt header | Put `-I$PWD/ros2_ws/install/qrb_inference_manager/include` **first**, before any `/opt/ros` include | `./workshop/scripts/fastforward.sh 5` |
| `error while loading shared libraries: libqrb_inference_manager.so.0` | 5 | Missing rpath | Add `-Wl,-rpath,$PWD/ros2_ws/install/qrb_inference_manager/lib` | `./workshop/scripts/fastforward.sh 5` |
| `qnn-net-run: command not found` | 5 | `qairt-tools` missing | `sudo apt-get install -y qairt-tools` | — |
| `Could not open context binary` | 5 | Wrong bundle path | `ls artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075/` | — |
| `minimal_npu` hangs or is killed | 5 | The node still holds the NPUs | Stop the node first | `./workshop/scripts/fastforward.sh 5` |

## Modules 6–7 (stretch)

| Symptom (verbatim) | CP | Root cause | Fix | Escape hatch |
| --- | --- | --- | --- | --- |
| Accuracy unchanged with fewer denoise steps | 6 | The node was not restarted with the new `denoise_steps` | Relaunch with `denoise_steps:=N`; the benchmark's `--steps` does not affect a running node | `./workshop/scripts/fastforward.sh 6` |
| `FAIL token_emb: prefix_emb max_abs=68.65` | 7 | `qnn-net-run` parsed int32 tokens as float32 and zeroed them | Ensure `--use_native_input_files` is passed (the committed script does) | `./workshop/scripts/fastforward.sh 7` |
| Verification hangs for minutes | 7 | Normal — it replays 45 tensors through the NPU (~3 min) | Wait | `./workshop/scripts/fastforward.sh 7` |

## Module 8 (stretch) — closed-loop simulator

| Symptom (verbatim) | CP | Root cause | Fix | Escape hatch |
| --- | --- | --- | --- | --- |
| `ModuleNotFoundError: No module named 'libero'` | 8 | The LIBERO env file was not sourced, so `PYTHONPATH` lacks the repo | `source ~/libero/libero-env.sh` | `./workshop/scripts/fastforward.sh 8` |
| Hangs silently at `import libero.libero` with no output | 8 | `LIBERO_CONFIG_PATH` unset, so LIBERO prompts on stdin for asset paths | `source ~/libero/libero-env.sh`; the installer writes the config it wants | `./workshop/scripts/fastforward.sh 8` |
| `AttributeError: 'MjData' object has no attribute 'qM'` | 8 | mujoco newer than 3.2.7; 3.3.0 renamed `qM` to `M` and robosuite 1.4.0 still uses the old name | `uv pip install --python ~/libero/venv/bin/python 'mujoco==3.2.7'` | `./workshop/scripts/fastforward.sh 8` |
| `success 0/2 = 0.0%` — the loop runs but never completes | 8 | Observation path wrong: images not rotated 180°, or the 8-D state in the wrong order. Neither errors | Re-run `bench/verify_libero_contract.py`; it compares against the recorded episode and prints the required orientation | `./workshop/scripts/fastforward.sh 8` |
| `chunk timeout after 8s` repeatedly | 8 | The VLA node is not running, or another process holds the NPUs | `pgrep -f vla_node` — if empty, relaunch it; if two, `pkill -f vla_node` and start one | `./workshop/scripts/fastforward.sh 8` |
| `stale chunks:` a non-zero number | 8 | Two clients are publishing camera frames, so chunks get matched to the wrong observation | `pkill -f synthetic_publisher.py` — the simulator drives the cameras itself | `./workshop/scripts/fastforward.sh 8` |
| `Unrecognized image encoding [nv12]` | 8 | Something is publishing a non-`rgb8` image; the node drops it and waits forever | Publish `rgb8`; it is the only zero-copy encoding the node accepts | `./workshop/scripts/fastforward.sh 8` |
| `'MjSim' object has no attribute 'model'` | 8 | A simulator handle cached from before `reset()`; `hard_reset` rebuilds `MjSim` | Re-fetch `env.env.sim` after every `reset()` | `./workshop/scripts/fastforward.sh 8` |
| Episodes take far longer than ~50 s each | 8 | Something else is loading the CPU — rendering is software here and wants ~2 cores | `uptime`; stop other work. Thermals are not the usual cause (we measured 48→53 °C with no throttling) | `./workshop/scripts/fastforward.sh 8` |

## Not used in this workshop

| Symptom (verbatim) | Note |
| --- | --- |
| `OGRE EXCEPTION(3:RenderingAPIException): OpenGL 3.3 is not supported` | Gazebo cannot render camera sensors on this board: Ogre2 requires OpenGL 3.3 **core** and the Adreno driver exposes only OpenGL ES, while forcing Mesa llvmpipe segfaults inside `Ogre2RenderEngine::LoadImpl`. This does **not** apply to Module 8 — LIBERO's MuJoCo renderer is satisfied by the OpenGL 4.5 **compatibility** profile the same llvmpipe provides, which is why the closed-loop module runs on the board with no laptop. |

## Nuclear options

| Situation | Action |
| --- | --- |
| Board is confused, state unclear | Reboot. It takes ~24 s — budget 60 s. A cheap, legitimate fix. |
| Workspace is in a bad state | `rm -rf ros2_ws/build ros2_ws/install && ./workshop/scripts/fastforward.sh 1` |
| Stray processes holding the NPUs | `pkill -f vla_node; pkill -f synthetic_publisher` |
| Hopelessly behind | `./workshop/scripts/fastforward.sh N` for the current module — cumulative and idempotent. |
| Board is genuinely broken | Pair with a neighbour. Hand the board to a helper to fix out-of-band. |
