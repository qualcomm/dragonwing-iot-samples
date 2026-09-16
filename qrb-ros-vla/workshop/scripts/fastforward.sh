#!/usr/bin/env bash
# fastforward.sh N - reach exactly the state that check.sh N asserts.
#
# The escape hatch for every module. An attendee who is lost, broken, or arrived
# late runs ONE command and rejoins. Levels are cumulative and idempotent, so
# `fastforward.sh 4` from a bare board does 1-4 in order and re-running is safe.
#
# Works offline, provided PREWORK.md was completed (or the USB stick was copied
# over). Nothing here downloads except the LIBERO episode, which pre-work already
# converted to artifacts/libero_ep0.npz.
#
# Unlike the attendee path, this drives the node itself rather than assuming two
# terminals are open -- so it can rebuild the room's state unattended.
set -euo pipefail

TARGET="${1:-}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="$REPO_ROOT/ros2_ws"
BUNDLE="$REPO_ROOT/artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075"
TOKENIZER="$REPO_ROOT/artifacts/paligemma_tokenizer.model"
BENCH="$WS/install/qrb_ros_vla/lib/qrb_ros_vla/pi05_bench"
EPISODE="$REPO_ROOT/artifacts/libero_ep0.npz"

[[ -n "$TARGET" ]] || { echo "usage: fastforward.sh <0-8>"; exit 2; }

say() { printf '\n>>> %s\n' "$1"; }

setup_ros() {
  # shellcheck source=/dev/null
  set +u; source /opt/ros/jazzy/setup.bash; set -u
  if [[ -f "$WS/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    set +u; source "$WS/install/setup.bash"; set -u
  fi
}

ensure_built() {
  if [[ ! -x "$BENCH" ]]; then
    say "workspace not built yet - building (~60 s)"
    setup_ros
    (cd "$WS" && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release)
    setup_ros
  fi
}

stop_node() { pkill -f "qrb_ros_vla/vla_node" 2>/dev/null || true; sleep 1; }
stop_pub()  { pkill -f "synthetic_publisher.py" 2>/dev/null || true; sleep 1; }

# Start the node and wait until it is actually producing chunks.
start_node_and_publisher() {
  setup_ros
  stop_node; stop_pub
  : > /tmp/ff-node.log
  QNN_HTP_BURST=1 nohup ros2 run qrb_ros_vla vla_node --ros-args \
    -p bundle_dir:="$BUNDLE" \
    -p tokenizer_model:="$TOKENIZER" \
    -p action_dof:=7 >>/tmp/ff-node.log 2>&1 &
  echo "  node starting (maps 2.9 GB of weights) ..."
  : > /tmp/ff-pub.log
  nohup python3 "$REPO_ROOT/demo/synthetic_publisher.py" --rate 1.0 >>/tmp/ff-pub.log 2>&1 &
  for _ in $(seq 45); do
    grep -q "chunk:" /tmp/ff-node.log 2>/dev/null && { echo "  chunks flowing"; return 0; }
    sleep 2
  done
  echo "  WARNING: no chunks after 90 s; see /tmp/ff-node.log" >&2
}

ff_1() {
  say "Module 1: get the VLA running and publishing action chunks"
  ensure_built
  start_node_and_publisher
}

ff_2() {
  say "Module 2: send a new task over ROS and confirm it takes effect"
  setup_ros
  grep -q "chunk:" /tmp/ff-node.log 2>/dev/null || start_node_and_publisher
  ros2 topic pub --once /qrb_ros_vla/task std_msgs/String \
    "{data: 'put the red block in the drawer'}" >/dev/null 2>&1 || true
  sleep 3
}

ff_3() {
  say "Module 3: replay a human demonstration and score it (~40 s)"
  ensure_built
  setup_ros
  grep -q "chunk:" /tmp/ff-node.log 2>/dev/null || start_node_and_publisher
  # The replay drives the cameras itself; the synthetic publisher would fight it.
  stop_pub
  [[ -f "$EPISODE" ]] || {
    say "episode missing - preparing it (needs pyarrow; normally done in pre-work)"
    "$REPO_ROOT/scripts/fetch-libero-episode.sh" /tmp/libero
    python3 "$REPO_ROOT/scripts/prepare-libero-npz.py" --max-frames 200
  }
  python3 "$REPO_ROOT/demo/libero_replay.py" --episode "$EPISODE" \
    --steps 8 --stride 20 --horizon 10 2>&1 | tee /tmp/cp3-replay.log
}

ff_4() {
  say "Module 4: benchmark across both NPUs"
  ensure_built
  setup_ros
  # Free the NPUs: the node holds all four contexts resident.
  stop_node; stop_pub
  QNN_HTP_BURST=1 "$BENCH" --bundle "$BUNDLE" \
    --iters 5 --warmup 2 --devices "1,1,0,0" \
    --json /tmp/cp4-latency.json >/tmp/cp4-bench.log 2>&1
  grep -E "TOTAL|chunks/s" /tmp/cp4-bench.log || true
}

ff_5() {
  say "Module 5: build and run the 3-call minimal example"
  ensure_built
  g++ -std=c++17 -O2 \
    -I"$WS/install/qrb_inference_manager/include" \
    "$REPO_ROOT/workshop/examples/minimal_npu.cpp" \
    -L"$WS/install/qrb_inference_manager/lib" -lqrb_inference_manager \
    -Wl,-rpath,"$WS/install/qrb_inference_manager/lib" \
    -o /tmp/minimal_npu
  stop_node
  QNN_HTP_BURST=1 /tmp/minimal_npu "$BUNDLE/vision_encoder.bin" 2>&1 | tee /tmp/cp5-minimal.log
}

ff_6() {
  say "Module 6 [STRETCH]: sweep denoise steps"
  ensure_built
  setup_ros
  stop_node; stop_pub
  : > /tmp/cp6-steps.log
  for S in 4 6 10; do
    echo "--- denoise_steps=$S ---" | tee -a /tmp/cp6-steps.log
    QNN_HTP_BURST=1 "$BENCH" --bundle "$BUNDLE" --iters 3 --warmup 1 --steps "$S" 2>&1 \
      | grep -E "action_expert|TOTAL|chunks/s" | tee -a /tmp/cp6-steps.log
  done
}

ff_7() {
  say "Module 7 [STRETCH]: verify bitwise against qnn-net-run (~3 min)"
  ensure_built
  setup_ros
  stop_node; stop_pub
  rm -rf /tmp/cp7dump
  QNN_HTP_BURST=1 "$BENCH" --bundle "$BUNDLE" \
    --iters 1 --warmup 0 --dump-dir /tmp/cp7dump >/dev/null 2>&1
  python3 "$REPO_ROOT/bench/verify_against_qnn_net_run.py" \
    --dump-dir /tmp/cp7dump --bundle "$BUNDLE" \
    --graph-io "$REPO_ROOT/bench/graph-io" 2>&1 | tee /tmp/cp7-verify.log
}

ff_8() {
  say "Module 8 [STRETCH]: drive a simulator closed-loop and complete a task (~3 min)"
  ensure_built
  setup_ros

  LIBERO_ENV="${LIBERO_ENV:-$HOME/libero/libero-env.sh}"
  [[ -f "$LIBERO_ENV" ]] || {
    echo "  LIBERO is not installed ($LIBERO_ENV missing)." >&2
    echo "  Run scripts/install-libero.sh once; pre-work normally does this." >&2
    return 1
  }
  set +u
  # shellcheck source=/dev/null
  source "$LIBERO_ENV"
  set -u

  # Deliberately a process check, not `grep chunk: /tmp/ff-node.log`. Levels 4-7 stop
  # the node, but the log they left behind still contains "chunk:" from level 1-3 -- so
  # the log-grep idiom used earlier would conclude the node is up when it is not, and
  # every rollout would then time out.
  pgrep -f "qrb_ros_vla/vla_node" >/dev/null || start_node_and_publisher
  # The simulator publishes the camera topics itself; the synthetic publisher would
  # fight it for the node's one-fresh-frame-per-chunk gate.
  stop_pub

  # Two episodes keeps the escape hatch inside a workshop's patience: each is ~50 s of
  # NPU plus rendering. The measured rate on this hardware is 5/5 per task, so CP-8's
  # "at least one success" has margin.
  "${LIBERO_VENV_PYTHON:-python3}" "$REPO_ROOT/demo/libero_closed_loop.py" \
    --suite libero_10 --task-index 0 --episodes 2 \
    --stats-npz "$EPISODE" \
    --replan-horizon 10 --max-steps 520 2>&1 | tee /tmp/cp8-closedloop.log
}

for level in 1 2 3 4 5 6 7 8; do
  (( level <= TARGET )) || continue
  "ff_$level"
done

# Modules 1-3 and 8 need the node up for their checkpoints; 4-7 deliberately stop it to
# free the NPUs. Bring it back so the room is in a runnable state either way. Level 8
# already ensures the node itself, so re-starting it here would waste a 2.9 GB remap.
if (( TARGET >= 4 && TARGET < 8 )); then
  say "restarting the node so you are left with a running VLA"
  start_node_and_publisher
fi

say "fast-forward to CP-$TARGET complete; verifying"
"$REPO_ROOT/workshop/scripts/check.sh" "$TARGET"
