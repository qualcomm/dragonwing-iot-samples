#!/usr/bin/env bash
# check.sh N - assert the state checkpoint CP-N. Prints CP-N PASS or CP-N FAIL.
#
# Every assertion is a literal string, an exit code, a topic name, or a number in
# a range. Never "it should work". Each checkpoint is exactly what
# fastforward.sh N guarantees to produce, so the two files must stay in step.
#
# Checkpoint map (module numbering as in README.md):
#   0 preflight              4 dual-NPU speedup
#   1 VLA running            5 minimal_npu (3-call API)
#   2 task changed live      6 denoise-step tuning      [STRETCH]
#   3 scored vs human        7 bitwise verification     [STRETCH]
#                            8 closed-loop task success [STRETCH]
set -uo pipefail

CP="${1:-}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="$REPO_ROOT/ros2_ws"

die() { echo "CP-$CP FAIL: $1"; exit 1; }
ok() { echo "CP-$CP PASS: $1"; exit 0; }

[[ -n "$CP" ]] || { echo "usage: check.sh <0-8>"; exit 2; }

# shellcheck source=/dev/null
[[ -f /opt/ros/jazzy/setup.bash ]] && { set +u; source /opt/ros/jazzy/setup.bash; set -u; }
# shellcheck source=/dev/null
[[ -f "$WS/install/setup.bash" ]] && { set +u; source "$WS/install/setup.bash"; set -u; }

# Is the node up and advertising its output topic?
node_publishing() {
  command -v ros2 >/dev/null || return 1
  timeout 20 ros2 topic list 2>/dev/null | grep -q "/qrb_ros_vla/action_chunk"
}

# Pull one mean value out of a pi05_bench --json file.
bench_mean() { # file, key
  grep -o "\"$2\": {\"mean\": [0-9.]*" "$1" 2>/dev/null | grep -o '[0-9.]*$' | cut -d. -f1
}

case "$CP" in
  0)  # Module 0: the board is workshop-ready.
    "$REPO_ROOT/workshop/scripts/preflight.sh" >/dev/null 2>&1 \
      || die "preflight is not all-green; run workshop/scripts/preflight.sh and read the [FAIL] lines"
    ok "board passes preflight"
    ;;

  1)  # Module 1: the VLA is running and producing action chunks.
    node_publishing \
      || die "/qrb_ros_vla/action_chunk not advertised; is 'ros2 launch qrb_ros_vla vla.launch.py' running in Terminal A?"
    # Prove chunks are actually flowing, not just that the topic exists.
    OUT=$(timeout 45 ros2 topic echo /qrb_ros_vla/action_chunk --once 2>/dev/null)
    [[ -n "$OUT" ]] \
      || die "topic exists but no chunk arrived in 45 s; is the camera publisher running in Terminal B?"
    echo "$OUT" | grep -q "chunk_size: 50" \
      || die "chunk_size is not 50; the model bundle may be wrong"
    ok "node is publishing action chunks"
    ;;

  2)  # Module 2: the node accepted a new instruction and kept working.
    node_publishing || die "node is not publishing; re-run Module 1"
    OUT=$(timeout 45 ros2 topic echo /qrb_ros_vla/action_chunk --once 2>/dev/null)
    [[ -n "$OUT" ]] || die "no chunk arrived in 45 s; is the camera publisher still running?"
    TASK=$(echo "$OUT" | sed -n 's/^task: //p' | tr -d "'\"")
    [[ -n "$TASK" ]] || die "chunk carries no task string"
    ok "node accepted a task ('${TASK:0:40}...') and is still publishing chunks"
    ;;

  3)  # Module 3: predictions scored against a real human demonstration.
    L=/tmp/cp3-replay.log
    [[ -f "$L" ]] || die "no replay output at $L; re-run Module 3 (or ./fastforward.sh 3)"
    NMAE=$(awk '/^ *ALL/ {print $3}' "$L")
    [[ -n "$NMAE" ]] || die "no ALL row in $L; the replay did not complete"
    # Predicting the dataset mean scores 1.0 by construction. Under 0.5 means the
    # model is clearly tracking the demonstrator; we measure 0.12-0.15.
    awk -v v="$NMAE" 'BEGIN { exit !(v > 0 && v < 0.5) }' \
      || die "normalized MAE $NMAE is not in (0, 0.5); predictions are not tracking the demo"
    ok "normalized MAE $NMAE against the human demonstration"
    ;;

  4)  # Module 4: the dual-NPU speedup, with no weight swapping.
    J=/tmp/cp4-latency.json
    [[ -f "$J" ]] || die "no benchmark output at $J; re-run Module 4"
    MS=$(bench_mean "$J" total_per_chunk)
    CTX=$(bench_mean "$J" context_create_free)
    [[ -n "$MS" ]] || die "cannot parse latency from $J"
    (( MS < 1600 )) || die "chunk latency ${MS} ms is not under 1600 ms; check --devices is 1,1,0,0"
    (( ${CTX:-999} < 5 )) || die "context create/free is ${CTX} ms, expected ~0; weights are still swapping"
    ok "dual-NPU action chunk in ${MS} ms with zero context paging"
    ;;

  5)  # Module 5: the attendee built and ran the 3-call example themselves.
    L=/tmp/cp5-minimal.log
    [[ -x /tmp/minimal_npu ]] || die "/tmp/minimal_npu not built; re-run Module 5's g++ line"
    [[ -f "$L" ]] || die "no output at $L; run minimal_npu and tee it there, or ./fastforward.sh 5"
    grep -q "img_embed" "$L" || die "minimal_npu did not report an img_embed tensor; read $L"
    grep -q "bytes=2097152" "$L" \
      || die "img_embed is the wrong size; expected 2097152 bytes (1x256x2048 float32)"
    ok "minimal_npu built and ran a model on the NPU"
    ;;

  6)  # Module 6 [STRETCH]: denoise-step tuning was measured.
    L=/tmp/cp6-steps.log
    [[ -f "$L" ]] || die "no sweep output at $L; re-run Module 6"
    N=$(grep -c "chunks/s" "$L")
    (( N >= 2 )) || die "found $N benchmark runs in $L, expected at least 2 denoise-step settings"
    ok "measured $N denoise-step settings"
    ;;

  7)  # Module 7 [STRETCH]: bitwise agreement with Qualcomm's own runner.
    L=/tmp/cp7-verify.log
    [[ -f "$L" ]] || die "no verification log at $L; re-run Module 7"
    grep -q "VERIFIED" "$L" || die "verification did not print VERIFIED; read $L"
    N=$(grep -c "^PASS" "$L")
    [[ "$N" == "4" ]] || die "only $N of 4 components passed; read $L"
    ok "all 4 components bitwise identical to qnn-net-run"
    ;;

  8)  # Module 8 [STRETCH]: the policy drove a simulator to complete a real task.
    L=/tmp/cp8-closedloop.log
    [[ -f "$L" ]] || die "no closed-loop output at $L; re-run Module 8 (or ./fastforward.sh 8)"
    # "success 2/2 = 100.0%  (95% CI ...)"
    read -r K N < <(sed -n 's#^success \([0-9]\+\)/\([0-9]\+\) .*#\1 \2#p' "$L" | tail -1)
    [[ -n "${N:-}" ]] || die "no 'success K/N' line in $L; the rollout did not finish"
    (( N >= 1 )) || die "no episodes completed; read $L"
    # Proof-of-NPU is per chunk here, not asserted once: a CPU fallback would report a
    # different backend and be roughly an order of magnitude slower.
    grep -q "libQnnHtp.so" "$L" \
      || die "no libQnnHtp.so in the backends line of $L; inference may have fallen back to CPU"
    # A late chunk matched to a newer observation would silently apply a stale plan, so
    # the harness counts those and we require none.
    grep -q "stale chunks: 0" "$L" \
      || die "the harness discarded mismatched chunks; the loop was not synchronized (read $L)"
    # A broken observation path (wrong image orientation, wrong state layout, missed
    # un-normalization) still runs and still renders -- it just never completes the task.
    # So the assertion is that at least one episode actually succeeded. Measured on this
    # hardware: 5/5 on each of libero_10 tasks 0-4, so 1 success in 2 episodes has margin.
    (( K >= 1 )) || die "$K of $N episodes succeeded; the loop ran but completed no task, which usually means the observation path is wrong (read $L)"
    ok "$K/$N closed-loop episodes completed the task on libQnnHtp.so"
    ;;

  *) echo "unknown checkpoint CP-$CP (valid: 0-8)"; exit 2 ;;
esac
