#!/usr/bin/env bash
# Run one hobby-arm ablation condition across a LIBERO suite and record every episode.
#
# Each condition reproduces one physical limitation of the VUPN2355 arm inside the simulator
# and is scored against the measured 86/100 baseline (bench/libero-closed-loop-pooled.json).
# See bench/arm_embodiment.py for what each mode does and why.
#
# RUN THE INSTRUMENT CHECK FIRST. bench/verify_arm_embodiment.py takes under a second and
# confirms the perturbations are the perturbations they claim to be; this script refuses to
# start without it, because a table of success rates produced by a broken quantizer looks
# exactly like a table produced by a working one.
#
# The VLA node must ALREADY be running and must own the NPUs exclusively (docs/DESIGN.md
# section 11) -- two processes mapping the 2.9 GB bundle contend for the same CDSP mapping
# budget and invalidate both correctness and latency. Like scripts/run-libero-sweep.sh, this
# script deliberately does not start or stop the node.
#
# Usage:
#   scripts/run-arm-ablation.sh <mode> [episodes_per_task] [out.jsonl]
#   scripts/run-arm-ablation.sh none 5                    # the sanity gate
#   scripts/run-arm-ablation.sh state-const 5             # E2
#   FRAME_RATE=330 scripts/run-arm-ablation.sh quant 5    # E5 at a finer PWM quantum
set -euo pipefail

MODE="${1:?usage: run-arm-ablation.sh <mode> [episodes] [out.jsonl]}"
EPISODES="${2:-5}"
OUT="${3:-bench/libero-arm-ablation-${MODE}.jsonl}"
SUITE="${SUITE:-libero_10}"
TASKS="${TASKS:-0 1 2 3 4 5 6 7 8 9}"
MAX_STEPS="${MAX_STEPS:-520}"
HORIZON="${HORIZON:-10}"
INIT_START="${INIT_START:-0}"
SPEC="${SPEC:-bench/arm-spec-measured.json}"
PY="${PY:-${LIBERO_VENV_PYTHON:-python3}}"

: "${LIBERO_CONFIG_PATH:?set LIBERO_CONFIG_PATH to a dir containing config.yaml}"
: "${MUJOCO_GL:?set MUJOCO_GL=osmesa}"

if ! pgrep -x vla_node >/dev/null; then
  echo "ERROR: vla_node is not running; start it first (it must own the NPUs)" >&2
  exit 1
fi

echo "==> instrument check"
"$PY" -u bench/verify_arm_embodiment.py >/dev/null \
  || { echo "ERROR: bench/verify_arm_embodiment.py failed; the ablation would measure nothing" >&2; exit 1; }
echo "    arm model verified"

# A second process on the NPU inflates latency and can trip the err 1002 mapping path, so the
# load at the start of a 45-minute condition is worth recording next to its result.
echo "==> loadavg at start: $(cut -d' ' -f1-3 /proc/loadavg)"

FRAME_ARGS=()
if [[ -n "${FRAME_RATE:-}" ]]; then
  FRAME_ARGS=(--arm-frame-rate "$FRAME_RATE")
  echo "==> PWM frame rate override: ${FRAME_RATE} Hz"
fi

echo "ablation: mode=$MODE suite=$SUITE tasks=[$TASKS] episodes=$EPISODES"
echo "          horizon=$HORIZON max_steps=$MAX_STEPS inits $INIT_START..$((INIT_START + EPISODES - 1))"
echo "out:      $OUT"

for t in $TASKS; do
  echo "--- task $t ---"
  # -u because stdout is block-buffered through the pipe below, and without it a long
  # condition shows no per-episode progress until each task finishes.
  "$PY" -u demo/libero_closed_loop.py \
    --suite "$SUITE" --task-index "$t" \
    --episodes "$EPISODES" --init-start "$INIT_START" \
    --replan-horizon "$HORIZON" --max-steps "$MAX_STEPS" \
    --arm-model "$SPEC" --ablate "$MODE" "${FRAME_ARGS[@]}" \
    --jsonl "$OUT" 2>&1 \
    | grep --line-buffered -viE 'robosuite WARNING|Gym has been|Please upgrade|migration guide|^See the migration|Warning\]: datasets' \
    || echo "task $t exited non-zero; continuing"
done

n=$(wc -l < "$OUT")
echo "ablation '$MODE' done: $n episode records in $OUT"
echo "loadavg at end: $(cut -d' ' -f1-3 /proc/loadavg)"
echo "next: bench/analyze_arm_ablation.py"
