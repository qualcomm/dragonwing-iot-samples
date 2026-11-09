#!/usr/bin/env bash
# Sweep a LIBERO suite closed-loop through Pi0.5 on the NPUs and record every episode.
#
# Breadth over depth on purpose: N episodes on each of the suite's 10 tasks says more about
# the policy than 10N episodes on one task, because LIBERO's tasks differ in difficulty and
# a single-task rate hides that entirely.
#
# The VLA node must ALREADY be running and must own the NPUs exclusively (docs/DESIGN.md
# section 10) -- two processes mapping the 2.9 GB bundle contend for the same CDSP mapping
# budget and invalidate both correctness and latency. This script deliberately does not
# start or stop the node: whoever runs it owns that decision.
#
# Usage:
#   scripts/run-libero-sweep.sh [suite] [episodes_per_task] [out.jsonl]
set -euo pipefail

SUITE="${1:-libero_10}"
EPISODES="${2:-5}"
OUT="${3:-bench/libero-closed-loop-sweep.jsonl}"
TASKS="${TASKS:-0 1 2 3 4 5 6 7 8 9}"
MAX_STEPS="${MAX_STEPS:-520}"
HORIZON="${HORIZON:-10}"
# Which of the 50 initial states LIBERO ships per task to start from. Sweeping a second
# band (e.g. INIT_START=20) is how you test whether the first few initial states are
# unrepresentatively easy -- which a single band cannot tell you.
INIT_START="${INIT_START:-0}"
# The env file scripts/install-libero.sh writes exports LIBERO_VENV_PYTHON; fall back to
# python3 rather than a scratch path that will not survive a reboot.
PY="${PY:-${LIBERO_VENV_PYTHON:-python3}}"

: "${LIBERO_CONFIG_PATH:?set LIBERO_CONFIG_PATH to a dir containing config.yaml}"
: "${MUJOCO_GL:?set MUJOCO_GL=osmesa}"

if ! pgrep -x vla_node >/dev/null; then
  echo "ERROR: vla_node is not running; start it first (it must own the NPUs)" >&2
  exit 1
fi

echo "sweep: suite=$SUITE tasks=[$TASKS] episodes=$EPISODES horizon=$HORIZON max_steps=$MAX_STEPS"
echo "       init states $INIT_START..$((INIT_START + EPISODES - 1))"
echo "out:   $OUT"

for t in $TASKS; do
  echo "--- task $t ---"
  # -u because stdout is block-buffered through the pipe below, and without it a 45-minute
  # sweep shows no per-episode progress at all until each task finishes.
  "$PY" -u demo/libero_closed_loop.py \
    --suite "$SUITE" --task-index "$t" \
    --episodes "$EPISODES" --init-start "$INIT_START" \
    --replan-horizon "$HORIZON" --max-steps "$MAX_STEPS" \
    --jsonl "$OUT" 2>&1 \
    | grep --line-buffered -viE 'robosuite WARNING|Gym has been|Please upgrade|migration guide|^See the migration|Warning\]: datasets' \
    || echo "task $t exited non-zero; continuing"
done

n=$(wc -l < "$OUT")
echo "sweep done: $n episode records in $OUT"
