#!/usr/bin/env bash
# Dump the authoritative graph I/O order out of each pi0.5 QNN context binary.
#
# Why this exists: QrbInferenceManager::inference_execute() takes ONE flat byte
# buffer and splits it across the graph's inputs in *graph order*. That order is
# baked into the .bin at compile time and is NOT the order in metadata.json, nor
# the order in the upstream python get_input_spec_static(). Guessing it produces
# silently wrong numbers with no error. So we read it from the binary itself.
set -euo pipefail

BUNDLE="${1:-artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075}"
OUT="${2:-bench/graph-io}"

command -v qnn-context-binary-utility >/dev/null || {
  echo "ERROR: qnn-context-binary-utility not found. Install qairt-tools." >&2; exit 1; }

mkdir -p "$OUT"
for m in vision_encoder token_emb backbone action_expert; do
  [[ -f "$BUNDLE/$m.bin" ]] || { echo "ERROR: missing $BUNDLE/$m.bin" >&2; exit 1; }
  qnn-context-binary-utility \
    --context_binary="$BUNDLE/$m.bin" \
    --json_file="$OUT/$m.json" >"$OUT/$m.log" 2>&1
  echo "  wrote $OUT/$m.json"
done
echo "done: dumped graph I/O for 4 components into $OUT"
