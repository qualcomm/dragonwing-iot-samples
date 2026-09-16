#!/usr/bin/env bash
# Fetch one LIBERO episode plus dataset metadata for demo/libero_replay.py.
#
# physical-intelligence/libero is the LeRobot-format LIBERO dataset Pi0.5 was
# calibrated and evaluated on: Franka Panda, 10 Hz, 256x256 base + wrist cameras,
# 8-D state, 7-D action. It stores one parquet per episode, so a single episode is
# ~28 MB instead of the full ~50 GB. No authentication required.
#
# meta/stats.json matters as much as the episode: it carries the mean/std used for
# the MEAN_STD normalization that lerobot/pi05_libero's policy_preprocessor.json
# declares for STATE and ACTION. Without it, prompts and predicted actions are in
# the wrong units.
set -euo pipefail

DEST="${1:-/tmp/libero}"
EPISODE="${EPISODE:-000000}"
BASE="https://huggingface.co/datasets/physical-intelligence/libero/resolve/main"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PYTHON="${PYTHON:-python3}"


mkdir -p "$DEST"

for f in meta/info.json meta/tasks.jsonl meta/stats.json; do
  out="$DEST/$(basename "$f")"
  [[ -s "$out" ]] && { echo "have $(basename "$f")"; continue; }
  echo "fetching $f"
  curl -fSL --retry 3 -o "$out" "$BASE/$f"
done

EP_OUT="$DEST/ep${EPISODE}.parquet"
if [[ -s "$EP_OUT" ]]; then
  echo "have $(basename "$EP_OUT")"
else
  echo "fetching episode ${EPISODE}"
  curl -fSL --retry 3 -o "$EP_OUT" \
    "$BASE/data/chunk-000/episode_${EPISODE}.parquet"
fi

# Symlink so the demo's default --episode path works for any episode choice.
ln -sf "$(basename "$EP_OUT")" "$DEST/ep0.parquet"

"$PYTHON" "$SCRIPT_DIR/prepare-libero-npz.py" \
  --episode "$DEST/ep0.parquet" \
  --stats "$DEST/stats.json" \
  --tasks "$DEST/tasks.jsonl" \
  --out "$REPO_ROOT/artifacts/libero_ep0.npz"

echo "done: $DEST ($(du -sh "$DEST" | cut -f1)); episode ${EPISODE} ready; artifacts/libero_ep0.npz ready"
