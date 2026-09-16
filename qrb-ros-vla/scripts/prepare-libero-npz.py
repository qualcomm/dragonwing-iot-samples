#!/usr/bin/env python3
"""Convert a LIBERO parquet episode into a dependency-free .npz for the workshop.

Reading LIBERO's parquet needs pyarrow, which has no apt package on arm64 Ubuntu
24.04 -- meaning a live `pip install` in the middle of a workshop, which is
exactly what we forbid. So the conversion happens once during pre-work (or on the
instructor's mirror), and the live path loads a plain .npz with numpy only.

Images are decoded to raw uint8 arrays here too, so the live path needs no JPEG
decoder either.

Usage:
  scripts/prepare-libero-npz.py --episode /tmp/libero/ep0.parquet \
      --out artifacts/libero_ep0.npz
"""

from __future__ import annotations

import argparse
import io
import json
import sys
from pathlib import Path

import numpy as np


def decode(cell) -> np.ndarray:
    raw = cell["bytes"] if isinstance(cell, dict) else cell
    from PIL import Image

    return np.asarray(Image.open(io.BytesIO(raw)).convert("RGB"))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--episode", default="/tmp/libero/ep0.parquet")
    ap.add_argument("--stats", default="/tmp/libero/stats.json")
    ap.add_argument("--tasks", default="/tmp/libero/tasks.jsonl")
    ap.add_argument("--out", default="artifacts/libero_ep0.npz")
    ap.add_argument("--max-frames", type=int, default=0, help="0 = all frames")
    args = ap.parse_args()

    try:
        import pyarrow.parquet as pq
    except ImportError:
        print(
            "ERROR: pyarrow is required for THIS script only (run it during pre-work,\n"
            "       on a laptop or in a venv: pip install pyarrow pillow).\n"
            "       The workshop's live path needs only numpy.",
            file=sys.stderr,
        )
        return 1

    for p in (args.episode, args.stats, args.tasks):
        if not Path(p).exists():
            print(f"ERROR: missing {p}; run scripts/fetch-libero-episode.sh", file=sys.stderr)
            return 1

    data = pq.read_table(args.episode).to_pydict()
    stats = json.loads(Path(args.stats).read_text())
    tasks = {
        json.loads(line)["task_index"]: json.loads(line)["task"]
        for line in Path(args.tasks).read_text().splitlines()
        if line.strip()
    }

    n = len(data["state"])
    if args.max_frames:
        n = min(n, args.max_frames)

    base = np.stack([decode(data["image"][i]) for i in range(n)])
    wrist = np.stack([decode(data["wrist_image"][i]) for i in range(n)])

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        out,
        image=base,
        wrist_image=wrist,
        state=np.asarray(data["state"][:n], dtype=np.float32),
        actions=np.asarray(data["actions"][:n], dtype=np.float32),
        state_mean=np.asarray(stats["state"]["mean"], dtype=np.float32),
        state_std=np.asarray(stats["state"]["std"], dtype=np.float32),
        action_mean=np.asarray(stats["actions"]["mean"], dtype=np.float32),
        action_std=np.asarray(stats["actions"]["std"], dtype=np.float32),
        task=np.asarray(tasks.get(int(data["task_index"][0]), "complete the task")),
        fps=np.asarray(10.0, dtype=np.float32),
    )
    mb = out.stat().st_size / 1024 / 1024
    print(f"wrote {out} ({mb:.1f} MB, {n} frames, images {base.shape[1]}x{base.shape[2]})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
