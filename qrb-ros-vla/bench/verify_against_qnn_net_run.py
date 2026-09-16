#!/usr/bin/env python3
"""Prove Pi05Runner's hand-packed graph ordering is correct.

Pi05Runner has to hand-assemble one flat byte buffer per component because
QrbInferenceManager::inference_execute() slices it across graph inputs in
compile-time graph order. Get that order wrong -- and action_expert's 36 KV
inputs really are ordered l0,l1,l10..l17,l2..l9 while backbone emits l0..l17 --
and you get plausible-looking garbage with no error anywhere.

This script closes that loop. It takes the per-stage tensors dumped by
`pi05_bench --dump-dir`, re-splits each flat input buffer using the graph order
read from the context binary, replays it through Qualcomm's own `qnn-net-run`,
and diffs qnn-net-run's outputs against what Pi05Runner produced. Identical
outputs mean the packing is right.

Usage:
  bench/verify_against_qnn_net_run.py [--dump-dir /tmp/pi05dump]
                                      [--bundle artifacts/pi05-...-qcs9075]
                                      [--graph-io bench/graph-io]
                                      [--work /tmp/pi05verify]
"""

from __future__ import annotations

import argparse
import json
import shutil
import struct
import subprocess
import sys
from pathlib import Path

# Artifact defaults are anchored to the repository, not the caller's working directory: these
# scripts are invoked by workshop/scripts/fastforward.sh and by hand from various places, and
# a bare relative default fails with FileNotFoundError depending on where you happen to be.
# An explicitly passed path is still interpreted relative to the cwd, which is what a user
# typing one expects.
_REPO = Path(__file__).resolve().parent.parent


def _repo_path(rel: str) -> str:
    return str(_REPO / rel)


COMPONENTS = ("vision_encoder", "token_emb", "backbone", "action_expert")

DTYPE_BYTES = {
    "QNN_DATATYPE_FLOAT_32": 4,
    "QNN_DATATYPE_INT_32": 4,
    "QNN_DATATYPE_UINT_32": 4,
    "QNN_DATATYPE_FLOAT_16": 2,
}
FLOAT_TYPES = {"QNN_DATATYPE_FLOAT_32"}


def find_key(obj, key, depth=0):
    if depth > 8:
        return None
    if isinstance(obj, dict):
        if key in obj:
            return obj[key]
        for value in obj.values():
            found = find_key(value, key, depth + 1)
            if found is not None:
                return found
    elif isinstance(obj, list):
        for value in obj:
            found = find_key(value, key, depth + 1)
            if found is not None:
                return found
    return None


def graph_io(graph_io_dir: Path, comp: str):
    graphs = find_key(json.loads((graph_io_dir / f"{comp}.json").read_text()), "graphs")
    info = graphs[0].get("info", graphs[0])

    def tensors(which):
        out = []
        for entry in info.get(which) or []:
            t = entry.get("info", entry)
            count = 1
            for d in t["dimensions"]:
                count *= d
            dtype = t["dataType"]
            out.append((t["name"], count, count * DTYPE_BYTES[dtype], dtype))
        return out

    return tensors("graphInputs"), tensors("graphOutputs")


def read_floats(path: Path):
    raw = path.read_bytes()
    return struct.unpack(f"<{len(raw) // 4}f", raw)


def compare(a_path: Path, b_path: Path, dtype: str):
    a, b = a_path.read_bytes(), b_path.read_bytes()
    if len(a) != len(b):
        return None, f"size mismatch {len(a)} vs {len(b)}"
    if a == b:
        return 0.0, "bitwise identical"
    if dtype not in FLOAT_TYPES:
        differing = sum(1 for x, y in zip(a, b) if x != y)
        return None, f"{differing} of {len(a)} bytes differ (non-float tensor)"
    fa, fb = read_floats(a_path), read_floats(b_path)
    max_abs = 0.0
    max_rel = 0.0
    for x, y in zip(fa, fb):
        d = abs(x - y)
        if d > max_abs:
            max_abs = d
        denom = max(abs(x), abs(y))
        if denom > 1e-6:
            r = d / denom
            if r > max_rel:
                max_rel = r
    return max_abs, f"max_abs={max_abs:.6g} max_rel={max_rel:.6g}"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dump-dir", default="/tmp/pi05dump")
    ap.add_argument(
        "--bundle", default=_repo_path("artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075")
    )
    ap.add_argument("--graph-io", default=_repo_path("bench/graph-io"))
    ap.add_argument("--work", default="/tmp/pi05verify")
    args = ap.parse_args()

    dump = Path(args.dump_dir)
    bundle = Path(args.bundle)
    graph_io_dir = Path(args.graph_io)
    work = Path(args.work)

    if not shutil.which("qnn-net-run"):
        print("ERROR: qnn-net-run not on PATH (install qairt-tools)", file=sys.stderr)
        return 1
    if not dump.is_dir():
        print(f"ERROR: {dump} missing; run pi05_bench --dump-dir first", file=sys.stderr)
        return 1

    if work.exists():
        shutil.rmtree(work)
    work.mkdir(parents=True)

    overall_ok = True
    summary = []

    for comp in COMPONENTS:
        in_specs, out_specs = graph_io(graph_io_dir, comp)
        flat_path = dump / f"{comp}_call0_IN.raw"
        if not flat_path.exists():
            print(f"SKIP {comp}: {flat_path.name} not dumped")
            continue

        blob = flat_path.read_bytes()
        expected = sum(s[2] for s in in_specs)
        if len(blob) != expected:
            print(f"FAIL {comp}: dumped input is {len(blob)} B, graph wants {expected} B")
            overall_ok = False
            continue

        comp_work = work / comp
        comp_work.mkdir(parents=True)
        entries = []
        offset = 0
        for name, _count, nbytes, _dtype in in_specs:
            p = comp_work / f"{name}.raw"
            p.write_bytes(blob[offset : offset + nbytes])
            offset += nbytes
            entries.append(f"{name}:={p}")
        (comp_work / "input_list.txt").write_text(" ".join(entries) + "\n")

        out_dir = comp_work / "out"
        cmd = [
            "qnn-net-run",
            "--backend",
            "/usr/lib/libQnnHtp.so",
            "--retrieve_context",
            str(bundle / f"{comp}.bin"),
            "--input_list",
            str(comp_work / "input_list.txt"),
            "--output_dir",
            str(out_dir),
            # Our dumps are already in each tensor's native dtype. Without this,
            # qnn-net-run parses every input file as float32 and casts, which
            # silently turns token_emb's int32 lang_tokens into all-zero padding
            # ids -- and then "disagrees" with a runner that is actually correct.
            "--use_native_input_files",
        ]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if proc.returncode != 0:
            print(f"FAIL {comp}: qnn-net-run exited {proc.returncode}")
            print("  " + (proc.stderr or proc.stdout).strip()[-500:])
            overall_ok = False
            continue

        comp_ok = True
        details = []
        for name, _count, _nbytes, dtype in out_specs:
            ref = out_dir / "Result_0" / f"{name}.raw"
            mine = dump / f"{comp}_call0_OUT_{name}.raw"
            if not ref.exists():
                details.append(f"{name}: qnn-net-run produced no output")
                comp_ok = False
                continue
            if not mine.exists():
                details.append(f"{name}: not dumped by Pi05Runner")
                comp_ok = False
                continue
            max_abs, msg = compare(ref, mine, dtype)
            if max_abs is None or max_abs > 0.0:
                comp_ok = False
                details.append(f"{name}: {msg}")

        if comp_ok:
            print(f"PASS {comp}: all {len(out_specs)} output tensors bitwise identical "
                  f"to qnn-net-run")
            summary.append((comp, "PASS", len(out_specs)))
        else:
            print(f"FAIL {comp}:")
            for d in details[:8]:
                print(f"    {d}")
            if len(details) > 8:
                print(f"    ... and {len(details) - 8} more")
            overall_ok = False
            summary.append((comp, "FAIL", len(out_specs)))

    print()
    for comp, status, n in summary:
        print(f"  {status:4s} {comp:16s} ({n} output tensors)")
    print()
    if overall_ok:
        print("VERIFIED: Pi05Runner's graph-order packing reproduces qnn-net-run exactly.")
        return 0
    print("NOT VERIFIED: see failures above.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
