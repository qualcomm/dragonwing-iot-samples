#!/usr/bin/env python3
"""Pool closed-loop sweep records into the one number we are willing to publish.

Every episode written by demo/libero_closed_loop.py is a self-describing JSON object, so the
reported success rate should be derived from those records by a script rather than assembled
by hand. That is the difference between a number that can be re-checked and a number that
has to be trusted.

It also answers the question a single sweep cannot: **were the initial states we happened to
pick unrepresentatively easy?** LIBERO ships 50 initial states per task and a five-episode
sweep touches five of them. Running a second band at different indices and comparing is the
cheapest available check, so this script reports each band separately, pools them, and says
whether they are consistent.

Intervals are Wilson, not the textbook Wald p +/- z*sqrt(p(1-p)/n): at k=0 or k=n Wald
collapses to zero width, which is exactly the wrong answer at the boundaries where small
sweeps tend to sit.

Usage:
    bench/analyze_libero_sweep.py bench/libero-closed-loop-sweep.jsonl \
        bench/libero-closed-loop-inits20.jsonl
"""

from __future__ import annotations

import argparse
import collections
import json
import sys
from pathlib import Path

import numpy as np


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    d = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / d
    half = z * float(np.sqrt(p * (1 - p) / n + z * z / (4 * n * n))) / d
    return (max(0.0, centre - half), min(1.0, centre + half))


def band_label(rows: list[dict]) -> str:
    lo = min(r["init_index"] for r in rows)
    hi = max(r["init_index"] for r in rows)
    return f"inits {lo}-{hi}"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+", help="one or more sweep .jsonl files")
    ap.add_argument("--json", default="")
    args = ap.parse_args()

    bands: dict[str, list[dict]] = {}
    for f in args.files:
        p = Path(f)
        if not p.exists():
            sys.exit(f"missing {f}")
        rows = [json.loads(line) for line in p.read_text().splitlines() if line.strip()]
        if not rows:
            print(f"warning: {f} is empty, skipping", file=sys.stderr)
            continue
        bands[p.name] = rows

    if not bands:
        sys.exit("no records")

    allrows = [r for rows in bands.values() for r in rows]

    # Provenance must agree across pooled files, or the pooled number is meaningless.
    keys = ("suite", "replan_horizon", "max_steps", "mujoco", "robosuite",
            "image_orientation", "steps_per_action")
    prov = {k: sorted({str(r.get(k)) for r in allrows}) for k in keys}
    mismatched = {k: v for k, v in prov.items() if len(v) > 1}
    print("=== provenance ===")
    for k, v in prov.items():
        print(f"  {k:18s} {', '.join(v)}")
    if mismatched:
        print("\n  REFUSING TO POOL: these differ across files and would make a pooled rate")
        print("  meaningless. Report the bands separately instead:")
        for k, v in mismatched.items():
            print(f"    {k}: {v}")
        return 1

    print("\n=== per band ===")
    band_stats = []
    for name, rows in bands.items():
        k = sum(r["success"] for r in rows)
        n = len(rows)
        lo, hi = wilson(k, n)
        band_stats.append({"file": name, "label": band_label(rows), "k": k, "n": n,
                           "rate": k / n, "ci": [lo, hi]})
        print(f"  {band_label(rows):16s} {k:3d}/{n:<3d} = {k/n:6.1%}   95% CI {lo:5.1%} - {hi:5.1%}   ({name})")

    # Comparing bands that cover different task sets is meaningless, and the failure is
    # seductive: an incomplete band that happens to be missing the hardest tasks reports a
    # flatteringly high rate, and pooling then drags the headline up for no real reason.
    # Task difficulty varies enormously here (5/5 on six tasks, 2/5 on two), so this is not
    # a hypothetical.
    task_sets = {b["label"]: {r["task_index"] for r in rows}
                 for b, rows in zip(band_stats, bands.values())}
    common = set.intersection(*task_sets.values()) if len(task_sets) > 1 else set()
    union = set.union(*task_sets.values())
    incomparable = len(task_sets) > 1 and any(s != union for s in task_sets.values())

    if incomparable:
        print("\n  WARNING: the bands do not cover the same tasks, so their rates are NOT")
        print("  comparable and the pooled rate is biased toward whichever tasks are")
        print("  over-represented. Missing per band:")
        for label, s in task_sets.items():
            missing = sorted(union - s)
            if missing:
                print(f"    {label}: missing tasks {missing}")
        print("  Restricted to the tasks all bands share:")
        for b, rows in zip(band_stats, bands.values()):
            rs = [r for r in rows if r["task_index"] in common]
            if rs:
                kk = sum(r["success"] for r in rs)
                clo, chi = wilson(kk, len(rs))
                print(f"    {b['label']:16s} {kk:3d}/{len(rs):<3d} = {kk/len(rs):6.1%}"
                      f"   95% CI {clo:5.1%} - {chi:5.1%}")

    if len(band_stats) > 1:
        los = [b["ci"][0] for b in band_stats]
        his = [b["ci"][1] for b in band_stats]
        overlap = max(los) <= min(his)
        verdict = ("consistent; pooling is reasonable" if overlap else
                   "the bands disagree; do NOT pool, report separately")
        if incomparable:
            verdict += " -- but see the coverage warning above first"
        print(f"\n  intervals {'OVERLAP' if overlap else 'DO NOT OVERLAP'} -> {verdict}")

    print("\n=== per task ===")
    by_task: dict[int, dict[str, list[dict]]] = collections.defaultdict(dict)
    for name, rows in bands.items():
        for r in rows:
            by_task[r["task_index"]].setdefault(name, []).append(r)
    names = list(bands)
    hdr = "  ".join(f"{band_label(bands[n]):>12s}" for n in names)
    print(f"  {'#':>2}  {hdr}  {'pooled':>8}  task")
    for t in sorted(by_task):
        cells = []
        for n in names:
            rs = by_task[t].get(n, [])
            cells.append(f"{sum(r['success'] for r in rs)}/{len(rs)}" if rs else "-")
        pk = sum(r["success"] for rs in by_task[t].values() for r in rs)
        pn = sum(len(rs) for rs in by_task[t].values())
        text = next(iter(by_task[t].values()))[0]["task"]
        row = "  ".join(f"{c:>12s}" for c in cells)
        print(f"  {t:>2}  {row}  {f'{pk}/{pn}':>8}  {text[:52]}")

    k = sum(r["success"] for r in allrows)
    n = len(allrows)
    lo, hi = wilson(k, n)
    lat = np.array([r["latency_ms_mean"] for r in allrows if r.get("latency_ms_mean")])
    fails = [r for r in allrows if not r["success"]]
    at_cap = sum(1 for r in fails if r["steps"] >= r["max_steps"])
    backends = sorted({b for r in allrows for b in r.get("backends", [])})

    print("\n=== pooled ===")
    if incomparable:
        print("  (BIASED -- bands cover different tasks; see the warning above. Do not publish")
        print("   this figure until every band covers the same task set.)")
    print(f"  success            {k}/{n} = {k/n:.1%}   95% CI {lo:.1%} - {hi:.1%}  (Wilson)")
    print(f"  chunks             {sum(r['chunks'] for r in allrows)}")
    print(f"  backends           {backends}")
    print(f"  stale chunks       {sum(r['stale_chunks'] for r in allrows)}")
    print(f"  inference retries  {sum(r['timeouts'] for r in allrows)}")
    if lat.size:
        print(f"  NPU latency        {lat.mean():.1f} ms mean, {lat.min():.1f}-{lat.max():.1f} ms range")
    print(f"  wall clock         {sum(r['wall_s'] for r in allrows)/60:.0f} min")
    print(f"  failures           {len(fails)}, of which {at_cap} hit the {allrows[0]['max_steps']}-step cap")
    if fails and at_cap == len(fails):
        print("                     every failure ran out of step budget rather than diverging,")
        print("                     so the cap is part of the result and must be reported with it")

    if args.json:
        Path(args.json).write_text(json.dumps({
            "pooled": {"k": k, "n": n, "rate": k / n, "ci95_wilson": [lo, hi]},
            "bands": band_stats,
            "chunks": sum(r["chunks"] for r in allrows),
            "backends": backends,
            "stale_chunks": sum(r["stale_chunks"] for r in allrows),
            "retries": sum(r["timeouts"] for r in allrows),
            "latency_ms": {"mean": float(lat.mean()), "min": float(lat.min()),
                           "max": float(lat.max())} if lat.size else None,
            "failures": {"count": len(fails), "at_step_cap": at_cap},
            "provenance": {k2: v[0] for k2, v in prov.items()},
        }, indent=2) + "\n")
        print(f"\nwrote {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
