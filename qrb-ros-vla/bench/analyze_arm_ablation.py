#!/usr/bin/env python3
"""Pool the hobby-arm ablation conditions and report what each limitation costs.

Reads every bench/libero-arm-ablation-*.jsonl, groups by condition, and reports k/n with a
Wilson interval against the measured baseline. The headline output is the per-condition delta:
how many success-rate points that one physical limitation removes.

TWO DISCIPLINES THIS ENFORCES, both of which are easy to get wrong by accident:

  * The 'none' condition is a GATE, not a result. If it does not land inside the baseline's
    interval, the ablation harness is perturbing something it should not be and every other
    row is untrustworthy. Reported first and loudly.
  * Conditions are never pooled with each other, and a delta is only called significant when
    the Wilson intervals are disjoint. With n=50 per condition the intervals are wide, so
    small differences genuinely are not resolvable and saying so is the honest reading.

Usage:
    bench/analyze_arm_ablation.py
    bench/analyze_arm_ablation.py --glob 'bench/libero-arm-ablation-*.jsonl'
"""

from __future__ import annotations

import argparse
import glob
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_libero_sweep import wilson  # noqa: E402

_REPO = Path(__file__).resolve().parent.parent

# What each condition is asking, in one line, so the table reads without the source open.
QUESTIONS = {
    "none": "GATE: does the ablation harness leave the loop unchanged?",
    "state-const": "Can the policy work when told nothing about joint state?",
    "state-deadreckon": "Can it work on commanded-pose feedback, as a PWM servo arm supplies?",
    "dof5": "What does the wrist DOF a 5-joint arm lacks cost?",
    "quant": "What does the PWM command quantum cost?",
    "slew": "What does the servo's finite joint speed cost?",
    "composite": "All of the above at once: the ceiling for a correctly-sized hobby arm.",
}


def load(pattern: str) -> dict[str, list[dict]]:
    """Group episodes into conditions.

    GROUPING INCLUDES THE PWM FRAME RATE, not just the mode. 'quant at 50 Hz' and 'quant at
    200 Hz' are different physical conditions -- the whole point of sweeping the frame rate is
    that they differ -- so keying on mode alone would silently pool a catastrophic condition
    with a healthy one and report their average as though it meant something. The rate only
    appears in the label when a mode actually has more than one, to keep the common case
    readable.
    """
    raw: dict[tuple[str, float | None], list[dict]] = {}
    for path in sorted(glob.glob(pattern)):
        for line in Path(path).read_text().splitlines():
            line = line.strip()
            if not line:
                continue
            r = json.loads(line)
            mode = r.get("arm_mode", r.get("ablate", "unknown"))
            raw.setdefault((mode, r.get("arm_frame_rate_hz")), []).append(r)

    rates_per_mode: dict[str, set] = {}
    for mode, hz in raw:
        rates_per_mode.setdefault(mode, set()).add(hz)

    by_cond: dict[str, list[dict]] = {}
    for (mode, hz), rows in raw.items():
        label = mode
        if len(rates_per_mode[mode]) > 1 and hz is not None:
            label = f"{mode}@{hz:.0f}Hz"
        by_cond.setdefault(label, []).extend(rows)
    return by_cond


def baseline(path: Path) -> dict | None:
    if not path.exists():
        return None
    d = json.loads(path.read_text())
    p = d["pooled"]
    return {"k": p["k"], "n": p["n"], "rate": p["rate"], "ci": p["ci95_wilson"]}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--glob", default=str(_REPO / "bench" / "libero-arm-ablation-*.jsonl"))
    ap.add_argument("--baseline",
                    default=str(_REPO / "bench" / "libero-closed-loop-pooled.json"))
    ap.add_argument("--json", default=str(_REPO / "bench" / "arm-ablation.json"))
    args = ap.parse_args()

    by_mode = load(args.glob)
    if not by_mode:
        print(f"no ablation records matched {args.glob}", file=sys.stderr)
        return 1
    base = baseline(Path(args.baseline))

    conditions = []
    for mode, rows in by_mode.items():
        n = len(rows)
        k = sum(bool(r["success"]) for r in rows)
        lo, hi = wilson(k, n)
        spec_status = {r.get("arm_spec_status") for r in rows}
        rates = {r.get("arm_frame_rate_hz") for r in rows}
        clipped = sum(int(r.get("arm_slew_clipped_steps", 0) or 0) for r in rows)
        total_steps = sum(int(r.get("arm_steps", 0) or 0) for r in rows)
        cond = {
            "mode": mode,
            "question": QUESTIONS.get(mode, ""),
            "k": k, "n": n, "rate": round(k / n, 4),
            "ci95_wilson": [round(lo, 4), round(hi, 4)],
            "arm_spec_status": sorted(x for x in spec_status if x),
            "arm_frame_rate_hz": sorted(x for x in rates if x is not None),
            "slew_clipped_steps": clipped,
            "arm_steps": total_steps,
            "tasks": sorted({r["task_index"] for r in rows}),
            "at_step_cap": sum(1 for r in rows
                               if not r["success"] and r["steps"] >= r["max_steps"]),
        }
        if base:
            cond["delta_vs_baseline_pts"] = round((k / n - base["rate"]) * 100.0, 1)
            # Disjoint Wilson intervals is a deliberately conservative test. With n=50 the
            # intervals span ~20 points, so anything smaller is simply not resolvable here
            # and calling it a result would be overreading the data.
            cond["separated_from_baseline"] = bool(hi < base["ci"][0] or lo > base["ci"][1])
        conditions.append(cond)

    order = list(QUESTIONS)
    conditions.sort(key=lambda c: order.index(c["mode"]) if c["mode"] in order else 99)

    gate = next((c for c in conditions if c["mode"] == "none"), None)
    gate_ok = None
    if gate and base:
        # The gate passes if its interval overlaps the baseline's. Not equality: these are
        # different episode samples, so some spread is expected and demanding a match would
        # fail for the wrong reason.
        gate_ok = not (gate["ci95_wilson"][1] < base["ci"][0]
                       or gate["ci95_wilson"][0] > base["ci"][1])

    out = {
        "baseline": base,
        "gate": {
            "ran": gate is not None,
            "pass": gate_ok,
            "note": ("'none' must overlap the baseline interval, else the harness perturbs "
                     "something it should not and no other row can be trusted."),
        },
        "conditions": conditions,
    }
    Path(args.json).write_text(json.dumps(out, indent=2) + "\n")

    # --- report ---------------------------------------------------------------------
    if base:
        print(f"baseline: {base['k']}/{base['n']} = {base['rate']:.1%} "
              f"(CI {base['ci'][0]:.1%}-{base['ci'][1]:.1%})")
    if gate is None:
        print("\nGATE: NOT RUN -- run `scripts/run-arm-ablation.sh none 5` before trusting "
              "any condition below.")
    elif gate_ok:
        print(f"\nGATE: PASS -- 'none' scored {gate['k']}/{gate['n']} = {gate['rate']:.1%}, "
              "overlapping the baseline.")
    else:
        print(f"\nGATE: *** FAIL *** -- 'none' scored {gate['k']}/{gate['n']} = "
              f"{gate['rate']:.1%}, outside the baseline interval. The ablation harness is "
              "altering the loop; every row below is suspect.")

    print()
    print(f"  {'condition':<18} {'k/n':>7} {'rate':>7} {'95% CI':>15} {'delta':>7}  sep")
    for c in conditions:
        ci = f"{c['ci95_wilson'][0]:.0%}-{c['ci95_wilson'][1]:.0%}"
        d = c.get("delta_vs_baseline_pts")
        sep = "yes" if c.get("separated_from_baseline") else "no"
        print(f"  {c['mode']:<18} {c['k']:>3}/{c['n']:<3} {c['rate']:>6.1%} {ci:>15} "
              f"{(f'{d:+.1f}' if d is not None else '   -'):>7}  {sep}")
    print()
    for c in conditions:
        if c["question"]:
            print(f"  {c['mode']:<18} {c['question']}")

    statuses = {s for c in conditions for s in c["arm_spec_status"]}
    if "assumed" in statuses:
        print("\nNOTE: at least one condition used ASSUMED arm geometry "
              "(bench/arm-spec-measured.json m1_checklist not yet done). Quantization and "
              "slew magnitudes are datasheet-derived, so treat their absolute values as "
              "provisional; the state conditions do not depend on arm geometry at all.")
    print(f"\nwrote {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
