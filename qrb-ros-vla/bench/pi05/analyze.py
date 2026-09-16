#!/usr/bin/env python3
"""Turn the raw measure.py JSONL into the canonical pi05 cost model.

Every number printed here traces back to a line in a results_*.jsonl file produced by
measure.py on a real IQ-9075. Nothing is estimated. Stages that were not measured print TBD.

The per-observation model is a SUM OF MEASURED STAGES, not a measured end-to-end figure:
the four graphs form a strict dependency chain (vision -> token_emb -> backbone -> N x action
expert), so summing is legitimate, but it excludes host-side glue a real ROS 2 node would add.
That distinction is deliberate and is labelled in the output.
"""
from __future__ import annotations

import json
import statistics
from pathlib import Path

HERE = Path(__file__).resolve().parent
# 3 camera views feed token_emb's img_embed1/2/3; flow matching runs the action expert
# N times per observation, re-using one backbone pass.
N_CAMERAS = 3
N_DENOISE = 10


def load(*names: str) -> list[dict]:
    rows: list[dict] = []
    for n in names:
        p = HERE / n
        if p.exists():
            rows += [json.loads(l) for l in p.read_text().splitlines() if l.strip()]
    return rows


def best(rows: list[dict], graph: str, cond: str) -> tuple[float | None, float | None, int]:
    """Best (lowest) min-latency and init across repeats for one graph+condition."""
    sel = [r for r in rows if r["graph"] == graph and r["cond"] == cond
           and r.get("netrun_min_ms") and r.get("npu_confirmed")]
    if not sel:
        return None, None, 0
    lat = min(r["netrun_min_ms"] for r in sel)
    ini = min((r["init_ms"] for r in sel if r.get("init_ms")), default=None)
    return lat, ini, len(sel)


def spread(rows: list[dict], graph: str, cond: str) -> str:
    sel = [r["netrun_min_ms"] for r in rows if r["graph"] == graph and r["cond"] == cond
           and r.get("netrun_min_ms")]
    if len(sel) < 2:
        return "-"
    return f"{(max(sel) - min(sel)) / min(sel) * 100:.1f}%"


def main() -> None:
    rows = load("results_isolate.jsonl", "results_max.jsonl", "results_cliprof.jsonl",
                "results_init.jsonl", "results_profiles.jsonl", "results_cli.jsonl",
                "results_knobs.jsonl")
    if not rows:
        print("no results yet - run measure.py first")
        return

    # UNTUNED = how a naive caller invokes qnn-net-run: no backend-extensions config file,
    # so --perf_profile is silently dropped and the DSP stays at power_saver clocks.
    # TUNED = backend-extensions config file present + burst + shared_buffer.
    untuned_conds = ["z_nocfg_noshbuf", "c_nocfg_burst", "z_nocfg", "k_base", "p_nocfg_burst"]
    tuned_conds = ["c_burst", "m_base", "z_cfg_dev", "z_cfg_bare", "m_custom", "f_ref"]

    print("=" * 88)
    print("pi05 on Dragonwing IQ-9075 (QCS9075, HTP v73) - per-graph latency, measured")
    print("  primary statistic: MIN over inferences, best over repeats (contention-robust)")
    print("  NPU execution confirmed for every row (accelerator time + HVX thread count > 0)")
    print("=" * 88)
    hdr = f"{'graph':16}{'untuned':>10}{'tuned':>10}{'speedup':>9}{'init_ms':>9}{'rep_spread':>12}"
    print(hdr)

    table: dict[str, dict] = {}
    for g in ["vision_encoder", "token_emb", "backbone", "action_expert"]:
        u = max((best(rows, g, c)[0] for c in untuned_conds
                 if best(rows, g, c)[0]), default=None)
        cands = [(best(rows, g, c)[0], c) for c in tuned_conds if best(rows, g, c)[0]]
        t, tcond = min(cands) if cands else (None, None)
        ini = best(rows, g, tcond)[1] if tcond else None
        table[g] = {"untuned": u, "tuned": t, "init": ini, "cond": tcond}
        su = f"{u / t:.2f}x" if (u and t) else "-"
        print(f"{g:16}{(u or 0):10.2f}{(t or 0):10.2f}{su:>9}{(ini or 0):9.1f}"
              f"{spread(rows, g, tcond or ''):>12}")

    # ---------------------------------------------------------------- cost model
    def total(key: str) -> float | None:
        v = [table[g][key] for g in table]
        if any(x is None for x in v):
            return None
        ve, te, bb, ae = (table[g][key] for g in
                          ["vision_encoder", "token_emb", "backbone", "action_expert"])
        return N_CAMERAS * ve + te + bb + N_DENOISE * ae

    tu, tt = total("untuned"), total("tuned")
    print("\n" + "=" * 88)
    print(f"PER-OBSERVATION COST MODEL  ({N_CAMERAS} cameras, N={N_DENOISE} denoise steps)")
    print("  sum of measured stages along the dependency chain; excludes host-side glue")
    print("=" * 88)
    print(f"  {N_CAMERAS} x vision_encoder + 1 x token_emb + 1 x backbone "
          f"+ {N_DENOISE} x action_expert")
    for name, tot in (("untuned", tu), ("tuned", tt)):
        if tot is None:
            continue
        print(f"\n  {name:8} = {tot:8.1f} ms  ({1000 / tot:5.2f} obs/s)")
        for g, mult in (("vision_encoder", N_CAMERAS), ("token_emb", 1),
                        ("backbone", 1), ("action_expert", N_DENOISE)):
            v = table[g][name] * mult
            print(f"      {g:16} x{mult:<3} {v:8.1f} ms  {v / tot * 100:5.1f}%")
    if tu and tt:
        print(f"\n  end-to-end speedup from runtime configuration alone: {tu / tt:.2f}x")

    # One-time process startup: contexts are created once and held resident.
    inits = [table[g]["init"] for g in table if table[g]["init"]]
    if len(inits) == 4:
        print(f"\n  one-time context init (all four graphs): {sum(inits):.0f} ms")
        print("  -> a startup cost, NOT per-inference, provided the process keeps the")
        print("     contexts resident. Re-creating contexts per inference would add this")
        print(f"     to every observation ({sum(inits) / (tt or 1) * 100:.0f}% overhead).")

    # ------------------------------------------------- denoise-step sensitivity
    if table["action_expert"]["tuned"]:
        ae = table["action_expert"]["tuned"]
        fixed = tt - N_DENOISE * ae if tt else None
        if fixed:
            print("\n" + "=" * 88)
            print("DENOISE-STEP SENSITIVITY (the largest remaining lever, tuned config)")
            print("  changes model output - an accuracy claim needs a task-success")
            print("  evaluation, which is NOT measured here")
            print("=" * 88)
            print(f"  {'N':>3}{'total_ms':>11}{'obs/s':>9}{'vs N=10':>9}")
            for n in (10, 8, 5, 4, 2, 1):
                tot = fixed + n * ae
                print(f"  {n:3d}{tot:11.1f}{1000 / tot:9.2f}{tt / tot:8.2f}x")

    # ------------------------------------------------------ regressions / inert
    print("\n" + "=" * 88)
    print("MEASURED NON-WINS (kept so nobody re-tries them)")
    print("=" * 88)
    for g in ["action_expert", "vision_encoder", "backbone"]:
        ref = best(rows, g, "m_base")[0]
        if not ref:
            continue
        for cond, label in [("m_custom", "perf_profile=custom, all corners pinned MAX"),
                            ("m_custom_ddr", "ddr_perf_mode=true"),
                            ("m_all", "every power knob at once"),
                            ("m_custom_poll", "MAX corners + rpc polling")]:
            v = best(rows, g, cond)[0]
            if v:
                print(f"  {g:16}{label:44}{ref / v:5.2f}x")
        break

    print("\nraw data: " + ", ".join(sorted(p.name for p in HERE.glob('results_*.jsonl'))))


if __name__ == "__main__":
    main()
