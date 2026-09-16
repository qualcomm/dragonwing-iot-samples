#!/usr/bin/env python3
"""What does Pi0.5 actually ask a robot to do, and can this arm do it?

Costs nothing to run: no NPU, no simulator, no rendering. It reads the recorded episode's
state trajectory and compares the motion demanded there against the arm's physical limits,
which answers three of the four feasibility questions before any hardware is wired.

The three it answers:

  * SLEW -- can the servo traverse the commanded angle within one control tick? Compares the
    measured per-step rotation against slew_deg_per_tick.
  * QUANTIZATION -- is one PWM count small compared with one commanded step? This is the
    decisive question for ARM_WIRING_BRIEF.md section 5.5, and it is decisive in *rotation*
    rather than translation, which is not the intuitive expectation. Reported as
    "counts per commanded step": below ~2 the commanded motion is being quantized away.
  * WORKSPACE -- does the end-effector trajectory fit inside the arm's reach envelope? A task
    needing more lateral span than the arm has is infeasible at any policy quality, so this
    gates everything downstream.

The fourth -- whether the policy can work without joint feedback -- cannot be answered from a
trajectory. It needs the closed-loop ablations (E2/E3).

Usage:
    bench/analyze_arm_envelope.py
    bench/analyze_arm_envelope.py --npz artifacts/libero_ep0.npz --json bench/arm-envelope.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from arm_embodiment import DEG, ArmSpec  # noqa: E402

_REPO = Path(__file__).resolve().parent.parent


def _repo_path(rel: str) -> str:
    return str(_REPO / rel)


def percentiles(x: np.ndarray, qs=(50, 90, 95, 99, 100)) -> dict:
    return {f"p{q}": round(float(np.percentile(x, q)), 4) for q in qs}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", default=_repo_path("artifacts/libero_ep0.npz"))
    ap.add_argument("--spec", default=_repo_path("bench/arm-spec-measured.json"))
    ap.add_argument("--json", default=_repo_path("bench/arm-envelope.json"))
    args = ap.parse_args()

    z = np.load(args.npz, allow_pickle=False)
    state = np.asarray(z["state"], dtype=np.float64)
    fps = float(z["fps"])
    task = str(z["task"])
    pos, rot = state[:, :3], state[:, 3:6]

    spec_raw_early = json.loads(Path(args.spec).read_text())

    # NO REACH ANALYSIS HERE, DELIBERATELY. Reach has to be measured from the robot's base,
    # and in LIBERO the base position is SCENE-DEPENDENT -- three distinct values across
    # libero_10 (x = -0.51, -0.66, -0.75; z = 0.42 or 0.912). A single .npz carries no record
    # of which scene it came from, so any base this script assumed would be wrong for most
    # episodes. An earlier version did assume one and reported 666-812 mm for an episode whose
    # true requirement is 604 mm.
    #
    # So the division of labour is: this script owns the MOTION demand (per-step translation
    # and rotation, which are base-independent), and bench/verify_arm_workspace.py owns reach,
    # reading the base from the live sim for every task.

    # --- what the task demands ------------------------------------------------------
    span_mm = (pos.max(0) - pos.min(0)) * 1000.0
    d_pos_mm = np.linalg.norm(np.diff(pos, axis=0), axis=1) * 1000.0
    d_rot_deg = np.linalg.norm(np.diff(rot, axis=0), axis=1) / DEG

    demand = {
        "task": task,
        "frames": int(state.shape[0]),
        "fps": fps,
        "eef_span_mm": {ax: round(float(span_mm[i]), 2) for i, ax in enumerate("xyz")},
        "eef_bbox_diagonal_mm": round(float(np.linalg.norm(span_mm)), 2),
        "step_translation_mm": percentiles(d_pos_mm),
        "step_rotation_deg": percentiles(d_rot_deg),
        "total_path_mm": round(float(d_pos_mm.sum()), 1),
    }

    # --- what the arm can supply, per candidate PWM frame rate ----------------------
    spec_raw = json.loads(Path(args.spec).read_text())
    candidates = spec_raw["pwm"]["frame_rate_hz_candidates"]
    step_rot_med = float(np.percentile(d_rot_deg, 50))
    step_pos_med = float(np.percentile(d_pos_mm, 50))

    rates = []
    for hz in candidates:
        spec = ArmSpec.load(args.spec, frame_rate_hz=hz)
        s = spec.summary()
        # "Counts per commanded step" is the number that matters: a quantum comparable to
        # the motion increment means the motion does not survive quantization.
        s["counts_per_median_rotation_step"] = round(step_rot_med / spec.deg_per_count, 2)
        s["counts_per_median_translation_step"] = round(
            step_pos_med / spec.mm_per_count(), 2
        )
        s["verdict"] = (
            "adequate" if min(s["counts_per_median_rotation_step"],
                              s["counts_per_median_translation_step"]) >= 4.0
            else "marginal" if min(s["counts_per_median_rotation_step"],
                                   s["counts_per_median_translation_step"]) >= 2.0
            else "quantizes the commanded motion away"
        )
        rates.append(s)

    spec = ArmSpec.load(args.spec)

    # --- verdicts -------------------------------------------------------------------
    slew_needed = float(np.percentile(d_rot_deg, 100))
    slew_margin = spec.slew_deg_per_tick / slew_needed if slew_needed > 0 else float("inf")

    verdicts = {
        "slew": {
            "needed_deg_per_tick": round(slew_needed, 3),
            "available_deg_per_tick": round(spec.slew_deg_per_tick, 1),
            "margin_x": round(slew_margin, 1),
            "binding": bool(slew_margin < 2.0),
        },
        "quantization": {
            "at_default_frame_rate_hz": spec.frame_rate_hz,
            "counts_per_median_rotation_step": round(step_rot_med / spec.deg_per_count, 2),
            "binding": bool((step_rot_med / spec.deg_per_count) < 4.0),
            "recommended_frame_rate_hz": next(
                (r["frame_rate_hz"] for r in rates if r["verdict"] == "adequate"), None
            ),
        },
        "workspace": {
            "analyzed_here": False,
            "reason": ("Reach must be measured from the robot base, and LIBERO's base position "
                       "is scene-dependent (three distinct values across libero_10). A .npz "
                       "does not record which scene it came from, so this script does not "
                       "guess. See bench/arm-workspace-screen.json."),
            "authority": "bench/verify_arm_workspace.py",
        },
    }

    out = {
        "source_npz": str(Path(args.npz).relative_to(_REPO)
                          if Path(args.npz).is_relative_to(_REPO) else args.npz),
        "arm_spec_status": spec.status,
        "arm_spec_digest": spec.digest,
        "controller_scaling": spec_raw["controller_scaling"],
        "demand": demand,
        "frame_rate_candidates": rates,
        "verdicts": verdicts,
    }
    Path(args.json).write_text(json.dumps(out, indent=2) + "\n")

    # --- human summary --------------------------------------------------------------
    print(f"task: {task}")
    print(f"arm spec: {spec.status.upper()} (digest {spec.digest})")
    print()
    print("DEMAND (from the recorded episode)")
    print(f"  eef bbox span      {demand['eef_span_mm']}  diagonal "
          f"{demand['eef_bbox_diagonal_mm']} mm")
    print(f"  step translation   median {step_pos_med:.2f} mm, max "
          f"{d_pos_mm.max():.2f} mm")
    print(f"  step rotation      median {step_rot_med:.2f} deg, max "
          f"{d_rot_deg.max():.2f} deg")
    print()
    print("PWM FRAME RATE (counts per median commanded step)")
    print(f"  {'Hz':>6} {'deg/count':>10} {'mm/count':>9} {'rot':>7} {'trans':>7}  verdict")
    for r in rates:
        print(f"  {r['frame_rate_hz']:>6.0f} {r['deg_per_count']:>10.4f} "
              f"{r['mm_per_count_at_max_reach']:>9.3f} "
              f"{r['counts_per_median_rotation_step']:>7.2f} "
              f"{r['counts_per_median_translation_step']:>7.2f}  {r['verdict']}")
    print()
    print("VERDICTS")
    v = verdicts["slew"]
    print(f"  slew          {v['available_deg_per_tick']} deg/tick available vs "
          f"{v['needed_deg_per_tick']} needed -> {v['margin_x']}x margin, "
          f"{'BINDING' if v['binding'] else 'not binding'}")
    v = verdicts["quantization"]
    print(f"  quantization  {v['counts_per_median_rotation_step']} counts per median "
          f"rotation step at {v['at_default_frame_rate_hz']:.0f} Hz -> "
          f"{'BINDING' if v['binding'] else 'not binding'}"
          + (f"; use {v['recommended_frame_rate_hz']:.0f} Hz"
             if v["binding"] and v["recommended_frame_rate_hz"] else ""))
    print("  workspace     not analyzed here (base position is scene-dependent) -- see "
          "bench/verify_arm_workspace.py")
    print()
    print(f"wrote {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
