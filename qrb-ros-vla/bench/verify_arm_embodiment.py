#!/usr/bin/env python3
"""Does the arm model actually do what each ablation claims?

An ablation study is only evidence if the perturbation it names is the perturbation it
applies. A quantizer that silently passes sub-quantum motion through, or a "none" mode that
is not a strict identity, would produce a plausible table of success rates that measured
nothing -- and would look exactly the same from the outside. Same reasoning as
bench/verify_libero_null_policy.py, one level up: check the instrument, not just the result.

Needs no NPU, no simulator and no rendering. Runs in under a second, so there is no excuse
for not running it before a sweep.

Usage:
    bench/verify_arm_embodiment.py
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from arm_embodiment import DEG, ArmModel, ArmSpec  # noqa: E402

_REPO = Path(__file__).resolve().parent.parent
OUT_POS, OUT_ROT = 0.05, 0.5  # robosuite OSC_POSE defaults; the live values are read at run time
MEAN = np.array([0.0, 0.0, 0.6, 3.14, 0.0, 0.0, 0.02, -0.02])
START = MEAN.copy()


def drive(spec_path: str, mode: str, actions: np.ndarray,
          frame_rate: float | None = None) -> tuple[ArmModel, np.ndarray]:
    spec = ArmSpec.load(spec_path, frame_rate_hz=frame_rate)
    m = ArmModel(spec, mode, OUT_POS, OUT_ROT, state_mean=MEAN)
    m.reset(START)
    return m, np.array([m.apply(a) for a in actions])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--spec", default=str(_REPO / "bench" / "arm-spec-measured.json"))
    ap.add_argument("--json", default=str(_REPO / "bench" / "arm-embodiment-verdict.json"))
    args = ap.parse_args()

    checks: list[dict] = []

    def check(name: str, ok: bool, detail: str) -> None:
        checks.append({"check": name, "pass": bool(ok), "detail": detail})
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")

    spec = ArmSpec.load(args.spec)
    print(f"spec {spec.status} ({spec.digest}); {spec.deg_per_count:.4f} deg/count at "
          f"{spec.frame_rate_hz:.0f} Hz\n")

    # 1. "none" must be a bit-exact identity, or the sanity gate against the 86/100 baseline
    #    is meaningless and every published number is at risk.
    a_in = np.array([0.13, -0.27, 0.40, 0.10, -0.20, 0.30, 1.0])
    m, out = drive(args.spec, "none", np.tile(a_in, (5, 1)))
    truth = np.arange(8.0)
    check("none is an action identity", np.allclose(out, np.tile(a_in, (5, 1))),
          "5 steps returned unchanged")
    check("none is a state identity", np.allclose(m.state_estimate(truth), truth),
          "ground truth passed through")

    # 2. The quantizer must discretize, and must do so on the ABSOLUTE pose. A stream of
    #    sub-quantum requests has to come out as a staircase of {0, one count}, not as the
    #    requested smooth ramp -- that is the difference between modelling a servo and
    #    modelling nothing.
    step_mm = 1.0
    tiny = np.tile(np.array([step_mm / (OUT_POS * 1000), 0, 0, 0, 0, 0, -1.0]), (20, 1))
    m, out = drive(args.spec, "quant", tiny)
    levels = sorted({round(v, 6) for v in out[:, 0]})
    q_mm = m.q_pos * OUT_POS * 1000.0
    check("quantizer discretizes", len(levels) == 2 and 0.0 in levels,
          f"{len(levels)} output levels {levels}, quantum {q_mm:.3f} mm vs "
          f"{step_mm:.1f} mm requested per step")
    check("quantizer conserves travel", abs(out[:, 0].sum() - tiny[:, 0].sum()) <= m.q_pos,
          f"realized {out[:, 0].sum() * OUT_POS * 1000:.2f} mm vs requested "
          f"{tiny[:, 0].sum() * OUT_POS * 1000:.2f} mm (within one quantum)")

    # 3. A faster frame rate must give a FINER quantum. This is the counter-intuitive claim
    #    the frame-rate recommendation rests on, so it gets asserted rather than argued.
    m50, _ = drive(args.spec, "quant", tiny, frame_rate=50.0)
    m330, _ = drive(args.spec, "quant", tiny, frame_rate=330.0)
    check("faster PWM frame rate is finer", m330.q_pos < m50.q_pos,
          f"50 Hz -> {m50.q_pos * OUT_POS * 1000:.3f} mm, "
          f"330 Hz -> {m330.q_pos * OUT_POS * 1000:.3f} mm")

    # 3b. THE ROTATION PATH, which the first version of this file did not test at all -- every
    #     quantization check above drives translation only. That gap let a genuinely broken
    #     rotation quantizer pass 12/12 and invalidated a 50-episode sweep. The three properties
    #     below are the ones that were violated, so they are asserted directly.
    #
    #     A smooth rotation stream at the scale the policy actually commands (median 0.54 deg,
    #     max 1.20 deg per step -- bench/arm-envelope.json), applied from LIBERO's near-pi home
    #     pose, which is where naive axis-angle arithmetic breaks.
    rot_step_deg = 0.54
    rot_stream = np.tile(
        np.array([0, 0, 0, 0.0, rot_step_deg * DEG / OUT_ROT, 0, -1.0]), (60, 1)
    )
    noise = {}
    for hz in (50.0, 200.0):
        m, out = drive(args.spec, "quant", rot_stream, frame_rate=hz)
        req, got = rot_stream[:, 3:6], out[:, 3:6]
        err = np.linalg.norm(got - req, axis=1) / np.linalg.norm(req, axis=1)
        # Realized rotation must never oppose the request: a sign flip means the model is
        # steering the wrist the wrong way, which no quantizer does.
        flips = int(np.sum(np.sum(got * req, axis=1) < 0))
        noise[hz] = float(np.median(err))
        check(f"rotation quantizer does not reverse direction at {hz:.0f} Hz", flips == 0,
              f"{flips}/{len(rot_stream)} steps opposed the commanded rotation")

    check("rotation error falls with a finer quantum",
          noise[200.0] < noise[50.0] * 0.6,
          f"median error/signal {noise[50.0]:.3f} at 50 Hz -> {noise[200.0]:.3f} at 200 Hz")
    check("rotation error is smaller than the signal at 200 Hz", noise[200.0] < 0.5,
          f"median error/signal {noise[200.0]:.3f}")

    # 3c. Sub-quantum rotation must ACCUMULATE, not vanish forever: over many steps the
    #     realized total has to track the requested total even when each step is below one count.
    tiny_rot = np.tile(
        np.array([0, 0, 0, 0.0, 0.02 * DEG / OUT_ROT, 0, -1.0]), (200, 1)
    )
    m, out = drive(args.spec, "quant", tiny_rot)   # 0.02 deg/step vs a 0.4395 deg quantum
    req_tot = float(np.sum(tiny_rot[:, 4])) * OUT_ROT / DEG
    got_tot = float(np.sum(out[:, 4])) * OUT_ROT / DEG
    check("sub-quantum rotation accumulates rather than vanishing",
          abs(got_tot - req_tot) < spec.deg_per_count,
          f"requested {req_tot:.2f} deg in 0.02 deg steps, realized {got_tot:.2f} deg "
          f"(within one {spec.deg_per_count:.3f} deg count)")

    # 4. A frame rate whose period cannot contain the servo's longest pulse is a wiring
    #    error, not a slow arm. It must fail loudly.
    try:
        ArmSpec.load(args.spec, frame_rate_hz=450.0).us_per_count
        check("impossible frame rate rejected", False, "450 Hz was accepted")
    except ValueError as e:
        check("impossible frame rate rejected", True, str(e).split(",")[0])

    # 5. dof5 removes exactly the world-z rotation component.
    m, out = drive(args.spec, "dof5", np.tile(a_in, (3, 1)))
    check("dof5 removes only world-z rotation",
          out[0, 5] == 0.0 and out[0, 3] == a_in[3] and out[0, 4] == a_in[4]
          and np.allclose(out[0, :3], a_in[:3]),
          f"rot {a_in[3:6]} -> {out[0, 3:6]}, translation untouched")

    # 6. Dead reckoning must follow the COMMANDS and ignore the truth -- that is the whole
    #    point, and getting it backwards would quietly restore the feedback we are removing.
    n, per = 10, 0.1
    m, _ = drive(args.spec, "state-deadreckon",
                 np.tile(np.array([per, 0, 0, 0, 0, 0, -1.0]), (n, 1)))
    est = m.state_estimate(START)
    expect = START[0] + n * per * OUT_POS
    check("dead reckoning integrates commands", abs(est[0] - expect) < 1e-9,
          f"after {n} x {per * OUT_POS * 1000:.0f} mm, estimate x={est[0]:.4f} "
          f"(truth {START[0]:.4f})")
    check("dead reckoning ignores truth",
          np.allclose(m.state_estimate(np.arange(8.0))[:3], est[:3]),
          "estimate unchanged when a different truth is supplied")
    check("dead-reckoned gripper tracks the command", est[6] > 0.03 and est[7] < -0.03,
          f"gripper action -1 (open) -> qpos {np.round(est[6:8], 4)}")

    # 7. state-const must hand back the dataset mean, which is what normalizes to zeros.
    m, _ = drive(args.spec, "state-const",
                 np.tile(np.array([0.5, 0, 0, 0, 0, 0, 1.0]), (5, 1)))
    check("state-const returns the dataset mean",
          np.allclose(m.state_estimate(np.arange(8.0)), MEAN),
          "policy is told nothing, whatever the true state is")

    # 8. Slew limiting must cap the rotation magnitude at the servo's per-tick budget.
    big = np.tile(np.array([0, 0, 0, 5.0, 0, 0, -1.0]), (4, 1))
    m, out = drive(args.spec, "slew", big)
    cap = spec.slew_deg_per_tick * DEG / OUT_ROT
    check("slew caps per-tick rotation",
          np.all(np.linalg.norm(out[:, 3:6], axis=1) <= cap + 1e-9)
          and m.clipped_steps == len(big),
          f"requested 5.0 action units, cap {cap:.3f}, clipped {m.clipped_steps}/"
          f"{len(big)} steps")

    n_pass = sum(c["pass"] for c in checks)
    verdict = {
        "arm_spec_status": spec.status,
        "arm_spec_digest": spec.digest,
        "controller_scaling": {"output_max_pos_m": OUT_POS, "output_max_rot_rad": OUT_ROT},
        "checks": checks,
        "passed": n_pass,
        "total": len(checks),
        "pass": n_pass == len(checks),
    }
    Path(args.json).write_text(json.dumps(verdict, indent=2) + "\n")
    print(f"\n{n_pass}/{len(checks)} checks passed; wrote {args.json}")
    return 0 if n_pass == len(checks) else 1


if __name__ == "__main__":
    raise SystemExit(main())
