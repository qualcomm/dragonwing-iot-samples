#!/usr/bin/env python3
"""The language-sensitivity math, in one place.

Both ``bench/prompt_probe.py`` (offline measurement) and ``demo/libero_web_demo.py``
(the live A/B button) import this, so the number on stage and the number in the
repository are computed the same way.

The idea: freeze one observation, change only the prompt, and compare the action
chunks. A raw difference means nothing on its own -- the flow-matching sampler is
not perfectly repeatable -- so the same prompt is also run twice on the same
observation to get a noise floor. Language "worked" only if the prompt-to-prompt
difference clears that floor.
"""

from __future__ import annotations

import numpy as np

# Multiple of the repeat-noise floor above which a prompt difference is called
# real. 2.0 is deliberately conservative: measured in-distribution object
# swaps land near 3.7x (bench/prompt-probe/compare.json).
CLEAR_FACTOR = 2.0


def normalized_diff(a: np.ndarray, b: np.ndarray, action_std: np.ndarray) -> float:
    """Mean |a-b| per action dimension, in units of the dataset's action std.

    Normalizing by std is what makes the number comparable across the 7 DoF: a
    2 cm difference in x and a 0.2 rad difference in yaw are not the same size in
    raw units but are directly comparable as fractions of how much that dimension
    varies in the training data.
    """
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    n = min(len(a), len(b))
    dof = min(a.shape[1], b.shape[1], len(action_std))
    std = np.asarray(action_std, dtype=np.float64)[:dof]
    return float(np.mean(np.abs(a[:n, :dof] - b[:n, :dof]) / (std + 1e-8)))


def verdict(pair_diff: float, noise_floor: float) -> dict[str, object]:
    """Turn the two numbers into the sentence a presenter can say out loud."""
    ratio = pair_diff / noise_floor if noise_floor > 1e-9 else float("inf")
    clear = ratio >= CLEAR_FACTOR
    return {
        "pair_diff": round(pair_diff, 4),
        "noise_floor": round(noise_floor, 4),
        "ratio": round(ratio, 2),
        "language_changed_the_plan": bool(clear),
        "summary": (
            f"{ratio:.1f}x the repeat-noise floor - the wording changed the plan"
            if clear else
            f"{ratio:.1f}x the repeat-noise floor - indistinguishable from re-running "
            "the same prompt, so the model ignored the wording"
        ),
    }
