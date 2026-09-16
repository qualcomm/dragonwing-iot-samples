# Retracted measurements

These files are kept for the record, not for use. They sit outside the
`bench/libero-arm-ablation-*.jsonl` glob so `bench/analyze_arm_ablation.py` cannot pick them up.

## `libero-arm-ablation-quant.jsonl` (50 Hz, n=50) and `libero-arm-ablation-quant-200hz.jsonl` (200 Hz, n=10, partial)

**Invalid: they measured a bug, not PWM quantization.**

`ArmModel.apply()` quantized the *absolute* rotation vector componentwise and differenced the result
to obtain a delta. Subtracting two absolute rotation vectors is not the axis-angle of the rotation
between them — that identity holds only for small angles about a common axis — and LIBERO's home pose
sits at `|rotvec| ≈ π`, the worst case for it.

Measured on the recorded action stream (`artifacts/libero_ep0.npz`):

| Model | rotation error / signal | steps where realized rotation opposed the request |
|---|---|---|
| broken, 50 Hz | 1.54 | 54/200 |
| broken, 200 Hz | 1.51 | 59/200 |
| fixed, 50 Hz | 0.125 | 0/200 |
| fixed, 200 Hz | 0.031 | 0/200 |

**The tell was that an 8× finer quantum barely changed the error.** A merely coarse quantizer improves
when you refine the grid; a broken one does not. That is what prompted the audit — the closed-loop
result (10/50 at 50 Hz, and 200 Hz failing to recover) looked like a strong finding and was an artifact.

**Why the instrument check did not catch it.** `bench/verify_arm_embodiment.py` passed 12/12 at the
time, but every quantization check drove *translation only* — the rotation path was never exercised.
It now asserts, from LIBERO's near-π home pose: no direction reversal, error falling with a finer
quantum, error below the signal at 200 Hz, and sub-quantum accumulation. 17/17.

The lesson worth keeping: an instrument check is only as good as its coverage, and "12/12 passed" was
more reassuring than it deserved to be. The `dof5` and `state-const` conditions are unaffected —
neither touches the rotation quantization path.
