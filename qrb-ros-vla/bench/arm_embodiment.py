#!/usr/bin/env python3
"""A physical model of the VUPN2355 hobby arm, as perturbations on LIBERO's action stream.

WHY THIS EXISTS. The Pi0.5 export is calibrated on LIBERO's Franka Panda (docs/DESIGN.md
section 11). Driving a $329 PWM-servo arm instead means accepting four degradations at once:
no joint feedback, one missing positioning DOF, a coarse command quantum, and a finite slew
rate. Buying the arm's way into the loop and then discovering it cannot work is the expensive
order to do this in. Instead each degradation is reproduced *inside* LIBERO and costed against
the measured 86/100 baseline (bench/libero-closed-loop-pooled.json), which is only meaningful
because the null policy scores 0/24 (bench/libero-null-policy.json).

Deliberately free of ROS, LIBERO and robosuite imports: it is pure numpy over arrays, so it
can be exercised without a simulator, an NPU or a 2.9 GB bundle mapped.

WHY ONE STATEFUL CLASS RATHER THAN FOUR PURE FUNCTIONS. A real servo quantizes the *absolute*
angle it is commanded to hold, not each delta in isolation. Quantizing deltas independently
is a different -- and wrongly forgiving -- error process: it lets sub-quantum motion through
by accumulating in the controller's integrator, when on the real arm sub-quantum motion simply
does not happen. So the quantizer has to ride on an integrated pose, and once you are
integrating the pose anyway, dead-reckoned state estimation is the same bookkeeping. Hence
ArmModel owns the pose and the stages compose in a fixed, physically-ordered pipeline.

ACTION UNITS. LIBERO drives robosuite's OSC_POSE controller with control_delta=true and
output_max = [0.05, 0.05, 0.05, 0.5, 0.5, 0.5]. So an action component of 1.0 means 50 mm of
commanded translation or 0.5 rad of commanded rotation *per control step*. The harness passes
these scalings in from the live env rather than trusting the copy in arm-spec-measured.json,
because a robosuite version bump that changed them would otherwise silently rescale every
quantum in this file.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# quat_to_rotvec carries the continuity branch convention that LIBERO's near-pi home pose
# makes mandatory (see its docstring). Importing it rather than re-deriving it is the same
# single-definition rule demo/libero_closed_loop.py follows for the same reason.
from verify_libero_contract import quat_to_rotvec

DEG = np.pi / 180.0


# --- rotation helpers -----------------------------------------------------------------
# Hand-rolled to match quat_to_rotvec's no-scipy convention. Quaternions are (x, y, z, w),
# which is robosuite's order -- NOT scipy's default and not (w, x, y, z).

def rotvec_to_quat(rotvec: np.ndarray) -> np.ndarray:
    """(x, y, z, w) quaternion from an axis-angle vector."""
    v = np.asarray(rotvec, dtype=np.float64)
    angle = float(np.linalg.norm(v))
    if angle < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    axis = v / angle
    s = np.sin(angle / 2.0)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, np.cos(angle / 2.0)])


def quat_conj(q: np.ndarray) -> np.ndarray:
    """Conjugate (inverse, for unit quaternions) of an (x, y, z, w) quaternion."""
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product a*b for (x, y, z, w) quaternions, i.e. apply b then a."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


# --- the spec -------------------------------------------------------------------------

@dataclass(frozen=True)
class ArmSpec:
    """Measured (or, until M1 is done, assumed) physical parameters of the arm.

    Loaded from bench/arm-spec-measured.json so the numbers live in data and can be
    replaced by measurements without touching code. `status` and `digest` are stamped into
    every artifact produced from this spec, so a result computed from assumed geometry can
    never be quietly read as a measured one.
    """

    reach_max_mm: float
    reach_min_mm: float
    positioning_joints: int
    base_travel_deg: float
    joint_travel_deg: float
    slew_s_per_60deg: float
    backlash_deg: float
    pulse_min_us: float
    pulse_max_us: float
    resolution_bits: int
    frame_rate_hz: float
    command_rate_hz: float
    status: str
    digest: str

    @classmethod
    def load(cls, path: str | Path, frame_rate_hz: float | None = None) -> "ArmSpec":
        raw = Path(path).read_bytes()
        d = json.loads(raw)
        g, s, p, c = (d["geometry"], d["servo_dynamics"], d["pwm"], d["control"])
        return cls(
            reach_max_mm=float(g["reach_max_mm"]),
            reach_min_mm=float(g["reach_min_mm"]),
            positioning_joints=int(g["positioning_joints"]),
            base_travel_deg=float(g["base_travel_deg"]),
            joint_travel_deg=float(g["joint_travel_deg"]),
            slew_s_per_60deg=float(s["slew_s_per_60deg"]),
            backlash_deg=float(s["backlash_deg"]),
            pulse_min_us=float(p["pulse_min_us"]),
            pulse_max_us=float(p["pulse_max_us"]),
            resolution_bits=int(p["resolution_bits"]),
            # Overridable because sweeping the PWM frame rate is exactly the point of E5:
            # it turns ARM_WIRING_BRIEF.md section 5.5 from a resolution argument into a
            # success-rate measurement.
            frame_rate_hz=float(frame_rate_hz if frame_rate_hz is not None
                                else p["frame_rate_hz"]),
            command_rate_hz=float(c["command_rate_hz"]),
            status=str(d.get("status", "unknown")),
            digest=hashlib.sha256(raw).hexdigest()[:12],
        )

    # --- derived quantities ---------------------------------------------------------

    @property
    def us_per_count(self) -> float:
        """PWM pulse resolution. The 12 bits span one PERIOD, not the servo's pulse band.

        This is why a faster frame rate buys FINER resolution, which is counter-intuitive
        and is the whole content of the frame-rate decision: at 50 Hz the 4096 counts are
        spread over 20 ms and only ~410 of them land inside the 500-2500 us band, but at
        330 Hz the same 4096 counts cover 3.03 ms and ~2700 of them are usable.
        """
        period_us = 1e6 / self.frame_rate_hz
        if period_us <= self.pulse_max_us:
            raise ValueError(
                f"frame_rate_hz={self.frame_rate_hz} gives a {period_us:.0f} us period, "
                f"which cannot contain a {self.pulse_max_us:.0f} us pulse"
            )
        return period_us / float(2 ** self.resolution_bits)

    @property
    def counts_over_travel(self) -> float:
        return (self.pulse_max_us - self.pulse_min_us) / self.us_per_count

    @property
    def deg_per_count(self) -> float:
        """Angular quantum of a single commanded joint. The arm's real resolution limit."""
        return self.joint_travel_deg / self.counts_over_travel

    @property
    def slew_deg_per_s(self) -> float:
        return 60.0 / self.slew_s_per_60deg

    @property
    def slew_deg_per_tick(self) -> float:
        """How far a joint can move between two policy commands."""
        return self.slew_deg_per_s / self.command_rate_hz

    def mm_per_count(self, radius_mm: float | None = None) -> float:
        """Linear tip quantum at a working radius. Defaults to the pessimistic max reach."""
        r = self.reach_max_mm if radius_mm is None else radius_mm
        return self.deg_per_count * DEG * r

    def summary(self) -> dict:
        return {
            "status": self.status,
            "spec_digest": self.digest,
            "frame_rate_hz": self.frame_rate_hz,
            "us_per_count": round(self.us_per_count, 4),
            "counts_over_travel": round(self.counts_over_travel, 1),
            "deg_per_count": round(self.deg_per_count, 4),
            "mm_per_count_at_max_reach": round(self.mm_per_count(), 4),
            "slew_deg_per_s": round(self.slew_deg_per_s, 1),
            "slew_deg_per_tick": round(self.slew_deg_per_tick, 1),
        }


# --- the model ------------------------------------------------------------------------

# Which pipeline stages each --ablate mode turns on. Kept as data so the harness cannot
# drift from the analyzer's idea of what a condition means.
MODES: dict[str, dict[str, bool]] = {
    "none":               {"slew": False, "dof5": False, "quant": False, "deadreckon": False},
    "state-const":        {"slew": False, "dof5": False, "quant": False, "deadreckon": False},
    "state-deadreckon":   {"slew": False, "dof5": False, "quant": False, "deadreckon": True},
    "dof5":               {"slew": False, "dof5": True,  "quant": False, "deadreckon": False},
    "quant":              {"slew": False, "dof5": False, "quant": True,  "deadreckon": False},
    "slew":               {"slew": True,  "dof5": False, "quant": False, "deadreckon": False},
    "composite":          {"slew": True,  "dof5": True,  "quant": True,  "deadreckon": True},
}


class ArmModel:
    """Turns the actions Pi0.5 emits into the actions this arm could actually execute.

    One instance per episode; `reset()` at each episode start with the true initial state.

    Stage order is physical, not arbitrary:
      1. slew limit      -- the servo cannot traverse more than so many degrees per tick
      2. 5-DOF projection -- the commanded pose is snapped onto the reachable manifold
      3. integrate        -- accumulate into an absolute commanded pose
      4. quantize         -- snap that absolute pose onto the PWM count grid
      5. differentiate    -- emit the delta that actually gets realized

    Steps 3-5 are what make quantization faithful rather than forgiving: sub-quantum
    commands vanish instead of accumulating in the controller's integrator.
    """

    def __init__(self, spec: ArmSpec, mode: str,
                 output_max_pos_m: float, output_max_rot_rad: float,
                 state_mean: np.ndarray, quant_radius_mm: float | None = None):
        if mode not in MODES:
            raise ValueError(f"unknown ablate mode {mode!r}; expected one of {sorted(MODES)}")
        self.spec = spec
        self.mode = mode
        self.stages = MODES[mode]
        self.state_const = (mode == "state-const")
        # Held so state_estimate() can always return a real 8-D vector: the "tell the policy
        # nothing" condition is *publishing the dataset mean*, which normalizes to all zeros.
        self.state_mean = np.asarray(state_mean, dtype=np.float64)
        self.out_pos = float(output_max_pos_m)
        self.out_rot = float(output_max_rot_rad)
        self.quant_radius_mm = quant_radius_mm

        # Command quanta expressed in ACTION units, so the pipeline never leaves the space
        # the controller speaks. A position action of 1.0 is out_pos metres; one PWM count
        # is mm_per_count millimetres; hence the ratio.
        self.q_pos = self.spec.mm_per_count(quant_radius_mm) / (self.out_pos * 1000.0)
        self.q_rot = (self.spec.deg_per_count * DEG) / self.out_rot
        self.slew_action = (self.spec.slew_deg_per_tick * DEG) / self.out_rot

        self.pos = np.zeros(3)
        self.rotq = np.array([0.0, 0.0, 0.0, 1.0])
        self.rotvec = np.zeros(3)
        self.grip = -1.0
        self.realized_pos = np.zeros(3)
        self.realized_rotq = np.array([0.0, 0.0, 0.0, 1.0])
        self.realized_rotvec = np.zeros(3)
        self.clipped_steps = 0
        self.steps = 0

    # --- lifecycle ------------------------------------------------------------------

    def reset(self, state8: np.ndarray) -> None:
        """Anchor the integrator on the true state once, at episode start.

        A real arm is homed and calibrated before a run, so seeding from truth models the
        arm accurately -- it is only *thereafter* that it has no idea where it is. Seeding
        from zero instead would measure a calibration failure we are not trying to measure.
        """
        s = np.asarray(state8, dtype=np.float64)
        self.pos = s[:3].copy()
        self.rotvec = s[3:6].copy()
        self.rotq = rotvec_to_quat(self.rotvec)
        self.grip = -1.0
        self.realized_pos = self.pos.copy()
        self.realized_rotq = self.rotq.copy()
        self.realized_rotvec = self.rotvec.copy()
        self.clipped_steps = 0
        self.steps = 0

    # --- the pipeline ---------------------------------------------------------------

    def apply(self, action: np.ndarray) -> np.ndarray:
        """One 7-D action in, the arm-realizable 7-D action out."""
        a = np.asarray(action, dtype=np.float64).copy()
        self.steps += 1

        if self.stages["slew"]:
            rot = a[3:6]
            mag = float(np.linalg.norm(rot))
            if mag > self.slew_action and mag > 0.0:
                a[3:6] = rot * (self.slew_action / mag)
                self.clipped_steps += 1

        if self.stages["dof5"]:
            a[3:6] = self._project_5dof(a[3:6])

        # Integrate the request into an absolute commanded pose.
        self.pos = self.pos + a[:3] * self.out_pos
        dq = rotvec_to_quat(a[3:6] * self.out_rot)
        self.rotq = quat_mul(dq, self.rotq)          # world-frame delta ⇒ pre-multiply
        self.rotq = self.rotq / (np.linalg.norm(self.rotq) + 1e-12)
        self.rotvec = quat_to_rotvec(self.rotq, self.rotvec)
        self.grip = float(np.clip(a[6], -1.0, 1.0))

        if not self.stages["quant"]:
            self.realized_pos = self.pos.copy()
            self.realized_rotq = self.rotq.copy()
            self.realized_rotvec = self.rotvec.copy()
            return a

        out = a.copy()

        # Position: snap the ABSOLUTE pose to the count grid and emit the implied delta.
        # Valid componentwise -- translation axes are independent.
        step_pos = self.q_pos * self.out_pos
        target_pos = np.round(self.pos / step_pos) * step_pos
        out[:3] = (target_pos - self.realized_pos) / self.out_pos
        self.realized_pos = target_pos

        # Rotation: quantize the RELATIVE rotation, never the absolute rotation vector.
        #
        # Subtracting two absolute rotation vectors is not the axis-angle of the rotation
        # between them -- that identity only holds for small angles about a common axis. LIBERO's
        # home pose sits at |rotvec| ~ pi, so an earlier version of this code that quantized
        # self.rotvec componentwise and differenced the result injected error of ~1.5x the
        # commanded signal and REVERSED the commanded direction on 27% of steps, and did so
        # almost independently of the quantum -- an 8x finer grid barely changed it. That is the
        # signature of a broken rotation model rather than a coarse one, and it invalidated the
        # first `quant` measurements entirely.
        #
        # Doing it through quaternions keeps every componentwise operation on a SMALL rotation
        # (the residual between requested and realized, at most a few degrees) where it is valid,
        # while still accumulating: a sub-quantum request stays in the residual and is applied
        # once it grows past one count, which is exactly how a servo behaves.
        q_rel = quat_mul(self.rotq, quat_conj(self.realized_rotq))
        if q_rel[3] < 0.0:
            q_rel = -q_rel                    # shorter arc; same rotation
        rel = quat_to_rotvec(q_rel)           # small, so no continuity branch needed
        step_rot = self.q_rot * self.out_rot
        rel_q = np.round(rel / step_rot) * step_rot
        out[3:6] = rel_q / self.out_rot
        self.realized_rotq = quat_mul(rotvec_to_quat(rel_q), self.realized_rotq)
        self.realized_rotq = self.realized_rotq / (np.linalg.norm(self.realized_rotq) + 1e-12)
        self.realized_rotvec = quat_to_rotvec(self.realized_rotq, self.realized_rotvec)
        return out

    def _project_5dof(self, drot: np.ndarray) -> np.ndarray:
        """Drop the world-z component of the commanded rotation delta.

        The arm's topology is base yaw about world z, then a chain of pitches in the
        vertical plane that yaw defines, then a roll about the tool axis. So the tool's
        azimuth is not independently commandable: it is whatever base yaw already had to be
        to put the tip where it is. Tool yaw is the DOF a 5-joint arm does not have.

        APPROXIMATION, stated rather than buried: robosuite's OSC_POSE delta orientation is
        world-framed, so zeroing the z component of the axis-angle delta removes yaw only to
        first order -- for large simultaneous rotations the axis-angle components are not
        independent. Commanded rotations here are small (measured median 0.54 deg/step,
        max 1.20 deg -- bench/arm-envelope.json), which is the regime where this holds. The
        faithful alternative is a full pose projection onto the manifold; if this ablation
        turns out to be the deciding one, that is the refinement to make.
        """
        out = np.asarray(drot, dtype=np.float64).copy()
        out[2] = 0.0
        return out

    # --- state estimation -----------------------------------------------------------

    def state_estimate(self, true_state8: np.ndarray) -> np.ndarray:
        """The 8-D state the policy gets fed, under this condition's feedback assumption.

        - "state-const": the dataset mean. Post-normalization this is all zeros, i.e. the
          policy is told nothing. Not physical -- it is the *upper bound* on how much
          missing proprioception can cost, and one cheap run of it can retire the whole
          question if the rate barely moves.
        - deadreckon: commanded pose fed back as if measured. This IS what a PWM servo arm
          can supply: it knows what it asked for and nothing about what happened.
        - otherwise: ground truth, i.e. the baseline.
        """
        if self.state_const:
            return self.state_mean.copy()
        if not self.stages["deadreckon"]:
            return np.asarray(true_state8, dtype=np.float64)
        # gripper_qpos is 2 symmetric finger positions; the dataset shows col0 in
        # [0.0024, 0.0399] with col1 = -col0. Map the commanded -1 (open) / +1 (closed)
        # onto that observed span, since an open-loop gripper reports nothing either.
        frac = (1.0 - self.grip) / 2.0          # -1 -> 1.0 (open), +1 -> 0.0 (closed)
        finger = 0.0024 + frac * (0.0399 - 0.0024)
        return np.concatenate([self.realized_pos, self.realized_rotvec,
                               [finger, -finger]])

    def audit(self) -> dict:
        """Per-episode record of what the model actually did, for the JSONL."""
        return {
            "arm_mode": self.mode,
            "arm_stages": dict(self.stages),
            "arm_spec_status": self.spec.status,
            "arm_spec_digest": self.spec.digest,
            "arm_frame_rate_hz": self.spec.frame_rate_hz,
            "arm_q_pos_action": round(self.q_pos, 6),
            "arm_q_rot_action": round(self.q_rot, 6),
            "arm_slew_action": round(self.slew_action, 6),
            "arm_slew_clipped_steps": self.clipped_steps,
            "arm_steps": self.steps,
            "arm_drift_mm": (
                round(float(np.linalg.norm(self.realized_pos - self.pos)) * 1000.0, 3)
            ),
        }
