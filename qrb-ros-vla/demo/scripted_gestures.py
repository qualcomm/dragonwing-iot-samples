#!/usr/bin/env python3
"""Scripted arm gestures for the free-play demo. **No model is involved.**

Pi0.5 is trained on LIBERO tabletop manipulation demonstrations. Prompts like
"wave hi" are outside that distribution: the policy will emit *something*, but
nothing in its training data teaches a greeting. So gestures the audience asks
for socially are produced here, by hand-written open-loop trajectories in the
same action space the VLA writes to, and the UI labels them `scripted` so the
distinction is never blurred.

Action space (LIBERO / robosuite ``OSC_POSE`` on a Panda, 7-DoF):
  ``[dx, dy, dz, droll, dpitch, dyaw, gripper]``, each in [-1, 1].
  Position deltas scale to +/-0.05 m per step, rotation to +/-0.5 rad per step,
  gripper is -1 open / +1 closed.

Everything here is pure numpy: it does not import LIBERO, ROS, or torch, so it
is testable on its own.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

OPEN = -1.0
CLOSED = 1.0


def _seg(steps: int, *, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0,
         droll: float = 0.0, dpitch: float = 0.0, dyaw: float = 0.0,
         grip: float = OPEN) -> np.ndarray:
    """``steps`` copies of one constant 7-DoF action."""
    action = np.array([dx, dy, dz, droll, dpitch, dyaw, grip], dtype=np.float64)
    return np.tile(action, (steps, 1))


def _wave_hi() -> np.ndarray:
    """Lift the gripper high, then swing it side to side four times.

    The sway runs one full amplitude for 7 steps a side: at 0.85 for 5 steps the
    end effector only travels ~5 cm, which reads as a twitch rather than a wave
    from the back of a room.
    """
    segments = [_seg(12, dz=0.85), _seg(3, dz=0.2)]
    for _ in range(4):
        segments.append(_seg(7, dy=1.0))
        segments.append(_seg(7, dy=-1.0))
    segments.append(_seg(8, dz=-0.5))
    return np.concatenate(segments)


def _nod_yes() -> np.ndarray:
    """Pitch the wrist down/up three times, like a nod."""
    segments = [_seg(8, dz=0.5)]
    for _ in range(3):
        segments.append(_seg(4, dpitch=0.7))
        segments.append(_seg(4, dpitch=-0.7))
    return np.concatenate(segments)


def _shake_no() -> np.ndarray:
    """Yaw the wrist left/right three times, like a head shake."""
    segments = [_seg(8, dz=0.5)]
    for _ in range(3):
        segments.append(_seg(4, dyaw=0.8))
        segments.append(_seg(4, dyaw=-0.8))
    return np.concatenate(segments)


def _bow() -> np.ndarray:
    """Reach forward and down, then come back up."""
    return np.concatenate([
        _seg(10, dx=0.5, dz=-0.45, dpitch=0.25),
        _seg(6),
        _seg(10, dx=-0.5, dz=0.45, dpitch=-0.25),
    ])


def _reach_up() -> np.ndarray:
    return np.concatenate([_seg(16, dz=0.8), _seg(6)])


def _reach_left() -> np.ndarray:
    return np.concatenate([_seg(14, dy=0.8), _seg(6)])


def _reach_right() -> np.ndarray:
    return np.concatenate([_seg(14, dy=-0.8), _seg(6)])


def _reach_forward() -> np.ndarray:
    return np.concatenate([_seg(14, dx=0.7), _seg(6)])


def _pump_gripper() -> np.ndarray:
    """Open/close the fingers three times in place."""
    segments = []
    for _ in range(3):
        segments.append(_seg(6, grip=CLOSED))
        segments.append(_seg(6, grip=OPEN))
    return np.concatenate(segments)


@dataclass(frozen=True)
class Gesture:
    name: str
    label: str
    description: str
    build: object  # callable returning (N, 7) float64

    def actions(self) -> np.ndarray:
        return np.asarray(self.build(), dtype=np.float64)  # type: ignore[operator]


GESTURES: dict[str, Gesture] = {g.name: g for g in (
    Gesture("wave_hi", "wave hi",
            "lift the gripper and swing it side to side", _wave_hi),
    Gesture("nod_yes", "nod yes", "pitch the wrist down and up", _nod_yes),
    Gesture("shake_no", "shake no", "yaw the wrist left and right", _shake_no),
    Gesture("bow", "bow", "reach forward and down, then return", _bow),
    Gesture("reach_up", "reach up", "raise the end effector", _reach_up),
    Gesture("reach_left", "reach left", "move the end effector left", _reach_left),
    Gesture("reach_right", "reach right", "move the end effector right", _reach_right),
    Gesture("reach_forward", "reach forward", "extend toward the table front", _reach_forward),
    Gesture("pump_gripper", "open and close the gripper",
            "cycle the fingers three times in place", _pump_gripper),
)}


def catalog() -> list[dict[str, object]]:
    """UI-facing description of every scripted gesture."""
    return [
        {"name": g.name, "label": g.label, "description": g.description,
         "steps": int(len(g.actions()))}
        for g in GESTURES.values()
    ]


if __name__ == "__main__":
    for gesture in GESTURES.values():
        a = gesture.actions()
        assert a.ndim == 2 and a.shape[1] == 7, gesture.name
        assert np.abs(a).max() <= 1.0, gesture.name
        print(f"{gesture.name:16s} {len(a):3d} steps  "
              f"range [{a.min():+.2f}, {a.max():+.2f}]")
