#!/usr/bin/env python3
"""Compose a recorded closed-loop rollout into an mp4 with a HUD burned in.

Runs entirely offline over what `demo/libero_closed_loop.py --save-frames` wrote, which is
the point: the policy is never re-run, the compositing cost never lands inside a measured
episode, and the styling can be changed without another 50 s of NPU time.

The HUD carries the things that make a rollout legible as *evidence* rather than just a
robot arm moving:

  * the task string the policy was actually conditioned on
  * the 50-action chunk as a strip, with the executed prefix shaded and a cursor on the
    action being applied right now -- so the replan boundaries are visible, and so is the
    fact that most of each chunk is discarded
  * the per-stage NPU latency from InferenceStats, including the backend, which is
    proof-of-NPU attached to the frame rather than asserted in prose
  * a SUCCESS banner the moment LIBERO's own goal predicate flips

Usage:
    demo/make_rollout_video.py /tmp/rollout/task0_init0 --out /tmp/rollout.mp4
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

# Qualcomm palette, matching the blog's mermaid diagrams and poster assets.
QC_PURPLE = (49, 1, 125)
QC_LIGHT = (244, 239, 250)
QC_GREY = (107, 107, 118)
QC_ACCENT = (232, 163, 61)
QC_OK = (23, 122, 60)
WHITE = (255, 255, 255)

FONT_CANDIDATES = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
]
MONO_CANDIDATES = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
]


def load_font(size: int, mono: bool = False):
    for path in (MONO_CANDIDATES if mono else FONT_CANDIDATES):
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    # PIL's built-in bitmap font ignores size, so the HUD will look cramped -- but the
    # video still builds, which beats failing on a font lookup.
    print("warning: no TrueType font found; falling back to PIL's bitmap font",
          file=sys.stderr)
    return ImageFont.load_default()


def wrap(draw, text: str, font, max_w: int) -> list[str]:
    words, lines, cur = text.split(), [], ""
    for w in words:
        trial = f"{cur} {w}".strip()
        if draw.textlength(trial, font=font) <= max_w or not cur:
            cur = trial
        else:
            lines.append(cur)
            cur = w
    if cur:
        lines.append(cur)
    return lines


def compose(frame_paths: list[Path], row: dict, ep: dict, scale: int,
            chunk_cache: dict) -> Image.Image:
    tiles = [Image.open(p).convert("RGB") for p in frame_paths]
    res = ep["res"]
    tw = res * scale
    tiles = [t.resize((tw, tw), Image.NEAREST) for t in tiles]

    # Sized to fit the content (a two-line task string plus four rows), not to a round
    # fraction -- 0.62 left half the panel empty and the success banner floating in it.
    # Both dimensions must be even: libx264 with yuv420p subsamples chroma 2x2 and fails
    # outright on an odd size, which surfaces as a bare non-zero ffmpeg exit far from here.
    hud_h = int(tw * 0.38) // 2 * 2
    W = tw * len(tiles) // 2 * 2
    canvas = Image.new("RGB", (W, tw + hud_h), WHITE)
    for i, t in enumerate(tiles):
        canvas.paste(t, (i * tw, 0))

    d = ImageDraw.Draw(canvas)
    f_task = load_font(int(11 * scale))
    f_lbl = load_font(int(8 * scale))
    f_mono = load_font(int(8 * scale), mono=True)
    f_big = load_font(int(16 * scale))

    # Camera names, over the images so they are unambiguous.
    for i, cam in enumerate(ep["cameras"]):
        label = "agentview" if "agentview" in cam else "wrist"
        d.rectangle([i * tw + 4, 4, i * tw + 6 + int(d.textlength(label, font=f_lbl)) + 6,
                     4 + int(11 * scale)], fill=(0, 0, 0))
        d.text((i * tw + 8, 5), label, font=f_lbl, fill=WHITE)

    y = tw + int(5 * scale)
    pad = int(7 * scale)

    for line in wrap(d, ep["task"], f_task, W - 2 * pad)[:2]:
        d.text((pad, y), line, font=f_task, fill=QC_PURPLE)
        y += int(13 * scale)

    y += int(2 * scale)
    d.text((pad, y),
           f"chunk {row['chunk']:>3}   sim step {row['sim_step']:>3} / {ep['max_steps']}"
           f"   executing {row['action_in_chunk']:>2} of {row['horizon']}",
           font=f_mono, fill=QC_GREY)
    y += int(12 * scale)

    # --- the action chunk strip -------------------------------------------------
    # 50 cells: shaded = the prefix that will actually be executed before the next
    # replan, outlined = predicted but discarded, filled = the action being applied now.
    if row.get("chunk_actions"):
        chunk_cache["actions"] = row["chunk_actions"]
    n_act = len(chunk_cache.get("actions") or []) or 50
    strip_w = W - 2 * pad
    cw = strip_w / n_act
    h = int(9 * scale)
    for i in range(n_act):
        x0 = pad + i * cw
        inside = i < row["horizon"]
        now = i == max(0, row["action_in_chunk"] - 1)
        d.rectangle([x0, y, x0 + cw - max(1, cw * 0.18), y + h],
                    fill=QC_ACCENT if now else (QC_LIGHT if inside else WHITE),
                    outline=QC_PURPLE if inside or now else (215, 210, 225),
                    width=1)
    y += h + int(3 * scale)
    d.text((pad, y), f"50-action chunk: {row['horizon']} executed, "
                     f"{n_act - row['horizon']} discarded at replan",
           font=f_lbl, fill=QC_GREY)
    y += int(12 * scale)

    # --- NPU latency, per stage ------------------------------------------------
    st = row.get("stats") or chunk_cache.get("stats")
    if st:
        chunk_cache["stats"] = st
        d.text((pad, y),
               f"NPU {st['total_ms']:7.1f} ms   vision {st['vision_encoder_ms']:5.1f}"
               f"   tok {st['token_emb_ms']:4.1f}   backbone {st['backbone_ms']:5.1f}"
               f"   expert {st['action_expert_ms']:5.1f}",
               font=f_mono, fill=QC_PURPLE)
        y += int(11 * scale)
        d.text((pad, y), f"backend {st['backend']}", font=f_mono, fill=QC_GREY)

    if row.get("success"):
        # Over the imagery, not in the HUD: the banner is the punchline and belongs where
        # the eye already is.
        txt = "SUCCESS"
        tl = d.textlength(txt, font=f_big)
        bw, bh = tl + int(20 * scale), int(26 * scale)
        bx, by = (W - bw) // 2, tw - bh - int(8 * scale)
        d.rectangle([bx, by, bx + bw, by + bh], fill=QC_OK)
        d.text((bx + int(10 * scale), by + int(3 * scale)), txt, font=f_big, fill=WHITE)

    return canvas


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("rollout_dir", help="a task*_init* directory from --save-frames")
    ap.add_argument("--out", default="/tmp/rollout.mp4")
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--scale", type=int, default=2, help="integer upscale of each tile")
    ap.add_argument("--poster", default="", help="also write a poster PNG here")
    args = ap.parse_args()

    root = Path(args.rollout_dir)
    ep_path, meta_path = root / "episode.json", root / "steps.jsonl"
    for p in (ep_path, meta_path):
        if not p.exists():
            sys.exit(f"missing {p}; was this directory written by --save-frames?")
    ep = json.loads(ep_path.read_text())
    rows = [json.loads(l) for l in meta_path.read_text().splitlines() if l.strip()]
    if not rows:
        sys.exit("steps.jsonl is empty")

    if not shutil.which("ffmpeg"):
        sys.exit("ffmpeg not found")

    print(f"{len(rows)} frames  task: {ep['task']!r}")
    tmp = Path(tempfile.mkdtemp(prefix="hud-"))
    chunk_cache: dict = {}
    wrote_poster = False  # NOT Path.exists(): a stale poster from an earlier run would
                          # then survive silently, which is how a wrong image ships.
    try:
        for k, row in enumerate(rows):
            paths = [root / "frames" / f"{row['frame']:05d}_{cam}.png"
                     for cam in ep["cameras"]]
            missing = [p for p in paths if not p.exists()]
            if missing:
                print(f"  skipping frame {row['frame']}: missing {missing[0].name}",
                      file=sys.stderr)
                continue
            img = compose(paths, row, ep, args.scale, chunk_cache)
            img.save(tmp / f"{k:05d}.png")
            if args.poster and row.get("success") and not wrote_poster:
                img.save(args.poster)
                wrote_poster = True
        built = sorted(tmp.glob("*.png"))
        if not built:
            sys.exit("no frames composed")
        # Hold the final frame so a success banner is readable rather than a flash.
        last = built[-1]
        for extra in range(1, args.fps + 1):
            shutil.copy(last, tmp / f"{len(built) + extra:05d}.png")

        w, h = Image.open(built[0]).size
        cmd = ["ffmpeg", "-v", "error", "-y", "-framerate", str(args.fps),
               "-pattern_type", "glob", "-i", str(tmp / "*.png"),
               "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", args.out]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if proc.returncode != 0:
            # Do not let ffmpeg fail silently behind -v error; the usual cause is an odd
            # frame dimension, so report the size alongside whatever it said.
            sys.exit(f"ffmpeg failed (exit {proc.returncode}) on {w}x{h} frames:\n"
                     f"{proc.stderr.strip() or '(no stderr)'}")
        size = Path(args.out).stat().st_size / 1e6
        print(f"wrote {args.out} ({size:.1f} MB, {len(built)} frames @ {args.fps} fps)")
        if args.poster and not wrote_poster:
            # No success frame in this rollout, so fall back to the midpoint. Always
            # rewrite: never leave a previous run's poster in place.
            Image.open(built[len(built) // 2]).save(args.poster)
            print(f"wrote {args.poster} (no success frame; used the midpoint)")
        elif args.poster:
            print(f"wrote {args.poster} (success frame)")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
