# Instructor notes

Read this before delivering. It is the operational layer the attendee-facing README deliberately
omits.

## What this workshop is for

**Two takeaways, in priority order:**

1. *"I got a vision-language-action model running on an IQ-9075."*
2. *"QRB ROS made that easy, and it was mostly just ROS 2 I already knew."*

Everything is subordinate to those. In particular: **do not turn this into a tour of hardware
quirks.** There are genuinely interesting gotchas (a CDSP weight-mapping ceiling, an ABI break
between the apt package and upstream, an int32 limitation in the released deb). All of them are
**pre-solved** by the repo and the pre-work. Attendees should never meet them, because meeting them
teaches "this platform is fragile" — the opposite of the message.

If someone asks, answer honestly and point at `docs/DESIGN.md`. Do not volunteer it from the front.

**The shape of the session:**

- **Module 1 is the emotional peak.** A 3B robot foundation model running on their own board within
  25 minutes of sitting down. Protect this above everything.
- **Modules 2–3 build trust.** It's ordinary ROS 2 (2), and the outputs are actually good (3).
- **Modules 4–5 give agency.** One parameter makes it 3× faster (4), and the whole API is three calls
  so they can do it with their own model (5).

## Cut order, decided in advance

Cut in this order without renegotiating in the moment:

1. **Module 8** (closed-loop simulator) — cut first, despite being the most impressive thing in the
   room. It is the only module with a pre-work dependency not everyone will have done (`sim=yes` in
   their preflight digest), and a 2-episode rollout is ~2 minutes of watching. Run **one** episode
   from the front instead: the HUD video makes the same point in 30 seconds, and you can leave the
   command on screen for anyone who wants it later.
2. **Module 7** (bitwise verification) — most self-contained. Show the `VERIFIED / 45 of 45` output
   from the front in 60 seconds instead.
3. **Module 6** (denoise tuning) — cut third.
4. **Module 2's** `ros2 node info` / `ros2 topic hz` exploration — trim to just the task change, which
   is the part that lands.

**Never cut 0, 1, 3, 4, or 5.** Never cut a break to buy time — a tired room fails checkpoints and
costs you more than the ten minutes.

If you are running *early* (likely — the commands are fast), slow down and let people poke at topics.
Do not add material.

## Timing reality

Measured on a real IQ-9075, warm apt cache:

| Step | Measured |
| --- | --- |
| `preflight.sh` | 2.6 s |
| `colcon build` (3 packages, clean) | 56 s |
| Node startup (maps 2.9 GB onto both NPUs) | ~20 s |
| First action chunk after images start | ~1.1 s |
| LIBERO replay, 8 steps | ~40 s |
| Single-NPU benchmark | ~25 s |
| Dual-NPU benchmark | ~30 s |
| `minimal_npu` build + run | ~15 s |
| Bitwise verification | ~3 min |
| `install-libero.sh` (pre-work, warm cache) | ~2 min |
| Closed-loop episode, LIBERO-10, H=10 | ~51 s |
| Closed-loop episode **with `--save-frames`** | ~64 s |

**The commands are fast; the talking fills the time.** Every module has slack built in.

**Module 8 is the exception and you should know its shape before you stand up.** An episode is
~51 s of which you can say nothing useful — the arm is just moving. Budget it as: 2 min explaining
what closed-loop means and why Module 3 was not it, start two episodes, talk over them about the
render-vs-NPU cost inversion, then read the output together. Success is per-task: on this hardware we
measured **86/100 across the suite**, but individual tasks range from 10/10 down to 4/10, so if a
room runs `--task-index 9` they may see two failures and think they broke something. Tell them the
per-task rates first, or pin the module to task 0 (10/10 in both our bands).

## The two things that will actually go wrong

**1. `ROS_DOMAIN_ID` collisions.** Thirty boards on domain 0 merge into a single ROS graph. Attendees
then see each other's topics, and CP-1 can pass *for the wrong reason* — they are echoing someone
else's chunks. `preflight.sh` assigns and persists a per-board id, but attendees must open a new shell
(or re-source) for it to apply, **and it must be set in all three terminals**. Say this out loud at
the start of Module 1 and again at Module 2.

**2. Three terminals.** Modules 1–3 need Terminal A (node), Terminal B (publisher/replay), and
Terminal C (inspection). People lose track of which is which. Put the terminal labels on a slide and
leave it up.

## Helpers

**Never teach alone.** One helper per 12 attendees, minimum two people in the room.

**Sticky-note protocol.** Two colours per attendee. Stuck = one colour on the laptop lid; module done
= the other. You read the room in O(1) from the front without asking.

**Routing rules:**

- Helpers go *to* the attendee. Attendees never come to the front.
- Cannot fix it in **two minutes**? Run the escape hatch, move on, note the symptom for the retro.
- **Never debug one board from the front while 30 people wait.** Helper takes the board out-of-band
  and pairs that person with a neighbour.
- Helpers should have read TROUBLESHOOTING.md and know `fastforward.sh N` exists for every N.

## Room setup

```
       [ screen ]              [ instructor + wired switch ]
   +-------------------------------------------------------+
   |  row 1   # # # # # #        <- helper A covers rows 1-2 |
   |  row 2   # # # # # #                                   |
   |  row 3   # # # # # #        <- helper B covers rows 3-4 |
   |  row 4   # # # # # #                                   |
   +-------------------------------------------------------+
          power strips along every row (boards + laptops)
```

Easy to forget:

- **Two power outlets per seat** (board PSU + laptop). The most common venue failure.
- A GbE switch and enough cable if you are serving a local mirror.
- Terminal font must be readable from row 4 — check from the back before starting.

## Network

Design for **zero internet**. Assume venue Wi-Fi fails.

- **USB sticks** are the primary fallback: one per four attendees, exFAT, version-labelled, holding
  the model bundle, tokenizer, `libero_ep0.npz`, apt cache, and a git bundle.
- **Local mirror**: `export ARTIFACT_BASE=http://10.0.0.1:8080` re-points everything with one
  variable. Never hand-edit URLs.
- **Forbidden in live modules:** `sudo apt upgrade`, `rosdep update`, un-pinned `pip install`,
  `docker pull`. Any one can silently eat 20 minutes.

## Delivery

- **One new concept per ~12 minutes**; a checkpoint every 10–15.
- **Narrate commands as you type.** Never read the document aloud — attendees read faster than you
  speak and will resent it.
- Say the checkpoint ID at each transition: *"everyone should be at CP-3 now."*
- In Module 1, when the first chunk appears, **stop and let it land.** Point at `libQnnHtp.so` in the
  output and say what it means. That is the moment they came for.
- In Module 4, frame it as *"you have a second NPU, here is how to use it"* — a capability, not a bug
  workaround. The single-NPU number exists only to make the comparison legible.
- Be scrupulous in Module 3: it is **action accuracy on one episode, not a task success rate.**
  Attendees will repeat whatever you say, and overclaiming here discredits the good numbers.

## Before delivery — non-negotiable checklist

- [ ] Full stopwatch-timed dry run on a **freshly flashed** board within 48 hours, driven from
      `PREWORK.md` alone. Update the runtime table from measurement.
- [ ] Every escape hatch executed **from a genuinely broken state**, not a working one.
- [ ] Whole workshop completed with the **network cable unplugged**.
- [ ] `preflight.sh` run on a board deliberately missing things, to confirm the `[FAIL]` hints are
      correct and copy-pasteable.
- [ ] Three-terminal layout tested on the projector; labels on a slide.
- [ ] Module numbering unchanged since the last dry run. **Never renumber** — checkpoint IDs are
      referenced by troubleshooting, helpers, and attendees' notes.
- [ ] Slides carry the Module 4 comparison table, in case you cut the live demo.
- [ ] **Module 8: collect `sim=` from the preflight digests** before the day and know the count. If
      fewer than half the room has `sim=yes`, plan to demo it from the front rather than have people
      watch neighbours.
- [ ] **Module 8: the rollout video is on the slides**, so the module survives being cut entirely.
      Never put a required instruction in the video; attendees must be able to finish from the written lab.
- [ ] ≤5-question feedback form ready for the last 5 minutes.

## Known-fragile points

| Risk | Likelihood | Mitigation |
| --- | --- | --- |
| Attendee did not do pre-work | **High** | Pair with someone who did; never attempt a 2.9 GB download in the room |
| `ROS_DOMAIN_ID` collision | **High** | preflight assigns it; verify all three shells were re-sourced |
| Confusion over which terminal is which | Medium | Labels on a slide, left up all session |
| Second NPU not enumerated on a board | Low | That board does Modules 1–3 and 5 fine; it just misses the speedup |
| Thermal throttling late in the session | Medium | Latency drifts up; checkpoint bands are deliberately generous. Mention it rather than hiding it |
| Boards on differing Ubuntu point releases | Medium | All assertions are floors and patterns, never exact versions |
| A board's model download is subtly corrupt | Medium | preflight checks **checksums**, not existence — exactly why |

## Questions you will be asked

**"Is this a robot success rate?"** No. Module 3 is open-loop action accuracy on one episode. The
model never sees the consequences of its own actions. Closed-loop evaluation needs a simulator.

**"Can I run this on a real arm?"** Not this policy as-is. Pi0.5's published export targets a 7-DoF
Franka Panda; a different arm needs a policy trained for it. The *plumbing* transfers directly.

**"Can I run my own model?"** Yes, and this is the takeaway you want them to leave with. Export from
AI Hub, then either the three-call API or `qrb_ros_nn_inference` for single-input models.

**"Why is device 1 slower?"** Measured consistently at 25–30% across all four components; we do not
have a confirmed cause. Say it is measured and unexplained — do not speculate from the front.

**"Why not just use the apt `qrb_ros_nn_inference`?"** For a normal single-model pipeline, you should.
Pi0.5 needs a custom node because it is four chained models, one with 41 inputs. Mention that the
released deb is also older than upstream if pressed, but do not lead with it.

**"Could it be faster?"** Yes — ~223 ms/chunk is CPU-side tensor copying. `qrb_ros_transport`
zero-copy DMA-buf is the known next step and is not yet wired up.

## If you have extra time

Good improvisations, in order of value:

1. `ros2 bag record /qrb_ros_vla/action_chunk /qrb_ros_vla/stats`, then replay it. Reinforces "just
   ROS 2".
2. Change `"libQnnHtp.so"` to `"libQnnCpu.so"` in `minimal_npu.cpp` and rebuild — the NPU/CPU switch
   is one string.
3. Point `minimal_npu` at a different component (`action_expert.bin`) and watch it report 41 inputs'
   worth of expectations.
