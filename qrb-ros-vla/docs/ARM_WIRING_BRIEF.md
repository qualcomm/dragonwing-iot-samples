# Brief for the hardware/wiring agent

Paste this whole file into a fresh agent session. It is an **electrical design and review task**, not
a software task and not a build task. Nothing is to be wired until the open questions in §5 are
answered, because the failure mode is a dead board.

Your counterpart agent runs on the Qualcomm Dragonwing IQ-9075 EVK itself and has already mapped the
software side. Everything in §1 and §2 was read off that board or from Qualcomm's published docs —
treat it as given and do not re-derive it. Everything marked **UNVERIFIED** is genuinely open and is
what we need you for.

> **READ THIS FIRST — the policy side has since been measured, and it changes the brief.**
> A simulation study (`docs/DESIGN.md` §12) established that **this arm cannot run the target tasks at
> all**: they need 604–751 mm of reach from the robot base and the arm has ~300 mm, so 0 of 10 tasks
> fit, by a factor of 2.0–2.5×. Two consequences for the questions below:
>
> - **§5.5 (PWM frame rate) is settled — 50 Hz is fine.** The earlier "≥200 Hz" recommendation was
>   withdrawn after measurement. Skip it.
> - **§5.8 (calibration) and §5.6 (slew) are low priority.** Slew has 31× margin; the arm is not
>   speed-limited.
>
> Everything electrical below still stands and is still worth answering — §5.1 (logic level), §5.3
> (power), §5.4 (e-stop), §5.7 (grounding) — because the arm is still worth driving as a demonstrator,
> and the failure modes there can destroy hardware. But do not treat "make this arm do the task" as the
> goal, because it cannot.

## 1. What we are connecting, and why the stakes are asymmetric

A 3B-parameter vision-language-action model (Pi0.5) runs on the board's two Hexagon NPUs at ~1.11 s
per 50-step action chunk. Today it drives a MuJoCo simulator. We want it to drive a real desktop
robot arm, with a camera on the claw.

Two things you should know about the context, because they should shape how conservative you are:

- **The policy is not trustworthy.** It was fine-tuned on a Franka Panda in simulation. Driving this
  arm is out-of-distribution for it, and we expect physically wrong commands — full-scale joint
  slams, sustained stall against a table, commands that fight gravity. **Design the electrical side
  assuming the commands are adversarial**, not assuming they are sane. This is not a normal
  "hobby servo project" threat model.
- **The board is shared and hard to replace.** It is the only IQ-9075 in the project and all other
  work is on it. A wiring mistake that kills the SoC stops everything. Prefer an extra $15 part over
  a clever direct connection.

## 2. Pinned facts — verified, do not re-guess

### Board

| Property | Value |
|---|---|
| Board / SoC | Qualcomm Dragonwing IQ-9075 EVK / QCS9075 (`qcom,sa8775p`) |
| OS / kernel | Ubuntu 24.04.4 LTS **Server**, aarch64, `6.8.0-1080-qcom` |
| Headless | No display, no GUI. SSH and serial only |
| Boot time | ~24 s — a reboot is a cheap recovery step |
| I2C character devices present | `/dev/i2c-18`, `-19`, `-20` (all report name `Geni-I2C`) |
| Also present, **not usable** | `/dev/i2c-21`, `-22` — these report `dpu_dp_aux`, i.e. DisplayPort AUX |
| `/dev/spidev*` | **absent** |
| `/dev/gpiochip*` | `0`–`9` present |
| `i2c-tools` | **not installed** (`i2cdetect` unavailable until `apt install i2c-tools`) |

### Expansion connector (from Qualcomm docs, not yet probed)

- One 40-pin low-speed connector, designated **JLS1**.
- **Pins 8 and 10 are I2C by default**, GPIO lines 32 and 33, mapping to `i2c4 = qup0_se4`
  at address `0x990000`.
- Qualcomm's page says a "Modify serial engine node" device-tree procedure may be required to enable
  the interface, and that after enabling you should expect `/dev/i2c-18` through `/dev/i2c-25`.
  **We only see 18–22.** Their worked example scans bus 20.
- GPIO numbering: `gpiochip4` is `platform/f000000.pinctrl` with base **560**. An LS connector pin's
  subsystem number is 560 + its GPIO line number. (Docs' example: LS pin 5 = GPIO 54 = 614.)
- Source: <https://dragonwingdocs.qualcomm.com/Ubuntu/devices/iq9075-evk/peripherals-interfaces/Low-Speed-Connectors>

### The arm — VUPN2355 6-DOF desktop arm ($329, on hand)

| Property | Value |
|---|---|
| Servos | 6 × **TD-8125MG**, identical |
| Travel | base 270°; the other five 180° each |
| Control | **PWM, 500–2500 µs pulse, 1500 µs centre** |
| Rated voltage | 5 V; operating range **DC 4.4–8.4 V** |
| Current, no load | 210 mA typ / 260 mA max **per servo** |
| Current, stall | **2600 mA typ / 3400 mA max per servo** |
| Stall torque | 23.5 kg·cm typ / 26.8 kg·cm max |
| Quoted consumption | ">2000 mA" (we read this as marketing, not a budget) |
| Extent | ~37 × 11 × 12 cm fully extended |
| Position feedback | **none** — no encoders, nothing reported back |
| SDK | none supplied |

Note: "6 DOF" with 6 servos almost certainly means **5 positioning joints + gripper**. Treat the
gripper as one of the six PWM channels.

### The driver board — HUAREW PCA9685 (2 on hand)

16-channel, 12-bit PWM over I2C, standard Adafruit-style breakout with a screw-terminal V+ for the
servo rail. Default address `0x40`. Has `OE` and reset lines.

### Timing the software side has already fixed

- Policy emits **50 actions per chunk**, played back at **10 Hz** (this rate is empirically
  established, not a guess — 20 Hz playback degrades tracking error from 9 mm to 207 mm).
- So: one chunk = **5.0 s of motion** against **~1.11 s of inference** — roughly a 22% duty cycle.
  The NPU comfortably outruns the arm; there is no need to optimise the control path for latency.
- Commands therefore arrive as **absolute joint targets at 10 Hz** (one every 100 ms), from the
  software side's point of view.

## 3. The intended topology

This is our starting sketch, not a decision. Correct it.

```
IQ-9075 JLS1 pin 8  (I2C SDA) ──┐
IQ-9075 JLS1 pin 10 (I2C SCL) ──┤
IQ-9075 JLS1 ground            ─┤
                                └──► [ level translator? ] ──► PCA9685 (0x40)
                                                                 │  ch 0..5
                                                                 ▼
                                                              6 × TD-8125MG
                                                                 ▲
                                         separate 6 V bench supply ──► PCA9685 V+ screw terminal
                                                 (with e-stop in this rail)

USB camera on the claw ──────────────────────► board USB (powered hub?)
```

## 4. Hard constraints

- **Never power the servos from the board.** Any rail, any pin, any current. The servo supply is
  separate, full stop.
- **Do not wire the PCA9685 to the LS connector until §5.1 is settled.** If the LS I2C is 1.8 V and
  the PCA9685 is at 5 V, connecting them can push 5 V into SoC pads.
- **Do not propose different hardware.** The arm, the PCA9685s and the board are what we own. You may
  propose *additional* small parts (translators, supplies, capacitors, connectors, e-stop hardware) —
  that is expected and welcome.
- **Do not design the kinematics, IK, or policy-to-joint mapping.** Separate problem, separately hard
  (the arm is 5-DOF and the policy emits 6-DOF Cartesian deltas). Out of scope here. Your output must
  be correct regardless of what the software eventually commands.
- **Do not assume the arm will ever work well.** The wiring has to be safe and measurable even if the
  policy is nonsense, because for a while it will be.
- Report **UNVERIFIED** honestly. Do not smooth over a gap you could not close — say which
  measurement or datasheet would close it. We would much rather have five confident answers and four
  flagged unknowns than nine confident-sounding ones.

## 5. Open questions — this is the actual work

### 5.1 Logic level (blocking everything else)

SA8775P TLMM IO is *believed* to be 1.8 V, but we could **not confirm this from the docs text** — the
level appears only inside a pinout image we cannot read programmatically. The PCA9685 needs
VDD 2.3–5.5 V and its I²C input thresholds scale with VDD.

- What is the JLS1 I2C logic level, and what source establishes it authoritatively?
- Given the answer, specify the translator concretely. Is a PCA9306 or TXS0102 the right choice for
  bidirectional I2C at this level shift? Something else?
- Pull-up resistor values on **each** side, and where they physically live. Note the Qualcomm docs
  mention `TLMM_GPIO_PULL_UP` at 2 mA drive for I2C — does the SoC side already pull up, and does
  that change the sizing?
- Bus speed we should ask for, given we only need 6 channel writes per 100 ms tick.

### 5.2 Which bus, and does it need a device-tree change

`/dev/i2c-18/19/20` already exist. Qualcomm's procedure implies a device-tree edit may still be
needed for the LS connector specifically.

- Which of 18/19/20 (if any) is JLS1 pins 8/10? How do we determine this **without** connecting the
  arm — is there a safe probe (an I2C EEPROM, a logic analyser, `i2cdetect` on an empty bus)?
- Is the "Modify serial engine node" step actually required here, and what exactly does it change?
- Docs expect `i2c-18`…`i2c-25`; we have 18–22. Is that a real discrepancy or a docs-vs-Ubuntu
  difference we can ignore?

### 5.3 Power budget and inrush

Six servos at 3400 mA absolute worst case is 20.4 A. That is a stall-all-six number we hope never to
hit, but see §1 — assume the commands are adversarial.

- What supply do you actually specify (voltage within 4.4–8.4 V, and current)? Justify the derating
  from 20.4 A rather than just picking a round number. Note higher voltage in that range buys speed
  and torque but also more stall current and heat.
- Bulk capacitance at the PCA9685 V+ terminal, and per-servo decoupling if any.
- **Power-on inrush**: at power-up all six servos snap to whatever the PCA9685 outputs before
  software initialises them. What does that do, and how do we prevent it? Soft-start? Hold the servo
  rail off until software is ready?
- Wire gauge and connector type for a rail that may carry >10 A.
- Thermal: TD-8125MG at partial stall holding an outstretched arm against gravity is the steady state
  we should design for, not the no-load figure. Does anything need airflow?

### 5.4 E-stop and failsafe — treat this as a first-class requirement

A neural network with no position feedback will command physically impossible things. We need to be
able to stop the arm *without* relying on a graceful software path, because the software may be the
thing that is wrong.

- Topology for a hardware kill that cuts the **servo rail only**, leaving the board, the PCA9685
  logic and the I2C bus alive so we keep telemetry and don't have to re-init the bus. Relay?
  High-side MOSFET? Latching mushroom switch inline?
- What happens to PCA9685 outputs if the controlling process dies mid-chunk — do they hold the last
  value indefinitely? Does the arm stay energised in a stalled pose until someone notices?
- Should `OE` be driven from a board GPIO as a software watchdog (GPIO goes low → outputs
  tri-state → servos unpowered-idle)? If so, which LS pin, and what's the safe polarity so that a
  board reset or an unconfigured GPIO defaults to **outputs off**?
- Anything to prevent the arm dropping under gravity when the rail is cut — or do we just accept that
  and keep the workspace clear?

### 5.5 PWM frame rate vs resolution — **SETTLED, 50 Hz is fine**

**This question is answered; do not spend time on it.** It is left in place because the reasoning is
counter-intuitive and the first answer we gave was wrong.

PCA9685's 12 bits span one PWM *period*, so a shorter period buys finer pulse resolution:

- At 50 Hz: 20000 µs / 4096 ≈ 4.88 µs per count → the 500–2500 µs span is only ~410 counts →
  **~0.44° per count** over 180°.
- At ~330 Hz: 3030 µs / 4096 ≈ 0.74 µs per count → ~2700 counts → ~0.067°.

Since the policy commands a median of 0.54° of rotation per step, 0.44° per count is barely one count,
and we initially concluded the rail had to run at ≥200 Hz. **We then measured it in closed loop and it
made no difference: 43/50 vs an 86/100 baseline, +0.0 points** (`docs/DESIGN.md` §12c). The resolution
argument describes open-loop fidelity, but the policy replans every 10 steps from a fresh observation
and the impedance controller chases a target rather than integrating a velocity, so the quantization
error is corrected instead of accumulating.

**So: run the servo rail at a plain 50 Hz.** No need to verify high-frame-rate support, and one less
thing to get wrong. If you want the finer quantum for other reasons (audible jitter, holding torque),
it is a free choice rather than a requirement — but then do check that the TD-8125MG accepts it.

### 5.6 Slew rate — can the servo even track a 10 Hz command stream?

Commands arrive every 100 ms as absolute targets. A servo of this class is roughly 0.16 s/60°, so a
60° step needs ~160 ms and the next command arrives before it lands.

- What is the largest joint delta per 100 ms tick this arm can actually track? A per-joint figure is
  better than one number, since the base carries the whole arm.
- Should the software interpolate between 10 Hz waypoints at a higher PWM update rate, and if so at
  what rate? Or is commanding a target and letting the servo's own loop chase it correct here?
- What does exceeding the trackable rate look like physically — stutter, overshoot, current spikes,
  audible buzzing? We want to be able to recognise it.

### 5.7 Grounding and the camera

- Ground topology between the servo supply, the PCA9685 and the board. Star point where?
- The claw camera is USB into the board. Any ground-loop or noise concern with a 10 A switching
  servo rail on the same chassis? Does the camera need a powered hub rather than the board's port?
- Cable routing along a moving 37 cm arm: strain relief, service loop, and how to keep the USB cable
  from becoming a spring that fights the wrist servos.

### 5.8 Calibration without feedback

We have no encoders, so pulse-width → actual joint angle must be established by hand, once.

- Procedure to find each servo's true mechanical limits **without stalling it into the endstops** and
  cooking a gearbox.
- How to detect that we are approaching a limit or a stall, given the only observable is supply
  current. Is per-channel or total current sensing worth adding? What part?
- Does the base servo's 270° range vs the others' 180° change anything in the wiring, or is it purely
  a software mapping concern?

## 6. Deliverable

Reply with, in this order:

1. **A pin-by-pin wiring table**: board LS pin → translator pin → PCA9685 pin, including grounds and
   pull-ups. Every row explicit; no "and connect power as usual".
2. **A power and e-stop schematic** in ASCII or a clear textual netlist, showing where the kill
   switch sits and what stays alive when it opens.
3. **A shopping list** of parts we don't have, with a rough price each and a one-line reason.
4. **A bring-up order**: the sequence of steps from bare parts to a first commanded servo motion,
   with what to measure at each step before proceeding to the next. Assume a multimeter is available;
   say if you need a scope or a logic analyser.
5. **An UNVERIFIED list**: every question above you could not close, and what specific measurement or
   document would close it.

Prefer "measure this before you connect that" over a finished design that assumes the best case.
