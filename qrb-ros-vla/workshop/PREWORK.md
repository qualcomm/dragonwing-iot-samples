# Pre-work — Pi0.5 VLA on the Dragonwing IQ-9075

**Deadline: complete this at least 7 days before the workshop.**

This is not optional preparation you can skip and catch up on. The workshop is BYOD — nobody can
pre-flash your board for you — and it downloads **~2.9 GB of model weights**. On a shared venue
network, 30 people doing that simultaneously takes over five hours. If you arrive without this done,
**you cannot follow along**; you will be paired with someone who did.

The whole thing is six steps and about **45 minutes of waiting**, most of it unattended. Step 5b is
optional and only needed for the last stretch module.

At the end you will paste **one line** into the event channel. That line is the only thing we need
from you.

---

## What you need

| Item | Notes |
| --- | --- |
| Qualcomm Dragonwing IQ-9075 EVK | Flashed and booting Ubuntu 24.04 Server or newer, arm64 |
| USB-to-serial cable | Proven working at least once (see step 1) |
| Network on the board | ~3 GB of downloads, wired strongly preferred (~3.5 GB if you do the optional Step 5b) |
| 30 GB free disk | The preflight script checks and reports the actual figure |
| A laptop | For SSH; no GUI is needed on the board at any point |

> **Do not attempt to flash the board the day before.** Flashing is a multi-stage,
> multi-power-cycle host-tool sequence that erases the device. If your board is not already booting
> Ubuntu, start that process **now** and ask for help in the event channel — not in the last 48 hours.

---

## Step 1 — Prove serial, then SSH

Serial is your recovery path when networking breaks. Prove it works once, now, while it is not
urgent.

**The board exposes four serial devices. The Linux console is the second one.**

```
LAPTOP
sudo screen /dev/ttyUSB1 115200
```

<details>
<summary>If that shows nothing</summary>

- On Windows you likely need the FTDI VCP driver installed manually.
- `/dev/ttyUSB0` is usually the SAIL console, not the main Linux console. It will look alive but
  will not give you an Ubuntu login prompt.
- Try `ls /dev/ttyUSB*` and work through them; the main console is the one with a `ubuntu login:`
  prompt after a reboot.

</details>

**Change the default password.** A stock `ubuntu`/`ubuntu` login blocks SSH key setup and is the
single most common cause of "I can't connect" on the day:

```
DEVICE
passwd
```

Then confirm SSH from your laptop and keep the address handy:

```
LAPTOP
export DEV_IP=<the board's IP from `ip addr` on the serial console>
ssh ubuntu@$DEV_IP "uname -a && nproc"
```

Everything from here runs on the board over SSH.

---

## Step 2 — Get the repository

```
DEVICE
sudo apt-get update
sudo apt-get install -y git
git clone https://github.com/qualcomm/dragonwing-iot-samples.git ~/dragonwing-iot-samples
cd ~/dragonwing-iot-samples/qrb-ros-vla
```

---

## Step 3 — Install ROS 2 Jazzy and the Qualcomm runtime

Two scripts, both idempotent — safe to re-run if they fail partway. Roughly 15 minutes.

```
DEVICE
cd ~/QRB-ROS-VLA
./scripts/install-ros2-jazzy.sh
./scripts/install-qrb-ros.sh
sudo apt-get install -y qairt-libs qairt-tools qairt-headers libsentencepiece-dev python3-pil
```

**Expected output** at the end of the first script:

```
done: ROS 2 jazzy installed, rmw=default, ros2 cli version ...
```

<details>
<summary>If apt fails with <code>Conflicting values set for option Trusted</code></summary>

The `qcom-ppa` repository is already on the stock image and something has added it a second time.
Remove the duplicate:

```
DEVICE
ls /etc/apt/sources.list.d/
sudo rm /etc/apt/sources.list.d/<the duplicate qcom-ppa file>
sudo apt-get update
```

</details>

<details>
<summary>If apt fails with <code>NO_PUBKEY</code></summary>

Import the key it names and retry:

```
DEVICE
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys <THE_KEY_ID_FROM_THE_ERROR>
sudo apt-get update
```

</details>

<details>
<summary>If <code>packages.ros.org</code> fails TLS verification</summary>

Expected on some networks — `packages.ros.org` resolves to a mirror whose certificate covers a
different name. `install-ros2-jazzy.sh` detects this and falls back to `http://`, which is what the
official Qualcomm setup page uses too. Package integrity still comes from GPG signatures. No action
needed.

</details>

---

## Step 4 — Download the model (~2.9 GB, the long one)

The one-shot setup script can do this for you. If you are following the workshop pre-work module by
module, run the artifact phase now so the live room never waits for AI Hub downloads.

```
DEVICE
cd ~/QRB-ROS-VLA
python3 -m venv .venv-tools
.venv-tools/bin/python -m pip install --upgrade pip qai-hub-models pyarrow pillow 'numpy<3' uv
./scripts/fetch-tokenizer.sh
cd artifacts
../.venv-tools/bin/qai-hub-models fetch Pi0.5 --runtime qnn_context_binary --precision mixed --chipset qualcomm-qcs9075
cd ..
```

This requires Qualcomm AI Hub access for `qai-hub-models`. If the fetch fails, authenticate for AI Hub
or copy a pre-fetched `pi05-qnn_context_binary-mixed-qualcomm_qcs9075/` directory from the offline
bundle.

**Expected output:** `artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075/` containing four
`.bin` files and `metadata.json`, totalling roughly 2.9 GB.


---

## Step 5 — Prepare the demo episode

The demo replays a real robot demonstration. Converting it needs `pyarrow`, so use the helper venv
created in step 4:

```
DEVICE
cd ~/QRB-ROS-VLA
PYTHON=.venv-tools/bin/python ./scripts/fetch-libero-episode.sh /tmp/libero
cd artifacts
sha256sum paligemma_tokenizer.model \
  pi05-qnn_context_binary-mixed-qualcomm_qcs9075/action_expert.bin \
  pi05-qnn_context_binary-mixed-qualcomm_qcs9075/backbone.bin \
  pi05-qnn_context_binary-mixed-qualcomm_qcs9075/token_emb.bin \
  pi05-qnn_context_binary-mixed-qualcomm_qcs9075/vision_encoder.bin \
  pi05-qnn_context_binary-mixed-qualcomm_qcs9075/metadata.json \
  libero_ep0.npz > manifest.sha256
```

**Expected output:**

```
wrote artifacts/libero_ep0.npz (29.3 MB, 200 frames, images 256x256)
```

---

## Step 5b `[STRETCH]` — Install the simulator, if you want Module 8

Module 8 is the last `[STRETCH]` module: instead of scoring the model's predictions against a human's,
you let it drive a simulated Franka arm and see whether it actually completes the task. Skip this if
you only want the required path — **nothing in Modules 0–7 needs it.**

It adds **~520 MB** of downloads on top of Step 4 and about **1.9 GB on disk**, and takes a couple of
minutes. Everything is wheels; nothing compiles.

```
DEVICE
cd ~/QRB-ROS-VLA
./scripts/install-libero.sh
```

**Expected output** (the `SyntaxWarning` and three `[robosuite WARNING]` lines are normal and
harmless — robosuite prints them on every import):

```
==> apt: OSMesa
==> venv at /home/ubuntu/libero/venv
==> pinned dependency set
==> LIBERO source at /home/ubuntu/libero/LIBERO
==> config (import calls input() without it and hangs)
==> env file /home/ubuntu/libero/libero-env.sh
==> smoke test
.../robosuite/__init__.py:30: SyntaxWarning: invalid escape sequence '\ '
[robosuite WARNING] No private macro file found! (__init__.py:7)
[robosuite WARNING] It is recommended to use a private macro file (__init__.py:8)
[robosuite WARNING] To setup, run: python .../robosuite/scripts/setup_macros.py (__init__.py:9)
    mujoco 3.2.7  robosuite 1.4.0  numpy 2.4.6
    offscreen render OK: (256, 256, 3) uint8
LIBERO installed at /home/ubuntu/libero (1.9G); source /home/ubuntu/libero/libero-env.sh before use.
```

The last two lines are the ones that matter. **`offscreen render OK` is the real check** — it means
MuJoCo found a working software OpenGL context on your board, which is the part that is not obvious
and the part that fails on other hardware. If it fails there, paste the output into the event channel;
do not wait for the day.

> **The `mujoco==3.2.7` pin is not conservatism.** mujoco 3.3.0 renamed `MjData.qM` to `M` and
> robosuite 1.4.0 still uses the old name, so anything newer dies when the environment is
> constructed. A plain `pip install mujoco` resolves to a much later version and will not work.

---

## Step 6 — Run preflight and paste one line

```
DEVICE
cd ~/QRB-ROS-VLA
./workshop/scripts/preflight.sh
```

It takes about 3 seconds and prints `[PASS]` / `[FAIL]` per check. **Every `[FAIL]` line tells you
the exact command to fix it.** Fix them and re-run until the last line says `OK`:

```
Paste this one line into the event channel:
PREFLIGHT v2 9075 OK 22/22 2f369c domain=64 sim=yes
```

**Paste that line into the event channel.** If it says `INCOMPLETE`, paste it anyway along with the
`[FAIL]` lines — we will help you before the day.

---

## Optional but recommended: see it run before you arrive

If you want the satisfaction early — and the confidence — this builds the workspace and starts the
VLA for real. About three minutes, mostly the build:

```
DEVICE
cd ~/QRB-ROS-VLA
./workshop/scripts/fastforward.sh 1
```

**Expected output:**

```
CP-1 PASS: node is publishing action chunks
```

That is a 3-billion-parameter vision-language-action model running on your board. If you see it, you
are completely ready.

Stop the background processes afterwards:

```
DEVICE
pkill -f vla_node; pkill -f synthetic_publisher
```

---

## Bring to the room

- The board, its power supply, and the serial cable
- An Ethernet cable if you have one
- The `PREFLIGHT ... OK` line already posted

## No network at the venue?

We design for zero internet. There will be USB sticks (one per four attendees) with every artifact,
and a local wired mirror. If you completed the steps above, you need neither.
