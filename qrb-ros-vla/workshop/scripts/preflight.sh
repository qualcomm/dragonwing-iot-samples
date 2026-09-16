#!/usr/bin/env bash
# preflight.sh - verify an IQ-9075 is ready for the Pi0.5 VLA workshop.
#
# Idempotent, read-only, network-optional, finishes in well under 30 s.
# Prints [PASS] name  or  [FAIL] name: what to do about it.
#
# Ends with one machine-readable digest line to paste into the event channel:
#   PREFLIGHT v2 9075 OK 18/18 a4f19c domain=64 sim=yes
#
# Verifies large artifacts by CHECKSUM, not existence: a truncated 1 GB context
# binary that fails 40 minutes later inside inference is the worst possible
# outcome for a live session.
set -uo pipefail

VERSION=2
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ARTIFACTS="${ARTIFACT_DIR:-$REPO_ROOT/artifacts}"
BUNDLE="$ARTIFACTS/pi05-qnn_context_binary-mixed-qualcomm_qcs9075"
MIN_FREE_GB=30

PASS=0
TOTAL=0
FAILED_NAMES=()

pass() { PASS=$((PASS + 1)); TOTAL=$((TOTAL + 1)); printf '[PASS] %s\n' "$1"; }
fail() {
  TOTAL=$((TOTAL + 1))
  FAILED_NAMES+=("$1")
  printf '[FAIL] %s: %s\n' "$1" "$2"
}

check() { # name, fix-hint, command...
  local name="$1" hint="$2"; shift 2
  if "$@" >/dev/null 2>&1; then pass "$name"; else fail "$name" "$hint"; fi
}

echo "=== Pi0.5 VLA workshop preflight (v$VERSION) ==="
echo

# ---------- board identity -------------------------------------------------
# Assert patterns and floors, never exact versions: shipping images drift.
if grep -qi "IQ.9075\|sa8775\|qcs9075" /proc/device-tree/model 2>/dev/null ||
   grep -qi "sa8775\|qcs9075" /proc/device-tree/compatible 2>/dev/null; then
  pass "board is an IQ-9075 / QCS9075"
else
  fail "board is an IQ-9075 / QCS9075" \
    "this workshop targets the IQ-9075; found '$(tr -d '\0' </proc/device-tree/model 2>/dev/null)'"
fi

check "8 CPU cores online" "expected 8, got $(nproc)" test "$(nproc)" -eq 8

if [[ "$(uname -m)" == "aarch64" ]]; then
  pass "aarch64 userspace"
else
  fail "aarch64 userspace" "found $(uname -m); reflash with the arm64 Ubuntu Server image"
fi

# Floor, not equality: 24.04 or newer is fine.
# shellcheck source=/dev/null
UBUNTU_VER="$(. /etc/os-release && echo "${VERSION_ID:-0}")"
if [[ "$(printf '%s\n24.04\n' "$UBUNTU_VER" | sort -V | head -1)" == "24.04" ]]; then
  pass "Ubuntu >= 24.04 (found $UBUNTU_VER)"
else
  fail "Ubuntu >= 24.04" "found $UBUNTU_VER; reflash with Ubuntu 24.04 Server or newer"
fi

if uname -r | grep -q -- "-qcom"; then
  pass "Qualcomm kernel flavour ($(uname -r))"
else
  fail "Qualcomm kernel flavour" "kernel $(uname -r) is not a -qcom build; reflash the Qualcomm image"
fi

# ---------- NPU reachability ----------------------------------------------
# Device nodes are the durable check; version strings are not.
check "/dev/fastrpc-cdsp present (NPU 0)" \
  "no CDSP device node; the DSP firmware did not come up - reboot, then reflash if it persists" \
  test -e /dev/fastrpc-cdsp

if [[ -e /dev/fastrpc-cdsp1 ]]; then
  pass "/dev/fastrpc-cdsp1 present (NPU 1)"
else
  fail "/dev/fastrpc-cdsp1 present (NPU 1)" \
    "second NPU missing; Module 5 (dual-NPU speedup) will not work on this board"
fi

if [[ -r /dev/fastrpc-cdsp ]]; then
  pass "CDSP readable by $(id -un)"
else
  fail "CDSP readable by $(id -un)" "run: sudo usermod -aG fastrpc $(id -un) && log out and back in"
fi

check "/dev/dma_heap/system present" \
  "DMA-buf heap missing; zero-copy transport is unavailable" \
  test -e /dev/dma_heap/system

ALL_RUNNING=yes
for f in /sys/class/remoteproc/remoteproc*/state; do
  [[ -r "$f" ]] || continue
  [[ "$(cat "$f")" == "running" ]] || ALL_RUNNING=no
done
if [[ "$ALL_RUNNING" == yes ]]; then
  pass "all remoteproc cores running"
else
  fail "all remoteproc cores running" "a DSP/firmware core is not up; reboot the board (~24 s)"
fi

# ---------- toolchain ------------------------------------------------------
check "ROS 2 Jazzy installed" \
  "run PREWORK.md step 3 (scripts/install-ros2-jazzy.sh)" \
  test -f /opt/ros/jazzy/setup.bash

check "colcon present" "sudo apt install -y python3-colcon-common-extensions" \
  command -v colcon

check "QAIRT runtime (libQnnHtp.so)" \
  "sudo apt install -y qairt-libs qairt-tools qairt-headers" \
  test -f /usr/lib/libQnnHtp.so

check "Hexagon v73 stub (matches IQ-9075)" \
  "qairt-libs is present but the v73 stub is missing; reinstall qairt-libs" \
  test -f /usr/lib/libQnnHtpV73Stub.so

check "qnn-net-run tool" "sudo apt install -y qairt-tools" command -v qnn-net-run

check "SentencePiece headers" "sudo apt install -y libsentencepiece-dev" \
  test -f /usr/include/sentencepiece_processor.h

check "QNN headers" "sudo apt install -y qairt-headers" test -d /usr/include/QNN

# ---------- artifacts, by checksum ----------------------------------------
if [[ -f "$ARTIFACTS/manifest.sha256" ]]; then
  if (cd "$ARTIFACTS" && sha256sum -c --quiet manifest.sha256) >/dev/null 2>&1; then
    pass "model artifacts match manifest.sha256 ($(du -sh "$BUNDLE" 2>/dev/null | cut -f1))"
  else
    BAD="$( (cd "$ARTIFACTS" && sha256sum -c manifest.sha256 2>/dev/null) | grep -v ': OK$' | head -3 | cut -d: -f1 | tr '\n' ' ')"
    # The manifest covers two things produced by different pre-work steps, so name the
    # right one: sending someone to re-download 2.9 GB when their 30 MB episode is the
    # broken file wastes 45 minutes.
    if [[ "$BAD" == *libero_ep0.npz* ]]; then
      HINT="re-run PREWORK.md step 5 (scripts/prepare-libero-npz.py), or copy libero_ep0.npz from the USB stick"
    else
      HINT="re-run PREWORK.md step 4, or copy the bundle from the USB stick"
    fi
    fail "model artifacts match manifest.sha256" \
      "corrupt or missing: ${BAD:-unknown}. $HINT"
  fi
else
  fail "model artifacts match manifest.sha256" \
    "no manifest at $ARTIFACTS/manifest.sha256; run PREWORK.md step 4"
fi

check "PaliGemma tokenizer present" \
  "run scripts/fetch-tokenizer.sh (4.3 MB), or copy from the USB stick" \
  test -s "$ARTIFACTS/paligemma_tokenizer.model"

# ---------- capacity ------------------------------------------------------
FREE_GB=$(df -BG --output=avail / 2>/dev/null | tail -1 | tr -dc '0-9')
FREE_GB=${FREE_GB:-0}
if (( FREE_GB >= MIN_FREE_GB )); then
  pass "free disk ${FREE_GB} GB (need ${MIN_FREE_GB} GB)"
else
  fail "free disk >= ${MIN_FREE_GB} GB" "only ${FREE_GB} GB free; clear space before the event"
fi

MEM_GB=$(awk '/MemTotal/ {printf "%d", $2/1024/1024}' /proc/meminfo)
if (( MEM_GB >= 8 )); then
  pass "RAM ${MEM_GB} GB"
else
  fail "RAM >= 8 GB" "found ${MEM_GB} GB"
fi

# ---------- seat assignment ----------------------------------------------
# Thirty boards on ROS_DOMAIN_ID 0 merge into one ROS graph and every attendee
# sees everyone else's topics. Assign once and persist.
DOMAIN_FILE="$HOME/.ros_domain_id"
if [[ -s "$DOMAIN_FILE" ]]; then
  DOMAIN="$(cat "$DOMAIN_FILE")"
else
  # Derive a stable per-board id from the machine id, in the range 1..101.
  DOMAIN=$(( ( 0x$(head -c 4 /etc/machine-id 2>/dev/null || echo 0001) % 101 ) + 1 ))
  echo "$DOMAIN" > "$DOMAIN_FILE"
fi
if grep -q "ROS_DOMAIN_ID" "$HOME/.bashrc" 2>/dev/null; then
  pass "ROS_DOMAIN_ID=$DOMAIN assigned and persisted"
else
  fail "ROS_DOMAIN_ID persisted in ~/.bashrc" \
    "run: echo 'export ROS_DOMAIN_ID=$DOMAIN' >> ~/.bashrc && echo 'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' >> ~/.bashrc"
fi

# ---------- optional: simulator for Module 8 [STRETCH] -------------------
# Deliberately does NOT count toward PASS/TOTAL. Module 8 is stretch, so an attendee who
# skipped PREWORK step 5b is still fully ready and must not be told they are INCOMPLETE.
# Inflating TOTAL for only some attendees would also break the one thing that makes the
# digest line useful: being comparable across the room at a glance. So it reports through a
# separate `sim=` field instead.
SIM=no
LIBERO_ENV="${LIBERO_ENV:-$HOME/libero/libero-env.sh}"
if [[ ! -f "$LIBERO_ENV" ]]; then
  printf '[SKIP] simulator for Module 8: not installed (optional; PREWORK step 5b)\n'
else
  # The real question is not "does it import" but "can MuJoCo get a GL context on this
  # board" -- that is the part that fails on other hardware, and it fails at import time.
  SIM_PY=$(sed -n 's/^export LIBERO_VENV_PYTHON=//p' "$LIBERO_ENV" | tail -1)
  if [[ -x "$SIM_PY" ]] && MUJOCO_GL=osmesa "$SIM_PY" -c '
import sys, mujoco
assert tuple(int(x) for x in mujoco.__version__.split(".")[:2]) <= (3, 2), mujoco.__version__
m = mujoco.MjModel.from_xml_string("<mujoco><worldbody/></mujoco>")
mujoco.Renderer(m, 64, 64).render()
' >/dev/null 2>&1; then
    SIM=yes
    printf '[INFO] simulator for Module 8: ready (offscreen render OK)\n'
  else
    printf '[INFO] simulator for Module 8: installed but NOT working -- re-run scripts/install-libero.sh\n'
  fi
fi

# ---------- digest -------------------------------------------------------
echo
if (( ${#FAILED_NAMES[@]} > 0 )); then
  echo "Fix these before the workshop:"
  for n in "${FAILED_NAMES[@]}"; do echo "  - $n"; done
  echo
fi

STATUS=$([[ $PASS -eq $TOTAL ]] && echo OK || echo INCOMPLETE)
FINGERPRINT=$(printf '%s' "$(cat /etc/machine-id 2>/dev/null)$PASS$TOTAL" | sha256sum | head -c 6)
echo "Paste this one line into the event channel:"
echo "PREFLIGHT v$VERSION 9075 $STATUS $PASS/$TOTAL $FINGERPRINT domain=$DOMAIN sim=$SIM"

[[ $PASS -eq $TOTAL ]] || exit 1
