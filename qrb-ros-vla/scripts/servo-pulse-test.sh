#!/usr/bin/env bash
#
# Pulse one joint of the VUPN2355 arm through a small symmetric sweep, then
# return the PCA9685 to a safe state. Used to identify which servo is on which
# channel and to prove the I2C -> PCA9685 -> servo path works.
#
# The channel binding, pulse ranges and per-joint degrees-per-microsecond all
# come from ros2_ws/src/qrb_ros_vla/config/arm_joints.yaml -- that file is the
# single source of truth, not this script. Override with $ARM_JOINTS_YAML.
#
# Safety properties, in order of importance:
#   - Every exit path (normal, error, Ctrl-C, SIGTERM) drives ALL SIXTEEN
#     channels full-OFF and puts the chip back to SLEEP. A killed process does
#     not leave a servo energised, and it also cleans up after some earlier
#     process that died badly.
#   - Pulse widths are clamped twice: to the joint's provisional soft limits
#     from the config, and to the servo's rated 500-2500 us.
#   - PRE_SCALE is written only while SLEEP=1, as the datasheet requires.
#   - ALLCALL is cleared, so the chip stops answering the 0x70 all-call address
#     and a stray write there cannot reach it.
#
# The chip is asleep on entry and exit, which means the servos are limp and
# back-drivable. Hand-position the joint near mid-travel BEFORE running this on
# a channel whose position you do not know: the first 1500 us command is a jump
# from wherever the joint currently sits, and on a heavy joint that jump is the
# dangerous part, not the sweep that follows.
#
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
CONFIG=${ARM_JOINTS_YAML:-$SCRIPT_DIR/../ros2_ws/src/qrb_ros_vla/config/arm_joints.yaml}

usage() {
	cat >&2 <<'EOF'
usage: servo-pulse-test.sh JOINT [AMPLITUDE_US]

  JOINT         joint name or PCA9685 channel number, resolved against
                ros2_ws/src/qrb_ros_vla/config/arm_joints.yaml
  AMPLITUDE_US  sweep amplitude either side of centre (default 200). Use 50-100
                for a joint whose channel binding is not yet confirmed.

examples:
  servo-pulse-test.sh gripper
  servo-pulse-test.sh 4 60
  servo-pulse-test.sh base_yaw 40

env:
  ARM_JOINTS_YAML   override the config path
EOF
	exit 2
}

[ $# -ge 1 ] || usage
TARGET=$1
AMP=${2:-200}

if ! [[ $AMP =~ ^[0-9]+$ ]] || [ "$AMP" -lt 1 ] || [ "$AMP" -gt 1000 ]; then
	echo "amplitude must be 1-1000 us, got '$AMP'" >&2
	exit 2
fi
command -v i2cset >/dev/null || { echo "need i2c-tools: apt install i2c-tools" >&2; exit 1; }
[ -r "$CONFIG" ] || { echo "cannot read config: $CONFIG" >&2; exit 1; }

resolve() {
	python3 - "$CONFIG" "$TARGET" <<'PY'
import sys, yaml

cfg = yaml.safe_load(open(sys.argv[1]))
key = sys.argv[2]
joints = cfg["joints"]

if key.isdigit():
    j = next((x for x in joints if x["channel"] == int(key)), None)
else:
    j = next((x for x in joints if x["name"] == key), None)
if j is None:
    names = ", ".join(f'{x["channel"]}:{x["name"]}' for x in joints)
    sys.exit(f"unknown joint '{key}'. known: {names}")

addr = cfg["address"]
addr = int(addr, 16) if isinstance(addr, str) else int(addr)

hz = cfg["pwm_hz"]
pre_plus_1 = round(25_000_000 / (4096 * hz))  # PRE_SCALE register is this minus 1

print(
    j["channel"], j["name"], cfg["bus"], f"0x{addr:02x}",
    f"0x{pre_plus_1 - 1:02x}", pre_plus_1,
    cfg["servo"]["pulse_centre_us"],
    cfg["servo"]["pulse_min_us"], cfg["servo"]["pulse_max_us"],
    j["soft_min_us"], j["soft_max_us"], j["deg_per_us"],
    j["verified"],
)
PY
}

resolved=$(resolve) || exit 1
read -r CH NAME BUS ADDR PRE_HEX PRE1 CENTRE RATED_MIN RATED_MAX \
	SOFT_MIN SOFT_MAX DEG_PER_US VERIFIED <<<"$resolved"

MODE1=0x00
PRESCALE=0xfe
ALL_OFF_H=0xfd # ALL_LED_OFF_H: writing bit 4 stops all 16 channels at once
FULL_OFF=0x10  # bit 4 of OFF_H
BASE=$((6 + CH * 4))
ON_L=$BASE ON_H=$((BASE + 1)) OFF_L=$((BASE + 2)) OFF_H=$((BASE + 3))

set_reg() { sudo i2cset -y "$BUS" "$ADDR" "$1" "$2"; }
get_reg() { sudo i2cget -y "$BUS" "$ADDR" "$1"; }

# counts = us * 25 / (PRE_SCALE + 1), rounded to nearest.
us_to_counts() { echo $(((($1 * 25) + (PRE1 / 2)) / PRE1)); }

# See the header: all 16 channels, then sleep. MODE1 reads back with bit 7
# (RESTART) set if PWM was still live when SLEEP was asserted; that is a status
# flag, not a driven output. Stopping the channels first keeps it clear.
safe_state() {
	set_reg "$ALL_OFF_H" "$FULL_OFF" 2>/dev/null || true
	set_reg "$OFF_H" "$FULL_OFF" 2>/dev/null || true
	set_reg "$MODE1" 0x10 2>/dev/null || true
}
trap safe_state EXIT INT TERM

goto_us() {
	local us=$1 hold=$2 counts hi lo
	if [ "$us" -lt "$RATED_MIN" ] || [ "$us" -gt "$RATED_MAX" ]; then
		echo "refusing $us us: outside the servo's rated ${RATED_MIN}-${RATED_MAX} us" >&2
		return 1
	fi
	counts=$(us_to_counts "$us")
	hi=$(printf '0x%02x' $((counts >> 8)))
	lo=$(printf '0x%02x' $((counts & 0xff)))
	set_reg "$OFF_L" "$lo"
	set_reg "$OFF_H" "$hi" # also clears the full-OFF bit
	printf '  %5d us  %4d counts  %+6.1f deg\n' "$us" "$counts" \
		"$(awk -v u="$us" -v c="$CENTRE" -v d="$DEG_PER_US" 'BEGIN{printf "%.1f",(u-c)*d}')"
	sleep "$hold"
}

lo_us=$((CENTRE - AMP))
hi_us=$((CENTRE + AMP))
if [ "$lo_us" -lt "$SOFT_MIN" ]; then
	echo "clamping low end ${lo_us} -> ${SOFT_MIN} us (soft limit for $NAME)" >&2
	lo_us=$SOFT_MIN
fi
if [ "$hi_us" -gt "$SOFT_MAX" ]; then
	echo "clamping high end ${hi_us} -> ${SOFT_MAX} us (soft limit for $NAME)" >&2
	hi_us=$SOFT_MAX
fi

get_reg "$MODE1" >/dev/null || { echo "no device at $ADDR on bus $BUS" >&2; exit 1; }

if [ "$VERIFIED" != observed ]; then
	echo "note: ch${CH} -> ${NAME} binding is '${VERIFIED}', not observed."
	echo "      hand-position this joint near mid-travel first; it is limp right now."
fi

# Force SLEEP so PRE_SCALE is writable, and clear ALLCALL while we are here.
set_reg "$MODE1" 0x10
set_reg "$PRESCALE" "$PRE_HEX"
# Programme centre while still asleep so waking produces no surprise edge.
set_reg "$ON_L" 0x00
set_reg "$ON_H" 0x00
centre_counts=$(us_to_counts "$CENTRE")
set_reg "$OFF_L" "$(printf '0x%02x' $((centre_counts & 0xff)))"
set_reg "$OFF_H" "$(printf '0x%02x' $((centre_counts >> 8)))"

echo "ch${CH} ${NAME} @ ${ADDR} on i2c-${BUS}: waking, ${PRE1}-prescale 50 Hz, ${lo_us}-${hi_us} us"
set_reg "$MODE1" 0x00
sleep 1

goto_us "$CENTRE" 1.0
goto_us "$lo_us" 1.5
goto_us "$hi_us" 1.5
goto_us "$lo_us" 1.5
goto_us "$hi_us" 1.5
goto_us "$CENTRE" 1.5

# trap restores the safe state
echo "ch${CH} ${NAME}: swept ${lo_us}-${hi_us} us, returned to centre, all outputs OFF, chip asleep"
