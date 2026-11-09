#!/usr/bin/env bash
# Start the ROS VLA node and the Pi0.5 flexibility lab in two tmux panes.
#
# The lab starts idle. It loads a world and serves the browser, but publishes
# nothing to /qrb_ros_vla/task until you type a prompt, run the A/B probe, or
# fire a scripted gesture.
set -euo pipefail

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
SESSION=${SESSION:-vla-libero-demo}
HOST=${HOST:-0.0.0.0}
PORT=${PORT:-8080}
WEB_ARGS=${WEB_ARGS:-}
LAUNCH_ARGS=${LAUNCH_ARGS:-}

usage() {
	cat >&2 <<'EOF'
usage: scripts/start-libero-web-demo.sh

Starts a tmux session with two panes:
  left/top:  ros2 launch qrb_ros_vla vla.launch.py
  right/bot: demo/libero_web_demo.py --host $HOST --port $PORT

Then open http://<board-ip>:$PORT and drive it from the browser: 41 worlds,
free-text prompts on the NPU, an A/B language probe, and a scripted gesture lane.

env overrides:
  SESSION      tmux session name (default: vla-libero-demo)
  HOST         web bind host (default: 0.0.0.0)
  PORT         web port (default: 8080)
  LAUNCH_ARGS  extra args passed to ros2 launch qrb_ros_vla vla.launch.py
  WEB_ARGS     extra args passed to demo/libero_web_demo.py

examples:
  PORT=8090 scripts/start-libero-web-demo.sh
  WEB_ARGS='--scene libero_goal:3' scripts/start-libero-web-demo.sh
  WEB_ARGS='--no-realtime' scripts/start-libero-web-demo.sh
EOF
	exit 2
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
	usage
fi

command -v tmux >/dev/null || { echo "tmux is required: sudo apt-get install tmux" >&2; exit 1; }
[ -r /opt/ros/jazzy/setup.bash ] || { echo "missing /opt/ros/jazzy/setup.bash" >&2; exit 1; }
[ -r "$ROOT/ros2_ws/install/setup.bash" ] || { echo "missing ROS workspace overlay: $ROOT/ros2_ws/install/setup.bash" >&2; exit 1; }
[ -r /home/ubuntu/libero/libero-env.sh ] || {
  echo "missing /home/ubuntu/libero/libero-env.sh" >&2
  echo "run: scripts/install-libero.sh" >&2
  exit 1
}

COMMON="cd '$ROOT' && set +u && source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash && source /home/ubuntu/libero/libero-env.sh && set -u"
VLA_CMD="$COMMON && ros2 launch qrb_ros_vla vla.launch.py $LAUNCH_ARGS"
WEB_CMD="$COMMON && \${LIBERO_VENV_PYTHON:-python3} demo/libero_web_demo.py --host '$HOST' --port '$PORT' $WEB_ARGS"

if tmux has-session -t "$SESSION" 2>/dev/null; then
	echo "tmux session '$SESSION' already exists; attaching" >&2
	exec tmux attach -t "$SESSION"
fi

tmux new-session -d -s "$SESSION" -n demo "$VLA_CMD"
tmux split-window -h -t "$SESSION:demo" "$WEB_CMD"
tmux select-layout -t "$SESSION:demo" even-horizontal
tmux set-option -t "$SESSION" remain-on-exit on >/dev/null

tmux display-message -t "$SESSION" "Open http://<board-ip>:$PORT after the web pane prints ready"
exec tmux attach -t "$SESSION"
