#!/usr/bin/env bash
# Install the LIBERO benchmark on the IQ-9075 for closed-loop VLA evaluation.
#
# Headless, aarch64, no torch, no compiler, no GPU. Every pin here is load-bearing and was
# established by failing without it -- see the notes inline. Writes an env file you source
# before running anything, so the three mandatory variables are never forgotten.
#
# Idempotent: safe to re-run. Prints one summary line.
#
# Usage:
#   scripts/install-libero.sh [install_root]
#   source <install_root>/libero-env.sh
set -euo pipefail

ROOT="${1:-$HOME/libero}"
LIBERO_COMMIT="${LIBERO_COMMIT:-master}"
REPO_DIR="$ROOT/LIBERO"
VENV="$ROOT/venv"
CFG_DIR="$ROOT/config"
ENV_FILE="$ROOT/libero-env.sh"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UV_BIN="${UV_BIN:-$PROJECT_DIR/.venv-tools/bin/uv}"

# MuJoCo needs a desktop-OpenGL context. On this board that means Mesa's llvmpipe software
# rasterizer, which offers OpenGL 4.5 *compatibility* -- enough for MuJoCo's classic
# renderer, and exactly what Gazebo's Ogre2 could not use because it demands 3.3 *core*.
# There is no hardware path: the Adreno EGL driver is OpenGL-ES-only.
echo "==> apt: OSMesa"
if ! dpkg -s libosmesa6 >/dev/null 2>&1 || ! dpkg -s libosmesa6-dev >/dev/null 2>&1; then
  sudo apt-get update -qq
  sudo apt-get install -y libosmesa6 libosmesa6-dev
else
  echo "    already installed"
fi

if [[ ! -x "$UV_BIN" ]]; then
  UV_BIN="$(command -v uv || true)"
fi
[[ -n "$UV_BIN" && -x "$UV_BIN" ]] || { echo "ERROR: uv not found; expected $PROJECT_DIR/.venv-tools/bin/uv or uv on PATH" >&2; exit 1; }

echo "==> venv at $VENV"
mkdir -p "$ROOT"
[[ -x "$VENV/bin/python" ]] || "$UV_BIN" venv "$VENV" --python 3.12
PY="$VENV/bin/python"

echo "==> pinned dependency set"
# mujoco: HARD CEILING at 3.2.7. robosuite 1.4.0 touches MjData.qM, which mujoco 3.3.0
#   renamed to M, so anything newer dies at env construction. A naive resolve picks 3.11.0.
# robosuite/bddl with --no-deps: their declared deps drag in pytest, jupytext, nbformat,
#   jsonschema and the GUI build of opencv-python, none of which are used here.
# The remaining packages are robosuite's *undeclared* imports, which otherwise fail one at
#   a time: termcolor, future, easydict, matplotlib, cloudpickle, pyyaml.
# Deliberately NO torch. LIBERO uses it only for torch.load on its .pruned_init files, and
#   installing it would pull 26 packages including the entire NVIDIA CUDA sbsa stack onto a
#   board with no NVIDIA GPU. bench/torchfree_init.py reads those files directly instead.
# mujoco WITH its dependencies, in its own command. Combining it with a --no-deps package
# applies --no-deps to both, which silently drops pyopengl -- and mujoco then fails at
# import time under MUJOCO_GL=osmesa with "No module named 'OpenGL'".
"$UV_BIN" pip install --python "$PY" --quiet "mujoco==3.2.7"
"$UV_BIN" pip install --python "$PY" --quiet "robosuite==1.4.0" --no-deps
"$UV_BIN" pip install --python "$PY" --quiet "bddl==1.0.1" --no-deps
"$UV_BIN" pip install --python "$PY" --quiet \
  "numpy<3" scipy numba pillow opencv-python-headless \
  termcolor future easydict networkx matplotlib "gym==0.25.2" cloudpickle pyyaml

echo "==> LIBERO source at $REPO_DIR"
if [[ -d "$REPO_DIR/.git" ]]; then
  echo "    already cloned"
else
  git clone --quiet https://github.com/Lifelong-Robot-Learning/LIBERO.git "$REPO_DIR"
fi
git -C "$REPO_DIR" checkout --quiet "$LIBERO_COMMIT"

# NOT 'pip install -e .': LIBERO ships no libero/__init__.py, so PEP 660 editable installs
# produce an empty package with an empty MAPPING and every import fails. Source on
# PYTHONPATH is the only thing that works.
echo "==> config (import calls input() without it and hangs)"
mkdir -p "$CFG_DIR"
cat > "$CFG_DIR/config.yaml" <<EOF
assets: $REPO_DIR/libero/libero/assets
bddl_files: $REPO_DIR/libero/libero/bddl_files
benchmark_root: $REPO_DIR/libero/libero
datasets: $REPO_DIR/libero/datasets
init_states: $REPO_DIR/libero/libero/init_files
EOF

echo "==> env file $ENV_FILE"
cat > "$ENV_FILE" <<EOF
# Source this before running demo/libero_closed_loop.py or the bench/verify_libero_* scripts.
# MUJOCO_GL must be exactly 'osmesa': robosuite routes any other value (including 'glfw')
# to the EGL path, which fails on this board's Adreno ICD.
export MUJOCO_GL=osmesa
export LIBERO_CONFIG_PATH=$CFG_DIR
export LIBERO_VENV_PYTHON=$PY
# One interpreter must see BOTH rclpy and LIBERO. ROS 2's Python packages are added by path
# rather than by installing ROS into the venv; the workspace overlay contributes
# qrb_ros_vla_msgs, which is built there and is not a system package.
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PROJECT_DIR/ros2_ws/install/qrb_ros_vla_msgs/lib/python3.12/site-packages:$REPO_DIR\${PYTHONPATH:+:\$PYTHONPATH}"
EOF

echo "==> smoke test"
# shellcheck source=/dev/null
source "$ENV_FILE"
"$PY" - <<'PY'
import numpy as np, mujoco, robosuite
print(f"    mujoco {mujoco.__version__}  robosuite {robosuite.__version__}  numpy {np.__version__}")
assert tuple(int(x) for x in mujoco.__version__.split(".")[:2]) <= (3, 2), \
    "mujoco > 3.2.x breaks robosuite 1.4.0 (MjData.qM was renamed to M)"
m = mujoco.MjModel.from_xml_string(
    '<mujoco><worldbody><light pos="0 0 3"/><geom type="plane" size="2 2 .1"/>'
    '</worldbody></mujoco>')
r = mujoco.Renderer(m, 256, 256)
r.update_scene(mujoco.MjData(m)); frame = r.render()
print(f"    offscreen render OK: {frame.shape} {frame.dtype}")
PY

SIZE=$(du -sh "$ROOT" 2>/dev/null | cut -f1)
echo "LIBERO installed at $ROOT ($SIZE); source $ENV_FILE before use."
