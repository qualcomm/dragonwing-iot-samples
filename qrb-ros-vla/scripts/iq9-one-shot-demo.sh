#!/usr/bin/env bash
# One command to prepare a clean Qualcomm Dragonwing IQ-9075 EVK and start the LIBERO web demo.
#
# Requirements before running:
#   * Ubuntu 24.04 on the IQ-9075
#   * sudo access
#   * internet access, unless a complete Pi0.5 QNN context bundle already exists
#   * Qualcomm AI Hub credentials available to qai-hub-models, unless a verified bundle is already present
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
TOOLS_VENV="${TOOLS_VENV:-$ROOT/.venv-tools}"
BUNDLE_NAME="pi05-qnn_context_binary-mixed-qualcomm_qcs9075"
BUNDLE_DIR="${BUNDLE_DIR:-$ROOT/artifacts/$BUNDLE_NAME}"
SETUP_ONLY=0

usage() {
  local code="${1:-2}"
  cat >&2 <<'EOF'
usage: scripts/iq9-one-shot-demo.sh [--setup-only]

Installs ROS 2 Jazzy, QRB ROS packages, QAIRT, Python tools, Pi0.5 artifacts,
the LIBERO replay episode, the ROS workspace, and LIBERO web-demo dependencies.
Then starts scripts/start-libero-web-demo.sh unless --setup-only is passed.

Environment overrides:
  TOOLS_VENV   helper Python venv for qai-hub-models and parquet conversion
  BUNDLE_DIR   existing/final Pi0.5 QNN context bundle directory
EOF
  exit "$code"
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage 0
elif [[ "${1:-}" == "--setup-only" ]]; then
  SETUP_ONLY=1
elif [[ $# -gt 0 ]]; then
  usage
fi

APT_GET="sudo DEBIAN_FRONTEND=noninteractive apt-get -y -o Dpkg::Use-Pty=0"

step() {
  printf '\n==> %s\n' "$*"
}

have_bundle() {
  local file

  [[ -d "$BUNDLE_DIR" ]] || return 1
  for file in token_emb.bin backbone.bin vision_encoder.bin action_expert.bin metadata.json; do
    [[ -s "$BUNDLE_DIR/$file" ]] || return 1
  done
}

fetch_bundle() {
  local qai_hub_models="$TOOLS_VENV/bin/qai-hub-models"

  if [[ ! -x "$qai_hub_models" ]]; then
    echo "ERROR: qai-hub-models is missing from $TOOLS_VENV" >&2
    exit 1
  fi

  rm -rf "$BUNDLE_DIR"
  if ! (cd "$ROOT/artifacts" && "$qai_hub_models" fetch Pi0.5 \
      --runtime qnn_context_binary \
      --precision mixed \
      --chipset qualcomm-qcs9075); then
    cat >&2 <<EOF
ERROR: Pi0.5 bundle fetch failed.
Authenticate Qualcomm AI Hub on this board, or copy the bundle to:
  $BUNDLE_DIR
Then rerun this script.
EOF
    exit 1
  fi
}

write_artifact_manifest() {
  local manifest="$ROOT/artifacts/manifest.sha256"
  (
    cd "$ROOT/artifacts"
    sha256sum \
      paligemma_tokenizer.model \
      "$BUNDLE_NAME/action_expert.bin" \
      "$BUNDLE_NAME/backbone.bin" \
      "$BUNDLE_NAME/token_emb.bin" \
      "$BUNDLE_NAME/vision_encoder.bin" \
      "$BUNDLE_NAME/metadata.json" \
      libero_ep0.npz > "$manifest"
  )
  echo "wrote artifact manifest: $manifest"
}


cd "$ROOT"
mkdir -p artifacts

step "ROS 2 Jazzy"
"$ROOT/scripts/install-ros2-jazzy.sh"

step "QRB ROS packages"
"$ROOT/scripts/install-qrb-ros.sh"

step "System packages"
$APT_GET install \
  curl git tmux python3-venv python3-pip python3-numpy python3-pil \
  qairt-libs qairt-tools qairt-headers libsentencepiece-dev

step "Python helper tools"
if [[ ! -x "$TOOLS_VENV/bin/python" ]]; then
  python3 -m venv "$TOOLS_VENV"
fi
"$TOOLS_VENV/bin/python" -m pip install --upgrade pip
"$TOOLS_VENV/bin/python" -m pip install --upgrade \
  qai-hub-models pyarrow pillow 'numpy<3' uv
# Keep the helper venv off PATH. colcon/ament must use the system Python that owns ROS packages.

step "Tokenizer"
"$ROOT/scripts/fetch-tokenizer.sh" "$ROOT/artifacts/paligemma_tokenizer.model"

step "Pi0.5 QNN context bundle"
if have_bundle; then
  echo "already present and complete: $BUNDLE_DIR"
else
  echo "missing or incomplete bundle: $BUNDLE_DIR"
  fetch_bundle
  have_bundle || { echo "ERROR: fetched Pi0.5 bundle is incomplete: $BUNDLE_DIR" >&2; exit 1; }
fi

step "LIBERO replay episode"
PYTHON="$TOOLS_VENV/bin/python" "$ROOT/scripts/fetch-libero-episode.sh" /tmp/libero

step "Artifact manifest"
write_artifact_manifest

step "ROS workspace build"
# shellcheck disable=SC1091
set +u; source /opt/ros/jazzy/setup.bash; set -u
(
  cd "$ROOT/ros2_ws"
  rm -rf build/qrb_inference_manager build/qrb_ros_vla build/qrb_ros_vla_msgs \
    install/qrb_inference_manager install/qrb_ros_vla install/qrb_ros_vla_msgs
  colcon build --allow-overriding qrb_inference_manager \
    --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
)
# shellcheck disable=SC1091
set +u; source "$ROOT/ros2_ws/install/setup.bash"; set -u
ros2 pkg prefix qrb_ros_vla >/dev/null

step "LIBERO web demo dependencies"
"$ROOT/scripts/install-libero.sh" /home/ubuntu/libero

cat <<EOF

Setup complete.
Overlay for manual shells:
  source /opt/ros/jazzy/setup.bash
  source $ROOT/ros2_ws/install/setup.bash
EOF

if [[ "$SETUP_ONLY" -eq 1 ]]; then
  echo "Run the web demo later with: scripts/start-libero-web-demo.sh"
  exit 0
fi

step "Starting LIBERO web demo"
exec "$ROOT/scripts/start-libero-web-demo.sh"
