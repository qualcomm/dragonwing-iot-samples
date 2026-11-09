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
# QAIRT runtime from ppa:ubuntu-qcom-iot/qcom-ppa. Pinned so every board runs the same
# runtime, and the Pi0.5 bundle selected below stays loadable on it.
QAIRT_APT_VERSION="${QAIRT_APT_VERSION:-2.49.0-0ubuntu1~bpo24.04.2}"
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
  QAIRT_APT_VERSION
               qairt-* package version to install and hold
               (default 2.49.0-0ubuntu1~bpo24.04.2)
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

BUNDLE_FETCH_ARGS=(--runtime qnn_context_binary --precision mixed --chipset qualcomm-qcs9075)

# QNN context binaries load only on a QAIRT runtime at least as new as the SDK that
# compiled them ("Using newer context binary on old SDK", err 5000). AI Hub's latest
# Pi0.5 build can be ahead of the qairt-libs in the PPA, so the bundle has to be
# chosen to match the runtime installed on this board, not just the newest one.

# Trim "2.49.0-0ubuntu1~bpo24.04.2" or "2.45.0.260326154327" to "2.49.0" / "2.45.0".
short_version() {
  sed -E 's/^([0-9]+\.[0-9]+\.[0-9]+).*/\1/' <<<"$1"
}

installed_qairt_version() {
  local version
  version="$(dpkg-query -W -f='${Version}' qairt-libs 2>/dev/null)" || return 1
  [[ -n "$version" ]] || return 1
  short_version "$version"
}

bundle_qairt_version() {
  python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["tool_versions"]["qairt"])' \
    "$BUNDLE_DIR/metadata.json" 2>/dev/null
}

# True if a bundle compiled with QAIRT $1 can run on QAIRT runtime $2.
qairt_runs_on() {
  dpkg --compare-versions "$(short_version "$1")" le "$2"
}

MANIFEST="$ROOT/artifacts/manifest.sha256"

# The manifest is written right after a pinned fetch, so it records exactly which bundle
# was selected for this board. On later runs, check the bundle against it rather than
# re-hashing whatever is on disk: rewriting it every time would bless a truncated or
# swapped bundle. No manifest entries for the bundle (e.g. first run, or a bundle copied
# in by hand) means there is nothing to compare against, which counts as a pass.
bundle_matches_manifest() {
  local entries bad

  [[ -f "$MANIFEST" ]] || return 0
  entries="$(grep -F "  $BUNDLE_NAME/" "$MANIFEST" || true)"
  [[ -n "$entries" ]] || return 0

  if bad="$(cd "$ROOT/artifacts" && sha256sum -c --quiet <<<"$entries" 2>&1)"; then
    return 0
  fi
  echo "bundle does not match $MANIFEST:"
  sed 's/^/  /' <<<"$bad"
  return 1
}

have_bundle() {
  local file runtime_qairt bundle_qairt

  [[ -d "$BUNDLE_DIR" ]] || return 1
  for file in token_emb.bin backbone.bin vision_encoder.bin action_expert.bin metadata.json; do
    [[ -s "$BUNDLE_DIR/$file" ]] || return 1
  done

  bundle_matches_manifest || return 1

  runtime_qairt="$(installed_qairt_version)" || return 0
  bundle_qairt="$(bundle_qairt_version)" || return 0
  if ! qairt_runs_on "$bundle_qairt" "$runtime_qairt"; then
    echo "bundle was built with QAIRT $bundle_qairt, newer than installed runtime $runtime_qairt"
    return 1
  fi
}

# Print "<ai-hub-models release> <qairt version>" for the newest Pi0.5 bundle that the
# installed QAIRT runtime can load. Releases are walked newest first.
select_bundle_release() {
  local qai_hub_models="$1" runtime_qairt="$2" release sdk best

  for release in $("$qai_hub_models" versions 2>/dev/null |
      grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | sort -t. -k1,1nr -k2,2nr -k3,3nr | uniq); do
    best=""
    while read -r sdk; do
      [[ -n "$sdk" ]] || continue
      qairt_runs_on "$sdk" "$runtime_qairt" || continue
      if [[ -z "$best" ]] || dpkg --compare-versions "$sdk" gt "$best"; then
        best="$sdk"
      fi
    done < <("$qai_hub_models" fetch Pi0.5 --info --version "$release" "${BUNDLE_FETCH_ARGS[@]}" 2>/dev/null |
      grep -oE 'QAIRT [0-9][0-9.]*' | awk '{print $2}')
    if [[ -n "$best" ]]; then
      echo "$release $best"
      return 0
    fi
    echo "  ai-hub-models $release: no Pi0.5 bundle runnable on QAIRT $runtime_qairt" >&2
  done
  return 1
}

fetch_bundle() {
  local qai_hub_models="$TOOLS_VENV/bin/qai-hub-models"
  local runtime_qairt release_args=() selection release sdk

  if [[ ! -x "$qai_hub_models" ]]; then
    echo "ERROR: qai-hub-models is missing from $TOOLS_VENV" >&2
    exit 1
  fi

  if runtime_qairt="$(installed_qairt_version)"; then
    echo "installed QAIRT runtime: $runtime_qairt; selecting a compatible Pi0.5 bundle"
    if ! selection="$(select_bundle_release "$qai_hub_models" "$runtime_qairt")"; then
      cat >&2 <<EOF
ERROR: no AI Hub Models release has a Pi0.5 bundle that runs on QAIRT $runtime_qairt.
Upgrade qairt-libs, or copy a compatible bundle to:
  $BUNDLE_DIR
EOF
      exit 1
    fi
    read -r release sdk <<<"$selection"
    echo "using ai-hub-models $release (bundle built with QAIRT $sdk)"
    release_args=(--version "$release" --sdk-version "qairt=$sdk")
  else
    echo "WARNING: qairt-libs version unknown; fetching the latest Pi0.5 bundle" >&2
  fi

  rm -rf "$BUNDLE_DIR"
  if ! (cd "$ROOT/artifacts" && "$qai_hub_models" fetch Pi0.5 \
      "${BUNDLE_FETCH_ARGS[@]}" "${release_args[@]}"); then
    cat >&2 <<EOF
ERROR: Pi0.5 bundle fetch failed.
Authenticate Qualcomm AI Hub on this board, or copy the bundle to:
  $BUNDLE_DIR
Then rerun this script.
EOF
    exit 1
  fi
}

# Rewrite the manifest. Bundle entries are carried over from the existing manifest when
# present (have_bundle has already verified them); they are only re-hashed after a fresh
# fetch or when no record exists. The tokenizer is pinned by fetch-tokenizer.sh and the
# episode is regenerated each run, so those two are always re-hashed.
write_artifact_manifest() {
  local fresh_bundle="$1" bundle_entries="" tmp="$MANIFEST.tmp"

  if [[ "$fresh_bundle" -eq 0 && -f "$MANIFEST" ]]; then
    bundle_entries="$(grep -F "  $BUNDLE_NAME/" "$MANIFEST" || true)"
  fi
  (
    cd "$ROOT/artifacts"
    sha256sum paligemma_tokenizer.model
    if [[ -n "$bundle_entries" ]]; then
      printf '%s\n' "$bundle_entries"
    else
      sha256sum \
        "$BUNDLE_NAME/action_expert.bin" \
        "$BUNDLE_NAME/backbone.bin" \
        "$BUNDLE_NAME/token_emb.bin" \
        "$BUNDLE_NAME/vision_encoder.bin" \
        "$BUNDLE_NAME/metadata.json"
    fi
    sha256sum libero_ep0.npz
  ) > "$tmp"
  mv "$tmp" "$MANIFEST"
  echo "wrote artifact manifest: $MANIFEST"
}


cd "$ROOT"
mkdir -p artifacts

step "ROS 2 Jazzy"
"$ROOT/scripts/install-ros2-jazzy.sh"

step "QRB ROS packages"
"$ROOT/scripts/install-qrb-ros.sh"

step "System packages"
$APT_GET install \
  curl git tmux python3-venv python3-pip python3-numpy python3-pil libsentencepiece-dev

step "QAIRT $QAIRT_APT_VERSION (pinned)"
# All four packages are pinned explicitly: qairt-tools depends on qairt-libs without a
# version, so pinning qairt-libs alone would not stop the tools drifting. The hold keeps a
# later `apt upgrade` from moving the runtime under the bundle that was chosen for it.
QAIRT_PKGS=(qairt-libs qairt-tools qairt-headers qairt-dsp-binaries)
if ! $APT_GET install --allow-downgrades --allow-change-held-packages \
    "${QAIRT_PKGS[@]/%/=$QAIRT_APT_VERSION}"; then
  cat >&2 <<EOF
ERROR: could not install QAIRT $QAIRT_APT_VERSION from the PPA.
Launchpad PPAs usually publish only the newest build, so this version may have been
superseded. Versions currently available:
$(apt-cache madison qairt-libs 2>/dev/null | sed 's/^/  /')
Set QAIRT_APT_VERSION to one of them (the Pi0.5 bundle is selected to match it), e.g.
  QAIRT_APT_VERSION=<version> scripts/iq9-one-shot-demo.sh
EOF
  exit 1
fi
sudo apt-mark hold "${QAIRT_PKGS[@]}" >/dev/null

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
FRESH_BUNDLE=0
if have_bundle; then
  echo "already present and verified: $BUNDLE_DIR"
else
  echo "missing, incomplete, or unverified bundle: $BUNDLE_DIR"
  fetch_bundle
  FRESH_BUNDLE=1
  # The old manifest describes the bundle that was just replaced; drop its entries
  # so the re-check below only looks at presence and QAIRT compatibility.
  rm -f "$MANIFEST"
  have_bundle || { echo "ERROR: fetched Pi0.5 bundle is incomplete: $BUNDLE_DIR" >&2; exit 1; }
fi

step "LIBERO replay episode"
PYTHON="$TOOLS_VENV/bin/python" "$ROOT/scripts/fetch-libero-episode.sh" /tmp/libero

step "Artifact manifest"
write_artifact_manifest "$FRESH_BUNDLE"

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
