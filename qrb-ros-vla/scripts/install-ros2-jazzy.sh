#!/usr/bin/env bash
# Install ROS 2 Jazzy Jalisco (ros-base + dev tools) on Ubuntu 24.04 arm64 / IQ-9075.
# Idempotent: safe to re-run. Does not touch the pre-existing qcom-ppa source.
set -euo pipefail

APT_GET="sudo DEBIAN_FRONTEND=noninteractive apt-get -y -o Dpkg::Use-Pty=0"
KEYRING=/usr/share/keyrings/ros-archive-keyring.gpg
LIST=/etc/apt/sources.list.d/ros2.list

# packages.ros.org is a CNAME to ftp.osuosl.org, whose TLS cert covers only
# *.osuosl.org. On networks where that mismatch is not papered over by a CDN,
# https:// fails and we must fall back to http://. Integrity is still enforced
# by the repository GPG signature, which is what apt actually trusts.
pick_scheme() {
  if curl -fsS --max-time 15 -o /dev/null \
      https://packages.ros.org/ros2/ubuntu/dists/noble/Release 2>/dev/null; then
    echo https
  elif curl -fsS --max-time 15 -o /dev/null \
      http://packages.ros.org/ros2/ubuntu/dists/noble/Release 2>/dev/null; then
    echo "  note: https to packages.ros.org failed (cert covers *.osuosl.org only);" >&2
    echo "        falling back to http. GPG signature verification is unaffected." >&2
    echo http
  else
    echo "ERROR: packages.ros.org unreachable over https or http." >&2
    exit 1
  fi
}

echo "== 1/5 prerequisites =="
$APT_GET update
$APT_GET install curl gnupg ca-certificates software-properties-common

echo "== 2/5 universe repository =="
sudo add-apt-repository -y universe

echo "== 3/5 ROS 2 apt key + source =="
if [[ ! -s "$KEYRING" ]]; then
  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | sudo gpg --dearmor -o "$KEYRING"
fi
SCHEME="$(pick_scheme)"
echo "deb [arch=$(dpkg --print-architecture) signed-by=$KEYRING] ${SCHEME}://packages.ros.org/ros2/ubuntu noble main" \
  | sudo tee "$LIST" >/dev/null

echo "== 4/5 apt update =="
$APT_GET update

echo "== 5/5 install ros-jazzy-ros-base + build tooling =="
$APT_GET install \
  ros-jazzy-ros-base \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-vision-msgs \
  ros-jazzy-rosbag2-storage-mcap \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-vcstool

# shellcheck disable=SC1091
set +u; source /opt/ros/jazzy/setup.bash; set -u
echo "done: ROS 2 ${ROS_DISTRO} installed, rmw=$(printenv RMW_IMPLEMENTATION 2>/dev/null || echo default), $(ros2 --version 2>&1 | head -1)"
