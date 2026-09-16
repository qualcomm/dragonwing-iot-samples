#!/usr/bin/env bash
# Install Gazebo Harmonic + the ROS 2 bridge, for the qrb_ros_simulation demo path.
#
# Notes for the IQ-9075 specifically:
#  * gz-harmonic DOES have arm64 packages, so this works on the board itself.
#  * The board is Ubuntu Server with no display. Gazebo must run headless
#    (`gz sim -s`, server only). Camera sensors still need a GL context, which
#    comes from EGL on the Adreno GPU -- /dev/dri/renderD128 and
#    libEGL_adreno.so.1 are both present on the stock image.
#  * ros_gz is preferred from apt; it is a large source build otherwise.
set -euo pipefail

APT_GET="sudo DEBIAN_FRONTEND=noninteractive apt-get -y -o Dpkg::Use-Pty=0"
KEYRING=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
LIST=/etc/apt/sources.list.d/gazebo-stable.list

echo "== 1/4 osrf apt source =="
$APT_GET install curl gnupg lsb-release
if [[ ! -s "$KEYRING" ]]; then
  curl -fsSL https://packages.osrfoundation.org/gazebo.gpg | sudo tee "$KEYRING" >/dev/null
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=$KEYRING] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee "$LIST" >/dev/null

echo "== 2/4 apt update =="
$APT_GET update

echo "== 3/4 install gazebo harmonic =="
$APT_GET install gz-harmonic

echo "== 4/4 ros 2 <-> gazebo bridge =="
# ros-jazzy-ros-gz pulls the whole bridge/sim stack when a binary exists for this
# arch; fall back to just the bridge if the metapackage is unavailable.
if apt-cache show ros-jazzy-ros-gz >/dev/null 2>&1; then
  $APT_GET install ros-jazzy-ros-gz
else
  echo "  ros-jazzy-ros-gz unavailable; installing bridge components individually"
  $APT_GET install ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image ros-jazzy-ros-gz-sim
fi

echo "done: $(gz sim --versions 2>/dev/null | head -1) installed; headless render check:"
echo "  gz sim -s -r --iterations 1 <world>.sdf"
