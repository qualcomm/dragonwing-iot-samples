#!/usr/bin/env bash
# Add the Qualcomm QIRP PPA and install the QRB ROS packages we need.
#
# Idempotency notes (both learned the hard way):
#  * ppa:ubuntu-qcom-iot/qcom-ppa is ALREADY on the stock IQ-9075 Ubuntu image as
#    /etc/apt/sources.list.d/ubuntu-qcom-iot-ubuntu-qcom-ppa-noble.sources. Adding it
#    again produces a fatal "Conflicting values set for option Trusted". So: detect, skip.
#  * ppa:ubuntu-qcom-iot/qirp is NOT on the stock image and its signing key may be
#    missing, which makes `apt update` fail with NO_PUBKEY. add-apt-repository imports
#    the key for us; we verify afterwards rather than assuming.
set -euo pipefail

APT_GET="sudo DEBIAN_FRONTEND=noninteractive apt-get -y -o Dpkg::Use-Pty=0"

have_ppa() {
  # $1 = ppa path fragment, e.g. ubuntu-qcom-iot/qirp
  local owner="${1%%/*}" name="${1##*/}"
  grep -rqs "ppa.launchpadcontent.net/${owner}/${name}/ubuntu" \
    /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null
}

for ppa in ubuntu-qcom-iot/qcom-ppa ubuntu-qcom-iot/qirp; do
  if have_ppa "$ppa"; then
    echo "skip: ppa:${ppa} already configured"
  else
    echo "add:  ppa:${ppa}"
    sudo add-apt-repository -y "ppa:${ppa}"
  fi
done

echo "== apt update =="
if ! $APT_GET update 2>&1 | tee /tmp/qrb-apt-update.log; then
  if grep -q NO_PUBKEY /tmp/qrb-apt-update.log; then
    KEY=$(grep -o 'NO_PUBKEY [0-9A-F]*' /tmp/qrb-apt-update.log | awk '{print $2}' | head -1)
    echo "ERROR: apt update failed with NO_PUBKEY ${KEY}." >&2
    echo "       Import it with:  sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys ${KEY}" >&2
  fi
  exit 1
fi
grep -q NO_PUBKEY /tmp/qrb-apt-update.log && { echo "ERROR: NO_PUBKEY in apt update output" >&2; exit 1; }

echo "== available qrb/qirp packages =="
apt-cache search -n '^ros-jazzy-qrb' | sort | sed 's/^/  /'

echo "== install =="
$APT_GET install \
  ros-jazzy-qrb-ros-nn-inference \
  ros-jazzy-qrb-ros-transport-image-type \
  ros-jazzy-qrb-ros-tensor-list-msgs

echo "done: installed $(dpkg -l | grep -c '^ii  ros-jazzy-qrb') qrb ros packages"
