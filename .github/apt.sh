#!/bin/sh
# Installs Ubuntu packages from archive.ubuntu.com, since the runner's own mirror can stall for hours.
set -e
for f in /etc/apt/sources.list /etc/apt/sources.list.d/*.sources; do
  if [ -f "$f" ]; then
    sudo sed -i -E 's#https?://[a-z0-9.-]*archive\.ubuntu\.com/ubuntu#http://archive.ubuntu.com/ubuntu#' "$f"
  fi
done
sudo apt-get update
sudo apt-get install -y --no-install-recommends "$@"
