#!/usr/bin/env bash
set -euo pipefail

if ! command -v apt-get >/dev/null 2>&1; then
  echo "apt-get not found. Install system packages manually for your Linux distribution."
  exit 1
fi

sudo apt-get update || true
sudo apt-get install -y \
  can-utils \
  iproute2 \
  libxcb-cursor0 \
  libxkbcommon-x11-0 \
  libxcb-xinerama0 \
  libxcb-icccm4 \
  libxcb-image0 \
  libxcb-keysyms1 \
  libxcb-render-util0
