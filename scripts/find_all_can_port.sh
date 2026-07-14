#!/usr/bin/env bash
set -euo pipefail

if ! command -v ip >/dev/null 2>&1; then
  echo "ip command not found. Install iproute2 first."
  exit 1
fi

if ! command -v ethtool >/dev/null 2>&1; then
  echo "ethtool not found. Install it with: sudo apt update && sudo apt install ethtool"
  exit 1
fi

FOUND=0
for iface in $(ip -br link show type can | awk '{print $1}'); do
  FOUND=1
  BUS_INFO=$(ethtool -i "$iface" 2>/dev/null | grep 'bus-info' | awk '{print $2}' || true)
  STATE=$(ip -details link show "$iface" | grep -oP 'state \K\S+' | head -n 1 || true)
  BITRATE=$(ip -details link show "$iface" | grep -oP 'bitrate \K\d+' | head -n 1 || true)
  echo "$iface bus=${BUS_INFO:-unknown} state=${STATE:-unknown} bitrate=${BITRATE:-unset}"
done

if [ "$FOUND" -eq 0 ]; then
  echo "No CAN interfaces found"
  exit 1
fi
