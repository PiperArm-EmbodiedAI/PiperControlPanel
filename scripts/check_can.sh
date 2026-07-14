#!/usr/bin/env bash
set -euo pipefail

CAN_IFACE="${1:-can0}"

echo "[check_can] interface=${CAN_IFACE}"

if ! command -v ip >/dev/null 2>&1; then
  echo "ip command not found. Install iproute2 first."
  exit 1
fi

if ! ip link show "${CAN_IFACE}" >/dev/null 2>&1; then
  echo "Interface ${CAN_IFACE} not found"
  exit 1
fi

ip -details link show "${CAN_IFACE}"

STATE=$(ip -details link show "${CAN_IFACE}" | grep -oP 'state \K\S+' | head -n 1 || true)
BITRATE=$(ip -details link show "${CAN_IFACE}" | grep -oP 'bitrate \K\d+' | head -n 1 || true)
BUS_INFO=""
if command -v ethtool >/dev/null 2>&1; then
  BUS_INFO=$(ethtool -i "${CAN_IFACE}" 2>/dev/null | grep 'bus-info' | awk '{print $2}' || true)
fi

echo "[check_can] state=${STATE:-unknown} bitrate=${BITRATE:-unset} bus=${BUS_INFO:-unknown}"
