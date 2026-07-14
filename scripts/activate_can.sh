#!/usr/bin/env bash
set -euo pipefail

TARGET_CAN_NAME="${1:-can0}"
TARGET_BITRATE="${2:-1000000}"
USB_ADDRESS="${3:-}"

if ! command -v ip >/dev/null 2>&1; then
  echo "ip command not found. Install iproute2 first."
  exit 1
fi

if ! command -v ethtool >/dev/null 2>&1; then
  echo "ethtool not found. Install it with: sudo apt update && sudo apt install ethtool"
  exit 1
fi

if ! command -v candump >/dev/null 2>&1; then
  echo "can-utils not found in PATH. Install it with: sudo apt update && sudo apt install can-utils"
  exit 1
fi

mapfile -t CAN_INTERFACES < <(ip -br link show type can | awk '{print $1}')
if [ "${#CAN_INTERFACES[@]}" -eq 0 ]; then
  echo "No CAN interfaces found"
  exit 1
fi

resolve_interface() {
  local desired_usb="$1"
  if [ -n "$desired_usb" ]; then
    for iface in "${CAN_INTERFACES[@]}"; do
      local bus_info
      bus_info=$(ethtool -i "$iface" 2>/dev/null | grep 'bus-info' | awk '{print $2}' || true)
      if [ "$bus_info" = "$desired_usb" ]; then
        printf '%s\n' "$iface"
        return 0
      fi
    done
    return 1
  fi

  if [ "${#CAN_INTERFACES[@]}" -eq 1 ]; then
    printf '%s\n' "${CAN_INTERFACES[0]}"
    return 0
  fi

  return 2
}

INTERFACE_NAME=""
if ! INTERFACE_NAME=$(resolve_interface "$USB_ADDRESS"); then
  status=$?
  if [ "$status" -eq 1 ]; then
    echo "Could not find a CAN interface for USB address: $USB_ADDRESS"
  else
    echo "Multiple CAN interfaces found. Provide the USB address as the third argument."
  fi
  echo "Detected interfaces:"
  bash "$(dirname "$0")/find_all_can_port.sh" || true
  exit 1
fi

CURRENT_STATE=$(ip -details link show "$INTERFACE_NAME" | grep -oP 'state \K\S+' | head -n 1 || true)
CURRENT_BITRATE=$(ip -details link show "$INTERFACE_NAME" | grep -oP 'bitrate \K\d+' | head -n 1 || true)
BUS_INFO=$(ethtool -i "$INTERFACE_NAME" 2>/dev/null | grep 'bus-info' | awk '{print $2}' || true)

echo "[activate_can] interface=$INTERFACE_NAME target_name=$TARGET_CAN_NAME target_bitrate=$TARGET_BITRATE bus=${BUS_INFO:-unknown}"

action_needed=0
if [ "$CURRENT_STATE" != "UP" ]; then
  action_needed=1
fi
if [ "$CURRENT_BITRATE" != "$TARGET_BITRATE" ]; then
  action_needed=1
fi
if [ "$INTERFACE_NAME" != "$TARGET_CAN_NAME" ]; then
  action_needed=1
fi

if [ "$action_needed" -eq 0 ]; then
  echo "[activate_can] interface already configured"
  bash "$(dirname "$0")/check_can.sh" "$TARGET_CAN_NAME"
  exit 0
fi

sudo ip link set "$INTERFACE_NAME" down || true
sudo ip link set "$INTERFACE_NAME" type can bitrate "$TARGET_BITRATE"

if [ "$INTERFACE_NAME" != "$TARGET_CAN_NAME" ]; then
  sudo ip link set "$INTERFACE_NAME" name "$TARGET_CAN_NAME"
  INTERFACE_NAME="$TARGET_CAN_NAME"
fi

sudo ip link set "$INTERFACE_NAME" up

echo "[activate_can] CAN interface is up"
bash "$(dirname "$0")/check_can.sh" "$INTERFACE_NAME"
