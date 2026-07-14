#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-mock}"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python}"

if [[ -x "/home/dig/anaconda3/envs/piper_tool/bin/python" ]]; then
  PYTHON_BIN="/home/dig/anaconda3/envs/piper_tool/bin/python"
elif [[ -x "/home/leeviy/miniconda3/envs/PiperArm/bin/python" ]]; then
  PYTHON_BIN="/home/leeviy/miniconda3/envs/PiperArm/bin/python"
fi

cd "${PROJECT_DIR}"

if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" && "${QT_QPA_PLATFORM:-}" != "offscreen" ]]; then
  cat <<'EOF'
No graphical display was detected.

If you are running over SSH, start a desktop session/VNC on the VM, or reconnect with X11 forwarding:
  ssh -X user@host

For local VM usage, open a terminal inside the Ubuntu desktop and run this script there.
For headless smoke testing only, use:
  QT_QPA_PLATFORM=offscreen bash scripts/launch_gui.sh mock
EOF
  exit 1
fi

"${PYTHON_BIN}" -m src.gui --mode "${MODE}"
