#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3.12}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
VENV_DIR="${VENV_DIR:-$ROOT/.venv}"
export UV_CACHE_DIR="${UV_CACHE_DIR:-/tmp/robotfetch-uv-cache}"

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "Required interpreter not found: $PYTHON_BIN" >&2
  exit 1
fi

if [[ -f "$ROS_SETUP" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  set -u
else
  echo "ROS setup script not found: $ROS_SETUP" >&2
  exit 1
fi

if command -v uv >/dev/null 2>&1; then
  uv venv --clear --seed --python "$PYTHON_BIN" --system-site-packages "$VENV_DIR"
  uv pip install --python "$VENV_DIR/bin/python" -e "$ROOT/BaseDetect" pytest
else
  "$PYTHON_BIN" -m venv --system-site-packages "$VENV_DIR"
  "$VENV_DIR/bin/python" -m pip install -e "$ROOT/BaseDetect" pytest
fi

"$VENV_DIR/bin/python" -c "import yaml, rclpy, basedetect; print('Environment ready:', basedetect.__file__)"
