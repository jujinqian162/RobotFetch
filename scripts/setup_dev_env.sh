#!/usr/bin/env bash
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  echo "Run this script instead of sourcing it: ./scripts/setup_dev_env.sh" >&2
  return 1
fi

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-}"
ROS_ROOT="${ROS_ROOT:-/opt/ros}"
ROS_SETUP="${ROS_SETUP:-}"
VENV_DIR="${VENV_DIR:-$ROOT/.venv}"
export UV_CACHE_DIR="${UV_CACHE_DIR:-/tmp/robotfetch-uv-cache}"

resolve_ros_setup() {
  if [[ -n "$ROS_SETUP" ]]; then
    echo "$ROS_SETUP"
    return 0
  fi

  if [[ -n "${ROS_DISTRO:-}" && -f "$ROS_ROOT/$ROS_DISTRO/setup.bash" ]]; then
    echo "$ROS_ROOT/$ROS_DISTRO/setup.bash"
    return 0
  fi

  if [[ -n "${ROS_DISTRO_HINT:-}" && -f "$ROS_ROOT/$ROS_DISTRO_HINT/setup.bash" ]]; then
    echo "$ROS_ROOT/$ROS_DISTRO_HINT/setup.bash"
    return 0
  fi

  local discovered
  for discovered in "$ROS_ROOT"/*/setup.bash; do
    if [[ -f "$discovered" ]]; then
      echo "$discovered"
      return 0
    fi
  done

  return 1
}

resolve_python_bin() {
  if [[ -n "$PYTHON_BIN" ]]; then
    echo "$PYTHON_BIN"
    return 0
  fi

  if command -v python3 >/dev/null 2>&1; then
    command -v python3
    return 0
  fi

  if command -v python >/dev/null 2>&1; then
    command -v python
    return 0
  fi

  return 1
}

if ! ROS_SETUP="$(resolve_ros_setup)"; then
  echo "No ROS setup script found under: $ROS_ROOT" >&2
  echo "Set ROS_SETUP explicitly, e.g. ROS_SETUP=/opt/ros/<distro>/setup.bash" >&2
  exit 1
fi

if [[ -f "$ROS_SETUP" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  set -u
else
  echo "ROS setup script not found: $ROS_SETUP" >&2
  echo "Set ROS_SETUP explicitly, e.g. ROS_SETUP=/opt/ros/<distro>/setup.bash" >&2
  exit 1
fi

if ! PYTHON_BIN="$(resolve_python_bin)"; then
  echo "No Python interpreter found. Set PYTHON_BIN explicitly." >&2
  exit 1
fi

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "Required interpreter is not executable: $PYTHON_BIN" >&2
  exit 1
fi

echo "Using ROS setup: $ROS_SETUP"
echo "Using Python: $PYTHON_BIN"

if command -v uv >/dev/null 2>&1; then
  uv venv --clear --seed --python "$PYTHON_BIN" --system-site-packages "$VENV_DIR"
  uv pip install --python "$VENV_DIR/bin/python" -e "$ROOT/BaseDetect" pytest
else
  "$PYTHON_BIN" -m venv --system-site-packages "$VENV_DIR"
  "$VENV_DIR/bin/python" -m pip install -e "$ROOT/BaseDetect" pytest
fi

"$VENV_DIR/bin/python" -c "import yaml, rclpy, basedetect; print('Environment ready:', basedetect.__file__)"
