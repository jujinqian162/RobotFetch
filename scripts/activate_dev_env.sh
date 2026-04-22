#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
VENV_DIR="${VENV_DIR:-$ROOT/.venv}"

if [[ -f "$ROS_SETUP" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  set -u
else
  echo "ROS setup script not found: $ROS_SETUP" >&2
  return 1 2>/dev/null || exit 1
fi

if [[ -f "$VENV_DIR/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source "$VENV_DIR/bin/activate"
else
  echo "Virtualenv not found: $VENV_DIR" >&2
  echo "Run scripts/setup_dev_env.sh first." >&2
  return 1 2>/dev/null || exit 1
fi
