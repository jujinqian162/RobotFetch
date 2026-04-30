#!/usr/bin/env bash

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_ROOT="${ROS_ROOT:-/opt/ros}"
ROS_SETUP="${ROS_SETUP:-}"
VENV_DIR="${VENV_DIR:-$ROOT/.venv}"

resolve_ros_setup() {
  if [[ -n "$ROS_SETUP" ]]; then
    echo "$ROS_SETUP"
    return 0
  fi

  if [[ -n "${ROS_DISTRO:-}" && -f "$ROS_ROOT/$ROS_DISTRO/setup.bash" ]]; then
    echo "$ROS_ROOT/$ROS_DISTRO/setup.bash"
    return 0
  fi

  local distros=()
  if [[ -n "${ROS_DISTRO_HINT:-}" ]]; then
    distros+=("$ROS_DISTRO_HINT")
  fi
  distros+=(humble iron jazzy rolling foxy galactic)

  local distro
  for distro in "${distros[@]}"; do
    if [[ -f "$ROS_ROOT/$distro/setup.bash" ]]; then
      echo "$ROS_ROOT/$distro/setup.bash"
      return 0
    fi
  done

  local discovered
  for discovered in "$ROS_ROOT"/*/setup.bash; do
    if [[ -f "$discovered" ]]; then
      echo "$discovered"
      return 0
    fi
  done

  return 1
}

if ! ROS_SETUP="$(resolve_ros_setup)"; then
  echo "No ROS setup script found under: $ROS_ROOT" >&2
  echo "Set ROS_SETUP explicitly, e.g. ROS_SETUP=/opt/ros/humble/setup.bash" >&2
  return 1 2>/dev/null || exit 1
fi

if [[ -f "$ROS_SETUP" ]]; then
  had_nounset=0
  case $- in
    *u*) had_nounset=1 ;;
  esac
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  if [[ "$had_nounset" -eq 1 ]]; then
    set -u
  fi
  unset had_nounset
else
  echo "ROS setup script not found: $ROS_SETUP" >&2
  echo "Tip: set ROS_SETUP or ROS_DISTRO_HINT (e.g. humble)." >&2
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
