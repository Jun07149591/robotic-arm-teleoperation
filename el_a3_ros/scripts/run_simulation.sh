#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${ROS_WS}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble/setup.bash" >&2
  echo "Install ROS 2 Humble on Ubuntu 22.04/WSL2, then run this script again." >&2
  exit 1
fi

source /opt/ros/humble/setup.bash

if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon was not found. Install it with:" >&2
  echo "  sudo apt install python3-colcon-common-extensions" >&2
  exit 1
fi

colcon build --symlink-install --packages-select el_a3_description el_a3_sim
source install/setup.bash

ros2 launch el_a3_sim sim.launch.py "$@"
