#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

if [ -d "/ws/ros2_ws/install" ]; then
    source /ws/ros2_ws/install/setup.bash
fi

# Install el_a3_sdk in editable mode (only once, skip if already installed)
if ! pip3 show el_a3_sdk &>/dev/null; then
    pip3 install --no-deps -e /ws 2>/dev/null || true
fi

exec "$@"
