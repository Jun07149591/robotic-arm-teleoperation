#!/bin/bash
# EL-A3 Xbox/gamepad teleop via ROS2
#
# Usage:
#   ./start_real_xbox_control.sh                          # auto-detect everything
#   ./start_real_xbox_control.sh can1                     # specify CAN interface
#   ./start_real_xbox_control.sh can0 /dev/input/js1      # specify CAN + joystick
#   ./start_real_xbox_control.sh can0 "" zikway_3537_1041 # force a profile

set -e

CAN_INTERFACE="${1:-can0}"
JS_OVERRIDE="${2:-}"
PROFILE_OVERRIDE="${3:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "============================================"
echo "  EL-A3 Gamepad Teleop (ROS2)"
echo "============================================"

# --- Ensure CAN interface is up ---
echo "检查 CAN 接口 ${CAN_INTERFACE}..."
if ! ip link show "${CAN_INTERFACE}" &>/dev/null; then
    echo "错误: CAN 接口 ${CAN_INTERFACE} 不存在"
    echo "  sudo ip link set ${CAN_INTERFACE} up type can bitrate 1000000"
    exit 1
fi

if ! ip link show "${CAN_INTERFACE}" | grep -q "UP"; then
    echo "正在启动 CAN 接口 ${CAN_INTERFACE}..."
    sudo ip link set "${CAN_INTERFACE}" up type can bitrate 1000000
fi
echo "CAN 接口 ${CAN_INTERFACE} 就绪"

# --- Source ROS environment ---
source /opt/ros/humble/setup.bash
if [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
    source "$PROJECT_ROOT/install/setup.bash"
fi

# --- FastDDS SHM workaround ---
export FASTRTPS_DEFAULT_PROFILES_FILE="$PROJECT_ROOT/fastrtps_no_shm.xml"

# --- Build launch arguments ---
LAUNCH_ARGS="can_interface:=${CAN_INTERFACE}"
if [ -n "$JS_OVERRIDE" ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS joy_dev_override:=${JS_OVERRIDE}"
fi
if [ -n "$PROFILE_OVERRIDE" ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS controller_profile_override:=${PROFILE_OVERRIDE}"
fi

echo ""
echo "启动参数: $LAUNCH_ARGS"
echo ""
echo "操作说明:"
echo "  左摇杆 Y/X   → 末端 X/Y 平移"
echo "  LT / RT       → 末端 Z 下/上"
echo "  右摇杆 X/Y   → Yaw / Roll"
echo "  LB / RB       → Pitch"
echo "  A (南)        → 切换速度档"
echo "  B (东)        → 回 Home"
echo "  X (西)        → 回零位"
echo "  Y (北)        → 零力矩模式"
echo "  D-pad ↑↓     → 夹爪"
echo "  Back          → 急停"
echo "============================================"
echo ""

ros2 launch el_a3_teleop real_xbox_teleop.launch.py $LAUNCH_ARGS
