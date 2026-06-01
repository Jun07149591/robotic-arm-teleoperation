#!/bin/bash
# NOTE: el_a3_teleop was removed from this repo; this script only checks workspace + joy/MoveIt deps.
# Xbox controller control system test script

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "=================================="
echo "Xbox Controller Control System Test"
echo "=================================="
echo ""

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check ROS2 environment
echo -n "1. Checking ROS2 environment... "
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}Failed${NC}"
    echo "   Please source ROS2 environment first: source /opt/ros/humble/setup.bash"
    exit 1
else
    echo -e "${GREEN}Passed${NC} (ROS_DISTRO=$ROS_DISTRO)"
fi

# Check workspace (colcon install at el_a3_ros root)
echo -n "2. Checking workspace... "
if [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
    echo -e "${GREEN}Passed${NC}"
else
    echo -e "${RED}Failed${NC}"
    echo "   Workspace not built, please run: colcon build (from el_a3_ros root)"
    exit 1
fi

source "$PROJECT_ROOT/install/setup.bash"

# Check joy package
echo -n "3. Checking joy package... "
if ros2 pkg list | grep -q joy; then
    echo -e "${GREEN}Passed${NC}"
else
    echo -e "${YELLOW}Warning${NC}"
    echo "   joy package not installed, please run: sudo apt install ros-humble-joy"
fi

# Check moveit_servo package
echo -n "4. Checking moveit_servo package... "
if ros2 pkg list | grep -q moveit_servo; then
    echo -e "${GREEN}Passed${NC}"
else
    echo -e "${YELLOW}Warning${NC}"
    echo "   moveit_servo package not installed, please run: sudo apt install ros-humble-moveit-servo"
fi

# Check controller device
echo -n "5. Checking controller device... "
if [ -e "/dev/input/js0" ]; then
    echo -e "${GREEN}Passed${NC}"
    echo "   Device: /dev/input/js0"
    
    # Check permissions
    if [ -r "/dev/input/js0" ]; then
        echo "   Permissions: ${GREEN}Readable${NC}"
    else
        echo "   Permissions: ${YELLOW}Not readable${NC}"
        echo "   Fix: sudo chmod 666 /dev/input/js0"
    fi
else
    echo -e "${YELLOW}Not detected${NC}"
    echo "   Please connect the Xbox controller"
fi

echo ""
echo "=================================="
echo "Test complete!"
echo "=================================="
echo ""
echo "el_a3_teleop was removed; use your own teleop stack if needed."
echo ""
echo "Test controller input:"
echo "   ${GREEN}ros2 run joy joy_node${NC}"
echo "   ${GREEN}ros2 topic echo /joy${NC}"
echo ""




