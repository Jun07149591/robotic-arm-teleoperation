#!/bin/bash
# Phase A.1.3 + A.1.4: URDF/Xacro 参数化验证 + 配置完整性检查
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

PASS=0
FAIL=0

pass() { echo "  [PASS] $1"; PASS=$((PASS + 1)); }
fail() { echo "  [FAIL] $1"; FAIL=$((FAIL + 1)); return 1; }

check_grep() {
    local file="$1" pattern="$2" desc="$3"
    if grep -q "$pattern" "$file"; then
        pass "$desc"
    else
        fail "$desc (pattern '$pattern' not found)"
    fi
}

check_grep_absent() {
    local file="$1" pattern="$2" desc="$3"
    if grep -q "$pattern" "$file"; then
        fail "$desc (unexpected pattern '$pattern' found)"
    else
        pass "$desc"
    fi
}

# Source ROS2 if available
if [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
    source "$PROJECT_ROOT/install/setup.bash"
fi

echo "============================================"
echo " A.1.3  URDF/Xacro 参数化验证"
echo "============================================"

XACRO_FILE="$PROJECT_ROOT/el_a3_description/urdf/el_a3.urdf.xacro"
URDF_EL05="/tmp/test_urdf_el05.xml"
URDF_RS05="/tmp/test_urdf_rs05.xml"

echo "  生成 EL05 URDF..."
xacro "$XACRO_FILE" use_mock_hardware:=true > "$URDF_EL05" 2>&1
pass "EL05 xacro 生成成功"

echo "  生成 RS05 URDF..."
xacro "$XACRO_FILE" use_mock_hardware:=true wrist_motor_type:=RS05 > "$URDF_RS05" 2>&1
pass "RS05 xacro 生成成功"

echo ""
echo "--- EL05 检查 ---"

# L4 effort in EL05 should be 6.0
if grep -P 'name="L4_joint"' "$URDF_EL05" -A5 | grep -q 'effort="6.0"'; then
    pass "EL05: L4 effort=6.0"
else
    fail "EL05: L4 effort 不是 6.0"
fi

if grep -P 'name="L7_joint"' "$URDF_EL05" -A5 | grep -q 'effort="6.0"'; then
    pass "EL05: L7 effort=6.0"
else
    fail "EL05: L7 effort 不是 6.0"
fi

# L1 effort should be 14 in both
if grep -P 'name="L1_joint"' "$URDF_EL05" -A5 | grep -q 'effort="14"'; then
    pass "EL05: L1 effort=14 (不受影响)"
else
    fail "EL05: L1 effort 不是 14"
fi

# ros2_control motor_type for L4
if grep -A3 'name="L4_joint"' "$URDF_EL05" | grep -q '>EL05<'; then
    pass "EL05: ros2_control L4 motor_type=EL05"
else
    fail "EL05: ros2_control L4 motor_type 不是 EL05"
fi

# effort command interface for L4
if grep -A20 'name="L4_joint"' "$URDF_EL05" | grep -A2 'name="effort"' | grep -q '>6.0<'; then
    pass "EL05: ros2_control L4 effort max=6.0"
else
    fail "EL05: ros2_control L4 effort max 不是 6.0"
fi

echo ""
echo "--- RS05 检查 ---"

if grep -P 'name="L4_joint"' "$URDF_RS05" -A5 | grep -q 'effort="5.5"'; then
    pass "RS05: L4 effort=5.5"
else
    fail "RS05: L4 effort 不是 5.5"
fi

if grep -P 'name="L5_joint"' "$URDF_RS05" -A5 | grep -q 'effort="5.5"'; then
    pass "RS05: L5 effort=5.5"
else
    fail "RS05: L5 effort 不是 5.5"
fi

if grep -P 'name="L6_joint"' "$URDF_RS05" -A5 | grep -q 'effort="5.5"'; then
    pass "RS05: L6 effort=5.5"
else
    fail "RS05: L6 effort 不是 5.5"
fi

if grep -P 'name="L7_joint"' "$URDF_RS05" -A5 | grep -q 'effort="5.5"'; then
    pass "RS05: L7 effort=5.5"
else
    fail "RS05: L7 effort 不是 5.5"
fi

if grep -P 'name="L1_joint"' "$URDF_RS05" -A5 | grep -q 'effort="14"'; then
    pass "RS05: L1 effort=14 (不受影响)"
else
    fail "RS05: L1 effort 不是 14"
fi

if grep -A3 'name="L4_joint"' "$URDF_RS05" | grep -q '>RS05<'; then
    pass "RS05: ros2_control L4 motor_type=RS05"
else
    fail "RS05: ros2_control L4 motor_type 不是 RS05"
fi

if grep -A20 'name="L4_joint"' "$URDF_RS05" | grep -A2 'name="effort"' | grep -q '>5.5<'; then
    pass "RS05: ros2_control L4 effort max=5.5"
else
    fail "RS05: ros2_control L4 effort max 不是 5.5"
fi

if grep -A20 'name="L4_joint"' "$URDF_RS05" | grep -A2 'name="effort"' | grep -q '>-5.5<'; then
    pass "RS05: ros2_control L4 effort min=-5.5"
else
    fail "RS05: ros2_control L4 effort min 不是 -5.5"
fi

echo ""
echo "============================================"
echo " A.1.4  配置文件完整性检查"
echo "============================================"

CTRL_YAML="$PROJECT_ROOT/el_a3_description/config/el_a3_controllers.yaml"
MOVEIT_CTRL="$PROJECT_ROOT/el_a3_moveit_config/config/moveit_controllers.yaml"
SRDF_FILE="$PROJECT_ROOT/el_a3_moveit_config/config/el_a3.srdf"

echo "  检查 el_a3_controllers.yaml..."
check_grep "$CTRL_YAML" "arm_controller" "arm_controller 存在"
check_grep "$CTRL_YAML" "gripper_controller" "gripper_controller 存在"
check_grep "$CTRL_YAML" "zero_torque_controller" "zero_torque_controller 存在"

echo "  检查 moveit_controllers.yaml..."
check_grep "$MOVEIT_CTRL" "arm_controller" "MoveIt arm_controller 存在"
check_grep "$MOVEIT_CTRL" "gripper_controller" "MoveIt gripper_controller 存在"

echo "  检查 el_a3.srdf..."
check_grep "$SRDF_FILE" 'group name="arm"' "SRDF arm 规划组存在"
check_grep "$SRDF_FILE" 'group name="gripper"' "SRDF gripper 规划组存在"
check_grep "$SRDF_FILE" 'group_state name="open"' "SRDF open state 存在"
check_grep "$SRDF_FILE" 'group_state name="close"' "SRDF close state 存在"

echo ""
echo "============================================"
echo " 结果: $PASS passed, $FAIL failed"
echo "============================================"

rm -f "$URDF_EL05" "$URDF_RS05"
[ "$FAIL" -eq 0 ] || exit 1
