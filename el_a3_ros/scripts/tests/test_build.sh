#!/bin/bash
# ROS2 C++ 编译检查
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

PASS=0
FAIL=0

pass() { echo "  [PASS] $1"; PASS=$((PASS + 1)); }
fail() { echo "  [FAIL] $1"; FAIL=$((FAIL + 1)); }

echo "============================================"
echo " C++ / ROS2 编译检查"
echo "============================================"

cd "$PROJECT_ROOT"

if colcon build \
    --packages-select el_a3_hardware el_a3_description el_a3_moveit_config \
    --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tee /tmp/test_build_output.log; then
    pass "colcon build 成功"
else
    fail "colcon build 失败"
    echo "  查看详情: /tmp/test_build_output.log"
fi

echo ""
echo "============================================"
echo " 结果: $PASS passed, $FAIL failed"
echo "============================================"

[ "$FAIL" -eq 0 ] || exit 1
