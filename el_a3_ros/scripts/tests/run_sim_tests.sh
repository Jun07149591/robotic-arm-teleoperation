#!/bin/bash
# Phase A 仿真测试统一运行器
# 按顺序执行 A.1 -> A.2 -> A.3，任一步骤失败则终止
# 用法:
#   ./scripts/tests/run_sim_tests.sh          # 运行全部 Phase A
#   ./scripts/tests/run_sim_tests.sh --quick   # 仅 A.1 + A.2（不启动 ROS）
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
QUICK_MODE=false

if [ "$1" = "--quick" ]; then
    QUICK_MODE=true
fi

# Source ROS2 environment
if [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
    source "$PROJECT_ROOT/install/setup.bash"
fi

echo "========================================================"
echo "  EL-A3 Phase A: 仿真测试"
echo "  $(date)"
echo "========================================================"
echo ""

# -------------------------------------------------------
# A.1 编译 + URDF/Xacro 验证
# -------------------------------------------------------
echo "========================================"
echo " [A.1] 编译检查"
echo "========================================"
bash "$SCRIPT_DIR/test_build.sh"
echo ""

echo "========================================"
echo " [A.1] URDF/Xacro 生成验证"
echo "========================================"
bash "$SCRIPT_DIR/test_xacro_gen.sh"
echo ""

# -------------------------------------------------------
# A.2 SDK 单元测试
# -------------------------------------------------------
echo "========================================"
echo " [A.2] Python SDK 单元测试"
echo "========================================"
cd "$PROJECT_ROOT"
python3 -m pytest \
    "$SCRIPT_DIR/test_protocol.py" \
    "$SCRIPT_DIR/test_can_codec.py" \
    "$SCRIPT_DIR/test_state_machine.py" \
    "$SCRIPT_DIR/test_joint_config.py" \
    -v --tb=short
echo ""

if [ "$QUICK_MODE" = true ]; then
    echo "========================================"
    echo " --quick 模式: 跳过 A.3 Mock 集成测试"
    echo "========================================"
    echo ""
    echo "========================================================"
    echo "  Phase A (quick) 全部通过!"
    echo "========================================================"
    exit 0
fi

# -------------------------------------------------------
# A.3 Mock 仿真集成测试
# -------------------------------------------------------
echo "========================================"
echo " [A.3] Mock 仿真集成测试"
echo "========================================"
python3 "$SCRIPT_DIR/test_mock_launch.py"
echo ""

echo "========================================================"
echo "  Phase A 全部通过! 可进入 Phase B 实物测试。"
echo "========================================================"
