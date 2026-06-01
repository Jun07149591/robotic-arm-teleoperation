#!/usr/bin/env python3
"""
Phase B.2.1: 零力矩模式测试

前置条件:
  - ros2_control 已启动 (el_a3_control.launch.py)

测试内容:
  - 通过 switch_controller 激活 zero_torque_controller
  - 读取关节状态确认模式切换
  - 切回 arm_controller 确认关节锁定

用法:
  python3 scripts/tests/test_hw_zero_torque.py
"""
import subprocess
import time
import sys

PASS = 0
FAIL = 0


def log_pass(msg):
    global PASS
    PASS += 1
    print(f"  [PASS] {msg}")


def log_fail(msg):
    global FAIL
    FAIL += 1
    print(f"  [FAIL] {msg}")


def run_cmd(cmd, timeout=15):
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=timeout
        )
        return result.stdout.strip(), result.stderr.strip(), result.returncode
    except subprocess.TimeoutExpired:
        return "", "TIMEOUT", -1


def switch_controllers(activate, deactivate):
    activate_str = str(activate).replace("'", '"')
    deactivate_str = str(deactivate).replace("'", '"')
    cmd = (
        f'ros2 service call /controller_manager/switch_controller '
        f'controller_manager_msgs/srv/SwitchController '
        f'"{{activate_controllers: {activate_str}, '
        f'deactivate_controllers: {deactivate_str}, '
        f'strictness: 1}}"'
    )
    out, _, rc = run_cmd(cmd, timeout=10)
    return "True" in out or "true" in out


def check_controller_state(name, expected_state):
    out, _, _ = run_cmd("ros2 control list_controllers")
    if name in out:
        line = out.split(name)[1].split("\n")[0]
        if expected_state in line:
            return True
    return False


def main():
    print("============================================")
    print(" B.2.1  零力矩模式测试")
    print("============================================")

    # 1. 确认初始状态: arm_controller active
    print("\n  [测试] 初始状态: arm_controller 应为 active")
    if check_controller_state("arm_controller", "active"):
        log_pass("arm_controller [active]")
    else:
        log_fail("arm_controller 不是 active (请先启动 ros2_control)")
        sys.exit(1)

    # 2. 切换到零力矩模式
    print("\n  [测试] 切换到 zero_torque_controller...")
    if switch_controllers(["zero_torque_controller"], ["arm_controller"]):
        log_pass("switch_controller 调用成功")
    else:
        log_fail("switch_controller 调用失败")

    time.sleep(2)

    if check_controller_state("zero_torque_controller", "active"):
        log_pass("zero_torque_controller [active]")
    else:
        log_fail("zero_torque_controller 未激活")

    if check_controller_state("arm_controller", "inactive"):
        log_pass("arm_controller [inactive]")
    else:
        log_fail("arm_controller 未停用")

    # 3. 零力矩模式下等待（手动拖动验证）
    print("\n  *** 零力矩模式已激活 ***")
    print("  请尝试手动拖动机械臂，确认可自由移动。")
    print("  按 Enter 继续测试...")
    try:
        input()
    except EOFError:
        time.sleep(3)

    # 4. 切回 arm_controller
    print("\n  [测试] 切回 arm_controller...")
    if switch_controllers(["arm_controller"], ["zero_torque_controller"]):
        log_pass("切回 arm_controller 成功")
    else:
        log_fail("切回 arm_controller 失败")

    time.sleep(2)

    if check_controller_state("arm_controller", "active"):
        log_pass("arm_controller [active] (关节锁定)")
    else:
        log_fail("arm_controller 未恢复到 active")

    print(f"\n============================================")
    print(f" 结果: {PASS} passed, {FAIL} failed")
    print(f"============================================")
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
