#!/usr/bin/env python3
"""
Phase B.2.2: 夹爪控制测试

前置条件:
  - ros2_control 已启动 (el_a3_control.launch.py)
  - gripper_controller 处于 active 状态

用法:
  python3 scripts/tests/test_hw_gripper.py
"""
import subprocess
import time
import sys
import re
import json

PASS = 0
FAIL = 0

GRIPPER_JOINT = "L7_joint"
GRIPPER_CONTROLLER = "gripper_controller"
POSITION_TOLERANCE = 0.1  # rad


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


def send_gripper_position(target):
    torque = 0.65 if target > 0.03 else 0.0
    torque_cmd = (
        f"ros2 topic pub --once /{GRIPPER_CONTROLLER}/torque_limit "
        f'std_msgs/msg/Float64 "{{data: {torque}}}"'
    )
    run_cmd(torque_cmd, timeout=5)

    cmd = (
        f'ros2 topic pub --once /{GRIPPER_CONTROLLER}/joint_trajectory '
        f'trajectory_msgs/msg/JointTrajectory '
        f'"{{joint_names: [\\"{GRIPPER_JOINT}\\"], '
        f'points: [{{positions: [{target}], '
        f'time_from_start: {{sec: 2, nanosec: 0}}}}]}}"'
    )
    _, _, rc = run_cmd(cmd, timeout=10)
    return rc == 0


def get_gripper_position():
    out, _, rc = run_cmd("ros2 topic echo /joint_states --once", timeout=10)
    if rc != 0 or not out:
        return None
    names = re.findall(r"- (L\d+_joint)", out)
    positions_match = re.search(r"position:\s*\n(.*?)velocity:", out, re.DOTALL)
    if positions_match:
        pos_values = re.findall(r"[-\d.]+e?[-+]?\d*", positions_match.group(1))
        pos_values = [float(v) for v in pos_values]
        if GRIPPER_JOINT in names and len(names) == len(pos_values):
            idx = names.index(GRIPPER_JOINT)
            return pos_values[idx]
    return None


def main():
    print("============================================")
    print(" B.2.2  夹爪控制测试")
    print("============================================")

    test_positions = [
        (0.0, "零点"),
        (0.5, "半开"),
        (1.0, "全开"),
        (0.0, "闭合"),
    ]

    for target, desc in test_positions:
        print(f"\n  [测试] 夹爪 -> {target:.1f} rad ({desc})")
        if send_gripper_position(target):
            log_pass(f"轨迹发送成功: {desc}")
        else:
            log_fail(f"轨迹发送失败: {desc}")
            continue

        time.sleep(3)

        current = get_gripper_position()
        if current is not None:
            error = abs(current - target)
            if error < POSITION_TOLERANCE:
                log_pass(f"到位: 当前={current:.3f}, 目标={target:.1f}, 误差={error:.4f}")
            else:
                log_fail(f"误差过大: 当前={current:.3f}, 目标={target:.1f}, 误差={error:.4f}")
        else:
            log_fail(f"无法读取夹爪位置")

    print(f"\n============================================")
    print(f" 结果: {PASS} passed, {FAIL} failed")
    print(f"============================================")
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
