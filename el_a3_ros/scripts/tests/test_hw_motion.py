#!/usr/bin/env python3
"""
Phase B.1.2: 单关节运动测试

前置条件:
  - Phase A 全部通过
  - ros2_control 已启动 (el_a3_control.launch.py)

用法:
  python3 scripts/tests/test_hw_motion.py [--controller arm_controller]

测试内容:
  - 逐个关节移动到中间位置并返回零点
  - 检查位置跟踪误差 < 0.05 rad
"""
import sys
import os
import argparse
import time
import subprocess
import json

PASS = 0
FAIL = 0

ARM_JOINTS = [f"L{i}_joint" for i in range(1, 7)]
GRIPPER_JOINTS = ["L7_joint"]

TEST_POSITIONS = {
    "L1_joint": 0.5,
    "L2_joint": 1.0,
    "L3_joint": -1.0,
    "L4_joint": 0.5,
    "L5_joint": 0.5,
    "L6_joint": 0.5,
}

POSITION_TOLERANCE = 0.05  # rad


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


def send_arm_trajectory(controller, positions, duration_sec=3):
    """发送臂部轨迹"""
    positions_str = str(positions)
    cmd = (
        f'ros2 topic pub --once /{controller}/joint_trajectory '
        f'trajectory_msgs/msg/JointTrajectory '
        f'"{{joint_names: {json.dumps(ARM_JOINTS)}, '
        f'points: [{{positions: {positions_str}, '
        f'time_from_start: {{sec: {duration_sec}, nanosec: 0}}}}]}}"'
    )
    _, _, rc = run_cmd(cmd, timeout=10)
    return rc == 0


def get_joint_positions():
    """从 /joint_states 获取当前关节位置"""
    import re
    out, _, rc = run_cmd("ros2 topic echo /joint_states --once", timeout=10)
    if rc != 0 or not out:
        return None

    names = re.findall(r"- (L\d+_joint)", out)
    positions_match = re.search(r"position:\s*\n(.*?)velocity:", out, re.DOTALL)
    if positions_match:
        pos_values = re.findall(r"[-\d.]+", positions_match.group(1))
        pos_values = [float(v) for v in pos_values]
        if len(names) == len(pos_values):
            return dict(zip(names, pos_values))
    return None


def main():
    parser = argparse.ArgumentParser(description="B.1.2 单关节运动测试")
    parser.add_argument("--controller", default="arm_controller",
                        help="臂部控制器名称")
    args = parser.parse_args()

    print("============================================")
    print(f" B.1.2  单关节运动测试")
    print("============================================")

    # 初始: 回到零点
    print("\n  [测试] 回到零点...")
    home = [0.0] * 6
    if send_arm_trajectory(args.controller, home, duration_sec=5):
        log_pass("发送 home 轨迹成功")
    else:
        log_fail("发送 home 轨迹失败")
        sys.exit(1)
    time.sleep(6)

    # 逐个关节测试
    for joint_name, target_pos in TEST_POSITIONS.items():
        print(f"\n  [测试] {joint_name} -> {target_pos:.2f} rad")
        idx = ARM_JOINTS.index(joint_name)
        positions = [0.0] * 6
        positions[idx] = target_pos

        if not send_arm_trajectory(args.controller, positions, duration_sec=3):
            log_fail(f"{joint_name}: 轨迹发送失败")
            continue

        time.sleep(4)

        current = get_joint_positions()
        if current and joint_name in current:
            error = abs(current[joint_name] - target_pos)
            if error < POSITION_TOLERANCE:
                log_pass(f"{joint_name}: 到位 (误差={error:.4f} rad)")
            else:
                log_fail(f"{joint_name}: 误差过大 ({error:.4f} rad > {POSITION_TOLERANCE})")
        else:
            log_fail(f"{joint_name}: 无法读取当前位置")

        # 回零
        send_arm_trajectory(args.controller, home, duration_sec=3)
        time.sleep(4)

    # 最终回零
    print("\n  [测试] 最终回到零点...")
    send_arm_trajectory(args.controller, home, duration_sec=5)
    time.sleep(6)

    current = get_joint_positions()
    if current:
        max_error = max(abs(current.get(j, 0.0)) for j in ARM_JOINTS)
        if max_error < POSITION_TOLERANCE:
            log_pass(f"最终位置误差: {max_error:.4f} rad")
        else:
            log_fail(f"最终位置误差过大: {max_error:.4f} rad")

    print(f"\n============================================")
    print(f" 结果: {PASS} passed, {FAIL} failed")
    print(f"============================================")
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
