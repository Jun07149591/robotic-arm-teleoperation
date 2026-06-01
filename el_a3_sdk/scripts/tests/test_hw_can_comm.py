#!/usr/bin/env python3
"""
Phase B.1.1: 真实 CAN 通信测试

前置条件:
  - Phase A 全部通过
  - CAN 接口已配置: sudo ./scripts/setup_can.sh can0
  - 机械臂已上电

用法:
  python3 scripts/tests/test_hw_can_comm.py [--can can0] [--motor-type EL05]
"""
import sys
import os
import argparse
import time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from el_a3_sdk import ELA3Interface, MotorType
from el_a3_sdk.protocol import MOTOR_PARAMS

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


def build_motor_type_map(wrist_type_str):
    wrist_type = MotorType.RS05 if wrist_type_str == "RS05" else MotorType.EL05
    return {
        1: MotorType.RS00, 2: MotorType.RS00, 3: MotorType.RS00,
        4: wrist_type, 5: wrist_type, 6: wrist_type, 7: wrist_type,
    }


def main():
    parser = argparse.ArgumentParser(description="B.1.1 CAN 通信测试")
    parser.add_argument("--can", default="can0", help="CAN 接口名")
    parser.add_argument("--motor-type", default="EL05", choices=["EL05", "RS05"],
                        help="腕部电机型号")
    args = parser.parse_args()

    motor_map = build_motor_type_map(args.motor_type)

    print("============================================")
    print(f" B.1.1  CAN 通信测试 ({args.can}, {args.motor_type})")
    print("============================================")

    arm = ELA3Interface(
        can_name=args.can,
        motor_type_map=motor_map,
    )

    # 1. 连接
    print("\n  [测试] 连接 CAN 端口...")
    try:
        arm.ConnectPort()
        log_pass(f"ConnectPort({args.can}) 成功")
    except Exception as e:
        log_fail(f"ConnectPort 失败: {e}")
        print(f"\n  结果: {PASS} passed, {FAIL} failed")
        sys.exit(1)

    time.sleep(0.5)

    # 2. 使能
    print("\n  [测试] 使能电机...")
    try:
        arm.EnableArm()
        log_pass("EnableArm 成功")
    except Exception as e:
        log_fail(f"EnableArm 失败: {e}")

    time.sleep(1.0)

    # 3. 读取关节状态
    print("\n  [测试] 读取关节状态...")
    try:
        states = arm.GetArmJointStates()
        joints = states.to_list(include_gripper=True)
        if len(joints) == 7:
            log_pass(f"关节状态包含 7 个值: {[f'{j:.3f}' for j in joints]}")
        else:
            log_fail(f"关节状态只有 {len(joints)} 个值")
    except Exception as e:
        log_fail(f"GetArmJointStates 失败: {e}")

    # 4. 读取电机反馈
    print("\n  [测试] 读取电机反馈...")
    try:
        feedbacks = arm.GetMotorStates()
        valid_count = sum(1 for fb in feedbacks.values() if fb.is_valid)
        if valid_count >= 7:
            log_pass(f"{valid_count} 个电机有效反馈")
            for mid, fb in sorted(feedbacks.items()):
                if fb.is_valid:
                    print(f"    Motor {mid}: pos={fb.position:.3f} rad, "
                          f"vel={fb.velocity:.3f} rad/s, "
                          f"torque={fb.torque:.3f} Nm, "
                          f"temp={fb.temperature:.1f}°C, "
                          f"fault={fb.fault_code}")
        else:
            log_fail(f"只有 {valid_count} 个电机有效 (期望 >=7)")
    except Exception as e:
        log_fail(f"GetMotorStates 失败: {e}")

    # 5. 温度检查
    print("\n  [测试] 温度检查...")
    try:
        feedbacks = arm.GetMotorStates()
        all_ok = True
        for mid, fb in sorted(feedbacks.items()):
            if fb.is_valid and fb.temperature > 80.0:
                log_fail(f"Motor {mid} 温度过高: {fb.temperature}°C")
                all_ok = False
        if all_ok:
            log_pass("所有电机温度正常 (<80°C)")
    except Exception as e:
        log_fail(f"温度检查失败: {e}")

    # 6. 故障码检查
    print("\n  [测试] 故障码检查...")
    try:
        feedbacks = arm.GetMotorStates()
        faults = {mid: fb.fault_code for mid, fb in feedbacks.items()
                  if fb.is_valid and fb.fault_code != 0}
        if not faults:
            log_pass("无故障码")
        else:
            for mid, code in faults.items():
                log_fail(f"Motor {mid} 故障码: {code}")
    except Exception as e:
        log_fail(f"故障码检查失败: {e}")

    # 7. 禁用并断开
    print("\n  [测试] 禁用电机...")
    try:
        arm.DisableArm()
        log_pass("DisableArm 成功")
    except Exception as e:
        log_fail(f"DisableArm 失败: {e}")

    print(f"\n============================================")
    print(f" 结果: {PASS} passed, {FAIL} failed")
    print(f"============================================")
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
