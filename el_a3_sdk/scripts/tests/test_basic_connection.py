#!/usr/bin/env python3
"""
基础测试 1: 连接与生命周期测试

覆盖 README API:
  - ConnectPort / DisconnectPort
  - EnableArm / DisableArm
  - GetArmJointMsgs / GetArmJointVelocities / GetArmJointEfforts
  - GetArmEndPoseMsgs (需 Pinocchio，失败则 SKIP)
  - GetArmStatus
  - 温度 / 故障码检查

前置条件:
  - CAN 接口已配置: sudo ./scripts/setup_can.sh can0
  - 机械臂已上电

用法:
  python3 scripts/tests/test_basic_connection.py [--can can0] [--motor-type EL05]
"""
import sys
import os
import argparse
import math
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from el_a3_sdk import ELA3Interface, LogLevel
from el_a3_sdk.protocol import MotorType, MOTOR_PARAMS

PASS = 0
FAIL = 0
SKIP = 0


def log_pass(msg):
    global PASS
    PASS += 1
    print(f"  [PASS] {msg}")


def log_fail(msg):
    global FAIL
    FAIL += 1
    print(f"  [FAIL] {msg}")


def log_skip(msg):
    global SKIP
    SKIP += 1
    print(f"  [SKIP] {msg}")


def build_motor_type_map(wrist_type_str):
    wrist_type = MotorType.RS05 if wrist_type_str == "RS05" else MotorType.EL05
    return {
        1: MotorType.RS00, 2: MotorType.RS00, 3: MotorType.RS00,
        4: wrist_type, 5: wrist_type, 6: wrist_type, 7: wrist_type,
    }


def main():
    parser = argparse.ArgumentParser(description="基础测试 1: 连接与生命周期")
    parser.add_argument("--can", default="can0", help="CAN 接口名")
    parser.add_argument("--motor-type", default="EL05",
                        choices=["EL05", "RS05"], help="腕部电机型号")
    args = parser.parse_args()

    motor_map = build_motor_type_map(args.motor_type)

    print("=" * 50)
    print(f" 基础测试 1: 连接与生命周期 ({args.can}, {args.motor_type})")
    print("=" * 50)

    arm = ELA3Interface(
        can_name=args.can,
        motor_type_map=motor_map,
        logger_level=LogLevel.INFO,
    )

    # ---- 1. ConnectPort ----
    print("\n  --- ConnectPort ---")
    try:
        result = arm.ConnectPort()
        if result:
            log_pass("ConnectPort 返回 True")
        else:
            log_fail("ConnectPort 返回 False")
            print(f"\n  结果: {PASS} passed, {FAIL} failed, {SKIP} skipped")
            sys.exit(1)
    except Exception as e:
        log_fail(f"ConnectPort 异常: {e}")
        print(f"\n  结果: {PASS} passed, {FAIL} failed, {SKIP} skipped")
        sys.exit(1)

    time.sleep(0.5)

    # ---- 2. EnableArm ----
    print("\n  --- EnableArm ---")
    try:
        arm.EnableArm()
        log_pass("EnableArm 成功")
    except Exception as e:
        log_fail(f"EnableArm 异常: {e}")

    time.sleep(1.0)

    # ---- 3. GetArmJointMsgs ----
    print("\n  --- GetArmJointMsgs ---")
    try:
        joints = arm.GetArmJointMsgs()
        jlist = joints.to_list(include_gripper=True)
        if len(jlist) == 7:
            log_pass(f"关节位置 7 个值: {[f'{j:.3f}' for j in jlist]}")
        else:
            log_fail(f"关节位置只有 {len(jlist)} 个值 (期望 7)")

        degs = [j * 180.0 / math.pi for j in jlist]
        all_reasonable = all(abs(d) < 360.0 for d in degs)
        if all_reasonable:
            log_pass(f"关节角度在合理范围 (|angle| < 360°)")
        else:
            log_fail(f"关节角度超出范围: {[f'{d:.1f}°' for d in degs]}")
    except Exception as e:
        log_fail(f"GetArmJointMsgs 异常: {e}")

    # ---- 4. GetArmJointVelocities ----
    print("\n  --- GetArmJointVelocities ---")
    try:
        velocities = arm.GetArmJointVelocities()
        vlist = velocities.to_list(include_gripper=True)
        if len(vlist) == 7:
            log_pass(f"关节速度 7 个值: {[f'{v:.3f}' for v in vlist]}")
        else:
            log_fail(f"关节速度只有 {len(vlist)} 个值 (期望 7)")

        all_still = all(abs(v) < 1.0 for v in vlist)
        if all_still:
            log_pass("静止状态速度在合理范围 (|vel| < 1 rad/s)")
        else:
            log_fail(f"静止状态速度异常: {[f'{v:.3f}' for v in vlist]}")
    except Exception as e:
        log_fail(f"GetArmJointVelocities 异常: {e}")

    # ---- 5. GetArmJointEfforts ----
    print("\n  --- GetArmJointEfforts ---")
    try:
        efforts = arm.GetArmJointEfforts()
        elist = efforts.to_list(include_gripper=True)
        if len(elist) == 7:
            log_pass(f"关节力矩 7 个值: {[f'{e:.3f}' for e in elist]}")
        else:
            log_fail(f"关节力矩只有 {len(elist)} 个值 (期望 7)")

        rs00_limit = MOTOR_PARAMS[MotorType.RS00].t_max
        wrist_type = MotorType.RS05 if args.motor_type == "RS05" else MotorType.EL05
        wrist_limit = MOTOR_PARAMS[wrist_type].t_max
        limits = [rs00_limit] * 3 + [wrist_limit] * 4
        all_in_range = all(abs(e) <= lim for e, lim in zip(elist, limits))
        if all_in_range:
            log_pass("关节力矩在电机限制范围内")
        else:
            log_fail(f"关节力矩超出限制: {[f'{e:.3f}' for e in elist]}")
    except Exception as e:
        log_fail(f"GetArmJointEfforts 异常: {e}")

    # ---- 6. GetArmEndPoseMsgs (需 Pinocchio) ----
    print("\n  --- GetArmEndPoseMsgs ---")
    try:
        pose = arm.GetArmEndPoseMsgs()
        attrs = ['x', 'y', 'z', 'rx', 'ry', 'rz']
        all_exist = all(hasattr(pose, a) for a in attrs)
        if all_exist:
            log_pass(f"末端位姿属性完整: x={pose.x:.4f} y={pose.y:.4f} "
                     f"z={pose.z:.4f} rx={pose.rx:.4f} ry={pose.ry:.4f} "
                     f"rz={pose.rz:.4f}")
            has_value = any(abs(getattr(pose, a)) > 1e-6 for a in attrs)
            if has_value:
                log_pass("末端位姿包含有效数值 (Pinocchio FK 正常)")
            else:
                log_skip("末端位姿全零，可能未安装 Pinocchio")
        else:
            log_fail(f"末端位姿缺少属性")
    except Exception as e:
        log_skip(f"GetArmEndPoseMsgs 跳过 (可能无 Pinocchio): {e}")

    # ---- 7. GetArmStatus ----
    print("\n  --- GetArmStatus ---")
    try:
        status = arm.GetArmStatus()
        log_pass(f"GetArmStatus 返回成功: ctrl_mode={status.ctrl_mode}, "
                 f"move_mode={status.move_mode}")
    except Exception as e:
        log_fail(f"GetArmStatus 异常: {e}")

    # ---- 8. 温度检查 ----
    print("\n  --- 温度检查 ---")
    try:
        feedbacks = arm.GetMotorStates()
        all_ok = True
        for mid, fb in sorted(feedbacks.items()):
            if fb.is_valid:
                if fb.temperature > 80.0:
                    log_fail(f"Motor {mid} 温度过高: {fb.temperature:.1f}°C")
                    all_ok = False
                else:
                    print(f"    Motor {mid}: {fb.temperature:.1f}°C")
        if all_ok:
            log_pass("所有电机温度正常 (<80°C)")
    except Exception as e:
        log_fail(f"温度检查异常: {e}")

    # ---- 9. 故障码检查 ----
    print("\n  --- 故障码检查 ---")
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
        log_fail(f"故障码检查异常: {e}")

    # ---- 10. DisableArm ----
    print("\n  --- DisableArm ---")
    try:
        arm.DisableArm()
        log_pass("DisableArm 成功")
    except Exception as e:
        log_fail(f"DisableArm 异常: {e}")

    # ---- 11. DisconnectPort ----
    print("\n  --- DisconnectPort ---")
    try:
        arm.DisconnectPort()
        log_pass("DisconnectPort 成功")
    except Exception as e:
        log_fail(f"DisconnectPort 异常: {e}")

    print(f"\n{'=' * 50}")
    print(f" 结果: {PASS} passed, {FAIL} failed, {SKIP} skipped")
    print(f"{'=' * 50}")
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
