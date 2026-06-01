#!/usr/bin/env python3
"""
基础测试 3: 运动控制测试

覆盖 README API:
  - start_control_loop / stop_control_loop
  - MoveJ (阻塞 / 非阻塞)
  - JointCtrl / JointCtrlList
  - is_moving / wait_for_motion / cancel_motion
  - GripperCtrl

前置条件:
  - CAN 接口已配置
  - 机械臂已上电
  - 机械臂周围无障碍物

用法:
  python3 scripts/tests/test_basic_motion.py [--can can0]
"""
import sys
import os
import argparse
import math
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from el_a3_sdk import ELA3Interface, LogLevel

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


def deg2rad(d):
    return d * math.pi / 180.0


def main():
    parser = argparse.ArgumentParser(description="基础测试 3: 运动控制")
    parser.add_argument("--can", default="can0", help="CAN 接口名")
    args = parser.parse_args()

    print("=" * 50)
    print(f" 基础测试 3: 运动控制 ({args.can})")
    print("=" * 50)

    arm = ELA3Interface(
        can_name=args.can,
        logger_level=LogLevel.INFO,
        gravity_feedforward_ratio=1.0,
    )

    try:
        if not arm.ConnectPort():
            log_fail("ConnectPort 失败")
            sys.exit(1)
    except Exception as e:
        log_fail(f"ConnectPort 异常: {e}")
        sys.exit(1)

    arm.EnableArm()
    time.sleep(0.5)

    try:
        _run_motion_tests(arm)
    except KeyboardInterrupt:
        print("\n\n  用户中断")
    except Exception as e:
        log_fail(f"未预期的异常: {e}")
    finally:
        print("\n  --- 清理 ---")
        try:
            arm.stop_control_loop()
        except Exception:
            pass
        try:
            arm.DisableArm()
            arm.DisconnectPort()
            log_pass("清理完成")
        except Exception as e:
            log_fail(f"清理异常: {e}")

        print(f"\n{'=' * 50}")
        print(f" 结果: {PASS} passed, {FAIL} failed")
        print(f"{'=' * 50}")
        sys.exit(0 if FAIL == 0 else 1)


def _run_motion_tests(arm: ELA3Interface):
    # ---- 1. start_control_loop ----
    print("\n  --- start_control_loop ---")
    try:
        arm.start_control_loop(rate_hz=200.0)
        log_pass("start_control_loop(200Hz) 启动成功")
    except Exception as e:
        log_fail(f"start_control_loop 异常: {e}")
        return

    time.sleep(0.5)

    # ---- 2. MoveJ 阻塞模式 - 回零位 ----
    print("\n  --- MoveJ 阻塞 - 回零位 ---")
    try:
        result = arm.MoveJ([0.0] * 6, duration=2.0, block=True)
        if result:
            log_pass("MoveJ 回零位完成")
        else:
            log_fail("MoveJ 回零位返回 False")
    except Exception as e:
        log_fail(f"MoveJ 回零位异常: {e}")

    time.sleep(0.5)

    # ---- 3. MoveJ 阻塞模式 - 安全位置 + 精度验证 ----
    print("\n  --- MoveJ 阻塞 - 精度验证 ---")
    target = [0.0, deg2rad(20), deg2rad(-20), 0.0, 0.0, 0.0]
    try:
        result = arm.MoveJ(target, duration=2.0, block=True)
        joints = arm.GetArmJointMsgs().to_list()[:6]
        max_err = max(abs(a - b) for a, b in zip(joints, target))
        if result and max_err < 0.05:
            log_pass(f"MoveJ 阻塞精度: 最大误差 {max_err:.4f} rad")
        else:
            log_fail(f"MoveJ 阻塞精度不足: 最大误差 {max_err:.4f} rad, "
                     f"result={result}")
    except Exception as e:
        log_fail(f"MoveJ 精度验证异常: {e}")

    # ---- 4. MoveJ 非阻塞模式 + is_moving + wait_for_motion ----
    print("\n  --- MoveJ 非阻塞 + is_moving + wait_for_motion ---")
    try:
        result = arm.MoveJ([0.0] * 6, duration=2.0, block=False)
        if not result:
            log_fail("MoveJ 非阻塞提交返回 False")
        else:
            log_pass("MoveJ 非阻塞提交成功")

        time.sleep(0.1)
        moving = arm.is_moving()
        if moving:
            log_pass("is_moving() 返回 True (轨迹执行中)")
        else:
            log_fail("is_moving() 返回 False (期望 True)")

        done = arm.wait_for_motion(timeout=5.0)
        if done:
            log_pass("wait_for_motion 等待完成")
        else:
            log_fail("wait_for_motion 超时")

        moving_after = arm.is_moving()
        if not moving_after:
            log_pass("运动完成后 is_moving() 返回 False")
        else:
            log_fail("运动完成后 is_moving() 仍为 True")
    except Exception as e:
        log_fail(f"MoveJ 非阻塞测试异常: {e}")

    # ---- 5. cancel_motion ----
    print("\n  --- cancel_motion ---")
    try:
        arm.MoveJ([0.0, deg2rad(15), deg2rad(-15), 0.0, 0.0, 0.0],
                  duration=3.0, block=False)
        time.sleep(0.3)

        if arm.is_moving():
            arm.cancel_motion()
            time.sleep(0.1)
            if not arm.is_moving():
                log_pass("cancel_motion 成功取消轨迹")
            else:
                log_fail("cancel_motion 后 is_moving() 仍为 True")
        else:
            log_fail("cancel_motion 测试: 轨迹未启动")
    except Exception as e:
        log_fail(f"cancel_motion 异常: {e}")

    time.sleep(0.5)

    # ---- 6. JointCtrl ----
    print("\n  --- JointCtrl ---")
    try:
        result = arm.JointCtrl(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        if result:
            log_pass("JointCtrl 回零位调用成功")
        else:
            log_fail("JointCtrl 回零位返回 False")
    except Exception as e:
        log_fail(f"JointCtrl 异常: {e}")

    time.sleep(1.0)

    # ---- 7. JointCtrlList ----
    print("\n  --- JointCtrlList ---")
    try:
        result = arm.JointCtrlList([0.0] * 6)
        if result:
            log_pass("JointCtrlList([0]*6) 调用成功")
        else:
            log_fail("JointCtrlList([0]*6) 返回 False")
    except Exception as e:
        log_fail(f"JointCtrlList 异常: {e}")

    time.sleep(0.5)

    # ---- 8. GripperCtrl ----
    print("\n  --- GripperCtrl ---")
    try:
        joints_before = arm.GetArmJointMsgs().to_list(include_gripper=True)
        gripper_before = joints_before[6] if len(joints_before) >= 7 else None

        result = arm.GripperCtrl(gripper_angle=0.5)
        if result:
            log_pass("GripperCtrl(0.5) 调用成功")
        else:
            log_fail("GripperCtrl(0.5) 返回 False")

        time.sleep(1.5)

        joints_after = arm.GetArmJointMsgs().to_list(include_gripper=True)
        gripper_after = joints_after[6] if len(joints_after) >= 7 else None

        if gripper_before is not None and gripper_after is not None:
            delta = abs(gripper_after - gripper_before)
            if delta > 0.01:
                log_pass(f"夹爪位置变化: {delta:.3f} rad")
            else:
                log_fail(f"夹爪位置未变化: delta={delta:.4f} rad")
    except Exception as e:
        log_fail(f"GripperCtrl 异常: {e}")

    try:
        result = arm.GripperCtrl(gripper_angle=0.0)
        time.sleep(1.0)
        if result:
            log_pass("GripperCtrl(0.0) 回零成功")
        else:
            log_fail("GripperCtrl(0.0) 返回 False")
    except Exception as e:
        log_fail(f"GripperCtrl 回零异常: {e}")

    # ---- 9. MoveJ 回零 (测试结束前) ----
    print("\n  --- MoveJ 回零 (收尾) ---")
    try:
        arm.MoveJ([0.0] * 6, duration=2.0, block=True)
        log_pass("收尾回零完成")
    except Exception as e:
        log_fail(f"收尾回零异常: {e}")

    # ---- 10. stop_control_loop ----
    print("\n  --- stop_control_loop ---")
    try:
        arm.stop_control_loop()
        log_pass("stop_control_loop 成功")
    except Exception as e:
        log_fail(f"stop_control_loop 异常: {e}")


if __name__ == "__main__":
    main()
