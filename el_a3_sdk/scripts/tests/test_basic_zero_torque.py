#!/usr/bin/env python3
"""
基础测试 5: 零力矩模式测试

覆盖 README API:
  - ZeroTorqueMode
  - ZeroTorqueModeWithGravity
  - ComputeGravityTorques (辅助验证)

前置条件:
  - CAN 接口已配置
  - 机械臂已上电

用法:
  python3 scripts/tests/test_basic_zero_torque.py [--can can0]
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
SKIP = 0

EFFORT_THRESHOLD = 3.0  # Nm，零力矩模式下允许的最大力矩


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


def main():
    parser = argparse.ArgumentParser(description="基础测试 5: 零力矩模式")
    parser.add_argument("--can", default="can0", help="CAN 接口名")
    args = parser.parse_args()

    print("=" * 50)
    print(f" 基础测试 5: 零力矩模式 ({args.can})")
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
        _run_zero_torque_tests(arm)
    except KeyboardInterrupt:
        print("\n\n  用户中断")
    except Exception as e:
        log_fail(f"未预期的异常: {e}")
    finally:
        print("\n  --- 清理 ---")
        try:
            arm.ZeroTorqueMode(enable=False)
        except Exception:
            pass
        try:
            arm.ZeroTorqueModeWithGravity(enable=False)
        except Exception:
            pass
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
        print(f" 结果: {PASS} passed, {FAIL} failed, {SKIP} skipped")
        print(f"{'=' * 50}")
        sys.exit(0 if FAIL == 0 else 1)


def _run_zero_torque_tests(arm: ELA3Interface):
    arm.start_control_loop(rate_hz=200.0)
    time.sleep(0.3)

    # ---- 1. ZeroTorqueMode 进入 ----
    print("\n  --- ZeroTorqueMode (基础) ---")
    try:
        result = arm.ZeroTorqueMode(enable=True, kd=0.5)
        if result:
            log_pass("ZeroTorqueMode(True, kd=0.5) 进入成功")
        else:
            log_fail("ZeroTorqueMode(True) 返回 False")
    except Exception as e:
        log_fail(f"ZeroTorqueMode(True) 异常: {e}")

    time.sleep(1.0)

    # 验证力矩
    try:
        efforts = arm.GetArmJointEfforts()
        elist = efforts.to_list()[:6]
        max_effort = max(abs(e) for e in elist)
        print(f"    力矩: {[f'{e:.2f}' for e in elist]} Nm")
        if max_effort < EFFORT_THRESHOLD:
            log_pass(f"零力矩模式力矩合理: max={max_effort:.2f} Nm")
        else:
            log_fail(f"零力矩模式力矩偏大: max={max_effort:.2f} Nm "
                     f"(阈值 {EFFORT_THRESHOLD} Nm，可能受重力影响)")
    except Exception as e:
        log_fail(f"读取力矩异常: {e}")

    # ---- 2. ZeroTorqueMode 退出 ----
    try:
        result = arm.ZeroTorqueMode(enable=False)
        if result:
            log_pass("ZeroTorqueMode(False) 退出成功")
        else:
            log_fail("ZeroTorqueMode(False) 返回 False")
    except Exception as e:
        log_fail(f"ZeroTorqueMode(False) 异常: {e}")

    time.sleep(0.5)

    # ---- 3. ZeroTorqueModeWithGravity 进入 ----
    print("\n  --- ZeroTorqueModeWithGravity (重力补偿) ---")
    try:
        result = arm.ZeroTorqueModeWithGravity(enable=True, kd=0.5)
        if result:
            log_pass("ZeroTorqueModeWithGravity(True, kd=0.5) 进入成功")
        else:
            log_fail("ZeroTorqueModeWithGravity(True) 返回 False")
    except Exception as e:
        log_fail(f"ZeroTorqueModeWithGravity(True) 异常: {e}")

    time.sleep(1.5)

    # 验证力矩 (带重力补偿应更接近零)
    try:
        efforts = arm.GetArmJointEfforts()
        elist = efforts.to_list()[:6]
        max_effort = max(abs(e) for e in elist)
        print(f"    力矩: {[f'{e:.2f}' for e in elist]} Nm")
        if max_effort < EFFORT_THRESHOLD:
            log_pass(f"重力补偿零力矩力矩合理: max={max_effort:.2f} Nm")
        else:
            log_fail(f"重力补偿零力矩力矩偏大: max={max_effort:.2f} Nm")
    except Exception as e:
        log_fail(f"读取力矩异常: {e}")

    # 验证重力补偿力矩
    try:
        grav = arm.ComputeGravityTorques()
        if len(grav) >= 6 and not all(g == 0.0 for g in grav):
            log_pass(f"重力补偿力矩: {[f'{g:.3f}' for g in grav]} Nm")
        else:
            log_skip("重力补偿力矩全零 (可能无 Pinocchio)")
    except Exception as e:
        log_skip(f"ComputeGravityTorques 跳过: {e}")

    # ---- 4. ZeroTorqueModeWithGravity 退出 ----
    try:
        result = arm.ZeroTorqueModeWithGravity(enable=False)
        if result:
            log_pass("ZeroTorqueModeWithGravity(False) 退出成功")
        else:
            log_fail("ZeroTorqueModeWithGravity(False) 返回 False")
    except Exception as e:
        log_fail(f"ZeroTorqueModeWithGravity(False) 异常: {e}")

    time.sleep(0.5)

    # ---- 5. 重复进入/退出测试 ----
    print("\n  --- 重复进入/退出测试 ---")
    try:
        arm.ZeroTorqueMode(enable=True, kd=0.3)
        time.sleep(0.5)
        arm.ZeroTorqueMode(enable=False)
        time.sleep(0.3)
        arm.ZeroTorqueModeWithGravity(enable=True, kd=0.5)
        time.sleep(0.5)
        arm.ZeroTorqueModeWithGravity(enable=False)
        log_pass("重复进入/退出零力矩模式无异常")
    except Exception as e:
        log_fail(f"重复进入/退出异常: {e}")

    arm.stop_control_loop()


if __name__ == "__main__":
    main()
