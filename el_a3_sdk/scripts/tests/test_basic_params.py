#!/usr/bin/env python3
"""
基础测试 2: 参数设置测试

覆盖 README API:
  - SetPositionPD
  - SetSmoothingAlpha
  - SetGravityFeedforwardRatio
  - SetJointLimitEnabled

纯 API 调用验证，不做实际运动。

前置条件:
  - CAN 接口已配置
  - 机械臂已上电

用法:
  python3 scripts/tests/test_basic_params.py [--can can0]
"""
import sys
import os
import argparse
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


def main():
    parser = argparse.ArgumentParser(description="基础测试 2: 参数设置")
    parser.add_argument("--can", default="can0", help="CAN 接口名")
    args = parser.parse_args()

    print("=" * 50)
    print(f" 基础测试 2: 参数设置 ({args.can})")
    print("=" * 50)

    arm = ELA3Interface(can_name=args.can, logger_level=LogLevel.INFO)

    try:
        result = arm.ConnectPort()
        if not result:
            log_fail("ConnectPort 失败")
            sys.exit(1)
    except Exception as e:
        log_fail(f"ConnectPort 异常: {e}")
        sys.exit(1)

    time.sleep(0.3)
    arm.EnableArm()
    time.sleep(0.5)

    # ---- 1. SetPositionPD ----
    print("\n  --- SetPositionPD ---")
    try:
        arm.SetPositionPD(kp=60.0, kd=3.5)
        log_pass("SetPositionPD(kp=60.0, kd=3.5) 无异常")
    except Exception as e:
        log_fail(f"SetPositionPD(60, 3.5) 异常: {e}")

    try:
        arm.SetPositionPD(kp=80.0, kd=4.0)
        log_pass("SetPositionPD(kp=80.0, kd=4.0) 恢复默认值")
    except Exception as e:
        log_fail(f"SetPositionPD(80, 4) 异常: {e}")

    # 边界值: kp=0, kd=0
    try:
        arm.SetPositionPD(kp=0.0, kd=0.0)
        log_pass("SetPositionPD(kp=0, kd=0) 边界值无异常")
    except Exception as e:
        log_fail(f"SetPositionPD(0, 0) 异常: {e}")

    # 边界值: 极大值 (应被 clamp)
    try:
        arm.SetPositionPD(kp=9999.0, kd=9999.0)
        log_pass("SetPositionPD(kp=9999, kd=9999) 极大值无异常 (clamp)")
    except Exception as e:
        log_fail(f"SetPositionPD(9999, 9999) 异常: {e}")

    arm.SetPositionPD(kp=80.0, kd=4.0)

    # ---- 2. SetSmoothingAlpha ----
    print("\n  --- SetSmoothingAlpha ---")
    try:
        arm.SetSmoothingAlpha(0.5)
        log_pass("SetSmoothingAlpha(0.5) 无异常")
    except Exception as e:
        log_fail(f"SetSmoothingAlpha(0.5) 异常: {e}")

    try:
        arm.SetSmoothingAlpha(0.8)
        log_pass("SetSmoothingAlpha(0.8) 恢复默认值")
    except Exception as e:
        log_fail(f"SetSmoothingAlpha(0.8) 异常: {e}")

    try:
        arm.SetSmoothingAlpha(0.01)
        log_pass("SetSmoothingAlpha(0.01) 最小值无异常")
    except Exception as e:
        log_fail(f"SetSmoothingAlpha(0.01) 异常: {e}")

    try:
        arm.SetSmoothingAlpha(1.0)
        log_pass("SetSmoothingAlpha(1.0) 最大值无异常")
    except Exception as e:
        log_fail(f"SetSmoothingAlpha(1.0) 异常: {e}")

    arm.SetSmoothingAlpha(0.8)

    # ---- 3. SetGravityFeedforwardRatio ----
    print("\n  --- SetGravityFeedforwardRatio ---")
    try:
        arm.SetGravityFeedforwardRatio(0.5)
        log_pass("SetGravityFeedforwardRatio(0.5) 无异常")
    except Exception as e:
        log_fail(f"SetGravityFeedforwardRatio(0.5) 异常: {e}")

    try:
        arm.SetGravityFeedforwardRatio(1.0)
        log_pass("SetGravityFeedforwardRatio(1.0) 恢复默认值")
    except Exception as e:
        log_fail(f"SetGravityFeedforwardRatio(1.0) 异常: {e}")

    try:
        arm.SetGravityFeedforwardRatio(0.0)
        log_pass("SetGravityFeedforwardRatio(0.0) 最小值无异常")
    except Exception as e:
        log_fail(f"SetGravityFeedforwardRatio(0.0) 异常: {e}")

    # 越界测试 (应被 clamp 到 [0, 1])
    try:
        arm.SetGravityFeedforwardRatio(5.0)
        log_pass("SetGravityFeedforwardRatio(5.0) 越界值无异常 (clamp)")
    except Exception as e:
        log_fail(f"SetGravityFeedforwardRatio(5.0) 异常: {e}")

    arm.SetGravityFeedforwardRatio(1.0)

    # ---- 4. SetJointLimitEnabled ----
    print("\n  --- SetJointLimitEnabled ---")
    try:
        arm.SetJointLimitEnabled(True)
        log_pass("SetJointLimitEnabled(True) 无异常")
    except Exception as e:
        log_fail(f"SetJointLimitEnabled(True) 异常: {e}")

    try:
        arm.SetJointLimitEnabled(False)
        log_pass("SetJointLimitEnabled(False) 无异常")
    except Exception as e:
        log_fail(f"SetJointLimitEnabled(False) 异常: {e}")

    try:
        arm.SetJointLimitEnabled(True)
        log_pass("SetJointLimitEnabled(True) 再次开启无异常")
    except Exception as e:
        log_fail(f"SetJointLimitEnabled(True) 异常: {e}")

    # ---- 清理 ----
    print("\n  --- 清理 ---")
    try:
        arm.DisableArm()
        log_pass("DisableArm 成功")
    except Exception as e:
        log_fail(f"DisableArm 异常: {e}")

    try:
        arm.DisconnectPort()
        log_pass("DisconnectPort 成功")
    except Exception as e:
        log_fail(f"DisconnectPort 异常: {e}")

    print(f"\n{'=' * 50}")
    print(f" 结果: {PASS} passed, {FAIL} failed")
    print(f"{'=' * 50}")
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
