#!/usr/bin/env python3
"""
基础测试 6: 多臂管理测试

覆盖 README API:
  - ArmManager() 创建
  - register_can_arm() 注册
  - get_arm() / has_arm() / arm_names 查询
  - disconnect_all() 断开

注意: 此测试仅验证 ArmManager 的管理逻辑，
      使用单个 CAN 接口进行验证。

前置条件:
  - CAN 接口已配置
  - 机械臂已上电

用法:
  python3 scripts/tests/test_basic_multi_arm.py [--can can0]
"""
import sys
import os
import argparse
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from el_a3_sdk import ArmManager, LogLevel

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
    parser = argparse.ArgumentParser(description="基础测试 6: 多臂管理")
    parser.add_argument("--can", default="can0", help="CAN 接口名")
    args = parser.parse_args()

    print("=" * 50)
    print(f" 基础测试 6: 多臂管理 ({args.can})")
    print("=" * 50)

    # 先重置 Singleton
    ArmManager.reset()

    # ---- 1. 创建 ArmManager ----
    print("\n  --- ArmManager 创建 ---")
    try:
        mgr = ArmManager()
        log_pass("ArmManager() 创建成功")
    except Exception as e:
        log_fail(f"ArmManager() 异常: {e}")
        sys.exit(1)

    # ---- 2. Singleton 验证 ----
    print("\n  --- Singleton 验证 ---")
    try:
        mgr2 = ArmManager()
        if mgr is mgr2:
            log_pass("Singleton: 两次创建返回同一实例")
        else:
            log_fail("Singleton: 两次创建返回不同实例")
    except Exception as e:
        log_fail(f"Singleton 验证异常: {e}")

    # ---- 3. register_can_arm ----
    print("\n  --- register_can_arm ---")
    try:
        arm = mgr.register_can_arm("test_arm", can_name=args.can)
        if arm is not None:
            log_pass(f"register_can_arm('test_arm', '{args.can}') 返回实例")
        else:
            log_fail("register_can_arm 返回 None")
    except Exception as e:
        log_fail(f"register_can_arm 异常: {e}")
        ArmManager.reset()
        sys.exit(1)

    # ---- 4. 重复注册 ----
    print("\n  --- 重复注册 ---")
    try:
        arm2 = mgr.register_can_arm("test_arm", can_name=args.can)
        if arm2 is arm:
            log_pass("重复注册返回同一实例")
        else:
            log_fail("重复注册返回不同实例")
    except Exception as e:
        log_fail(f"重复注册异常: {e}")

    # ---- 5. has_arm / get_arm / arm_names ----
    print("\n  --- 查询接口 ---")
    try:
        if mgr.has_arm("test_arm"):
            log_pass("has_arm('test_arm') 返回 True")
        else:
            log_fail("has_arm('test_arm') 返回 False")

        if not mgr.has_arm("nonexistent"):
            log_pass("has_arm('nonexistent') 返回 False")
        else:
            log_fail("has_arm('nonexistent') 返回 True")
    except Exception as e:
        log_fail(f"has_arm 异常: {e}")

    try:
        fetched = mgr.get_arm("test_arm")
        if fetched is arm:
            log_pass("get_arm('test_arm') 返回正确实例")
        else:
            log_fail("get_arm 返回错误实例")
    except Exception as e:
        log_fail(f"get_arm 异常: {e}")

    try:
        names = mgr.arm_names
        if "test_arm" in names:
            log_pass(f"arm_names 包含 'test_arm': {names}")
        else:
            log_fail(f"arm_names 不含 'test_arm': {names}")
    except Exception as e:
        log_fail(f"arm_names 异常: {e}")

    try:
        bracket_arm = mgr["test_arm"]
        if bracket_arm is arm:
            log_pass("mgr['test_arm'] 下标访问正确")
        else:
            log_fail("mgr['test_arm'] 返回错误实例")
    except Exception as e:
        log_fail(f"下标访问异常: {e}")

    try:
        if "test_arm" in mgr:
            log_pass("'test_arm' in mgr 返回 True")
        else:
            log_fail("'test_arm' in mgr 返回 False")
    except Exception as e:
        log_fail(f"in 运算异常: {e}")

    # ---- 6. ConnectPort + EnableArm + GetArmJointMsgs ----
    print("\n  --- 通过 ArmManager 实例控制 ---")
    try:
        result = arm.ConnectPort()
        if result:
            log_pass("通过管理器实例 ConnectPort 成功")
        else:
            log_fail("通过管理器实例 ConnectPort 失败")
    except Exception as e:
        log_fail(f"ConnectPort 异常: {e}")
        ArmManager.reset()
        sys.exit(1)

    time.sleep(0.3)

    try:
        arm.EnableArm()
        log_pass("通过管理器实例 EnableArm 成功")
    except Exception as e:
        log_fail(f"EnableArm 异常: {e}")

    time.sleep(0.5)

    try:
        joints = arm.GetArmJointMsgs()
        jlist = joints.to_list(include_gripper=True)
        if len(jlist) == 7:
            log_pass(f"关节读取正常: {[f'{j:.3f}' for j in jlist]}")
        else:
            log_fail(f"关节读取异常: 长度 {len(jlist)}")
    except Exception as e:
        log_fail(f"GetArmJointMsgs 异常: {e}")

    try:
        arm.DisableArm()
        log_pass("DisableArm 成功")
    except Exception as e:
        log_fail(f"DisableArm 异常: {e}")

    # ---- 7. disconnect_all ----
    print("\n  --- disconnect_all ---")
    try:
        mgr.disconnect_all()
        log_pass("disconnect_all 成功")
    except Exception as e:
        log_fail(f"disconnect_all 异常: {e}")

    # ---- 8. len 验证 ----
    try:
        if len(mgr) == 0:
            log_pass("disconnect_all 后 len(mgr) == 0")
        else:
            log_fail(f"disconnect_all 后 len(mgr) = {len(mgr)}")
    except Exception as e:
        log_fail(f"len 异常: {e}")

    # 清理 Singleton
    ArmManager.reset()

    print(f"\n{'=' * 50}")
    print(f" 结果: {PASS} passed, {FAIL} failed")
    print(f"{'=' * 50}")
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
