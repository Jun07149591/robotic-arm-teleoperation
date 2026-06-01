#!/usr/bin/env python3
"""
基础测试 4: 笛卡尔控制 + 动力学接口测试

覆盖 README API:
  - GetArmEndPoseMsgs
  - EndPoseCtrl
  - MoveL
  - GetJacobian
  - GetMassMatrix
  - ComputeGravityTorques
  - InverseDynamics
  - ForwardDynamics

前置条件:
  - CAN 接口已配置
  - 机械臂已上电
  - 需安装 Pinocchio: pip install pin

用法:
  python3 scripts/tests/test_basic_cartesian.py [--can can0]
"""
import sys
import os
import argparse
import math
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

try:
    import numpy as np
except ImportError:
    print("  [SKIP] 未安装 numpy，跳过笛卡尔测试")
    sys.exit(0)

from el_a3_sdk import ELA3Interface, LogLevel
from el_a3_sdk.data_types import ArmEndPose

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


def main():
    parser = argparse.ArgumentParser(description="基础测试 4: 笛卡尔控制 + 动力学")
    parser.add_argument("--can", default="can0", help="CAN 接口名")
    args = parser.parse_args()

    print("=" * 50)
    print(f" 基础测试 4: 笛卡尔控制 + 动力学 ({args.can})")
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

    # 检查 Pinocchio 是否可用
    pose = arm.GetArmEndPoseMsgs()
    has_pinocchio = any(abs(getattr(pose, a, 0.0)) > 1e-9
                        for a in ['x', 'y', 'z'])
    if not has_pinocchio:
        print("\n  未检测到有效 FK 数据 (需 Pinocchio)")
        print("  安装: pip install pin")
        log_skip("Pinocchio 不可用，跳过全部笛卡尔测试")
        arm.DisableArm()
        arm.DisconnectPort()
        print(f"\n{'=' * 50}")
        print(f" 结果: {PASS} passed, {FAIL} failed, {SKIP} skipped")
        print(f"{'=' * 50}")
        sys.exit(0)

    try:
        _run_cartesian_tests(arm)
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
        print(f" 结果: {PASS} passed, {FAIL} failed, {SKIP} skipped")
        print(f"{'=' * 50}")
        sys.exit(0 if FAIL == 0 else 1)


def _run_cartesian_tests(arm: ELA3Interface):
    arm.start_control_loop(rate_hz=200.0)
    time.sleep(0.3)

    # 先回零位
    arm.MoveJ([0.0] * 6, duration=2.0, block=True)
    time.sleep(0.5)

    # ---- 1. GetArmEndPoseMsgs ----
    print("\n  --- GetArmEndPoseMsgs ---")
    try:
        pose = arm.GetArmEndPoseMsgs()
        log_pass(f"末端位姿: x={pose.x:.4f} y={pose.y:.4f} z={pose.z:.4f} "
                 f"rx={pose.rx:.4f} ry={pose.ry:.4f} rz={pose.rz:.4f}")
        base_pose = pose
    except Exception as e:
        log_fail(f"GetArmEndPoseMsgs 异常: {e}")
        return

    # ---- 2. EndPoseCtrl ----
    print("\n  --- EndPoseCtrl ---")
    try:
        target_x = base_pose.x + 0.03
        result = arm.EndPoseCtrl(
            target_x, base_pose.y, base_pose.z,
            base_pose.rx, base_pose.ry, base_pose.rz,
            duration=3.0, block=True,
        )
        if result:
            new_pose = arm.GetArmEndPoseMsgs()
            dx = abs(new_pose.x - target_x)
            if dx < 0.01:
                log_pass(f"EndPoseCtrl 到达精度: dx={dx:.4f} m")
            else:
                log_fail(f"EndPoseCtrl 精度不足: dx={dx:.4f} m")
        else:
            log_fail("EndPoseCtrl 返回 False")
    except Exception as e:
        log_fail(f"EndPoseCtrl 异常: {e}")

    time.sleep(0.5)

    # ---- 3. MoveL ----
    print("\n  --- MoveL ---")
    try:
        result = arm.MoveL(base_pose, duration=3.0, block=True)
        if result:
            after = arm.GetArmEndPoseMsgs()
            dist = math.sqrt(
                (after.x - base_pose.x) ** 2
                + (after.y - base_pose.y) ** 2
                + (after.z - base_pose.z) ** 2
            )
            if dist < 0.01:
                log_pass(f"MoveL 回原位精度: {dist:.4f} m")
            else:
                log_fail(f"MoveL 精度不足: {dist:.4f} m")
        else:
            log_fail("MoveL 返回 False")
    except Exception as e:
        log_fail(f"MoveL 异常: {e}")

    time.sleep(0.5)

    # ---- 4. GetJacobian ----
    print("\n  --- GetJacobian ---")
    try:
        J = arm.GetJacobian()
        if J.shape[0] == 6 and J.shape[1] >= 6:
            log_pass(f"Jacobian shape: {J.shape}")
        else:
            log_fail(f"Jacobian shape 异常: {J.shape} (期望 6xN)")

        if not np.allclose(J, 0.0):
            log_pass("Jacobian 包含非零元素")
        else:
            log_fail("Jacobian 全零")
    except Exception as e:
        log_fail(f"GetJacobian 异常: {e}")

    # ---- 5. GetMassMatrix ----
    print("\n  --- GetMassMatrix ---")
    try:
        M = arm.GetMassMatrix()
        n = M.shape[0]
        if M.shape == (n, n):
            log_pass(f"质量矩阵 shape: {M.shape}")
        else:
            log_fail(f"质量矩阵 shape 异常: {M.shape}")

        sym_err = np.max(np.abs(M - M.T))
        if sym_err < 1e-6:
            log_pass(f"质量矩阵对称性: 最大误差 {sym_err:.2e}")
        else:
            log_fail(f"质量矩阵不对称: 最大误差 {sym_err:.2e}")

        eigvals = np.linalg.eigvalsh(M)
        if np.all(eigvals > 0):
            log_pass(f"质量矩阵正定: 最小特征值 {eigvals.min():.6f}")
        else:
            log_fail(f"质量矩阵非正定: 最小特征值 {eigvals.min():.6f}")
    except Exception as e:
        log_fail(f"GetMassMatrix 异常: {e}")

    # ---- 6. ComputeGravityTorques ----
    print("\n  --- ComputeGravityTorques ---")
    try:
        grav = arm.ComputeGravityTorques()
        if len(grav) >= 6:
            log_pass(f"重力力矩 ({len(grav)} 个值): "
                     f"{[f'{g:.3f}' for g in grav]}")
        else:
            log_fail(f"重力力矩只有 {len(grav)} 个值")

        if not all(g == 0.0 for g in grav):
            log_pass("重力力矩包含非零分量")
        else:
            log_fail("重力力矩全零 (不合理)")
    except Exception as e:
        log_fail(f"ComputeGravityTorques 异常: {e}")

    # ---- 7. InverseDynamics ----
    print("\n  --- InverseDynamics ---")
    try:
        q = arm.GetArmJointMsgs().to_list()
        n = len(q)
        tau = arm.InverseDynamics(q, [0.0] * n, [0.0] * n)
        if len(tau) >= 6:
            log_pass(f"InverseDynamics (静态): {[f'{t:.3f}' for t in tau]}")
        else:
            log_fail(f"InverseDynamics 返回长度异常: {len(tau)}")

        grav = arm.ComputeGravityTorques(q)
        if len(tau) == len(grav):
            max_diff = max(abs(a - b) for a, b in zip(tau, grav))
            if max_diff < 0.01:
                log_pass(f"静态 ID ≈ 重力力矩: 最大差 {max_diff:.4f} Nm")
            else:
                log_fail(f"静态 ID ≠ 重力力矩: 最大差 {max_diff:.4f} Nm")
    except Exception as e:
        log_fail(f"InverseDynamics 异常: {e}")

    # ---- 8. ForwardDynamics ----
    print("\n  --- ForwardDynamics ---")
    try:
        q = arm.GetArmJointMsgs().to_list()
        n = len(q)
        grav = arm.ComputeGravityTorques(q)
        accel = arm.ForwardDynamics(q, [0.0] * n, grav)
        if len(accel) >= 6:
            log_pass(f"ForwardDynamics 返回: {[f'{a:.4f}' for a in accel]}")
        else:
            log_fail(f"ForwardDynamics 返回长度异常: {len(accel)}")

        max_accel = max(abs(a) for a in accel)
        if max_accel < 1.0:
            log_pass(f"静态下加速度接近零: max={max_accel:.4f} rad/s²")
        else:
            log_fail(f"静态下加速度过大: max={max_accel:.4f} rad/s²")
    except Exception as e:
        log_fail(f"ForwardDynamics 异常: {e}")

    # 回零
    arm.MoveJ([0.0] * 6, duration=2.0, block=True)
    arm.stop_control_loop()


if __name__ == "__main__":
    main()
