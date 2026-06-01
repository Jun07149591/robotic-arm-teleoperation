#!/usr/bin/env python3
"""
10 路径点循环测试 -- 实机版

使用 ELA3Interface (Direct CAN) 驱动真实电机。

前置条件:
  1. CAN 接口已激活: bash scripts/setup_can.sh can0 1000000
  2. 机械臂已上电
  3. 机械臂周围无障碍物

用法:
  python3 demo/waypoint_loop_real.py --can can0 [--loops 3]
"""

import sys
import os
import argparse
import time
import math
import signal

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from el_a3_sdk import ELA3Interface, MotorType, LogLevel
from waypoints_config import WAYPOINTS, get_waypoint_summary


def build_motor_type_map(wrist_type_str: str) -> dict:
    wrist = MotorType.RS05 if wrist_type_str == "RS05" else MotorType.EL05
    return {
        1: MotorType.RS00, 2: MotorType.RS00, 3: MotorType.RS00,
        4: wrist, 5: wrist, 6: wrist, 7: wrist,
    }


def print_motor_status(arm: ELA3Interface):
    """打印各电机详细状态"""
    feedbacks = arm.GetMotorStates()
    for mid in range(1, 7):
        fb = feedbacks.get(mid)
        if fb and fb.is_valid:
            print(f"    M{mid}: {fb.temperature:.1f}°C  "
                  f"fault={'OK' if fb.fault_code == 0 else fb.fault_code}  "
                  f"mode={fb.mode_state}")


def main():
    parser = argparse.ArgumentParser(
        description="EL-A3 路径点循环测试 (实机版)")
    parser.add_argument("--can", default="can0",
                        help="CAN 接口名 (默认: can0)")
    parser.add_argument("--motor-type", default="EL05",
                        choices=["EL05", "RS05"],
                        help="腕部电机型号 (默认: EL05)")
    parser.add_argument("--kp", type=float, default=60.0,
                        help="位置增益 Kp (默认: 60.0)")
    parser.add_argument("--kd", type=float, default=3.5,
                        help="速度增益 Kd (默认: 3.5)")
    parser.add_argument("--loops", type=int, default=0,
                        help="循环次数，0=无限循环 (默认: 0)")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="速度倍率，影响等待时间 (默认: 1.0)")
    args = parser.parse_args()

    motor_map = build_motor_type_map(args.motor_type)

    print("=" * 60)
    print("  EL-A3 路径点循环测试 -- 实机版")
    print(f"  CAN: {args.can}  |  腕部: {args.motor_type}")
    print(f"  Kp: {args.kp}  |  Kd: {args.kd}")
    print(f"  循环次数: {'无限' if args.loops == 0 else args.loops}")
    print(f"  速度倍率: {args.speed}x")
    print("  ⚠  电机将使能并运动，确保周围安全！")
    print("=" * 60)
    print(f"\n{get_waypoint_summary()}\n")

    arm = ELA3Interface(
        can_name=args.can,
        motor_type_map=motor_map,
        default_kp=args.kp,
        default_kd=args.kd,
        logger_level=LogLevel.INFO,
    )

    if not arm.ConnectPort():
        print(f"[ERROR] CAN 接口 {args.can} 连接失败")
        return

    print(f"[OK] CAN 接口 {args.can} 已连接")

    time.sleep(0.3)

    print("[INFO] 使能电机...")
    if not arm.EnableArm():
        print("[ERROR] 使能失败")
        arm.DisconnectPort()
        return

    arm.SetPositionPD(kp=args.kp, kd=args.kd)
    print(f"[OK] 电机已使能 (Kp={args.kp}, Kd={args.kd})")
    time.sleep(0.5)

    shutdown = False

    def on_signal(_sig, _frame):
        nonlocal shutdown
        shutdown = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    loop_count = 0
    try:
        while not shutdown:
            loop_count += 1
            if args.loops > 0 and loop_count > args.loops:
                break

            loops_str = f"{loop_count}" if args.loops == 0 else f"{loop_count}/{args.loops}"
            print(f"\n{'━' * 60}")
            print(f"  循环 {loops_str}  |  CAN FPS: {arm.GetCanFps():.0f}")
            print(f"{'━' * 60}")

            for i, wp in enumerate(WAYPOINTS):
                if shutdown:
                    break

                degs = [f"{p * 180.0 / math.pi:.1f}°" for p in wp.positions]
                print(f"\n  [{i}/{len(WAYPOINTS)-1}] -> {wp.name}  {degs}")

                arm.JointCtrl(*wp.positions)

                hold = wp.hold_time / args.speed
                dt = 0.02
                steps = int(hold / dt)
                for _ in range(steps):
                    if shutdown:
                        break
                    arm.JointCtrl(*wp.positions)
                    time.sleep(dt)

                js = arm.GetArmJointMsgs()
                current = js.to_list()[:6]
                errors = [abs(current[j] - wp.positions[j]) * 180.0 / math.pi
                          for j in range(6)]
                max_err = max(errors)
                current_degs = [f"{c * 180.0 / math.pi:.1f}°" for c in current]
                print(f"       到达: {current_degs}  最大误差: {max_err:.2f}°")

            print_motor_status(arm)

        print(f"\n{'=' * 60}")
        print(f"  循环完成 (共 {loop_count - 1} 轮)")
        print(f"{'=' * 60}")

    finally:
        print("\n[INFO] 回零位...")
        zero = [0.0] * 6
        for _ in range(100):
            arm.JointCtrl(*zero)
            time.sleep(0.02)
        time.sleep(1.0)

        print("[INFO] 失能电机...")
        arm.DisableArm()
        arm.DisconnectPort()
        print("[OK] 实机测试结束")


if __name__ == "__main__":
    main()
