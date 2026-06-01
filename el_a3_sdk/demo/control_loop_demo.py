#!/usr/bin/env python3
"""
示例: 后台控制循环（200Hz）

展示使用后台高频控制循环进行运动控制：
  - start_control_loop() 启动 200Hz 后台控制循环
  - JointCtrl() 设置目标位置，由控制循环以 EMA 平滑 + 速度前馈 + 重力补偿 发送
  - MoveJ() 异步执行 S-curve 轨迹
  - ZeroTorqueModeWithGravity() 进入示教模式

使用前确保:
  1. CAN 接口已激活 (sudo ip link set can0 up type can bitrate 1000000)
  2. 机械臂已上电
  3. 机械臂周围无障碍物
"""

import time
import math
import signal
from el_a3_sdk import ELA3Interface, LogLevel


def deg2rad(d: float) -> float:
    return d * math.pi / 180.0


def main():
    arm = ELA3Interface(
        can_name="can0",
        logger_level=LogLevel.INFO,
        gravity_feedforward_ratio=1.0,
    )

    if not arm.ConnectPort():
        print("连接失败")
        return

    arm.EnableArm()
    time.sleep(0.5)

    # 启动 200Hz 后台控制循环
    arm.start_control_loop(rate_hz=200.0)
    print("控制循环已启动\n")

    shutdown = False

    def on_signal(_sig, _frame):
        nonlocal shutdown
        shutdown = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        # --- 1. S-curve 轨迹运动（阻塞模式）---
        print("=== 1. MoveJ 到初始位置 ===")
        arm.MoveJ([0.0, deg2rad(30), deg2rad(-30), 0.0, 0.0, 0.0],
                  duration=2.0, block=True)
        print(f"到达位置: {arm.GetArmJointMsgs().to_list()[:6]}\n")

        if shutdown:
            return

        # --- 2. 连续轨迹（非阻塞 + wait）---
        print("=== 2. MoveJ 非阻塞模式 ===")
        arm.MoveJ([deg2rad(-20), deg2rad(60), deg2rad(-45), 0.0, 0.0, 0.0],
                  duration=2.0, block=False)
        print("轨迹已提交，等待完成...")
        while arm.is_moving() and not shutdown:
            js = arm.GetArmJointMsgs()
            degs = [f"{v * 180 / math.pi:.1f}" for v in js.to_list()[:6]]
            print(f"  当前: {degs}")
            time.sleep(0.5)
        print(f"到达位置: {arm.GetArmJointMsgs().to_list()[:6]}\n")

        if shutdown:
            return

        # --- 3. 实时 JointCtrl 逐点控制（由控制循环平滑）---
        print("=== 3. 实时 JointCtrl 正弦波 ===")
        t0 = time.time()
        while time.time() - t0 < 4.0 and not shutdown:
            t = time.time() - t0
            j1 = deg2rad(30) * math.sin(2 * math.pi * 0.25 * t)
            j2 = deg2rad(45) + deg2rad(15) * math.sin(2 * math.pi * 0.25 * t)
            arm.JointCtrl(j1, j2, deg2rad(-30), 0.0, 0.0, 0.0)
            time.sleep(0.02)
        print()

        if shutdown:
            return

        # --- 4. 回零 ---
        print("=== 4. 回零位 ===")
        arm.MoveJ([0.0] * 6, duration=2.0, block=True)
        print("已回零\n")

        if shutdown:
            return

        # --- 5. 零力矩示教模式 ---
        print("=== 5. 零力矩示教模式（自适应 Kd + 重力补偿）===")
        print("可以手动拖动机械臂，按 Ctrl+C 退出")
        arm.ZeroTorqueModeWithGravity(True, kd=0.5)
        while not shutdown:
            js = arm.GetArmJointMsgs()
            degs = [f"{v * 180 / math.pi:.1f}" for v in js.to_list()[:6]]
            print(f"  当前: {degs}", end="\r")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        print("\n正在清理...")
        arm.ZeroTorqueModeWithGravity(False)
        arm.stop_control_loop()
        arm.DisableArm()
        arm.DisconnectPort()
        print("完成")


if __name__ == "__main__":
    main()
