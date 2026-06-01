#!/usr/bin/env python3
"""
示例: 读取关节角度

使用前确保:
  1. CAN 接口已激活: bash scripts/setup_can.sh can0 1000000
  2. 机械臂已上电
"""

import time
import math
from el_a3_sdk import ELA3Interface, LogLevel


def main():
    arm = ELA3Interface(can_name="can0", logger_level=LogLevel.INFO)

    if not arm.ConnectPort():
        print("连接失败，请检查 CAN 接口")
        return

    # 使能电机（运控模式 Kp=0 软启动）
    arm.EnableArm()
    time.sleep(0.5)

    print("\n=== 关节角度反馈 (按 Ctrl+C 停止) ===\n")

    try:
        while True:
            joints = arm.GetArmJointMsgs()
            deg = [j * 180.0 / math.pi for j in joints.to_list()]

            print(f"J1={deg[0]:7.2f}° J2={deg[1]:7.2f}° J3={deg[2]:7.2f}° "
                  f"J4={deg[3]:7.2f}° J5={deg[4]:7.2f}° J6={deg[5]:7.2f}°  "
                  f"FPS={arm.GetCanFps():.0f}", end="\r")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n\n停止")
    finally:
        arm.DisableArm()
        arm.DisconnectPort()


if __name__ == "__main__":
    main()
