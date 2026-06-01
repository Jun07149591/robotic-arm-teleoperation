#!/usr/bin/env python3
"""
示例: 零力矩拖动模式 (带 Pinocchio 重力补偿)

启用零力矩模式后，可手动拖动机械臂各关节，
Pinocchio 实时计算重力补偿力矩，让拖动更轻松。

两种模式:
  1. 基础模式: ZeroTorqueMode (无重力补偿)
  2. 增强模式: ZeroTorqueModeWithGravity (Pinocchio 重力补偿)

使用前确保:
  1. CAN 接口已激活
  2. 机械臂已上电
"""

import os
import time
import math
import sys
_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, _PROJECT_ROOT)

from el_a3_sdk import ELA3Interface, LogLevel


def main():
    use_gravity_comp = "--gravity" in sys.argv or "-g" in sys.argv

    can_name = "can0"
    for i, arg in enumerate(sys.argv):
        if arg == "--can" and i + 1 < len(sys.argv):
            can_name = sys.argv[i + 1]

    arm = ELA3Interface(
        can_name=can_name,
        logger_level=LogLevel.INFO,
        inertia_config_path=os.path.join(_PROJECT_ROOT, 'resources', 'config', 'inertia_params.yaml'),
        per_joint_kd_min={4: 0.005, 5: 0.005, 6: 0.005, 7: 0.02},
        per_joint_kd_max={4: 0.10, 5: 0.05, 6: 0.05, 7: 0.10},
    )

    if not arm.ConnectPort():
        print("连接失败")
        return

    arm.EnableArm()
    time.sleep(0.3)

    if use_gravity_comp:
        print("\n=== 零力矩拖动模式 (Pinocchio 重力补偿) ===")
        print("机械臂可手动拖动，重力由 Pinocchio 实时补偿\n")
        arm.ZeroTorqueModeWithGravity(enable=True, kd=[0.05, 0.05, 0.05, 0.0125, 0.0125, 0.0125, 0.05], update_rate=100.0)
    else:
        print("\n=== 零力矩拖动模式 (基础) ===")
        print("机械臂可手动拖动，按 Ctrl+C 退出")
        print("提示: 使用 --gravity 或 -g 参数启用重力补偿\n")
        arm.ZeroTorqueMode(enable=True, kd=0.05)

    try:
        while True:
            joints = arm.GetArmJointMsgs()
            deg = [j * 180.0 / math.pi for j in joints.to_list()]
            efforts = arm.GetArmJointEfforts()
            torques = efforts.to_list()

            gravity_str = ""
            if use_gravity_comp:
                grav = arm.ComputeGravityTorques()
                gravity_str = f"  |  重力: {' '.join(f'{g:6.2f}' for g in grav)} Nm"

            print(f"角度: {' '.join(f'{d:7.1f}°' for d in deg)}  |  "
                  f"力矩: {' '.join(f'{t:6.2f}' for t in torques)} Nm"
                  f"{gravity_str}", end="\r")
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n\n退出零力矩模式")
    finally:
        if use_gravity_comp:
            arm.ZeroTorqueModeWithGravity(enable=False)
        else:
            arm.ZeroTorqueMode(enable=False)
        time.sleep(0.1)
        arm.DisableArm()
        arm.DisconnectPort()


if __name__ == "__main__":
    main()
