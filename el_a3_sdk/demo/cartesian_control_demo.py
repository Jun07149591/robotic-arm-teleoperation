#!/usr/bin/env python3
"""
笛卡尔位姿控制示例

演示使用 SDK 的 EndPoseCtrl 和 MoveL 功能。
需要 Pinocchio：pip install pin
"""

import os
import time
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from el_a3_sdk import ELA3Interface, ArmEndPose


def main():
    arm = ELA3Interface(can_name="can0", default_kp=80.0, default_kd=4.0)
    arm.ConnectPort()
    arm.EnableArm()
    time.sleep(0.5)

    # 获取当前末端位姿
    pose = arm.GetArmEndPoseMsgs()
    print(f"当前末端位姿: x={pose.x:.4f}, y={pose.y:.4f}, z={pose.z:.4f}")
    print(f"              rx={pose.rx:.4f}, ry={pose.ry:.4f}, rz={pose.rz:.4f}")

    # EndPoseCtrl: 移动到指定笛卡尔位姿
    print("\n--- EndPoseCtrl 测试 ---")
    target_x = pose.x + 0.05
    target_z = pose.z + 0.03
    print(f"目标: x={target_x:.4f}, z={target_z:.4f}")
    success = arm.EndPoseCtrl(target_x, pose.y, target_z, pose.rx, pose.ry, pose.rz, duration=3.0)
    print(f"EndPoseCtrl 结果: {'成功' if success else '失败'}")
    time.sleep(1.0)

    # MoveL: 直线运动回到原始位姿
    print("\n--- MoveL 测试 ---")
    print(f"直线运动回到: x={pose.x:.4f}, z={pose.z:.4f}")
    success = arm.MoveL(pose, duration=3.0)
    print(f"MoveL 结果: {'成功' if success else '失败'}")
    time.sleep(1.0)

    # 获取 Jacobian
    print("\n--- Jacobian ---")
    J = arm.GetJacobian()
    print(f"Jacobian shape: {J.shape}")
    print(f"Jacobian:\n{J}")

    arm.DisableArm()
    arm.DisconnectPort()
    print("\n完成")


if __name__ == "__main__":
    main()
