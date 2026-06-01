#!/usr/bin/env python3
"""
EL-A3 MoveIt2 点位测试程序

通过 /move_group action 接口测试 MoveIt2 规划与执行：
  1. 预定义姿态 (zero → home → ready → zero)
  2. 自定义关节角度路径点
  3. 回零位

前置: ros2 launch el_a3_moveit_config demo.launch.py
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
)

JOINT_NAMES = [
    "L1_joint", "L2_joint", "L3_joint",
    "L4_joint", "L5_joint", "L6_joint",
]

NAMED_TARGETS = {
    "zero": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "home": [0.0, 0.785, -0.785, 0.0, 0.0, 0.0],
    "ready": [0.0, 0.785, -1.57, 0.0, 0.785, 0.0],
}

WAYPOINTS = [
    ("前伸抬臂", [0.52, 0.79, -0.52, 0.0, 0.0, 0.0]),
    ("左侧展开", [-0.52, 1.05, -1.05, 0.52, 0.0, 0.0]),
    ("高位直臂", [0.0, 1.57, -1.57, 0.0, 0.79, 0.0]),
    ("右前方",   [1.05, 0.52, -0.79, -0.52, 0.0, 0.52]),
    ("左前方",   [-1.05, 0.52, -0.79, 0.52, 0.0, -0.52]),
    ("折叠",     [0.0, 2.09, -2.09, 0.0, -0.52, 0.0]),
]


class MoveItWaypointTest(Node):
    def __init__(self):
        super().__init__("moveit_waypoint_test")
        cb_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            self, MoveGroup, "/move_action", callback_group=cb_group,
        )
        self.get_logger().info("等待 /move_action action server …")
        if not self._action_client.wait_for_server(timeout_sec=60.0):
            self.get_logger().error("/move_action 不可用，请先启动 MoveIt demo")
            raise RuntimeError("MoveGroup action server not available")
        self.get_logger().info("/move_action 已连接")

    # ------------------------------------------------------------------
    def _build_goal(self, positions: list,
                    vel_scale: float = 0.3,
                    acc_scale: float = 0.3,
                    planning_time: float = 5.0) -> MoveGroup.Goal:
        goal = MoveGroup.Goal()

        req = MotionPlanRequest()
        req.group_name = "arm"
        req.num_planning_attempts = 10
        req.allowed_planning_time = planning_time
        req.max_velocity_scaling_factor = vel_scale
        req.max_acceleration_scaling_factor = acc_scale

        constraints = Constraints()
        for jname, val in zip(JOINT_NAMES, positions):
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)

        goal.request = req
        opts = PlanningOptions()
        opts.plan_only = False
        opts.replan = True
        opts.replan_attempts = 3
        goal.planning_options = opts
        return goal

    # ------------------------------------------------------------------
    def move_to(self, label: str, positions: list,
                vel_scale: float = 0.3, acc_scale: float = 0.3) -> bool:
        degs = [f"{p * 180.0 / math.pi:.1f}°" for p in positions]
        self.get_logger().info(f"→ {label}  {degs}")

        goal = self._build_goal(positions, vel_scale, acc_scale)

        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"  ✗ {label} — 目标被拒绝")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        result = result_future.result()

        if result is not None and result.result.error_code.val == 1:
            self.get_logger().info(f"  ✓ {label} — 成功")
            return True

        code = result.result.error_code.val if result else "超时"
        self.get_logger().error(f"  ✗ {label} — 失败 (error_code={code})")
        return False

    def move_to_named(self, name: str) -> bool:
        if name not in NAMED_TARGETS:
            self.get_logger().error(f"未知姿态: {name}")
            return False
        return self.move_to(name, NAMED_TARGETS[name])


# ======================================================================
def main():
    rclpy.init()
    try:
        node = MoveItWaypointTest()
    except RuntimeError:
        rclpy.shutdown()
        sys.exit(1)

    results: list[tuple[str, bool]] = []

    print("\n" + "=" * 60)
    print("  EL-A3 MoveIt2 点位测试")
    print("=" * 60)

    # --- Phase 1: named targets ---
    print("\n--- 阶段 1: 预定义姿态 ---")
    for name in ["zero", "home", "ready", "zero"]:
        ok = node.move_to_named(name)
        results.append((f"[named] {name}", ok))
        time.sleep(0.5)

    # --- Phase 2: custom waypoints ---
    print("\n--- 阶段 2: 自定义路径点 ---")
    for wp_name, positions in WAYPOINTS:
        ok = node.move_to(wp_name, positions)
        results.append((f"[waypoint] {wp_name}", ok))
        time.sleep(0.5)

    # --- Return to zero ---
    print("\n--- 回零位 ---")
    ok = node.move_to_named("zero")
    results.append(("[return] zero", ok))

    # --- Summary ---
    passed = sum(1 for _, ok in results if ok)
    total = len(results)
    print("\n" + "=" * 60)
    print("  测试结果汇总")
    print("=" * 60)
    for label, ok in results:
        status = "✓ 通过" if ok else "✗ 失败"
        print(f"  {status}  {label}")
    print(f"\n  总计: {passed}/{total} 通过")
    print("=" * 60 + "\n")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    main()
