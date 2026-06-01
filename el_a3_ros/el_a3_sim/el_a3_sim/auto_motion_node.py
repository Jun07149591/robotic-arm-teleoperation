#!/usr/bin/env python3
"""Send repeatable demo trajectories to the EL-A3 mock ros2_control system."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINTS = [f"L{i}_joint" for i in range(1, 7)]
GRIPPER_JOINT = "L7_joint"


@dataclass(frozen=True)
class DemoPose:
    name: str
    arm: List[float]
    gripper: Optional[float]
    duration_sec: float
    dwell_sec: float = 0.75


DEMO_SEQUENCE = [
    DemoPose(
        name="ready",
        arm=[0.0, 0.75, -0.85, 0.0, 0.35, 0.0],
        gripper=0.20,
        duration_sec=3.0,
    ),
    DemoPose(
        name="left_reach",
        arm=[0.45, 1.05, -1.25, -0.35, 0.55, 0.30],
        gripper=0.55,
        duration_sec=4.0,
    ),
    DemoPose(
        name="right_reach",
        arm=[-0.45, 0.85, -1.05, 0.35, -0.35, -0.30],
        gripper=0.05,
        duration_sec=4.0,
    ),
    DemoPose(
        name="inspect",
        arm=[0.20, 1.30, -1.60, 0.60, 0.80, -0.45],
        gripper=0.35,
        duration_sec=4.0,
    ),
    DemoPose(
        name="home",
        arm=[0.0, 0.75, -0.85, 0.0, 0.35, 0.0],
        gripper=0.20,
        duration_sec=3.0,
    ),
]


class AutoMotionNode(Node):
    """Loops through a small, joint-limit-safe trajectory sequence."""

    def __init__(self) -> None:
        super().__init__("el_a3_auto_motion")

        self.declare_parameter("loop", True)
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("start_delay_sec", 6.0)
        self.declare_parameter("dwell_scale", 1.0)
        self.declare_parameter("home_on_exit", True)

        self._arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
        )
        self._gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
        )
        self._stop_requested = False

    def run(self) -> None:
        start_delay = max(0.0, self._param_float("start_delay_sec"))
        if start_delay:
            self.get_logger().info(f"Waiting {start_delay:.1f}s for controllers...")
            self._sleep_with_spin(start_delay)

        if not self._wait_for_servers():
            self.get_logger().error("Trajectory action servers are not available.")
            return

        loop = self.get_parameter("loop").get_parameter_value().bool_value
        self.get_logger().info(
            "Starting EL-A3 mock motion demo "
            f"({'loop' if loop else 'single pass'})."
        )

        try:
            while rclpy.ok() and not self._stop_requested:
                self._run_sequence(DEMO_SEQUENCE)
                if not loop:
                    break
        finally:
            if self.get_parameter("home_on_exit").get_parameter_value().bool_value:
                self.get_logger().info("Returning to ready pose before exit.")
                self._send_pose(DEMO_SEQUENCE[0])

    def stop(self) -> None:
        self._stop_requested = True

    def _run_sequence(self, sequence: Iterable[DemoPose]) -> None:
        for pose in sequence:
            if not rclpy.ok() or self._stop_requested:
                return
            self._send_pose(pose)
            dwell = max(0.0, pose.dwell_sec * self._param_float("dwell_scale"))
            self._sleep_with_spin(dwell)

    def _send_pose(self, pose: DemoPose) -> None:
        duration = self._scaled_duration(pose.duration_sec)
        self.get_logger().info(
            f"Pose {pose.name}: arm duration={duration:.2f}s"
            + (f", gripper={pose.gripper:.2f}rad" if pose.gripper is not None else "")
        )
        self._send_trajectory(
            client=self._arm_client,
            joint_names=ARM_JOINTS,
            positions=pose.arm,
            duration_sec=duration,
        )
        if pose.gripper is not None:
            self._send_trajectory(
                client=self._gripper_client,
                joint_names=[GRIPPER_JOINT],
                positions=[pose.gripper],
                duration_sec=max(1.0, min(duration, 2.0)),
            )

    def _send_trajectory(
        self,
        client: ActionClient,
        joint_names: List[str],
        positions: List[float],
        duration_sec: float,
    ) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.time_from_start = self._duration_msg(duration_sec)
        goal.trajectory.points = [point]

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        if goal_handle is None:
            self.get_logger().error(f"No response from action server for {joint_names}.")
            return False
        if not goal_handle.accepted:
            self.get_logger().error(f"Trajectory goal rejected for {joint_names}.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=duration_sec + 5.0,
        )
        result = result_future.result()
        if result is None:
            self.get_logger().warn("No trajectory result received before timeout.")
            return False
        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().warn(
                "Trajectory finished with error "
                f"{result.result.error_code}: {result.result.error_string}"
            )
            return False
        return True

    def _wait_for_servers(self) -> bool:
        arm_ready = self._arm_client.wait_for_server(timeout_sec=30.0)
        gripper_ready = self._gripper_client.wait_for_server(timeout_sec=30.0)
        return arm_ready and gripper_ready

    def _scaled_duration(self, duration_sec: float) -> float:
        speed_scale = self._param_float("speed_scale")
        if not math.isfinite(speed_scale) or speed_scale <= 0.0:
            speed_scale = 1.0
        return max(0.5, duration_sec / speed_scale)

    def _param_float(self, name: str) -> float:
        return float(self.get_parameter(name).get_parameter_value().double_value)

    @staticmethod
    def _duration_msg(seconds: float) -> Duration:
        sec = int(seconds)
        nanosec = int((seconds - sec) * 1_000_000_000)
        return Duration(sec=sec, nanosec=nanosec)

    def _sleep_with_spin(self, seconds: float) -> None:
        end_time = time.monotonic() + seconds
        while rclpy.ok() and not self._stop_requested and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = AutoMotionNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
