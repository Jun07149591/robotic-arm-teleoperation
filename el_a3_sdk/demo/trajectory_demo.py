#!/usr/bin/env python3
"""
轨迹规划示例

演示 S-curve 轨迹规划器和多关节同步。
无需硬件和 ROS，纯离线计算。
"""

import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from el_a3_sdk.motion import SCurvePlanner, MultiJointPlanner, CubicSplinePlanner


def demo_single_joint():
    print("=== 单关节 S-curve ===")
    planner = SCurvePlanner(v_max=3.0, a_max=10.0, j_max=50.0)

    prof = planner.plan(start=0.0, end=1.5)
    print(f"从 0.0 到 1.5 rad:")
    print(f"  总时间: {prof.total_time:.4f} s")
    print(f"  巡航速度: {prof.v_cruise:.4f} rad/s")
    print(f"  段时间: {[f'{t:.4f}' for t in prof.segment_times]}")

    traj = planner.generate_trajectory(prof, dt=0.01)
    print(f"  轨迹点数: {len(traj)}")
    print(f"  起始: pos={traj[0].positions[0]:.4f}")
    print(f"  结束: pos={traj[-1].positions[0]:.4f}")
    mid = len(traj) // 2
    print(f"  中点: t={traj[mid].time:.3f}s, pos={traj[mid].positions[0]:.4f}, vel={traj[mid].velocities[0]:.4f}")


def demo_multi_joint():
    print("\n=== 多关节同步 S-curve ===")
    planner = MultiJointPlanner(n_joints=6, v_max=3.0, a_max=10.0, j_max=50.0)

    starts = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ends = [1.0, 2.0, -1.5, 0.5, -0.3, 0.8]

    profiles = planner.plan_sync(starts, ends)
    print(f"各关节总时间: {[f'{p.total_time:.4f}' for p in profiles]}")

    traj = planner.generate_trajectory(profiles, dt=0.01)
    print(f"轨迹点数: {len(traj)}")
    print(f"起始: {[f'{p:.3f}' for p in traj[0].positions]}")
    print(f"结束: {[f'{p:.3f}' for p in traj[-1].positions]}")
    print(f"目标: {[f'{p:.3f}' for p in ends]}")


def demo_cubic_spline():
    print("\n=== 三次样条多路点 ===")
    waypoints = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.5, 1.0, -0.5, 0.2, 0.0, 0.0],
        [1.0, 0.5, -1.0, 0.0, 0.3, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ]
    durations = [2.0, 1.5, 2.0]

    traj = CubicSplinePlanner.plan_waypoints(waypoints, durations, dt=0.02)
    print(f"路点数: {len(waypoints)}, 段数: {len(durations)}")
    print(f"总时间: {sum(durations):.1f} s")
    print(f"轨迹点数: {len(traj)}")
    print(f"起始: {[f'{p:.3f}' for p in traj[0].positions]}")
    mid = len(traj) // 3
    print(f"1/3处: t={traj[mid].time:.2f}s, pos={[f'{p:.3f}' for p in traj[mid].positions]}")
    print(f"结束: {[f'{p:.3f}' for p in traj[-1].positions]}")


if __name__ == "__main__":
    demo_single_joint()
    demo_multi_joint()
    demo_cubic_spline()
    print("\n完成")
