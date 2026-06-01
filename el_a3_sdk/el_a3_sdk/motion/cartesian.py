"""Stateless helpers used by Cartesian and trajectory execution."""

from typing import List, Tuple

from el_a3_sdk.data_types import ArmEndPose
from el_a3_sdk.utils import clamp, slerp_euler


def smooth_time_scale(tau: float) -> float:
    """Quintic time scaling with zero velocity/acceleration at endpoints."""

    tau = clamp(tau, 0.0, 1.0)
    return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau))


def interpolate_pose(start_pose: ArmEndPose, target_pose: ArmEndPose, s: float) -> ArmEndPose:
    """Interpolate position linearly and orientation with quaternion slerp."""

    interp_rx, interp_ry, interp_rz = slerp_euler(
        start_pose.rx,
        start_pose.ry,
        start_pose.rz,
        target_pose.rx,
        target_pose.ry,
        target_pose.rz,
        s,
    )
    return ArmEndPose(
        x=start_pose.x + s * (target_pose.x - start_pose.x),
        y=start_pose.y + s * (target_pose.y - start_pose.y),
        z=start_pose.z + s * (target_pose.z - start_pose.z),
        rx=interp_rx,
        ry=interp_ry,
        rz=interp_rz,
    )


def sample_trajectory(
    traj_points: List,
    index: int,
    elapsed: float,
    n_joints: int,
) -> Tuple[List[float], List[float]]:
    """Linearly sample trajectory points at elapsed time."""

    if not traj_points:
        return [], []

    if len(traj_points) == 1 or elapsed <= traj_points[0].time:
        pt = traj_points[0]
        velocities = list(pt.velocities) if pt.velocities else [0.0] * len(pt.positions)
        return list(pt.positions), velocities

    last = traj_points[-1]
    if elapsed >= last.time:
        velocities = list(last.velocities) if last.velocities else [0.0] * len(last.positions)
        return list(last.positions), velocities

    idx = max(0, min(index, len(traj_points) - 2))
    p0 = traj_points[idx]
    p1 = traj_points[idx + 1]
    seg_dt = max(p1.time - p0.time, 1e-9)
    u = clamp((elapsed - p0.time) / seg_dt, 0.0, 1.0)

    n = min(len(p0.positions), len(p1.positions), n_joints)
    positions = [
        p0.positions[i] + u * (p1.positions[i] - p0.positions[i])
        for i in range(n)
    ]

    if p0.velocities and p1.velocities:
        nv = min(len(p0.velocities), len(p1.velocities), n_joints)
        velocities = [
            p0.velocities[i] + u * (p1.velocities[i] - p0.velocities[i])
            for i in range(nv)
        ]
    else:
        velocities = [
            (p1.positions[i] - p0.positions[i]) / seg_dt
            for i in range(n)
        ]

    return positions, velocities


def fill_trajectory_derivatives(traj_points: List, n_joints: int) -> None:
    """Fill velocity and acceleration feedforward by central differences."""

    if not traj_points:
        return
    n_joints = min(len(traj_points[0].positions), n_joints)
    zeros = [0.0] * n_joints
    if len(traj_points) < 3:
        for pt in traj_points:
            pt.velocities = list(zeros)
            pt.accelerations = list(zeros)
        return

    velocities = []
    accelerations = []
    for idx, pt in enumerate(traj_points):
        if idx == 0 or idx == len(traj_points) - 1:
            velocities.append(list(zeros))
            accelerations.append(list(zeros))
            continue

        prev_pt = traj_points[idx - 1]
        next_pt = traj_points[idx + 1]
        dt = max(next_pt.time - prev_pt.time, 1e-9)
        velocities.append([
            (next_pt.positions[j] - prev_pt.positions[j]) / dt
            for j in range(n_joints)
        ])

        dt_prev = max(pt.time - prev_pt.time, 1e-9)
        dt_next = max(next_pt.time - pt.time, 1e-9)
        accelerations.append([
            2.0
            * (
                (next_pt.positions[j] - pt.positions[j]) / dt_next
                - (pt.positions[j] - prev_pt.positions[j]) / dt_prev
            )
            / (dt_prev + dt_next)
            for j in range(n_joints)
        ])

    for pt, vel, acc in zip(traj_points, velocities, accelerations):
        pt.velocities = vel
        pt.accelerations = acc


__all__ = [
    "smooth_time_scale",
    "interpolate_pose",
    "sample_trajectory",
    "fill_trajectory_derivatives",
]
