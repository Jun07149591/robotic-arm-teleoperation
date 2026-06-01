"""
EL-A3 轨迹规划模块

提供 S-curve 和三次样条轨迹规划，参考 C++ s_curve_generator.hpp 实现。
无 ROS 依赖，CAN 和 ROS 模式共用。
"""

import math
import logging
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

logger = logging.getLogger("el_a3_sdk.motion.trajectory")


@dataclass
class TrajectoryPoint:
    """单个轨迹点"""
    time: float = 0.0
    positions: List[float] = field(default_factory=list)
    velocities: List[float] = field(default_factory=list)
    accelerations: List[float] = field(default_factory=list)


@dataclass
class SCurveProfile:
    """S-curve 7 段剖面参数"""
    t1: float = 0.0  # jerk ramp up (accel phase)
    t2: float = 0.0  # constant accel
    t3: float = 0.0  # jerk ramp down (accel phase end)
    t4: float = 0.0  # cruise
    t5: float = 0.0  # jerk ramp down (decel phase)
    t6: float = 0.0  # constant decel
    t7: float = 0.0  # jerk ramp up (decel phase end)
    total_time: float = 0.0
    j_max: float = 50.0
    a_max: float = 10.0
    v_max: float = 3.0
    distance: float = 0.0
    direction: float = 1.0
    v_cruise: float = 0.0
    a_limit: float = 0.0
    p0: float = 0.0
    v0: float = 0.0

    @property
    def segment_times(self) -> List[float]:
        return [self.t1, self.t2, self.t3, self.t4, self.t5, self.t6, self.t7]


class SCurvePlanner:
    """
    单关节 S-curve 轨迹规划器

    实现标准 7 段式 S-curve 速度剖面。
    """

    EPSILON = 1e-9

    def __init__(self, v_max: float = 3.0, a_max: float = 10.0, j_max: float = 50.0):
        self.v_max = v_max
        self.a_max = a_max
        self.j_max = j_max

    def plan(
        self, start: float, end: float,
        v_max: Optional[float] = None,
        a_max: Optional[float] = None,
        j_max: Optional[float] = None,
    ) -> SCurveProfile:
        """
        计算从 start 到 end 的 S-curve 剖面

        统一处理所有情况：
        1. 确定自然峰值速度 v_peak（无巡航、无 vm 约束时能覆盖 d 的速度）
        2. 用 min(v_peak, vm) 作为实际峰值速度
        3. 若 2*d_accel < d，补上巡航段

        Returns:
            SCurveProfile
        """
        vm = v_max or self.v_max
        am = a_max or self.a_max
        jm = j_max or self.j_max

        dist = end - start
        direction = 1.0 if dist >= 0 else -1.0
        d = abs(dist)

        if d < self.EPSILON:
            prof = SCurveProfile(p0=start)
            return prof

        t_j = am / jm

        # --- Step 1: natural peak velocity (no cruise, no vm cap) ---
        B = am * am / jm
        disc = B * B + 4.0 * am * d
        v_nat_trap = 0.5 * (-B + math.sqrt(disc)) if disc >= 0 else 0.0

        if v_nat_trap > am * t_j:
            v_natural = v_nat_trap
        else:
            t1_tri = (d / (2.0 * jm)) ** (1.0 / 3.0)
            v_natural = jm * t1_tri * t1_tri

        v_peak = min(v_natural, vm)

        # --- Step 2: accel/decel segment timing ---
        if v_peak >= am * t_j - self.EPSILON:
            t1 = t_j
            t2 = max(0.0, v_peak / am - t_j)
            t3 = t_j
            d_accel = v_peak * (am / jm + v_peak / am)
        else:
            t1 = math.sqrt(v_peak / jm)
            t2 = 0.0
            t3 = t1
            d_accel = jm * t1 * t1 * t1

        # --- Step 3: cruise phase ---
        d_cruise = d - 2.0 * d_accel
        t4 = max(0.0, d_cruise / v_peak) if v_peak > self.EPSILON else 0.0

        prof = SCurveProfile()
        prof.t1, prof.t2, prof.t3 = t1, t2, t3
        prof.t4 = t4
        prof.t5, prof.t6, prof.t7 = t3, t2, t1
        prof.total_time = sum(prof.segment_times)
        prof.v_cruise = v_peak
        prof.a_limit = min(am, jm * t1)
        prof.direction = direction
        prof.distance = d
        prof.p0 = start
        prof.j_max = jm
        prof.a_max = am
        prof.v_max = vm

        return prof

    def evaluate(self, prof: SCurveProfile, t: float) -> Tuple[float, float, float]:
        """
        在时刻 t 对剖面求值

        Returns:
            (position, velocity, acceleration)
        """
        t = max(0.0, min(t, prof.total_time))
        d = prof.direction
        jm = prof.j_max

        segs = prof.segment_times
        boundaries = [0.0]
        for s in segs:
            boundaries.append(boundaries[-1] + s)

        p = prof.p0
        v = prof.v0
        a = 0.0

        dt_remaining = t

        for seg_idx in range(7):
            seg_dur = segs[seg_idx]
            if seg_dur < self.EPSILON:
                continue

            dt = min(dt_remaining, seg_dur)

            if seg_idx == 0:
                j = d * jm
            elif seg_idx == 1:
                j = 0.0
            elif seg_idx == 2:
                j = -d * jm
            elif seg_idx == 3:
                j = 0.0
            elif seg_idx == 4:
                j = -d * jm
            elif seg_idx == 5:
                j = 0.0
            else:
                j = d * jm

            p += v * dt + 0.5 * a * dt**2 + (1.0 / 6.0) * j * dt**3
            v += a * dt + 0.5 * j * dt**2
            a += j * dt

            dt_remaining -= dt
            if dt_remaining <= self.EPSILON:
                break

        return (p, v, a)

    def generate_trajectory(
        self, prof: SCurveProfile, dt: float = 0.005,
    ) -> List[TrajectoryPoint]:
        """
        生成时间采样的轨迹点列表

        Args:
            prof: S-curve 剖面
            dt: 采样间隔（秒）

        Returns:
            TrajectoryPoint 列表
        """
        points = []
        n_steps = max(1, int(math.ceil(prof.total_time / dt)))

        for i in range(n_steps + 1):
            t = min(i * dt, prof.total_time)
            p, v, a = self.evaluate(prof, t)
            points.append(TrajectoryPoint(
                time=t, positions=[p], velocities=[v], accelerations=[a],
            ))

        return points

    def _long_profile(self, d: float, vm: float, am: float, jm: float) -> SCurveProfile:
        """距离足以达到最大速度"""
        t_j = am / jm
        t_a = vm / am
        t1 = t_j
        t2 = t_a - t_j
        t3 = t_j

        if t2 < -self.EPSILON:
            return self._short_profile(d, vm, am, jm)

        t2 = max(0.0, t2)
        d_accel = vm * (am / jm + vm / am)
        d_decel = d_accel
        d_cruise = d - d_accel - d_decel

        if d_cruise < -self.EPSILON:
            return self._short_profile(d, vm, am, jm)

        t4 = d_cruise / vm
        t5 = t_j
        t6 = t2
        t7 = t_j

        prof = SCurveProfile()
        prof.t1, prof.t2, prof.t3 = t1, max(0, t2), t3
        prof.t4 = max(0, t4)
        prof.t5, prof.t6, prof.t7 = t5, max(0, t6), t7
        prof.total_time = sum(prof.segment_times)
        prof.v_cruise = vm
        prof.a_limit = am
        return prof

    def _short_profile(self, d: float, vm: float, am: float, jm: float) -> SCurveProfile:
        """距离不足以达到最大速度，需降低巡航速度"""
        t_j = am / jm

        B = am * am / jm
        disc = B * B + 4.0 * am * d
        v_peak = 0.5 * (-B + math.sqrt(disc)) if disc >= 0 else 0.0
        v_peak = min(v_peak, vm)

        if v_peak >= am * t_j - self.EPSILON:
            t1 = t_j
            t2 = v_peak / am - t_j
            t3 = t_j
        else:
            t1 = (d / (2.0 * jm)) ** (1.0 / 3.0)
            v_peak = jm * t1 * t1
            t2 = 0.0
            t3 = t1

        prof = SCurveProfile()
        prof.t1, prof.t2, prof.t3 = t1, max(0, t2), t3
        prof.t4 = 0.0
        prof.t5, prof.t6, prof.t7 = t3, max(0, t2), t1
        prof.total_time = sum(prof.segment_times)
        prof.v_cruise = v_peak
        prof.a_limit = min(am, jm * t1)
        return prof


class MultiJointPlanner:
    """
    多关节同步 S-curve 规划器

    所有关节在相同时间内完成运动，维持比例协调。
    """

    def __init__(self, n_joints: int = 6,
                 v_max: float = 3.0, a_max: float = 10.0, j_max: float = 50.0):
        self.n_joints = n_joints
        self.v_max = v_max
        self.a_max = a_max
        self.j_max = j_max
        self._planners = [SCurvePlanner(v_max, a_max, j_max) for _ in range(n_joints)]

    def plan_sync(
        self,
        starts: List[float],
        ends: List[float],
        v_max: Optional[List[float]] = None,
        a_max: Optional[List[float]] = None,
    ) -> List[SCurveProfile]:
        """
        同步规划：所有关节同时到达

        通过二分搜索 v_max 使每个关节的规划时间匹配最慢关节。

        Returns:
            各关节的 SCurveProfile 列表
        """
        vm_list = v_max or [self.v_max] * self.n_joints
        am_list = a_max or [self.a_max] * self.n_joints

        profiles = []
        max_time = 0.0
        for i in range(self.n_joints):
            prof = self._planners[i].plan(starts[i], ends[i], vm_list[i], am_list[i])
            profiles.append(prof)
            max_time = max(max_time, prof.total_time)

        if max_time < SCurvePlanner.EPSILON:
            return profiles

        synced = []
        for i in range(self.n_joints):
            d = abs(ends[i] - starts[i])
            if d < SCurvePlanner.EPSILON:
                prof = SCurveProfile(p0=starts[i])
                prof.total_time = max_time
                synced.append(prof)
                continue

            if abs(profiles[i].total_time - max_time) < 1e-4:
                synced.append(profiles[i])
                continue

            vm_hi = vm_list[i]
            vm_lo = d / max_time
            am_i = am_list[i]
            best = profiles[i]

            for _ in range(20):
                vm_mid = 0.5 * (vm_lo + vm_hi)
                trial = self._planners[i].plan(starts[i], ends[i], vm_mid, am_i)
                if trial.total_time > max_time:
                    vm_lo = vm_mid
                else:
                    vm_hi = vm_mid
                    best = trial
                if abs(trial.total_time - max_time) < 1e-4:
                    best = trial
                    break

            synced.append(best)

        return synced

    def generate_trajectory(
        self, profiles: List[SCurveProfile], dt: float = 0.005,
    ) -> List[TrajectoryPoint]:
        """
        将多个单关节剖面合并为多关节轨迹点列表

        Returns:
            TrajectoryPoint 列表（每个点包含所有关节的 pos/vel/acc）
        """
        max_time = max(p.total_time for p in profiles) if profiles else 0.0
        if max_time < SCurvePlanner.EPSILON:
            return [TrajectoryPoint(
                time=0.0,
                positions=[p.p0 for p in profiles],
                velocities=[0.0] * len(profiles),
                accelerations=[0.0] * len(profiles),
            )]

        n_steps = max(1, int(math.ceil(max_time / dt)))
        points = []
        evaluator = SCurvePlanner()

        for step in range(n_steps + 1):
            t = min(step * dt, max_time)
            pos, vel, acc = [], [], []
            for i, prof in enumerate(profiles):
                p, v, a = evaluator.evaluate(prof, t)
                pos.append(p)
                vel.append(v)
                acc.append(a)
            points.append(TrajectoryPoint(time=t, positions=pos, velocities=vel, accelerations=acc))

        if points:
            last = points[-1]
            for i, prof in enumerate(profiles):
                target = prof.p0 + prof.direction * prof.distance
                last.positions[i] = target
                last.velocities[i] = 0.0
                last.accelerations[i] = 0.0

        return points


class CubicSplinePlanner:
    """
    三次样条插值轨迹规划器

    适用于多路点轨迹平滑。
    """

    @staticmethod
    def plan_waypoints(
        waypoints: List[List[float]],
        durations: List[float],
        dt: float = 0.005,
    ) -> List[TrajectoryPoint]:
        """
        三次样条多路点轨迹规划

        Args:
            waypoints: 路点列表，每个路点为关节角度列表
            durations: 各段持续时间（长度 = len(waypoints) - 1）
            dt: 采样间隔

        Returns:
            TrajectoryPoint 列表
        """
        n_wp = len(waypoints)
        if n_wp < 2:
            return [TrajectoryPoint(positions=list(waypoints[0]))] if waypoints else []

        n_joints = len(waypoints[0])
        cum_times = [0.0]
        for d in durations:
            cum_times.append(cum_times[-1] + d)
        total_time = cum_times[-1]

        n_steps = max(1, int(math.ceil(total_time / dt)))
        points = []

        for step in range(n_steps + 1):
            t = min(step * dt, total_time)

            seg = 0
            for k in range(len(durations)):
                if t <= cum_times[k + 1] + 1e-9:
                    seg = k
                    break
                seg = k

            t_local = t - cum_times[seg]
            seg_dur = durations[seg]
            if seg_dur < 1e-9:
                s = 1.0
            else:
                s = t_local / seg_dur

            s2 = s * s
            s3 = s2 * s
            h00 = 2 * s3 - 3 * s2 + 1
            h10 = s3 - 2 * s2 + s
            h01 = -2 * s3 + 3 * s2
            h11 = s3 - s2

            pos = []
            vel = []
            for j in range(n_joints):
                p0 = waypoints[seg][j]
                p1 = waypoints[seg + 1][j] if seg + 1 < n_wp else p0

                if seg == 0:
                    m0 = (p1 - p0) if seg_dur < 1e-9 else (p1 - p0)
                else:
                    m0 = 0.5 * (p1 - waypoints[seg - 1][j])
                if seg + 1 >= n_wp - 1:
                    m1 = (p1 - p0) if seg_dur < 1e-9 else (p1 - p0)
                else:
                    m1 = 0.5 * (waypoints[seg + 2][j] - p0)

                p = h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1
                pos.append(p)

                if seg_dur > 1e-9:
                    dh00 = 6 * s2 - 6 * s
                    dh10 = 3 * s2 - 4 * s + 1
                    dh01 = -6 * s2 + 6 * s
                    dh11 = 3 * s2 - 2 * s
                    dp = (dh00 * p0 + dh10 * m0 + dh01 * p1 + dh11 * m1) / seg_dur
                    vel.append(dp)
                else:
                    vel.append(0.0)

            points.append(TrajectoryPoint(
                time=t, positions=pos, velocities=vel,
                accelerations=[0.0] * n_joints,
            ))

        return points
