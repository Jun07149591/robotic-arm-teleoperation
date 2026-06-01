"""示教轨迹录制与回放"""

import json
import time
from pathlib import Path
from dataclasses import dataclass, field, asdict
from typing import List, Optional


@dataclass
class TrajectoryPoint:
    """单个轨迹点"""
    timestamp: float = 0.0
    positions: List[float] = field(default_factory=lambda: [0.0] * 7)


@dataclass
class RecordedTrajectory:
    """录制的完整轨迹"""
    name: str = ""
    created_at: str = ""
    sample_rate_hz: float = 10.0
    num_joints: int = 7
    points: List[TrajectoryPoint] = field(default_factory=list)

    @property
    def duration(self) -> float:
        if len(self.points) < 2:
            return 0.0
        return self.points[-1].timestamp - self.points[0].timestamp

    @property
    def num_points(self) -> int:
        return len(self.points)


class TrajectoryRecorder:
    """录制和回放控制器"""

    def __init__(self, sample_rate_hz: float = 10.0):
        self.sample_rate_hz = sample_rate_hz
        self._recording = False
        self._current_trajectory: Optional[RecordedTrajectory] = None
        self._start_time = 0.0
        self._last_sample_time = 0.0
        self.trajectories: List[RecordedTrajectory] = []

    @property
    def is_recording(self) -> bool:
        return self._recording

    @property
    def current_trajectory(self) -> Optional[RecordedTrajectory]:
        return self._current_trajectory

    def start_recording(self, name: str = ""):
        self._current_trajectory = RecordedTrajectory(
            name=name or f"trajectory_{len(self.trajectories)+1}",
            created_at=time.strftime("%Y-%m-%d %H:%M:%S"),
            sample_rate_hz=self.sample_rate_hz,
        )
        self._start_time = time.time()
        self._last_sample_time = 0.0
        self._recording = True

    def stop_recording(self) -> Optional[RecordedTrajectory]:
        if not self._recording:
            return None
        self._recording = False
        traj = self._current_trajectory
        if traj and traj.num_points > 0:
            self.trajectories.append(traj)
        return traj

    def add_sample(self, positions: List[float]) -> bool:
        if not self._recording or self._current_trajectory is None:
            return False
        now = time.time()
        elapsed = now - self._start_time
        interval = 1.0 / self.sample_rate_hz
        if elapsed - self._last_sample_time < interval * 0.9:
            return False
        self._last_sample_time = elapsed
        self._current_trajectory.points.append(
            TrajectoryPoint(timestamp=elapsed, positions=list(positions))
        )
        return True

    def save_trajectory(self, trajectory: RecordedTrajectory, filepath: str):
        data = asdict(trajectory)
        with open(filepath, "w") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

    def load_trajectory(self, filepath: str) -> RecordedTrajectory:
        with open(filepath, "r") as f:
            data = json.load(f)
        points = [TrajectoryPoint(**p) for p in data.get("points", [])]
        traj = RecordedTrajectory(
            name=data.get("name", ""),
            created_at=data.get("created_at", ""),
            sample_rate_hz=data.get("sample_rate_hz", 10.0),
            num_joints=data.get("num_joints", 7),
            points=points,
        )
        self.trajectories.append(traj)
        return traj
