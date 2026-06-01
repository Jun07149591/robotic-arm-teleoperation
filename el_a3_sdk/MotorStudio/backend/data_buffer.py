"""环形数据缓冲区，用于实时曲线绘制"""

import numpy as np
import csv
import time
from pathlib import Path


class DataBuffer:
    """
    固定长度的环形缓冲区，存储多通道时间序列数据。
    线程安全通过 numpy 数组的原子性赋值保证（单写者场景）。
    """

    def __init__(self, max_samples: int = 500, num_channels: int = 7):
        self.max_samples = max_samples
        self.num_channels = num_channels
        self._index = 0
        self._count = 0

        self.positions = np.zeros((max_samples, num_channels))
        self.velocities = np.zeros((max_samples, num_channels))
        self.torques = np.zeros((max_samples, num_channels))
        self.temperatures = np.zeros((max_samples, num_channels))
        self.timestamps = np.zeros(max_samples)

    def append(self, timestamp: float, positions=None, velocities=None,
               torques=None, temperatures=None):
        idx = self._index % self.max_samples
        self.timestamps[idx] = timestamp

        if positions is not None:
            self.positions[idx, :len(positions)] = positions[:self.num_channels]
        if velocities is not None:
            self.velocities[idx, :len(velocities)] = velocities[:self.num_channels]
        if torques is not None:
            self.torques[idx, :len(torques)] = torques[:self.num_channels]
        if temperatures is not None:
            self.temperatures[idx, :len(temperatures)] = temperatures[:self.num_channels]

        self._index += 1
        self._count = min(self._count + 1, self.max_samples)

    def get_data(self):
        """返回按时间顺序排列的数据切片"""
        if self._count < self.max_samples:
            sl = slice(0, self._count)
            return (
                self.timestamps[sl].copy(),
                self.positions[sl].copy(),
                self.velocities[sl].copy(),
                self.torques[sl].copy(),
                self.temperatures[sl].copy(),
            )
        start = self._index % self.max_samples
        order = np.roll(np.arange(self.max_samples), -start)
        return (
            self.timestamps[order].copy(),
            self.positions[order].copy(),
            self.velocities[order].copy(),
            self.torques[order].copy(),
            self.temperatures[order].copy(),
        )

    @property
    def count(self):
        return self._count

    def clear(self):
        self._index = 0
        self._count = 0
        self.positions[:] = 0
        self.velocities[:] = 0
        self.torques[:] = 0
        self.temperatures[:] = 0
        self.timestamps[:] = 0

    def export_csv(self, filepath: str):
        ts, pos, vel, torq, temp = self.get_data()
        path = Path(filepath)
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            header = ["timestamp"]
            for prefix in ["pos", "vel", "torque", "temp"]:
                for j in range(self.num_channels):
                    header.append(f"{prefix}_L{j+1}")
            writer.writerow(header)
            for i in range(len(ts)):
                row = [f"{ts[i]:.4f}"]
                for arr in [pos, vel, torq, temp]:
                    for j in range(self.num_channels):
                        row.append(f"{arr[i, j]:.6f}")
                writer.writerow(row)
