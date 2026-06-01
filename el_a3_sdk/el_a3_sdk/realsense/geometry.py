"""Small geometry helpers for camera and robot coordinate transforms."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Sequence, Union

import numpy as np


ArrayLike3 = Union[Sequence[float], np.ndarray]
MatrixLike4 = Union[Sequence[Sequence[float]], np.ndarray]
PathLike = Union[str, Path]


def rpy_to_matrix(rpy_rad: ArrayLike3) -> np.ndarray:
    """Convert roll, pitch, yaw radians to a rotation matrix.

    The convention is ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``.
    """

    roll, pitch, yaw = np.asarray(rpy_rad, dtype=float).reshape(3)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    rotation_x = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, cr, -sr],
            [0.0, sr, cr],
        ],
        dtype=float,
    )
    rotation_y = np.array(
        [
            [cp, 0.0, sp],
            [0.0, 1.0, 0.0],
            [-sp, 0.0, cp],
        ],
        dtype=float,
    )
    rotation_z = np.array(
        [
            [cy, -sy, 0.0],
            [sy, cy, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    return rotation_z @ rotation_y @ rotation_x


@dataclass(frozen=True)
class RigidTransform:
    """3D rigid transform where ``target = R @ source + t``."""

    rotation: np.ndarray
    translation: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "rotation",
            np.asarray(self.rotation, dtype=float).reshape(3, 3),
        )
        object.__setattr__(
            self,
            "translation",
            np.asarray(self.translation, dtype=float).reshape(3),
        )

    @classmethod
    def identity(cls) -> "RigidTransform":
        return cls(rotation=np.eye(3), translation=np.zeros(3))

    @classmethod
    def from_xyz_rpy_deg(
        cls,
        xyz_m: ArrayLike3,
        rpy_deg: ArrayLike3,
    ) -> "RigidTransform":
        """Build a transform from translation in meters and RPY in degrees."""

        return cls(
            rotation=rpy_to_matrix(np.deg2rad(np.asarray(rpy_deg, dtype=float))),
            translation=np.asarray(xyz_m, dtype=float),
        )

    @classmethod
    def from_matrix(cls, matrix: MatrixLike4) -> "RigidTransform":
        """Build a transform from a 4x4 homogeneous transform matrix."""

        transform = np.asarray(matrix, dtype=float).reshape(4, 4)
        return cls(rotation=transform[:3, :3], translation=transform[:3, 3])

    @classmethod
    def from_json(cls, path: PathLike) -> "RigidTransform":
        """Load a transform from a JSON calibration file.

        Supported formats:

        ``{"xyz_m": [x, y, z], "rpy_deg": [roll, pitch, yaw]}``
        ``{"matrix": [[...], [...], [...], [...]]}``
        """

        with Path(path).open("r", encoding="utf-8") as file:
            payload = json.load(file)

        if "xyz_m" in payload and "rpy_deg" in payload:
            return cls.from_xyz_rpy_deg(payload["xyz_m"], payload["rpy_deg"])
        if "matrix" in payload:
            return cls.from_matrix(payload["matrix"])
        raise ValueError("标定文件必须包含 xyz_m 和 rpy_deg，或包含 matrix。")

    @property
    def matrix(self) -> np.ndarray:
        transform = np.eye(4)
        transform[:3, :3] = self.rotation
        transform[:3, 3] = self.translation
        return transform

    def transform_point(self, point: ArrayLike3) -> np.ndarray:
        """Transform one 3D point from source to target coordinates."""

        return self.rotation @ np.asarray(point, dtype=float).reshape(3) + self.translation

    def transform_points(self, points: np.ndarray) -> np.ndarray:
        """Transform an ``N x 3`` array from source to target coordinates."""

        values = np.asarray(points, dtype=float).reshape(-1, 3)
        return values @ self.rotation.T + self.translation


__all__ = ["RigidTransform", "rpy_to_matrix"]
