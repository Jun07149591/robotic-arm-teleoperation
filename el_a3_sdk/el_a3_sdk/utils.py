"""
EL-A3 SDK 工具函数

数据映射转换、单位换算等。
"""

import math
import os
import sys

import numpy as np

# Prefer pip-installed pinocchio (cmeel prefix) over ROS 2 Python 3.10 packages
def _prefer_cmeel_site_packages():
    cmeel_paths = []
    for base in list(sys.path):
        if "site-packages" in base:
            cmeel = os.path.join(
                base, "cmeel.prefix", "lib",
                f"python{sys.version_info.major}.{sys.version_info.minor}",
                "site-packages",
            )
            if os.path.isdir(cmeel):
                cmeel_paths.append(cmeel)
    for p in reversed(cmeel_paths):
        if p in sys.path:
            sys.path.remove(p)
        sys.path.insert(0, p)

_prefer_cmeel_site_packages()

try:
    import pinocchio as pin
    PINOCCHIO_AVAILABLE = True
except ImportError:
    PINOCCHIO_AVAILABLE = False


def float_to_uint16(x: float, x_min: float, x_max: float) -> int:
    """浮点数线性映射到 uint16 (0~65535)"""
    x = max(x_min, min(x_max, x))
    return int((x - x_min) * 65535.0 / (x_max - x_min))


def uint16_to_float(x_int: int, x_min: float, x_max: float) -> float:
    """uint16 (0~65535) 线性映射到浮点数"""
    return x_int * (x_max - x_min) / 65535.0 + x_min


def rad_to_deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


def clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(max_val, value))


def euler_to_quat(rx: float, ry: float, rz: float):
    """RPY (rad) -> quaternion (w, x, y, z)."""
    if PINOCCHIO_AVAILABLE:
        R = pin.rpy.rpyToMatrix(rx, ry, rz)
        return _matrix_to_quat(R)
    cx, sx = math.cos(rx / 2), math.sin(rx / 2)
    cy, sy = math.cos(ry / 2), math.sin(ry / 2)
    cz, sz = math.cos(rz / 2), math.sin(rz / 2)
    w = cx * cy * cz - sx * sy * sz
    x = sx * cy * cz + cx * sy * sz
    y = cx * sy * cz - sx * cy * sz
    z = cx * cy * sz + sx * sy * cz
    return (w, x, y, z)


def quat_to_euler(w: float, x: float, y: float, z: float):
    """Quaternion (w, x, y, z) -> RPY (rad)."""
    if PINOCCHIO_AVAILABLE:
        R = _quat_to_matrix(np.array([w, x, y, z], dtype=float))
        rx, ry, rz = pin.rpy.matrixToRpy(R)
        return (float(rx), float(ry), float(rz))

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    rx = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = clamp(sinp, -1.0, 1.0)
    ry = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    rz = math.atan2(siny_cosp, cosy_cosp)

    return (rx, ry, rz)


def slerp_euler(rx0: float, ry0: float, rz0: float,
                rx1: float, ry1: float, rz1: float,
                t: float):
    """Spherical linear interpolation between two RPY poses."""
    t = clamp(t, 0.0, 1.0)

    if PINOCCHIO_AVAILABLE:
        R0 = pin.rpy.rpyToMatrix(rx0, ry0, rz0)
        R1 = pin.rpy.rpyToMatrix(rx1, ry1, rz1)
        q0 = _matrix_to_quat(R0)
        q1 = _matrix_to_quat(R1)
        q = _slerp_quat(q0, q1, t)
        R = _quat_to_matrix(q)
        rx, ry, rz = pin.rpy.matrixToRpy(R)
        return (float(rx), float(ry), float(rz))

    # Fallback: shortest-path linear interpolation on wrapped angles.
    return tuple(_lerp_angle(a0, a1, t) for a0, a1 in ((rx0, rx1), (ry0, ry1), (rz0, rz1)))


def _normalize_quat(q: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / norm


def _matrix_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> quaternion (w, x, y, z)."""
    m = np.asarray(R, dtype=float)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return _normalize_quat(np.array([w, x, y, z], dtype=float))


def _quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """Quaternion (w, x, y, z) -> rotation matrix."""
    w, x, y, z = _normalize_quat(np.asarray(q, dtype=float))
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),     2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w),       1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w),       2.0 * (y * z + x * w),     1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)


def _slerp_quat(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    q0 = _normalize_quat(np.asarray(q0, dtype=float))
    q1 = _normalize_quat(np.asarray(q1, dtype=float))
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = clamp(dot, -1.0, 1.0)
    if dot > 0.9995:
        return _normalize_quat(q0 + t * (q1 - q0))

    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    if abs(sin_theta) < 1e-12:
        return q0
    s0 = math.sin((1.0 - t) * theta) / sin_theta
    s1 = math.sin(t * theta) / sin_theta
    return _normalize_quat(s0 * q0 + s1 * q1)


def _lerp_angle(a0: float, a1: float, t: float) -> float:
    delta = (a1 - a0 + math.pi) % (2.0 * math.pi) - math.pi
    return a0 + delta * t
