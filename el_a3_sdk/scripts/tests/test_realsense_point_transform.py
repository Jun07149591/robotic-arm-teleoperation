import numpy as np

from MotorStudio.widgets.realsense_point_panel import (
    apply_tcp_offset_correction,
    camera_point_to_robot_target,
)


def test_camera_point_transform():
    point = np.array([0.01, -0.02, 0.03], dtype=float)
    target = camera_point_to_robot_target(point)
    expected = np.array([0.02, -0.275, 0.17], dtype=float)
    np.testing.assert_allclose(target, expected, rtol=0, atol=1e-9)


def test_camera_point_transform_with_tcp_offset():
    point = np.array([0.01, -0.02, 0.03], dtype=float)
    robot_point = camera_point_to_robot_target(point)
    tcp_offset = [0.10, 0.0, 0.0, 0.0, 0.0, 0.0]
    corrected = apply_tcp_offset_correction(robot_point, tcp_offset, [0.0, 0.0, 0.0])
    expected = np.array([-0.08, -0.275, 0.17], dtype=float)
    np.testing.assert_allclose(corrected, expected, rtol=0, atol=1e-9)


def test_tcp_offset_correction():
    point = np.array([0.20, 0.10, 0.30], dtype=float)
    tcp_offset = [0.10, 0.0, 0.0, 0.0, 0.0, 0.0]
    corrected = apply_tcp_offset_correction(point, tcp_offset, [0.0, 0.0, 0.0])
    expected = np.array([0.10, 0.10, 0.30], dtype=float)
    np.testing.assert_allclose(corrected, expected, rtol=0, atol=1e-9)
