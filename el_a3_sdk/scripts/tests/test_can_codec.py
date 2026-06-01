#!/usr/bin/env python3
"""Phase A.2.2: CAN 数据编解码单元测试"""
import sys
import os
import math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from el_a3_sdk.utils import float_to_uint16, uint16_to_float
from el_a3_sdk.protocol import MotorType, MOTOR_PARAMS


class TestFloatToUint16:
    def test_midpoint_maps_to_32768(self):
        raw = float_to_uint16(0.0, -10.0, 10.0)
        assert abs(raw - 32768) < 2

    def test_min_maps_to_0(self):
        raw = float_to_uint16(-10.0, -10.0, 10.0)
        assert raw == 0

    def test_max_maps_to_65535(self):
        raw = float_to_uint16(10.0, -10.0, 10.0)
        assert raw == 65535

    def test_clamps_below_min(self):
        raw = float_to_uint16(-20.0, -10.0, 10.0)
        assert raw == 0

    def test_clamps_above_max(self):
        raw = float_to_uint16(20.0, -10.0, 10.0)
        assert raw == 65535


class TestUint16ToFloat:
    def test_0_maps_to_min(self):
        val = uint16_to_float(0, -10.0, 10.0)
        assert abs(val - (-10.0)) < 0.001

    def test_65535_maps_to_max(self):
        val = uint16_to_float(65535, -10.0, 10.0)
        assert abs(val - 10.0) < 0.001

    def test_32768_maps_to_midpoint(self):
        val = uint16_to_float(32768, -10.0, 10.0)
        assert abs(val) < 0.01


class TestRoundTrip:
    """验证 float -> uint16 -> float 往返转换精度"""

    def _roundtrip(self, value, vmin, vmax, tolerance=0.01):
        raw = float_to_uint16(value, vmin, vmax)
        back = uint16_to_float(raw, vmin, vmax)
        assert abs(back - value) < tolerance, \
            f"roundtrip({value}, [{vmin},{vmax}]): raw={raw}, back={back}"

    def test_rs05_torque_zero(self):
        p = MOTOR_PARAMS[MotorType.RS05]
        self._roundtrip(0.0, p.t_min, p.t_max)

    def test_rs05_torque_max(self):
        p = MOTOR_PARAMS[MotorType.RS05]
        self._roundtrip(5.5, p.t_min, p.t_max)

    def test_rs05_torque_min(self):
        p = MOTOR_PARAMS[MotorType.RS05]
        self._roundtrip(-5.5, p.t_min, p.t_max)

    def test_rs05_torque_half(self):
        p = MOTOR_PARAMS[MotorType.RS05]
        self._roundtrip(2.75, p.t_min, p.t_max)

    def test_el05_torque_max(self):
        p = MOTOR_PARAMS[MotorType.EL05]
        self._roundtrip(6.0, p.t_min, p.t_max)

    def test_el05_torque_min(self):
        p = MOTOR_PARAMS[MotorType.EL05]
        self._roundtrip(-6.0, p.t_min, p.t_max)

    def test_rs00_torque_max(self):
        p = MOTOR_PARAMS[MotorType.RS00]
        self._roundtrip(14.0, p.t_min, p.t_max)

    def test_position_roundtrip(self):
        p = MOTOR_PARAMS[MotorType.RS05]
        self._roundtrip(3.14, p.p_min, p.p_max)

    def test_velocity_roundtrip(self):
        p = MOTOR_PARAMS[MotorType.RS05]
        self._roundtrip(25.0, p.v_min, p.v_max)


class TestMotorSpecificMapping:
    """验证不同电机类型使用正确的映射范围"""

    def test_rs05_torque_raw_values(self):
        p = MOTOR_PARAMS[MotorType.RS05]
        raw_zero = float_to_uint16(0.0, p.t_min, p.t_max)
        assert abs(raw_zero - 32768) < 2

        raw_max = float_to_uint16(5.5, p.t_min, p.t_max)
        assert raw_max == 65535

        raw_min = float_to_uint16(-5.5, p.t_min, p.t_max)
        assert raw_min == 0

    def test_el05_torque_raw_values(self):
        p = MOTOR_PARAMS[MotorType.EL05]
        raw_max = float_to_uint16(6.0, p.t_min, p.t_max)
        assert raw_max == 65535

        raw_min = float_to_uint16(-6.0, p.t_min, p.t_max)
        assert raw_min == 0

    def test_different_motors_different_raw_for_same_torque(self):
        """同一物理力矩在不同电机型号下映射到不同 raw 值"""
        torque = 3.0
        raw_rs05 = float_to_uint16(torque, -5.5, 5.5)
        raw_el05 = float_to_uint16(torque, -6.0, 6.0)
        raw_rs00 = float_to_uint16(torque, -14.0, 14.0)
        assert raw_rs05 > raw_el05 > raw_rs00
