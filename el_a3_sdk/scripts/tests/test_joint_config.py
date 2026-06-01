#!/usr/bin/env python3
"""Phase A.2.4: 关节配置单元测试"""
import sys
import os
import math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from el_a3_sdk.protocol import (
    DEFAULT_JOINT_LIMITS, DEFAULT_JOINT_DIRECTIONS, DEFAULT_JOINT_OFFSETS,
    DEFAULT_MOTOR_TYPE_MAP, MOTOR_PARAMS, MotorType,
)


class TestJointLimits:
    def test_all_7_joints_defined(self):
        assert len(DEFAULT_JOINT_LIMITS) == 7
        for mid in range(1, 8):
            assert mid in DEFAULT_JOINT_LIMITS

    def test_lower_less_than_upper(self):
        for mid, (lo, hi) in DEFAULT_JOINT_LIMITS.items():
            assert lo < hi, f"Joint {mid}: lower({lo}) >= upper({hi})"

    def test_limits_within_motor_range(self):
        for mid, (lo, hi) in DEFAULT_JOINT_LIMITS.items():
            motor_type = DEFAULT_MOTOR_TYPE_MAP[mid]
            params = MOTOR_PARAMS[motor_type]
            assert lo >= params.p_min, \
                f"Joint {mid}: lower({lo}) < motor p_min({params.p_min})"
            assert hi <= params.p_max, \
                f"Joint {mid}: upper({hi}) > motor p_max({params.p_max})"

    def test_l1_limits(self):
        lo, hi = DEFAULT_JOINT_LIMITS[1]
        assert abs(lo - (-2.79253)) < 0.001
        assert abs(hi - 2.79253) < 0.001

    def test_l2_limits(self):
        lo, hi = DEFAULT_JOINT_LIMITS[2]
        assert abs(lo - 0.0) < 0.001
        assert abs(hi - 3.66519) < 0.001

    def test_l4_to_l7_symmetric(self):
        for mid in [4, 5, 6, 7]:
            lo, hi = DEFAULT_JOINT_LIMITS[mid]
            assert abs(lo + hi) < 0.001, \
                f"Joint {mid}: limits not symmetric ({lo}, {hi})"


class TestJointDirections:
    def test_all_7_joints_defined(self):
        assert len(DEFAULT_JOINT_DIRECTIONS) == 7
        for mid in range(1, 8):
            assert mid in DEFAULT_JOINT_DIRECTIONS

    def test_valid_direction_values(self):
        for mid, d in DEFAULT_JOINT_DIRECTIONS.items():
            assert d in (-1.0, 1.0), \
                f"Joint {mid}: invalid direction {d}"

    def test_specific_directions(self):
        assert DEFAULT_JOINT_DIRECTIONS[1] == -1.0
        assert DEFAULT_JOINT_DIRECTIONS[2] == 1.0
        assert DEFAULT_JOINT_DIRECTIONS[3] == -1.0
        assert DEFAULT_JOINT_DIRECTIONS[4] == 1.0
        assert DEFAULT_JOINT_DIRECTIONS[5] == -1.0
        assert DEFAULT_JOINT_DIRECTIONS[6] == 1.0
        assert DEFAULT_JOINT_DIRECTIONS[7] == 1.0


class TestJointOffsets:
    def test_all_7_joints_defined(self):
        assert len(DEFAULT_JOINT_OFFSETS) == 7

    def test_all_zero_by_default(self):
        for mid, offset in DEFAULT_JOINT_OFFSETS.items():
            assert offset == 0.0, \
                f"Joint {mid}: non-zero offset {offset}"


class TestMotorTypeConsistency:
    def test_rs00_joints_have_higher_torque(self):
        rs00_t = MOTOR_PARAMS[MotorType.RS00].t_max
        el05_t = MOTOR_PARAMS[MotorType.EL05].t_max
        rs05_t = MOTOR_PARAMS[MotorType.RS05].t_max
        assert rs00_t > el05_t > rs05_t

    def test_el05_and_rs05_same_velocity(self):
        el05_v = MOTOR_PARAMS[MotorType.EL05].v_max
        rs05_v = MOTOR_PARAMS[MotorType.RS05].v_max
        assert el05_v == rs05_v == 50.0

    def test_all_motors_same_position_range(self):
        for mt in MotorType:
            p = MOTOR_PARAMS[mt]
            assert p.p_min == -12.57
            assert p.p_max == 12.57

    def test_all_motors_same_kp_kd_range(self):
        for mt in MotorType:
            p = MOTOR_PARAMS[mt]
            assert p.kp_max == 500.0
            assert p.kd_max == 5.0
