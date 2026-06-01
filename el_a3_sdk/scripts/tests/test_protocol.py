#!/usr/bin/env python3
"""Phase A.2.1: SDK 协议参数单元测试"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from el_a3_sdk.protocol import (
    MotorType, MOTOR_PARAMS, DEFAULT_MOTOR_TYPE_MAP,
    DEFAULT_JOINT_LIMITS, DEFAULT_JOINT_DIRECTIONS, DEFAULT_JOINT_OFFSETS,
    ArmState, RunMode, MotorParams,
)


class TestMotorTypeEnum:
    def test_values(self):
        assert MotorType.RS00 == 0
        assert MotorType.EL05 == 1
        assert MotorType.RS05 == 2

    def test_all_types_in_params(self):
        for mt in MotorType:
            assert mt in MOTOR_PARAMS, f"{mt.name} not in MOTOR_PARAMS"


class TestMotorParamsRS05:
    def test_torque_range(self):
        p = MOTOR_PARAMS[MotorType.RS05]
        assert p.t_min == -5.5
        assert p.t_max == 5.5

    def test_velocity_range(self):
        p = MOTOR_PARAMS[MotorType.RS05]
        assert p.v_min == -50.0
        assert p.v_max == 50.0

    def test_position_range(self):
        p = MOTOR_PARAMS[MotorType.RS05]
        assert p.p_min == -12.57
        assert p.p_max == 12.57

    def test_kp_kd_range(self):
        p = MOTOR_PARAMS[MotorType.RS05]
        assert p.kp_min == 0.0
        assert p.kp_max == 500.0
        assert p.kd_min == 0.0
        assert p.kd_max == 5.0


class TestMotorParamsEL05:
    def test_torque_range(self):
        p = MOTOR_PARAMS[MotorType.EL05]
        assert p.t_min == -6.0
        assert p.t_max == 6.0

    def test_velocity_range(self):
        p = MOTOR_PARAMS[MotorType.EL05]
        assert p.v_min == -50.0
        assert p.v_max == 50.0


class TestMotorParamsRS00:
    def test_torque_range(self):
        p = MOTOR_PARAMS[MotorType.RS00]
        assert p.t_min == -14.0
        assert p.t_max == 14.0

    def test_velocity_range(self):
        p = MOTOR_PARAMS[MotorType.RS00]
        assert p.v_min == -33.0
        assert p.v_max == 33.0


class TestDefaultMotorTypeMap:
    def test_joints_1_3_are_rs00(self):
        for mid in [1, 2, 3]:
            assert DEFAULT_MOTOR_TYPE_MAP[mid] == MotorType.RS00, \
                f"Motor {mid} should be RS00"

    def test_joints_4_7_are_el05(self):
        for mid in [4, 5, 6, 7]:
            assert DEFAULT_MOTOR_TYPE_MAP[mid] == MotorType.EL05, \
                f"Motor {mid} should be EL05"

    def test_has_7_entries(self):
        assert len(DEFAULT_MOTOR_TYPE_MAP) == 7

    def test_custom_rs05_map(self):
        custom = {**DEFAULT_MOTOR_TYPE_MAP,
                  4: MotorType.RS05, 5: MotorType.RS05,
                  6: MotorType.RS05, 7: MotorType.RS05}
        for mid in [4, 5, 6, 7]:
            assert MOTOR_PARAMS[custom[mid]].t_max == 5.5


class TestArmState:
    def test_values(self):
        assert ArmState.DISCONNECTED == 0
        assert ArmState.IDLE == 1
        assert ArmState.ENABLED == 2
        assert ArmState.RUNNING == 3
        assert ArmState.ZERO_TORQUE == 4
        assert ArmState.ERROR == 5

    def test_all_states_defined(self):
        expected = {'DISCONNECTED', 'IDLE', 'ENABLED', 'RUNNING', 'ZERO_TORQUE', 'ERROR'}
        actual = {s.name for s in ArmState}
        assert expected == actual


class TestRunMode:
    def test_motion_control(self):
        assert RunMode.MOTION_CONTROL == 0

    def test_position_pp(self):
        assert RunMode.POSITION_PP == 1

    def test_velocity(self):
        assert RunMode.VELOCITY == 2

    def test_current(self):
        assert RunMode.CURRENT == 3
