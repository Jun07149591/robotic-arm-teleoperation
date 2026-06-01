#!/usr/bin/env python3
"""Phase A.2.3: ArmState 状态机测试"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from el_a3_sdk.protocol import ArmState


VALID_TRANSITIONS = {
    ArmState.DISCONNECTED: {ArmState.IDLE},
    ArmState.IDLE: {ArmState.ENABLED, ArmState.DISCONNECTED, ArmState.ERROR},
    ArmState.ENABLED: {ArmState.RUNNING, ArmState.ZERO_TORQUE,
                       ArmState.IDLE, ArmState.ERROR},
    ArmState.RUNNING: {ArmState.ENABLED, ArmState.ZERO_TORQUE,
                       ArmState.ERROR},
    ArmState.ZERO_TORQUE: {ArmState.ENABLED, ArmState.ERROR},
    ArmState.ERROR: {ArmState.IDLE, ArmState.DISCONNECTED},
}


def is_valid_transition(from_state: ArmState, to_state: ArmState) -> bool:
    return to_state in VALID_TRANSITIONS.get(from_state, set())


class TestArmStateValues:
    def test_disconnected(self):
        assert ArmState.DISCONNECTED == 0

    def test_idle(self):
        assert ArmState.IDLE == 1

    def test_enabled(self):
        assert ArmState.ENABLED == 2

    def test_running(self):
        assert ArmState.RUNNING == 3

    def test_zero_torque(self):
        assert ArmState.ZERO_TORQUE == 4

    def test_error(self):
        assert ArmState.ERROR == 5


class TestValidTransitions:
    def test_disconnected_to_idle(self):
        assert is_valid_transition(ArmState.DISCONNECTED, ArmState.IDLE)

    def test_idle_to_enabled(self):
        assert is_valid_transition(ArmState.IDLE, ArmState.ENABLED)

    def test_enabled_to_running(self):
        assert is_valid_transition(ArmState.ENABLED, ArmState.RUNNING)

    def test_enabled_to_zero_torque(self):
        assert is_valid_transition(ArmState.ENABLED, ArmState.ZERO_TORQUE)

    def test_zero_torque_to_enabled(self):
        assert is_valid_transition(ArmState.ZERO_TORQUE, ArmState.ENABLED)

    def test_enabled_to_idle(self):
        assert is_valid_transition(ArmState.ENABLED, ArmState.IDLE)

    def test_running_to_enabled(self):
        assert is_valid_transition(ArmState.RUNNING, ArmState.ENABLED)

    def test_any_to_error(self):
        for state in [ArmState.IDLE, ArmState.ENABLED,
                      ArmState.RUNNING, ArmState.ZERO_TORQUE]:
            assert is_valid_transition(state, ArmState.ERROR), \
                f"{state.name} -> ERROR should be valid"

    def test_error_to_idle(self):
        assert is_valid_transition(ArmState.ERROR, ArmState.IDLE)

    def test_error_to_disconnected(self):
        assert is_valid_transition(ArmState.ERROR, ArmState.DISCONNECTED)


class TestInvalidTransitions:
    def test_disconnected_to_enabled(self):
        assert not is_valid_transition(ArmState.DISCONNECTED, ArmState.ENABLED)

    def test_disconnected_to_running(self):
        assert not is_valid_transition(ArmState.DISCONNECTED, ArmState.RUNNING)

    def test_idle_to_running(self):
        assert not is_valid_transition(ArmState.IDLE, ArmState.RUNNING)

    def test_idle_to_zero_torque(self):
        assert not is_valid_transition(ArmState.IDLE, ArmState.ZERO_TORQUE)

    def test_zero_torque_to_running(self):
        assert not is_valid_transition(ArmState.ZERO_TORQUE, ArmState.RUNNING)

    def test_error_to_enabled(self):
        assert not is_valid_transition(ArmState.ERROR, ArmState.ENABLED)

    def test_error_to_running(self):
        assert not is_valid_transition(ArmState.ERROR, ArmState.RUNNING)

    def test_self_transition_disconnected(self):
        assert not is_valid_transition(ArmState.DISCONNECTED, ArmState.DISCONNECTED)


class TestFullLifecycle:
    """验证完整生命周期路径"""

    def test_normal_lifecycle(self):
        path = [
            ArmState.DISCONNECTED,
            ArmState.IDLE,
            ArmState.ENABLED,
            ArmState.RUNNING,
            ArmState.ENABLED,
            ArmState.ZERO_TORQUE,
            ArmState.ENABLED,
            ArmState.IDLE,
            ArmState.DISCONNECTED,
        ]
        for i in range(len(path) - 1):
            assert is_valid_transition(path[i], path[i + 1]), \
                f"Step {i}: {path[i].name} -> {path[i+1].name} should be valid"

    def test_error_recovery(self):
        path = [
            ArmState.DISCONNECTED,
            ArmState.IDLE,
            ArmState.ENABLED,
            ArmState.ERROR,
            ArmState.IDLE,
            ArmState.ENABLED,
        ]
        for i in range(len(path) - 1):
            assert is_valid_transition(path[i], path[i + 1]), \
                f"Step {i}: {path[i].name} -> {path[i+1].name} should be valid"
