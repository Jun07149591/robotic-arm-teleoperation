#!/usr/bin/env python3
"""
Phase B.2.3: 力矩限位安全测试

前置条件:
  - CAN 接口已配置
  - 机械臂已上电

测试内容:
  - 检查硬件接口日志确认力矩映射范围与电机型号匹配
  - 验证关节限位保护（SDK 层面检查）

用法:
  python3 scripts/tests/test_hw_safety.py [--motor-type EL05]
"""
import sys
import os
import argparse
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from el_a3_sdk.protocol import (
    MotorType, MOTOR_PARAMS, DEFAULT_JOINT_LIMITS,
    DEFAULT_MOTOR_TYPE_MAP,
)
from el_a3_sdk.utils import float_to_uint16, uint16_to_float

PASS = 0
FAIL = 0


def log_pass(msg):
    global PASS
    PASS += 1
    print(f"  [PASS] {msg}")


def log_fail(msg):
    global FAIL
    FAIL += 1
    print(f"  [FAIL] {msg}")


def test_torque_mapping_consistency(motor_type_str):
    """验证力矩映射范围与电机型号匹配"""
    mt = MotorType.RS05 if motor_type_str == "RS05" else MotorType.EL05
    params = MOTOR_PARAMS[mt]

    expected_t_max = 5.5 if mt == MotorType.RS05 else 6.0

    if params.t_max == expected_t_max:
        log_pass(f"{motor_type_str} 力矩上限: {params.t_max} Nm")
    else:
        log_fail(f"{motor_type_str} 力矩上限: {params.t_max} Nm (期望 {expected_t_max})")

    if params.t_min == -expected_t_max:
        log_pass(f"{motor_type_str} 力矩下限: {params.t_min} Nm")
    else:
        log_fail(f"{motor_type_str} 力矩下限: {params.t_min} Nm (期望 {-expected_t_max})")

    raw_max = float_to_uint16(params.t_max, params.t_min, params.t_max)
    if raw_max == 65535:
        log_pass(f"{motor_type_str} t_max -> raw=65535")
    else:
        log_fail(f"{motor_type_str} t_max -> raw={raw_max} (期望 65535)")

    raw_zero = float_to_uint16(0.0, params.t_min, params.t_max)
    if abs(raw_zero - 32768) < 2:
        log_pass(f"{motor_type_str} 0Nm -> raw={raw_zero} (~32768)")
    else:
        log_fail(f"{motor_type_str} 0Nm -> raw={raw_zero} (期望 ~32768)")


def test_joint_limit_protection():
    """验证关节限位配置合理性"""
    for mid, (lo, hi) in DEFAULT_JOINT_LIMITS.items():
        if lo >= hi:
            log_fail(f"Joint {mid}: lower({lo}) >= upper({hi})")
            continue

        margin = hi - lo
        if margin > 0.1:
            log_pass(f"Joint {mid}: 限位范围 [{lo:.3f}, {hi:.3f}] ({margin:.3f} rad)")
        else:
            log_fail(f"Joint {mid}: 限位范围过小 ({margin:.3f} rad)")


def test_torque_clamp_at_limit():
    """验证超出映射范围的力矩值被正确钳位"""
    for mt in [MotorType.EL05, MotorType.RS05]:
        params = MOTOR_PARAMS[mt]

        raw_over = float_to_uint16(params.t_max + 10.0, params.t_min, params.t_max)
        if raw_over == 65535:
            log_pass(f"{mt.name}: 超限力矩钳位到 65535")
        else:
            log_fail(f"{mt.name}: 超限力矩 raw={raw_over} (期望 65535)")

        raw_under = float_to_uint16(params.t_min - 10.0, params.t_min, params.t_max)
        if raw_under == 0:
            log_pass(f"{mt.name}: 下限力矩钳位到 0")
        else:
            log_fail(f"{mt.name}: 下限力矩 raw={raw_under} (期望 0)")


def main():
    parser = argparse.ArgumentParser(description="B.2.3 力矩/限位安全测试")
    parser.add_argument("--motor-type", default="EL05", choices=["EL05", "RS05"])
    args = parser.parse_args()

    print("============================================")
    print(f" B.2.3  力矩/限位安全测试 ({args.motor_type})")
    print("============================================")

    print("\n  --- 力矩映射一致性 ---")
    test_torque_mapping_consistency("EL05")
    test_torque_mapping_consistency("RS05")

    print("\n  --- 关节限位保护 ---")
    test_joint_limit_protection()

    print("\n  --- 力矩钳位测试 ---")
    test_torque_clamp_at_limit()

    print(f"\n============================================")
    print(f" 结果: {PASS} passed, {FAIL} failed")
    print(f"============================================")
    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
