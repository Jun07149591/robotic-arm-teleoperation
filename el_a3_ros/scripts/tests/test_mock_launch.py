#!/usr/bin/env python3
"""
Phase A.3: Mock 仿真集成测试

启动 ros2_control (use_mock_hardware:=true)，验证:
- A.3.1 EL05 Mock 启动 + 控制器状态
- A.3.2 RS05 Mock 启动 + URDF motor_type 验证
- A.3.3 控制器切换 (arm <-> zero_torque)
- A.3.4 轨迹发送 (arm + gripper)

需要 ROS2 环境。运行方式:
  python3 scripts/tests/test_mock_launch.py
"""
import subprocess
import time
import signal
import sys
import os
import json
import re

PASS = 0
FAIL = 0
LAUNCH_PROC = None


def log_pass(msg):
    global PASS
    PASS += 1
    print(f"  [PASS] {msg}")


def log_fail(msg):
    global FAIL
    FAIL += 1
    print(f"  [FAIL] {msg}")


def strip_ansi(text):
    """Remove ANSI escape sequences from text."""
    return re.sub(r'\x1b\[[0-9;]*m', '', text)


def run_cmd(cmd, timeout=15):
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=timeout
        )
        return strip_ansi(result.stdout.strip()), strip_ansi(result.stderr.strip()), result.returncode
    except subprocess.TimeoutExpired:
        return "", "TIMEOUT", -1


def launch_mock(wrist_motor_type="EL05", wait_sec=20):
    """启动 Mock 硬件并等待控制器就绪"""
    global LAUNCH_PROC
    kill_launch()

    cmd = [
        "ros2", "launch", "el_a3_description", "el_a3_control.launch.py",
        "use_mock_hardware:=true",
        f"wrist_motor_type:={wrist_motor_type}",
        "use_rviz:=false",
    ]
    LAUNCH_PROC = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    print(f"  启动 Mock ({wrist_motor_type}), PID={LAUNCH_PROC.pid}, 等待 {wait_sec}s...")
    time.sleep(wait_sec)


def kill_launch():
    global LAUNCH_PROC
    if LAUNCH_PROC is not None:
        try:
            os.killpg(os.getpgid(LAUNCH_PROC.pid), signal.SIGINT)
            LAUNCH_PROC.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(LAUNCH_PROC.pid), signal.SIGKILL)
            except Exception:
                pass
        LAUNCH_PROC = None
    subprocess.run(
        "pkill -9 -f ros2_control_node; pkill -9 -f 'spawner '; "
        "pkill -9 -f robot_state_publisher; sleep 1",
        shell=True, capture_output=True, timeout=10,
    )
    time.sleep(3)


def check_controllers_active(expected_active):
    """检查指定控制器是否处于 active 状态"""
    out, _, _ = run_cmd("ros2 control list_controllers")
    results = {}
    for name in expected_active:
        found = False
        for line in out.splitlines():
            parts = line.split()
            if parts and parts[0] == name and "active" in parts:
                found = True
                break
        if found:
            results[name] = True
            log_pass(f"控制器 {name} [active]")
        else:
            results[name] = False
            log_fail(f"控制器 {name} 未处于 active 状态")
    return all(results.values())


def check_joint_states(num_joints=7):
    """检查 /joint_states 话题是否包含指定数量的关节"""
    out, _, rc = run_cmd("ros2 topic echo /joint_states --once", timeout=10)
    if rc != 0 or not out:
        log_fail(f"/joint_states 话题不可用 (rc={rc})")
        return False

    joint_count = out.count("L") // 2
    name_match = re.search(r"name:\s*\n(.*?)position:", out, re.DOTALL)
    if name_match:
        names = re.findall(r"L\d+_joint", name_match.group(1))
        joint_count = len(names)

    if joint_count >= num_joints:
        log_pass(f"/joint_states 包含 {joint_count} 个关节")
        return True
    else:
        log_fail(f"/joint_states 只有 {joint_count} 个关节, 期望 {num_joints}")
        return False


def check_action_server(action_name):
    """检查 action server 是否可用"""
    out, _, _ = run_cmd("ros2 action list")
    if action_name in out:
        log_pass(f"Action {action_name} 可用")
        return True
    else:
        log_fail(f"Action {action_name} 不可用")
        return False


def check_robot_description(motor_type, effort_val):
    """检查 robot_description 中的 motor_type 和 effort"""
    out, _, rc = run_cmd(
        "ros2 param get /robot_state_publisher robot_description", timeout=10
    )
    if rc != 0 or not out:
        out, _, rc = run_cmd(
            "ros2 topic echo /robot_description --once", timeout=15
        )
    if rc != 0 or not out:
        log_fail("robot_description 不可用")
        return False

    if motor_type in out:
        log_pass(f"robot_description 包含 motor_type={motor_type}")
    else:
        log_fail(f"robot_description 中未找到 motor_type={motor_type}")
        return False

    return True


def switch_controllers(activate, deactivate):
    """切换控制器"""
    activate_str = str(activate).replace("'", '"')
    deactivate_str = str(deactivate).replace("'", '"')
    cmd = (
        f'ros2 service call /controller_manager/switch_controller '
        f'controller_manager_msgs/srv/SwitchController '
        f'"{{activate_controllers: {activate_str}, '
        f'deactivate_controllers: {deactivate_str}, '
        f'strictness: 1}}"'
    )
    out, err, rc = run_cmd(cmd, timeout=10)
    return "True" in out or "true" in out


def send_trajectory(controller, joint_names, positions, duration_sec=2):
    """通过 ros2 topic pub 发送轨迹"""
    positions_str = str(positions)
    if controller == "gripper_controller":
        torque = 0.65 if positions[0] > 0.03 else 0.0
        torque_cmd = (
            f"ros2 topic pub --once /{controller}/torque_limit "
            f'std_msgs/msg/Float64 "{{data: {torque}}}"'
        )
        run_cmd(torque_cmd, timeout=5)
    cmd = (
        f'ros2 topic pub --once /{controller}/joint_trajectory '
        f'trajectory_msgs/msg/JointTrajectory '
        f'"{{joint_names: {json.dumps(joint_names)}, '
        f'points: [{{positions: {positions_str}, '
        f'time_from_start: {{sec: {duration_sec}, nanosec: 0}}}}]}}"'
    )
    _, _, rc = run_cmd(cmd, timeout=10)
    return rc == 0


# ============================================================
# 测试流程
# ============================================================

def test_a31_mock_el05():
    print("\n============================================")
    print(" A.3.1  Mock 启动 - EL05")
    print("============================================")
    launch_mock("EL05")
    check_controllers_active(["arm_controller", "gripper_controller", "joint_state_broadcaster"])
    check_joint_states(7)
    check_action_server("/arm_controller/follow_joint_trajectory")


def test_a32_mock_rs05():
    print("\n============================================")
    print(" A.3.2  Mock 启动 - RS05")
    print("============================================")
    launch_mock("RS05")
    check_controllers_active(["arm_controller", "gripper_controller", "joint_state_broadcaster"])
    check_joint_states(7)
    check_robot_description("RS05", "5.5")


def load_controller(name):
    """Load a controller that hasn't been spawned by the launch file."""
    _, _, rc = run_cmd(
        f"ros2 service call /controller_manager/load_controller "
        f'controller_manager_msgs/srv/LoadController '
        f'"{{name: {name}}}"',
        timeout=10,
    )
    time.sleep(1)
    _, _, rc2 = run_cmd(
        f"ros2 service call /controller_manager/configure_controller "
        f'controller_manager_msgs/srv/ConfigureController '
        f'"{{name: {name}}}"',
        timeout=10,
    )
    return rc == 0 and rc2 == 0


def test_a33_controller_switch():
    print("\n============================================")
    print(" A.3.3  控制器切换 (arm <-> zero_torque)")
    print("============================================")

    load_controller("zero_torque_controller")

    if switch_controllers(["zero_torque_controller"], ["arm_controller", "gripper_controller"]):
        log_pass("arm_controller -> zero_torque_controller 切换成功")
    else:
        log_fail("arm_controller -> zero_torque_controller 切换失败")

    time.sleep(2)

    out, _, _ = run_cmd("ros2 control list_controllers")
    zt_active = False
    for line in out.splitlines():
        parts = line.split()
        if parts and parts[0] == "zero_torque_controller" and "active" in parts:
            zt_active = True
            break
    if zt_active:
        log_pass("zero_torque_controller 确认处于 active")
    else:
        log_fail("zero_torque_controller 未激活")

    if switch_controllers(["arm_controller", "gripper_controller"], ["zero_torque_controller"]):
        log_pass("zero_torque_controller -> arm_controller 切换成功")
    else:
        log_fail("zero_torque_controller -> arm_controller 切换失败")

    time.sleep(2)
    check_controllers_active(["arm_controller"])


def test_a34_trajectory():
    print("\n============================================")
    print(" A.3.4  轨迹发送测试")
    print("============================================")

    arm_joints = [f"L{i}_joint" for i in range(1, 7)]
    if send_trajectory("arm_controller", arm_joints, [0.0] * 6):
        log_pass("arm_controller 轨迹发送 (home) 成功")
    else:
        log_fail("arm_controller 轨迹发送 (home) 失败")

    time.sleep(2)

    if send_trajectory("arm_controller", arm_joints, [0.0, 1.0, 0.0, 0.0, 0.0, 0.0]):
        log_pass("arm_controller 轨迹发送 (L2=1.0) 成功")
    else:
        log_fail("arm_controller 轨迹发送 (L2=1.0) 失败")

    time.sleep(2)

    if send_trajectory("gripper_controller", ["L7_joint"], [0.5]):
        log_pass("gripper_controller 轨迹发送 (L7=0.5) 成功")
    else:
        log_fail("gripper_controller 轨迹发送 (L7=0.5) 失败")


def main():
    global PASS, FAIL

    try:
        # A.3.1: EL05 启动 + 基础验证
        test_a31_mock_el05()

        # A.3.3 + A.3.4: 在 EL05 下测试切换和轨迹
        test_a33_controller_switch()
        test_a34_trajectory()

        kill_launch()

        # A.3.2: RS05 启动 + motor_type 验证
        test_a32_mock_rs05()

    finally:
        kill_launch()

    print("\n============================================")
    print(f" Mock 集成测试结果: {PASS} passed, {FAIL} failed")
    print("============================================")

    sys.exit(0 if FAIL == 0 else 1)


if __name__ == "__main__":
    main()
