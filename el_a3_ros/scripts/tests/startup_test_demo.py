#!/usr/bin/env python3
"""
EL-A3 ROS2 启动测试 Demo

端到端验证流程: 系统启动 -> 控制器状态 -> 关节读取 -> 运动测试 -> 夹爪 -> 控制器切换

运行模式:
  --mode mock     自动启动 mock 仿真 (默认, 无需硬件)
  --mode real     启动真实硬件控制
  --mode connect  连接已运行的系统

可选:
  --test-zero-torque   测试 zero_torque 控制器切换
  --can-interface      CAN 接口名 (real 模式, 默认 can0)
  --wrist-motor-type   腕部电机型号 (EL05 或 RS05, 默认 EL05)
  --wait-sec           启动后等待秒数 (默认 25)

用法:
  python3 scripts/tests/startup_test_demo.py
  python3 scripts/tests/startup_test_demo.py --mode real --can-interface can0
  python3 scripts/tests/startup_test_demo.py --mode connect --test-zero-torque
"""

import argparse
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

# ---------------------------------------------------------------------------
# 全局状态
# ---------------------------------------------------------------------------
PASS_COUNT = 0
FAIL_COUNT = 0
LAUNCH_PROC = None

ARM_JOINTS = [f"L{i}_joint" for i in range(1, 7)]
ALL_JOINTS = [f"L{i}_joint" for i in range(1, 8)]
POSITION_TOLERANCE = 0.05  # rad

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_ROS_WS_DIR = os.path.abspath(os.path.join(_SCRIPT_DIR, os.pardir, os.pardir))


def _ros_env() -> dict:
    """构建包含 ROS workspace 环境变量的 env dict."""
    setup_bash = os.path.join(_ROS_WS_DIR, "install", "setup.bash")
    if not os.path.isfile(setup_bash):
        return dict(os.environ)
    result = subprocess.run(
        f'source /opt/ros/humble/setup.bash && source "{setup_bash}" && env',
        shell=True, capture_output=True, text=True, executable="/bin/bash",
    )
    env = {}
    for line in result.stdout.splitlines():
        k, _, v = line.partition("=")
        if k:
            env[k] = v
    return env if env else dict(os.environ)


def log_pass(msg: str):
    global PASS_COUNT
    PASS_COUNT += 1
    print(f"  [PASS] {msg}")


def log_fail(msg: str):
    global FAIL_COUNT
    FAIL_COUNT += 1
    print(f"  [FAIL] {msg}")


def strip_ansi(text: str) -> str:
    return re.sub(r'\x1b\[[0-9;]*m', '', text)


def run_cmd(cmd: str, timeout: int = 15):
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=timeout,
            env=_ros_env(), executable="/bin/bash",
        )
        return strip_ansi(result.stdout.strip()), strip_ansi(result.stderr.strip()), result.returncode
    except subprocess.TimeoutExpired:
        return "", "TIMEOUT", -1


# ---------------------------------------------------------------------------
# Launch 进程管理
# ---------------------------------------------------------------------------
def launch_system(mode: str, can_interface: str, wrist_motor_type: str, wait_sec: int):
    global LAUNCH_PROC
    kill_launch()

    if mode == "mock":
        cmd = [
            "ros2", "launch", "el_a3_description", "el_a3_control.launch.py",
            "use_mock_hardware:=true",
            f"wrist_motor_type:={wrist_motor_type}",
            "use_rviz:=false",
        ]
    elif mode == "real":
        cmd = [
            "ros2", "launch", "el_a3_description", "el_a3_control.launch.py",
            "use_mock_hardware:=false",
            f"can_interface:={can_interface}",
            f"wrist_motor_type:={wrist_motor_type}",
            "use_rviz:=false",
        ]
    else:
        return

    LAUNCH_PROC = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid, env=_ros_env(),
    )
    print(f"  启动 launch ({mode}), PID={LAUNCH_PROC.pid}, 等待 {wait_sec}s...")
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
    time.sleep(2)


# ---------------------------------------------------------------------------
# 测试节点
# ---------------------------------------------------------------------------
class StartupTestDemo(Node):

    def __init__(self, mode: str):
        super().__init__('startup_test_demo')
        self._mode = mode
        self._latest_joint_state = None

        self._arm_action = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        self._gripper_action = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, qos
        )

    def _joint_state_cb(self, msg: JointState):
        self._latest_joint_state = msg

    def _spin_for(self, seconds: float):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    # ------------------------------------------------------------------
    # Phase 1: 系统启动与健康检查
    # ------------------------------------------------------------------
    def phase1_health_check(self):
        print("\nPhase 1: 系统启动与健康检查")
        print("-" * 44)

        out, _, _ = run_cmd("ros2 node list", timeout=10)
        nodes = out.splitlines()

        cm_found = any("controller_manager" in n for n in nodes)
        if cm_found:
            log_pass("controller_manager 节点在线")
        else:
            log_fail("controller_manager 节点不在线")

        rsp_found = any("robot_state_publisher" in n for n in nodes)
        if rsp_found:
            log_pass("robot_state_publisher 节点在线")
        else:
            log_fail("robot_state_publisher 节点不在线")

        out, _, rc = run_cmd(
            "ros2 param get /robot_state_publisher robot_description", timeout=10
        )
        if rc == 0 and out:
            log_pass("robot_description 参数可读")
        else:
            log_fail("robot_description 参数不可读")

        out, _, _ = run_cmd("ros2 topic list", timeout=10)
        if "/tf" in out.splitlines():
            log_pass("/tf topic 存在")
        else:
            log_fail("/tf topic 不存在")

    # ------------------------------------------------------------------
    # Phase 2: 控制器状态验证
    # ------------------------------------------------------------------
    def phase2_controller_status(self):
        print("\nPhase 2: 控制器状态验证")
        print("-" * 44)

        out, _, _ = run_cmd("ros2 control list_controllers", timeout=10)

        for name in ["joint_state_broadcaster", "arm_controller", "gripper_controller"]:
            found_active = False
            for line in out.splitlines():
                parts = line.split()
                if parts and parts[0] == name and "active" in parts:
                    found_active = True
                    break
            if found_active:
                log_pass(f"{name} [active]")
            else:
                log_fail(f"{name} 未处于 active 状态")

        out_hw, _, _ = run_cmd("ros2 control list_hardware_interfaces", timeout=10)
        claimed_count = out_hw.count("[claimed]") if out_hw else 0
        if claimed_count > 0:
            log_pass(f"硬件接口已 claimed ({claimed_count} 个)")
        else:
            log_fail("无硬件接口被 claimed")

        if self._arm_action.wait_for_server(timeout_sec=5.0):
            log_pass("Action /arm_controller/follow_joint_trajectory 可用")
        else:
            log_fail("Action /arm_controller/follow_joint_trajectory 不可用")

        if self._gripper_action.wait_for_server(timeout_sec=5.0):
            log_pass("Action /gripper_controller/follow_joint_trajectory 可用")
        else:
            log_fail("Action /gripper_controller/follow_joint_trajectory 不可用")

    # ------------------------------------------------------------------
    # Phase 3: 关节状态读取
    # ------------------------------------------------------------------
    def phase3_joint_state_read(self):
        print("\nPhase 3: 关节状态读取")
        print("-" * 44)

        self._latest_joint_state = None
        self._spin_for(2.0)

        if self._latest_joint_state is None:
            log_fail("/joint_states 未收到数据")
            return

        msg = self._latest_joint_state
        received_joints = list(msg.name)

        missing = [j for j in ALL_JOINTS if j not in received_joints]
        if not missing:
            log_pass(f"/joint_states 包含全部 {len(ALL_JOINTS)} 个关节")
        else:
            log_fail(f"/joint_states 缺少关节: {missing}")

        pos_map = dict(zip(msg.name, msg.position))
        pos_str = " ".join(f"{j.replace('_joint','')}={pos_map.get(j, 0.0):.3f}" for j in ALL_JOINTS)
        print(f"  当前位置: {pos_str}")

    # ------------------------------------------------------------------
    # Phase 4: 基础运动测试
    # ------------------------------------------------------------------
    def phase4_arm_motion(self):
        print("\nPhase 4: 基础运动测试")
        print("-" * 44)

        test_moves = [
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "归零位", 5),
            ([0.0, 0.3, 0.0, 0.0, 0.0, 0.0], "L2 单关节 +0.3", 5),
            ([0.15, 0.3, -0.15, 0.1, 0.1, 0.1], "多关节联合", 6),
            ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "归零位", 6),
        ]

        for positions, desc, dur in test_moves:
            ok = self._send_arm_trajectory(positions, duration_sec=dur)
            if ok:
                log_pass(f"{desc} 完成")
            else:
                log_fail(f"{desc} 失败")
                continue

            if self._mode == "mock":
                self._spin_for(1.0)
                if self._latest_joint_state is not None:
                    pos_map = dict(zip(
                        self._latest_joint_state.name,
                        self._latest_joint_state.position
                    ))
                    max_err = max(
                        abs(pos_map.get(ARM_JOINTS[i], 0.0) - positions[i])
                        for i in range(len(ARM_JOINTS))
                    )
                    if max_err < POSITION_TOLERANCE:
                        log_pass(f"  位置跟踪 OK (最大误差={max_err:.4f})")
                    else:
                        log_fail(f"  位置跟踪误差过大 ({max_err:.4f} > {POSITION_TOLERANCE})")

    def _send_arm_trajectory(self, positions: list, duration_sec: int = 3) -> bool:
        if not self._arm_action.server_is_ready():
            if not self._arm_action.wait_for_server(timeout_sec=5.0):
                self.get_logger().warn("arm action server 不可用")
                return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)

        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.velocities = [0.0] * len(positions)
        pt.time_from_start = Duration(sec=duration_sec, nanosec=0)
        goal.trajectory.points = [pt]

        future = self._arm_action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            self.get_logger().warn("send_goal 无响应")
            return False
        if not future.result().accepted:
            self.get_logger().warn("goal 被拒绝")
            return False

        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 10)
        if result_future.result() is None:
            self._spin_for(float(duration_sec))
            return True
        res = result_future.result().result
        if res.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().warn(f"轨迹执行失败, error_code={res.error_code}, error_string={res.error_string}")
            return False
        return True

    # ------------------------------------------------------------------
    # Phase 5: 夹爪测试
    # ------------------------------------------------------------------
    def phase5_gripper(self):
        print("\nPhase 5: 夹爪测试")
        print("-" * 44)

        for target, desc in [(0.3, "半开"), (0.0, "闭合")]:
            ok = self._send_gripper_trajectory(target, duration_sec=4)
            if ok:
                log_pass(f"L7 -> {target} ({desc}) 完成")
            else:
                log_fail(f"L7 -> {target} ({desc}) 失败")

    def _send_gripper_trajectory(self, position: float, duration_sec: int = 2) -> bool:
        if not self._gripper_action.server_is_ready():
            if not self._gripper_action.wait_for_server(timeout_sec=5.0):
                self.get_logger().warn("gripper action server 不可用")
                return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["L7_joint"]

        pt = JointTrajectoryPoint()
        pt.positions = [position]
        pt.velocities = [0.0]
        pt.time_from_start = Duration(sec=duration_sec, nanosec=0)
        goal.trajectory.points = [pt]

        future = self._gripper_action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            self.get_logger().warn("gripper send_goal 无响应")
            return False
        if not future.result().accepted:
            self.get_logger().warn("gripper goal 被拒绝")
            return False

        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 10)
        if result_future.result() is None:
            self._spin_for(float(duration_sec))
            return True
        res = result_future.result().result
        if res.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().warn(f"gripper 执行失败, error_code={res.error_code}, error_string={res.error_string}")
            return False
        return True

    # ------------------------------------------------------------------
    # Phase 6: 控制器切换测试 (zero_torque)
    # ------------------------------------------------------------------
    def phase6_controller_switch(self):
        print("\nPhase 6: 控制器切换测试 (zero_torque)")
        print("-" * 44)

        if self._mode == "mock":
            self._load_controller("zero_torque_controller")

        ok = self._switch_controllers(
            activate=["zero_torque_controller"],
            deactivate=["arm_controller", "gripper_controller"],
        )
        if ok:
            log_pass("arm_controller -> zero_torque_controller 切换成功")
        else:
            log_fail("arm_controller -> zero_torque_controller 切换失败")

        time.sleep(2)

        out, _, _ = run_cmd("ros2 control list_controllers")
        zt_active = any(
            "zero_torque_controller" in line and "active" in line
            for line in out.splitlines()
        )
        if zt_active:
            log_pass("zero_torque_controller [active]")
        else:
            log_fail("zero_torque_controller 未激活")

        ok = self._switch_controllers(
            activate=["arm_controller", "gripper_controller"],
            deactivate=["zero_torque_controller"],
        )
        if ok:
            log_pass("zero_torque_controller -> arm_controller 恢复成功")
        else:
            log_fail("zero_torque_controller -> arm_controller 恢复失败")

        time.sleep(2)

        out, _, _ = run_cmd("ros2 control list_controllers")
        arm_active = any(
            "arm_controller" in line and "active" in line
            for line in out.splitlines()
        )
        if arm_active:
            log_pass("arm_controller [active] (已恢复)")
        else:
            log_fail("arm_controller 未恢复到 active")

    @staticmethod
    def _load_controller(name: str):
        run_cmd(
            f'ros2 service call /controller_manager/load_controller '
            f'controller_manager_msgs/srv/LoadController '
            f'"{{name: {name}}}"',
            timeout=10,
        )
        time.sleep(1)
        run_cmd(
            f'ros2 service call /controller_manager/configure_controller '
            f'controller_manager_msgs/srv/ConfigureController '
            f'"{{name: {name}}}"',
            timeout=10,
        )
        time.sleep(1)

    @staticmethod
    def _switch_controllers(activate: list, deactivate: list) -> bool:
        activate_str = str(activate).replace("'", '"')
        deactivate_str = str(deactivate).replace("'", '"')
        cmd = (
            f'ros2 service call /controller_manager/switch_controller '
            f'controller_manager_msgs/srv/SwitchController '
            f'"{{activate_controllers: {activate_str}, '
            f'deactivate_controllers: {deactivate_str}, '
            f'strictness: 1}}"'
        )
        out, _, _ = run_cmd(cmd, timeout=10)
        return "True" in out or "true" in out

    # ------------------------------------------------------------------
    # 统一执行入口
    # ------------------------------------------------------------------
    def run_all_phases(self, test_zero_torque: bool = False):
        self.phase1_health_check()
        self.phase2_controller_status()
        self.phase3_joint_state_read()
        self.phase4_arm_motion()
        self.phase5_gripper()
        if test_zero_torque:
            self.phase6_controller_switch()


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def _inject_ros_env():
    """将 ROS workspace 环境注入当前进程，确保 rclpy 能发现所有包."""
    env = _ros_env()
    for key in ("AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH",
                "LD_LIBRARY_PATH", "PYTHONPATH", "PATH"):
        if key in env:
            os.environ[key] = env[key]


def main():
    _inject_ros_env()

    parser = argparse.ArgumentParser(
        description="EL-A3 ROS2 启动测试 Demo",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--mode", choices=["mock", "real", "connect"], default="mock",
        help="运行模式: mock(仿真) / real(真实硬件) / connect(连接已运行系统)",
    )
    parser.add_argument("--can-interface", default="can0", help="CAN 接口名 (real 模式)")
    parser.add_argument("--wrist-motor-type", default="EL05", help="腕部电机型号 (EL05/RS05)")
    parser.add_argument("--wait-sec", type=int, default=25, help="launch 启动后等待秒数")
    parser.add_argument("--test-zero-torque", action="store_true", help="测试零力矩控制器切换")
    args = parser.parse_args()

    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print("=" * 44)
    print(f" EL-A3 ROS2 启动测试 Demo")
    print(f" 模式: {args.mode} | 时间: {now}")
    print("=" * 44)

    original_sigint = signal.getsignal(signal.SIGINT)

    def _cleanup_handler(signum, frame):
        print("\n\n  收到中断信号，正在清理...")
        kill_launch()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        sys.exit(1)

    signal.signal(signal.SIGINT, _cleanup_handler)
    signal.signal(signal.SIGTERM, _cleanup_handler)

    try:
        if args.mode in ("mock", "real"):
            launch_system(args.mode, args.can_interface, args.wrist_motor_type, args.wait_sec)

        rclpy.init()
        node = StartupTestDemo(mode=args.mode)
        node.run_all_phases(test_zero_torque=args.test_zero_torque)
        node.destroy_node()
        rclpy.shutdown()

    finally:
        if args.mode in ("mock", "real"):
            kill_launch()

    print("\n" + "=" * 44)
    print(f" 结果: {PASS_COUNT} passed, {FAIL_COUNT} failed")
    print("=" * 44)

    sys.exit(0 if FAIL_COUNT == 0 else 1)


if __name__ == "__main__":
    main()
