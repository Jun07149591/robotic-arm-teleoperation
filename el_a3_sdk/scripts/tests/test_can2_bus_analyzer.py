#!/usr/bin/env python3
"""
CAN2 总线 Type1/Type2 帧分析器

被动监听 CAN 总线，解码 Type 1（运控指令）和 Type 2（电机反馈）帧，
分析电机 ID 1-7 的收发信息，并实时发布为 ROS2 调试话题供 Foxglove 可视化。

前置条件:
  - CAN 接口已配置: sudo ./scripts/setup_can.sh can2
  - ROS2 环境已 source（除非使用 --no-ros）
  - (可选) foxglove_bridge 运行中

用法:
  python3 scripts/tests/test_can2_bus_analyzer.py [--can can2] [--motor-type EL05]
  python3 scripts/tests/test_can2_bus_analyzer.py --no-ros   # 纯终端模式
"""

import sys
import os
import socket
import struct
import time
import threading
import argparse
import signal
import json
from collections import defaultdict

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from el_a3_sdk.protocol import CommType, MotorType, MOTOR_PARAMS
from el_a3_sdk.utils import uint16_to_float

CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = 16
CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF

COMM_TYPE_NAMES = {int(ct): ct.name for ct in CommType}
MODE_STATE_NAMES = {0: "Reset", 1: "Cali", 2: "Motor"}
MOTOR_IDS = range(1, 8)


class MotorStats:
    """每个电机的收发统计"""
    __slots__ = [
        't1_count', 't1_window', 't1_hz',
        't1_pos', 't1_vel', 't1_torque', 't1_kp', 't1_kd', 't1_ts',
        't2_count', 't2_window', 't2_hz',
        't2_pos', 't2_vel', 't2_torque', 't2_temp',
        't2_mode', 't2_fault', 't2_ts',
    ]

    def __init__(self):
        self.t1_count = 0
        self.t1_window = 0
        self.t1_hz = 0.0
        self.t1_pos = 0.0
        self.t1_vel = 0.0
        self.t1_torque = 0.0
        self.t1_kp = 0.0
        self.t1_kd = 0.0
        self.t1_ts = 0.0

        self.t2_count = 0
        self.t2_window = 0
        self.t2_hz = 0.0
        self.t2_pos = 0.0
        self.t2_vel = 0.0
        self.t2_torque = 0.0
        self.t2_temp = 0.0
        self.t2_mode = 0
        self.t2_fault = 0
        self.t2_ts = 0.0


class Can2BusAnalyzer:
    def __init__(self, can_interface: str, motor_type_map: dict,
                 use_ros: bool, verbose: bool, show_raw: bool):
        self.can_interface = can_interface
        self.motor_type_map = motor_type_map
        self.use_ros = use_ros
        self.verbose = verbose
        self.show_raw = show_raw

        self._socket = None
        self._running = False
        self._recv_thread = None
        self._stats = {mid: MotorStats() for mid in MOTOR_IDS}
        self._lock = threading.Lock()
        self._start_time = 0.0
        self._total_frames = 0
        self._other_type_counts = defaultdict(int)

        self._node = None
        self._pub_t1_cmd = None
        self._pub_t1_gains = None
        self._pub_t2_fb = None
        self._pub_t2_temp = None
        self._pub_stats = None
        self._JointState = None
        self._String = None

        if self.use_ros:
            self._init_ros()

    # ========== ROS2 初始化 ==========

    def _init_ros(self):
        try:
            import rclpy
            from sensor_msgs.msg import JointState
            from std_msgs.msg import String
        except ImportError:
            print("[WARN] rclpy 未安装或未 source ROS2 环境，回退到纯终端模式")
            self.use_ros = False
            return

        rclpy.init()
        self._node = rclpy.create_node('can2_bus_analyzer')
        self._JointState = JointState
        self._String = String

        qos_depth = 50
        self._pub_t1_cmd = self._node.create_publisher(
            JointState, '/debug/can2/type1_command', qos_depth)
        self._pub_t1_gains = self._node.create_publisher(
            JointState, '/debug/can2/type1_gains', qos_depth)
        self._pub_t2_fb = self._node.create_publisher(
            JointState, '/debug/can2/type2_feedback', qos_depth)
        self._pub_t2_temp = self._node.create_publisher(
            JointState, '/debug/can2/type2_temperature', qos_depth)
        self._pub_stats = self._node.create_publisher(
            String, '/debug/can2/bus_stats', 10)

        self._node.create_timer(1.0, self._ros_publish_stats)
        self._node.get_logger().info(
            f"调试话题已创建: /debug/can2/type1_command, type1_gains, "
            f"type2_feedback, type2_temperature, bus_stats")

    def _make_stamp(self, t: float):
        """系统时间 -> ROS2 Time"""
        from builtin_interfaces.msg import Time as RosTime
        sec = int(t)
        nanosec = int((t - sec) * 1e9)
        return RosTime(sec=sec, nanosec=nanosec)

    # ========== CAN 连接与收发 ==========

    def connect(self) -> bool:
        try:
            self._socket = socket.socket(
                socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self._socket.bind((self.can_interface,))
            self._socket.settimeout(0.1)
            print(f"[OK] CAN 接口 {self.can_interface} 已连接")
            return True
        except Exception as e:
            print(f"[ERROR] CAN 接口 {self.can_interface} 连接失败: {e}")
            return False

    def start(self):
        self._running = True
        self._start_time = time.time()
        self._recv_thread = threading.Thread(
            target=self._receive_loop, daemon=True, name="can2_recv")
        self._recv_thread.start()
        print(f"[OK] 接收线程已启动，被动监听 {self.can_interface} ...")

    def stop(self):
        self._running = False
        if self._recv_thread:
            self._recv_thread.join(timeout=2.0)
        if self._socket:
            self._socket.close()
            self._socket = None
        if self.use_ros:
            import rclpy
            if self._node:
                self._node.destroy_node()
            rclpy.try_shutdown()

    def spin_ros(self):
        if not self.use_ros or not self._node:
            return
        import rclpy
        threading.Thread(
            target=lambda: rclpy.spin(self._node),
            daemon=True, name="ros_spin"
        ).start()

    # ========== 接收与解析 ==========

    def _receive_loop(self):
        while self._running:
            try:
                raw = self._socket.recv(CAN_FRAME_SIZE)
                recv_time = time.time()
                if len(raw) == CAN_FRAME_SIZE:
                    self._total_frames += 1
                    self._parse_frame(raw, recv_time)
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    print(f"[WARN] 接收错误: {e}")

    def _parse_frame(self, raw: bytes, recv_time: float):
        can_id_raw = struct.unpack_from("=I", raw, 0)[0]
        data = raw[8:16]

        if not (can_id_raw & CAN_EFF_FLAG):
            return

        can_id = can_id_raw & CAN_EFF_MASK
        comm_type = (can_id >> 24) & 0x1F

        if self.show_raw:
            ct_name = COMM_TYPE_NAMES.get(comm_type, f"T{comm_type}")
            print(f"  [RAW] ID=0x{can_id:08X} type={ct_name} data={data.hex(' ')}")

        if comm_type == CommType.MOTION_CONTROL:
            self._handle_type1(can_id, data, recv_time)
        elif comm_type == CommType.FEEDBACK:
            self._handle_type2(can_id, data, recv_time)
        else:
            self._other_type_counts[comm_type] += 1

    def _handle_type1(self, can_id: int, data: bytes, recv_time: float):
        """Type 1 运控指令: target_id 在 bit0-7, torque_ff 在 bit8-23"""
        motor_id = can_id & 0xFF
        if motor_id not in self._stats:
            return

        torque_ff_raw = (can_id >> 8) & 0xFFFF
        pos_raw, vel_raw, kp_raw, kd_raw = struct.unpack(">HHHH", data)

        mt = self.motor_type_map.get(motor_id, MotorType.RS00)
        p = MOTOR_PARAMS[mt]

        pos = uint16_to_float(pos_raw, p.p_min, p.p_max)
        vel = uint16_to_float(vel_raw, -50.0, 50.0)
        torque = uint16_to_float(torque_ff_raw, p.t_min, p.t_max)
        kp = uint16_to_float(kp_raw, p.kp_min, p.kp_max)
        kd = uint16_to_float(kd_raw, p.kd_min, p.kd_max)

        with self._lock:
            s = self._stats[motor_id]
            s.t1_count += 1
            s.t1_window += 1
            s.t1_pos = pos
            s.t1_vel = vel
            s.t1_torque = torque
            s.t1_kp = kp
            s.t1_kd = kd
            s.t1_ts = recv_time

        if self.verbose:
            print(f"  [T1] M{motor_id} pos={pos:.3f} vel={vel:.3f} "
                  f"torque={torque:.3f} kp={kp:.1f} kd={kd:.2f} "
                  f"t={recv_time:.6f}")

        if self.use_ros:
            self._ros_publish_type1(recv_time)

    def _handle_type2(self, can_id: int, data: bytes, recv_time: float):
        """Type 2 电机反馈: motor_id 在 bit8-15, mode/fault 在 bit16-23"""
        motor_id = (can_id >> 8) & 0xFF
        if motor_id not in self._stats:
            return

        mode_state = (can_id >> 22) & 0x03
        fault_code = (can_id >> 16) & 0x3F

        pos_raw, vel_raw, torque_raw, temp_raw = struct.unpack(">HHHH", data)

        mt = self.motor_type_map.get(motor_id, MotorType.RS00)
        p = MOTOR_PARAMS[mt]

        pos = uint16_to_float(pos_raw, p.p_min, p.p_max)
        vel = uint16_to_float(vel_raw, p.v_min, p.v_max)
        torque = uint16_to_float(torque_raw, p.t_min, p.t_max)
        temp = temp_raw / 10.0

        with self._lock:
            s = self._stats[motor_id]
            s.t2_count += 1
            s.t2_window += 1
            s.t2_pos = pos
            s.t2_vel = vel
            s.t2_torque = torque
            s.t2_temp = temp
            s.t2_mode = mode_state
            s.t2_fault = fault_code
            s.t2_ts = recv_time

        if self.verbose:
            mn = MODE_STATE_NAMES.get(mode_state, f"?{mode_state}")
            print(f"  [T2] M{motor_id} pos={pos:.3f} vel={vel:.3f} "
                  f"torque={torque:.3f} temp={temp:.1f}°C "
                  f"mode={mn} fault={fault_code} t={recv_time:.6f}")

        if self.use_ros:
            self._ros_publish_type2(recv_time)

    # ========== ROS2 实时发布（帧驱动） ==========

    def _ros_publish_type1(self, recv_time: float):
        stamp = self._make_stamp(recv_time)
        names = [f"motor_{mid}" for mid in MOTOR_IDS]

        with self._lock:
            cmd = self._JointState()
            cmd.header.stamp = stamp
            cmd.name = names
            cmd.position = [self._stats[mid].t1_pos for mid in MOTOR_IDS]
            cmd.velocity = [self._stats[mid].t1_vel for mid in MOTOR_IDS]
            cmd.effort = [self._stats[mid].t1_torque for mid in MOTOR_IDS]

            gains = self._JointState()
            gains.header.stamp = stamp
            gains.name = names
            gains.position = [self._stats[mid].t1_kp for mid in MOTOR_IDS]
            gains.velocity = [self._stats[mid].t1_kd for mid in MOTOR_IDS]

        self._pub_t1_cmd.publish(cmd)
        self._pub_t1_gains.publish(gains)

    def _ros_publish_type2(self, recv_time: float):
        stamp = self._make_stamp(recv_time)
        names = [f"motor_{mid}" for mid in MOTOR_IDS]

        with self._lock:
            fb = self._JointState()
            fb.header.stamp = stamp
            fb.name = names
            fb.position = [self._stats[mid].t2_pos for mid in MOTOR_IDS]
            fb.velocity = [self._stats[mid].t2_vel for mid in MOTOR_IDS]
            fb.effort = [self._stats[mid].t2_torque for mid in MOTOR_IDS]

            tm = self._JointState()
            tm.header.stamp = stamp
            tm.name = names
            tm.effort = [self._stats[mid].t2_temp for mid in MOTOR_IDS]

        self._pub_t2_fb.publish(fb)
        self._pub_t2_temp.publish(tm)

    def _ros_publish_stats(self):
        """1Hz 定时器回调: 发布 bus_stats JSON"""
        msg = self._String()
        msg.data = json.dumps(self._build_stats_dict(), ensure_ascii=False)
        self._pub_stats.publish(msg)

    # ========== 统计 ==========

    def _build_stats_dict(self) -> dict:
        with self._lock:
            motors = {}
            for mid in MOTOR_IDS:
                s = self._stats[mid]
                motors[f"motor_{mid}"] = {
                    "type1_total": s.t1_count,
                    "type1_hz": round(s.t1_hz, 1),
                    "type1_last_ts": round(s.t1_ts, 6) if s.t1_ts else None,
                    "type2_total": s.t2_count,
                    "type2_hz": round(s.t2_hz, 1),
                    "type2_last_ts": round(s.t2_ts, 6) if s.t2_ts else None,
                    "fault_code": s.t2_fault,
                    "mode_state": s.t2_mode,
                }
            return {
                "elapsed_s": round(time.time() - self._start_time, 1),
                "total_frames": self._total_frames,
                "can_interface": self.can_interface,
                "motors": motors,
                "other_types": {
                    COMM_TYPE_NAMES.get(k, f"T{k}"): v
                    for k, v in self._other_type_counts.items()
                },
            }

    def update_hz(self):
        """每秒调用一次，计算各电机帧率"""
        with self._lock:
            for mid in MOTOR_IDS:
                s = self._stats[mid]
                s.t1_hz = float(s.t1_window)
                s.t2_hz = float(s.t2_window)
                s.t1_window = 0
                s.t2_window = 0

    # ========== 终端显示 ==========

    def print_table(self):
        elapsed = time.time() - self._start_time
        lines = []
        lines.append(f"\033[2J\033[H")
        lines.append(
            f"═══ CAN2 Bus Analyzer ═══  "
            f"接口: {self.can_interface}  "
            f"运行: {elapsed:.0f}s  "
            f"总帧: {self._total_frames}  "
            f"ROS: {'ON' if self.use_ros else 'OFF'}")
        lines.append("")
        lines.append(
            f"{'Motor':>7} {'T1 Hz':>7} {'T2 Hz':>7} "
            f"{'CmdPos':>9} {'FbPos':>9} {'FbVel':>9} "
            f"{'FbTorq':>9} {'Temp':>7} {'Fault':>6} {'Mode':>6} "
            f"{'T2 TS':>15}")
        lines.append("─" * 105)

        with self._lock:
            for mid in MOTOR_IDS:
                s = self._stats[mid]
                mode_name = MODE_STATE_NAMES.get(s.t2_mode, "?")
                fault_str = f"{s.t2_fault}" if s.t2_fault else "OK"
                ts_str = f"{s.t2_ts:.4f}"[-10:] if s.t2_ts else "---"
                lines.append(
                    f"  M{mid:>2}   {s.t1_hz:>6.0f}  {s.t2_hz:>6.0f}  "
                    f"{s.t1_pos:>8.3f}  {s.t2_pos:>8.3f}  "
                    f"{s.t2_vel:>8.3f}  {s.t2_torque:>8.3f}  "
                    f"{s.t2_temp:>6.1f}  {fault_str:>5}  {mode_name:>5}  "
                    f"{ts_str:>14}")

        if self._other_type_counts:
            other = ", ".join(
                f"{COMM_TYPE_NAMES.get(ct, f'T{ct}')}:{cnt}"
                for ct, cnt in sorted(self._other_type_counts.items()))
            lines.append(f"\n  其他帧类型: {other}")

        print("\n".join(lines))

    def print_summary(self):
        elapsed = time.time() - self._start_time
        print(f"\n{'═' * 70}")
        print(f"  总计统计  (接口: {self.can_interface}, 运行 {elapsed:.1f}s)")
        print(f"{'═' * 70}")
        print(f"  总帧数: {self._total_frames}")
        print(f"  {'Motor':>7} {'T1 Total':>10} {'T2 Total':>10} "
              f"{'Last Fault':>12} {'Last Mode':>12}")
        with self._lock:
            for mid in MOTOR_IDS:
                s = self._stats[mid]
                mn = MODE_STATE_NAMES.get(s.t2_mode, "?")
                print(f"    M{mid:>2}   {s.t1_count:>10}  {s.t2_count:>10}  "
                      f"{s.t2_fault:>10}  {mn:>10}")
        if self._other_type_counts:
            other = ", ".join(
                f"{COMM_TYPE_NAMES.get(ct, f'T{ct}')}:{cnt}"
                for ct, cnt in sorted(self._other_type_counts.items()))
            print(f"  其他帧类型: {other}")
        print(f"{'═' * 70}")


def build_motor_type_map(wrist_type_str: str) -> dict:
    wrist = MotorType.RS05 if wrist_type_str == "RS05" else MotorType.EL05
    return {
        1: MotorType.RS00, 2: MotorType.RS00, 3: MotorType.RS00,
        4: wrist, 5: wrist, 6: wrist, 7: wrist,
    }


def main():
    parser = argparse.ArgumentParser(
        description="CAN2 总线 Type1/Type2 帧分析器 (Foxglove 调试话题)")
    parser.add_argument("--can", default="can2",
                        help="CAN 接口名 (默认: can2)")
    parser.add_argument("--duration", type=float, default=0,
                        help="监听时长(秒)，0=持续监听 (默认: 0)")
    parser.add_argument("--motor-type", default="EL05",
                        choices=["EL05", "RS05"],
                        help="腕部电机型号 (默认: EL05)")
    parser.add_argument("--verbose", action="store_true",
                        help="打印每帧详细解码")
    parser.add_argument("--raw", action="store_true",
                        help="打印原始帧十六进制")
    parser.add_argument("--no-ros", action="store_true",
                        help="禁用 ROS2 话题发布（纯终端模式）")
    args = parser.parse_args()

    motor_map = build_motor_type_map(args.motor_type)

    print("============================================")
    print(f"  CAN2 Bus Analyzer ({args.can}, {args.motor_type})")
    print("============================================")
    if not args.no_ros:
        print("  ROS2 话题: /debug/can2/type1_command")
        print("             /debug/can2/type1_gains")
        print("             /debug/can2/type2_feedback")
        print("             /debug/can2/type2_temperature")
        print("             /debug/can2/bus_stats")
    print("============================================\n")

    analyzer = Can2BusAnalyzer(
        can_interface=args.can,
        motor_type_map=motor_map,
        use_ros=not args.no_ros,
        verbose=args.verbose,
        show_raw=args.raw,
    )

    if not analyzer.connect():
        sys.exit(1)

    shutdown_event = threading.Event()

    def on_signal(_sig, _frame):
        shutdown_event.set()
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    analyzer.start()
    analyzer.spin_ros()

    deadline = (time.time() + args.duration) if args.duration > 0 else float('inf')

    try:
        while not shutdown_event.is_set() and time.time() < deadline:
            shutdown_event.wait(timeout=1.0)
            analyzer.update_hz()
            if not args.verbose and not args.raw:
                analyzer.print_table()
    finally:
        analyzer.print_summary()
        analyzer.stop()


if __name__ == "__main__":
    main()
