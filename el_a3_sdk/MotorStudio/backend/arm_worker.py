"""后台工作线程：封装 ELA3Interface，50Hz 数据采集 + 线程安全命令队列"""

import sys
import os
import time
import math
import logging
import traceback
from pathlib import Path
from typing import List, Optional
from queue import Queue, Empty

from PyQt6.QtCore import QThread, pyqtSignal

from MotorStudio.backend.data_buffer import DataBuffer
from MotorStudio.utils.tcp_offset_store import load_tcp_offset, save_tcp_offset

logger = logging.getLogger("MotorStudio.worker")

# Sentinel for typing
try:
    from el_a3_sdk import ELA3Interface, ArmJointStates, ArmEndPose, ArmStatus
    from el_a3_sdk.protocol import DEFAULT_JOINT_LIMITS, DEFAULT_MOTOR_TYPE_MAP, ParamIndex
except ImportError:
    ELA3Interface = None


class ArmWorker(QThread):
    """
    后台线程：
    - 50Hz 轮询 SDK 反馈
    - 通过 Signal 将数据发送到 UI 线程
    - 通过命令队列接收 UI 的控制指令
    """

    joints_updated = pyqtSignal(object)
    velocities_updated = pyqtSignal(object)
    efforts_updated = pyqtSignal(object)
    status_updated = pyqtSignal(object)
    end_pose_updated = pyqtSignal(object)
    motor_feedback_updated = pyqtSignal(object)
    error_occurred = pyqtSignal(str)
    connected_changed = pyqtSignal(bool)
    enabled_changed = pyqtSignal(bool)
    control_loop_changed = pyqtSignal(bool)
    log_message = pyqtSignal(str)
    can_fps_updated = pyqtSignal(float)
    zero_sta_verified = pyqtSignal(list)
    motor_scan_result = pyqtSignal(list)
    move_j_done = pyqtSignal()
    tcp_offset_updated = pyqtSignal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.arm: Optional[object] = None
        self.data_buffer = DataBuffer(max_samples=500, num_channels=7)
        self._running = False
        self._connected = False
        self._enabled = False
        self._cmd_queue: Queue = Queue()
        self._poll_rate_hz = 50.0
        self._sim_mode = False
        self._tcp_offset = load_tcp_offset()

        self._slow_poll_counter = 0

        # Simulation state
        self._sim_positions = [0.0] * 7
        self._sim_velocities = [0.0] * 7
        self._sim_torques = [0.0] * 7
        self._sim_target = [0.0] * 7
        self._sim_kin = None
        self._sim_kin_failed = False

    @property
    def is_connected(self):
        return self._connected

    @property
    def is_enabled(self):
        return self._enabled

    def submit_command(self, cmd: str, *args, **kwargs):
        self._cmd_queue.put((cmd, args, kwargs))

    def run(self):
        self._running = True
        interval = 1.0 / self._poll_rate_hz
        while self._running:
            t0 = time.time()
            try:
                self._process_commands()
                if self._connected:
                    self._poll_feedback()
            except Exception as e:
                logger.error(f"Worker error: {e}\n{traceback.format_exc()}")
                self.error_occurred.emit(str(e))
            elapsed = time.time() - t0
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop(self):
        self._running = False
        self.wait(3000)

    def _process_commands(self):
        while True:
            try:
                cmd, args, kwargs = self._cmd_queue.get_nowait()
            except Empty:
                break
            try:
                self._execute_command(cmd, args, kwargs)
            except Exception as e:
                logger.error(f"Command '{cmd}' failed: {e}")
                self.error_occurred.emit(f"命令 {cmd} 失败: {e}")

    def _execute_command(self, cmd: str, args, kwargs):
        if cmd == "connect":
            self._do_connect(*args, **kwargs)
        elif cmd == "disconnect":
            self._do_disconnect()
        elif cmd == "enable":
            self._do_enable()
        elif cmd == "disable":
            self._do_disable()
        elif cmd == "emergency_stop":
            self._do_emergency_stop()
        elif cmd == "joint_ctrl":
            self._do_joint_ctrl(*args)
        elif cmd == "move_j":
            self._do_move_j(*args, **kwargs)
        elif cmd == "move_l":
            self._do_move_l(*args, **kwargs)
        elif cmd == "end_pose_ctrl":
            self._do_end_pose_ctrl(*args, **kwargs)
        elif cmd == "cancel_motion":
            self._do_cancel_motion()
        elif cmd == "zero_torque":
            self._do_zero_torque(*args, **kwargs)
        elif cmd == "zero_torque_gravity":
            self._do_zero_torque_gravity(*args, **kwargs)
        elif cmd == "gripper_ctrl":
            self._do_gripper_ctrl(*args)
        elif cmd == "set_zero_position":
            self._do_set_zero(*args)
        elif cmd == "verify_zero_sta":
            self._do_verify_zero_sta()
        elif cmd == "set_all_zero_sta":
            self._do_set_all_zero_sta()
        elif cmd == "scan_motors":
            self._do_scan_motors()
        elif cmd == "read_motor_param":
            self._do_read_param(*args)
        elif cmd == "write_motor_param":
            self._do_write_param(*args)
        elif cmd == "start_control_loop":
            self._do_start_control_loop(*args)
        elif cmd == "stop_control_loop":
            self._do_stop_control_loop()
        elif cmd == "move_j_block":
            self._do_move_j_block(*args, **kwargs)
        elif cmd == "set_smoothing_alpha":
            if self.arm and not self._sim_mode:
                self.arm.SetSmoothingAlpha(args[0])
        elif cmd == "set_tcp_offset":
            self._do_set_tcp_offset(*args, persist=False)
        elif cmd == "save_tcp_offset":
            self._do_set_tcp_offset(*args, persist=True)
        elif cmd == "restore_tcp_offset":
            self._do_restore_tcp_offset()

    def _do_connect(self, can_name="can0", sim_mode=False,
                    backend="socketcan", serial_port=None,
                    serial_baudrate=2000000, can_bitrate=1000000):
        if self._connected:
            return
        self._sim_mode = sim_mode
        if sim_mode:
            self._connected = True
            self.connected_changed.emit(True)
            self.tcp_offset_updated.emit(list(self._tcp_offset))
            self.log_message.emit("已连接（模拟模式）")
            return
        if ELA3Interface is None:
            self.error_occurred.emit("el_a3_sdk 未安装")
            return

        if backend != "slcan":
            from MotorStudio.utils.can_utils import get_can_state
            state = get_can_state(can_name)
            if state != "UP":
                self.error_occurred.emit(
                    f"CAN 接口 {can_name} 未开启（当前状态: {state}），请先在工具栏中开启"
                )
                return

        try:
            if getattr(sys, "frozen", False):
                sdk_root = Path(sys._MEIPASS)
            else:
                try:
                    import el_a3_sdk as _el_a3_sdk_pkg
                    sdk_root = Path(_el_a3_sdk_pkg.__file__).resolve().parent.parent
                except Exception:
                    sdk_root = Path(__file__).resolve().parent.parent.parent
            inertia_path = sdk_root / "resources" / "config" / "inertia_params.yaml"
            default_urdf_path = sdk_root / "resources" / "urdf" / "el_a3.urdf"
            kwargs = dict(can_name=can_name)
            if inertia_path.exists():
                kwargs["inertia_config_path"] = str(inertia_path)
            if default_urdf_path.exists():
                kwargs["urdf_path"] = str(default_urdf_path)
            kwargs["per_joint_kd_min"] = {4: 0.005, 5: 0.005, 6: 0.005, 7: 0.02}
            kwargs["per_joint_kd_max"] = {4: 0.10, 5: 0.05, 6: 0.05, 7: 0.10}
            kwargs["gravity_joint_scale"] = {4: 2.0}
            if backend == "slcan":
                kwargs["backend"] = "slcan"
                kwargs["serial_port"] = serial_port or can_name
                kwargs["serial_baudrate"] = serial_baudrate
                kwargs["can_bitrate"] = can_bitrate
            self.arm = ELA3Interface(**kwargs)
            if not self.arm.ConnectPort():
                raise RuntimeError("ConnectPort 返回 False")
            self.arm.SetTcpOffset(self._tcp_offset)
            self._connected = True
            self.connected_changed.emit(True)
            self.tcp_offset_updated.emit(list(self._tcp_offset))
            display_name = f"{serial_port or can_name} (SLCAN)" if backend == "slcan" else can_name
            self.log_message.emit(f"已连接到 {display_name}")
        except Exception as e:
            self.arm = None
            self.error_occurred.emit(f"连接失败: {e}")

    def _do_disconnect(self):
        if not self._connected:
            return
        if self._sim_mode:
            self._connected = False
            self._enabled = False
            self.connected_changed.emit(False)
            self.enabled_changed.emit(False)
            self.log_message.emit("已断开（模拟模式）")
            return
        try:
            if self.arm:
                if self._enabled:
                    self.arm.DisableArm()
                self.arm.DisconnectPort()
                self.arm = None
            self._connected = False
            self._enabled = False
            self.connected_changed.emit(False)
            self.enabled_changed.emit(False)
            self.log_message.emit("已断开连接")
        except Exception as e:
            self.error_occurred.emit(f"断开失败: {e}")

    def _do_enable(self):
        if not self._connected:
            return
        if self._sim_mode:
            self._enabled = True
            self.enabled_changed.emit(True)
            self.log_message.emit("电机已使能（模拟模式）")
            return
        try:
            self.arm.EnableArm()
            self._enabled = True
            self.enabled_changed.emit(True)
            self.log_message.emit("电机已使能")
        except Exception as e:
            self.error_occurred.emit(f"使能失败: {e}")

    def _do_disable(self):
        if self._sim_mode:
            self._enabled = False
            self.enabled_changed.emit(False)
            self.log_message.emit("电机已失能（模拟模式）")
            return
        try:
            if self.arm:
                self.arm.DisableArm()
            self._enabled = False
            self.enabled_changed.emit(False)
            self.log_message.emit("电机已失能")
        except Exception as e:
            self.error_occurred.emit(f"失能失败: {e}")

    def _do_emergency_stop(self):
        if self._sim_mode:
            self._enabled = False
            self.enabled_changed.emit(False)
            self.log_message.emit("急停已触发（模拟模式）")
            return
        try:
            if self.arm:
                self.arm.EmergencyStop()
            self._enabled = False
            self.enabled_changed.emit(False)
            self.log_message.emit("急停已触发")
        except Exception as e:
            self.error_occurred.emit(f"急停失败: {e}")

    def _ensure_control_loop(self):
        """运动命令前自动启动控制循环（200Hz EMA 平滑 + 速度前馈 + 重力补偿）"""
        if self.arm and not self.arm.control_loop_running:
            self.arm.start_control_loop(rate_hz=200.0)
            self.control_loop_changed.emit(True)
            self.log_message.emit("控制循环已自动启动 (200Hz)")

    def _do_joint_ctrl(self, positions: List[float]):
        if self._sim_mode:
            self._sim_target = list(positions[:7])
            return
        if self.arm and self._enabled:
            self._ensure_control_loop()
            self.arm.JointCtrlList(positions[:6])

    def _do_move_j(self, positions, duration=2.0, block=False):
        if self._sim_mode:
            self._sim_target = list(positions[:7]) + [0.0] * (7 - len(positions))
            self.log_message.emit(f"MoveJ 执行（模拟）duration={duration}s")
            return
        if self.arm and self._enabled:
            self._ensure_control_loop()
            self.arm.MoveJ(positions, duration=duration, block=block)
            self.log_message.emit(f"MoveJ 执行中 duration={duration}s")

    def _do_move_j_block(self, positions, duration=2.0):
        """阻塞式 MoveJ，完成后发出 move_j_done 信号（供标定流程使用）。"""
        if self._sim_mode:
            self._sim_target = list(positions[:7]) + [0.0] * (7 - len(positions))
            import time as _t
            _t.sleep(min(duration, 2.0))
            self.move_j_done.emit()
            return
        if self.arm and self._enabled:
            self._ensure_control_loop()
            self.arm.MoveJ(positions, duration=duration, block=True)
            self.move_j_done.emit()

    def _do_move_l(self, target_pose, duration=2.0, block=False):
        target_pose = self._coerce_end_pose(target_pose)
        if target_pose is None:
            self.error_occurred.emit("MoveL 目标位姿无效")
            return
        if self._sim_mode:
            q_target = self._solve_sim_ik(target_pose)
            if q_target is None:
                self.error_occurred.emit("MoveL IK 失败（模拟），请检查目标位姿或末端点")
                return
            self._sim_target = list(q_target[:6]) + [self._sim_target[6]]
            self.log_message.emit(f"MoveL 执行（模拟 IK）duration={duration}s")
            return
        if self.arm and self._enabled:
            self._ensure_control_loop()
            ok = self.arm.MoveL(target_pose, duration=duration, block=block)
            if ok:
                self.log_message.emit(f"MoveL 执行中 duration={duration}s")
            else:
                self.error_occurred.emit("MoveL 执行失败，请检查目标位姿是否可达")

    def _do_end_pose_ctrl(self, x, y, z, rx, ry, rz, duration=2.0):
        if self._sim_mode:
            target_pose = ArmEndPose(x=x, y=y, z=z, rx=rx, ry=ry, rz=rz)
            q_target = self._solve_sim_ik(target_pose)
            if q_target is None:
                self.error_occurred.emit("EndPoseCtrl IK 失败（模拟）")
                return
            self._sim_target = list(q_target[:6]) + [self._sim_target[6]]
            self.log_message.emit(
                f"EndPoseCtrl（模拟 IK）[{x * 100.0:.2f}cm,{y * 100.0:.2f}cm,{z * 100.0:.2f}cm]"
            )
            return
        if self.arm and self._enabled:
            self._ensure_control_loop()
            self.arm.EndPoseCtrl(x, y, z, rx, ry, rz, duration=duration)

    def _do_cancel_motion(self):
        if self._sim_mode:
            return
        if self.arm:
            self.arm.cancel_motion()
            self.log_message.emit("运动已取消")

    _ZERO_TORQUE_KD = [0.05, 0.05, 0.05, 0.05, 0.0125, 0.0125, 0.05]

    def _do_zero_torque(self, enable):
        if self._sim_mode:
            state = "开启" if enable else "关闭"
            self.log_message.emit(f"零力矩模式{state}（模拟）")
            return
        if self.arm:
            kd = self._ZERO_TORQUE_KD[0] if enable else 1.0
            self.arm.ZeroTorqueMode(enable, kd=kd)
            state = "开启" if enable else "关闭"
            self.log_message.emit(f"零力矩模式{state}")

    def _do_zero_torque_gravity(self, enable):
        if self._sim_mode:
            state = "开启" if enable else "关闭"
            self.log_message.emit(f"重力补偿零力矩{state}（模拟）")
            return
        if self.arm:
            self.arm.ZeroTorqueModeWithGravity(
                enable, kd=self._ZERO_TORQUE_KD, update_rate=100.0)
            state = "开启" if enable else "关闭"
            self.log_message.emit(f"重力补偿零力矩{state}")

    def _do_gripper_ctrl(self, angle):
        if self._sim_mode:
            self._sim_target[6] = angle
            return
        if self.arm and self._enabled:
            self.arm.GripperCtrl(angle)

    def _do_set_zero(self, motor_num=0xFF):
        if self._sim_mode:
            self._sim_positions = [0.0] * 7
            self.log_message.emit("零位已设置（模拟）")
            return
        if self.arm:
            self.arm.SetZeroPosition(motor_num)
            self.log_message.emit(f"电机{motor_num}零位已设置")

    def _do_verify_zero_sta(self):
        from el_a3_sdk.protocol import ParamIndex
        results = []
        if not self.arm:
            return
        self.log_message.emit("开始校验 ZERO_STA 参数...")
        all_ok = True
        for mid in range(1, 8):
            result = self.arm.ReadMotorParameter(mid, ParamIndex.ZERO_STA)
            if result and result.success:
                val = result.value_uint8
                ok = (val == 1)
                results.append((mid, val, True))
                status = "✓" if ok else "✗"
                raw_hex = result.raw_bytes.hex() if result.raw_bytes else ""
                self.log_message.emit(
                    f"  电机{mid} ZERO_STA = {val} {status}  (raw: {raw_hex})")
                if not ok:
                    all_ok = False
            else:
                results.append((mid, 0, False))
                self.log_message.emit(f"  电机{mid} ZERO_STA 读取失败")
                all_ok = False
        self.zero_sta_verified.emit(results)
        if all_ok:
            self.log_message.emit("ZERO_STA 校验完成: 全部通过")
        else:
            self.log_message.emit("ZERO_STA 校验完成: 存在异常")

    def _do_set_all_zero_sta(self):
        from el_a3_sdk.protocol import ParamIndex
        if self._sim_mode:
            self.log_message.emit("一键设置 ZERO_STA=1（模拟）")
            return
        if not self.arm or not self._connected:
            self.log_message.emit("未连接，无法设置 ZERO_STA")
            return
        self.log_message.emit("开始设置全部电机 ZERO_STA=1 ...")
        fail_count = 0
        for mid in range(1, 8):
            ok = self.arm.WriteMotorParameterInt(mid, ParamIndex.ZERO_STA, 1)
            if ok:
                self.log_message.emit(f"  电机{mid} ZERO_STA 已设置为 1")
            else:
                self.log_message.emit(f"  电机{mid} ZERO_STA 设置失败")
                fail_count += 1
        if fail_count == 0:
            self.log_message.emit("全部电机 ZERO_STA 设置完成，保存参数到 Flash ...")
            import time as _t
            _t.sleep(0.05)
            self.arm.SaveParameters(0xFF)
            self.log_message.emit("参数已保存")
        else:
            self.log_message.emit(
                f"ZERO_STA 设置完成: {fail_count} 个电机失败，跳过保存")
        self._do_verify_zero_sta()

    def _do_scan_motors(self):
        results = []
        if self._sim_mode:
            results = [(mid, True, "v1.0.0-sim", 24.0) for mid in range(1, 8)]
            self.motor_scan_result.emit(results)
            self.log_message.emit("电机扫描完成（模拟）: 7/7 在线")
            return
        if not self.arm:
            return
        self.log_message.emit("开始扫描电机...")
        online_count = 0
        for mid in range(1, 8):
            fw = self.arm.GetFirmwareVersion(mid)
            voltage = self.arm.GetMotorVoltage(mid)
            online = fw is not None or voltage is not None
            fw_str = fw.version_str if fw else ""
            if online:
                online_count += 1
                v_str = f" {voltage:.1f}V" if voltage is not None else ""
                self.log_message.emit(
                    f"  电机{mid}: 在线  固件={fw_str or '—'}{v_str}")
            else:
                self.log_message.emit(f"  电机{mid}: 离线")
            results.append((mid, online, fw_str, voltage))
        self.motor_scan_result.emit(results)
        self.log_message.emit(f"电机扫描完成: {online_count}/7 在线")

    def _do_read_param(self, motor_id, param_index):
        if self._sim_mode:
            self.log_message.emit(f"读取参数（模拟）motor={motor_id} param=0x{param_index:04X}")
            return
        if self.arm:
            result = self.arm.ReadMotorParameter(motor_id, param_index)
            self.log_message.emit(
                f"电机{motor_id} 参数0x{param_index:04X} = {result}"
            )

    def _do_write_param(self, motor_id, param_index, value):
        if self._sim_mode:
            self.log_message.emit(
                f"写入参数（模拟）motor={motor_id} param=0x{param_index:04X} val={value}"
            )
            return
        if self.arm:
            self.arm.WriteMotorParameter(motor_id, param_index, value)
            self.log_message.emit(
                f"电机{motor_id} 参数0x{param_index:04X} 已写入 {value}"
            )

    def _do_start_control_loop(self, rate_hz=200.0):
        if self._sim_mode:
            self.control_loop_changed.emit(True)
            self.log_message.emit(f"控制循环已启动（模拟）{rate_hz}Hz")
            return
        if self.arm:
            self.arm.start_control_loop(rate_hz=rate_hz)
            self.control_loop_changed.emit(True)
            self.log_message.emit(f"控制循环已启动 {rate_hz}Hz")

    def _do_stop_control_loop(self):
        if self._sim_mode:
            self.control_loop_changed.emit(False)
            self.log_message.emit("控制循环已停止（模拟）")
            return
        if self.arm:
            self.arm.stop_control_loop()
            self.control_loop_changed.emit(False)
            self.log_message.emit("控制循环已停止")

    def _do_set_tcp_offset(self, tcp_offset, persist: bool = False):
        values = self._normalize_tcp_offset(tcp_offset)
        self._tcp_offset = list(values)
        if persist:
            try:
                save_tcp_offset(values)
            except Exception as e:
                self.error_occurred.emit(f"末端点保存失败: {e}")
        if self._sim_mode:
            self.tcp_offset_updated.emit(list(values))
            self.log_message.emit(
                f"末端点已{'保存' if persist else '应用'}（模拟）: "
                f"{self._format_tcp_offset_for_log(values)}"
            )
            self._emit_sim_end_pose(time.time())
            return
        if self.arm:
            try:
                self.arm.SetTcpOffset(values)
                self.tcp_offset_updated.emit(list(values))
                self.log_message.emit(
                    f"末端点已{'保存' if persist else '应用'}: "
                    f"{self._format_tcp_offset_for_log(values)}"
                )
                try:
                    self.end_pose_updated.emit(self.arm.GetArmEndPoseMsgs())
                except Exception:
                    pass
            except Exception as e:
                self.error_occurred.emit(f"末端点设置失败: {e}")

    def _do_restore_tcp_offset(self):
        self._do_set_tcp_offset([0.0] * 6, persist=True)

    def _poll_feedback(self):
        now = time.time()
        self._slow_poll_counter += 1
        do_slow_poll = (self._slow_poll_counter % 10 == 0)  # ~5Hz for heavy queries

        if self._sim_mode:
            self._update_sim_state()
            positions = list(self._sim_positions)
            velocities = list(self._sim_velocities)
            torques = list(self._sim_torques)
            temperatures = [25.0 + i * 0.5 for i in range(7)]
        else:
            try:
                joint_msg = self.arm.GetArmJointMsgs()
                vel_msg = self.arm.GetArmJointVelocities()
                eff_msg = self.arm.GetArmJointEfforts()

                positions = joint_msg.to_list()
                velocities = vel_msg.to_list()
                torques = eff_msg.to_list()
                temperatures = [0.0] * 7

                self.joints_updated.emit(joint_msg)
                self.velocities_updated.emit(vel_msg)
                self.efforts_updated.emit(eff_msg)

                if do_slow_poll:
                    try:
                        status = self.arm.GetArmStatus()
                        self.status_updated.emit(status)
                    except Exception:
                        pass

                    try:
                        end_pose = self.arm.GetArmEndPoseMsgs()
                        self.end_pose_updated.emit(end_pose)
                    except Exception:
                        pass

                    try:
                        fps = self.arm.GetCanFps()
                        self.can_fps_updated.emit(fps)
                    except Exception:
                        pass

                    motor_fb_list = []
                    try:
                        states = self.arm.GetMotorStates()
                        if states:
                            for mid in range(1, 8):
                                fb = states.get(mid)
                                if fb is not None:
                                    motor_fb_list.append(fb)
                                    if hasattr(fb, 'temperature'):
                                        temperatures[mid - 1] = fb.temperature
                    except Exception:
                        pass

                    if motor_fb_list:
                        self.motor_feedback_updated.emit(motor_fb_list)

            except Exception as e:
                logger.debug(f"Poll error: {e}")
                return

        self.data_buffer.append(
            now, positions, velocities, torques, temperatures
        )

        if self._sim_mode:
            from el_a3_sdk.data_types import ArmJointStates
            js = ArmJointStates.from_list(positions, timestamp=now)
            js.hz = self._poll_rate_hz
            self.joints_updated.emit(js)
            vs = ArmJointStates.from_list(velocities, timestamp=now)
            self.velocities_updated.emit(vs)
            es = ArmJointStates.from_list(torques, timestamp=now)
            self.efforts_updated.emit(es)
            if do_slow_poll:
                self._emit_sim_end_pose(now)
            self.can_fps_updated.emit(200.0)

    def _update_sim_state(self):
        """简单的一阶位置模拟"""
        alpha = 0.05
        for i in range(7):
            diff = self._sim_target[i] - self._sim_positions[i]
            self._sim_velocities[i] = diff * 2.0
            self._sim_positions[i] += diff * alpha
            self._sim_torques[i] = diff * 10.0
            if abs(diff) < 0.001:
                self._sim_velocities[i] = 0.0
                self._sim_torques[i] = 0.0

    def _coerce_end_pose(self, target_pose):
        if target_pose is None:
            return None
        if all(hasattr(target_pose, attr) for attr in ("x", "y", "z", "rx", "ry", "rz")):
            return target_pose
        try:
            vals = list(target_pose)
            if len(vals) < 6:
                return None
            return ArmEndPose(
                x=float(vals[0]), y=float(vals[1]), z=float(vals[2]),
                rx=float(vals[3]), ry=float(vals[4]), rz=float(vals[5]),
            )
        except Exception:
            return None

    def _emit_sim_end_pose(self, timestamp: float):
        try:
            if not self._ensure_sim_kin():
                return
            pose = self._sim_kin.forward_kinematics(self._sim_positions[:6])
            pose.timestamp = timestamp
            self.end_pose_updated.emit(pose)
        except Exception:
            self._sim_kin_failed = True

    def _ensure_sim_kin(self):
        if self._sim_kin_failed:
            return False
        try:
            if self._sim_kin is None:
                from el_a3_sdk.kinematics import ELA3Kinematics
                self._sim_kin = ELA3Kinematics(tcp_offset=self._tcp_offset)
            else:
                self._sim_kin.set_tcp_offset(self._tcp_offset)
            return True
        except Exception as e:
            self._sim_kin_failed = True
            self.error_occurred.emit(f"仿真运动学初始化失败: {e}")
            return False

    def _solve_sim_ik(self, target_pose):
        if not self._ensure_sim_kin():
            return None
        q_current = list(self._sim_positions[:6])
        try:
            q_next, err_norm = self._sim_kin.ik_step(
                target_pose,
                q_current,
                damping=8e-3,
                max_step=0.25,
                max_iter=20,
                converge_eps=3e-4,
            )
            if q_next is not None and err_norm <= 3e-4:
                return q_next
            return self._sim_kin.inverse_kinematics(
                target_pose,
                q_init=q_next or q_current,
                max_iter=200,
                eps=3e-4,
            )
        except Exception as e:
            self.error_occurred.emit(f"仿真 IK 异常: {e}")
            return None

    @staticmethod
    def _normalize_tcp_offset(values):
        offset = [0.0] * 6
        if values is None:
            return offset
        seq = list(values)
        for idx in range(min(6, len(seq))):
            try:
                offset[idx] = float(seq[idx])
            except (TypeError, ValueError):
                offset[idx] = 0.0
        return offset

    @staticmethod
    def _format_tcp_offset_for_log(values):
        vals = list(values) + [0.0] * (6 - len(values))
        return (
            f"X={vals[2] * 100.0:.2f}cm, "
            f"Y={vals[0] * 100.0:.2f}cm, "
            f"Z={vals[1] * 100.0:.2f}cm, "
            f"Rx={math.degrees(vals[3]):.2f}°, "
            f"Ry={math.degrees(vals[4]):.2f}°, "
            f"Rz={math.degrees(vals[5]):.2f}°"
        )
