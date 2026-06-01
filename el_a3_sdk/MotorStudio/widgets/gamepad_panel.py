"""手柄控制面板：设备连接 + 输入数据监控 + 手柄控臂"""

import glob
import threading
import logging
from typing import Optional

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QComboBox, QPushButton, QGroupBox, QProgressBar,
    QScrollArea, QFrame,
)
from PyQt6.QtCore import pyqtSignal, QTimer, Qt

from MotorStudio.utils.i18n import tr
from MotorStudio.utils.theme_manager import ThemeManager
from MotorStudio.utils.style import SCENE_COLORS

from el_a3_sdk.joystick import LinuxJoystick
from el_a3_sdk.controller_profiles import (
    PROFILES, detect_controller, ControllerDetection,
)

logger = logging.getLogger("MotorStudio.gamepad")

M_TO_CM = 100.0


class _AxisBar(QProgressBar):
    """双向进度条，用于显示 -1.0 ~ 1.0 的轴值"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setRange(0, 2000)
        self.setValue(1000)
        self.setTextVisible(False)
        self.setFixedHeight(16)
        self.apply_theme()

    def apply_theme(self):
        sc = SCENE_COLORS[ThemeManager.instance().theme]
        self.setStyleSheet(
            f"QProgressBar {{ background: {sc['progress_bg']}; border: 1px solid {sc['progress_border']};"
            f"  border-radius: 4px; }}"
            f"QProgressBar::chunk {{ background: {sc['progress_chunk']}; border-radius: 3px; }}"
        )

    def set_value(self, v: float):
        mapped = int((v + 1.0) * 1000)
        self.setValue(max(0, min(2000, mapped)))


class _ButtonIndicator(QLabel):
    """单个按钮指示灯"""

    def __init__(self, index: int, parent=None):
        super().__init__(str(index), parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._pressed = False
        self.apply_theme()

    def apply_theme(self):
        sc = SCENE_COLORS[ThemeManager.instance().theme]
        if self._pressed:
            self.setStyleSheet(
                f"background: {sc['indicator_on_bg']}; border: 1px solid {sc['indicator_on_border']};"
                f"border-radius: 4px; min-width: 28px; min-height: 20px;"
                f"color: {sc['indicator_on_fg']}; font-size: 11px; font-weight: bold;"
                "qproperty-alignment: AlignCenter;"
            )
        else:
            self.setStyleSheet(
                f"background: {sc['indicator_off_bg']}; border: 1px solid {sc['indicator_off_border']};"
                f"border-radius: 4px; min-width: 28px; min-height: 20px;"
                f"color: {sc['indicator_off_fg']}; font-size: 11px;"
                "qproperty-alignment: AlignCenter;"
            )

    def set_pressed(self, pressed: bool):
        self._pressed = pressed
        self.apply_theme()


class GamepadPanel(QWidget):
    """手柄控制 Tab 面板"""

    gamepad_log = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

        self._joystick: Optional[LinuxJoystick] = None
        self._arm = None
        self._controller = None
        self._ctrl_thread: Optional[threading.Thread] = None
        self._detection: Optional[ControllerDetection] = None

        self._speed_idx = 2
        self._ctrl_running = False

        self._init_ui()

        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(50)  # 20 Hz
        self._poll_timer.timeout.connect(self._poll_input)

    def _scene(self):
        return SCENE_COLORS[ThemeManager.instance().theme]

    @staticmethod
    def _speed_level_specs():
        return [
            (tr("gp.speed_very_slow"), 0.10),
            (tr("gp.speed_slow"), 0.25),
            (tr("gp.speed_medium"), 0.50),
            (tr("gp.speed_fast"), 0.75),
            (tr("gp.speed_max"), 1.00),
        ]

    def _apply_monitor_header_styles(self):
        sc = self._scene()
        self._axes_title.setStyleSheet(
            f"font-weight: bold; color: {sc['accent']}; margin-top: 2px;")
        self._buttons_title.setStyleSheet(
            f"font-weight: bold; color: {sc['accent']}; margin-top: 4px;")
        self._mapped_title.setStyleSheet(
            f"font-weight: bold; color: {sc['accent']}; margin-top: 4px;")
        self._mapped_display.setStyleSheet(
            f"font-family: monospace; font-size: 11px; color: {sc['mapped_fg']}; "
            f"background: {sc['mapped_bg']}; padding: 4px; border-radius: 3px;"
        )
        self._info_label.setStyleSheet(
            f"color: {sc['subtext']}; font-size: 11px;")

    # ---- UI Construction ----

    def _init_ui(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        inner = QWidget()
        layout = QVBoxLayout(inner)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(6)

        self._conn_group = self._build_connection_group()
        layout.addWidget(self._conn_group)
        self._monitor_group = self._build_monitor_group()
        layout.addWidget(self._monitor_group)
        self._ctrl_group = self._build_control_group()
        layout.addWidget(self._ctrl_group)
        layout.addStretch()

        scroll.setWidget(inner)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

        self.retranslate_ui()

    def _build_connection_group(self) -> QGroupBox:
        grp = QGroupBox()
        layout = QVBoxLayout()
        layout.setSpacing(4)

        row1 = QHBoxLayout()
        self._lbl_device = QLabel()
        row1.addWidget(self._lbl_device)
        self._dev_combo = QComboBox()
        self._dev_combo.setEditable(True)
        self._dev_combo.setMinimumWidth(140)
        row1.addWidget(self._dev_combo, 1)

        self._refresh_btn = QPushButton()
        self._refresh_btn.setFixedWidth(50)
        self._refresh_btn.clicked.connect(self._scan_devices)
        row1.addWidget(self._refresh_btn)
        layout.addLayout(row1)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Profile:"))
        self._profile_combo = QComboBox()
        self._profile_combo.addItem("auto", "auto")
        for pid, prof in PROFILES.items():
            self._profile_combo.addItem(prof.display_name, pid)
        row2.addWidget(self._profile_combo, 1)
        layout.addLayout(row2)

        row3 = QHBoxLayout()
        self._connect_btn = QPushButton()
        self._connect_btn.setObjectName("connectBtn")
        self._connect_btn.clicked.connect(self._toggle_connection)
        row3.addWidget(self._connect_btn)
        row3.addStretch()
        layout.addLayout(row3)

        self._info_label = QLabel()
        self._info_label.setWordWrap(True)
        layout.addWidget(self._info_label)

        grp.setLayout(layout)
        self._scan_devices()
        return grp

    def _build_monitor_group(self) -> QGroupBox:
        grp = QGroupBox()
        layout = QVBoxLayout()
        layout.setSpacing(4)

        self._axes_title = QLabel()
        layout.addWidget(self._axes_title)

        self._axis_bars: list[_AxisBar] = []
        self._axis_labels: list[QLabel] = []
        axes_grid = QGridLayout()
        axes_grid.setSpacing(2)
        for i in range(LinuxJoystick.MAX_AXES):
            name_lbl = QLabel(f"A{i}:")
            name_lbl.setFixedWidth(24)
            name_lbl.setStyleSheet("font-size: 11px;")
            bar = _AxisBar()
            val_lbl = QLabel(" 0.000")
            val_lbl.setFixedWidth(48)
            val_lbl.setStyleSheet("font-family: monospace; font-size: 11px;")
            axes_grid.addWidget(name_lbl, i, 0)
            axes_grid.addWidget(bar, i, 1)
            axes_grid.addWidget(val_lbl, i, 2)
            self._axis_bars.append(bar)
            self._axis_labels.append(val_lbl)
        layout.addLayout(axes_grid)

        self._buttons_title = QLabel()
        layout.addWidget(self._buttons_title)

        self._btn_indicators: list[_ButtonIndicator] = []
        btn_grid = QGridLayout()
        btn_grid.setSpacing(3)
        for i in range(LinuxJoystick.MAX_BUTTONS):
            ind = _ButtonIndicator(i)
            btn_grid.addWidget(ind, i // 8, i % 8)
            self._btn_indicators.append(ind)
        layout.addLayout(btn_grid)

        self._mapped_title = QLabel()
        layout.addWidget(self._mapped_title)

        self._mapped_display = QLabel("—")
        self._mapped_display.setWordWrap(True)
        layout.addWidget(self._mapped_display)

        grp.setLayout(layout)
        return grp

    def _build_control_group(self) -> QGroupBox:
        grp = QGroupBox()
        layout = QVBoxLayout()
        layout.setSpacing(4)

        row1 = QHBoxLayout()
        self._ctrl_btn = QPushButton()
        self._ctrl_btn.setObjectName("enableBtn")
        self._ctrl_btn.setEnabled(False)
        self._ctrl_btn.clicked.connect(self._toggle_control)
        row1.addWidget(self._ctrl_btn)
        row1.addStretch()
        layout.addLayout(row1)

        info_grid = QGridLayout()
        info_grid.setSpacing(2)

        self._lbl_speed_level = QLabel()
        info_grid.addWidget(self._lbl_speed_level, 0, 0)
        self._speed_label = QLabel()
        self._speed_label.setStyleSheet("font-weight: bold;")
        info_grid.addWidget(self._speed_label, 0, 1)

        self._lbl_ctrl_mode = QLabel()
        info_grid.addWidget(self._lbl_ctrl_mode, 1, 0)
        self._mode_label = QLabel("—")
        info_grid.addWidget(self._mode_label, 1, 1)

        self._lbl_end_pose = QLabel()
        info_grid.addWidget(self._lbl_end_pose, 2, 0)
        self._pose_label = QLabel("—")
        self._pose_label.setStyleSheet("font-family: monospace; font-size: 11px;")
        self._pose_label.setWordWrap(True)
        info_grid.addWidget(self._pose_label, 2, 1)

        layout.addLayout(info_grid)
        grp.setLayout(layout)
        return grp

    def retranslate_ui(self):
        self._conn_group.setTitle(tr("gp.conn_group"))
        self._lbl_device.setText(tr("gp.device"))
        self._refresh_btn.setText(tr("gp.refresh"))
        if self._joystick and self._joystick.connected:
            self._connect_btn.setText(tr("gp.disconnect"))
        else:
            self._connect_btn.setText(tr("gp.connect"))
        if self._joystick and self._joystick.connected and self._detection:
            det = self._detection
            self._info_label.setText(
                tr(
                    "gp.connected_info",
                    name=det.name or "unknown",
                    vendor=det.vendor,
                    product=det.product,
                    pname=det.profile.display_name,
                    source=det.source,
                )
            )
        else:
            self._info_label.setText(tr("gp.not_connected"))

        self._monitor_group.setTitle(tr("gp.monitor_group"))
        self._axes_title.setText(tr("gp.axes"))
        self._buttons_title.setText(tr("gp.buttons"))
        self._mapped_title.setText(tr("gp.mapped"))

        self._ctrl_group.setTitle(tr("gp.ctrl_group"))
        self._update_ctrl_btn_state()
        self._lbl_speed_level.setText(tr("gp.speed_level"))
        self._lbl_ctrl_mode.setText(tr("gp.ctrl_mode"))
        self._lbl_end_pose.setText(tr("gp.end_pose"))

        specs = self._speed_level_specs()
        self._speed_label.setText(
            f"{self._speed_idx + 1}/5 [{specs[self._speed_idx][0]}]")
        self._apply_monitor_header_styles()
        for bar in self._axis_bars:
            bar.apply_theme()
        for ind in self._btn_indicators:
            ind.apply_theme()

    # ---- Device Scanning ----

    def _scan_devices(self):
        self._dev_combo.clear()
        devices = sorted(glob.glob("/dev/input/js*"))
        if devices:
            for d in devices:
                self._dev_combo.addItem(d)
        else:
            self._dev_combo.addItem("/dev/input/js0")

    # ---- Connection ----

    def _toggle_connection(self):
        if self._joystick and self._joystick.connected:
            self._disconnect_gamepad()
        else:
            self._connect_gamepad()

    def _connect_gamepad(self):
        device = self._dev_combo.currentText().strip()
        if not device:
            return

        profile_id = self._profile_combo.currentData()

        try:
            self._detection = detect_controller(device, requested_profile=profile_id)
        except Exception as e:
            es = str(e)
            self._info_label.setText(tr("gp.profile_fail", e=es))
            self.gamepad_log.emit(tr("gp.profile_fail_log", e=es))
            return

        joy = LinuxJoystick(device=device)
        if not joy.connect():
            self._info_label.setText(tr("gp.cannot_open", device=device))
            self.gamepad_log.emit(tr("gp.connect_fail_log", device=device))
            return

        self._joystick = joy
        det = self._detection
        self._info_label.setText(
            tr(
                "gp.connected_info",
                name=det.name or "unknown",
                vendor=det.vendor,
                product=det.product,
                pname=det.profile.display_name,
                source=det.source,
            )
        )
        self._connect_btn.setText(tr("gp.disconnect"))
        self._connect_btn.setObjectName("disconnectBtn")
        self._connect_btn.setStyle(self._connect_btn.style())
        self._poll_timer.start()
        self._update_ctrl_btn_state()

        self.gamepad_log.emit(
            tr(
                "gp.connected_log",
                device=device,
                profile=det.profile.display_name,
                source=det.source,
            )
        )

    def _disconnect_gamepad(self):
        if self._ctrl_running:
            self._stop_control()

        self._poll_timer.stop()
        if self._joystick:
            self._joystick.disconnect()
            self._joystick = None
        self._detection = None

        self._connect_btn.setText(tr("gp.connect"))
        self._connect_btn.setObjectName("connectBtn")
        self._connect_btn.setStyle(self._connect_btn.style())
        self._info_label.setText(tr("gp.not_connected"))
        self._update_ctrl_btn_state()

        for bar in self._axis_bars:
            bar.set_value(0.0)
        for lbl in self._axis_labels:
            lbl.setText(" 0.000")
        for ind in self._btn_indicators:
            ind.set_pressed(False)
        self._mapped_display.setText("—")

        self.gamepad_log.emit(tr("gp.gamepad_disconnected"))

    # ---- Input Polling (20 Hz) ----

    def _poll_input(self):
        joy = self._joystick
        if not joy or not joy.connected:
            self._disconnect_gamepad()
            return

        for i in range(LinuxJoystick.MAX_AXES):
            v = joy.axes[i]
            self._axis_bars[i].set_value(v)
            self._axis_labels[i].setText(f"{v:+.3f}")

        for i in range(LinuxJoystick.MAX_BUTTONS):
            self._btn_indicators[i].set_pressed(bool(joy.buttons[i]))

        if self._detection:
            prof = self._detection.profile
            s = prof.sticks
            parts = []
            parts.append(f"LX={s.lx.read(joy.axes):+.2f}")
            parts.append(f"LY={s.ly.read(joy.axes):+.2f}")
            parts.append(f"RX={s.rx.read(joy.axes):+.2f}")
            parts.append(f"RY={s.ry.read(joy.axes):+.2f}")
            parts.append(f"LT={s.lt.read(joy.axes, joy.buttons):.2f}")
            parts.append(f"RT={s.rt.read(joy.axes, joy.buttons):.2f}")
            parts.append(f"DX={s.dpad_x.read(joy.axes):+.1f}")
            parts.append(f"DY={s.dpad_y.read(joy.axes):+.1f}")
            self._mapped_display.setText("  ".join(parts))

        if self._ctrl_running and self._controller:
            ctrl = self._controller
            idx = ctrl._speed_idx
            specs = self._speed_level_specs()
            name, _ = specs[idx]
            self._speed_label.setText(f"{idx+1}/5 [{name}]")

            sc = self._scene()
            if ctrl._zero_torque:
                self._mode_label.setText(tr("gp.mode_zt"))
                self._mode_label.setStyleSheet(
                    f"color: {sc['mode_zt']}; font-weight: bold;")
            elif ctrl._estop:
                self._mode_label.setText(tr("gp.mode_estop"))
                self._mode_label.setStyleSheet(
                    f"color: {sc['mode_estop']}; font-weight: bold;")
            else:
                self._mode_label.setText(tr("gp.mode_normal"))
                self._mode_label.setStyleSheet(
                    f"color: {sc['mode_normal']}; font-weight: bold;")

            if ctrl._target_pose:
                p = ctrl._target_pose
                self._pose_label.setText(
                    f"XYZ: ({p.x * M_TO_CM:.2f}, {p.y * M_TO_CM:.2f}, {p.z * M_TO_CM:.2f}) cm\n"
                    f"RPY: ({p.rx:.2f}, {p.ry:.2f}, {p.rz:.2f}) rad"
                )

            if ctrl.exit_requested:
                self._stop_control()
                self.gamepad_log.emit(tr("gp.ctrl_exit"))

    # ---- Arm Control ----

    def set_arm(self, arm):
        self._arm = arm
        self._update_ctrl_btn_state()

    def _update_ctrl_btn_state(self):
        can_start = (
            self._joystick is not None
            and self._joystick.connected
            and self._arm is not None
            and not self._ctrl_running
        )
        self._ctrl_btn.setEnabled(can_start or self._ctrl_running)
        if self._ctrl_running:
            self._ctrl_btn.setText(tr("gp.stop_ctrl"))
            self._ctrl_btn.setObjectName("disconnectBtn")
        else:
            self._ctrl_btn.setText(tr("gp.start_ctrl"))
            self._ctrl_btn.setObjectName("enableBtn")
        self._ctrl_btn.setStyle(self._ctrl_btn.style())

    def _toggle_control(self):
        if self._ctrl_running:
            self._stop_control()
        else:
            self._start_control()

    def _start_control(self):
        if not self._joystick or not self._joystick.connected:
            self.gamepad_log.emit(tr("gp.connect_first"))
            return
        if not self._arm:
            self.gamepad_log.emit(tr("gp.connect_arm_first"))
            return
        if not self._detection:
            return

        from demo.xbox_control import XboxArmController

        self._controller = XboxArmController(
            arm=self._arm,
            joystick=self._joystick,
            profile=self._detection.profile,
            update_rate=100.0,
        )

        self._ctrl_running = True
        self._ctrl_thread = threading.Thread(
            target=self._run_control, daemon=True, name="gamepad_arm_ctrl"
        )
        self._ctrl_thread.start()
        self._update_ctrl_btn_state()
        self.gamepad_log.emit(tr("gp.ctrl_started"))

    def _run_control(self):
        try:
            self._controller.start()
        except Exception as e:
            err_s = str(e)
            logger.error(tr("gp.ctrl_error", e=err_s))
            self.gamepad_log.emit(tr("gp.ctrl_error", e=err_s))
        finally:
            self._ctrl_running = False

    def _stop_control(self):
        if self._controller:
            self._controller.stop()
        if self._ctrl_thread:
            self._ctrl_thread.join(timeout=2.0)
            self._ctrl_thread = None
        self._controller = None
        self._ctrl_running = False
        self._update_ctrl_btn_state()

        self._mode_label.setText("—")
        self._mode_label.setStyleSheet("")
        self._pose_label.setText("—")
        specs = self._speed_level_specs()
        self._speed_label.setText(
            f"{self._speed_idx+1}/5 [{specs[self._speed_idx][0]}]")

        self.gamepad_log.emit(tr("gp.ctrl_stopped"))

    # ---- Cleanup ----

    def cleanup(self):
        if self._ctrl_running:
            self._stop_control()
        if self._joystick and self._joystick.connected:
            self._joystick.disconnect()
        self._poll_timer.stop()
