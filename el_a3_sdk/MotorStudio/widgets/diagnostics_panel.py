"""诊断面板：电机参数、CAN 总线统计、参数读写"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QGroupBox, QGridLayout,
    QTableWidget, QTableWidgetItem, QHeaderView,
    QSpinBox, QComboBox, QDoubleSpinBox,
)
from PyQt6.QtCore import pyqtSignal, Qt

from MotorStudio.utils.i18n import tr
from MotorStudio.utils.theme_manager import ThemeManager
from MotorStudio.utils.style import SCENE_COLORS

PARAM_ORDER = [
    (0x7005, "diag.p_7005"),
    (0x7006, "diag.p_7006"),
    (0x700A, "diag.p_700a"),
    (0x700B, "diag.p_700b"),
    (0x7010, "diag.p_7010"),
    (0x7011, "diag.p_7011"),
    (0x7016, "diag.p_7016"),
    (0x7017, "diag.p_7017"),
    (0x7018, "diag.p_7018"),
    (0x7019, "diag.p_7019"),
    (0x701A, "diag.p_701a"),
    (0x701B, "diag.p_701b"),
    (0x701C, "diag.p_701c"),
    (0x701E, "diag.p_701e"),
    (0x701F, "diag.p_701f"),
    (0x7020, "diag.p_7020"),
]


class DiagnosticsPanel(QWidget):
    """电机诊断与 CAN 总线监控"""

    read_param_requested = pyqtSignal(int, int)   # motor_id, param_index
    write_param_requested = pyqtSignal(int, int, float)  # motor_id, param_index, value
    set_zero_requested = pyqtSignal(int)  # motor_num
    verify_zero_sta_requested = pyqtSignal()
    set_all_zero_sta_requested = pyqtSignal()
    scan_motors_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._last_scan_results = None
        self._last_verify_results = None
        self._last_scan_online_count = 0
        self._init_ui()

    def _scene(self):
        return SCENE_COLORS[ThemeManager.instance().theme]

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        self.motor_group = QGroupBox()
        motor_layout = QVBoxLayout()

        self.motor_table = QTableWidget(7, 8)
        self.motor_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        self.motor_table.setEditTriggers(
            QTableWidget.EditTrigger.NoEditTriggers
        )
        for i in range(7):
            self.motor_table.setItem(i, 0, QTableWidgetItem(str(i + 1)))
            for j in range(1, 8):
                self.motor_table.setItem(i, j, QTableWidgetItem("--"))
        self.motor_table.setMaximumHeight(200)
        self.motor_table.verticalHeader().setDefaultSectionSize(24)
        self.motor_table.verticalHeader().setVisible(False)
        motor_layout.addWidget(self.motor_table)
        self.motor_group.setLayout(motor_layout)
        layout.addWidget(self.motor_group)

        self.scan_group = QGroupBox()
        scan_vlayout = QVBoxLayout()
        scan_vlayout.setSpacing(4)

        scan_row = QHBoxLayout()
        self.scan_btn = QPushButton()
        self.scan_btn.clicked.connect(self.scan_motors_requested.emit)
        scan_row.addWidget(self.scan_btn)
        self.scan_status = QLabel()
        self.scan_status.setStyleSheet("font-weight: bold;")
        scan_row.addWidget(self.scan_status)
        scan_row.addStretch()
        scan_vlayout.addLayout(scan_row)

        self.scan_table = QTableWidget(7, 4)
        self.scan_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        self.scan_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        for i in range(7):
            self.scan_table.setItem(i, 0, QTableWidgetItem(str(i + 1)))
            for j in range(1, 4):
                self.scan_table.setItem(i, j, QTableWidgetItem("—"))
        self.scan_table.setMaximumHeight(200)
        self.scan_table.verticalHeader().setDefaultSectionSize(24)
        self.scan_table.verticalHeader().setVisible(False)
        scan_vlayout.addWidget(self.scan_table)

        self.scan_group.setLayout(scan_vlayout)
        layout.addWidget(self.scan_group)

        self.param_group = QGroupBox()
        param_vlayout = QVBoxLayout()
        param_vlayout.setSpacing(4)

        row1 = QHBoxLayout()
        self._lbl_motor_id = QLabel()
        row1.addWidget(self._lbl_motor_id)
        self.motor_id_spin = QSpinBox()
        self.motor_id_spin.setRange(1, 7)
        self.motor_id_spin.setFixedWidth(50)
        row1.addWidget(self.motor_id_spin)
        row1.addSpacing(8)
        self._lbl_param = QLabel()
        row1.addWidget(self._lbl_param)
        self.param_combo = QComboBox()
        row1.addWidget(self.param_combo, 1)
        param_vlayout.addLayout(row1)

        row2 = QHBoxLayout()
        self.read_btn = QPushButton()
        self.read_btn.setFixedWidth(60)
        self.read_btn.clicked.connect(self._on_read)
        row2.addWidget(self.read_btn)
        row2.addSpacing(8)
        self._lbl_value = QLabel()
        row2.addWidget(self._lbl_value)
        self.value_spin = QDoubleSpinBox()
        self.value_spin.setRange(-10000, 10000)
        self.value_spin.setDecimals(4)
        self.value_spin.setFixedWidth(120)
        row2.addWidget(self.value_spin)
        row2.addSpacing(8)
        self.write_btn = QPushButton()
        self.write_btn.setFixedWidth(60)
        self.write_btn.clicked.connect(self._on_write)
        row2.addWidget(self.write_btn)
        row2.addStretch()
        param_vlayout.addLayout(row2)

        self.param_group.setLayout(param_vlayout)
        layout.addWidget(self.param_group)

        self.verify_group = QGroupBox()
        verify_layout = QVBoxLayout()
        verify_layout.setSpacing(4)

        verify_row = QHBoxLayout()
        self.verify_btn = QPushButton()
        self.verify_btn.clicked.connect(self.verify_zero_sta_requested.emit)
        verify_row.addWidget(self.verify_btn)

        self.set_all_zero_sta_btn = QPushButton()
        self.set_all_zero_sta_btn.setMinimumWidth(120)
        self.set_all_zero_sta_btn.clicked.connect(
            self.set_all_zero_sta_requested.emit)
        verify_row.addWidget(self.set_all_zero_sta_btn)

        self.verify_status = QLabel()
        self.verify_status.setStyleSheet("font-weight: bold;")
        verify_row.addWidget(self.verify_status)
        verify_row.addStretch()
        verify_layout.addLayout(verify_row)

        self._verify_motor_labels: list[QLabel] = []
        self._verify_labels: list[QLabel] = []
        verify_grid = QGridLayout()
        verify_grid.setSpacing(2)
        for i in range(7):
            mid_lbl = QLabel()
            mid_lbl.setFixedWidth(50)
            val_lbl = QLabel("—")
            val_lbl.setStyleSheet("font-family: monospace;")
            verify_grid.addWidget(mid_lbl, i // 4, (i % 4) * 2)
            verify_grid.addWidget(val_lbl, i // 4, (i % 4) * 2 + 1)
            self._verify_motor_labels.append(mid_lbl)
            self._verify_labels.append(val_lbl)
        verify_layout.addLayout(verify_grid)

        self.verify_group.setLayout(verify_layout)
        layout.addWidget(self.verify_group)

        self.op_group = QGroupBox()
        op_layout = QHBoxLayout()

        zero_layout = QHBoxLayout()
        self._lbl_zero_motor = QLabel()
        zero_layout.addWidget(self._lbl_zero_motor)
        self.zero_motor_spin = QSpinBox()
        self.zero_motor_spin.setRange(1, 7)
        zero_layout.addWidget(self.zero_motor_spin)
        self.zero_btn = QPushButton()
        self.zero_btn.clicked.connect(
            lambda: self.set_zero_requested.emit(self.zero_motor_spin.value())
        )
        zero_layout.addWidget(self.zero_btn)

        self.zero_all_btn = QPushButton()
        self.zero_all_btn.clicked.connect(lambda: self.set_zero_requested.emit(0xFF))
        zero_layout.addWidget(self.zero_all_btn)

        op_layout.addLayout(zero_layout)
        self.op_group.setLayout(op_layout)
        layout.addWidget(self.op_group)

        self.can_group = QGroupBox()
        can_layout = QHBoxLayout()
        self.can_fps_label = QLabel("FPS: --")
        can_layout.addWidget(self.can_fps_label)
        self.can_tx_label = QLabel("TX: --")
        can_layout.addWidget(self.can_tx_label)
        self.can_state_label = QLabel()
        can_layout.addWidget(self.can_state_label)
        self.can_group.setLayout(can_layout)
        layout.addWidget(self.can_group)

        self.retranslate_ui()

    def _rebuild_param_combo(self):
        cur = self.param_combo.currentData()
        self.param_combo.clear()
        for idx, key in PARAM_ORDER:
            self.param_combo.addItem(f"0x{idx:04X} - {tr(key)}", idx)
        if cur is not None:
            for i in range(self.param_combo.count()):
                if self.param_combo.itemData(i) == cur:
                    self.param_combo.setCurrentIndex(i)
                    break

    def retranslate_ui(self):
        self.motor_group.setTitle(tr("diag.motor_group"))
        self.motor_table.setHorizontalHeaderLabels([
            tr("diag.h_id"), tr("diag.h_pos"), tr("diag.h_vel"), tr("diag.h_torque"),
            tr("diag.h_temp"), tr("diag.h_fault"), tr("diag.h_mode"), tr("diag.h_enable"),
        ])
        self.scan_group.setTitle(tr("diag.scan_group"))
        self.scan_btn.setText(tr("diag.scan_btn"))
        self.scan_btn.setToolTip(tr("diag.scan_tip"))
        self.scan_table.setHorizontalHeaderLabels([
            tr("diag.sh_id"), tr("diag.sh_status"), tr("diag.sh_fw"), tr("diag.sh_vbus"),
        ])
        self.param_group.setTitle(tr("diag.param_group"))
        self._lbl_motor_id.setText(tr("diag.motor_id"))
        self._lbl_param.setText(tr("diag.param"))
        self.read_btn.setText(tr("diag.read"))
        self._lbl_value.setText(tr("diag.value"))
        self.write_btn.setText(tr("diag.write"))
        self.verify_group.setTitle(tr("diag.verify_group"))
        self.verify_btn.setText(tr("diag.verify_btn"))
        self.verify_btn.setToolTip(tr("diag.verify_tip"))
        self.set_all_zero_sta_btn.setText(tr("diag.set_all_zero_sta"))
        self.set_all_zero_sta_btn.setToolTip(tr("diag.set_all_zero_sta_tip"))
        for i, lbl in enumerate(self._verify_motor_labels):
            lbl.setText(tr("diag.motor_n", n=i + 1))
        self.op_group.setTitle(tr("diag.op_group"))
        self._lbl_zero_motor.setText(tr("diag.zero_motor"))
        self.zero_btn.setText(tr("diag.set_zero"))
        self.zero_all_btn.setText(tr("diag.set_zero_all"))
        self.can_group.setTitle(tr("diag.can_group"))
        self.can_state_label.setText(tr("diag.state"))
        self._rebuild_param_combo()

        if self._last_scan_results is not None:
            self.update_scan_result(self._last_scan_results)
        else:
            self.scan_status.setText(tr("diag.not_scanned"))
            self.scan_status.setStyleSheet("font-weight: bold;")

        if self._last_verify_results is not None:
            self.update_zero_sta_result(self._last_verify_results)
        else:
            self.verify_status.setText(tr("diag.not_verified"))
            self.verify_status.setStyleSheet("font-weight: bold;")
            for lbl in self._verify_labels:
                lbl.setText("—")
                lbl.setStyleSheet("font-family: monospace;")

    def _on_read(self):
        motor_id = self.motor_id_spin.value()
        param_idx = self.param_combo.currentData()
        if param_idx is not None:
            self.read_param_requested.emit(motor_id, param_idx)

    def _on_write(self):
        motor_id = self.motor_id_spin.value()
        param_idx = self.param_combo.currentData()
        value = self.value_spin.value()
        if param_idx is not None:
            self.write_param_requested.emit(motor_id, param_idx, value)

    def update_motor_states(self, joint_pos, joint_vel=None, joint_eff=None):
        """更新电机状态表格"""
        pos_list = joint_pos.to_list(include_gripper=True) if joint_pos else [0]*7
        vel_list = joint_vel.to_list(include_gripper=True) if joint_vel else [0]*7
        eff_list = joint_eff.to_list(include_gripper=True) if joint_eff else [0]*7
        for i in range(7):
            self.motor_table.item(i, 1).setText(f"{pos_list[i]:.4f}")
            self.motor_table.item(i, 2).setText(f"{vel_list[i]:.4f}")
            self.motor_table.item(i, 3).setText(f"{eff_list[i]:.4f}")

    def update_motor_feedback(self, feedbacks):
        """从 MotorFeedback 列表更新温度/故障等"""
        for fb in feedbacks:
            if hasattr(fb, 'motor_id'):
                row = fb.motor_id - 1
                if 0 <= row < 7:
                    if hasattr(fb, 'temperature'):
                        self.motor_table.item(row, 4).setText(f"{fb.temperature:.1f}")
                    if hasattr(fb, 'fault_code'):
                        self.motor_table.item(row, 5).setText(f"0x{fb.fault_code:02X}")
                    if hasattr(fb, 'mode_state'):
                        self.motor_table.item(row, 6).setText(str(fb.mode_state))
                    if hasattr(fb, 'is_valid'):
                        self.motor_table.item(row, 7).setText(
                            tr("diag.yes") if fb.is_valid else tr("diag.no"))

    def update_scan_result(self, results: list):
        """更新电机扫描结果: [(motor_id, online, firmware_str, voltage), ...]"""
        self._last_scan_results = list(results)
        sc = self._scene()
        online_count = 0
        for motor_id, online, fw_str, voltage in results:
            row = motor_id - 1
            if row < 0 or row >= 7:
                continue
            if online:
                online_count += 1
                self.scan_table.item(row, 1).setText(tr("diag.online"))
                self.scan_table.item(row, 1).setForeground(
                    self.scan_table.item(row, 1).foreground())
                item_status = self.scan_table.item(row, 1)
                item_status.setText(tr("diag.online"))
                self.scan_table.item(row, 2).setText(fw_str if fw_str else "—")
                self.scan_table.item(row, 3).setText(
                    f"{voltage:.1f}" if voltage is not None else "—")
            else:
                item_status = self.scan_table.item(row, 1)
                item_status.setText(tr("diag.offline"))
                self.scan_table.item(row, 2).setText("—")
                self.scan_table.item(row, 3).setText("—")

        self._last_scan_online_count = online_count
        self.scan_status.setText(tr("diag.scan_result", n=online_count))
        if online_count == 7:
            self.scan_status.setStyleSheet(
                f"font-weight: bold; color: {sc['success']};")
        elif online_count > 0:
            self.scan_status.setStyleSheet(
                f"font-weight: bold; color: {sc['warning']};")
        else:
            self.scan_status.setStyleSheet(
                f"font-weight: bold; color: {sc['error']};")

    def update_zero_sta_result(self, results: list):
        """更新 ZERO_STA 校验结果: [(motor_id, value, success), ...]"""
        self._last_verify_results = list(results)
        sc = self._scene()
        all_ok = True
        for motor_id, value, success in results:
            idx = motor_id - 1
            if 0 <= idx < len(self._verify_labels):
                if not success:
                    self._verify_labels[idx].setText(tr("diag.read_fail"))
                    self._verify_labels[idx].setStyleSheet(
                        f"font-family: monospace; color: {sc['error']}; font-weight: bold;")
                    all_ok = False
                elif value == 1:
                    self._verify_labels[idx].setText(f"{value} ✓")
                    self._verify_labels[idx].setStyleSheet(
                        f"font-family: monospace; color: {sc['success']}; font-weight: bold;")
                else:
                    self._verify_labels[idx].setText(f"{value} ✗")
                    self._verify_labels[idx].setStyleSheet(
                        f"font-family: monospace; color: {sc['error']}; font-weight: bold;")
                    all_ok = False

        if all_ok:
            self.verify_status.setText(tr("diag.all_pass"))
            self.verify_status.setStyleSheet(
                f"font-weight: bold; color: {sc['success']};")
        else:
            self.verify_status.setText(tr("diag.has_error"))
            self.verify_status.setStyleSheet(
                f"font-weight: bold; color: {sc['error']};")

    def update_can_stats(self, fps: float):
        self.can_fps_label.setText(f"FPS: {fps:.0f}")
