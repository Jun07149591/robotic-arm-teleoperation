"""关节控制面板：7 关节滑块 + SpinBox 控制"""

import math
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QSlider, QDoubleSpinBox, QPushButton, QGroupBox,
    QGridLayout,
)
from PyQt6.QtCore import pyqtSignal, Qt

from MotorStudio.utils.style import JOINT_COLORS, SCENE_COLORS
from MotorStudio.utils.theme_manager import ThemeManager
from MotorStudio.utils.i18n import tr

JOINT_LIMITS_DEG = {
    1: (-160.0, 160.0),
    2: (0.0, 210.0),
    3: (-230.0, 0.0),
    4: (-90.0, 90.0),
    5: (-90.0, 90.0),
    6: (-90.0, 90.0),
    7: (-90.0, 90.0),
}

JOINT_NAMES = ["L1", "L2", "L3", "L4", "L5", "L6", "L7"]

SLIDER_RESOLUTION = 1000


class JointControlPanel(QWidget):
    """7 关节手动控制面板"""

    joint_command = pyqtSignal(list)  # 7 个弧度值
    go_zero_requested = pyqtSignal(list)  # 6 个弧度值（MoveJ 规划型回零）

    def __init__(self, parent=None):
        super().__init__(parent)
        self._updating_from_feedback = False
        self._enabled = False
        self._sliders = []
        self._spinboxes = []
        self._feedback_labels = []
        self._header_labels = []
        self._name_labels = []
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.setSpacing(1)

        grid = QGridLayout()
        grid.setSpacing(2)
        grid.setContentsMargins(0, 0, 0, 0)

        headers = [tr("jc.header_joint"), tr("jc.header_ctrl"), tr("jc.header_target"), tr("jc.header_actual")]
        for col, h in enumerate(headers):
            lbl = QLabel(h)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            sc = SCENE_COLORS[ThemeManager.instance().theme]
            lbl.setStyleSheet(f"font-weight: bold; color: {sc['header_text']};")
            lbl.setFixedHeight(20)
            grid.addWidget(lbl, 0, col)
            self._header_labels.append(lbl)

        for i in range(7):
            row = i + 1
            joint_id = i + 1
            lo, hi = JOINT_LIMITS_DEG[joint_id]

            name_label = QLabel(JOINT_NAMES[i])
            name_label.setStyleSheet(f"color: {JOINT_COLORS[i]}; font-weight: bold;")
            name_label.setFixedWidth(28)
            name_label.setFixedHeight(22)
            name_label.setToolTip(tr(f"jc.tooltip_L{i+1}"))
            grid.addWidget(name_label, row, 0)
            self._name_labels.append(name_label)

            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(0, SLIDER_RESOLUTION)
            mid = int((0.0 - lo) / (hi - lo) * SLIDER_RESOLUTION) if hi != lo else SLIDER_RESOLUTION // 2
            slider.setValue(max(0, min(SLIDER_RESOLUTION, mid)))
            slider.setFixedHeight(20)
            slider.valueChanged.connect(lambda v, idx=i: self._on_slider_changed(idx, v))
            self._sliders.append(slider)
            grid.addWidget(slider, row, 1)

            spinbox = QDoubleSpinBox()
            spinbox.setRange(lo, hi)
            spinbox.setDecimals(1)
            spinbox.setSingleStep(1.0)
            spinbox.setSuffix("°")
            spinbox.setFixedWidth(85)
            spinbox.setFixedHeight(24)
            spinbox.setValue(0.0 if lo <= 0.0 <= hi else lo)
            spinbox.valueChanged.connect(lambda v, idx=i: self._on_spinbox_changed(idx, v))
            self._spinboxes.append(spinbox)
            grid.addWidget(spinbox, row, 2)

            fb_label = QLabel("0.00°")
            fb_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            fb_label.setFixedWidth(65)
            fb_label.setFixedHeight(24)
            fb_label.setStyleSheet(f"color: {JOINT_COLORS[i]};")
            self._feedback_labels.append(fb_label)
            grid.addWidget(fb_label, row, 3)

        grid.setColumnStretch(0, 0)
        grid.setColumnStretch(1, 5)
        grid.setColumnStretch(2, 0)
        grid.setColumnStretch(3, 0)
        layout.addLayout(grid)

        btn_layout = QHBoxLayout()
        btn_layout.setContentsMargins(0, 2, 0, 0)
        self.zero_btn = QPushButton(tr("jc.go_zero"))
        self.zero_btn.setFixedHeight(26)
        self.zero_btn.clicked.connect(self._on_go_zero)
        btn_layout.addWidget(self.zero_btn)

        self.sync_btn = QPushButton(tr("jc.sync_pos"))
        self.sync_btn.setFixedHeight(26)
        self.sync_btn.clicked.connect(self._sync_to_feedback)
        btn_layout.addWidget(self.sync_btn)

        btn_layout.addStretch()
        self.send_btn = QPushButton(tr("jc.send"))
        self.send_btn.setObjectName("enableBtn")
        self.send_btn.setFixedHeight(26)
        self.send_btn.clicked.connect(self._emit_command)
        btn_layout.addWidget(self.send_btn)

        layout.addLayout(btn_layout)
        layout.addStretch()

    def retranslate_ui(self):
        sc = SCENE_COLORS[ThemeManager.instance().theme]
        headers = [tr("jc.header_joint"), tr("jc.header_ctrl"), tr("jc.header_target"), tr("jc.header_actual")]
        for i, lbl in enumerate(self._header_labels):
            lbl.setText(headers[i])
            lbl.setStyleSheet(f"font-weight: bold; color: {sc['header_text']};")
        self.zero_btn.setText(tr("jc.go_zero"))
        self.sync_btn.setText(tr("jc.sync_pos"))
        self.send_btn.setText(tr("jc.send"))
        tooltips = [tr(f"jc.tooltip_L{i+1}") for i in range(7)]
        for i, lbl in enumerate(self._name_labels):
            lbl.setToolTip(tooltips[i])

    def _slider_to_deg(self, joint_idx, slider_val):
        lo, hi = JOINT_LIMITS_DEG[joint_idx + 1]
        return lo + (hi - lo) * slider_val / SLIDER_RESOLUTION

    def _deg_to_slider(self, joint_idx, deg_val):
        lo, hi = JOINT_LIMITS_DEG[joint_idx + 1]
        if hi == lo:
            return SLIDER_RESOLUTION // 2
        return int((deg_val - lo) / (hi - lo) * SLIDER_RESOLUTION)

    def _on_slider_changed(self, idx, val):
        if self._updating_from_feedback:
            return
        deg = self._slider_to_deg(idx, val)
        self._updating_from_feedback = True
        self._spinboxes[idx].setValue(deg)
        self._updating_from_feedback = False

    def _on_spinbox_changed(self, idx, val):
        if self._updating_from_feedback:
            return
        self._updating_from_feedback = True
        self._sliders[idx].setValue(self._deg_to_slider(idx, val))
        self._updating_from_feedback = False

    def _emit_command(self):
        positions_rad = []
        for i in range(7):
            deg = self._spinboxes[i].value()
            positions_rad.append(math.radians(deg))
        self.joint_command.emit(positions_rad)

    def _on_go_zero(self):
        self._updating_from_feedback = True
        positions_rad = []
        for i in range(7):
            lo, hi = JOINT_LIMITS_DEG[i + 1]
            zero_val = 0.0 if lo <= 0.0 <= hi else lo
            self._spinboxes[i].setValue(zero_val)
            self._sliders[i].setValue(self._deg_to_slider(i, zero_val))
            if i < 6:
                positions_rad.append(math.radians(zero_val))
        self._updating_from_feedback = False
        self.go_zero_requested.emit(positions_rad)

    def _sync_to_feedback(self):
        """将控制滑块同步到当前反馈位置"""
        self._updating_from_feedback = True
        for i in range(7):
            text = self._feedback_labels[i].text().replace("°", "")
            try:
                deg = float(text)
                self._spinboxes[i].setValue(deg)
                self._sliders[i].setValue(self._deg_to_slider(i, deg))
            except ValueError:
                pass
        self._updating_from_feedback = False

    def update_feedback(self, joint_states):
        """从 ArmJointStates 更新反馈显示"""
        positions = joint_states.to_list(include_gripper=True)
        for i in range(min(7, len(positions))):
            deg = math.degrees(positions[i])
            self._feedback_labels[i].setText(f"{deg:.2f}°")

    def set_enabled(self, enabled: bool):
        self._enabled = enabled
        for s in self._sliders:
            s.setEnabled(enabled)
        for s in self._spinboxes:
            s.setEnabled(enabled)
        self.send_btn.setEnabled(enabled)
        self.zero_btn.setEnabled(enabled)
