"""夹爪控制面板"""

import math
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QSlider, QDoubleSpinBox, QPushButton, QGroupBox,
)
from PyQt6.QtCore import pyqtSignal, Qt

from MotorStudio.utils.i18n import tr
from MotorStudio.utils.theme_manager import ThemeManager
from MotorStudio.utils.style import SCENE_COLORS


class GripperPanel(QWidget):
    """夹爪控制：角度滑块、全开/全关、设零"""

    gripper_command = pyqtSignal(float)  # angle in rad
    set_zero_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._updating = False
        self._init_ui()

    def _scene(self):
        return SCENE_COLORS[ThemeManager.instance().theme]

    def _apply_feedback_label_styles(self):
        sc = self._scene()
        self.fb_label.setStyleSheet(
            f"color: {sc['accent']}; font-weight: bold;")
        self.torque_label.setStyleSheet(f"color: {sc['warning']};")

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        self.group = QGroupBox()
        g_layout = QVBoxLayout()

        angle_layout = QHBoxLayout()
        self._lbl_angle = QLabel()
        angle_layout.addWidget(self._lbl_angle)
        self.angle_slider = QSlider(Qt.Orientation.Horizontal)
        self.angle_slider.setRange(-900, 900)
        self.angle_slider.setValue(0)
        self.angle_slider.valueChanged.connect(self._on_slider_changed)
        angle_layout.addWidget(self.angle_slider)

        self.angle_spin = QDoubleSpinBox()
        self.angle_spin.setRange(-90.0, 90.0)
        self.angle_spin.setDecimals(1)
        self.angle_spin.setSuffix("°")
        self.angle_spin.setFixedWidth(100)
        self.angle_spin.valueChanged.connect(self._on_spin_changed)
        angle_layout.addWidget(self.angle_spin)
        g_layout.addLayout(angle_layout)

        fb_layout = QHBoxLayout()
        self._lbl_actual = QLabel()
        fb_layout.addWidget(self._lbl_actual)
        self.fb_label = QLabel("0.0°")
        fb_layout.addWidget(self.fb_label)
        fb_layout.addSpacing(20)
        self._lbl_torque = QLabel()
        fb_layout.addWidget(self._lbl_torque)
        self.torque_label = QLabel("0.00 Nm")
        fb_layout.addWidget(self.torque_label)
        fb_layout.addStretch()
        g_layout.addLayout(fb_layout)

        btn_layout = QHBoxLayout()
        self.send_btn = QPushButton()
        self.send_btn.setObjectName("enableBtn")
        self.send_btn.clicked.connect(self._send_command)
        btn_layout.addWidget(self.send_btn)

        self.open_btn = QPushButton()
        self.open_btn.clicked.connect(lambda: self._set_angle(90.0))
        btn_layout.addWidget(self.open_btn)

        self.close_btn = QPushButton()
        self.close_btn.clicked.connect(lambda: self._set_angle(0.0))
        btn_layout.addWidget(self.close_btn)

        self.zero_btn = QPushButton()
        self.zero_btn.clicked.connect(self.set_zero_requested.emit)
        btn_layout.addWidget(self.zero_btn)

        btn_layout.addStretch()
        g_layout.addLayout(btn_layout)

        self.group.setLayout(g_layout)
        layout.addWidget(self.group)
        layout.addStretch()

        self.retranslate_ui()

    def retranslate_ui(self):
        self.group.setTitle(tr("grip.group"))
        self._lbl_angle.setText(tr("grip.angle"))
        self._lbl_actual.setText(tr("grip.actual"))
        self._lbl_torque.setText(tr("grip.torque"))
        self.send_btn.setText(tr("grip.send"))
        self.open_btn.setText(tr("grip.open"))
        self.close_btn.setText(tr("grip.close"))
        self.zero_btn.setText(tr("grip.set_zero"))
        self._apply_feedback_label_styles()

    def _on_slider_changed(self, val):
        if self._updating:
            return
        self._updating = True
        self.angle_spin.setValue(val / 10.0)
        self._updating = False

    def _on_spin_changed(self, val):
        if self._updating:
            return
        self._updating = True
        self.angle_slider.setValue(int(val * 10))
        self._updating = False

    def _set_angle(self, deg):
        self._updating = True
        self.angle_spin.setValue(deg)
        self.angle_slider.setValue(int(deg * 10))
        self._updating = False
        self._send_command()

    def _send_command(self):
        angle_rad = math.radians(self.angle_spin.value())
        self.gripper_command.emit(angle_rad)

    def update_feedback(self, joint_states, effort_states=None):
        positions = joint_states.to_list(include_gripper=True)
        if len(positions) >= 7:
            deg = math.degrees(positions[6])
            self.fb_label.setText(f"{deg:.1f}°")
        if effort_states:
            torques = effort_states.to_list(include_gripper=True)
            if len(torques) >= 7:
                self.torque_label.setText(f"{torques[6]:.2f} Nm")
