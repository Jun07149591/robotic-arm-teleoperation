"""TCP / 末端点偏移设置页面。"""

from __future__ import annotations

import math

from PyQt6.QtCore import QSignalBlocker, pyqtSignal
from PyQt6.QtWidgets import (
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from MotorStudio.utils.i18n import tr
from MotorStudio.utils.tcp_offset_store import load_tcp_offset, normalize_tcp_offset

M_TO_CM = 100.0
CM_TO_M = 0.01
UI_LINEAR_TO_SDK_INDEX = (2, 0, 1)


class TcpPanel(QWidget):
    """末端点偏移配置面板。"""

    tcp_apply_requested = pyqtSignal(list)
    tcp_save_requested = pyqtSignal(list)
    tcp_restore_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._labels = []
        self._spins = []
        self._tcp_offset = load_tcp_offset()
        self._init_ui()
        self.set_tcp_offset(self._tcp_offset)

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(6)

        self.group = QGroupBox(tr("tcp.group"))
        grid = QGridLayout()
        grid.setContentsMargins(6, 6, 6, 6)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(6)

        fields = [
            ("tcp.x", True),
            ("tcp.y", True),
            ("tcp.z", True),
            ("tcp.rx", False),
            ("tcp.ry", False),
            ("tcp.rz", False),
        ]
        for idx, (label_key, is_linear) in enumerate(fields):
            row = idx // 3
            col = (idx % 3) * 2

            label = QLabel(tr(label_key))
            self._labels.append(label)
            grid.addWidget(label, row, col)

            spin = QDoubleSpinBox()
            if is_linear:
                spin.setRange(-30.0, 30.0)
                spin.setDecimals(2)
                spin.setSingleStep(0.1)
                spin.setSuffix(" cm")
            else:
                spin.setRange(-180.0, 180.0)
                spin.setDecimals(2)
                spin.setSingleStep(1.0)
                spin.setSuffix("°")
            spin.setFixedWidth(110)
            self._spins.append(spin)
            grid.addWidget(spin, row, col + 1)

        self.group.setLayout(grid)
        layout.addWidget(self.group)

        btn_row = QHBoxLayout()
        btn_row.setContentsMargins(0, 0, 0, 0)

        self.apply_btn = QPushButton(tr("tcp.apply"))
        self.apply_btn.setObjectName("enableBtn")
        self.apply_btn.clicked.connect(self._on_apply)
        btn_row.addWidget(self.apply_btn)

        self.save_btn = QPushButton(tr("tcp.save"))
        self.save_btn.clicked.connect(self._on_save)
        btn_row.addWidget(self.save_btn)

        self.restore_btn = QPushButton(tr("tcp.restore"))
        self.restore_btn.clicked.connect(self._on_restore)
        btn_row.addWidget(self.restore_btn)

        btn_row.addStretch()
        layout.addLayout(btn_row)
        layout.addStretch()

    def retranslate_ui(self):
        self.group.setTitle(tr("tcp.group"))
        keys = ["tcp.x", "tcp.y", "tcp.z", "tcp.rx", "tcp.ry", "tcp.rz"]
        for label, key in zip(self._labels, keys):
            label.setText(tr(key))
        self.apply_btn.setText(tr("tcp.apply"))
        self.save_btn.setText(tr("tcp.save"))
        self.restore_btn.setText(tr("tcp.restore"))

    def get_tcp_offset(self):
        return self._collect_offset()

    def set_tcp_offset(self, offset):
        values = normalize_tcp_offset(offset)
        self._tcp_offset = list(values)
        display_values = [
            values[UI_LINEAR_TO_SDK_INDEX[0]],
            values[UI_LINEAR_TO_SDK_INDEX[1]],
            values[UI_LINEAR_TO_SDK_INDEX[2]],
            values[3],
            values[4],
            values[5],
        ]
        for idx, (spin, value) in enumerate(zip(self._spins, display_values)):
            blocker = QSignalBlocker(spin)
            if idx < 3:
                spin.setValue(float(value) * M_TO_CM)
            else:
                spin.setValue(math.degrees(float(value)))
            del blocker

    def _collect_offset(self):
        values = [0.0] * 6
        linear_values = [
            self._spins[0].value() * CM_TO_M,
            self._spins[1].value() * CM_TO_M,
            self._spins[2].value() * CM_TO_M,
        ]
        for ui_idx, sdk_idx in enumerate(UI_LINEAR_TO_SDK_INDEX):
            values[sdk_idx] = linear_values[ui_idx]
        values[3:] = [
            math.radians(self._spins[3].value()),
            math.radians(self._spins[4].value()),
            math.radians(self._spins[5].value()),
        ]
        return values

    def _on_save(self):
        self._tcp_offset = self._collect_offset()
        self.tcp_save_requested.emit(list(self._tcp_offset))

    def _on_restore(self):
        self.set_tcp_offset([0.0] * 6)
        self._tcp_offset = [0.0] * 6
        self.tcp_restore_requested.emit()

    def _on_apply(self):
        self._tcp_offset = self._collect_offset()
        self.tcp_apply_requested.emit(list(self._tcp_offset))
