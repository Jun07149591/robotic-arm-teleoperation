"""轨迹控制面板：MoveJ / MoveL / 路径点管理 — 含 3D 拖拽模式控制"""

import math
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QDoubleSpinBox, QPushButton, QGroupBox, QGridLayout,
    QTableWidget, QTableWidgetItem, QHeaderView, QTabWidget,
)
from PyQt6.QtCore import pyqtSignal, Qt, QTimer

from MotorStudio.utils.i18n import tr
from MotorStudio.utils.theme_manager import ThemeManager
from MotorStudio.utils.style import SCENE_COLORS

JOINT_NAMES_ORDERED = [
    "L1_joint", "L2_joint", "L3_joint",
    "L4_joint", "L5_joint", "L6_joint", "L7_joint",
]

M_TO_CM = 100.0
CM_TO_M = 0.01

MOVEL_DEBUG_JOINTS_DEG = [0.0, 35.0, -35.0, 0.0, -20.0, 0.0]
MOVEL_DEBUG_POSE = [-15.0, 0.0, 25.0, 90.0, 0.0, 90.0]


class TrajectoryPanel(QWidget):
    """轨迹控制：MoveJ、MoveL、路径点"""

    move_j_requested = pyqtSignal(list, float)
    move_l_requested = pyqtSignal(list, float)
    cancel_requested = pyqtSignal()
    request_end_pose = pyqtSignal()  # 请求 worker 同步查询一次末端位姿

    drag_mode_toggled = pyqtSignal(bool)
    sync_feedback_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._current_positions = [0.0] * 6
        self._current_end_pose = None
        self._movel_pose_initialized = False
        self._waypoints = []
        self._status_mode = "ready"
        self._drag_active = False
        self._updating_from_drag = False
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        self.tabs = QTabWidget()

        self.tabs.addTab(self._create_movej_tab(), "MoveJ")
        self.tabs.addTab(self._create_movel_tab(), "MoveL")
        self.tabs.addTab(self._create_waypoint_tab(), tr("traj.waypoints"))

        layout.addWidget(self.tabs)

        btn_layout = QHBoxLayout()
        self.cancel_btn = QPushButton(tr("traj.cancel"))
        sc = SCENE_COLORS[ThemeManager.instance().theme]
        self.cancel_btn.setStyleSheet(
            f"background-color: {sc['cancel_bg']}; color: white; border-radius: 6px;"
        )
        self.cancel_btn.clicked.connect(self.cancel_requested.emit)
        btn_layout.addWidget(self.cancel_btn)

        self.status_label = QLabel(tr("traj.ready"))
        btn_layout.addWidget(self.status_label)
        btn_layout.addStretch()
        layout.addLayout(btn_layout)

    def _create_movej_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)

        # --- 拖拽控制行 ---
        drag_row = QHBoxLayout()
        self.drag_btn = QPushButton(tr("v3d.drag_mode"))
        self.drag_btn.setCheckable(True)
        self.drag_btn.setToolTip(tr("v3d.drag_tip"))
        self.drag_btn.clicked.connect(self._on_drag_toggled)
        drag_row.addWidget(self.drag_btn)

        self.sync_btn = QPushButton(tr("v3d.sync_pos"))
        self.sync_btn.setToolTip(tr("v3d.sync_tip"))
        self.sync_btn.clicked.connect(self._on_sync_clicked)
        drag_row.addWidget(self.sync_btn)

        self.read_btn = QPushButton(tr("traj.read_pos"))
        self.read_btn.setToolTip(tr("v3d.sync_tip"))
        self.read_btn.clicked.connect(self._fill_current_positions)
        drag_row.addWidget(self.read_btn)

        drag_row.addStretch()
        layout.addLayout(drag_row)

        # --- 关节目标 ---
        self.movej_group = QGroupBox(tr("traj.joint_target"))
        grid = QGridLayout()
        self._movej_spins = []
        joint_names = ["L1", "L2", "L3", "L4", "L5", "L6"]
        limits = [(-160, 160), (0, 210), (-230, 0), (-90, 90), (-90, 90), (-90, 90)]
        for i in range(6):
            grid.addWidget(QLabel(joint_names[i]), i // 3, (i % 3) * 2)
            spin = QDoubleSpinBox()
            lo, hi = limits[i]
            spin.setRange(lo, hi)
            spin.setDecimals(2)
            spin.setSuffix("°")
            spin.setValue(0.0 if lo <= 0 <= hi else lo)
            self._movej_spins.append(spin)
            grid.addWidget(spin, i // 3, (i % 3) * 2 + 1)
        self.movej_group.setLayout(grid)
        layout.addWidget(self.movej_group)

        # --- 时长 + 执行 ---
        dur_layout = QHBoxLayout()
        self.movej_dur_label = QLabel(tr("traj.duration"))
        dur_layout.addWidget(self.movej_dur_label)
        self.movej_duration = QDoubleSpinBox()
        self.movej_duration.setRange(0.5, 30.0)
        self.movej_duration.setValue(2.0)
        self.movej_duration.setSuffix(" s")
        dur_layout.addWidget(self.movej_duration)
        dur_layout.addStretch()

        self.movej_exec_btn = QPushButton(tr("traj.exec_movej"))
        self.movej_exec_btn.setObjectName("enableBtn")
        self.movej_exec_btn.clicked.connect(self._on_exec_movej)
        dur_layout.addWidget(self.movej_exec_btn)
        layout.addLayout(dur_layout)

        layout.addStretch()
        return w

    def _create_movel_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)

        pose_row = QHBoxLayout()
        self.read_pose_btn = QPushButton(tr("traj.read_pose"))
        self.read_pose_btn.setToolTip(tr("traj.read_pose_tip"))
        self.read_pose_btn.clicked.connect(self._fill_current_end_pose)
        pose_row.addWidget(self.read_pose_btn)

        self.movel_debug_btn = QPushButton(tr("traj.move_debug"))
        self.movel_debug_btn.setToolTip(tr("traj.move_debug_tip"))
        self.movel_debug_btn.clicked.connect(self._move_to_debug_pose)
        pose_row.addWidget(self.movel_debug_btn)
        pose_row.addStretch()
        layout.addLayout(pose_row)

        self.movel_group = QGroupBox(tr("traj.cart_target"))
        grid = QGridLayout()
        labels = ["X (cm)", "Y (cm)", "Z (cm)", "Rx (°)", "Ry (°)", "Rz (°)"]
        defaults = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ranges = [(-100, 100), (-100, 100), (0, 100), (-180, 180), (-180, 180), (-180, 180)]
        self._movel_spins = []
        for i, (label, default, (lo, hi)) in enumerate(zip(labels, defaults, ranges)):
            grid.addWidget(QLabel(label), i // 3, (i % 3) * 2)
            spin = QDoubleSpinBox()
            spin.setRange(lo, hi)
            spin.setDecimals(2 if i < 3 else 2)
            spin.setSingleStep(0.10 if i < 3 else 1.0)
            spin.setValue(default)
            self._movel_spins.append(spin)
            grid.addWidget(spin, i // 3, (i % 3) * 2 + 1)
        self.movel_group.setLayout(grid)
        layout.addWidget(self.movel_group)

        dur_layout = QHBoxLayout()
        self.movel_dur_label = QLabel(tr("traj.duration"))
        dur_layout.addWidget(self.movel_dur_label)
        self.movel_duration = QDoubleSpinBox()
        self.movel_duration.setRange(0.5, 30.0)
        self.movel_duration.setValue(2.0)
        self.movel_duration.setSuffix(" s")
        dur_layout.addWidget(self.movel_duration)
        dur_layout.addStretch()

        self.movel_exec_btn = QPushButton(tr("traj.exec_movel"))
        self.movel_exec_btn.setObjectName("enableBtn")
        self.movel_exec_btn.clicked.connect(self._on_exec_movel)
        dur_layout.addWidget(self.movel_exec_btn)
        layout.addLayout(dur_layout)

        layout.addStretch()
        return w

    def _create_waypoint_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)

        self.waypoint_table = QTableWidget(0, 7)
        self.waypoint_table.setHorizontalHeaderLabels(
            ["L1°", "L2°", "L3°", "L4°", "L5°", "L6°", tr("traj.time_header")]
        )
        self.waypoint_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        layout.addWidget(self.waypoint_table)

        btn_layout = QHBoxLayout()
        self.add_btn = QPushButton(tr("traj.add_cur"))
        self.add_btn.clicked.connect(self._add_current_waypoint)
        btn_layout.addWidget(self.add_btn)

        self.del_btn = QPushButton(tr("traj.del_sel"))
        self.del_btn.clicked.connect(self._delete_selected_waypoint)
        btn_layout.addWidget(self.del_btn)

        self.clear_btn = QPushButton(tr("traj.clear"))
        self.clear_btn.clicked.connect(lambda: self.waypoint_table.setRowCount(0))
        btn_layout.addWidget(self.clear_btn)

        btn_layout.addStretch()

        self.exec_all_btn = QPushButton(tr("traj.exec_all"))
        self.exec_all_btn.setObjectName("enableBtn")
        self.exec_all_btn.clicked.connect(self._exec_waypoints)
        btn_layout.addWidget(self.exec_all_btn)

        layout.addLayout(btn_layout)
        return w

    # ==================================================================
    # 拖拽模式控制
    # ==================================================================

    def _on_drag_toggled(self, checked: bool):
        self._drag_active = checked
        self.drag_mode_toggled.emit(checked)

    def _on_sync_clicked(self):
        self.sync_feedback_requested.emit()

    def update_drag_angles(self, angles: dict):
        """Receive drag angle dict from 3D viewer and update MoveJ spinboxes."""
        self._updating_from_drag = True
        for i, jname in enumerate(JOINT_NAMES_ORDERED[:6]):
            if jname in angles:
                self._movej_spins[i].setValue(math.degrees(angles[jname]))
        self._updating_from_drag = False

    def set_drag_enabled(self, enabled: bool):
        """Control whether drag button is toggleable (requires arm enabled)."""
        if not enabled and self._drag_active:
            self.drag_btn.setChecked(False)
            self._drag_active = False
            self.drag_mode_toggled.emit(False)

    # ==================================================================
    # retranslate
    # ==================================================================

    def retranslate_ui(self):
        self.tabs.setTabText(2, tr("traj.waypoints"))
        self.cancel_btn.setText(tr("traj.cancel"))

        self.drag_btn.setText(tr("v3d.drag_mode"))
        self.drag_btn.setToolTip(tr("v3d.drag_tip"))
        self.sync_btn.setText(tr("v3d.sync_pos"))
        self.sync_btn.setToolTip(tr("v3d.sync_tip"))
        self.read_btn.setText(tr("traj.read_pos"))

        self.movej_group.setTitle(tr("traj.joint_target"))
        self.movej_dur_label.setText(tr("traj.duration"))
        self.movej_exec_btn.setText(tr("traj.exec_movej"))
        self.movel_group.setTitle(tr("traj.cart_target"))
        self.read_pose_btn.setText(tr("traj.read_pose"))
        self.read_pose_btn.setToolTip(tr("traj.read_pose_tip"))
        self.movel_debug_btn.setText(tr("traj.move_debug"))
        self.movel_debug_btn.setToolTip(tr("traj.move_debug_tip"))
        self.movel_dur_label.setText(tr("traj.duration"))
        self.movel_exec_btn.setText(tr("traj.exec_movel"))
        self.waypoint_table.setHorizontalHeaderLabels(
            ["L1°", "L2°", "L3°", "L4°", "L5°", "L6°", tr("traj.time_header")]
        )
        self.add_btn.setText(tr("traj.add_cur"))
        self.del_btn.setText(tr("traj.del_sel"))
        self.clear_btn.setText(tr("traj.clear"))
        self.exec_all_btn.setText(tr("traj.exec_all"))
        if self._status_mode == "ready":
            self.status_label.setText(tr("traj.ready"))
        elif self._status_mode == "movej":
            self.status_label.setText(tr("traj.movej_running"))
        elif self._status_mode == "movel":
            self.status_label.setText(tr("traj.movel_running"))
        elif self._status_mode == "waypoints":
            self.status_label.setText(tr("traj.exec_waypoints", n=self.waypoint_table.rowCount()))

    # ==================================================================
    # MoveJ / MoveL 执行
    # ==================================================================

    def _on_exec_movej(self):
        positions = [math.radians(s.value()) for s in self._movej_spins]
        duration = self.movej_duration.value()
        self.move_j_requested.emit(positions, duration)
        self._status_mode = "movej"
        self.status_label.setText(tr("traj.movej_running"))

    def _on_exec_movel(self):
        x = self._movel_spins[0].value() * CM_TO_M
        y = self._movel_spins[1].value() * CM_TO_M
        z = self._movel_spins[2].value() * CM_TO_M
        rx = math.radians(self._movel_spins[3].value())
        ry = math.radians(self._movel_spins[4].value())
        rz = math.radians(self._movel_spins[5].value())
        duration = self.movel_duration.value()
        self.move_l_requested.emit([x, y, z, rx, ry, rz], duration)
        self._status_mode = "movel"
        self.status_label.setText(tr("traj.movel_running"))

    def _fill_current_positions(self):
        for i in range(min(6, len(self._current_positions))):
            self._movej_spins[i].setValue(math.degrees(self._current_positions[i]))

    def _move_to_debug_pose(self):
        positions = [math.radians(v) for v in MOVEL_DEBUG_JOINTS_DEG]
        duration = max(2.0, self.movel_duration.value())
        self.move_j_requested.emit(positions, duration)
        for spin, value in zip(self._movel_spins, MOVEL_DEBUG_POSE):
            spin.setValue(value)
        self._movel_pose_initialized = True
        self._status_mode = "movej"
        self.status_label.setText(tr("traj.debug_moving"))

    def _fill_current_end_pose(self):
        if self._current_end_pose is None or self._is_zero_pose(self._current_end_pose):
            self.status_label.setText(tr("traj.querying"))
            self.request_end_pose.emit()
            QTimer.singleShot(120, self._retry_fill_end_pose)
            return

        self._set_movel_pose_inputs(self._current_end_pose)
        self._movel_pose_initialized = True
        self.status_label.setText(tr("traj.pose_loaded"))

    def _retry_fill_end_pose(self):
        if self._current_end_pose is not None and not self._is_zero_pose(self._current_end_pose):
            self._set_movel_pose_inputs(self._current_end_pose)
            self._movel_pose_initialized = True
            self.status_label.setText(tr("traj.pose_loaded"))
        else:
            self.status_label.setText(tr("traj.no_pose"))

    @staticmethod
    def _is_zero_pose(pose) -> bool:
        return (
            abs(pose.x) < 1e-9 and abs(pose.y) < 1e-9 and abs(pose.z) < 1e-9
            and abs(pose.rx) < 1e-9 and abs(pose.ry) < 1e-9 and abs(pose.rz) < 1e-9
        )

    def _set_movel_pose_inputs(self, pose):
        if pose is None:
            return
        values = [
            pose.x * M_TO_CM,
            pose.y * M_TO_CM,
            pose.z * M_TO_CM,
            math.degrees(pose.rx),
            math.degrees(pose.ry),
            math.degrees(pose.rz),
        ]
        for spin, value in zip(self._movel_spins, values):
            spin.setValue(value)

    def update_current_positions(self, joint_states):
        positions = joint_states.to_list(include_gripper=False)
        self._current_positions = positions[:6]

    def update_current_end_pose(self, end_pose):
        self._current_end_pose = end_pose
        if not self._movel_pose_initialized:
            self._set_movel_pose_inputs(end_pose)
            self._movel_pose_initialized = True

    def _add_current_waypoint(self):
        row = self.waypoint_table.rowCount()
        self.waypoint_table.insertRow(row)
        for i in range(6):
            deg = math.degrees(self._current_positions[i]) if i < len(self._current_positions) else 0.0
            self.waypoint_table.setItem(row, i, QTableWidgetItem(f"{deg:.2f}"))
        self.waypoint_table.setItem(row, 6, QTableWidgetItem("2.0"))

    def _delete_selected_waypoint(self):
        rows = set(idx.row() for idx in self.waypoint_table.selectedIndexes())
        for row in sorted(rows, reverse=True):
            self.waypoint_table.removeRow(row)

    def _exec_waypoints(self):
        for row in range(self.waypoint_table.rowCount()):
            positions = []
            for col in range(6):
                item = self.waypoint_table.item(row, col)
                val = float(item.text()) if item else 0.0
                positions.append(math.radians(val))
            dur_item = self.waypoint_table.item(row, 6)
            duration = float(dur_item.text()) if dur_item else 2.0
            self.move_j_requested.emit(positions, duration)
        self._status_mode = "waypoints"
        self.status_label.setText(tr("traj.exec_waypoints", n=self.waypoint_table.rowCount()))
