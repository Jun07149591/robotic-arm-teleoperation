"""示教面板：内嵌 Tab — 示教（零力矩 + 录制）+ 标定"""

import math
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QDoubleSpinBox, QPushButton, QGroupBox,
    QListWidget, QFileDialog, QCheckBox, QTabWidget,
)
from PyQt6.QtCore import pyqtSignal, QTimer

from MotorStudio.backend.trajectory_recorder import TrajectoryRecorder
from MotorStudio.widgets.calibration_panel import CalibrationPanel
from MotorStudio.utils.i18n import tr
from MotorStudio.utils.theme_manager import ThemeManager
from MotorStudio.utils.style import SCENE_COLORS


class TeachingPanel(QWidget):
    """示教模式：零力矩拖动 + 轨迹录制回放 + 标定"""

    zero_torque_requested = pyqtSignal(bool)
    zero_torque_gravity_requested = pyqtSignal(bool)
    move_j_requested = pyqtSignal(list, float)  # positions, duration

    def __init__(self, parent=None):
        super().__init__(parent)
        self.recorder = TrajectoryRecorder(sample_rate_hz=10.0)
        self._zero_torque_active = False
        self._current_positions = [0.0] * 7
        self._rec_complete_info = None
        self._init_ui()

    def _init_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        self._tabs = QTabWidget()
        outer.addWidget(self._tabs)

        self._teaching_content = QWidget()
        self._build_teaching_tab(self._teaching_content)
        self._tabs.addTab(self._teaching_content, tr("teach.tab_teaching"))

        self.calibration_panel = CalibrationPanel()
        self._tabs.addTab(self.calibration_panel, tr("teach.tab_calibration"))

        self._rec_timer = QTimer(self)
        self._rec_timer.timeout.connect(self._sample_position)

    def _build_teaching_tab(self, parent: QWidget):
        layout = QVBoxLayout(parent)
        layout.setContentsMargins(4, 4, 4, 4)

        self.zt_group = QGroupBox(tr("teach.zt_group"))
        zt_layout = QVBoxLayout()

        self.gravity_check = QCheckBox(tr("teach.gravity"))
        self.gravity_check.setChecked(True)
        zt_layout.addWidget(self.gravity_check)

        self.zt_toggle_btn = QPushButton(tr("teach.zt_on"))
        self.zt_toggle_btn.setObjectName("enableBtn")
        self.zt_toggle_btn.clicked.connect(self._toggle_zero_torque)
        zt_layout.addWidget(self.zt_toggle_btn)

        self.zt_group.setLayout(zt_layout)
        layout.addWidget(self.zt_group)

        self.rec_group = QGroupBox(tr("teach.rec_group"))
        rec_layout = QVBoxLayout()

        rate_layout = QHBoxLayout()
        self.rate_label = QLabel(tr("teach.sample_rate"))
        rate_layout.addWidget(self.rate_label)
        self.rate_spin = QDoubleSpinBox()
        self.rate_spin.setRange(1.0, 50.0)
        self.rate_spin.setValue(10.0)
        self.rate_spin.setSuffix(" Hz")
        rate_layout.addWidget(self.rate_spin)
        rate_layout.addStretch()
        rec_layout.addLayout(rate_layout)

        btn_layout = QHBoxLayout()
        self.rec_btn = QPushButton(tr("teach.start_rec"))
        self.rec_btn.clicked.connect(self._toggle_recording)
        btn_layout.addWidget(self.rec_btn)

        self.rec_status = QLabel(tr("teach.not_rec"))
        btn_layout.addWidget(self.rec_status)
        btn_layout.addStretch()
        rec_layout.addLayout(btn_layout)

        self.rec_group.setLayout(rec_layout)
        layout.addWidget(self.rec_group)

        self.traj_group = QGroupBox(tr("teach.traj_group"))
        traj_layout = QVBoxLayout()

        self.traj_list = QListWidget()
        traj_layout.addWidget(self.traj_list)

        traj_btn_layout = QHBoxLayout()
        self.play_btn = QPushButton(tr("teach.play"))
        self.play_btn.clicked.connect(self._playback)
        traj_btn_layout.addWidget(self.play_btn)

        self.save_btn = QPushButton(tr("teach.save"))
        self.save_btn.clicked.connect(self._save_trajectory)
        traj_btn_layout.addWidget(self.save_btn)

        self.load_btn = QPushButton(tr("teach.load"))
        self.load_btn.clicked.connect(self._load_trajectory)
        traj_btn_layout.addWidget(self.load_btn)

        self.del_btn = QPushButton(tr("teach.delete"))
        self.del_btn.clicked.connect(self._delete_trajectory)
        traj_btn_layout.addWidget(self.del_btn)

        traj_layout.addLayout(traj_btn_layout)
        self.traj_group.setLayout(traj_layout)
        layout.addWidget(self.traj_group)

    def retranslate_ui(self):
        self._tabs.setTabText(0, tr("teach.tab_teaching"))
        self._tabs.setTabText(1, tr("teach.tab_calibration"))

        self.zt_group.setTitle(tr("teach.zt_group"))
        self.gravity_check.setText(tr("teach.gravity"))
        if self._zero_torque_active:
            self.zt_toggle_btn.setText(tr("teach.zt_off"))
        else:
            self.zt_toggle_btn.setText(tr("teach.zt_on"))
        self.rec_group.setTitle(tr("teach.rec_group"))
        self.rate_label.setText(tr("teach.sample_rate"))
        if self.recorder.is_recording:
            self.rec_btn.setText(tr("teach.stop_rec"))
        else:
            self.rec_btn.setText(tr("teach.start_rec"))
        self._refresh_rec_status_text()
        self.traj_group.setTitle(tr("teach.traj_group"))
        self.play_btn.setText(tr("teach.play"))
        self.save_btn.setText(tr("teach.save"))
        self.load_btn.setText(tr("teach.load"))
        self.del_btn.setText(tr("teach.delete"))

        self.calibration_panel.retranslate_ui()

    def _refresh_rec_status_text(self):
        if self.recorder.is_recording:
            if self.recorder.current_trajectory:
                n = self.recorder.current_trajectory.num_points
                if n > 0:
                    self.rec_status.setText(tr("teach.recording_n", n=n))
                else:
                    self.rec_status.setText(tr("teach.recording"))
            else:
                self.rec_status.setText(tr("teach.recording"))
        elif self._rec_complete_info is not None:
            n, t = self._rec_complete_info
            self.rec_status.setText(tr("teach.rec_done", n=n, t=t))
        else:
            self.rec_status.setText(tr("teach.not_rec"))

    def _toggle_zero_torque(self):
        self._zero_torque_active = not self._zero_torque_active

        if self._zero_torque_active:
            self.zt_toggle_btn.setText(tr("teach.zt_off"))
            sc = SCENE_COLORS[ThemeManager.instance().theme]
            self.zt_toggle_btn.setStyleSheet(
                f"background-color: {sc['zt_active_bg']}; color: white; border-radius: 6px;"
            )
            if self.gravity_check.isChecked():
                self.zero_torque_gravity_requested.emit(True)
            else:
                self.zero_torque_requested.emit(True)
        else:
            self.zt_toggle_btn.setText(tr("teach.zt_on"))
            self.zt_toggle_btn.setStyleSheet("")
            self.zt_toggle_btn.setObjectName("enableBtn")
            if self.gravity_check.isChecked():
                self.zero_torque_gravity_requested.emit(False)
            else:
                self.zero_torque_requested.emit(False)

    def _toggle_recording(self):
        if self.recorder.is_recording:
            traj = self.recorder.stop_recording()
            self._rec_timer.stop()
            self.rec_btn.setText(tr("teach.start_rec"))
            self.rec_btn.setStyleSheet("")
            if traj:
                self._rec_complete_info = (traj.num_points, traj.duration)
                self.rec_status.setText(
                    tr("teach.rec_done", n=traj.num_points, t=traj.duration)
                )
                self.traj_list.addItem(
                    f"{traj.name} ({traj.num_points}点, {traj.duration:.1f}s)"
                )
        else:
            self._rec_complete_info = None
            self.recorder.sample_rate_hz = self.rate_spin.value()
            self.recorder.start_recording()
            interval_ms = int(1000.0 / self.rate_spin.value())
            self._rec_timer.start(interval_ms)
            self.rec_btn.setText(tr("teach.stop_rec"))
            sc = SCENE_COLORS[ThemeManager.instance().theme]
            self.rec_btn.setStyleSheet(
                f"background-color: {sc['recording_bg']}; color: white; border-radius: 6px;"
            )
            self.rec_status.setText(tr("teach.recording"))

    def _sample_position(self):
        self.recorder.add_sample(self._current_positions)
        if self.recorder.current_trajectory:
            n = self.recorder.current_trajectory.num_points
            self.rec_status.setText(tr("teach.recording_n", n=n))

    def update_positions(self, joint_states):
        self._current_positions = joint_states.to_list(include_gripper=True)

    def _playback(self):
        idx = self.traj_list.currentRow()
        if idx < 0 or idx >= len(self.recorder.trajectories):
            return
        traj = self.recorder.trajectories[idx]
        if not traj.points:
            return
        dt = 1.0 / max(traj.sample_rate_hz, 1.0)
        for pt in traj.points:
            self.move_j_requested.emit(pt.positions[:6], dt)

    def _save_trajectory(self):
        idx = self.traj_list.currentRow()
        if idx < 0 or idx >= len(self.recorder.trajectories):
            return
        traj = self.recorder.trajectories[idx]
        filepath, _ = QFileDialog.getSaveFileName(
            self, tr("teach.save_title"), f"{traj.name}.json", "JSON Files (*.json)"
        )
        if filepath:
            self.recorder.save_trajectory(traj, filepath)

    def _load_trajectory(self):
        filepath, _ = QFileDialog.getOpenFileName(
            self, tr("teach.load_title"), "", "JSON Files (*.json)"
        )
        if filepath:
            traj = self.recorder.load_trajectory(filepath)
            self.traj_list.addItem(
                f"{traj.name} ({traj.num_points}点, {traj.duration:.1f}s)"
            )

    def _delete_trajectory(self):
        idx = self.traj_list.currentRow()
        if idx < 0 or idx >= len(self.recorder.trajectories):
            return
        self.recorder.trajectories.pop(idx)
        self.traj_list.takeItem(idx)
