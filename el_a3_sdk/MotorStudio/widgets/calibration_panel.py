"""标定面板：高精度标定 / 已知质量标定"""

from __future__ import annotations

import json
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QComboBox, QStackedWidget, QGroupBox, QFormLayout,
    QDoubleSpinBox, QSpinBox,
    QPushButton, QProgressBar, QTextEdit,
    QTableWidget, QTableWidgetItem, QHeaderView,
    QFileDialog, QScrollArea, QSizePolicy,
)
from PyQt6.QtCore import pyqtSignal, Qt

from MotorStudio.backend.calibration_worker import (
    CalibrationWorker, CalibrationConfig,
    has_saved_calibration_data, clear_saved_calibration_data,
)
from MotorStudio.utils.i18n import tr
from MotorStudio.utils.theme_manager import ThemeManager
from MotorStudio.utils.style import SCENE_COLORS

LINK_NAMES = ["L2", "L3", "L4", "L5", "L6"]
M_TO_CM = 100.0


class CalibrationPanel(QWidget):
    """惯量参数标定面板（内嵌在示教 Tab 中）。"""

    move_j_requested = pyqtSignal(list, float, bool)
    start_calibration_requested = pyqtSignal()
    stop_calibration_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._worker: CalibrationWorker | None = None
        self._last_results: dict | None = None
        self._init_ui()

    # ==================================================================
    # UI
    # ==================================================================

    def _init_ui(self):
        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.Shape.NoFrame)

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(4)

        # --- 标定模式 ---
        mode_row = QHBoxLayout()
        mode_row.setSpacing(6)
        self._mode_label = QLabel(tr("cal.mode_label"))
        mode_row.addWidget(self._mode_label)
        self._mode_combo = QComboBox()
        self._mode_combo.addItem(tr("cal.mode_high"), "high_precision")
        self._mode_combo.addItem(tr("cal.mode_known_mass"), "known_mass")
        self._mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        mode_row.addWidget(self._mode_combo, 1)
        layout.addLayout(mode_row)

        # --- 参数页 (QStackedWidget，只占需要的高度) ---
        self._stack = QStackedWidget()
        self._stack.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)

        self._page_high = self._build_high_precision_page()
        self._stack.addWidget(self._page_high)

        self._page_mass = self._build_known_mass_page()
        self._stack.addWidget(self._page_mass)

        layout.addWidget(self._stack)

        # --- 控制按钮 + 进度 ---
        ctrl_row = QHBoxLayout()
        ctrl_row.setSpacing(6)
        self._start_btn = QPushButton(tr("cal.start"))
        self._start_btn.setMinimumWidth(80)
        self._start_btn.clicked.connect(self._on_start)
        ctrl_row.addWidget(self._start_btn)

        self._stop_btn = QPushButton(tr("cal.stop"))
        self._stop_btn.setMinimumWidth(80)
        self._stop_btn.setEnabled(False)
        self._stop_btn.clicked.connect(self._on_stop)
        ctrl_row.addWidget(self._stop_btn)

        self._restart_btn = QPushButton(tr("cal.restart"))
        self._restart_btn.setMinimumWidth(80)
        self._restart_btn.clicked.connect(self._on_restart)
        ctrl_row.addWidget(self._restart_btn)

        ctrl_row.addStretch()
        layout.addLayout(ctrl_row)

        self._progress = QProgressBar()
        self._progress.setFixedHeight(16)
        self._progress.setValue(0)
        layout.addWidget(self._progress)

        # --- 标定日志 ---
        self._log_group = QGroupBox(tr("cal.log_group"))
        lg_layout = QVBoxLayout()
        lg_layout.setContentsMargins(4, 2, 4, 2)
        self._log_text = QTextEdit()
        self._log_text.setReadOnly(True)
        self._log_text.setFixedHeight(90)
        lg_layout.addWidget(self._log_text)
        self._log_group.setLayout(lg_layout)
        layout.addWidget(self._log_group)

        # --- 标定结果 ---
        self._result_group = QGroupBox(tr("cal.result_group"))
        rg_layout = QVBoxLayout()
        rg_layout.setContentsMargins(4, 2, 4, 2)

        self._result_table = QTableWidget(5, 7)
        self._result_table.setHorizontalHeaderLabels([
            tr("cal.col_joint"), tr("cal.col_mass"),
            tr("cal.col_com_x"), tr("cal.col_com_y"), tr("cal.col_com_z"),
            tr("cal.col_rmse"), tr("cal.col_r2"),
        ])
        self._result_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        self._result_table.verticalHeader().setVisible(False)
        self._result_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self._result_table.verticalHeader().setDefaultSectionSize(24)
        self._result_table.setFixedHeight(24 * 5 + self._result_table.horizontalHeader().defaultSectionSize() + 4)
        for row, lnk in enumerate(LINK_NAMES):
            self._result_table.setItem(row, 0, QTableWidgetItem(lnk))
            for col in range(1, 7):
                self._result_table.setItem(row, col, QTableWidgetItem("--"))
        rg_layout.addWidget(self._result_table)

        res_btn_row = QHBoxLayout()
        self._apply_btn = QPushButton(tr("cal.apply"))
        self._apply_btn.setMinimumWidth(80)
        self._apply_btn.setEnabled(False)
        self._apply_btn.clicked.connect(self._on_apply)
        res_btn_row.addWidget(self._apply_btn)

        self._export_btn = QPushButton(tr("cal.export"))
        self._export_btn.setMinimumWidth(80)
        self._export_btn.setEnabled(False)
        self._export_btn.clicked.connect(self._on_export)
        res_btn_row.addWidget(self._export_btn)
        res_btn_row.addStretch()
        rg_layout.addLayout(res_btn_row)

        self._result_group.setLayout(rg_layout)
        layout.addWidget(self._result_group)

        layout.addStretch()
        scroll.setWidget(container)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

        self._refresh_start_btn()

    def _build_high_precision_page(self) -> QWidget:
        page = QWidget()
        form = QFormLayout(page)
        form.setContentsMargins(0, 4, 0, 0)
        form.setVerticalSpacing(4)
        form.setHorizontalSpacing(8)

        self._hp_points_label = QLabel(tr("cal.num_points"))
        self._hp_points_spin = QSpinBox()
        self._hp_points_spin.setRange(10, 200)
        self._hp_points_spin.setValue(30)
        self._hp_points_spin.setFixedWidth(100)
        form.addRow(self._hp_points_label, self._hp_points_spin)

        self._hp_samples_label = QLabel(tr("cal.samples_per_point"))
        self._hp_samples_spin = QSpinBox()
        self._hp_samples_spin.setRange(10, 200)
        self._hp_samples_spin.setValue(50)
        self._hp_samples_spin.setFixedWidth(100)
        form.addRow(self._hp_samples_label, self._hp_samples_spin)

        self._hp_settle_label = QLabel(tr("cal.settle_time"))
        self._hp_settle_spin = QDoubleSpinBox()
        self._hp_settle_spin.setRange(0.5, 10.0)
        self._hp_settle_spin.setValue(1.5)
        self._hp_settle_spin.setSingleStep(0.5)
        self._hp_settle_spin.setFixedWidth(100)
        form.addRow(self._hp_settle_label, self._hp_settle_spin)

        self._hp_duration_label = QLabel(tr("cal.motion_duration"))
        self._hp_duration_spin = QDoubleSpinBox()
        self._hp_duration_spin.setRange(1.0, 20.0)
        self._hp_duration_spin.setValue(6.0)
        self._hp_duration_spin.setSingleStep(0.5)
        self._hp_duration_spin.setFixedWidth(100)
        form.addRow(self._hp_duration_label, self._hp_duration_spin)

        return page

    def _build_known_mass_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(0, 4, 0, 0)
        layout.setSpacing(2)

        self._mass_label = QLabel(tr("cal.mass_input"))
        layout.addWidget(self._mass_label)

        mass_form = QFormLayout()
        mass_form.setVerticalSpacing(2)
        mass_form.setHorizontalSpacing(8)
        self._mass_spins: dict[str, QDoubleSpinBox] = {}
        defaults = {"L2": 0.877, "L3": 0.251, "L4": 0.556, "L5": 0.018, "L6": 0.668}
        for lnk in LINK_NAMES:
            spin = QDoubleSpinBox()
            spin.setRange(0.001, 5.0)
            spin.setDecimals(4)
            spin.setSingleStep(0.01)
            spin.setValue(defaults.get(lnk, 0.5))
            spin.setSuffix(" kg")
            spin.setFixedWidth(120)
            self._mass_spins[lnk] = spin
            mass_form.addRow(QLabel(f"{lnk}:"), spin)
        layout.addLayout(mass_form)

        param_form = QFormLayout()
        param_form.setVerticalSpacing(4)
        param_form.setHorizontalSpacing(8)

        self._km_points_label = QLabel(tr("cal.num_points"))
        self._km_points_spin = QSpinBox()
        self._km_points_spin.setRange(10, 200)
        self._km_points_spin.setValue(30)
        self._km_points_spin.setFixedWidth(100)
        param_form.addRow(self._km_points_label, self._km_points_spin)

        self._km_samples_label = QLabel(tr("cal.samples_per_point"))
        self._km_samples_spin = QSpinBox()
        self._km_samples_spin.setRange(10, 200)
        self._km_samples_spin.setValue(50)
        self._km_samples_spin.setFixedWidth(100)
        param_form.addRow(self._km_samples_label, self._km_samples_spin)

        self._km_settle_label = QLabel(tr("cal.settle_time"))
        self._km_settle_spin = QDoubleSpinBox()
        self._km_settle_spin.setRange(0.5, 10.0)
        self._km_settle_spin.setValue(1.5)
        self._km_settle_spin.setSingleStep(0.5)
        self._km_settle_spin.setFixedWidth(100)
        param_form.addRow(self._km_settle_label, self._km_settle_spin)

        self._km_duration_label = QLabel(tr("cal.motion_duration"))
        self._km_duration_spin = QDoubleSpinBox()
        self._km_duration_spin.setRange(1.0, 20.0)
        self._km_duration_spin.setValue(6.0)
        self._km_duration_spin.setSingleStep(0.5)
        self._km_duration_spin.setFixedWidth(100)
        param_form.addRow(self._km_duration_label, self._km_duration_spin)

        layout.addLayout(param_form)
        return page

    # ==================================================================
    # Mode switching
    # ==================================================================

    def _on_mode_changed(self, index: int):
        self._stack.setCurrentIndex(index)
        self._refresh_start_btn()

    # ==================================================================
    # Resume helpers
    # ==================================================================

    def _refresh_start_btn(self):
        """Update start button text based on saved data availability."""
        exists, done, total, saved_mode = has_saved_calibration_data()
        cur_mode = self._mode_combo.currentData()
        if exists and done < total and saved_mode == cur_mode:
            self._start_btn.setText(
                tr("cal.resume_fmt", done=done, total=total))
            self._restart_btn.setEnabled(True)
        else:
            self._start_btn.setText(tr("cal.start"))
            self._restart_btn.setEnabled(exists)

    # ==================================================================
    # Start / Stop / Restart
    # ==================================================================

    def _build_config(self, *, resume: bool = False) -> CalibrationConfig:
        mode_key = self._mode_combo.currentData()
        joints = list(range(6))

        if mode_key == "known_mass":
            num_pts = self._km_points_spin.value()
            samples = self._km_samples_spin.value()
            settle = self._km_settle_spin.value()
            duration = self._km_duration_spin.value()
            masses = {lnk: sp.value() for lnk, sp in self._mass_spins.items()}
        else:
            num_pts = self._hp_points_spin.value()
            samples = self._hp_samples_spin.value()
            settle = self._hp_settle_spin.value()
            duration = self._hp_duration_spin.value()
            masses = None

        return CalibrationConfig(
            mode=mode_key,
            num_points=num_pts,
            samples_per_point=samples,
            settle_time=settle,
            motion_duration=duration,
            joints_to_calibrate=joints,
            known_masses=masses,
            resume=resume,
        )

    def _launch_worker(self, cfg: CalibrationConfig):
        self._worker = CalibrationWorker(cfg)
        self._worker.progress_updated.connect(self._on_progress)
        self._worker.log_message.connect(self._on_log)
        self._worker.error_occurred.connect(self._on_error)
        self._worker.calibration_finished.connect(self._on_finished)
        self._worker.move_j_requested.connect(self.move_j_requested)
        self._worker.finished.connect(self._on_thread_done)

        self._start_btn.setEnabled(False)
        self._stop_btn.setEnabled(True)
        self._restart_btn.setEnabled(False)
        self._apply_btn.setEnabled(False)
        self._export_btn.setEnabled(False)
        self._progress.setValue(0)

        self._worker.start()

    def _on_start(self):
        exists, done, total, saved_mode = has_saved_calibration_data()
        cur_mode = self._mode_combo.currentData()
        resume = exists and done < total and saved_mode == cur_mode
        cfg = self._build_config(resume=resume)
        if not resume:
            self._log_text.clear()
        self._launch_worker(cfg)

    def _on_stop(self):
        if self._worker and self._worker.isRunning():
            self._worker.request_stop()
            self._on_log(tr("cal.paused"))

    def _on_restart(self):
        clear_saved_calibration_data()
        self._on_log(tr("cal.data_cleared"))
        self._refresh_start_btn()

    def _on_thread_done(self):
        self._start_btn.setEnabled(True)
        self._stop_btn.setEnabled(False)
        self._restart_btn.setEnabled(True)
        self._refresh_start_btn()

    # ==================================================================
    # Worker callbacks
    # ==================================================================

    def _on_progress(self, cur: int, total: int, msg: str):
        if total > 0:
            self._progress.setMaximum(total)
            self._progress.setValue(cur)

    def _on_log(self, msg: str):
        self._log_text.append(msg)
        sb = self._log_text.verticalScrollBar()
        sb.setValue(sb.maximum())

    def _on_error(self, msg: str):
        self._on_log(f"[错误] {msg}")

    def _on_finished(self, results: dict):
        self._last_results = results
        self._apply_btn.setEnabled(True)
        self._export_btn.setEnabled(True)

        info = results.get("calibration_info", {})
        rmse = info.get("rmse", 0)
        r2 = info.get("r_squared", 0)

        for row, lnk in enumerate(LINK_NAMES):
            p = results.get(lnk, {})
            mass = p.get("mass", 0)
            com = p.get("com", [0, 0, 0])
            self._result_table.item(row, 1).setText(f"{mass:.4f}")
            self._result_table.item(row, 2).setText(f"{com[0] * M_TO_CM:.2f}")
            self._result_table.item(row, 3).setText(f"{com[1] * M_TO_CM:.2f}")
            self._result_table.item(row, 4).setText(f"{com[2] * M_TO_CM:.2f}")
            self._result_table.item(row, 5).setText(f"{rmse:.4f}")
            self._result_table.item(row, 6).setText(f"{r2:.4f}")

        self._on_log(tr("cal.finished"))

    # ==================================================================
    # Apply / Export
    # ==================================================================

    def _on_apply(self):
        if self._last_results:
            CalibrationWorker.save_yaml(self._last_results)
            self._on_log(tr("cal.applied"))

    def _on_export(self):
        if not self._last_results:
            return
        filepath, _ = QFileDialog.getSaveFileName(
            self, tr("cal.export_title"), "calibration_result.json",
            "JSON Files (*.json)",
        )
        if filepath:
            with open(filepath, "w", encoding="utf-8") as f:
                json.dump(self._last_results, f, indent=2, ensure_ascii=False)
            self._on_log(f"结果已导出: {filepath}")

    # ==================================================================
    # External data feed
    # ==================================================================

    def feed_efforts(self, efforts):
        if self._worker and self._worker.isRunning():
            self._worker.feed_efforts(efforts)

    def feed_positions(self, positions):
        if self._worker and self._worker.isRunning():
            self._worker.feed_positions(positions)

    def notify_move_done(self):
        if self._worker and self._worker.isRunning():
            self._worker.notify_move_done()

    # ==================================================================
    # retranslate
    # ==================================================================

    def retranslate_ui(self):
        self._mode_label.setText(tr("cal.mode_label"))
        self._mode_combo.setItemText(0, tr("cal.mode_high"))
        self._mode_combo.setItemText(1, tr("cal.mode_known_mass"))

        self._hp_points_label.setText(tr("cal.num_points"))
        self._hp_samples_label.setText(tr("cal.samples_per_point"))
        self._hp_settle_label.setText(tr("cal.settle_time"))
        self._hp_duration_label.setText(tr("cal.motion_duration"))

        self._km_points_label.setText(tr("cal.num_points"))
        self._km_samples_label.setText(tr("cal.samples_per_point"))
        self._km_settle_label.setText(tr("cal.settle_time"))
        self._km_duration_label.setText(tr("cal.motion_duration"))
        self._mass_label.setText(tr("cal.mass_input"))

        self._refresh_start_btn()
        self._stop_btn.setText(tr("cal.stop"))
        self._restart_btn.setText(tr("cal.restart"))
        self._log_group.setTitle(tr("cal.log_group"))
        self._result_group.setTitle(tr("cal.result_group"))
        self._apply_btn.setText(tr("cal.apply"))
        self._export_btn.setText(tr("cal.export"))

        headers = [
            tr("cal.col_joint"), tr("cal.col_mass"),
            tr("cal.col_com_x"), tr("cal.col_com_y"), tr("cal.col_com_z"),
            tr("cal.col_rmse"), tr("cal.col_r2"),
        ]
        self._result_table.setHorizontalHeaderLabels(headers)
