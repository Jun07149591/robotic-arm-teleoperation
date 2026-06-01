"""实时监控面板：4 通道 pyqtgraph 实时曲线"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QGridLayout, QFileDialog, QLabel,
)
from PyQt6.QtCore import QTimer

import pyqtgraph as pg

from MotorStudio.utils.i18n import tr
from MotorStudio.utils.theme_manager import ThemeManager
from MotorStudio.utils.style import SCENE_COLORS, JOINT_COLORS
from MotorStudio.backend.data_buffer import DataBuffer

pg.setConfigOptions(antialias=True)

CHANNEL_NAMES = ["L1", "L2", "L3", "L4", "L5", "L6", "L7"]


class MonitoringPanel(QWidget):
    """2x2 实时曲线监控面板"""

    def __init__(self, data_buffer: DataBuffer, parent=None):
        super().__init__(parent)
        self.data_buffer = data_buffer
        self._paused = False
        self._plots = []
        self._curves = []
        self._init_ui()
        self.apply_theme()
        self.retranslate_ui()
        self._start_timer()

    def apply_theme(self):
        sc = SCENE_COLORS[ThemeManager.instance().theme]
        for pw in self._plots:
            pw.setBackground(sc["pg_bg"])

    def retranslate_ui(self):
        titles = [
            tr("mon.title_pos"),
            tr("mon.title_vel"),
            tr("mon.title_torque"),
            tr("mon.title_temp"),
        ]
        for pw, title in zip(self._plots, titles):
            pw.setTitle(title)
            pw.setLabel("bottom", tr("mon.time"), units="s")
        self.pause_btn.setText(tr("mon.resume") if self._paused else tr("mon.pause"))
        self.clear_btn.setText(tr("mon.clear"))
        self.export_btn.setText(tr("mon.export"))

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        top_bar = QHBoxLayout()
        self.pause_btn = QPushButton()
        self.pause_btn.setFixedWidth(60)
        self.pause_btn.clicked.connect(self._toggle_pause)
        top_bar.addWidget(self.pause_btn)

        self.clear_btn = QPushButton()
        self.clear_btn.setFixedWidth(60)
        self.clear_btn.clicked.connect(self._clear_data)
        top_bar.addWidget(self.clear_btn)

        self.export_btn = QPushButton()
        self.export_btn.setFixedWidth(80)
        self.export_btn.clicked.connect(self._export_csv)
        top_bar.addWidget(self.export_btn)

        top_bar.addSpacing(20)
        self._channel_swatches: list[QLabel] = []
        for ch in range(7):
            swatch = QLabel()
            swatch.setStyleSheet(
                f"color: {JOINT_COLORS[ch]}; font-weight: bold; font-size: 11px;"
            )
            top_bar.addWidget(swatch)
            self._channel_swatches.append(swatch)

        top_bar.addStretch()
        layout.addLayout(top_bar)

        grid = QGridLayout()
        grid.setSpacing(4)

        for idx in range(4):
            pw = pg.PlotWidget()
            pw.showGrid(x=True, y=True, alpha=0.3)

            curves = []
            for ch in range(7):
                pen = pg.mkPen(color=JOINT_COLORS[ch], width=1.5)
                curve = pw.plot([], [], pen=pen)
                curves.append(curve)

            self._plots.append(pw)
            self._curves.append(curves)
            grid.addWidget(pw, idx // 2, idx % 2)

        layout.addLayout(grid)

        for ch, lbl in enumerate(self._channel_swatches):
            lbl.setText(f"■ {CHANNEL_NAMES[ch]}")

    def _start_timer(self):
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._update_plots)
        self._timer.start(50)  # 20Hz display refresh

    def _update_plots(self):
        if self._paused or self.data_buffer.count == 0:
            return

        ts, pos, vel, torq, temp = self.data_buffer.get_data()
        if len(ts) == 0:
            return

        t_rel = ts - ts[0]
        datasets = [pos, vel, torq, temp]

        for plot_idx, data_2d in enumerate(datasets):
            for ch in range(7):
                self._curves[plot_idx][ch].setData(t_rel, data_2d[:, ch])

    def _toggle_pause(self):
        self._paused = not self._paused
        self.pause_btn.setText(tr("mon.resume") if self._paused else tr("mon.pause"))

    def _clear_data(self):
        self.data_buffer.clear()
        for plot_curves in self._curves:
            for c in plot_curves:
                c.setData([], [])

    def _export_csv(self):
        filepath, _ = QFileDialog.getSaveFileName(
            self, tr("mon.export_title"), "arm_data.csv", "CSV Files (*.csv)"
        )
        if filepath:
            self.data_buffer.export_csv(filepath)
