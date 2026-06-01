"""工具栏面板：后端选择 (SocketCAN/SLCAN) + 端口选择/开启 + 连接/使能/急停/状态指示 + 主题/语言切换"""

from PyQt6.QtWidgets import (
    QWidget, QHBoxLayout, QLabel, QPushButton,
    QComboBox, QMessageBox, QFrame,
)
from PyQt6.QtCore import pyqtSignal, Qt

from MotorStudio.utils.style import SCENE_COLORS

from MotorStudio.utils.can_utils import (
    detect_can_interfaces, get_can_state, get_can_bitrate,
    setup_can_interface, shutdown_can_interface,
    detect_serial_ports,
)
from MotorStudio.utils.i18n import tr
from MotorStudio.utils.theme_manager import ThemeManager

BITRATE_OPTIONS = [
    ("1M", 1000000),
    ("500K", 500000),
    ("250K", 250000),
]

SERIAL_BAUDRATE_OPTIONS = [
    ("2M", 2000000),
    ("1M", 1000000),
    ("921.6K", 921600),
    ("576K", 576000),
    ("500K", 500000),
    ("115.2K", 115200),
]

CAN_BITRATE_OPTIONS = [
    ("1M", 1000000),
    ("500K", 500000),
    ("250K", 250000),
    ("125K", 125000),
]


class ToolbarPanel(QWidget):
    """顶部工具栏（单行固定高度）"""

    connect_requested = pyqtSignal(str, dict)
    disconnect_requested = pyqtSignal()
    enable_requested = pyqtSignal()
    disable_requested = pyqtSignal()
    emergency_stop_requested = pyqtSignal()
    open_monitor_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._connected = False
        self._enabled = False
        self._interfaces = []
        self.setFixedHeight(48)
        self._separators: list[QFrame] = []
        self.setStyleSheet(
            "ToolbarPanel QPushButton { padding: 5px 10px; min-height: 20px; }"
            "ToolbarPanel QComboBox { min-height: 20px; padding: 2px 6px; }"
            "ToolbarPanel QCheckBox { min-height: 20px; }"
        )
        self._init_ui()
        self._on_backend_changed()

        tm = ThemeManager.instance()
        tm.theme_changed.connect(self._on_theme_changed)
        tm.language_changed.connect(lambda _: self.retranslate_ui())

    def _init_ui(self):
        row = QHBoxLayout(self)
        row.setContentsMargins(8, 4, 8, 4)
        row.setSpacing(6)

        # -- 后端选择 --
        self.backend_combo = QComboBox()
        self.backend_combo.addItem("SocketCAN", "socketcan")
        self.backend_combo.addItem("SLCAN", "slcan")
        self.backend_combo.setFixedWidth(90)
        self.backend_combo.setCurrentIndex(0)
        self.backend_combo.currentIndexChanged.connect(self._on_backend_changed)
        row.addWidget(self.backend_combo)

        # -- SocketCAN 控件组 --
        self.socketcan_label = QLabel(tr("tb.can_label"))
        row.addWidget(self.socketcan_label)

        self.can_combo = QComboBox()
        self.can_combo.setFixedWidth(120)
        self.can_combo.setEditable(True)
        self.can_combo.lineEdit().setPlaceholderText("can0")
        row.addWidget(self.can_combo)

        self.can_toggle_btn = QPushButton(tr("tb.open"))
        self.can_toggle_btn.setFixedWidth(46)
        self.can_toggle_btn.clicked.connect(self._on_can_toggle)
        row.addWidget(self.can_toggle_btn)

        self.bitrate_combo = QComboBox()
        self.bitrate_combo.setFixedWidth(60)
        for label, val in BITRATE_OPTIONS:
            self.bitrate_combo.addItem(label, val)
        self.bitrate_combo.setCurrentIndex(0)
        row.addWidget(self.bitrate_combo)

        # -- SLCAN 控件组 --
        self.slcan_label = QLabel(tr("tb.serial_label"))
        row.addWidget(self.slcan_label)

        self.serial_combo = QComboBox()
        self.serial_combo.setFixedWidth(140)
        self.serial_combo.setEditable(True)
        self.serial_combo.lineEdit().setPlaceholderText("COM3")
        row.addWidget(self.serial_combo)

        self.serial_baud_combo = QComboBox()
        self.serial_baud_combo.setFixedWidth(70)
        self.serial_baud_combo.setToolTip(tr("tb.serial_baud_tip"))
        for label, val in SERIAL_BAUDRATE_OPTIONS:
            self.serial_baud_combo.addItem(label, val)
        self.serial_baud_combo.setCurrentIndex(0)
        row.addWidget(self.serial_baud_combo)

        self.can_baud_label = QLabel(tr("tb.can_label"))
        row.addWidget(self.can_baud_label)

        self.can_baud_combo = QComboBox()
        self.can_baud_combo.setFixedWidth(60)
        self.can_baud_combo.setToolTip(tr("tb.can_baud_tip"))
        for label, val in CAN_BITRATE_OPTIONS:
            self.can_baud_combo.addItem(label, val)
        self.can_baud_combo.setCurrentIndex(0)
        row.addWidget(self.can_baud_combo)

        # -- 共用控件 --
        self.refresh_btn = QPushButton("⟳")
        self.refresh_btn.setFixedSize(28, 28)
        self.refresh_btn.setToolTip(tr("tb.refresh_tip"))
        self.refresh_btn.clicked.connect(self._on_refresh_clicked)
        row.addWidget(self.refresh_btn)

        self._add_sep(row)

        self.connect_btn = QPushButton(tr("tb.connect"))
        self.connect_btn.setObjectName("connectBtn")
        self.connect_btn.setFixedWidth(60)
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        row.addWidget(self.connect_btn)

        self.enable_btn = QPushButton(tr("tb.enable"))
        self.enable_btn.setObjectName("enableBtn")
        self.enable_btn.setFixedWidth(60)
        self.enable_btn.setEnabled(False)
        self.enable_btn.clicked.connect(self._on_enable_clicked)
        row.addWidget(self.enable_btn)

        self._add_sep(row)

        self.estop_btn = QPushButton(tr("tb.estop"))
        self.estop_btn.setObjectName("emergencyStop")
        self.estop_btn.setFixedWidth(60)
        self.estop_btn.clicked.connect(self.emergency_stop_requested.emit)
        row.addWidget(self.estop_btn)

        self._add_sep(row)

        self.monitor_btn = QPushButton(tr("tb.monitor"))
        self.monitor_btn.setFixedWidth(76)
        self.monitor_btn.setToolTip(tr("tb.monitor_tip"))
        self.monitor_btn.clicked.connect(self.open_monitor_requested.emit)
        row.addWidget(self.monitor_btn)

        row.addStretch()

        # -- 主题 / 语言切换 --
        self.theme_btn = QPushButton("🌙")
        self.theme_btn.setFixedSize(36, 30)
        self.theme_btn.setToolTip("Dark / Light")
        self.theme_btn.clicked.connect(ThemeManager.instance().toggle_theme)
        row.addWidget(self.theme_btn)

        self.lang_btn = QPushButton("EN")
        self.lang_btn.setFixedSize(36, 30)
        self.lang_btn.setToolTip("中文 / English")
        self.lang_btn.clicked.connect(ThemeManager.instance().toggle_language)
        row.addWidget(self.lang_btn)

        self._add_sep(row)

        self.status_label = QLabel(tr("tb.not_connected"))
        self.status_label.setObjectName("statusLabel")
        row.addWidget(self.status_label)

        self.fps_label = QLabel("FPS: --")
        self.fps_label.setObjectName("fpsLabel")
        self.fps_label.setFixedWidth(70)
        row.addWidget(self.fps_label)

    def _add_sep(self, layout):
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.VLine)
        sep.setFixedWidth(1)
        sep.setFixedHeight(24)
        sc = SCENE_COLORS[ThemeManager.instance().theme]
        sep.setStyleSheet(f"color: {sc['separator']}; background: {sc['separator']};")
        self._separators.append(sep)
        layout.addWidget(sep)

    # ---- 主题 / 语言 ----

    def _on_theme_changed(self, theme: str):
        icon = "☀️" if theme == "dark" else "🌙"
        self.theme_btn.setText(icon)
        sc = SCENE_COLORS[theme]
        for sep in self._separators:
            sep.setStyleSheet(f"color: {sc['separator']}; background: {sc['separator']};")

    def retranslate_ui(self):
        tm = ThemeManager.instance()
        self.lang_btn.setText("中" if tm.language == "en" else "EN")

        self.socketcan_label.setText(tr("tb.can_label"))
        self.slcan_label.setText(tr("tb.serial_label"))
        self.can_baud_label.setText(tr("tb.can_label"))
        self.serial_baud_combo.setToolTip(tr("tb.serial_baud_tip"))
        self.can_baud_combo.setToolTip(tr("tb.can_baud_tip"))
        self.refresh_btn.setToolTip(tr("tb.refresh_tip"))
        self.estop_btn.setText(tr("tb.estop"))
        self.monitor_btn.setText(tr("tb.monitor"))
        self.monitor_btn.setToolTip(tr("tb.monitor_tip"))

        if self._connected:
            self.connect_btn.setText(tr("tb.disconnect"))
        else:
            self.connect_btn.setText(tr("tb.connect"))

        if self._enabled:
            self.enable_btn.setText(tr("tb.disable"))
            self.status_label.setText(tr("tb.enabled"))
        elif self._connected:
            self.enable_btn.setText(tr("tb.enable"))
            self.status_label.setText(tr("tb.connected"))
        else:
            self.enable_btn.setText(tr("tb.enable"))
            self.status_label.setText(tr("tb.not_connected"))

        self._update_can_toggle_label()

    # ---- 后端切换 ----

    def _is_slcan_mode(self) -> bool:
        return self.backend_combo.currentData() == "slcan"

    def _on_backend_changed(self):
        slcan = self._is_slcan_mode()

        self.socketcan_label.setVisible(not slcan)
        self.can_combo.setVisible(not slcan)
        self.can_toggle_btn.setVisible(not slcan)
        self.bitrate_combo.setVisible(not slcan)

        self.slcan_label.setVisible(slcan)
        self.serial_combo.setVisible(slcan)
        self.serial_baud_combo.setVisible(slcan)
        self.can_baud_label.setVisible(slcan)
        self.can_baud_combo.setVisible(slcan)

        self._on_refresh_clicked()

    # ---- 刷新接口 ----

    def _on_refresh_clicked(self):
        if self._is_slcan_mode():
            self._refresh_serial_ports()
        else:
            self._refresh_can_interfaces()

    def _refresh_can_interfaces(self):
        prev_text = self.can_combo.currentText().split(" ")[0].strip()
        self.can_combo.clear()

        self._interfaces = detect_can_interfaces()
        restore_idx = -1

        for i, iface in enumerate(self._interfaces):
            name = iface["name"]
            state = iface["state"]
            bitrate = iface["bitrate"]
            br_str = f" {bitrate // 1000}K" if bitrate else ""
            label = f"{name} ({state}{br_str})"
            self.can_combo.addItem(label, name)
            if name == prev_text:
                restore_idx = i
            if state == "UP" and bitrate:
                self._sync_bitrate_combo(bitrate)

        if not self._interfaces:
            self.can_combo.addItem(f"can0 {tr('tb.no_can')}")
            self.can_combo.setItemData(0, "can0")

        if restore_idx >= 0:
            self.can_combo.setCurrentIndex(restore_idx)

        self._update_can_toggle_label()

    def _refresh_serial_ports(self):
        prev_text = self.serial_combo.currentText().split(" ")[0].strip()
        self.serial_combo.clear()

        ports = detect_serial_ports()
        restore_idx = -1

        for i, info in enumerate(ports):
            port = info["port"]
            desc = info["desc"]
            label = f"{port} ({desc})" if desc and desc != port else port
            self.serial_combo.addItem(label, port)
            if port == prev_text:
                restore_idx = i

        if not ports:
            self.serial_combo.addItem(f"COM3 {tr('tb.no_can')}")
            self.serial_combo.setItemData(0, "COM3")

        if restore_idx >= 0:
            self.serial_combo.setCurrentIndex(restore_idx)

    def _sync_bitrate_combo(self, bitrate: int):
        for i in range(self.bitrate_combo.count()):
            if self.bitrate_combo.itemData(i) == bitrate:
                self.bitrate_combo.setCurrentIndex(i)
                return

    def _get_selected_can_name(self) -> str:
        idx = self.can_combo.currentIndex()
        if idx >= 0 and self.can_combo.itemData(idx):
            return self.can_combo.itemData(idx)
        raw = self.can_combo.currentText().split(" ")[0].strip()
        return raw or "can0"

    def _get_selected_serial_port(self) -> str:
        idx = self.serial_combo.currentIndex()
        if idx >= 0 and self.serial_combo.itemData(idx):
            return self.serial_combo.itemData(idx)
        raw = self.serial_combo.currentText().split(" ")[0].strip()
        return raw or "COM3"

    def _get_selected_state(self) -> str:
        name = self._get_selected_can_name()
        for iface in self._interfaces:
            if iface["name"] == name:
                return iface["state"]
        return get_can_state(name)

    def _update_can_toggle_label(self):
        state = self._get_selected_state()
        if state == "UP":
            self.can_toggle_btn.setText(tr("tb.close"))
        else:
            self.can_toggle_btn.setText(tr("tb.open"))

    def _on_can_toggle(self):
        name = self._get_selected_can_name()
        state = self._get_selected_state()

        if state == "UP":
            ok, msg = shutdown_can_interface(name)
        else:
            bitrate = self.bitrate_combo.currentData() or 1000000
            ok, msg = setup_can_interface(name, bitrate)

        if ok:
            self._refresh_can_interfaces()
        else:
            QMessageBox.warning(self, tr("tb.can_fail"), msg)
            self._refresh_can_interfaces()

    # ---- 连接 / 使能 / 循环 ----

    def _on_connect_clicked(self):
        if not self._connected:
            if self._is_slcan_mode():
                port = self._get_selected_serial_port()
                kwargs = {
                    "backend": "slcan",
                    "serial_port": port,
                    "serial_baudrate": self.serial_baud_combo.currentData() or 2000000,
                    "can_bitrate": self.can_baud_combo.currentData() or 1000000,
                }
                self.connect_requested.emit(port, kwargs)
            else:
                can_name = self._get_selected_can_name()
                self.connect_requested.emit(can_name, {})
        else:
            self.disconnect_requested.emit()

    def _on_enable_clicked(self):
        if not self._enabled:
            self.enable_requested.emit()
        else:
            self.disable_requested.emit()

    # ---- 外部状态更新 ----

    def set_connected(self, connected: bool):
        self._connected = connected
        can_area_enabled = not connected
        if connected:
            self.connect_btn.setText(tr("tb.disconnect"))
            self.connect_btn.setObjectName("disconnectBtn")
            self.enable_btn.setEnabled(True)
            self.status_label.setText(tr("tb.connected"))
        else:
            self.connect_btn.setText(tr("tb.connect"))
            self.connect_btn.setObjectName("connectBtn")
            self.enable_btn.setEnabled(False)
            self._enabled = False
            self.status_label.setText(tr("tb.not_connected"))

        self.backend_combo.setEnabled(can_area_enabled)
        self.refresh_btn.setEnabled(can_area_enabled)

        self.can_combo.setEnabled(can_area_enabled)
        self.can_toggle_btn.setEnabled(can_area_enabled)
        self.bitrate_combo.setEnabled(can_area_enabled)

        self.serial_combo.setEnabled(can_area_enabled)
        self.serial_baud_combo.setEnabled(can_area_enabled)
        self.can_baud_combo.setEnabled(can_area_enabled)

        self.connect_btn.style().unpolish(self.connect_btn)
        self.connect_btn.style().polish(self.connect_btn)

    def set_enabled(self, enabled: bool):
        self._enabled = enabled
        if enabled:
            self.enable_btn.setText(tr("tb.disable"))
            self.status_label.setText(tr("tb.enabled"))
            self.status_label.setObjectName("statusLabel")
        else:
            self.enable_btn.setText(tr("tb.enable"))
            if self._connected:
                self.status_label.setText(tr("tb.connected"))
            self.status_label.setObjectName("statusLabel")
        self.status_label.style().unpolish(self.status_label)
        self.status_label.style().polish(self.status_label)

    def set_fps(self, fps: float):
        self.fps_label.setText(f"FPS: {fps:.0f}")

    def set_error(self, msg: str):
        self.status_label.setText(tr("tb.error", msg=msg))
        self.status_label.setObjectName("errorLabel")
        self.status_label.style().unpolish(self.status_label)
        self.status_label.style().polish(self.status_label)
