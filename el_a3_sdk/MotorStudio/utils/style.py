"""主题 QSS 样式表（深色 / 浅色）+ 场景配色 — 现代化重设计"""

_FONT_STACK = '"Noto Sans CJK SC", "Source Han Sans SC", "Microsoft YaHei", "Segoe UI", sans-serif'

# ─────────────────────── 深色主题（Modern Dark） ───────────────────────

DARK_THEME = f"""
/* ── 全局 ── */
QMainWindow, QWidget {{
    background-color: #1a1b2e;
    color: #cdd6f4;
    font-family: {_FONT_STACK};
    font-size: 13px;
}}

/* ── 菜单 ── */
QMenuBar {{
    background-color: #16172a;
    color: #cdd6f4;
    border-bottom: 1px solid #2a2b40;
}}
QMenuBar::item:selected {{ background-color: #2a2b40; }}
QMenu {{
    background-color: #1a1b2e;
    color: #cdd6f4;
    border: 1px solid #2a2b40;
    border-radius: 6px;
    padding: 4px 0;
}}
QMenu::item {{
    padding: 6px 24px;
    border-radius: 4px;
    margin: 2px 4px;
}}
QMenu::item:selected {{ background-color: #2a2b40; }}

/* ── 工具栏 / 状态栏 ── */
QToolBar {{
    background-color: #16172a;
    border-bottom: 1px solid #2a2b40;
    spacing: 6px;
    padding: 4px;
}}
QStatusBar {{
    background-color: #16172a;
    color: #8b8fa8;
    border-top: 1px solid #2a2b40;
    font-size: 12px;
}}

/* ── Dock ── */
QDockWidget {{
    color: #cdd6f4;
    titlebar-close-icon: none;
}}
QDockWidget::title {{
    background-color: transparent;
    padding: 0;
    border: none;
}}

/* ── Tab ── */
QTabWidget::pane {{
    border: 1px solid #2a2b40;
    border-top: none;
    background-color: #1a1b2e;
    border-radius: 0 0 8px 8px;
}}
QTabBar::tab {{
    background-color: #16172a;
    color: #8b8fa8;
    padding: 9px 18px;
    border: 1px solid #2a2b40;
    border-bottom: none;
    margin-right: 3px;
    border-radius: 6px 6px 0 0;
}}
QTabBar::tab:selected {{
    background-color: #1a1b2e;
    color: #89b4fa;
    border-bottom: 2px solid #89b4fa;
}}
QTabBar::tab:hover:!selected {{ color: #cdd6f4; background-color: #1f2037; }}

/* ── 按钮 ── */
QPushButton {{
    background-color: #282a40;
    color: #cdd6f4;
    border: 1px solid #363850;
    border-radius: 6px;
    padding: 7px 16px;
    min-height: 22px;
    font-weight: 500;
}}
QPushButton:hover {{
    background-color: #323456;
    border-color: #89b4fa;
}}
QPushButton:pressed {{ background-color: #3d4060; }}
QPushButton:disabled {{
    color: #555770;
    background-color: #1a1b2e;
    border-color: #2a2b40;
}}

QPushButton#emergencyStop {{
    background-color: #c0392b;
    color: white;
    font-weight: bold;
    font-size: 14px;
    border: 2px solid #e74c3c;
    border-radius: 8px;
    min-width: 60px;
}}
QPushButton#emergencyStop:hover {{ background-color: #e74c3c; border-color: #ff6b6b; }}

QPushButton#connectBtn {{
    background-color: #2ecc71;
    color: white;
    font-weight: bold;
    border: 1px solid #27ae60;
}}
QPushButton#connectBtn:hover {{ background-color: #27ae60; border-color: #2ecc71; }}

QPushButton#disconnectBtn {{
    background-color: #e67e22;
    color: white;
    font-weight: bold;
    border: 1px solid #d35400;
}}
QPushButton#disconnectBtn:hover {{ background-color: #d35400; border-color: #e67e22; }}

QPushButton#enableBtn {{
    background-color: #3498db;
    color: white;
    font-weight: bold;
    border: 1px solid #2980b9;
}}
QPushButton#enableBtn:hover {{ background-color: #2980b9; border-color: #3498db; }}

/* ── 输入控件 ── */
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{
    background-color: #222338;
    color: #cdd6f4;
    border: 1px solid #363850;
    border-radius: 5px;
    padding: 5px 8px;
    min-height: 22px;
    selection-background-color: #3d5a99;
}}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{
    border: 1.5px solid #89b4fa;
}}

/* SpinBox 上下按钮 */
QSpinBox::up-button, QDoubleSpinBox::up-button {{
    subcontrol-origin: border;
    subcontrol-position: top right;
    width: 20px;
    border: none;
    border-left: 1px solid #363850;
    border-top-right-radius: 5px;
    background: #282a40;
}}
QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover {{
    background: #363850;
}}
QSpinBox::down-button, QDoubleSpinBox::down-button {{
    subcontrol-origin: border;
    subcontrol-position: bottom right;
    width: 20px;
    border: none;
    border-left: 1px solid #363850;
    border-bottom-right-radius: 5px;
    background: #282a40;
}}
QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover {{
    background: #363850;
}}
QSpinBox::up-arrow, QDoubleSpinBox::up-arrow {{
    image: none;
    border-left: 4px solid transparent;
    border-right: 4px solid transparent;
    border-bottom: 5px solid #8b8fa8;
    width: 0; height: 0;
}}
QSpinBox::up-arrow:hover, QDoubleSpinBox::up-arrow:hover {{
    border-bottom-color: #89b4fa;
}}
QSpinBox::down-arrow, QDoubleSpinBox::down-arrow {{
    image: none;
    border-left: 4px solid transparent;
    border-right: 4px solid transparent;
    border-top: 5px solid #8b8fa8;
    width: 0; height: 0;
}}
QSpinBox::down-arrow:hover, QDoubleSpinBox::down-arrow:hover {{
    border-top-color: #89b4fa;
}}

QComboBox::drop-down {{
    border: none;
    width: 22px;
    border-top-right-radius: 5px;
    border-bottom-right-radius: 5px;
}}
QComboBox QAbstractItemView {{
    background-color: #222338;
    color: #cdd6f4;
    border: 1px solid #363850;
    border-radius: 4px;
    selection-background-color: #2a2b40;
    outline: none;
}}

/* ── 滑块 ── */
QSlider::groove:horizontal {{
    border: none;
    height: 8px;
    background: #363850;
    border-radius: 4px;
}}
QSlider::handle:horizontal {{
    background: #89b4fa;
    border: 2px solid #6a9be6;
    width: 16px;
    height: 16px;
    margin: -5px 0;
    border-radius: 9px;
}}
QSlider::handle:horizontal:hover {{ background: #b4befe; border-color: #89b4fa; }}

/* ── 表格 ── */
QTableWidget {{
    background-color: #1a1b2e;
    color: #cdd6f4;
    gridline-color: #2a2b40;
    border: 1px solid #2a2b40;
    border-radius: 6px;
    selection-background-color: #2a2b40;
}}
QTableWidget::item {{ padding: 5px; }}
QHeaderView::section {{
    background-color: #16172a;
    color: #8b8fa8;
    padding: 7px;
    border: 1px solid #2a2b40;
    font-weight: bold;
}}

/* ── GroupBox ── */
QGroupBox {{
    border: 1px solid #2a2b40;
    border-radius: 8px;
    margin-top: 14px;
    padding-top: 18px;
    font-weight: bold;
    color: #89b4fa;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 6px;
}}

/* ── 特殊标签 ── */
QLabel#statusLabel {{ color: #a6e3a1; font-weight: bold; }}
QLabel#errorLabel {{ color: #f38ba8; font-weight: bold; }}
QLabel#fpsLabel {{ color: #f9e2af; font-size: 12px; }}

/* ── CheckBox ── */
QCheckBox {{ color: #cdd6f4; spacing: 6px; }}
QCheckBox::indicator {{
    width: 18px; height: 18px;
    border: 1.5px solid #363850;
    border-radius: 4px;
    background-color: #222338;
}}
QCheckBox::indicator:checked {{
    background-color: #89b4fa;
    border-color: #89b4fa;
}}
QCheckBox::indicator:hover {{ border-color: #89b4fa; }}

/* ── 滚动条 ── */
QScrollBar:vertical {{
    background: #1a1b2e;
    width: 10px;
    border: none;
    border-radius: 5px;
}}
QScrollBar::handle:vertical {{
    background: #363850;
    border-radius: 5px;
    min-height: 24px;
}}
QScrollBar::handle:vertical:hover {{ background: #4a4d6a; }}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}

QScrollBar:horizontal {{
    background: #1a1b2e;
    height: 10px;
    border: none;
    border-radius: 5px;
}}
QScrollBar::handle:horizontal {{
    background: #363850;
    border-radius: 5px;
    min-width: 24px;
}}
QScrollBar::handle:horizontal:hover {{ background: #4a4d6a; }}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{ width: 0; }}

/* ── 日志控制台 ── */
QTextEdit#logConsole {{
    background-color: #12132a;
    color: #8b8fa8;
    font-family: "JetBrains Mono", "Cascadia Code", "Fira Code", monospace;
    font-size: 12px;
    border: none;
    border-radius: 0;
}}

/* ── 进度条 ── */
QProgressBar {{
    background: #222338;
    border: 1px solid #363850;
    border-radius: 4px;
    text-align: center;
    color: #cdd6f4;
    font-size: 11px;
}}
QProgressBar::chunk {{
    background: #89b4fa;
    border-radius: 3px;
}}
"""

# ─────────────────────── 浅色主题（Modern Light） ───────────────────────

LIGHT_THEME = f"""
/* ── 全局 ── */
QMainWindow, QWidget {{
    background-color: #f8f9fc;
    color: #1d1d2e;
    font-family: {_FONT_STACK};
    font-size: 13px;
}}

/* ── 菜单 ── */
QMenuBar {{
    background-color: #eef0f5;
    color: #1d1d2e;
    border-bottom: 1px solid #d8dbe5;
}}
QMenuBar::item:selected {{ background-color: #d8dbe5; }}
QMenu {{
    background-color: #ffffff;
    color: #1d1d2e;
    border: 1px solid #d8dbe5;
    border-radius: 6px;
    padding: 4px 0;
}}
QMenu::item {{
    padding: 6px 24px;
    border-radius: 4px;
    margin: 2px 4px;
}}
QMenu::item:selected {{ background-color: #eef0f5; }}

/* ── 工具栏 / 状态栏 ── */
QToolBar {{
    background-color: #eef0f5;
    border-bottom: 1px solid #d8dbe5;
    spacing: 6px;
    padding: 4px;
}}
QStatusBar {{
    background-color: #eef0f5;
    color: #6b7084;
    border-top: 1px solid #d8dbe5;
    font-size: 12px;
}}

/* ── Dock ── */
QDockWidget {{
    color: #1d1d2e;
    titlebar-close-icon: none;
}}
QDockWidget::title {{
    background-color: transparent;
    padding: 0;
    border: none;
}}

/* ── Tab ── */
QTabWidget::pane {{
    border: 1px solid #d8dbe5;
    border-top: none;
    background-color: #f8f9fc;
    border-radius: 0 0 8px 8px;
}}
QTabBar::tab {{
    background-color: #eef0f5;
    color: #6b7084;
    padding: 9px 18px;
    border: 1px solid #d8dbe5;
    border-bottom: none;
    margin-right: 3px;
    border-radius: 6px 6px 0 0;
}}
QTabBar::tab:selected {{
    background-color: #f8f9fc;
    color: #3478f6;
    border-bottom: 2px solid #3478f6;
}}
QTabBar::tab:hover:!selected {{ color: #1d1d2e; background-color: #e8eaf2; }}

/* ── 按钮 ── */
QPushButton {{
    background-color: #e8eaf2;
    color: #1d1d2e;
    border: 1px solid #c8cad5;
    border-radius: 6px;
    padding: 7px 16px;
    min-height: 22px;
    font-weight: 500;
}}
QPushButton:hover {{
    background-color: #dcdfe8;
    border-color: #3478f6;
}}
QPushButton:pressed {{ background-color: #cfd2dd; }}
QPushButton:disabled {{
    color: #a0a3b0;
    background-color: #f0f1f5;
    border-color: #dcdfe8;
}}

QPushButton#emergencyStop {{
    background-color: #ff3b30;
    color: white;
    font-weight: bold;
    font-size: 14px;
    border: 2px solid #ff3b30;
    border-radius: 8px;
    min-width: 60px;
}}
QPushButton#emergencyStop:hover {{ background-color: #ff6961; border-color: #ff6961; }}

QPushButton#connectBtn {{
    background-color: #34c759;
    color: white;
    font-weight: bold;
    border: 1px solid #28a745;
}}
QPushButton#connectBtn:hover {{ background-color: #28a745; border-color: #34c759; }}

QPushButton#disconnectBtn {{
    background-color: #ff9500;
    color: white;
    font-weight: bold;
    border: 1px solid #e08600;
}}
QPushButton#disconnectBtn:hover {{ background-color: #e08600; border-color: #ff9500; }}

QPushButton#enableBtn {{
    background-color: #3478f6;
    color: white;
    font-weight: bold;
    border: 1px solid #2860d8;
}}
QPushButton#enableBtn:hover {{ background-color: #2860d8; border-color: #3478f6; }}

/* ── 输入控件 ── */
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {{
    background-color: #ffffff;
    color: #1d1d2e;
    border: 1px solid #c8cad5;
    border-radius: 5px;
    padding: 5px 8px;
    min-height: 22px;
    selection-background-color: #b3d4fc;
}}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {{
    border: 1.5px solid #3478f6;
}}

/* SpinBox 上下按钮 */
QSpinBox::up-button, QDoubleSpinBox::up-button {{
    subcontrol-origin: border;
    subcontrol-position: top right;
    width: 20px;
    border: none;
    border-left: 1px solid #d8dbe5;
    border-top-right-radius: 5px;
    background: #eef0f5;
}}
QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover {{
    background: #dcdfe8;
}}
QSpinBox::down-button, QDoubleSpinBox::down-button {{
    subcontrol-origin: border;
    subcontrol-position: bottom right;
    width: 20px;
    border: none;
    border-left: 1px solid #d8dbe5;
    border-bottom-right-radius: 5px;
    background: #eef0f5;
}}
QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover {{
    background: #dcdfe8;
}}
QSpinBox::up-arrow, QDoubleSpinBox::up-arrow {{
    image: none;
    border-left: 4px solid transparent;
    border-right: 4px solid transparent;
    border-bottom: 5px solid #6b7084;
    width: 0; height: 0;
}}
QSpinBox::up-arrow:hover, QDoubleSpinBox::up-arrow:hover {{
    border-bottom-color: #3478f6;
}}
QSpinBox::down-arrow, QDoubleSpinBox::down-arrow {{
    image: none;
    border-left: 4px solid transparent;
    border-right: 4px solid transparent;
    border-top: 5px solid #6b7084;
    width: 0; height: 0;
}}
QSpinBox::down-arrow:hover, QDoubleSpinBox::down-arrow:hover {{
    border-top-color: #3478f6;
}}

QComboBox::drop-down {{
    border: none;
    width: 22px;
    border-top-right-radius: 5px;
    border-bottom-right-radius: 5px;
}}
QComboBox QAbstractItemView {{
    background-color: #ffffff;
    color: #1d1d2e;
    border: 1px solid #d8dbe5;
    border-radius: 4px;
    selection-background-color: #eef0f5;
    outline: none;
}}

/* ── 滑块 ── */
QSlider::groove:horizontal {{
    border: none;
    height: 8px;
    background: #d8dbe5;
    border-radius: 4px;
}}
QSlider::handle:horizontal {{
    background: #3478f6;
    border: 2px solid #2860d8;
    width: 16px;
    height: 16px;
    margin: -5px 0;
    border-radius: 9px;
}}
QSlider::handle:horizontal:hover {{ background: #5a94ff; border-color: #3478f6; }}

/* ── 表格 ── */
QTableWidget {{
    background-color: #ffffff;
    color: #1d1d2e;
    gridline-color: #d8dbe5;
    border: 1px solid #d8dbe5;
    border-radius: 6px;
    selection-background-color: #eef0f5;
}}
QTableWidget::item {{ padding: 5px; }}
QHeaderView::section {{
    background-color: #eef0f5;
    color: #6b7084;
    padding: 7px;
    border: 1px solid #d8dbe5;
    font-weight: bold;
}}

/* ── GroupBox ── */
QGroupBox {{
    border: 1px solid #d8dbe5;
    border-radius: 8px;
    margin-top: 14px;
    padding-top: 18px;
    font-weight: bold;
    color: #3478f6;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 6px;
}}

/* ── 特殊标签 ── */
QLabel#statusLabel {{ color: #34c759; font-weight: bold; }}
QLabel#errorLabel {{ color: #ff3b30; font-weight: bold; }}
QLabel#fpsLabel {{ color: #ff9500; font-size: 12px; }}

/* ── CheckBox ── */
QCheckBox {{ color: #1d1d2e; spacing: 6px; }}
QCheckBox::indicator {{
    width: 18px; height: 18px;
    border: 1.5px solid #c8cad5;
    border-radius: 4px;
    background-color: #ffffff;
}}
QCheckBox::indicator:checked {{
    background-color: #3478f6;
    border-color: #3478f6;
}}
QCheckBox::indicator:hover {{ border-color: #3478f6; }}

/* ── 滚动条 ── */
QScrollBar:vertical {{
    background: #f8f9fc;
    width: 10px;
    border: none;
    border-radius: 5px;
}}
QScrollBar::handle:vertical {{
    background: #c8cad5;
    border-radius: 5px;
    min-height: 24px;
}}
QScrollBar::handle:vertical:hover {{ background: #a0a3b0; }}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}

QScrollBar:horizontal {{
    background: #f8f9fc;
    height: 10px;
    border: none;
    border-radius: 5px;
}}
QScrollBar::handle:horizontal {{
    background: #c8cad5;
    border-radius: 5px;
    min-width: 24px;
}}
QScrollBar::handle:horizontal:hover {{ background: #a0a3b0; }}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{ width: 0; }}

/* ── 日志控制台 ── */
QTextEdit#logConsole {{
    background-color: #ffffff;
    color: #4a4d5e;
    font-family: "JetBrains Mono", "Cascadia Code", "Fira Code", monospace;
    font-size: 12px;
    border: none;
    border-radius: 0;
}}

/* ── 进度条 ── */
QProgressBar {{
    background: #e8eaf2;
    border: 1px solid #d8dbe5;
    border-radius: 4px;
    text-align: center;
    color: #1d1d2e;
    font-size: 11px;
}}
QProgressBar::chunk {{
    background: #3478f6;
    border-radius: 3px;
}}
"""

# ─────────────────────── 主题映射 ───────────────────────

THEMES = {
    "dark": DARK_THEME,
    "light": LIGHT_THEME,
}

# ─────────────────────── 3D 场景配色 ───────────────────────

SCENE_COLORS = {
    "dark": {
        "bg_bottom": "#1a1b2e",
        "bg_top": "#252640",
        "ground": "#2a2b40",
        "ground_opacity": 0.3,
        "pg_bg": "#1a1b2e",
        "pg_fg": "#cdd6f4",
        "separator": "#363850",
        "subtext": "#8b8fa8",
        "text": "#cdd6f4",
        "header_text": "#8b8fa8",
        "accent": "#89b4fa",
        "success": "#a6e3a1",
        "error": "#f38ba8",
        "warning": "#f9e2af",
        "cancel_bg": "#e67e22",
        "recording_bg": "#e74c3c",
        "zt_active_bg": "#e67e22",
        "mode_zt": "#e67e22",
        "mode_estop": "#e74c3c",
        "mode_normal": "#a6e3a1",
        "mapped_bg": "#16172a",
        "mapped_fg": "#cdd6f4",
        "card_bg": "#222338",
        "card_border": "#2a2b40",
        "btn_bg": "#282a40",
        "btn_border": "#363850",
        "btn_hover_border": "#89b4fa",
        "indicator_off_bg": "#222338",
        "indicator_off_border": "#363850",
        "indicator_off_fg": "#555770",
        "indicator_on_bg": "#a6e3a1",
        "indicator_on_border": "#a6e3a1",
        "indicator_on_fg": "#1a1b2e",
        "progress_bg": "#222338",
        "progress_border": "#363850",
        "progress_chunk": "#89b4fa",
        "disabled_fg": "#555770",
        "disabled_bg": "#1a1b2e",
    },
    "light": {
        "bg_bottom": "#f0f1f5",
        "bg_top": "#ffffff",
        "ground": "#d8dbe5",
        "ground_opacity": 0.25,
        "pg_bg": "#ffffff",
        "pg_fg": "#1d1d2e",
        "separator": "#c8cad5",
        "subtext": "#6b7084",
        "text": "#1d1d2e",
        "header_text": "#6b7084",
        "accent": "#3478f6",
        "success": "#34c759",
        "error": "#ff3b30",
        "warning": "#ff9500",
        "cancel_bg": "#ff9500",
        "recording_bg": "#ff3b30",
        "zt_active_bg": "#ff9500",
        "mode_zt": "#ff9500",
        "mode_estop": "#ff3b30",
        "mode_normal": "#34c759",
        "mapped_bg": "#eef0f5",
        "mapped_fg": "#1d1d2e",
        "card_bg": "#ffffff",
        "card_border": "#d8dbe5",
        "btn_bg": "#e8eaf2",
        "btn_border": "#c8cad5",
        "btn_hover_border": "#3478f6",
        "indicator_off_bg": "#e8eaf2",
        "indicator_off_border": "#d8dbe5",
        "indicator_off_fg": "#a0a3b0",
        "indicator_on_bg": "#34c759",
        "indicator_on_border": "#34c759",
        "indicator_on_fg": "#ffffff",
        "progress_bg": "#e8eaf2",
        "progress_border": "#d8dbe5",
        "progress_chunk": "#3478f6",
        "disabled_fg": "#a0a3b0",
        "disabled_bg": "#f0f1f5",
    },
}

# ─────────────────────── 关节颜色（两套主题通用） ───────────────────────

JOINT_COLORS = [
    "#f38ba8",  # L1 - red
    "#fab387",  # L2 - peach
    "#f9e2af",  # L3 - yellow
    "#a6e3a1",  # L4 - green
    "#89b4fa",  # L5 - blue
    "#cba6f7",  # L6 - mauve
    "#94e2d5",  # L7 - teal
]
