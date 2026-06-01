# -*- mode: python ; coding: utf-8 -*-
"""PyInstaller spec for EL-A3 Debugger"""

import os
from pathlib import Path
from PyInstaller.utils.hooks import collect_all, collect_submodules

block_cipher = None

SPEC_DIR = os.path.abspath(SPECPATH)

# ── 资源文件 ──────────────────────────────────────────────
datas = [
    (os.path.join(SPEC_DIR, "resources"), "resources"),
]

# ── collect_all 处理复杂的第三方包 ─────────────────────────
pyvista_datas, pyvista_binaries, pyvista_hiddenimports = collect_all("pyvista")
pyvistaqt_datas, pyvistaqt_binaries, pyvistaqt_hiddenimports = collect_all("pyvistaqt")
pyqtgraph_datas, pyqtgraph_binaries, pyqtgraph_hiddenimports = collect_all("pyqtgraph")
matplotlib_datas, matplotlib_binaries, matplotlib_hiddenimports = collect_all("matplotlib")

all_datas = datas + pyvista_datas + pyvistaqt_datas + pyqtgraph_datas + matplotlib_datas
all_binaries = pyvista_binaries + pyvistaqt_binaries + pyqtgraph_binaries + matplotlib_binaries

# ── hiddenimports ─────────────────────────────────────────
hiddenimports = (
    pyvista_hiddenimports
    + pyvistaqt_hiddenimports
    + pyqtgraph_hiddenimports
    + matplotlib_hiddenimports
    + collect_submodules("vtkmodules")
    + collect_submodules("el_a3_sdk")
    + [
        "debugger",
        "debugger.main",
        "debugger.main_window",
        "debugger.backend.arm_worker",
        "debugger.backend.calibration_worker",
        "debugger.backend.data_buffer",
        "debugger.backend.trajectory_recorder",
        "debugger.utils.urdf_loader",
        "debugger.utils.joint_drag_controls",
        "debugger.utils.can_utils",
        "debugger.utils.i18n",
        "debugger.utils.theme_manager",
        "debugger.utils.style",
        "debugger.widgets.toolbar_panel",
        "debugger.widgets.joint_control_panel",
        "debugger.widgets.trajectory_panel",
        "debugger.widgets.teaching_panel",
        "debugger.widgets.calibration_panel",
        "debugger.widgets.diagnostics_panel",
        "debugger.widgets.gripper_panel",
        "debugger.widgets.gamepad_panel",
        "debugger.widgets.viewer_3d",
        "debugger.widgets.monitoring_window",
        "debugger.widgets.monitoring_panel",
        "demo.xbox_control",
        "numpy",
        "numpy.core",
        "pyyaml",
        "yaml",
        "PyQt6",
        "PyQt6.QtWidgets",
        "PyQt6.QtCore",
        "PyQt6.QtGui",
        "PyQt6.sip",
    ]
)

# ── 可选依赖（如存在则打包）────────────────────────────────
try:
    import pinocchio  # noqa: F401
    hiddenimports += collect_submodules("pinocchio")
except ImportError:
    pass

try:
    import serial  # noqa: F401
    hiddenimports += ["serial", "serial.tools", "serial.tools.list_ports"]
except ImportError:
    pass

try:
    import scipy  # noqa: F401
    hiddenimports += collect_submodules("scipy")
except ImportError:
    pass

# ── Analysis ──────────────────────────────────────────────
a = Analysis(
    [os.path.join(SPEC_DIR, "debugger", "main.py")],
    pathex=[SPEC_DIR],
    binaries=all_binaries,
    datas=all_datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        "tkinter",
        "IPython",
        "jupyter",
        "notebook",
        "pytest",
        "sphinx",
    ],
    noarchive=False,
    cipher=block_cipher,
)

pyz = PYZ(a.pure, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="EL-A3-Debugger",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=True,
    icon=None,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name="EL-A3-Debugger",
)
