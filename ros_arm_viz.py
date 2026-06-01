#!/usr/bin/env python3
"""
EL-A3 仿真机械臂 2D 可视化 (纯 Qt QPainter, 无需 OpenGL)

用法:
    source el_a3_ros/install/setup.bash
    python ros_arm_viz.py
"""

import sys
import math
import threading
from pathlib import Path

import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                              QHBoxLayout, QLabel, QSlider)
from PyQt6.QtCore import Qt, QTimer, QPointF
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFont, QPainterPath

import rclpy
from sensor_msgs.msg import JointState

# ── FK ──
sys.path.insert(0, str(Path(__file__).parent))
from el_a3_sdk.kinematics import ELA3Kinematics

JOINT_NAMES = ["L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint", "L7_joint"]

# 连杆颜色
LINK_COLORS = [
    QColor("#4a5568"),  # base
    QColor("#e53e3e"),  # L1
    QColor("#dd6b20"),  # L2
    QColor("#d69e2e"),  # L3
    QColor("#38a169"),  # L4
    QColor("#3182ce"),  # L5
    QColor("#805ad5"),  # L6
]


class ArmCanvas(QWidget):
    """纯 Qt 2D 机械臂绘制 (三视图: 俯视 + 主视 + 侧视)"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(800, 600)
        self.setStyleSheet("background: #0f1116;")

        self._kin = None
        try:
            self._kin = ELA3Kinematics()
        except Exception as e:
            print(f"[Viz] Kinematics init failed: {e}", file=sys.stderr)
        self._lock = threading.Lock()
        self._joint_positions = [0.0] * 7
        self._dirty = True
        self._zoom = 1.0

        # 视角角度
        self._view_elev = 30
        self._view_azim = -60

    def set_joints(self, positions):
        with self._lock:
            self._joint_positions = list(positions)
            self._dirty = True

    def paintEvent(self, event):
        with self._lock:
            positions = list(self._joint_positions)
            dirty = self._dirty
            self._dirty = False

        if not dirty and hasattr(self, '_cached_pts') and self._cached_pts is not None:
            pts = self._cached_pts
        else:
            pts = self._compute_fk_points(positions)
            if pts is None:
                pts = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]])
            self._cached_pts = pts

        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()
        cx = w / 2
        cy = h / 2
        scale = min(w, h) * 0.35 * self._zoom

        # 网格
        pen = QPen(QColor("#1e293b"), 1, Qt.PenStyle.DotLine)
        p.setPen(pen)
        for i in range(-5, 6):
            x = cx + i * scale * 0.15
            p.drawLine(QPointF(x, cy - scale * 0.75), QPointF(x, cy + scale * 0.75))
            y = cy + i * scale * 0.15
            p.drawLine(QPointF(cx - scale * 0.75, y), QPointF(cx + scale * 0.75, y))

        # 坐标轴
        p.setPen(QPen(QColor("#475569"), 2))
        p.drawLine(QPointF(cx, 30), QPointF(cx, h - 30))

        # 3D → 2D 投影
        elev_rad = math.radians(self._view_elev)
        azim_rad = math.radians(self._view_azim)
        cos_e = math.cos(elev_rad)
        sin_e = math.sin(elev_rad)
        cos_a = math.cos(azim_rad)
        sin_a = math.sin(azim_rad)

        def project(x3, y3, z3):
            xr = cos_a * x3 + sin_a * y3
            yr = -sin_a * x3 + cos_a * y3
            zr = cos_e * yr + sin_e * z3
            sx = cx + xr * scale
            sy = cy - zr * scale
            return sx, sy

        # 画连杆
        for i in range(len(pts) - 1):
            x1, y1 = project(pts[i][0], pts[i][1], pts[i][2])
            x2, y2 = project(pts[i+1][0], pts[i+1][1], pts[i+1][2])

            color = LINK_COLORS[min(i, len(LINK_COLORS)-1)]
            p.setPen(QPen(color, 4 if i == 0 else 3))
            p.drawLine(QPointF(x1, y1), QPointF(x2, y2))

            # 关节圆
            r = 6 if i > 0 else 8
            p.setBrush(QBrush(color))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawEllipse(QPointF(x1, y1), r, r)

        # 最后一个关节 (末端)
        x_e, y_e = project(pts[-1][0], pts[-1][1], pts[-1][2])
        p.setBrush(QBrush(QColor("#f56565")))
        p.setPen(QPen(QColor("#fc8181"), 2))
        p.drawEllipse(QPointF(x_e, y_e), 8, 8)

        # 标签
        p.setPen(QColor("#e2e8f0"))
        p.setFont(QFont("monospace", 10))
        ee = pts[-1]
        p.drawText(10, 20, f"EE: ({ee[0]:+.3f}, {ee[1]:+.3f}, {ee[2]:+.3f}) m")

        # 关节角
        y_off = 40
        p.setFont(QFont("monospace", 9))
        for i, name in enumerate(JOINT_NAMES[:6]):
            ang = math.degrees(positions[i]) if i < len(positions) else 0
            color = LINK_COLORS[min(i+1, len(LINK_COLORS)-1)]
            p.setPen(color)
            p.drawText(10, y_off, f"{name}: {ang:+.1f}°")
            y_off += 16

        # 视角信息
        p.setPen(QColor("#64748b"))
        p.setFont(QFont("sans-serif", 9))
        p.drawText(w - 200, h - 20,
                   f"elev={self._view_elev}°  azim={self._view_azim}°  zoom={self._zoom:.1f}x  "
                   f"drag to rotate, scroll to zoom")

    def _compute_fk_points(self, positions):
        """Pinocchio FK → 各关节世界坐标"""
        if self._kin is None:
            return np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]])
        try:
            import pinocchio as pin
            q = list(positions[:6]) + [0.0]
            q_pin = np.array(q, dtype=float)
            if len(q_pin) < self._kin.model.nq:
                q_pin = np.concatenate([q_pin, np.zeros(self._kin.model.nq - len(q_pin))])
            pin.forwardKinematics(self._kin.model, self._kin.data, q_pin)
            pin.updateFramePlacements(self._kin.model, self._kin.data)

            pts = [[0.0, 0.0, 0.0]]
            for name in JOINT_NAMES[:6]:
                try:
                    joint_id = self._kin.model.getJointId(name)
                    if joint_id < self._kin.model.njoints:
                        p = self._kin.data.oMi[joint_id].translation
                        pts.append([float(p[0]), float(p[1]), float(p[2])])
                except Exception:
                    pass
            return np.array(pts)
        except Exception:
            return np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]])

    # ── 鼠标交互 ──
    def mousePressEvent(self, event):
        self._last_pos = event.position()

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.MouseButton.LeftButton:
            dx = event.position().x() - self._last_pos.x()
            dy = event.position().y() - self._last_pos.y()
            self._view_azim = (self._view_azim + dx * 0.5) % 360
            self._view_elev = max(-89, min(89, self._view_elev + dy * 0.5))
            self._last_pos = event.position()
            self._cached_pts = None  # force redraw
            self.update()

    def wheelEvent(self, event):
        delta = event.angleDelta().y() / 120
        self._zoom *= 1.1 ** delta
        self._zoom = max(0.1, min(5.0, self._zoom))
        self._cached_pts = None
        self.update()


class ArmViewerWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("EL-A3 VR Teleop — Simulation Viewer")
        self.resize(900, 750)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)

        self._canvas = ArmCanvas()
        layout.addWidget(self._canvas)

        # 底部状态栏
        self._status = QLabel("Waiting for /joint_states...")
        self._status.setFixedHeight(24)
        self._status.setStyleSheet("color: #94a3b8; font-size: 12px; padding: 2px 8px; background: #161b22;")
        layout.addWidget(self._status)

        # ROS
        self._ros_node = rclpy.create_node("arm_viewer")
        self._ros_node.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._ros_thread = threading.Thread(target=self._ros_spin, daemon=True, name="ros_viz")
        self._ros_thread.start()

        # 30Hz 刷新
        self._timer = QTimer()
        self._timer.timeout.connect(self._update_view)
        self._timer.start(33)

        self._joints = [0.0] * 7

    def _ros_spin(self):
        while rclpy.ok():
            rclpy.spin_once(self._ros_node, timeout_sec=0.05)

    def _on_joint_state(self, msg):
        positions = [0.0] * 7
        for i, name in enumerate(JOINT_NAMES):
            try:
                idx = msg.name.index(name)
                positions[i] = msg.position[idx]
            except ValueError:
                pass
        self._joints = positions

    def _update_view(self):
        self._canvas.set_joints(self._joints)
        self._canvas.update()

        # 状态文本
        q = self._joints
        self._status.setText(
            f"  L1={math.degrees(q[0]):+.1f}°  L2={math.degrees(q[1]):+.1f}°  "
            f"L3={math.degrees(q[2]):+.1f}°  L4={math.degrees(q[3]):+.1f}°  "
            f"L5={math.degrees(q[4]):+.1f}°  L6={math.degrees(q[5]):+.1f}°"
        )

    def closeEvent(self, event):
        if hasattr(self, '_ros_node'):
            self._ros_node.destroy_node()
        super().closeEvent(event)


def main():
    if not rclpy.ok():
        rclpy.init(args=sys.argv)
    app = QApplication(sys.argv)
    win = ArmViewerWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
