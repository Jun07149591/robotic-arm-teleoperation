"""3D URDF 可视化面板 (PyVistaQt) — 含单关节拖拽控制"""

import sys
import math
import time
import numpy as np
import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QHBoxLayout, QPushButton,
    QApplication, QSizePolicy, QMessageBox,
)
from PyQt6.QtCore import Qt, QEvent, pyqtSignal
from PyQt6.QtGui import QCursor

logger = logging.getLogger("MotorStudio.viewer3d")

try:
    import pyvista as pv
    from pyvistaqt import QtInteractor
    from vtkmodules.vtkRenderingCore import vtkCellPicker
    HAS_PYVISTA = True
except Exception as _exc:
    HAS_PYVISTA = False
    logger.warning("pyvista / pyvistaqt 不可用，3D 可视化已禁用: %s", _exc)

from MotorStudio.utils.urdf_loader import UrdfModel, _make_transform
from MotorStudio.utils.joint_drag_controls import JointDragController
from MotorStudio.utils.i18n import tr
from MotorStudio.utils.theme_manager import ThemeManager
from MotorStudio.utils.style import SCENE_COLORS

def _get_base_path() -> Path:
    if getattr(sys, "frozen", False):
        return Path(sys._MEIPASS)
    return Path(__file__).parent.parent.parent

URDF_PATH = _get_base_path() / "resources" / "urdf" / "el_a3.urdf"
if getattr(sys, "frozen", False):
    MESH_DIR = _get_base_path() / "resources" / "meshes"
else:
    MESH_DIR = Path(__file__).parent.parent.parent.parent / "el_a3_ros" / "el_a3_description" / "meshes"

JOINT_NAMES_ORDERED = [
    "L1_joint", "L2_joint", "L3_joint",
    "L4_joint", "L5_joint", "L6_joint", "L7_joint",
]

UPDATE_INTERVAL_S = 0.1  # 10 Hz max 3D refresh

# 材质高亮参数（模拟 robot_viewer 的 emissive 白色泛光效果）
HOVER_STYLE = {"color": (1.0, 1.0, 1.0), "ambient": 0.45, "diffuse": 0.55, "specular": 0.4}
DRAG_STYLE = {"color": (1.0, 1.0, 1.0), "ambient": 0.6, "diffuse": 0.4, "specular": 0.5}


def _btn_sp() -> QSizePolicy:
    """Preferred width, fixed height — buttons auto-fit text."""
    return QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)


class Viewer3DPanel(QWidget):
    """3D URDF 实时可视化 + 单关节拖拽"""

    drag_angles_changed = pyqtSignal(dict)
    home_position_requested = pyqtSignal()

    def __init__(self, parent=None, urdf_path=None, mesh_dir=None):
        super().__init__(parent)
        self._urdf_path = Path(urdf_path) if urdf_path else URDF_PATH
        self._mesh_dir = Path(mesh_dir) if mesh_dir else MESH_DIR
        self._model: Optional[UrdfModel] = None
        self._plotter: Optional[QtInteractor] = None

        self._link_actors: Dict[str, List[Tuple]] = {}
        self._link_meshes: Dict[str, List[Tuple]] = {}
        self._current_angles = {name: 0.0 for name in JOINT_NAMES_ORDERED}
        self._initialized = False
        self._last_update_time = 0.0

        # Drag state
        self._drag_ctrl: Optional[JointDragController] = None
        self._drag_mode = False
        self._actor_to_link: Dict[int, str] = {}
        self._picker: Optional[object] = None
        self._highlighted_actors: List[Tuple] = []
        self._hovered_link: Optional[str] = None
        self._filter_installed = False
        self._enabled = False
        self._tcp_actor = None
        self._tcp_visible = False

        self._init_ui()

    # ==================================================================
    # UI 构建
    # ==================================================================

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        if not HAS_PYVISTA:
            label = QLabel(tr("v3d.no_pyvista"))
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            sc = SCENE_COLORS[ThemeManager.instance().theme]
            label.setStyleSheet(f"color: {sc['error']}; font-size: 14px; padding: 40px;")
            layout.addWidget(label)
            return

        # --- 第一行：视图控制 ---
        row1 = QHBoxLayout()
        row1.setContentsMargins(4, 2, 4, 0)
        self._reset_btn = QPushButton(tr("v3d.reset"))
        self._reset_btn.setSizePolicy(_btn_sp())
        self._reset_btn.clicked.connect(self._reset_view)
        row1.addWidget(self._reset_btn)

        self._home_btn = QPushButton(tr("v3d.home_model"))
        self._home_btn.setSizePolicy(_btn_sp())
        self._home_btn.clicked.connect(self._on_home_clicked)
        row1.addWidget(self._home_btn)

        row1.addStretch()
        layout.addLayout(row1)

        # --- 状态标签 ---
        self._status_label = QLabel("")
        self._status_label.setFixedHeight(18)
        sc0 = SCENE_COLORS[ThemeManager.instance().theme]
        self._status_label.setStyleSheet(
            f"color: {sc0['subtext']}; font-size: 11px; padding-left: 4px;"
        )
        layout.addWidget(self._status_label)

        # --- 3D 渲染器 ---
        try:
            pv.global_theme.allow_empty_mesh = True
            self._plotter = QtInteractor(self, multi_samples=8)
            sc = SCENE_COLORS[ThemeManager.instance().theme]
            self._plotter.set_background(sc["bg_bottom"], top=sc["bg_top"])
            layout.addWidget(self._plotter.interactor)
        except Exception as e:
            logger.error(f"创建 3D 渲染器失败: {e}")
            self._plotter = None
            label = QLabel(tr("v3d.init_fail", e=e))
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            sc_err = SCENE_COLORS[ThemeManager.instance().theme]
            label.setStyleSheet(f"color: {sc_err['error']}; font-size: 13px; padding: 20px;")
            label.setWordWrap(True)
            layout.addWidget(label)
            return

    def apply_theme(self):
        """Update 3D scene colors and themed widgets for current theme."""
        tm = ThemeManager.instance()
        sc = SCENE_COLORS[tm.theme]
        if self._plotter:
            try:
                self._plotter.set_background(sc["bg_bottom"], top=sc["bg_top"])
                self._plotter.render()
            except Exception:
                pass
        if hasattr(self, "_status_label"):
            self._status_label.setStyleSheet(
                f"color: {sc['subtext']}; font-size: 11px; padding-left: 4px;"
            )

    def retranslate_ui(self):
        if hasattr(self, '_reset_btn'):
            self._reset_btn.setText(tr("v3d.reset"))
        if hasattr(self, '_home_btn'):
            self._home_btn.setText(tr("v3d.home_model"))

    # ==================================================================
    # 模型加载
    # ==================================================================

    def initialize_model(self):
        """加载 URDF 和 mesh，添加到场景（只调用一次）"""
        if not HAS_PYVISTA or self._plotter is None:
            return False
        if self._initialized:
            return True

        if not self._urdf_path.exists():
            logger.warning(f"URDF 文件不存在: {self._urdf_path}")
            return False
        if not self._mesh_dir.exists():
            logger.warning(f"Mesh 目录不存在: {self._mesh_dir}")
            return False

        try:
            sc = SCENE_COLORS[ThemeManager.instance().theme]
            self._plotter.set_background(sc["bg_bottom"], top=sc["bg_top"])
            self._model = UrdfModel(str(self._urdf_path), str(self._mesh_dir))
            self._link_meshes = self._model.load_meshes()
            logger.info(f"加载 {len(self._link_meshes)} 个 link mesh")

            transforms = self._model.compute_link_transforms(self._current_angles)

            for link_name, mesh_list in self._link_meshes.items():
                link_T = transforms.get(link_name, np.eye(4))
                actor_entries = []
                for mesh, vis_T, color in mesh_list:
                    world_T = link_T @ vis_T
                    display_mesh = mesh.copy()
                    display_mesh.compute_normals(
                        cell_normals=False, point_normals=True,
                        split_vertices=True, feature_angle=30.0,
                        inplace=True,
                    )
                    display_mesh.transform(world_T, inplace=True)
                    rgba = color if color is not None else [1.0, 1.0, 1.0, 1.0]
                    mesh_color = [rgba[0], rgba[1], rgba[2]]
                    mesh_opacity = float(rgba[3]) if len(rgba) > 3 else 1.0
                    actor = self._plotter.add_mesh(
                        display_mesh,
                        color=mesh_color,
                        opacity=mesh_opacity,
                        smooth_shading=True,
                        show_edges=False,
                        specular=0.3,
                        specular_power=50,
                        diffuse=0.65,
                        ambient=0.25,
                        name=f"{link_name}_{id(mesh)}",
                    )
                    actor.GetProperty().SetInterpolationToPhong()
                    orig_smooth = mesh.copy()
                    orig_smooth.compute_normals(
                        cell_normals=False, point_normals=True,
                        split_vertices=True, feature_angle=30.0,
                        inplace=True,
                    )
                    actor_entries.append((actor, orig_smooth, vis_T, display_mesh))
                    self._actor_to_link[id(actor)] = link_name
                self._link_actors[link_name] = actor_entries

            self._plotter.add_axes()
            grid = pv.Plane(
                center=(0, 0, -0.01), direction=(0, 0, 1),
                i_size=1.0, j_size=1.0, i_resolution=10, j_resolution=10,
            )
            self._plotter.add_mesh(
                grid,
                color=sc["ground"],
                opacity=sc["ground_opacity"],
                name="ground",
            )

            self._plotter.remove_all_lights()
            key_light = pv.Light(
                position=(2, -2, 4), focal_point=(0, 0, 0.15),
                intensity=0.8, light_type="scenelight",
            )
            fill_light = pv.Light(
                position=(-2, 2, 2), focal_point=(0, 0, 0.15),
                intensity=0.4, light_type="scenelight",
            )
            rim_light = pv.Light(
                position=(0, 3, 1), focal_point=(0, 0, 0.15),
                intensity=0.35, light_type="scenelight",
            )
            ambient_light = pv.Light(light_type="headlight", intensity=0.15)
            self._plotter.add_light(key_light)
            self._plotter.add_light(fill_light)
            self._plotter.add_light(rim_light)
            self._plotter.add_light(ambient_light)

            self._add_tcp_marker()

            # 初始化拖拽控制器
            self._drag_ctrl = JointDragController(self._model, JOINT_NAMES_ORDERED)
            self._picker = vtkCellPicker()
            self._picker.SetTolerance(0.005)

            self._reset_view()
            self._initialized = True
            return True

        except Exception as e:
            logger.error(f"模型初始化失败: {e}")
            import traceback
            traceback.print_exc()
            return False

    # ==================================================================
    # 反馈驱动的姿态更新（拖拽期间暂停）
    # ==================================================================

    def update_joint_angles(self, joint_states):
        """根据关节反馈更新 3D 模型姿态（节流到 10Hz）"""
        if not self._initialized or self._model is None:
            return
        if self._drag_mode:
            return

        now = time.monotonic()
        if now - self._last_update_time < UPDATE_INTERVAL_S:
            return
        self._last_update_time = now

        positions = joint_states.to_list(include_gripper=True)
        changed = False
        for i, name in enumerate(JOINT_NAMES_ORDERED):
            if i < len(positions):
                if abs(self._current_angles[name] - positions[i]) > 1e-5:
                    self._current_angles[name] = positions[i]
                    changed = True

        if not changed:
            return

        self._render_pose(self._current_angles)

    # ==================================================================
    # 拖拽模式
    # ==================================================================

    def set_drag_mode(self, enabled: bool):
        """Enable or disable drag mode (called by TrajectoryPanel)."""
        self._drag_mode = enabled
        if enabled:
            self._install_event_filter()
            if self._drag_ctrl:
                self._drag_ctrl.sync_from_feedback(self._current_angles)
                self.drag_angles_changed.emit(dict(self._drag_ctrl.target_angles))
            self._status_label.setText(tr("v3d.drag_hint"))
        else:
            self._remove_event_filter()
            self._clear_highlight()
            self._hovered_link = None
            if self._plotter:
                self._plotter.interactor.unsetCursor()
            if self._drag_ctrl:
                self._drag_ctrl.end_drag()
            self._status_label.setText("")

    def _install_event_filter(self):
        if not self._plotter or self._filter_installed:
            return
        self._plotter.interactor.installEventFilter(self)
        self._filter_installed = True

    def _remove_event_filter(self):
        if not self._plotter or not self._filter_installed:
            return
        self._plotter.interactor.removeEventFilter(self)
        self._filter_installed = False

    # ------------------------------------------------------------------
    # Qt 坐标 → VTK display 坐标（Y 轴翻转）
    # ------------------------------------------------------------------

    def _qt_to_vtk_coords(self, qt_event) -> Tuple[int, int]:
        x = int(qt_event.position().x())
        y = self._plotter.interactor.height() - int(qt_event.position().y()) - 1
        return x, y

    # ------------------------------------------------------------------
    # 辅助：拾取鼠标下方的 link
    # ------------------------------------------------------------------

    def _pick_link_at(self, vx: int, vy: int) -> Optional[str]:
        """在 VTK display 坐标 (vx, vy) 做射线拾取，返回命中的 link name 或 None。"""
        renderer = self._plotter.renderer
        hit = self._picker.Pick(vx, vy, 0, renderer)
        if not hit:
            return None
        actor = self._picker.GetActor()
        if actor is None:
            return None
        return self._actor_to_link.get(id(actor))

    # ------------------------------------------------------------------
    # Qt 事件过滤器（替代 VTK AddObserver，彻底隔离拖拽与轨道控制）
    # ------------------------------------------------------------------

    def eventFilter(self, obj, event):
        if not self._drag_mode or not self._drag_ctrl or not self._picker:
            return False

        etype = event.type()

        if etype == QEvent.Type.MouseButtonPress and event.button() == Qt.MouseButton.LeftButton:
            return self._handle_drag_press(event)

        if etype == QEvent.Type.MouseMove:
            if self._drag_ctrl.is_dragging:
                self._handle_drag_move(event)
                return True
            self._handle_hover(event)
            return False

        if etype == QEvent.Type.MouseButtonRelease and event.button() == Qt.MouseButton.LeftButton:
            if self._drag_ctrl.is_dragging:
                self._handle_drag_release()
                return True

        return False

    def _handle_drag_press(self, event) -> bool:
        """处理拖拽模式下的鼠标按下。返回 True 表示消费事件（开始拖拽）。"""
        vx, vy = self._qt_to_vtk_coords(event)
        link_name = self._pick_link_at(vx, vy)
        if link_name is None:
            return False

        pick_pos = np.array(self._picker.GetPickPosition())
        renderer = self._plotter.renderer
        renderer.SetWorldPoint(*pick_pos, 1.0)
        renderer.WorldToDisplay()
        depth = renderer.GetDisplayPoint()[2]

        started = self._drag_ctrl.begin_drag(link_name, pick_pos, depth)
        if not started:
            return False

        joint = self._drag_ctrl.active_joint
        if joint:
            self._highlight_link(joint.child_link, DRAG_STYLE)
            angle_deg = math.degrees(self._drag_ctrl.target_angles.get(joint.name, 0))
            self._status_label.setText(tr("v3d.drag_joint", name=joint.name, deg=angle_deg))
        return True

    def _handle_drag_move(self, event):
        """拖拽过程中处理鼠标移动。"""
        vx, vy = self._qt_to_vtk_coords(event)
        renderer = self._plotter.renderer

        depth = self._drag_ctrl._hit_depth
        renderer.SetDisplayPoint(vx, vy, depth)
        renderer.DisplayToWorld()
        wp = renderer.GetWorldPoint()
        world_pt = np.array(wp[:3]) / wp[3] if abs(wp[3]) > 1e-12 else np.array(wp[:3])

        cam = renderer.GetActiveCamera()
        cam_pos = np.array(cam.GetPosition())
        cam_focal = np.array(cam.GetFocalPoint())
        view_dir = cam_focal - cam_pos
        view_dir /= np.linalg.norm(view_dir) + 1e-12

        new_angle = self._drag_ctrl.update_drag(world_pt, view_dir)
        if new_angle is not None:
            self._render_pose(self._drag_ctrl.target_angles)
            self.drag_angles_changed.emit(dict(self._drag_ctrl.target_angles))
            joint = self._drag_ctrl.active_joint
            if joint:
                angle_deg = math.degrees(new_angle)
                self._status_label.setText(tr("v3d.drag_joint", name=joint.name, deg=angle_deg))

    def _handle_drag_release(self):
        """拖拽结束。"""
        self.drag_angles_changed.emit(dict(self._drag_ctrl.target_angles))
        self._drag_ctrl.end_drag()
        self._clear_highlight()
        self._hovered_link = None
        self._status_label.setText(tr("v3d.drag_done"))

    def _handle_hover(self, event):
        """非拖拽时的 hover 检测：高亮鼠标下方的连杆。"""
        vx, vy = self._qt_to_vtk_coords(event)
        link_name = self._pick_link_at(vx, vy)

        # 查找 link 对应的可拖拽关节（与 robot_viewer 的 findParentJoint 一致）
        resolved = None
        if link_name and self._model:
            joint = self._model.find_ancestor_revolute_joint(link_name)
            if joint and joint.name in self._drag_ctrl.target_angles:
                resolved = joint.child_link

        if resolved == self._hovered_link:
            return

        self._clear_highlight()

        if resolved:
            self._hovered_link = resolved
            self._highlight_link(resolved, HOVER_STYLE)
            self._plotter.interactor.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        else:
            self._hovered_link = None
            self._plotter.interactor.unsetCursor()

    # ==================================================================
    # 渲染 + 高亮
    # ==================================================================

    def _render_pose(self, angles: Dict[str, float]):
        """用给定角度字典更新 3D mesh 位姿并渲染。"""
        if not self._model:
            return

        transforms = self._model.compute_link_transforms(angles)
        for link_name, actor_entries in self._link_actors.items():
            link_T = transforms.get(link_name, np.eye(4))
            for actor, orig_mesh, vis_T, display_mesh in actor_entries:
                world_T = link_T @ vis_T
                R = world_T[:3, :3]
                t = world_T[:3, 3]
                new_points = (R @ orig_mesh.points.copy().T).T + t
                display_mesh.points = new_points
                if "Normals" in orig_mesh.point_data:
                    display_mesh.point_data["Normals"] = (R @ orig_mesh.point_data["Normals"].T).T

        try:
            self._plotter.render()
        except Exception:
            pass

    def _add_tcp_marker(self):
        """添加末端点标记。"""
        if not self._plotter:
            return
        try:
            if self._tcp_actor is not None:
                return
            marker = pv.Sphere(
                radius=0.008,
                center=(0.0, 0.0, 0.0),
                theta_resolution=24,
                phi_resolution=24,
            )
            self._tcp_actor = self._plotter.add_mesh(
                marker,
                color=(0.95, 0.2, 0.2),
                name="tcp_marker",
                smooth_shading=True,
                specular=0.4,
                ambient=0.55,
                diffuse=0.5,
            )
            self._tcp_actor.SetVisibility(False)
        except Exception as e:
            logger.debug("创建 TCP 标记失败: %s", e)

    def update_tcp_point(self, end_pose):
        """更新末端点在场景中的位置。"""
        if not self._plotter or self._tcp_actor is None or end_pose is None:
            return
        try:
            x = float(getattr(end_pose, "x", 0.0))
            y = float(getattr(end_pose, "y", 0.0))
            z = float(getattr(end_pose, "z", 0.0))
            self._tcp_actor.SetPosition(x, y, z)
            self._tcp_actor.SetVisibility(True)
            self._tcp_visible = True
            self._plotter.render()
        except Exception as e:
            logger.debug("更新 TCP 标记失败: %s", e)

    def _highlight_link(self, link_name: str, style: dict):
        """高亮连杆：用材质属性叠加模拟 robot_viewer 的 emissive 泛白效果。"""
        self._clear_highlight()
        entries = self._link_actors.get(link_name, [])
        for actor, *_ in entries:
            try:
                prop = actor.GetProperty()
                saved = (
                    actor,
                    prop.GetColor(),
                    prop.GetAmbient(),
                    prop.GetDiffuse(),
                    prop.GetSpecular(),
                    prop.GetOpacity(),
                )
                self._highlighted_actors.append(saved)
                prop.SetColor(*style["color"])
                prop.SetAmbient(style["ambient"])
                prop.SetDiffuse(style["diffuse"])
                prop.SetSpecular(style["specular"])
                prop.SetOpacity(1.0)
            except Exception:
                pass
        try:
            self._plotter.render()
        except Exception:
            pass

    def _clear_highlight(self):
        """恢复所有高亮 actor 的原始材质属性。"""
        for actor, color, ambient, diffuse, specular, opacity in self._highlighted_actors:
            try:
                prop = actor.GetProperty()
                prop.SetColor(*color)
                prop.SetAmbient(ambient)
                prop.SetDiffuse(diffuse)
                prop.SetSpecular(specular)
                prop.SetOpacity(opacity)
            except Exception:
                pass
        self._highlighted_actors.clear()
        try:
            if self._plotter:
                self._plotter.render()
        except Exception:
            pass

    # ==================================================================
    # 公开方法（供 TrajectoryPanel 调用）
    # ==================================================================

    def set_enabled(self, enabled: bool):
        self._enabled = enabled

    def sync_to_feedback(self):
        """将拖拽控制器目标角度同步为当前反馈角度，并刷新 3D。"""
        if self._drag_ctrl:
            self._drag_ctrl.sync_from_feedback(self._current_angles)
            self._render_pose(self._drag_ctrl.target_angles)
            self.drag_angles_changed.emit(dict(self._drag_ctrl.target_angles))
            self._status_label.setText(tr("v3d.synced"))

    # ==================================================================
    # 原有功能
    # ==================================================================

    def _reset_view(self):
        if self._plotter:
            self._plotter.camera_position = [
                (0.6, -0.6, 0.5),
                (0.0, 0.0, 0.15),
                (0.0, 0.0, 1.0),
            ]

    # ==================================================================
    # 模型回正
    # ==================================================================

    def _on_home_clicked(self):
        """弹出对话框让用户选择回正方式。"""
        box = QMessageBox(self)
        box.setWindowTitle(tr("v3d.home_confirm"))
        box.setText(tr("v3d.home_confirm"))
        btn_3d = box.addButton(tr("v3d.home_3d_only"), QMessageBox.ButtonRole.AcceptRole)
        btn_real = box.addButton(tr("v3d.home_real"), QMessageBox.ButtonRole.ActionRole)
        box.addButton(QMessageBox.StandardButton.Cancel)
        box.exec()

        clicked = box.clickedButton()
        if clicked == btn_3d:
            self._home_model_3d()
        elif clicked == btn_real:
            self._home_model_3d()
            self.home_position_requested.emit()

    def _home_model_3d(self):
        """将 3D 模型所有关节归零并刷新渲染。"""
        for name in JOINT_NAMES_ORDERED:
            self._current_angles[name] = 0.0
        self._render_pose(self._current_angles)
        if self._drag_ctrl:
            self._drag_ctrl.sync_from_feedback(self._current_angles)
        self._status_label.setText(tr("v3d.home_done"))

    def closeEvent(self, event):
        self._remove_event_filter()
        if self._plotter:
            try:
                self._plotter.close()
            except Exception:
                pass
        super().closeEvent(event)
