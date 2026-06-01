"""轻量级 URDF 解析器 + STL 加载 + FK 变换链计算"""

import xml.etree.ElementTree as ET
import numpy as np
from pathlib import Path
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import logging

logger = logging.getLogger("debugger.urdf")

try:
    import pyvista as pv
    HAS_PYVISTA = True
except ImportError:
    HAS_PYVISTA = False


@dataclass
class UrdfVisual:
    """URDF visual 几何体"""
    mesh_path: str = ""
    origin_xyz: np.ndarray = field(default_factory=lambda: np.zeros(3))
    origin_rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))
    color_rgba: np.ndarray = field(default_factory=lambda: np.array([0.5, 0.5, 0.5, 1.0]))


@dataclass
class UrdfLink:
    """URDF link"""
    name: str = ""
    visuals: List[UrdfVisual] = field(default_factory=list)


@dataclass
class UrdfJoint:
    """URDF revolute joint"""
    name: str = ""
    parent_link: str = ""
    child_link: str = ""
    origin_xyz: np.ndarray = field(default_factory=lambda: np.zeros(3))
    origin_rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))
    axis: np.ndarray = field(default_factory=lambda: np.array([0, 0, 1.0]))
    joint_type: str = "fixed"
    lower: float = -3.14
    upper: float = 3.14


def _parse_xyz(s: str) -> np.ndarray:
    return np.array([float(x) for x in s.split()])


def _rpy_to_rotation(rpy: np.ndarray) -> np.ndarray:
    """RPY(XYZ extrinsic) -> 3x3 rotation matrix"""
    cr, cp, cy = np.cos(rpy)
    sr, sp, sy = np.sin(rpy)
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr],
    ])
    return R


def _axis_angle_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation"""
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0],
    ])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K


def _make_transform(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    """构建 4x4 齐次变换"""
    T = np.eye(4)
    T[:3, :3] = _rpy_to_rotation(rpy)
    T[:3, 3] = xyz
    return T


MATERIAL_COLORS = {
    "orange": [0.972549, 0.529412, 0.00392157, 1.0],
    "gray": [0.647059, 0.647059, 0.647059, 1.0],
    "light_blue": [0.768627, 0.886275, 0.952941, 1.0],
    "white": [0.917647, 0.917647, 0.917647, 1.0],
    "dark_gray": [0.301961, 0.301961, 0.301961, 1.0],
    "blue": [0.231373, 0.380392, 0.705882, 1.0],
    "yellow": [0.980392, 0.713725, 0.00392157, 1.0],
    "green": [0.372549, 0.654902, 0.239216, 1.0],
}


class UrdfModel:
    """URDF 模型：解析 + FK + 3D mesh 管理"""

    def __init__(self, urdf_path: str, mesh_dir: str):
        self.urdf_path = Path(urdf_path)
        self.mesh_dir = Path(mesh_dir)
        self.links: Dict[str, UrdfLink] = {}
        self.joints: List[UrdfJoint] = []
        self.revolute_joints: List[UrdfJoint] = []
        self.materials: Dict[str, np.ndarray] = {}
        self._kinematic_chain: List[Tuple[UrdfJoint, str]] = []

        self._child_to_joint: Dict[str, UrdfJoint] = {}

        self._parse()
        self._build_chain()

    def _parse(self):
        tree = ET.parse(self.urdf_path)
        root = tree.getroot()

        for mat_elem in root.findall("material"):
            name = mat_elem.get("name", "")
            color_elem = mat_elem.find("color")
            if color_elem is not None:
                rgba = _parse_xyz(color_elem.get("rgba", "0.5 0.5 0.5 1"))
                self.materials[name] = rgba

        for link_elem in root.findall("link"):
            link_name = link_elem.get("name", "")
            visuals = []
            for vis_elem in link_elem.findall("visual"):
                v = UrdfVisual()
                origin = vis_elem.find("origin")
                if origin is not None:
                    if origin.get("xyz"):
                        v.origin_xyz = _parse_xyz(origin.get("xyz"))
                    if origin.get("rpy"):
                        v.origin_rpy = _parse_xyz(origin.get("rpy"))
                geom = vis_elem.find("geometry")
                if geom is not None:
                    mesh = geom.find("mesh")
                    if mesh is not None:
                        filename = mesh.get("filename", "")
                        v.mesh_path = filename
                mat = vis_elem.find("material")
                if mat is not None:
                    mat_name = mat.get("name", "")
                    if mat_name in self.materials:
                        v.color_rgba = self.materials[mat_name]
                    elif mat_name in MATERIAL_COLORS:
                        v.color_rgba = np.array(MATERIAL_COLORS[mat_name])
                    color_elem = mat.find("color")
                    if color_elem is not None:
                        v.color_rgba = _parse_xyz(color_elem.get("rgba", "0.5 0.5 0.5 1"))
                visuals.append(v)
            self.links[link_name] = UrdfLink(name=link_name, visuals=visuals)

        for joint_elem in root.findall("joint"):
            j = UrdfJoint()
            j.name = joint_elem.get("name", "")
            j.joint_type = joint_elem.get("type", "fixed")
            parent = joint_elem.find("parent")
            child = joint_elem.find("child")
            if parent is not None:
                j.parent_link = parent.get("link", "")
            if child is not None:
                j.child_link = child.get("link", "")
            origin = joint_elem.find("origin")
            if origin is not None:
                if origin.get("xyz"):
                    j.origin_xyz = _parse_xyz(origin.get("xyz"))
                if origin.get("rpy"):
                    j.origin_rpy = _parse_xyz(origin.get("rpy"))
            axis = joint_elem.find("axis")
            if axis is not None:
                j.axis = _parse_xyz(axis.get("xyz", "0 0 1"))
            limit = joint_elem.find("limit")
            if limit is not None:
                j.lower = float(limit.get("lower", "-3.14"))
                j.upper = float(limit.get("upper", "3.14"))
            self.joints.append(j)
            if j.joint_type == "revolute":
                self.revolute_joints.append(j)

    def _build_chain(self):
        """构建从 base_link 到末端的运动链（BFS）"""
        children = {}
        for j in self.joints:
            children.setdefault(j.parent_link, []).append(j)

        visited = set()
        queue = ["base_link"]
        if "base_link" not in self.links:
            for j in self.joints:
                if j.parent_link not in {jj.child_link for jj in self.joints}:
                    queue = [j.parent_link]
                    break

        chain = []
        while queue:
            link_name = queue.pop(0)
            if link_name in visited:
                continue
            visited.add(link_name)
            for j in children.get(link_name, []):
                chain.append((j, j.child_link))
                queue.append(j.child_link)
        self._kinematic_chain = chain
        self._child_to_joint = {j.child_link: j for j in self.joints}

    def compute_link_transforms(self, joint_angles: Dict[str, float]) -> Dict[str, np.ndarray]:
        """
        给定关节角度，计算所有 link 的世界坐标变换矩阵。
        joint_angles: {joint_name: angle_rad}
        returns: {link_name: 4x4 transform}
        """
        transforms = {}
        root_name = self._kinematic_chain[0][0].parent_link if self._kinematic_chain else "base_link"
        transforms[root_name] = np.eye(4)

        for joint, child_link in self._kinematic_chain:
            parent_T = transforms.get(joint.parent_link, np.eye(4))
            joint_T = _make_transform(joint.origin_xyz, joint.origin_rpy)

            if joint.joint_type == "revolute":
                angle = joint_angles.get(joint.name, 0.0)
                R_joint = np.eye(4)
                R_joint[:3, :3] = _axis_angle_rotation(joint.axis, angle)
                child_T = parent_T @ joint_T @ R_joint
            else:
                child_T = parent_T @ joint_T

            transforms[child_link] = child_T

        return transforms

    def get_link_parent_joint(self, link_name: str) -> Optional[UrdfJoint]:
        """查找 link 的直接父关节（即 child_link == link_name 的 joint）"""
        return self._child_to_joint.get(link_name)

    def find_ancestor_revolute_joint(self, link_name: str) -> Optional[UrdfJoint]:
        """向上遍历运动链，找到第一个 revolute 关节（跳过 fixed）。
        对应 robot_viewer 的 findParentJoint 逻辑。"""
        visited = set()
        current = link_name
        while current and current not in visited:
            visited.add(current)
            joint = self._child_to_joint.get(current)
            if joint is None:
                return None
            if joint.joint_type == "revolute":
                return joint
            current = joint.parent_link
        return None

    def resolve_mesh_path(self, mesh_filename: str) -> Optional[Path]:
        """将 package://... 路径解析为实际文件路径"""
        if mesh_filename.startswith("package://"):
            parts = mesh_filename.replace("package://", "").split("/", 1)
            if len(parts) == 2:
                filename = parts[1].split("/")[-1]
                resolved = self.mesh_dir / filename
                if resolved.exists():
                    return resolved
                meshes_subdir = self.mesh_dir / parts[1]
                if meshes_subdir.exists():
                    return meshes_subdir
        local = self.mesh_dir / Path(mesh_filename).name
        if local.exists():
            return local
        return None

    def load_meshes(self) -> Dict[str, List[Tuple]]:
        """
        加载所有 link 的 mesh 数据。
        返回 {link_name: [(pyvista_mesh, visual_transform_4x4, color_rgba), ...]}
        """
        if not HAS_PYVISTA:
            logger.warning("PyVista 未安装，跳过 mesh 加载")
            return {}

        result = {}
        for link_name, link in self.links.items():
            meshes = []
            for vis in link.visuals:
                mesh_path = self.resolve_mesh_path(vis.mesh_path)
                if mesh_path is None:
                    continue
                try:
                    mesh = pv.read(str(mesh_path))
                    vis_T = _make_transform(vis.origin_xyz, vis.origin_rpy)
                    meshes.append((mesh, vis_T, vis.color_rgba))
                except Exception as e:
                    logger.warning(f"加载 mesh {mesh_path} 失败: {e}")
            if meshes:
                result[link_name] = meshes
        return result
