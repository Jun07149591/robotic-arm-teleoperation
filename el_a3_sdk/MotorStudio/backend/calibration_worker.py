"""标定后台线程：多姿态力矩采集 + Pinocchio 惯量参数优化

移植自 el_a3_ros/scripts/dynamics_calibration.py，去除 ROS 依赖，
通过 ArmWorker 信号间接控制机械臂。

支持：
- 基于 hpp-fcl 的全臂 mesh 自碰撞检测（fallback 到 L2/L3 启发式）
- JSONL 增量保存 + 断点续传
"""

import sys
import os
import time
import json
import logging
import numpy as np
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from PyQt6.QtCore import QThread, pyqtSignal, QMutex, QWaitCondition

logger = logging.getLogger("debugger.calibration")

# Prefer pip-installed pinocchio (cmeel prefix) over ROS 2 Python 3.10 packages
def _prefer_cmeel_site_packages():
    cmeel_paths = []
    for base in list(sys.path):
        if "site-packages" in base:
            cmeel = os.path.join(
                base, "cmeel.prefix", "lib",
                f"python{sys.version_info.major}.{sys.version_info.minor}",
                "site-packages",
            )
            if os.path.isdir(cmeel):
                cmeel_paths.append(cmeel)
    for p in reversed(cmeel_paths):
        if p in sys.path:
            sys.path.remove(p)
        sys.path.insert(0, p)

_prefer_cmeel_site_packages()

try:
    import pinocchio as pin
    HAS_PINOCCHIO = True
except ImportError:
    HAS_PINOCCHIO = False

try:
    from scipy.optimize import minimize as scipy_minimize
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

def _get_sdk_root() -> Path:
    if getattr(sys, "frozen", False):
        return Path(sys._MEIPASS)
    return Path(__file__).parent.parent.parent

_SDK_ROOT = _get_sdk_root()
_WORKSPACE_ROOT = _SDK_ROOT.parent

URDF_PATH = _SDK_ROOT / "resources" / "urdf" / "el_a3.urdf"
INERTIA_YAML_PATH = _SDK_ROOT / "resources" / "config" / "inertia_params.yaml"
CAL_DATA_DIR = _SDK_ROOT / "resources" / "config"
CAL_DATA_FILE = CAL_DATA_DIR / "calibration_data.jsonl"

JOINT_NAMES = ["L1_joint", "L2_joint", "L3_joint",
               "L4_joint", "L5_joint", "L6_joint"]

HOME_POSITION = [0.0, 0.785, -0.785, 0.5, 0.5, 0.0]

# Collision pairs to disable (from SRDF: adjacent + structurally safe pairs).
# Pairs NOT in this list will be checked for self-collision.
_DISABLED_COLLISION_PAIRS = [
    ("l1_urdf_urdf_asm", "l1_link_urdf_asm"),
    ("l1_link_urdf_asm", "l2_l3_urdf_asm"),
    ("l2_l3_urdf_asm", "l3_lnik_urdf_asm"),
    ("l3_lnik_urdf_asm", "l4_l5_urdf_asm"),
    ("l4_l5_urdf_asm", "part_9"),
    ("part_9", "l5_l6_urdf_asm"),
    ("l5_l6_urdf_asm", "end_effector"),
    ("l5_l6_urdf_asm", "gripper_link"),
    ("gripper_link", "end_effector"),
    ("l1_urdf_urdf_asm", "l2_l3_urdf_asm"),
    ("l1_link_urdf_asm", "l3_lnik_urdf_asm"),
    ("l2_l3_urdf_asm", "l4_l5_urdf_asm"),
    ("l3_lnik_urdf_asm", "part_9"),
    ("l4_l5_urdf_asm", "l5_l6_urdf_asm"),
    ("part_9", "end_effector"),
    ("base_link", "l1_urdf_urdf_asm"),
    ("l2_l3_urdf_asm", "part_9"),
    ("l2_l3_urdf_asm", "l5_l6_urdf_asm"),
    ("base_link", "l1_link_urdf_asm"),
    ("base_link", "l2_l3_urdf_asm"),
    ("l1_urdf_urdf_asm", "l3_lnik_urdf_asm"),
    ("l1_link_urdf_asm", "l4_l5_urdf_asm"),
    ("l3_lnik_urdf_asm", "l5_l6_urdf_asm"),
    ("l4_l5_urdf_asm", "end_effector"),
    ("l4_l5_urdf_asm", "gripper_link"),
    ("part_9", "gripper_link"),
    ("l2_l3_urdf_asm", "end_effector"),
    ("l2_l3_urdf_asm", "gripper_link"),
    ("l3_lnik_urdf_asm", "end_effector"),
    ("l3_lnik_urdf_asm", "gripper_link"),
]


# ---------------------------------------------------------------------------
# JSONL helpers (per-point incremental save for resume support)
# ---------------------------------------------------------------------------

def _write_meta_line(path: Path, mode: str, total_points: int, home: list):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        meta = {
            "meta": True,
            "mode": mode,
            "total_points": total_points,
            "home": home,
            "start_time": datetime.now().isoformat(),
        }
        f.write(json.dumps(meta) + "\n")


def _append_data_line(path: Path, record: dict):
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(record) + "\n")
        f.flush()
        os.fsync(f.fileno())


def _read_jsonl(path: Path) -> Tuple[Optional[dict], List[dict]]:
    meta = None
    records: List[dict] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            if obj.get("meta"):
                meta = obj
            else:
                records.append(obj)
    return meta, records


def _dedup_records(records: List[dict]) -> List[dict]:
    by_idx: Dict[int, dict] = {}
    for r in records:
        by_idx[r["idx"]] = r
    return [by_idx[k] for k in sorted(by_idx.keys())]


def has_saved_calibration_data() -> Tuple[bool, int, int, Optional[str]]:
    """Check whether a resumable calibration_data.jsonl exists.

    Returns (exists, completed, total, mode).
    """
    if not CAL_DATA_FILE.exists():
        return False, 0, 0, None
    try:
        meta, records = _read_jsonl(CAL_DATA_FILE)
        if meta is None:
            return False, 0, 0, None
        records = _dedup_records(records)
        total = meta.get("total_points", 0)
        return True, len(records), total, meta.get("mode")
    except Exception:
        return False, 0, 0, None


def clear_saved_calibration_data():
    if CAL_DATA_FILE.exists():
        CAL_DATA_FILE.unlink()


@dataclass
class CalibrationConfig:
    mode: str = "high_precision"
    num_points: int = 30
    samples_per_point: int = 50
    settle_time: float = 1.5
    motion_duration: float = 6.0
    joints_to_calibrate: List[int] = field(default_factory=lambda: [0, 1, 2, 3, 4, 5])
    known_masses: Optional[Dict[str, float]] = None
    resume: bool = False


class CalibrationWorker(QThread):
    """后台标定线程。

    通过信号与 UI / ArmWorker 交互：
    - move_j_requested  → MainWindow 转发给 ArmWorker
    - 力矩反馈通过 feed_efforts() 推送
    """

    progress_updated = pyqtSignal(int, int, str)
    calibration_finished = pyqtSignal(dict)
    log_message = pyqtSignal(str)
    error_occurred = pyqtSignal(str)
    move_j_requested = pyqtSignal(list, float, bool)

    def __init__(self, config: CalibrationConfig, parent=None):
        super().__init__(parent)
        self._config = config
        self._stop_flag = False

        self._efforts_lock = QMutex()
        self._latest_efforts: List[float] = [0.0] * 6
        self._latest_positions: List[float] = [0.0] * 6

        self._move_done = False
        self._move_mutex = QMutex()
        self._move_cond = QWaitCondition()

        self._pin_model = None
        self._pin_data = None
        self._collision_model = None
        self._collision_data = None

    def request_stop(self):
        self._stop_flag = True

    def feed_efforts(self, efforts: List[float]):
        self._efforts_lock.lock()
        self._latest_efforts = list(efforts[:6])
        self._efforts_lock.unlock()

    def feed_positions(self, positions: List[float]):
        self._efforts_lock.lock()
        self._latest_positions = list(positions[:6])
        self._efforts_lock.unlock()

    def notify_move_done(self):
        self._move_mutex.lock()
        self._move_done = True
        self._move_cond.wakeAll()
        self._move_mutex.unlock()

    def _read_efforts(self) -> List[float]:
        self._efforts_lock.lock()
        e = list(self._latest_efforts)
        self._efforts_lock.unlock()
        return e

    def _read_positions(self) -> List[float]:
        self._efforts_lock.lock()
        p = list(self._latest_positions)
        self._efforts_lock.unlock()
        return p

    def _move_and_wait(self, target: List[float], duration: float):
        self._move_mutex.lock()
        self._move_done = False
        self._move_mutex.unlock()

        self.move_j_requested.emit(list(target), duration, True)

        timeout_ms = int((duration + 10) * 1000)
        self._move_mutex.lock()
        if not self._move_done:
            self._move_cond.wait(self._move_mutex, timeout_ms)
        self._move_mutex.unlock()

        time.sleep(0.5)

    # ------------------------------------------------------------------
    # Pinocchio
    # ------------------------------------------------------------------

    def _init_pinocchio(self) -> bool:
        if not HAS_PINOCCHIO:
            return False
        try:
            self._pin_model = pin.buildModelFromUrdf(str(URDF_PATH))
            self._pin_data = pin.Data(self._pin_model)
            self.log_message.emit(f"Pinocchio 模型加载成功 ({self._pin_model.njoints} joints)")
            return True
        except Exception as e:
            self.log_message.emit(f"Pinocchio 加载失败: {e}")
            return False

    def _compute_gravity_with_params(self, q: np.ndarray, params: np.ndarray) -> np.ndarray:
        if self._pin_model is None:
            return np.zeros(6)

        saved = []
        for i in range(1, min(7, self._pin_model.nbodies)):
            saved.append({
                "mass": self._pin_model.inertias[i].mass,
                "lever": self._pin_model.inertias[i].lever.copy(),
            })

        try:
            if len(params) > 0:  self._pin_model.inertias[2].mass     = params[0]
            if len(params) > 1:  self._pin_model.inertias[2].lever[0] = params[1]
            if len(params) > 2:  self._pin_model.inertias[3].mass     = params[2]
            if len(params) > 3:  self._pin_model.inertias[3].lever[0] = params[3]
            if len(params) > 4:  self._pin_model.inertias[3].lever[1] = params[4]
            if len(params) > 5:  self._pin_model.inertias[4].mass     = params[5]
            if len(params) > 6:  self._pin_model.inertias[4].lever[0] = params[6]
            if len(params) > 7:  self._pin_model.inertias[4].lever[1] = params[7]
            if len(params) > 8:  self._pin_model.inertias[5].mass     = params[8]
            if len(params) > 9:  self._pin_model.inertias[5].lever[2] = params[9]
            if len(params) > 10: self._pin_model.inertias[6].mass     = params[10]
            if len(params) > 11: self._pin_model.inertias[6].lever[2] = params[11]

            self._pin_data = pin.Data(self._pin_model)
            q_arr = np.array(q, dtype=float)
            if len(q_arr) < self._pin_model.nq:
                q_arr = np.concatenate([q_arr, np.zeros(self._pin_model.nq - len(q_arr))])
            q_pin = q_arr[: self._pin_model.nq]
            tau = pin.rnea(self._pin_model, self._pin_data, q_pin,
                           np.zeros(self._pin_model.nv), np.zeros(self._pin_model.nv))
        finally:
            for i, orig in enumerate(saved):
                self._pin_model.inertias[i + 1].mass = orig["mass"]
                self._pin_model.inertias[i + 1].lever = orig["lever"]
            self._pin_data = pin.Data(self._pin_model)

        return tau[:6]

    # ------------------------------------------------------------------
    # Collision model (hpp-fcl mesh collision)
    # ------------------------------------------------------------------

    def _init_collision_model(self) -> bool:
        """Load URDF collision geometry via Pinocchio + hpp-fcl.

        Returns True if the collision model was loaded successfully.
        Falls back silently (returns False) when hpp-fcl or meshes are
        unavailable — the heuristic check will be used instead.
        """
        if self._pin_model is None:
            return False
        try:
            package_dirs = []
            ros_pkg = _WORKSPACE_ROOT / "el_a3_ros"
            if ros_pkg.is_dir():
                package_dirs.append(str(ros_pkg))
            sdk_res = _SDK_ROOT / "resources"
            if sdk_res.is_dir():
                package_dirs.append(str(sdk_res))

            geom = pin.buildGeomFromUrdf(
                self._pin_model, str(URDF_PATH),
                pin.GeometryType.COLLISION, package_dirs,
            )
            geom.addAllCollisionPairs()

            frame_names: Dict[int, str] = {}
            for fid in range(self._pin_model.nframes):
                frame_names[fid] = self._pin_model.frames[fid].name

            for link_a, link_b in _DISABLED_COLLISION_PAIRS:
                ids_a = [i for i, go in enumerate(geom.geometryObjects)
                         if frame_names.get(go.parentFrame) == link_a]
                ids_b = [i for i, go in enumerate(geom.geometryObjects)
                         if frame_names.get(go.parentFrame) == link_b]
                for ga in ids_a:
                    for gb in ids_b:
                        pair = pin.CollisionPair(ga, gb)
                        if geom.existCollisionPair(pair):
                            geom.removeCollisionPair(pair)

            self._collision_model = geom
            self._collision_data = pin.GeometryData(geom)
            self.log_message.emit(
                f"碰撞模型加载成功 ({len(geom.collisionPairs)} 碰撞对)")
            return True
        except Exception as exc:
            self.log_message.emit(
                f"碰撞模型加载失败 (将使用启发式): {exc}")
            self._collision_model = None
            self._collision_data = None
            return False

    # ------------------------------------------------------------------
    # Self-collision check
    # ------------------------------------------------------------------

    @staticmethod
    def _check_collision_heuristic(cfg: List[float]) -> bool:
        """Legacy L2/L3 angle heuristic (fallback)."""
        l2, l3 = cfg[1], cfg[2]
        if l2 >= 1.2 and l3 > -0.4:
            return True
        if l2 > 1.35 and l3 > -0.6:
            return True
        if l2 >= 1.2 and l3 >= -0.55 and (l2 + l3 * 0.5) > 0.9:
            return True
        return False

    def _check_self_collision(self, cfg: List[float]) -> bool:
        """Full-arm self-collision test using hpp-fcl meshes.

        Falls back to the L2/L3 heuristic when the collision model
        is not available.
        """
        if self._collision_model is None or self._collision_data is None:
            return self._check_collision_heuristic(cfg)

        q = np.array(cfg, dtype=float)
        if len(q) < self._pin_model.nq:
            q = np.concatenate([q, np.zeros(self._pin_model.nq - len(q))])
        q = q[: self._pin_model.nq]

        pin.updateGeometryPlacements(
            self._pin_model, self._pin_data,
            self._collision_model, self._collision_data, q,
        )
        is_colliding = pin.computeCollisions(
            self._collision_model, self._collision_data, True,
        )
        return is_colliding

    # ------------------------------------------------------------------
    # Test configuration generation (ported from dynamics_calibration.py)
    # ------------------------------------------------------------------

    def _generate_test_configs(self, num_points: int) -> List[List[float]]:
        home = np.array(HOME_POSITION)

        if num_points <= 20:
            l2_r = np.linspace(0.4, 1.4, 6)
            l3_r = np.linspace(-1.2, -0.4, 5)
            l4_r = np.array([0.3, 0.7, 1.0])
            l5_r = np.array([0.3, 0.6])
        elif num_points <= 50:
            l2_r = np.linspace(0.3, 1.5, 10)
            l3_r = np.linspace(-1.3, -0.3, 10)
            l4_r = np.linspace(0.3, 1.2, 6)
            l5_r = np.linspace(0.3, 1.0, 5)
        else:
            l2_r = np.linspace(0.25, 1.55, 12)
            l3_r = np.linspace(-1.35, -0.25, 12)
            l4_r = np.linspace(0.25, 1.25, 7)
            l5_r = np.linspace(0.25, 1.05, 5)

        configs: List[List[float]] = []

        for l2 in l2_r:
            cfg = home.copy(); cfg[1] = l2
            configs.append(cfg.tolist())
        for l3 in l3_r:
            cfg = home.copy(); cfg[2] = l3
            configs.append(cfg.tolist())

        for l2 in l2_r[::2]:
            for l3 in l3_r[::2]:
                cfg = home.copy(); cfg[1] = l2; cfg[2] = l3
                configs.append(cfg.tolist())

        for l4 in l4_r:
            for l5 in l5_r:
                cfg = home.copy(); cfg[3] = l4; cfg[4] = l5
                configs.append(cfg.tolist())

        unique: List[List[float]] = []
        for cfg in configs:
            if not any(np.allclose(cfg, u, atol=0.05) for u in unique):
                unique.append(cfg)

        safe = [c for c in unique if not self._check_self_collision(c)]
        skipped = len(unique) - len(safe)
        if skipped:
            self.log_message.emit(f"碰撞过滤: 去除 {skipped} 个自干涉构型")

        if len(safe) > num_points:
            indices = np.linspace(0, len(safe) - 1, num_points, dtype=int)
            safe = [safe[i] for i in indices]

        return safe

    # ------------------------------------------------------------------
    # Optimization
    # ------------------------------------------------------------------

    def _objective(self, params: np.ndarray,
                   data: List[Tuple[np.ndarray, np.ndarray]]) -> float:
        total = 0.0
        for positions, measured in data:
            predicted = self._compute_gravity_with_params(positions, params)
            for i in range(1, 6):
                total += (measured[i] - predicted[i]) ** 2
        return total

    def _optimize_full(self, data: List[Tuple[np.ndarray, np.ndarray]]) -> Dict:
        """12-param full optimization: mass + CoG for L2-L6."""
        initial = np.array([
            0.8348, 0.095,
            0.1976, -0.056, 0.049,
            0.4606, -0.024, 0.031,
            0.0180, 0.018,
            0.5313, 0.070,
        ])
        bounds = [
            (0.1, 2.0), (-0.2, 0.2),
            (0.05, 0.5), (-0.15, 0.0), (-0.1, 0.1),
            (0.1, 1.0), (-0.1, 0.0), (0.0, 0.1),
            (0.001, 0.1), (0.0, 0.05),
            (0.1, 1.0), (-0.15, 0.0),
        ]

        init_err = self._objective(initial, data)
        result = scipy_minimize(
            lambda p: self._objective(p, data), initial,
            method="L-BFGS-B", bounds=bounds,
            options={"maxiter": 500, "disp": False},
        )
        opt = result.x
        return self._build_results(opt, result.fun, init_err, data)

    def _optimize_known_mass(self, data: List[Tuple[np.ndarray, np.ndarray]],
                             known: Dict[str, float]) -> Dict:
        """Fix known masses, only optimize CoG positions."""
        m_L2 = known.get("L2", 0.877)
        m_L3 = known.get("L3", 0.251)
        m_L4 = known.get("L4", 0.556)
        m_L5 = known.get("L5", 0.018)
        m_L6 = known.get("L6", 0.668)

        initial_free = np.array([
            0.095,
            -0.056, 0.049,
            -0.024, 0.031,
            0.018,
            -0.070,
        ])
        free_bounds = [
            (-0.2, 0.2),
            (-0.15, 0.0), (-0.1, 0.1),
            (-0.1, 0.0), (0.0, 0.1),
            (0.0, 0.05),
            (-0.15, 0.0),
        ]

        def _expand(free: np.ndarray) -> np.ndarray:
            return np.array([
                m_L2, free[0],
                m_L3, free[1], free[2],
                m_L4, free[3], free[4],
                m_L5, free[5],
                m_L6, free[6],
            ])

        init_err = self._objective(_expand(initial_free), data)
        result = scipy_minimize(
            lambda f: self._objective(_expand(f), data), initial_free,
            method="L-BFGS-B", bounds=free_bounds,
            options={"maxiter": 500, "disp": False},
        )
        opt = _expand(result.x)
        return self._build_results(opt, result.fun, init_err, data)

    def _build_results(self, opt: np.ndarray, final_err: float,
                       init_err: float,
                       data: List[Tuple[np.ndarray, np.ndarray]]) -> Dict:
        n_samples = len(data) * 5
        rmse = np.sqrt(final_err / n_samples) if n_samples else 0.0

        all_measured: List[float] = []
        for _, efforts in data:
            all_measured.extend(efforts[1:6])
        mean_m = np.mean(all_measured) if all_measured else 0.0
        ss_tot = float(sum((m - mean_m) ** 2 for m in all_measured))
        r_squared = 1 - final_err / ss_tot if ss_tot > 0 else 0.0

        return {
            "L2": {"mass": float(opt[0]),  "com": [float(opt[1]), 0.0, 0.0]},
            "L3": {"mass": float(opt[2]),  "com": [float(opt[3]), float(opt[4]), 0.003]},
            "L4": {"mass": float(opt[5]),  "com": [float(opt[6]), float(opt[7]), 0.0]},
            "L5": {"mass": float(opt[8]),  "com": [0.004, 0.0, float(opt[9])]},
            "L6": {"mass": float(opt[10]), "com": [0.0, -0.001, float(opt[11])]},
            "calibration_info": {
                "date": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "num_samples": len(data),
                "rmse": float(rmse),
                "r_squared": float(r_squared),
            },
        }

    # ------------------------------------------------------------------
    # Save
    # ------------------------------------------------------------------

    @staticmethod
    def save_yaml(results: Dict, path: Optional[str] = None):
        path = path or str(INERTIA_YAML_PATH)
        info = results["calibration_info"]
        lines = [
            "# EL-A3 arm inertia parameters",
            "# For Pinocchio gravity compensation",
            "# Fitted by debugger calibration panel",
            "#",
            f"# Calibration date: {info['date']}",
            f"# RMSE: {info['rmse']:.4f} Nm",
            f"# R^2: {info['r_squared']:.4f}",
            "",
            "use_calibrated_params: true",
            "",
            "inertia_params:",
        ]
        for lnk in ("L2", "L3", "L4", "L5", "L6"):
            p = results[lnk]
            lines.append(f"  {lnk}:")
            lines.append(f"    mass: {p['mass']:.4f}")
            lines.append(f"    com: {p['com']}")
            lines.append("")
        lines += [
            "calibration_info:",
            f'  date: "{info["date"]}"',
            f"  num_samples: {info['num_samples']}",
            f"  rmse: {info['rmse']:.4f}",
            f"  r_squared: {info['r_squared']:.4f}",
        ]
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")

    # ------------------------------------------------------------------
    # Main run (with resume + per-point JSONL save + collision pre-check)
    # ------------------------------------------------------------------

    def run(self):
        self._stop_flag = False
        cfg = self._config

        if not HAS_PINOCCHIO:
            self.error_occurred.emit("Pinocchio 未安装，无法执行标定")
            return
        if not HAS_SCIPY:
            self.error_occurred.emit("scipy 未安装，无法执行标定")
            return

        if not self._init_pinocchio():
            self.error_occurred.emit("Pinocchio 模型加载失败")
            return

        self._init_collision_model()

        test_configs = self._generate_test_configs(cfg.num_points)
        total = len(test_configs)
        self.log_message.emit(f"共 {total} 个测试姿态")

        # ---- resume logic -----------------------------------------------
        df = CAL_DATA_FILE
        completed = 0
        prior_data: List[Tuple[np.ndarray, np.ndarray]] = []

        if cfg.resume and df.exists():
            try:
                meta, records = _read_jsonl(df)
                if meta is None:
                    self.log_message.emit("数据文件损坏 (无 meta)，从头开始")
                elif meta.get("mode") != cfg.mode:
                    self.log_message.emit(
                        f"模式不匹配: 文件={meta.get('mode')}, "
                        f"当前={cfg.mode}。从头开始")
                else:
                    records = _dedup_records(records)
                    completed = len(records)
                    for r in records:
                        if not r.get("skipped"):
                            prior_data.append(
                                (np.array(r["position"]),
                                 np.array(r["effort"])))
                    self.log_message.emit(
                        f"断点续传: 已有 {completed}/{total} 个点 "
                        f"({len(prior_data)} 有效)")
            except Exception as exc:
                self.log_message.emit(f"读取数据文件失败: {exc}，从头开始")
                completed = 0
                prior_data.clear()

        if completed == 0:
            _write_meta_line(df, cfg.mode, total, HOME_POSITION)

        if completed >= total:
            self.log_message.emit("所有测试点已采集，直接进入优化")
        else:
            # ---- data collection ----------------------------------------
            self.log_message.emit("移动到 Home 位...")
            self.progress_updated.emit(completed, total, "move_home")
            self._move_and_wait(HOME_POSITION, cfg.motion_duration)

            if self._stop_flag:
                self.log_message.emit("标定已暂停，数据已保存")
                return

            remaining = total - completed
            self.log_message.emit(
                f"开始采集第 {completed}..{total-1} 点 (剩余 {remaining})")

            for idx in range(completed, total):
                if self._stop_flag:
                    self.log_message.emit("标定已暂停，数据已保存")
                    return

                target = test_configs[idx]
                self.progress_updated.emit(idx + 1, total, f"pose_{idx+1}")
                self.log_message.emit(
                    f"测试点 {idx+1}/{total}: "
                    f"{[f'{v:.2f}' for v in target]}")

                if self._check_self_collision(target):
                    self.log_message.emit(
                        f"  [跳过] 自碰撞检测: 该构型存在干涉")
                    _append_data_line(df, {
                        "idx": idx, "skipped": True,
                        "reason": "self_collision",
                        "config": target,
                        "timestamp": datetime.now().isoformat(),
                    })
                    continue

                self._move_and_wait(target, cfg.motion_duration)

                if self._stop_flag:
                    self.log_message.emit("标定已暂停，数据已保存")
                    return

                time.sleep(cfg.settle_time)

                efforts_samples: List[List[float]] = []
                positions_samples: List[List[float]] = []
                interval = 0.02
                for _ in range(cfg.samples_per_point):
                    if self._stop_flag:
                        self.log_message.emit("标定已暂停，数据已保存")
                        return
                    efforts_samples.append(self._read_efforts())
                    positions_samples.append(self._read_positions())
                    time.sleep(interval)

                mean_pos = np.mean(positions_samples, axis=0)
                mean_eff = np.mean(efforts_samples, axis=0)
                prior_data.append((mean_pos, mean_eff))

                _append_data_line(df, {
                    "idx": idx,
                    "config": target,
                    "position": mean_pos.tolist(),
                    "effort": mean_eff.tolist(),
                    "timestamp": datetime.now().isoformat(),
                })

                self.log_message.emit(
                    f"  力矩: [{', '.join(f'{v:.3f}' for v in mean_eff)}]"
                    f"  [已保存]")

        # ---- optimization -----------------------------------------------
        cal_data = [d for d in prior_data if len(d) == 2]
        if len(cal_data) < 10:
            self.error_occurred.emit(
                f"有效数据点不足 ({len(cal_data)}), 需要至少 10 个")
            return

        self.log_message.emit(
            f"数据采集完成 ({len(cal_data)} 有效点)，开始优化...")
        self.progress_updated.emit(total, total, "optimizing")

        try:
            if cfg.mode == "known_mass" and cfg.known_masses:
                results = self._optimize_known_mass(cal_data, cfg.known_masses)
            else:
                results = self._optimize_full(cal_data)
        except Exception as e:
            self.error_occurred.emit(f"优化失败: {e}")
            return

        info = results["calibration_info"]
        self.log_message.emit(
            f"优化完成  RMSE={info['rmse']:.4f} Nm  R²={info['r_squared']:.4f}"
        )
        for lnk in ("L2", "L3", "L4", "L5", "L6"):
            p = results[lnk]
            com_cm = [v * 100.0 for v in p["com"]]
            self.log_message.emit(
                f"  {lnk}: mass={p['mass']:.4f}  com(cm)={com_cm}"
            )

        self.log_message.emit("回到 Home 位...")
        self._move_and_wait(HOME_POSITION, cfg.motion_duration)

        self.calibration_finished.emit(results)
