#!/usr/bin/env python3
"""
EL-A3 dynamics parameter calibration with per-point incremental saving.

Based on inertia_calibration.py, this script saves data after EVERY test point
to a JSONL file, so interruptions only lose the current in-progress point.
On restart it automatically resumes from the last saved point, or you can
manually specify a starting index with --start.

Usage:
    # Auto-resume from last saved point
    python3 dynamics_calibration.py

    # Start from point 20 (0-indexed)
    python3 dynamics_calibration.py --start 20

    # Clear existing data and start fresh
    python3 dynamics_calibration.py --restart

    # Only run optimization on existing data
    python3 dynamics_calibration.py --optimize-only

    # Choose precision mode
    python3 dynamics_calibration.py --mode high
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

import os
import sys
import json
import numpy as np
from scipy.optimize import minimize
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional
from pathlib import Path
import time
import argparse
from datetime import datetime

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.abspath(os.path.join(_SCRIPT_DIR, '..'))

try:
    import pinocchio as pin
    PINOCCHIO_AVAILABLE = True
except ImportError:
    PINOCCHIO_AVAILABLE = False
    print("Error: Pinocchio library is required")
    print("  sudo apt install ros-humble-pinocchio")


# ---------------------------------------------------------------------------
# JSONL helpers
# ---------------------------------------------------------------------------

def _results_dir() -> Path:
    d = Path(_SCRIPT_DIR) / 'calibration_results'
    d.mkdir(parents=True, exist_ok=True)
    return d


def _data_file() -> Path:
    return _results_dir() / 'calibration_data.jsonl'


def write_meta_line(path: Path, mode: str, total_points: int, home: list):
    with open(path, 'w') as f:
        meta = {
            'meta': True,
            'mode': mode,
            'total_points': total_points,
            'home': home,
            'start_time': datetime.now().isoformat(),
        }
        f.write(json.dumps(meta) + '\n')


def append_data_line(path: Path, record: dict):
    with open(path, 'a') as f:
        f.write(json.dumps(record) + '\n')
        f.flush()
        os.fsync(f.fileno())


def read_jsonl(path: Path) -> Tuple[Optional[dict], List[dict]]:
    """Return (meta_dict_or_None, [data_records])."""
    meta = None
    records: List[dict] = []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            if obj.get('meta'):
                meta = obj
            else:
                records.append(obj)
    return meta, records


def dedup_records(records: List[dict]) -> List[dict]:
    """Keep the last record for each idx."""
    by_idx: Dict[int, dict] = {}
    for r in records:
        by_idx[r['idx']] = r
    return [by_idx[k] for k in sorted(by_idx.keys())]


# ---------------------------------------------------------------------------
# Calibration config
# ---------------------------------------------------------------------------

@dataclass
class CalibrationConfig:
    urdf_path: str = os.path.join(_PROJECT_ROOT, 'el_a3_description', 'urdf', 'el_a3.urdf')
    output_path: str = os.path.join(_PROJECT_ROOT, 'el_a3_description', 'config', 'inertia_params.yaml')

    joint_names: List[str] = field(default_factory=lambda: [
        'L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint'
    ])

    home_position: List[float] = field(default_factory=lambda: [0.0, 0.785, -0.785, 0.5, 0.5, 0.0])

    samples_per_config: int = 40
    settle_time: float = 1.5
    sample_interval: float = 0.02
    motion_duration: float = 5.0


# ---------------------------------------------------------------------------
# Calibrator node
# ---------------------------------------------------------------------------

class DynamicsCalibrator(Node):

    def __init__(self, config: CalibrationConfig):
        super().__init__('dynamics_calibrator')

        self.config = config
        self.joint_names = config.joint_names

        self.current_positions = [0.0] * 6
        self.current_efforts = [0.0] * 6
        self.state_received = False

        self.model = None
        self.data = None

        self.calibration_data: List[Tuple[np.ndarray, np.ndarray]] = []

        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)

        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

        if PINOCCHIO_AVAILABLE:
            self._init_pinocchio()

    # -- Pinocchio ---------------------------------------------------------

    def _init_pinocchio(self):
        try:
            self.model = pin.buildModelFromUrdf(self.config.urdf_path)
            self.data = pin.Data(self.model)
            self.get_logger().info(f'Pinocchio model loaded: {self.model.njoints} joints')
        except Exception as e:
            self.get_logger().error(f'Failed to load Pinocchio: {e}')

    # -- ROS2 callbacks ----------------------------------------------------

    def _joint_state_cb(self, msg: JointState):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
                if idx < len(msg.effort):
                    self.current_efforts[i] = msg.effort[idx]
        self.state_received = True

    # -- Motion ------------------------------------------------------------

    def move_to_position(self, target: List[float], duration: float = None) -> bool:
        if duration is None:
            duration = self.config.motion_duration

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            return False

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = list(target)
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(
            sec=int(duration), nanosec=int((duration % 1) * 1e9))
        goal_msg.trajectory.points = [point]

        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 10)
        return True

    def collect_samples(self) -> Tuple[np.ndarray, np.ndarray]:
        positions = []
        efforts = []
        for _ in range(self.config.samples_per_config):
            rclpy.spin_once(self, timeout_sec=self.config.sample_interval)
            positions.append(self.current_positions.copy())
            efforts.append(self.current_efforts.copy())
            time.sleep(self.config.sample_interval)
        return np.mean(positions, axis=0), np.mean(efforts, axis=0)

    # -- Collision check ---------------------------------------------------

    @staticmethod
    def check_collision_with_base(config: List[float]) -> bool:
        l2, l3 = config[1], config[2]
        if l2 >= 1.2 and l3 > -0.4:
            return True
        if l2 > 1.35 and l3 > -0.6:
            return True
        if l2 >= 1.2 and l3 >= -0.55 and (l2 + l3 * 0.5) > 0.9:
            return True
        return False

    # -- Configuration generation ------------------------------------------

    def generate_test_configurations(self, mode: str = 'full') -> List[List[float]]:
        """Deterministic config list — same mode always yields same order."""
        configs = []
        home = np.array(self.config.home_position)

        if mode == 'quick':
            l2_range = np.linspace(0.4, 1.4, 6)
            l3_range = np.linspace(-1.2, -0.4, 5)
            l4_range = np.array([0.3, 0.7, 1.0])
            l5_range = np.array([0.3, 0.6])
            grid_step = 2
        elif mode == 'high':
            l2_range = np.linspace(0.25, 1.55, 12)
            l3_range = np.linspace(-1.35, -0.25, 12)
            l4_range = np.linspace(0.25, 1.25, 7)
            l5_range = np.linspace(0.25, 1.05, 5)
            grid_step = 2
        elif mode == 'ultra':
            l2_range = np.linspace(0.2, 1.6, 15)
            l3_range = np.linspace(-1.4, -0.2, 15)
            l4_range = np.linspace(0.2, 1.3, 8)
            l5_range = np.linspace(0.2, 1.1, 6)
            grid_step = 2
        else:
            l2_range = np.linspace(0.3, 1.5, 10)
            l3_range = np.linspace(-1.3, -0.3, 10)
            l4_range = np.linspace(0.3, 1.2, 6)
            l5_range = np.linspace(0.3, 1.0, 5)
            grid_step = 2

        for l2 in l2_range:
            cfg = home.copy(); cfg[1] = l2
            configs.append(cfg.tolist())

        for l3 in l3_range:
            cfg = home.copy(); cfg[2] = l3
            configs.append(cfg.tolist())

        l2_grid = l2_range[::grid_step]
        l3_grid = l3_range[::grid_step]
        for l2 in l2_grid:
            for l3 in l3_grid:
                cfg = home.copy(); cfg[1] = l2; cfg[2] = l3
                configs.append(cfg.tolist())

        for l4 in l4_range:
            for l5 in l5_range:
                cfg = home.copy(); cfg[3] = l4; cfg[4] = l5
                configs.append(cfg.tolist())

        unique = []
        for cfg in configs:
            if not any(np.allclose(cfg, u, atol=0.05) for u in unique):
                unique.append(cfg)

        safe = [c for c in unique if not self.check_collision_with_base(c)]
        skipped = len(unique) - len(safe)
        if skipped:
            self.get_logger().info(f'Filtered {skipped} potentially colliding configs')
        self.get_logger().info(f'Total test points ({mode}): {len(safe)}')
        return safe

    # -- Pinocchio dynamics ------------------------------------------------

    def compute_gravity_with_params(self, q: np.ndarray, params: np.ndarray) -> np.ndarray:
        if self.model is None:
            return np.zeros(6)

        original_inertias = []
        for i in range(1, min(7, self.model.nbodies)):
            original_inertias.append({
                'mass': self.model.inertias[i].mass,
                'lever': self.model.inertias[i].lever.copy()
            })

        try:
            if len(params) > 0:  self.model.inertias[2].mass      = params[0]
            if len(params) > 1:  self.model.inertias[2].lever[0]  = params[1]
            if len(params) > 2:  self.model.inertias[3].mass      = params[2]
            if len(params) > 3:  self.model.inertias[3].lever[0]  = params[3]
            if len(params) > 4:  self.model.inertias[3].lever[1]  = params[4]
            if len(params) > 5:  self.model.inertias[4].mass      = params[5]
            if len(params) > 6:  self.model.inertias[4].lever[0]  = params[6]
            if len(params) > 7:  self.model.inertias[4].lever[1]  = params[7]
            if len(params) > 8:  self.model.inertias[5].mass      = params[8]
            if len(params) > 9:  self.model.inertias[5].lever[2]  = params[9]
            if len(params) > 10: self.model.inertias[6].mass      = params[10]
            if len(params) > 11: self.model.inertias[6].lever[2]  = params[11]

            self.data = pin.Data(self.model)
            q_arr = np.array(q, dtype=float)
            if len(q_arr) < self.model.nq:
                q_arr = np.concatenate([q_arr, np.zeros(self.model.nq - len(q_arr))])
            q_pin = q_arr[:self.model.nq]
            tau = pin.rnea(self.model, self.data, q_pin,
                           np.zeros(self.model.nv), np.zeros(self.model.nv))
        finally:
            for i, orig in enumerate(original_inertias):
                self.model.inertias[i + 1].mass = orig['mass']
                self.model.inertias[i + 1].lever = orig['lever']
            self.data = pin.Data(self.model)

        return tau[:6]

    def objective_function(self, params: np.ndarray) -> float:
        total_error = 0.0
        for positions, measured in self.calibration_data:
            predicted = self.compute_gravity_with_params(positions, params)
            for i in range(1, 6):
                total_error += (measured[i] - predicted[i]) ** 2
        return total_error

    # -- Optimization ------------------------------------------------------

    FIXED_MASSES = {
        'L2': 0.877,
        'L3': 0.251,
        'L4': 0.556,
        'L6': 0.668,
    }

    def optimize(self, fix_masses: bool = False) -> Dict:
        """Run L-BFGS-B on self.calibration_data and return results dict.

        Args:
            fix_masses: If True, lock L2/L3/L4/L6 masses to measured values
                        and only optimize COM positions (+ L5 mass).
        """
        mode_label = 'fixed-mass COM-only' if fix_masses else 'full 12-param'
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Optimizing inertia parameters ({mode_label})')
        self.get_logger().info('=' * 60)

        if fix_masses:
            fm = self.FIXED_MASSES
            self.get_logger().info(
                f'  Fixed masses: L2={fm["L2"]}, L3={fm["L3"]}, '
                f'L4={fm["L4"]}, L6={fm["L6"]}')

            # 8 free params: L2_com_x, L3_com_x, L3_com_y,
            #   L4_com_x, L4_com_y, L5_mass, L5_com_z, L6_com_z
            initial_free = np.array([
                0.095,
                -0.056, 0.049,
                -0.024, 0.031,
                0.0180, 0.018,
                -0.070,
            ])
            free_bounds = [
                (-0.2, 0.2),
                (-0.15, 0.0), (-0.1, 0.1),
                (-0.1, 0.0), (0.0, 0.1),
                (0.001, 0.1), (0.0, 0.05),
                (-0.15, 0.0),
            ]

            def _expand(free: np.ndarray) -> np.ndarray:
                """Expand 8 free params to full 12-param vector."""
                return np.array([
                    fm['L2'], free[0],
                    fm['L3'], free[1], free[2],
                    fm['L4'], free[3], free[4],
                    free[5], free[6],
                    fm['L6'], free[7],
                ])

            def _obj_fixed(free: np.ndarray) -> float:
                return self.objective_function(_expand(free))

            self.get_logger().info(f'Data points: {len(self.calibration_data)}')
            initial_error = _obj_fixed(initial_free)
            self.get_logger().info(f'Initial error: {initial_error:.6f}')

            result = minimize(
                _obj_fixed, initial_free,
                method='L-BFGS-B', bounds=free_bounds,
                options={'maxiter': 500, 'disp': False})

            opt = _expand(result.x)
        else:
            initial_params = np.array([
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

            self.get_logger().info(f'Data points: {len(self.calibration_data)}')
            initial_error = self.objective_function(initial_params)
            self.get_logger().info(f'Initial error: {initial_error:.6f}')

            result = minimize(
                self.objective_function, initial_params,
                method='L-BFGS-B', bounds=bounds,
                options={'maxiter': 500, 'disp': False})

            opt = result.x

        final_error = result.fun
        reduction = (1 - final_error / initial_error) * 100 if initial_error else 0
        self.get_logger().info(
            f'Optimization done! Final error: {final_error:.6f} (reduced {reduction:.1f}%)')

        n_samples = len(self.calibration_data) * 5
        rmse = np.sqrt(final_error / n_samples) if n_samples else 0

        all_measured = []
        for _, efforts in self.calibration_data:
            all_measured.extend(efforts[1:6])
        mean_m = np.mean(all_measured) if all_measured else 0
        ss_tot = sum((m - mean_m) ** 2 for m in all_measured)
        r_squared = 1 - final_error / ss_tot if ss_tot > 0 else 0

        self.get_logger().info(f'RMSE: {rmse:.4f} Nm')
        self.get_logger().info(f'R^2:  {r_squared:.4f}')

        results = {
            'L2': {'mass': float(opt[0]),  'com': [float(opt[1]), 0.0, 0.0]},
            'L3': {'mass': float(opt[2]),  'com': [float(opt[3]), float(opt[4]), 0.003]},
            'L4': {'mass': float(opt[5]),  'com': [float(opt[6]), float(opt[7]), 0.0]},
            'L5': {'mass': float(opt[8]),  'com': [0.004, 0.0, float(opt[9])]},
            'L6': {'mass': float(opt[10]), 'com': [0.0, -0.001, float(opt[11])]},
            'calibration_info': {
                'date': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'num_samples': len(self.calibration_data),
                'rmse': float(rmse),
                'r_squared': float(r_squared),
            }
        }

        self.get_logger().info('Optimized parameters:')
        for j in ['L2', 'L3', 'L4', 'L5', 'L6']:
            p = results[j]
            tag = ' [FIXED]' if fix_masses and j in self.FIXED_MASSES else ''
            self.get_logger().info(
                f'  {j}: mass={p["mass"]:.4f}{tag}, com={p["com"]}')

        return results

    # -- Save results ------------------------------------------------------

    def save_results(self, results: Dict, output_path: str):
        yaml_content = f"""# EL-A3 arm inertia parameters
# For Pinocchio gravity compensation
# Fitted by dynamics_calibration.py — override URDF defaults
#
# Calibration date: {results['calibration_info']['date']}
# Calibration script: scripts/dynamics_calibration.py
# RMSE: {results['calibration_info']['rmse']:.4f} Nm
# R^2: {results['calibration_info']['r_squared']:.4f}

use_calibrated_params: true

inertia_params:
  L2:
    mass: {results['L2']['mass']:.4f}
    com: {results['L2']['com']}

  L3:
    mass: {results['L3']['mass']:.4f}
    com: {results['L3']['com']}

  L4:
    mass: {results['L4']['mass']:.4f}
    com: {results['L4']['com']}

  L5:
    mass: {results['L5']['mass']:.4f}
    com: {results['L5']['com']}

  L6:
    mass: {results['L6']['mass']:.4f}
    com: {results['L6']['com']}

calibration_info:
  date: "{results['calibration_info']['date']}"
  num_samples: {results['calibration_info']['num_samples']}
  rmse: {results['calibration_info']['rmse']:.4f}
  r_squared: {results['calibration_info']['r_squared']:.4f}
"""
        with open(output_path, 'w') as f:
            f.write(yaml_content)
        self.get_logger().info(f'Results saved to: {output_path}')

        results_copy = _results_dir() / 'inertia_params.yaml'
        with open(results_copy, 'w') as f:
            f.write(yaml_content)
        self.get_logger().info(f'Copy saved to:    {results_copy}')

    # -- Wait for joint states ---------------------------------------------

    def wait_for_joint_states(self, timeout: float = 10.0) -> bool:
        self.get_logger().info('Waiting for joint states...')
        t0 = time.time()
        while not self.state_received and (time.time() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.state_received:
            self.get_logger().error('Failed to receive joint states')
        return self.state_received

    # -- Main collection loop (per-point saving) ---------------------------

    def run_collection(self, mode: str, start_idx: Optional[int], restart: bool):
        if self.model is None:
            self.get_logger().error('Pinocchio model not loaded')
            return False

        if not self.wait_for_joint_states():
            return False

        test_configs = self.generate_test_configurations(mode)
        total = len(test_configs)
        df = _data_file()

        if restart and df.exists():
            self.get_logger().info('--restart: removing existing data file')
            df.unlink()

        # Determine starting point
        if start_idx is not None:
            completed = start_idx
            if not df.exists():
                write_meta_line(df, mode, total, self.config.home_position)
            else:
                meta, _ = read_jsonl(df)
                if meta and meta.get('mode') != mode:
                    self.get_logger().error(
                        f'Mode mismatch: file has "{meta.get("mode")}", '
                        f'requested "{mode}". Use --restart to clear.')
                    return False
            self.get_logger().info(f'--start {start_idx}: resuming from point {start_idx}')
        elif df.exists():
            meta, records = read_jsonl(df)
            if meta is None:
                self.get_logger().error('Data file corrupt (no meta line). Use --restart.')
                return False
            if meta.get('mode') != mode:
                self.get_logger().error(
                    f'Mode mismatch: file has "{meta.get("mode")}", '
                    f'requested "{mode}". Use --restart to clear.')
                return False
            if meta.get('total_points') != total:
                self.get_logger().warn(
                    f'total_points mismatch: file={meta.get("total_points")}, '
                    f'generated={total}. Continuing with generated list.')
            completed = len(dedup_records(records))
            self.get_logger().info(
                f'Resuming: {completed}/{total} points already collected')
        else:
            completed = 0
            write_meta_line(df, mode, total, self.config.home_position)
            self.get_logger().info(f'Starting fresh collection: {total} points')

        if completed >= total:
            self.get_logger().info('All points already collected, skipping to optimization')
            return True

        self.get_logger().info(
            f'Collecting points {completed}..{total-1} '
            f'({total - completed} remaining)')

        for i in range(completed, total):
            cfg = test_configs[i]
            self.get_logger().info(
                f'[{i+1}/{total}] Target: '
                f'L2={cfg[1]:.2f} L3={cfg[2]:.2f} L4={cfg[3]:.2f} L5={cfg[4]:.2f}')

            if not self.move_to_position(cfg):
                self.get_logger().warn('  Move failed, skipping')
                continue

            time.sleep(self.config.settle_time)
            pos, eff = self.collect_samples()

            append_data_line(df, {
                'idx': i,
                'config': cfg,
                'position': pos.tolist(),
                'effort': eff.tolist(),
                'timestamp': datetime.now().isoformat(),
            })

            self.get_logger().info(
                f'  Pos: L2={pos[1]:.3f} L3={pos[2]:.3f} | '
                f'Torque: L2={eff[1]:.4f} L3={eff[2]:.4f} L4={eff[3]:.4f}  '
                f'[saved]')

        self.get_logger().info('Returning to home position...')
        self.move_to_position(self.config.home_position)
        return True

    # -- Load saved data for optimization ----------------------------------

    def load_saved_data(self) -> bool:
        df = _data_file()
        if not df.exists():
            self.get_logger().error(f'No data file found at {df}')
            return False

        _, records = read_jsonl(df)
        records = dedup_records(records)
        if len(records) < 10:
            self.get_logger().error(
                f'Too few data points ({len(records)}), need at least 10')
            return False

        self.calibration_data.clear()
        for r in records:
            self.calibration_data.append(
                (np.array(r['position']), np.array(r['effort'])))

        self.get_logger().info(f'Loaded {len(records)} data points from {df}')
        return True


def main():
    parser = argparse.ArgumentParser(
        description='EL-A3 dynamics calibration (per-point saving)')
    parser.add_argument('--mode', type=str, default='full',
                        choices=['quick', 'full', 'high', 'ultra'],
                        help='Precision mode (default: full)')
    parser.add_argument('--start', type=int, default=None,
                        help='Start from this point index (0-indexed)')
    parser.add_argument('--restart', action='store_true',
                        help='Clear existing data and start fresh')
    parser.add_argument('--optimize-only', action='store_true',
                        help='Skip collection, optimize existing data')
    parser.add_argument('--fix-masses', action='store_true',
                        help='Fix L2/L3/L4/L6 masses to measured values, only optimize COM')
    parser.add_argument('--samples', type=int, default=40,
                        help='Samples per point (default: 40)')
    parser.add_argument('--output', type=str,
                        default=os.path.join(
                            _PROJECT_ROOT, 'el_a3_description', 'config',
                            'inertia_params.yaml'),
                        help='Output YAML path')
    parser.add_argument('--urdf', type=str,
                        default=os.path.join(
                            _PROJECT_ROOT, 'el_a3_description', 'urdf',
                            'el_a3.urdf'),
                        help='URDF file path')
    args = parser.parse_args()

    if not PINOCCHIO_AVAILABLE:
        sys.exit(1)

    rclpy.init()

    config = CalibrationConfig(
        urdf_path=args.urdf,
        output_path=args.output,
        samples_per_config=args.samples,
    )

    calibrator = DynamicsCalibrator(config)

    try:
        if not args.optimize_only:
            ok = calibrator.run_collection(
                mode=args.mode, start_idx=args.start, restart=args.restart)
            if not ok:
                print('\nCollection failed or aborted')
                return

        if not calibrator.load_saved_data():
            return

        results = calibrator.optimize(fix_masses=args.fix_masses)
        if results:
            calibrator.save_results(results, args.output)
            print('\n' + '=' * 60)
            print('  Calibration completed!')
            print('  Restart the controller to apply new parameters')
            print('=' * 60)
        else:
            print('\nOptimization failed')

    except KeyboardInterrupt:
        df = _data_file()
        print(f'\nInterrupted. Data saved so far in {df}')
        print('Re-run to resume from last saved point.')

    finally:
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
