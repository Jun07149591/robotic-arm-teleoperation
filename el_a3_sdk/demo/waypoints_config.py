"""
10 路径点循环测试 -- 共享路径点定义

被 waypoint_loop_sim.py 和 waypoint_loop_real.py 共同导入。
所有角度单位为 rad，均在 URDF 关节限位范围内。

关节限位参考:
  L1: [-2.79, 2.79]  (±160°)
  L2: [0, 3.67]      (0°~210°)
  L3: [-4.01, 0]     (-230°~0°)
  L4: [-1.57, 1.57]  (±90°)
  L5: [-1.57, 1.57]  (±90°)
  L6: [-1.57, 1.57]  (±90°)
"""

from dataclasses import dataclass
from typing import List


@dataclass
class Waypoint:
    name: str
    positions: List[float]  # [L1, L2, L3, L4, L5, L6] in rad
    hold_time: float        # seconds to hold at this point


WAYPOINTS: List[Waypoint] = [
    Waypoint("零位",     [0.0,   0.0,   0.0,    0.0,   0.0,   0.0  ], 2.0),
    Waypoint("前伸抬臂", [0.52,  0.79, -0.52,   0.0,   0.0,   0.0  ], 2.0),
    Waypoint("左侧展开", [-0.52, 1.05, -1.05,   0.52,  0.0,   0.0  ], 2.0),
    Waypoint("高位直臂", [0.0,   1.57, -1.57,   0.0,   0.79,  0.0  ], 2.0),
    Waypoint("右前方",   [1.05,  0.52, -0.79,  -0.52,  0.0,   0.52 ], 2.0),
    Waypoint("左前方",   [-1.05, 0.52, -0.79,   0.52,  0.0,  -0.52 ], 2.0),
    Waypoint("折叠",     [0.0,   2.09, -2.09,   0.0,  -0.52,  0.0  ], 2.0),
    Waypoint("右上方",   [0.79,  1.05, -0.52,   0.26,  0.26,  0.26 ], 2.0),
    Waypoint("左上方",   [-0.79, 1.05, -0.52,  -0.26, -0.26, -0.26 ], 2.0),
    Waypoint("回零位",   [0.0,   0.0,   0.0,    0.0,   0.0,   0.0  ], 2.0),
]


def get_waypoint_summary() -> str:
    """返回路径点摘要字符串（用于终端显示）"""
    import math
    lines = []
    lines.append(f"{'序号':>4}  {'名称':<8}  {'L1':>7}  {'L2':>7}  {'L3':>7}  "
                 f"{'L4':>7}  {'L5':>7}  {'L6':>7}  {'停留':>5}")
    lines.append("─" * 78)
    for i, wp in enumerate(WAYPOINTS):
        degs = [f"{p * 180.0 / math.pi:6.1f}°" for p in wp.positions]
        lines.append(
            f"  {i:>2}  {wp.name:<8}  "
            + "  ".join(degs)
            + f"  {wp.hold_time:4.1f}s"
        )
    return "\n".join(lines)
