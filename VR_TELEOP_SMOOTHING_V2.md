# VR 遥操作顺滑优化 V2 — 实现文档

## 背景

pico_control 遥操作在工作空间边界（z≈0.16m + 大角度姿态）出现两个问题：

1. **MoveL IK 不收敛刷屏** — 目标位姿接近不可达，IK 残差 ~0.005 无法进入 3e-4 容差
2. **机械臂抖动** — 根因是 MoveL/MoveJ 在 100Hz 下被反复覆盖，轨迹永远跑不完

## 参考系统

| 系统 | 关键机制 |
|------|---------|
| Droid (15Hz) | 增量式关节控制 + 1000Hz 底层 PD，差值小步下发 |
| xr_teleoperate (30Hz) | IK 内平滑代价(0.1) + 4帧滑动平均 + 250Hz 速度限幅 |

## 信号链路

```
Pico VR 追踪 (30Hz, ≈毫米级抖动)
  │
  ▼
┌─────────────────────────────────────────────┐
│ ① 输入端一阶 LPF (fc≈6Hz, α=0.5)             │
│    cur = LPF(raw)                            │
│    滤追踪噪声，防抖动进入信号链                 │
└─────────────────────┬───────────────────────┘
                      │
  hand_delta → normalize → clip → EMA → integrate → target_pose
                      │
                      ▼
┌─────────────────────────────────────────────┐
│ ② IK 平滑代价 (smooth_weight=0.05)            │
│    ik_step 内惩罚 ||q - q_initial||²          │
│    防 IK 在肘上/肘下等多解构型间跳变           │
└─────────────────────┬───────────────────────┘
                      │
  ik_step → _ik_raw → 二阶关节滤波器 (已有, ω=14)
                      │
                    _hold_q
                      │
                      ▼
┌─────────────────────────────────────────────┐
│ ③ ServoJ — 关节空间短周期伺服                 │
│    virtual_q 以 max_joint_vel=3.0rad/s 限速追赶│
│    每帧走 ≤0.03rad，用 MoveJ(10ms) 微步下发    │
│    参考 Droid 增量式 + xr_teleoperate 速度限幅  │
└─────────────────────┬───────────────────────┘
                      │
          ┌─ MoveL 尝试 (保留，工作空间内笛卡尔直线)
          └─ 失败 → ServoJ fallback
                      │
                      ▼
            CAN 控制循环 (200Hz PD)
```

## 改动清单

### 1. 输入端位姿低通滤波

**文件**: `el_a3_sdk/demo/pico_control.py`
**位置**: `_ArmController` + `process_pose`

- 新增 `_hand_filtered` 状态变量
- `process_pose` 中 `cur_hand` 先过一阶 LPF (α=0.5, fc≈6Hz)
- 仅影响增量式速度计算的输入源，不改变后续逻辑

### 2. IK 平滑代价

**文件**: `el_a3_sdk/el_a3_sdk/kinematics.py`
**位置**: `ELA3Kinematics.ik_step`

- 新增 `smooth_weight` 参数 (默认 0.0，向后兼容)
- 存储初始关节角 `q_pin_initial`
- 每次 DLS 迭代在 RHS 侧加 `smooth_weight * (q_pin_initial - q_pin)`
- 参考 xr_teleoperate 的 `0.1 * ||q - q_last||²` 代价

**文件**: `el_a3_sdk/demo/pico_control.py`
- pico 调用 `ik_step` 时传入 `smooth_weight=0.05`

### 3. ServoJ 关节空间伺服

**文件**: `el_a3_sdk/demo/pico_control.py`
**位置**: `_ArmController._send_filtered` + 新增 `_servo_step`

- 新增状态: `_servo_q`, `_servo_max_vel=3.0`, `_servo_period=0.01`, `_servo_max_step=0.03`
- 新增方法 `_servo_step()`: 等比缩放追赶步长，不超过 `_servo_max_step`
- MoveL 成功 → `_servo_q` 清零，退出 ServoJ 模式
- MoveL 失败 → 进入 ServoJ，限速追赶 `_hold_q`
- 去重周期内如处于 ServoJ 模式，继续追赶不中断
- 归零/重同步时清零 `_servo_q`
- 日志改为 `MoveL→ServoJ fallback`

### 4. 日志降噪（上一轮已完成）

**文件**: `el_a3_sdk/el_a3_sdk/interface.py`
- MoveL IK 失败: `logger.error` → `logger.warning`

**文件**: `el_a3_sdk/el_a3_sdk/kinematics.py`
- IK 不收敛 warning: 每 50 次才打印 1 次（`_ik_warn_count` 计数器）

### 5. 去重优化（上一轮已完成）

**文件**: `el_a3_sdk/demo/pico_control.py`
- `_send_filtered` 中比较 `_target_pose` 是否变化
- 未变化时跳过 MoveL/MoveJ 调用（除非在 ServoJ 模式下继续追赶）
- 目标被替换时清空 `_last_sent_pose_tuple`

### 6. MoveL damping 修复（上一轮已完成）

| 文件 | 位置 | 改动 |
|------|------|------|
| `el_a3_sdk/el_a3_sdk/interface.py` | MoveL fallback (L1555) | `damping=5e-3` |
| `el_a3_sdk/el_a3_sdk/interface.py` | EndPoseCtrl (L1357) | `damping=5e-3` |
| `el_a3_sdk/MotorStudio/backend/arm_worker.py` | 模拟 IK fallback (L752) | `damping=5e-3` |

## 三层滤波职责

| 层 | 位置 | 滤什么 | 参数 |
|----|------|--------|------|
| ① 输入端 LPF | process_pose | Pico 追踪噪声 (10-30Hz) | α=0.5, fc≈6Hz |
| ② IK 平滑代价 | ik_step 内部 | 同一位姿的多解跳变 | smooth_weight=0.05 |
| ③ ServoJ 速度限幅 | _send_filtered 输出 | 任意帧间关节跳变 | max_joint_vel=3.0 rad/s |

三层不重叠，各管各的。输入端滤传感器噪声，IK 平滑稳求解器行为，ServoJ 兜安全底线。

## 可调参数

| 参数 | 值 | 调大效果 | 调小效果 |
|------|-----|---------|---------|
| `lpf_alpha` | 0.5 | 延迟大、更平滑 | 更跟手、噪声多 |
| `smooth_weight` | 0.05 | 约束紧、可能稍偏目标 | 允许构型切换 |
| `_servo_max_vel` | 3.0 rad/s | 响应快、可能抖 | 延迟大、更平滑 |

## 改动范围

| 文件 | 本轮 | 总计 |
|------|:---:|:---:|
| `el_a3_sdk/el_a3_sdk/kinematics.py` | IK 平滑代价 | 平滑代价 + warning 限频 + damping 默认值 |
| `el_a3_sdk/el_a3_sdk/interface.py` | — | MoveL damping + ERROR→WARNING |
| `el_a3_sdk/demo/pico_control.py` | 输入端 LPF + ServoJ + 平滑代价调用 | 去重 + fallback + LPF + ServoJ + 平滑代价 |
| `el_a3_sdk/MotorStudio/backend/arm_worker.py` | — | damping |
| `el_a3_sdk/el_a3_sdk/kinematics.py` (ik_step) | `smooth_weight` 新参数 | 向后兼容，默认 0.0 |
