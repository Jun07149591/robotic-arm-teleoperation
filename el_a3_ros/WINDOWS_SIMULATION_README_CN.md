# Windows 本机运行 EL-A3 ROS/RViz 仿真指南

本文说明如何在 Windows 上运行本项目的 EL-A3 机械臂 ROS 2/RViz 仿真。当前推荐使用 `-DirectRviz` 模式：它不需要 Visual Studio、不需要 `colcon build`，会直接发布 `/robot_description`、`/joint_states` 和 `/tf`，适合在 Windows 本机快速演示和调试机械臂运动。

## 当前机器路径

项目路径：

```powershell
D:\codex\EA3\EDULITE_A3
```

ROS 2 Humble/RoboStack 环境：

```powershell
D:\codex\EA3\.micromamba\envs\el-a3-ros
```

如果你把项目移动到了其他目录，请把下面命令里的路径替换成自己的实际路径。

## 快速启动

推荐先启动末端位姿 IK 控制模式：

```powershell
cd D:\codex\EA3\EDULITE_A3\el_a3_ros
powershell -ExecutionPolicy Bypass -File scripts\run_simulation.ps1 -DirectRviz -IKControl -CondaPrefix D:\codex\EA3\.micromamba\envs\el-a3-ros
```

启动后会出现两个窗口：

- RViz：显示机械臂模型和坐标系。
- EL-A3 末端位姿 IK 控制：通过末端三维坐标和四元数控制机械臂。

## 运行模式

### 1. 自动循环演示

机械臂按脚本预设轨迹自动运动：

```powershell
cd D:\codex\EA3\EDULITE_A3\el_a3_ros
powershell -ExecutionPolicy Bypass -File scripts\run_simulation.ps1 -DirectRviz -CondaPrefix D:\codex\EA3\.micromamba\envs\el-a3-ros -SpeedScale 1.0
```

调快速度：

```powershell
powershell -ExecutionPolicy Bypass -File scripts\run_simulation.ps1 -DirectRviz -CondaPrefix D:\codex\EA3\.micromamba\envs\el-a3-ros -SpeedScale 2.0
```

只运行一遍后退出：

```powershell
powershell -ExecutionPolicy Bypass -File scripts\run_simulation.ps1 -DirectRviz -CondaPrefix D:\codex\EA3\.micromamba\envs\el-a3-ros -Once -SpeedScale 10.0
```

### 2. 关节滑块控制

打开关节控制 GUI，通过 `L1_joint` 到 `L7_joint` 的滑块直接控制每个关节角：

```powershell
cd D:\codex\EA3\EDULITE_A3\el_a3_ros
powershell -ExecutionPolicy Bypass -File scripts\run_simulation.ps1 -DirectRviz -ManualControl -CondaPrefix D:\codex\EA3\.micromamba\envs\el-a3-ros
```

说明：

- 滑块单位是弧度。
- `L1` 到 `L6` 是机械臂主体关节。
- `L7` 是末端/夹爪附加关节。
- 点击 `回零位` 可恢复初始姿态。

### 3. 末端位姿 IK 控制

打开 IK GUI，通过末端位置和姿态控制机械臂：

```powershell
cd D:\codex\EA3\EDULITE_A3\el_a3_ros
powershell -ExecutionPolicy Bypass -File scripts\run_simulation.ps1 -DirectRviz -IKControl -CondaPrefix D:\codex\EA3\.micromamba\envs\el-a3-ros
```

IK 控制窗口参数：

- `x/y/z`：末端 `end_effector` 的目标位置，单位是米。
- `qx/qy/qz/qw`：末端姿态四元数。
- `L7`：末端/夹爪附加关节，单独控制。
- `IK OK`：逆运动学解算成功。
- `pos_err`：位置误差，单位是米。
- `rot_err`：姿态误差，单位是弧度。

使用建议：

- 先点击 `读取当前末端`，让目标值同步到当前机械臂姿态。
- 每次小幅拖动 `x/y/z`，更容易稳定收敛。
- 如果显示 `IK 未完全收敛`，通常是目标超出机械臂可达范围，或者四元数对应姿态太极端。
- 四元数会在程序内部归一化；如果姿态乱了，可以点击 `回零位` 重新开始。

### 4. 只启动 ROS 发布节点，不打开 RViz

用于测试话题发布：

```powershell
cd D:\codex\EA3\EDULITE_A3\el_a3_ros
powershell -ExecutionPolicy Bypass -File scripts\run_simulation.ps1 -DirectRviz -NoRviz -CondaPrefix D:\codex\EA3\.micromamba\envs\el-a3-ros
```

## RViz 基本操作

- 鼠标滚轮：缩放。
- 鼠标左键拖动：旋转视角。
- 鼠标中键拖动：平移视角。
- 左侧 `Displays` 面板里：
  - `RobotModel` 是机械臂模型。
  - `TF` 是坐标系显示，觉得太乱可以关闭或取消 `Show Names`。
  - `Grid` 是地面网格。
- `Global Options -> Fixed Frame` 应为 `base_link`。
- `RobotModel -> Description Topic` 应为 `/robot_description`。

## 停止仿真

如果从 PowerShell 前台启动，按：

```powershell
Ctrl+C
```

如果打开了 GUI，也可以直接关闭控制窗口。

如果后台进程残留，可以强制停止本仿真相关进程：

```powershell
Get-Process rviz2,python -ErrorAction SilentlyContinue | Where-Object {
  $_.Path -like 'D:\codex\EA3\.micromamba\envs\el-a3-ros\*' -or $_.ProcessName -eq 'rviz2'
} | Stop-Process -Force
```

## 关键文件

- Windows 启动脚本：`D:\codex\EA3\EDULITE_A3\el_a3_ros\scripts\run_simulation.ps1`
- Windows 无编译仿真节点：`D:\codex\EA3\EDULITE_A3\el_a3_ros\scripts\windows_rviz_sim.py`
- RViz 配置：`D:\codex\EA3\EDULITE_A3\el_a3_ros\el_a3_description\config\el_a3_view.rviz`
- URDF 模型：`D:\codex\EA3\EDULITE_A3\el_a3_ros\el_a3_description\urdf\el_a3.urdf`
- STL 网格：`D:\codex\EA3\EDULITE_A3\el_a3_ros\el_a3_description\meshes`
- 日志文件：`D:\codex\EA3\EDULITE_A3\el_a3_ros\windows_sim_stdout.log` 和 `windows_sim_stderr.log`

## 常见问题

### 1. 提示找不到 ROS setup 或 ros2

确认命令里带了：

```powershell
-CondaPrefix D:\codex\EA3\.micromamba\envs\el-a3-ros
```

这台机器当前使用的是 RoboStack/conda 里的 ROS 2，不依赖系统级 ROS 安装。

### 2. RViz 打开了但模型不动

优先重启仿真：

```powershell
Get-Process rviz2,python -ErrorAction SilentlyContinue | Where-Object {
  $_.Path -like 'D:\codex\EA3\.micromamba\envs\el-a3-ros\*' -or $_.ProcessName -eq 'rviz2'
} | Stop-Process -Force

cd D:\codex\EA3\EDULITE_A3\el_a3_ros
powershell -ExecutionPolicy Bypass -File scripts\run_simulation.ps1 -DirectRviz -IKControl -CondaPrefix D:\codex\EA3\.micromamba\envs\el-a3-ros
```

然后检查 RViz 左侧：

- `Fixed Frame` 是否为 `base_link`
- `RobotModel` 是否启用
- `Description Topic` 是否为 `/robot_description`

### 3. IK 显示未完全收敛

这通常不是程序崩溃，而是目标位姿不可达或姿态变化过大。处理方式：

- 点击 `读取当前末端`。
- 小幅移动 `x/y/z`。
- 暂时少动四元数，只先控制位置。
- 点击 `回零位` 后重新尝试。

### 4. PowerShell 执行策略阻止脚本

使用本文命令中的：

```powershell
-ExecutionPolicy Bypass
```

它只对当前这次 PowerShell 调用生效。

### 5. 为什么不用 colcon build 或 ros2_control

完整 ros2_control 仿真在 Windows 上通常需要 Visual Studio C++ 构建环境。当前这台机器没有管理员权限，因此推荐使用 `-DirectRviz` 模式。该模式直接读取 URDF、发布 TF 和关节状态，适合本机演示、教学和算法调试。
