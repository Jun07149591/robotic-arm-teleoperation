# Hardware — 硬件制造资料

> EL-A3 机械臂硬件设计、制造与装配相关文件。

## 目录说明

| 目录 | 内容 |
|------|------|
| [`step/`](step/) | 机械臂 STEP 3D 模型文件（`.step` / `.stp`） |
| [`3mf/`](3mf/) | 3MF 3D 打印制造文件（`.3mf`） |
| [`wiring/`](wiring/) | 线束图纸（接线定义、连接器 pinout 等） |
| [`pcb/`](pcb/) | PCB 相关资料（原理图、Gerber、BOM 等） |
| [`assembly_sop/`](assembly_sop/) | 组装 SOP 标准作业程序 |

## 注意事项

- 本目录下的大文件（STEP、3MF、STL、Gerber、PDF 等）通过 **Git LFS** 管理，请确保已安装并初始化 Git LFS：

  ```bash
  git lfs install
  ```

- 提交前请确认 `git lfs status` 中相关文件已被 LFS 追踪。
