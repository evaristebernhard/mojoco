---
description: Add Visualization with HUD, Real-Time Plots, and Parameter Panel
---

## 概述

本工作流为倒立摆仿真添加完整的可视化方案，包含三个核心组件：

1. **MuJoCo 3D Viewer + HUD** — 3D 仿真画面 + 状态文字叠加
2. **实时曲线图** — 位置/速度、摆角/角速度、控制力 三组曲线
3. **参数调节面板** — Q/R 权重、最大力、初始角度 等实时可调

## 前置条件

- VcXsrv 已安装并启动（Multiple Windows + Disable access control）
- WSL2 中设置 `export DISPLAY=:0`

---

## 步骤

### 1. 配置 DISPLAY 环境变量

```bash
export DISPLAY=:0
# 验证 X 服务器连通
xeyes  # 应弹出眼睛窗口
```

### 2. 创建可视化节点

**文件路径**: `~/mojoco/ros2_ws/src/mujoco_pendulum/mujoco_pendulum/mj_visualized_node.py`

该节点基于 `mj_lqr_node.py` 扩展，新增：

- **HUD 叠加层**: 在 MuJoCo viewer 上显示实时文字（时间、状态、力、LQR增益）
- **Matplotlib 实时曲线**: 3 个子图，分别显示：
  - `x_cart` 和 `x_dot` 随时间变化
  - `θ` 和 `θ_dot` 随时间变化
  - `control force` 随时间变化
- **Tkinter 参数面板**: 可调节 Q、R、max_force、initial_angle，带 Refresh 按钮

### 3. 注册新的 entry_point

在 `setup.py` 的 `console_scripts` 中添加：

```python
'mj_visualized_node = mujoco_pendulum.mj_visualized_node:main',
```

### 4. 构建

// turbo
```bash
cd ~/mojoco/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select mujoco_pendulum
```

### 5. 运行

```bash
export DISPLAY=:0
source /opt/ros/humble/setup.bash
source ~/mojoco/ros2_ws/install/setup.bash
ros2 run mujoco_pendulum mj_visualized_node
```

---

## 产出物

| 文件 | 说明 |
|------|------|
| `mj_visualized_node.py` | 带完整可视化的仿真节点 |
