---
description: Implement ROS 2 Nodes for Simulation and Monitoring
---

## 概述

本工作流将 MuJoCo 仿真和 LQR 控制器封装为 ROS 2 节点，实现：

1. **仿真节点** (`mj_lqr_node`): 加载模型 → 运行仿真 → 施加 LQR 控制 → 发布状态
2. **话题接口**: 通过 `/pendulum_state` 话题发布实时状态
3. **可视化**: MuJoCo 被动查看器 + `ros2 topic echo` 监控

---

## 架构

```
┌────────────────────────────────────────────────┐
│            mj_lqr_node (ROS 2 Node)            │
│                                                │
│  ┌─────────────┐    ┌──────────────────┐       │
│  │  MuJoCo     │    │  LQR Controller  │       │
│  │  Simulation │◄───│  (lqr_controller │       │
│  │  Engine     │    │   .py)           │       │
│  └──────┬──────┘    └──────────────────┘       │
│         │                                      │
│         ▼                                      │
│  ┌──────────────┐                              │
│  │  State Pub   │──── /pendulum_state ────►    │
│  │  (Float64    │     [x, θ, ẋ, θ̇]            │
│  │   MultiArray)│                              │
│  └──────────────┘                              │
│                                                │
│  ┌──────────────┐                              │
│  │  MuJoCo      │  (可选, 需 X Server)          │
│  │  Viewer      │                              │
│  └──────────────┘                              │
└────────────────────────────────────────────────┘
```

### ROS 2 话题

| 话题名 | 类型 | 方向 | 数据 |
|--------|------|------|------|
| `/pendulum_state` | `std_msgs/Float64MultiArray` | 发布 | `[x_cart, θ, ẋ_cart, θ̇]` |

---

## 步骤

### 1. 创建仿真节点 `mj_lqr_node.py`

**文件路径**: `~/mojoco/ros2_ws/src/mujoco_pendulum/mujoco_pendulum/mj_lqr_node.py`

该节点应：

- 使用 `ament_index_python.get_package_share_directory` 定位模型 XML
- 导入 `lqr_controller.py` 中的 `cart_pole_lqr_gain()` 获取增益 K
- 创建定时器，以模型 `timestep` 为周期执行 `step_simulation()` 回调
- 每步读取状态 `[qpos[0], qpos[1], qvel[0], qvel[1]]`
- 计算 `u = -K @ state` 并施加到 `data.ctrl[0]`
- 将状态发布至 `/pendulum_state`
- 尝试启动 MuJoCo 被动查看器（失败则 fallback 为 headless）

### 2. 注册 entry_point

在 `setup.py` 的 `console_scripts` 中注册：

```python
'mj_lqr_node = mujoco_pendulum.mj_lqr_node:main',
```

### 3. 构建并运行

// turbo
```bash
cd ~/mojoco/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mujoco_pendulum
```

运行节点：

```bash
source ~/mojoco/ros2_ws/install/setup.bash
ros2 run mujoco_pendulum mj_lqr_node
```

### 4. 验证话题输出

在另一个终端中验证状态话题：

```bash
source /opt/ros/humble/setup.bash
source ~/mojoco/ros2_ws/install/setup.bash
ros2 topic echo /pendulum_state --once
```

预期输出类似：

```
data:
- 0.001234    # x_cart (m)
- 0.000012    # θ (rad, 接近 0 = 竖直)
- 0.005678    # ẋ_cart (m/s)
- -0.000345   # θ̇ (rad/s)
```

### 5. 验证话题频率

```bash
ros2 topic hz /pendulum_state
```

预期频率约 `500 Hz`（= 1/0.002s）。

---

## WSL2 可视化配置（可选）

如需 MuJoCo 窗口，WSL2 需要 X Server：

1. Windows 端安装 [VcXsrv](https://sourceforge.net/projects/vcxsrv/) 或使用 WSLg
2. 在 WSL 终端中设置：
   ```bash
   export DISPLAY=:0
   ```
3. 重新运行节点即可看到 MuJoCo 可视化窗口

---

## 产出物

| 文件 | 说明 |
|------|------|
| `mujoco_pendulum/mj_lqr_node.py` | ROS 2 仿真+控制节点 |
| `setup.py` | 含 entry_point 注册 |

## 设计原则

- 仿真和控制在**同一节点**内完成（低延迟）
- 控制逻辑通过导入 `lqr_controller` 模块实现，保持解耦
- Viewer 故障不影响仿真运行（graceful fallback）
- 状态通过标准 ROS 话题发布，便于外部监控和录制
