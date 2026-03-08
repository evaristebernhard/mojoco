---
description: Implement LQR Control Logic
---

## 概述

本工作流实现倒立摆的 **LQR (Linear Quadratic Regulator)** 控制逻辑。
LQR 模块作为**独立的 Python 模块**实现，与 ROS 和 MuJoCo 解耦，便于单元测试和复用。

---

## 理论背景

### 线性化动力学模型

Cart-pole 系统在竖直平衡点 (θ=0) 附近线性化后，状态空间方程为：

```
ẋ = A·x + B·u
```

**状态向量**: `x = [x_cart, θ, ẋ_cart, θ̇]`
**控制输入**: `u = F` (施加在小车上的水平力)

系统矩阵（小角度近似）：

```
        ┌                       ┐       ┌           ┐
        │  0      0      1    0 │       │     0     │
        │  0      0      0    1 │       │     0     │
A   =   │  0   -mₚg/mₜ  0    0 │,  B = │   1/mₜ   │
        │  0    mₜg/Lmₜ 0    0 │       │  -1/Lmₜ  │
        └                       ┘       └           ┘

其中: mₜ = m_cart + m_pole
```

### LQR 求解

最优控制律 `u = -K·x` 通过求解连续时间代数 Riccati 方程（CARE）得到：

```
A'P + PA - PBR⁻¹B'P + Q = 0
K = R⁻¹ B' P
```

### 权重矩阵选择

| 矩阵 | 值 | 含义 |
|------|-----|------|
| Q | diag([10, 100, 1, 10]) | 状态权重：摆角最重要(100)，其次位置(10)和角速度(10) |
| R | [[0.1]] | 控制代价：较小，允许较大力控制 |

---

## 步骤

### 1. 创建 LQR 控制模块

在 `mujoco_pendulum/` 包中创建 `lqr_controller.py`：

**文件路径**: `~/mojoco/ros2_ws/src/mujoco_pendulum/mujoco_pendulum/lqr_controller.py`

该模块应包含：

- `lqr(A, B, Q, R)` 函数：通用 LQR 求解器，调用 `scipy.linalg.solve_continuous_are`
- `cart_pole_lqr_gain(m_cart, m_pole, L, g)` 函数：根据物理参数构建 A、B 矩阵，返回增益 K
- 默认参数应与 MuJoCo 模型中的质量参数一致

### 2. 验证 LQR 增益计算

// turbo
```bash
cd ~/mojoco/ros2_ws/src/mujoco_pendulum
python3 -c "
from mujoco_pendulum.lqr_controller import cart_pole_lqr_gain
import numpy as np
K = cart_pole_lqr_gain()
print('LQR 增益矩阵 K:')
print(K)
print(f'K 形状: {K.shape}')
assert K.shape == (1, 4), 'K 应为 1x4 矩阵'
print('✅ LQR 增益计算正常')
# 验证闭环特征值（应全部实部为负 → 稳定）
A = np.array([[0,0,1,0],[0,0,0,1],[0,-0.15*9.81/1.15,0,0],[0,1.15*9.81/(0.6*1.15),0,0]])
B = np.array([[0],[0],[1/1.15],[-1/(0.6*1.15)]])
eig = np.linalg.eigvals(A - B @ K)
print(f'闭环特征值: {eig}')
assert all(e.real < 0 for e in eig), '闭环应稳定（特征值实部均为负）'
print('✅ 闭环系统稳定')
"
```

### 3. 无 ROS 纯 MuJoCo 验证

创建一个临时验证脚本，确认 LQR 能在 MuJoCo 仿真中稳定摆起：

// turbo
```bash
python3 -c "
import mujoco, numpy as np, os, sys
sys.path.insert(0, os.path.expanduser('~/mojoco/ros2_ws/src/mujoco_pendulum'))
from mujoco_pendulum.lqr_controller import cart_pole_lqr_gain

model = mujoco.MjModel.from_xml_path(os.path.expanduser('~/mojoco/ros2_ws/src/mujoco_pendulum/models/inverted_pendulum.xml'))
data = mujoco.MjData(model)
K = cart_pole_lqr_gain()

# 给一个初始扰动
data.qpos[1] = 0.1  # ~6 deg tilt

for i in range(2000):
    state = np.array([data.qpos[0], data.qpos[1], data.qvel[0], data.qvel[1]])
    u = float(-K @ state)
    data.ctrl[0] = np.clip(u, -20, 20)
    mujoco.mj_step(model, data)

theta_final = abs(data.qpos[1])
print(f'仿真 {2000*model.opt.timestep:.1f}s 后:')
print(f'  摆角 θ = {np.degrees(data.qpos[1]):.4f}°')
print(f'  小车位置 x = {data.qpos[0]:.4f}m')
assert theta_final < 0.01, f'摆角应趋近0，实际 {theta_final:.4f}'
print('✅ LQR 控制在 MuJoCo 中有效，摆杆已稳定')
"
```

---

## 产出物

| 文件 | 说明 |
|------|------|
| `mujoco_pendulum/lqr_controller.py` | 独立 LQR 控制模块（无 ROS/MuJoCo 依赖） |

## 设计原则

- LQR 模块**不依赖 ROS 或 MuJoCo**，纯数学计算
- 物理参数默认值与 XML 模型一致，但可通过参数覆盖
- 先脱离 ROS 用纯 Python+MuJoCo 验证控制效果
