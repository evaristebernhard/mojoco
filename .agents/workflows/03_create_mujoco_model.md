---
description: Create MuJoCo XML Model for Inverted Pendulum
---

## 概述

本工作流专注于创建倒立摆的 MuJoCo 物理模型文件（MJCF XML 格式）。
该文件是纯物理建模层，**不包含任何控制逻辑或 ROS 依赖**，确保模型定义与算法及框架解耦。

---

## 物理模型设计

### Cart-Pole 系统参数

| 参数            | 符号      | 值          | 说明                      |
|-----------------|-----------|-------------|---------------------------|
| 小车质量        | m_cart    | 1.0 kg      | 矩形盒体                  |
| 摆杆质量        | m_pole    | 0.1 kg      | 胶囊体(capsule)           |
| 顶端球质量      | m_tip     | 0.05 kg     | 球体，增强视觉效果         |
| 摆杆长度        | L         | 0.6 m       | fromto 定义               |
| 重力加速度      | g         | 9.81 m/s²   | 沿 -Z 轴                  |
| 仿真步长        | dt        | 0.002 s     | 2ms，保证控制精度          |
| 执行器力范围    | F         | ±20 N       | 小车水平推力              |
| 滑轨范围        | x_range   | ±2.5 m      | 小车位移限制              |

### 运动学结构

```
world
 └── floor (plane, visual)
 └── cart (body, pos=[0,0,0.5])
      ├── slider joint (slide, x-axis, range ±2.5)
      ├── cart_geom (box)
      └── pole (body, pos=[0,0,0.1])
           ├── hinge joint (hinge, y-axis)
           ├── pole_geom (capsule, length=0.6)
           └── tip (body, pos=[0,0,0.6])
                └── tip_geom (sphere)
```

---

## 步骤

### 1. 创建模型文件目录

// turbo
```bash
mkdir -p ~/mojoco/ros2_ws/src/mujoco_pendulum/models
```

### 2. 编写 MJCF XML 模型

在 `models/inverted_pendulum.xml` 中编写完整的倒立摆模型，需包含：

- `<option>`: 时间步长 `0.002`, 积分器 `RK4`, 重力 `"0 0 -9.81"`
- `<default>`: 设置关节默认阻尼
- `<worldbody>`:
  - 地面 (`<geom type="plane">`)
  - 小车 (`<body>` + `<joint type="slide">` + `<geom type="box">`)
  - 摆杆 (`<body>` + `<joint type="hinge">` + `<geom type="capsule">`)
  - 顶端球 (`<body>` + `<geom type="sphere">`)
- `<actuator>`: 力执行器 (`<motor joint="slider">`)

**文件路径**: `~/mojoco/ros2_ws/src/mujoco_pendulum/models/inverted_pendulum.xml`

### 3. 验证模型加载

// turbo
```bash
python3 -c "
import mujoco
import os
model = mujoco.MjModel.from_xml_path(os.path.expanduser('~/mojoco/ros2_ws/src/mujoco_pendulum/models/inverted_pendulum.xml'))
data = mujoco.MjData(model)
print('✅ 模型加载成功')
print(f'   自由度 (nv): {model.nv}')
print(f'   执行器 (nu): {model.nu}')
print(f'   仿真步长 (dt): {model.opt.timestep}')
mujoco.mj_step(model, data)
print(f'   一步仿真后时间: {data.time:.4f}s')
print('✅ 仿真步进正常')
"
```

### 4. 在 setup.py 中注册模型资源

确保 `setup.py` 的 `data_files` 包含 models 目录：

```python
(os.path.join('share', package_name, 'models'), glob('models/*.xml')),
```

### 5. 重新构建验证

// turbo
```bash
cd ~/mojoco/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select mujoco_pendulum
```

---

## 产出物

| 文件 | 说明 |
|------|------|
| `models/inverted_pendulum.xml` | 倒立摆 MJCF 模型（含关节、执行器） |

## 设计原则

- 模型文件是**纯物理描述**，不含任何控制或 ROS 逻辑
- 执行器有物理范围限制 (±20N)，保证仿真真实性
- 使用 RK4 积分器提高数值稳定性
