# MuJoCo Inverted Pendulum with LQR Control (ROS 2)

A real-time **cart-pole (inverted pendulum)** simulation built with [MuJoCo](https://mujoco.org/) and controlled by an **LQR (Linear Quadratic Regulator)** controller, integrated into the **ROS 2 Humble** ecosystem.

## Demo

The system balances an inverted pendulum on a sliding cart by computing the optimal control force in real time via LQR.

```
     ●  ← tip mass (0.05 kg)
     |
     |  ← pole (0.1 kg, 0.6 m)
     |
  [=====]  ← cart (1.0 kg)
─────────────────────────────
      → F (LQR force, ±20 N)
```

---

## Features

- 🎮 **MuJoCo physics** — `RK4` integrator, 500 Hz simulation
- 🧮 **LQR controller** — solves the continuous Riccati equation via `scipy`
- 📡 **ROS 2 integration** — publishes state on `/pendulum_state` at 500 Hz
- 📊 **Real-time visualization** — 3D MuJoCo viewer + live Matplotlib plots
- 🎛️ **Live parameter tuning** — Tkinter panel to adjust Q/R weights on the fly

---

## System Requirements

| Dependency | Version |
|---|---|
| Ubuntu | 22.04 |
| ROS 2 | Humble |
| MuJoCo | ≥ 3.x |
| Python | 3.10 |
| numpy | any |
| scipy | any |
| matplotlib | any |

---

## Installation

### 1. Install ROS 2 Humble

```bash
# Follow official guide: https://docs.ros.org/en/humble/Installation.html
sudo apt install ros-humble-desktop python3-colcon-common-extensions
```

### 2. Install MuJoCo Python bindings

```bash
pip install mujoco
```

### 3. Clone and build

```bash
git clone https://github.com/evaristebernhard/mojoco.git
cd mojoco/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## Usage

### Run the LQR simulation node

```bash
ros2 run mujoco_pendulum mj_lqr_node
```

Starts the MuJoCo physics simulation with LQR control. Publishes state on `/pendulum_state`.

### Run the visualized node (with plots & parameter panel)

```bash
ros2 run mujoco_pendulum mj_visualized_node
```

Opens three windows simultaneously:
- **MuJoCo 3D viewer** — real-time physics rendering
- **Matplotlib plots** — cart position/velocity, pole angle/angular velocity, control force
- **Tkinter panel** — live Q/R weight tuning and simulation reset

> ⚠️ WSL2 users: requires an X server (e.g. [VcXsrv](https://sourceforge.net/projects/vcxsrv/)) for GUI windows.

### Monitor state from command line

```bash
# Real-time state stream [x_cart, θ, ẋ_cart, θ̇]
ros2 topic echo /pendulum_state

# Check publish rate (~500 Hz)
ros2 topic hz /pendulum_state
```

---

## Project Structure

```
ros2_ws/src/mujoco_pendulum/
├── models/
│   └── inverted_pendulum.xml   # MuJoCo model (cart + pole + tip)
├── mujoco_pendulum/
│   ├── lqr_controller.py       # Standalone LQR solver (numpy/scipy only)
│   ├── mj_lqr_node.py          # ROS 2 node: simulation + LQR control
│   └── mj_visualized_node.py   # ROS 2 node: 3D viewer + plots + panel
├── package.xml
└── setup.py
```

---

## LQR Theory

The cart-pole is linearised around the upright equilibrium (θ = 0):

```
ẋ = A·x + B·u
```

**State vector**: `x = [x_cart, θ, ẋ_cart, θ̇]`  
**Control input**: `u = F` (horizontal force on cart, Newtons)

The optimal gain **K** minimises:

```
J = ∫₀^∞ (xᵀQx + uᵀRu) dt
```

Default weights:

```python
Q = diag([10, 100, 1, 10])   # [x, θ, ẋ, θ̇] — θ most penalised
R = [[0.1]]                   # small → allows aggressive control
```

**Physical parameters** (matching the XML model):

| Parameter | Value |
|---|---|
| Cart mass | 1.0 kg |
| Pole mass (rod + tip) | 0.15 kg |
| Pole length | 0.6 m |
| Max control force | ±20 N |
| Simulation timestep | 2 ms (500 Hz) |

---

## ROS 2 Topics

| Topic | Type | Description |
|---|---|---|
| `/pendulum_state` | `std_msgs/Float64MultiArray` | `[x_cart, θ, ẋ_cart, θ̇]` published at 500 Hz |

---

## License

MIT
