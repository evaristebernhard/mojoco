#!/usr/bin/env python3
"""
MuJoCo Inverted Pendulum — Visualized Node

Features:
  1. MuJoCo 3D viewer with HUD text overlay
  2. Real-time Matplotlib plots (3 subplots: pos/vel, angle/angvel, force)
  3. Tkinter parameter panel (Q/R weights, max force, initial angle, reset)
"""

import os
import threading
import collections
import numpy as np
import mujoco
import mujoco.viewer
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory

from mujoco_pendulum.lqr_controller import cart_pole_lqr_gain


# ──────────────── Data history buffer ─────────────────────
HISTORY_LEN = 2000  # ~4 seconds at 500 Hz


class StateHistory:
    """Thread-safe ring buffer for plotting."""

    def __init__(self, n=HISTORY_LEN):
        self.t = collections.deque(maxlen=n)
        self.x = collections.deque(maxlen=n)
        self.theta = collections.deque(maxlen=n)
        self.x_dot = collections.deque(maxlen=n)
        self.theta_dot = collections.deque(maxlen=n)
        self.force = collections.deque(maxlen=n)
        self.lock = threading.Lock()

    def append(self, t, x, theta, x_dot, theta_dot, force):
        with self.lock:
            self.t.append(t)
            self.x.append(x)
            self.theta.append(np.degrees(theta))
            self.x_dot.append(x_dot)
            self.theta_dot.append(np.degrees(theta_dot))
            self.force.append(force)

    def snapshot(self):
        with self.lock:
            return (list(self.t), list(self.x), list(self.theta),
                    list(self.x_dot), list(self.theta_dot), list(self.force))


# ──────────────── Parameter Panel (Tkinter) ───────────────
class ParamPanel:
    """Small Tkinter window for live parameter tuning."""

    def __init__(self, on_refresh_cb):
        self.on_refresh = on_refresh_cb
        self.root = tk.Tk()
        self.root.title('LQR Parameter Panel')
        self.root.geometry('320x380')
        self.root.configure(bg='#1e1e2e')

        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TLabel', background='#1e1e2e', foreground='#cdd6f4',
                        font=('Consolas', 10))
        style.configure('TButton', font=('Consolas', 10, 'bold'))
        style.configure('TCheckbutton', background='#1e1e2e',
                        foreground='#cdd6f4', font=('Consolas', 10))

        frame = ttk.Frame(self.root, padding=12)
        frame.configure(style='TFrame')
        style.configure('TFrame', background='#1e1e2e')
        frame.pack(fill='both', expand=True)

        # --- Q weights ---
        self._add_label(frame, 'Q_cart', 0)
        self.q_cart = self._add_entry(frame, '10.0', 0)

        self._add_label(frame, 'Q_angle', 1)
        self.q_angle = self._add_entry(frame, '100.0', 1)

        self._add_label(frame, 'Q_cart_vel', 2)
        self.q_cart_vel = self._add_entry(frame, '1.0', 2)

        self._add_label(frame, 'Q_angle_vel', 3)
        self.q_angle_vel = self._add_entry(frame, '10.0', 3)

        # --- R ---
        self._add_label(frame, 'R_control', 4)
        self.r_control = self._add_entry(frame, '0.1', 4)

        # --- max force ---
        self._add_label(frame, 'max_force', 5)
        self.max_force = self._add_entry(frame, '20.0', 5)

        # --- initial angle ---
        self._add_label(frame, 'initial_angle_deg', 6)
        self.init_angle = self._add_entry(frame, '3.0', 6)

        # --- enable control ---
        self.control_enabled = tk.BooleanVar(value=True)
        ttk.Checkbutton(frame, text='enable_control',
                        variable=self.control_enabled).grid(
            row=7, column=0, columnspan=2, sticky='w', pady=4)

        # --- Refresh button ---
        ttk.Button(frame, text='  Refresh  ',
                   command=self._on_refresh).grid(
            row=8, column=0, columnspan=2, pady=10)

    def _add_label(self, parent, text, row):
        ttk.Label(parent, text=text).grid(row=row, column=0,
                                          sticky='w', pady=2)

    def _add_entry(self, parent, default, row):
        var = tk.StringVar(value=default)
        entry = ttk.Entry(parent, textvariable=var, width=14)
        entry.grid(row=row, column=1, sticky='e', pady=2, padx=(8, 0))
        return var

    def _on_refresh(self):
        params = {
            'Q': np.diag([
                float(self.q_cart.get()),
                float(self.q_angle.get()),
                float(self.q_cart_vel.get()),
                float(self.q_angle_vel.get()),
            ]),
            'R': np.array([[float(self.r_control.get())]]),
            'max_force': float(self.max_force.get()),
            'init_angle': np.radians(float(self.init_angle.get())),
            'control_enabled': self.control_enabled.get(),
        }
        self.on_refresh(params)

    def spin_once(self):
        self.root.update_idletasks()
        self.root.update()


# ──────────────── Real-time Matplotlib plots ──────────────
class LivePlotter:
    """Three-subplot figure updated via matplotlib FuncAnimation."""

    def __init__(self, history: StateHistory):
        self.history = history
        plt.style.use('dark_background')
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(
            3, 1, figsize=(7, 7), tight_layout=True)
        self.fig.canvas.manager.set_window_title('Cart-Pole Real-Time Plots')

        # Subplot 1: cart position & velocity
        self.line_x, = self.ax1.plot([], [], '#f38ba8', lw=1.2,
                                     label='cart_pos (m)')
        self.line_xd, = self.ax1.plot([], [], '#fab387', lw=1.0, ls='--',
                                      label='cart_vel (m/s)')
        self.ax1.set_ylabel('Cart')
        self.ax1.legend(loc='upper right', fontsize=8)
        self.ax1.grid(alpha=0.2)

        # Subplot 2: pole angle & angular velocity
        self.line_th, = self.ax2.plot([], [], '#a6e3a1', lw=1.2,
                                      label='pole_angle (°)')
        self.line_thd, = self.ax2.plot([], [], '#94e2d5', lw=1.0, ls='--',
                                       label='pole_angvel (°/s)')
        self.ax2.set_ylabel('Pole')
        self.ax2.legend(loc='upper right', fontsize=8)
        self.ax2.grid(alpha=0.2)

        # Subplot 3: control force
        self.line_f, = self.ax3.plot([], [], '#cba6f7', lw=1.2,
                                     label='control_force (N)')
        self.ax3.set_ylabel('Force')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.legend(loc='upper right', fontsize=8)
        self.ax3.grid(alpha=0.2)

        self.anim = FuncAnimation(self.fig, self._update, interval=100,
                                  blit=False, cache_frame_data=False)

    def _update(self, _frame):
        t, x, th, xd, thd, f = self.history.snapshot()
        if len(t) < 2:
            return

        self.line_x.set_data(t, x)
        self.line_xd.set_data(t, xd)
        self.line_th.set_data(t, th)
        self.line_thd.set_data(t, thd)
        self.line_f.set_data(t, f)

        for ax in (self.ax1, self.ax2, self.ax3):
            ax.set_xlim(t[0], t[-1] + 0.1)
            ax.relim()
            ax.autoscale_view(scalex=False)

    def show_nonblocking(self):
        plt.show(block=False)

    def spin_once(self):
        self.fig.canvas.flush_events()


# ──────────────── ROS 2 Visualized Node ───────────────────
class VisualizedPendulumNode(Node):
    def __init__(self):
        super().__init__('mj_visualized_node')
        self.get_logger().info('Starting Visualized Inverted Pendulum Node …')

        # --- Load MuJoCo model ---
        pkg_share = get_package_share_directory('mujoco_pendulum')
        model_path = os.path.join(pkg_share, 'models', 'inverted_pendulum.xml')
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # --- Params ---
        self.max_force = 20.0
        self.control_enabled = True

        # --- LQR gain ---
        self.K = cart_pole_lqr_gain()
        self.get_logger().info(f'LQR gain K = {self.K}')

        # --- State publisher ---
        self.state_pub = self.create_publisher(
            Float64MultiArray, '/pendulum_state', 10)

        # --- Initial disturbance ---
        self.data.qpos[1] = 0.05
        mujoco.mj_forward(self.model, self.data)

        # --- Data history ---
        self.history = StateHistory()

        # --- Current control ---
        self.current_force = 0.0

        # --- Param panel ---
        self.panel = ParamPanel(self._on_param_refresh)

        # --- MuJoCo viewer ---
        try:
            self.viewer = mujoco.viewer.launch_passive(
                self.model, self.data)
            self.has_viewer = True
            self.get_logger().info('MuJoCo viewer launched.')
        except Exception as e:
            self.has_viewer = False
            self.get_logger().warn(f'Could not launch viewer: {e}')

        # --- Live plotter ---
        self.plotter = LivePlotter(self.history)
        self.plotter.show_nonblocking()

        # --- Timer ---
        dt = float(self.model.opt.timestep)
        # Use a slower timer for the main loop to not overwhelm the GUI
        self.sim_steps_per_tick = 10  # 10 physics steps per timer tick
        self.sim_timer = self.create_timer(dt * self.sim_steps_per_tick,
                                           self.tick)

    def _on_param_refresh(self, params):
        """Called by the parameter panel when user clicks Refresh."""
        self.get_logger().info('Refreshing parameters …')
        Q = params['Q']
        R = params['R']
        self.max_force = params['max_force']
        self.control_enabled = params['control_enabled']

        # Recompute LQR gain
        self.K = cart_pole_lqr_gain(Q=Q, R=R)
        self.get_logger().info(f'New LQR gain K = {self.K}')

        # Reset simulation with new initial angle
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[1] = params['init_angle']
        mujoco.mj_forward(self.model, self.data)

        # Clear history
        self.history = StateHistory()
        self.plotter.history = self.history

    def tick(self):
        """Timer callback: run N physics steps, then update GUI."""
        for _ in range(self.sim_steps_per_tick):
            state = np.array([
                self.data.qpos[0], self.data.qpos[1],
                self.data.qvel[0], self.data.qvel[1],
            ])

            if self.control_enabled:
                u = float(-self.K @ state)
                self.current_force = np.clip(u, -self.max_force,
                                              self.max_force)
            else:
                self.current_force = 0.0

            self.data.ctrl[0] = self.current_force
            mujoco.mj_step(self.model, self.data)

        # Record final state of this tick
        state = np.array([
            self.data.qpos[0], self.data.qpos[1],
            self.data.qvel[0], self.data.qvel[1],
        ])
        self.history.append(
            self.data.time,
            state[0], state[1], state[2], state[3],
            self.current_force
        )

        # Publish
        msg = Float64MultiArray()
        msg.data = state.tolist()
        self.state_pub.publish(msg)

        # Sync MuJoCo viewer
        if self.has_viewer:
            self.viewer.sync()

        # Pump GUI event loops
        try:
            self.plotter.spin_once()
        except Exception:
            pass
        try:
            self.panel.spin_once()
        except Exception:
            pass

    def destroy_node(self):
        if self.has_viewer:
            self.viewer.close()
        plt.close('all')
        try:
            self.panel.root.destroy()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisualizedPendulumNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
