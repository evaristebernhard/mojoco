#!/usr/bin/env python3
"""
MuJoCo Inverted Pendulum LQR Controller — ROS 2 Node

This node:
  1. Loads an inverted-pendulum (cart-pole) MuJoCo model.
  2. Imports the LQR gain from the standalone lqr_controller module.
  3. Runs a real-time simulation loop, publishing state on /pendulum_state
     and applying LQR-computed forces to the cart.
"""

import os
import numpy as np
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory

from mujoco_pendulum.lqr_controller import cart_pole_lqr_gain


class InvertedPendulumLQRNode(Node):
    def __init__(self):
        super().__init__('mj_lqr_node')
        self.get_logger().info('Starting MuJoCo Inverted Pendulum LQR Node …')

        # --- Load MuJoCo model ---
        pkg_share = get_package_share_directory('mujoco_pendulum')
        model_path = os.path.join(pkg_share, 'models', 'inverted_pendulum.xml')
        self.get_logger().info(f'Loading model from: {model_path}')

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # --- Compute LQR gain (from standalone module) ---
        self.K = cart_pole_lqr_gain()
        self.get_logger().info(f'LQR gain K = {self.K}')

        # --- ROS publisher ---
        self.state_pub = self.create_publisher(
            Float64MultiArray, '/pendulum_state', 10
        )

        # --- Give the pendulum a small initial disturbance ---
        self.data.qpos[1] = 0.05  # ~3 deg tilt
        mujoco.mj_forward(self.model, self.data)

        # --- Timer at ~500 Hz (model timestep 2 ms) ---
        dt = float(self.model.opt.timestep)
        self.sim_timer = self.create_timer(dt, self.step_simulation)

        # --- MuJoCo passive viewer (optional, requires X server on WSL2) ---
        try:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.has_viewer = True
            self.get_logger().info('MuJoCo viewer launched.')
        except Exception as e:
            self.has_viewer = False
            self.get_logger().warn(f'Could not launch viewer: {e}')

    def step_simulation(self):
        """Timer callback: read state → LQR → apply force → step physics → publish."""
        # Read state: [x_cart, θ, ẋ_cart, θ̇]
        state = np.array([
            self.data.qpos[0],
            self.data.qpos[1],
            self.data.qvel[0],
            self.data.qvel[1],
        ])

        # LQR control: u = -K·x
        u = float(-self.K @ state)
        self.data.ctrl[0] = np.clip(u, -20.0, 20.0)

        # Step physics
        mujoco.mj_step(self.model, self.data)

        # Publish state
        msg = Float64MultiArray()
        msg.data = state.tolist()
        self.state_pub.publish(msg)

        # Sync viewer
        if self.has_viewer:
            self.viewer.sync()

    def destroy_node(self):
        if self.has_viewer:
            self.viewer.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InvertedPendulumLQRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
