#!/usr/bin/env python3
"""
LQR Controller Module for Cart-Pole (Inverted Pendulum)

This module is a **standalone** control library with NO dependencies on
ROS 2 or MuJoCo.  It only requires numpy and scipy.

Theory
------
The cart-pole is linearised around the upright equilibrium (θ = 0):

    ẋ = A·x + B·u

State vector : x = [x_cart, θ, ẋ_cart, θ̇]
Control input: u = F  (horizontal force on cart, in Newtons)

The optimal control law  u = −K·x  is obtained by solving the
continuous-time algebraic Riccati equation (CARE).
"""

import numpy as np
from scipy import linalg


# ─────────────────── Generic LQR solver ───────────────────
def lqr(A, B, Q, R):
    """Solve the continuous-time LQR problem.

    Finds the gain matrix K such that the control law  u = -K·x
    minimises the infinite-horizon cost  ∫₀^∞ (x'Qx + u'Ru) dt.

    Parameters
    ----------
    A : ndarray (n, n) – system dynamics matrix
    B : ndarray (n, m) – input matrix
    Q : ndarray (n, n) – state cost (positive semi-definite)
    R : ndarray (m, m) – control cost (positive definite)

    Returns
    -------
    K : ndarray (m, n) – optimal gain matrix
    """
    P = linalg.solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    return K


# ──────────────── Cart-Pole specific gain ─────────────────
def cart_pole_lqr_gain(m_cart=1.0, m_pole=0.15, L=0.6, g=9.81,
                       Q=None, R=None):
    """Compute the LQR gain K for a cart-pole system.

    Parameters
    ----------
    m_cart : float – mass of the cart  (kg)
    m_pole : float – total pole mass (rod + tip)  (kg)
    L      : float – effective pole length  (m)
    g      : float – gravitational acceleration  (m/s²)
    Q      : ndarray (4,4) or None – state cost matrix
    R      : ndarray (1,1) or None – control cost matrix

    Returns
    -------
    K : ndarray (1, 4) – LQR gain vector

    Notes
    -----
    Default Q = diag([10, 100, 1, 10]):
      - θ  weighted 100 (most important: keep upright)
      - x  weighted 10  (keep near centre)
      - θ̇  weighted 10  (damp oscillations)
      - ẋ  weighted 1   (least important)

    Default R = [[0.1]]:
      - Small value → allows large forces for aggressive stabilisation
    """
    mt = m_cart + m_pole  # total mass

    # Linearised A matrix (small-angle approximation)
    A = np.array([
        [0,  0,                 1,  0],
        [0,  0,                 0,  1],
        [0, -m_pole * g / mt,   0,  0],
        [0,  mt * g / (L * mt), 0,  0],
    ])

    # Linearised B matrix
    B = np.array([
        [0],
        [0],
        [1.0 / mt],
        [-1.0 / (L * mt)],
    ])

    if Q is None:
        Q = np.diag([10.0, 100.0, 1.0, 10.0])
    if R is None:
        R = np.array([[0.1]])

    return lqr(A, B, Q, R)


# ──────────────────── Quick self-test ─────────────────────
if __name__ == '__main__':
    K = cart_pole_lqr_gain()
    print(f'LQR gain K = {K}')
    print(f'K shape: {K.shape}')

    # Verify closed-loop stability
    mt = 1.0 + 0.15
    A = np.array([
        [0, 0, 1, 0],
        [0, 0, 0, 1],
        [0, -0.15 * 9.81 / mt, 0, 0],
        [0, mt * 9.81 / (0.6 * mt), 0, 0],
    ])
    B = np.array([[0], [0], [1 / mt], [-1 / (0.6 * mt)]])
    eigvals = np.linalg.eigvals(A - B @ K)
    print(f'Closed-loop eigenvalues: {eigvals}')
    assert all(e.real < 0 for e in eigvals), 'System is NOT stable!'
    print('✅ Closed-loop system is stable')
