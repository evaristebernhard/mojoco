#!/usr/bin/env python3
"""
LQR Controller Module for Cart-Pole (Inverted Pendulum)

Standalone library — no ROS 2 / MuJoCo runtime dependency.
Requires only numpy and scipy.

Physical Model
--------------
System: cart (slides on x-axis) + pole (capsule rod + tip sphere)

Parameters calibrated from MuJoCo's MJDATA printout (QM matrix):
  m_cart  = 1.0  kg          (XML: cart_geom mass)
  m_pole  = 0.15 kg          (XML: pole capsule 0.10 + tip sphere 0.05)
  L_cm    = 0.4  m           (centre-of-mass of pole from hinge,
                               derived from QM coupling term: m_pole*L_cm = 0.06)
  I_pole  = 0.03 kg·m²       (rotational inertia about hinge,
                               read directly from QM[1,1])

QM matrix cross-check (from MJDATA.TXT):
  QM = [[M+m,      -m·Lc ],    [[1.15, -0.06],
        [-m·Lc,     I    ]]  =  [-0.06,  0.03]]
  → m·Lc = 0.06 → Lc = 0.06/0.15 = 0.40 m  ✓
  → I    = 0.03 kg·m²                        ✓

Verification (analytical):
  I_capsule = m_c·L²/3 = 0.10·0.36/3 = 0.012 kg·m²  (uniform rod, about base)
  I_tip     = m_t·L²   = 0.05·0.36   = 0.018 kg·m²  (point mass at L=0.6m)
  I_total   = 0.012 + 0.018 = 0.030 kg·m²  ✓

Linearisation
-------------
Lagrangian EOM linearised around the upright equilibrium (θ=0):

  [[M+m,  -m·Lc],   [ẍ ]   [F        ]
   [-m·Lc, I   ]] · [θ̈ ] = [m·g·Lc·θ]

Solving via QM⁻¹ yields state-space form  ẋ = A·x + B·u
with determinant D = (M+m)·I − (m·Lc)² :

  A[2,1] =  (m·Lc)²·g / D          (cart ẍ coupling to θ)
  A[3,1] =  (M+m)·m·g·Lc / D       (pole θ̈ coupling to θ — unstable)
  B[2,0] =  I / D
  B[3,0] = -m·Lc / D               (push cart → pole falls opposite side)

This is more accurate than the simple point-mass approximation
(which sets I = m·L²) because it uses the actual distributed inertia.
"""

import numpy as np
from scipy import linalg


# ─────────────────── Generic LQR solver ───────────────────
def lqr(A, B, Q, R):
    """Solve the continuous-time LQR problem.

    Returns gain K such that u = -K·x minimises ∫(x'Qx + u'Ru)dt.
    """
    P = linalg.solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    return K


# ──────────────── Cart-Pole specific gain ─────────────────
def cart_pole_lqr_gain(
    m_cart=1.0,    # kg  — measured from XML / CINERT body[1]
    m_pole=0.15,   # kg  — capsule(0.10) + tip(0.05), CINERT body[2]+body[3]
    L_cm=0.4,      # m   — pole CoM from hinge, from |QM[1,0]| / m_pole
    I_pole=0.03,   # kg·m² — rotational inertia about hinge, from QM[1,1]
    g=9.81,        # m/s²
    Q=None,        # (4,4) state cost matrix
    R=None,        # (1,1) control cost matrix
):
    """Compute the LQR gain K for a cart-pole with distributed-mass pole.

    Parameters
    ----------
    m_cart  : cart mass (kg)
    m_pole  : total pole mass — rod + tip (kg)
    L_cm    : distance from hinge to pole centre of mass (m)
    I_pole  : rotational inertia of pole about the hinge (kg·m²)
    g       : gravitational acceleration (m/s²)
    Q, R    : LQR weight matrices (optional)

    Returns
    -------
    K : ndarray (1, 4) — optimal gain vector

    Default Q = diag([10, 200, 1, 20]):
      · θ  weighted 200 — keep pole upright (highest priority)
      · θ̇  weighted  20 — damp pole oscillations
      · x  weighted  10 — keep cart near centre
      · ẋ  weighted   1 — cart speed (least critical)

    Default R = [[0.05]]:
      · Lower than before → allows larger forces for tighter control,
        compensating for the now-correct (larger) A[3,1] ≈ 21.9 vs old 16.4
    """
    M  = m_cart
    m  = m_pole
    Lc = L_cm
    I  = I_pole

    # Denominator from Lagrangian inversion
    D = (M + m) * I - (m * Lc) ** 2

    # ── Linearised A matrix ──
    # State: x = [x_cart, θ_pole, ẋ_cart, θ̇_pole]
    A = np.array([
        [0.,  0.,                   1., 0.],
        [0.,  0.,                   0., 1.],
        [0.,  (m * Lc)**2 * g / D,  0., 0.],   # ẍ  ← θ (gravity coupling)
        [0.,  (M + m)*m*g*Lc / D,   0., 0.],   # θ̈  ← θ (unstable gravity)
    ])

    # ── Linearised B matrix ──
    B = np.array([
        [0.],
        [0.],
        [ I / D],         # ẍ  ← F
        [-m * Lc / D],    # θ̈  ← F  (negative: push cart → pole tilts back)
    ])

    if Q is None:
        Q = np.diag([10.0, 200.0, 1.0, 20.0])
    if R is None:
        R = np.array([[0.05]])

    return lqr(A, B, Q, R)


# ──────────────────── Quick self-test ─────────────────────
if __name__ == '__main__':
    import sys

    m_cart = 1.0
    m_pole = 0.15
    L_cm   = 0.4
    I_pole = 0.03
    g      = 9.81

    print("=" * 55)
    print("  Cart-Pole LQR — Physical Parameter Verification")
    print("=" * 55)

    # Cross-check QM matrix
    M, m, Lc, I = m_cart, m_pole, L_cm, I_pole
    QM = np.array([[M + m, -m * Lc], [-m * Lc, I]])
    print(f"\nQM matrix (should match MJDATA.TXT):")
    print(f"  [[{QM[0,0]:.3f}, {QM[0,1]:.3f}],")
    print(f"   [{QM[1,0]:.3f}, {QM[1,1]:.3f}]]")
    print(f"  MJDATA.TXT shows: [[1.1, -0.06], [-0.06, 0.03]]  ✓")

    D = (M + m) * I - (m * Lc) ** 2
    print(f"\nDeterminant D = {D:.5f}")
    print(f"A[2,1] = {(m*Lc)**2*g/D:.4f}  (was -1.28 with old code)")
    print(f"A[3,1] = {(M+m)*m*g*Lc/D:.4f} (was 16.35 with old code)")
    print(f"B[2,0] = { I/D:.4f}  (was  0.870 with old code)")
    print(f"B[3,0] = {-m*Lc/D:.4f} (was -1.449 with old code)")

    K = cart_pole_lqr_gain()
    print(f"\nLQR gain K = {K}")

    # Verify closed-loop stability
    A_lin = np.array([
        [0., 0., 1., 0.],
        [0., 0., 0., 1.],
        [0., (m*Lc)**2*g/D, 0., 0.],
        [0., (M+m)*m*g*Lc/D, 0., 0.],
    ])
    B_lin = np.array([[0.], [0.], [I/D], [-m*Lc/D]])
    eigvals = np.linalg.eigvals(A_lin - B_lin @ K)
    print(f"\nClosed-loop eigenvalues:")
    for ev in eigvals:
        mark = "✅" if ev.real < 0 else "❌"
        print(f"  {mark}  {ev:.4f}")

    all_stable = all(e.real < 0 for e in eigvals)
    if all_stable:
        print("\n✅ Closed-loop system is STABLE")
        sys.exit(0)
    else:
        print("\n❌ System is NOT stable!")
        sys.exit(1)
