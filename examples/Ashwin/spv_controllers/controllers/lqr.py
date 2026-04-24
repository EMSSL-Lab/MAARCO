"""LQR controller — direct port of results/lqr_controller.m."""

from __future__ import annotations

import math

import numpy as np
from scipy.linalg import expm

from ..params import RoverParams


def angle_wrap(e: float) -> float:
    return (e + math.pi) % (2 * math.pi) - math.pi


def dare_lqr(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> np.ndarray:
    """Solve discrete algebraic Riccati equation iteratively (no Control Toolbox)."""
    P = Q.copy()
    for _ in range(200):
        BPB_R = R + B.T @ P @ B
        K_new = np.linalg.solve(BPB_R, B.T @ P @ A)
        P_new = A.T @ P @ A - A.T @ P @ B @ K_new + Q
        if np.max(np.abs(P_new - P)) < 1e-10:
            break
        P = P_new
    return np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)


def linearize_5state(z: np.ndarray, u: np.ndarray, p: RoverParams):
    """Linearize the 5-state SPV prediction model.

    z = [x, y, theta, v, omega], u = [V_L, V_R]
    """
    theta = z[2]
    v = z[3]

    V_L, V_R = u[0], u[1]

    omL = max((V_L - p.V_offset) * p.omega_per_V, 0.0)
    omR = max((V_R - p.V_offset) * p.omega_per_V, 0.0)

    vbL = p.r_drive * omL * p.cos_alpha
    vbR = p.r_drive * omR * p.cos_alpha

    sL = vbL - v
    sR = vbR - v

    # Coulomb friction linearization: F = F_prop * slip / sqrt(slip^2 + eps^2)
    # dF/dslip = F_prop * eps^2 / (slip^2 + eps^2)^(3/2)
    F_prop = p.mu_p * p.N_drive  # force magnitude per drive [N]
    eps2 = p.eps_slip ** 2

    kL = F_prop * eps2 / (sL ** 2 + eps2) ** 1.5  # [N/(m/s)]
    kR = F_prop * eps2 / (sR ** 2 + eps2) ** 1.5

    # Minimum slope for numerical stability (avoids zero Jacobian at zero slip)
    min_k = F_prop * 0.01
    kL = max(kL, min_k)
    kR = max(kR, min_k)

    dFdv_L = -kL
    dFdv_R = -kR
    dFdomL = kL * p.r_drive * p.cos_alpha
    dFdomR = kR * p.r_drive * p.cos_alpha

    dFdVL = dFdomL * p.omega_per_V
    dFdVR = dFdomR * p.omega_per_V

    c_eff = p.c_drag_fwd * 2

    cth = math.cos(theta)
    sth = math.sin(theta)

    Ac = np.zeros((5, 5))
    Ac[0, 2] = -v * sth
    Ac[0, 3] = cth
    Ac[1, 2] = v * cth
    Ac[1, 3] = sth
    Ac[2, 4] = 1.0
    Ac[3, 3] = (dFdv_L + dFdv_R - c_eff) / p.m
    Ac[4, 4] = -p.c_yaw / p.I_yaw

    Bc = np.zeros((5, 2))
    Bc[3, 0] = dFdVL / p.m
    Bc[3, 1] = dFdVR / p.m
    Bc[4, 0] = -dFdVL * (p.d_drives / 2) / p.I_yaw
    Bc[4, 1] = dFdVR * (p.d_drives / 2) / p.I_yaw

    fc = np.zeros(5)
    return Ac, Bc, fc


def lqr_controller(
    s_plant: np.ndarray,
    z_ref: np.ndarray,
    u_prev: np.ndarray,
    p: RoverParams,
) -> tuple[float, float, dict]:
    """Time-varying LQR for SPV trajectory tracking."""
    theta = s_plant[2]
    dx, dy, dtheta = s_plant[3], s_plant[4], s_plant[5]
    v_fwd = dx * math.cos(theta) + dy * math.sin(theta)
    z = np.array([s_plant[0], s_plant[1], theta, v_fwd, dtheta])

    e = z - z_ref
    e[2] = angle_wrap(e[2])

    Ac, Bc, _ = linearize_5state(z, u_prev, p)

    # Discretize via ZOH
    dt = p.dt_ctrl
    n, m = 5, 2
    M_aug = np.zeros((n + m, n + m))
    M_aug[:n, :n] = Ac
    M_aug[:n, n:] = Bc
    eM = expm(M_aug * dt)
    Ad = eM[:n, :n]
    Bd = eM[:n, n:]

    try:
        K = dare_lqr(Ad, Bd, p.Q_lqr, p.R_lqr)
    except np.linalg.LinAlgError:
        return u_prev[0], u_prev[1], {"K": np.zeros((2, 5)), "status": "dare_failed"}

    # Feedforward
    v_ref, omega_ref = z_ref[3], z_ref[4]
    eta = p.eta_0 if p.eta_0 > 0 else 0.252
    V_avg_eq = v_ref / (eta * p.r_drive * p.cos_alpha * p.omega_per_V) + p.V_offset

    if p.eta_diff_0 > 0:
        dV_eq = -omega_ref * p.d_drives / (p.eta_diff_0 * p.r_drive * p.cos_alpha * p.omega_per_V * 2)
    else:
        dV_eq = 0.0

    u_eq = np.array([V_avg_eq + dV_eq, V_avg_eq - dV_eq])
    u = u_eq - K @ e

    V_L = max(min(float(u[0]), p.V_max), p.V_min)
    V_R = max(min(float(u[1]), p.V_max), p.V_min)

    return V_L, V_R, {"K": K, "e": e, "u_eq": u_eq, "status": "ok"}
