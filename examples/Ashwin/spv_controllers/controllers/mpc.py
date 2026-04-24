"""MPC controller — direct port of scripts/mpc_controller.m.

Linear time-varying MPC with slip-dependent thrust.
Uses OSQP for the QP solve (replaces MATLAB quadprog).
"""

from __future__ import annotations

import math

import numpy as np
from scipy.linalg import expm

from ..params import RoverParams

try:
    import osqp
    from scipy import sparse

    _HAS_OSQP = True
except ImportError:
    _HAS_OSQP = False


def _coulomb_force(s: float, F_prop: float, eps: float) -> float:
    """Regularized Coulomb friction: F_prop * s / sqrt(s^2 + eps^2)."""
    return F_prop * s / math.sqrt(s ** 2 + eps ** 2)


def _coulomb_slope(s: float, F_prop: float, eps: float) -> float:
    """Derivative of regularized Coulomb force w.r.t. slip."""
    eps2 = eps ** 2
    return F_prop * eps2 / (s ** 2 + eps2) ** 1.5


def _plant_to_mpc_state(s: np.ndarray) -> np.ndarray:
    theta = s[2]
    dx, dy, dtheta = s[3], s[4], s[5]
    v_fwd = dx * math.cos(theta) + dy * math.sin(theta)
    return np.array([s[0], s[1], theta, v_fwd, dtheta])


def _linearize(z0: np.ndarray, u0: np.ndarray, p: RoverParams):
    """Linearize 5-state prediction model (matches mpc_controller.m)."""
    nx, nu = 5, 2
    theta0, v0, omega0 = z0[2], z0[3], z0[4]
    VL0, VR0 = u0[0], u0[1]

    cth = math.cos(theta0)
    sth = math.sin(theta0)
    ca = p.cos_alpha
    sa = p.sin_alpha
    d2 = p.d_drives / 2
    rd = p.r_drive

    oL = max((VL0 - p.V_offset) * p.omega_per_V, 0.0)
    oR = max((VR0 - p.V_offset) * p.omega_per_V, 0.0)

    vbL_fwd = rd * oL * ca
    vbR_fwd = rd * oR * ca

    vcL_fwd = v0
    vcL_lat = d2 * omega0
    vcR_fwd = v0
    vcR_lat = -d2 * omega0

    sL_fwd = vbL_fwd - vcL_fwd
    sR_fwd = vbR_fwd - vcR_fwd

    # Coulomb friction model: F = F_prop * slip / sqrt(slip^2 + eps^2)
    F_prop = p.mu_p * p.N_drive  # force magnitude per drive [N]

    FL_fwd0 = _coulomb_force(sL_fwd, F_prop, p.eps_slip)
    FR_fwd0 = _coulomb_force(sR_fwd, F_prop, p.eps_slip)

    kL_fwd = _coulomb_slope(sL_fwd, F_prop, p.eps_slip)
    kR_fwd = _coulomb_slope(sR_fwd, F_prop, p.eps_slip)

    # Minimum slope for numerical stability
    min_k = F_prop * 0.01
    kL_fwd = max(kL_fwd, min_k)
    kR_fwd = max(kR_fwd, min_k)

    speed_L = math.sqrt(vcL_fwd ** 2 + vcL_lat ** 2 + p.eps_fric ** 2)
    speed_R = math.sqrt(vcR_fwd ** 2 + vcR_lat ** 2 + p.eps_fric ** 2)

    Fdrag_L_fwd = -(p.c_drag_fwd * vcL_fwd + p.F_coulomb * vcL_fwd / speed_L)
    Fdrag_R_fwd = -(p.c_drag_fwd * vcR_fwd + p.F_coulomb * vcR_fwd / speed_R)

    c_drag_slope_L = p.c_drag_fwd + p.F_coulomb / speed_L
    c_drag_slope_R = p.c_drag_fwd + p.F_coulomb / speed_R

    Ftot_fwd = FL_fwd0 + FR_fwd0 + Fdrag_L_fwd + Fdrag_R_fwd
    FL_fwd_total = FL_fwd0 + Fdrag_L_fwd
    FR_fwd_total = FR_fwd0 + Fdrag_R_fwd
    tau_yaw0 = d2 * (FR_fwd_total - FL_fwd_total) - p.c_yaw * omega0

    m = p.m
    I = p.I_yaw

    f0 = np.array([v0 * cth, v0 * sth, omega0, Ftot_fwd / m, tau_yaw0 / I])

    Ac = np.zeros((nx, nx))
    Ac[0, 2] = -v0 * sth
    Ac[0, 3] = cth
    Ac[1, 2] = v0 * cth
    Ac[1, 3] = sth
    Ac[2, 4] = 1.0
    Ac[3, 3] = -(kL_fwd + kR_fwd + c_drag_slope_L + c_drag_slope_R) / m
    Ac[4, 3] = d2 * (kL_fwd - kR_fwd) / I
    Ac[4, 4] = -p.c_yaw / I

    dv = rd * p.omega_per_V
    dFL_dVL = kL_fwd * dv * ca
    dFR_dVR = kR_fwd * dv * ca
    min_dF_dV = 0.05
    dFL_dVL = max(dFL_dVL, min_dF_dV)
    dFR_dVR = max(dFR_dVR, min_dF_dV)

    Bc = np.zeros((nx, nu))
    Bc[3, 0] = dFL_dVL / m
    Bc[3, 1] = dFR_dVR / m
    Bc[4, 0] = -d2 * dFL_dVL / I
    Bc[4, 1] = d2 * dFR_dVR / I

    fc = f0 - Ac @ z0 - Bc @ u0
    return Ac, Bc, fc


def _solve_qp_osqp(H: np.ndarray, f: np.ndarray, lb: np.ndarray, ub: np.ndarray) -> tuple[np.ndarray, int]:
    """Solve box-constrained QP: min 0.5*x'Hx + f'x  s.t. lb <= x <= ub."""
    n = len(f)
    P = sparse.csc_matrix(H)
    A = sparse.eye(n, format="csc")
    solver = osqp.OSQP()
    solver.setup(P, f, A, lb, ub, verbose=False, eps_abs=1e-8, eps_rel=1e-8, max_iter=4000)
    res = solver.solve()
    return res.x, res.info.status_val


def _solve_qp_fallback(H: np.ndarray, f: np.ndarray, lb: np.ndarray, ub: np.ndarray) -> tuple[np.ndarray, int]:
    """Unconstrained solve then clamp (fallback when OSQP unavailable)."""
    try:
        x = np.linalg.solve(H, -f)
    except np.linalg.LinAlgError:
        x = -np.linalg.lstsq(H, f, rcond=None)[0]
    return np.clip(x, lb, ub), -99


def mpc_controller(
    s_plant: np.ndarray,
    z_ref: np.ndarray,
    u_prev: np.ndarray,
    p: RoverParams,
) -> tuple[float, float, dict]:
    """Linear time-varying MPC.

    z_ref: (5, N) reference window over prediction horizon.
    """
    nx, nu = 5, 2
    N = p.N_horizon
    dt = p.dt_pred

    z0 = _plant_to_mpc_state(s_plant)
    u0 = u_prev

    Ac, Bc, fc = _linearize(z0, u0, p)

    # Discretize via matrix exponential
    M_aug = np.zeros((nx + nu + 1, nx + nu + 1))
    M_aug[:nx, :nx] = Ac
    M_aug[:nx, nx : nx + nu] = Bc
    M_aug[:nx, nx + nu] = fc
    eM = expm(M_aug * dt)
    Ad = eM[:nx, :nx]
    Bd = eM[:nx, nx : nx + nu]
    fd = eM[:nx, nx + nu]

    # Build condensed QP: Z = Phi*z0 + Gamma*U + Psi
    Ad_powers = [None] * N
    Ad_powers[0] = Ad.copy()
    for k in range(1, N):
        Ad_powers[k] = Ad_powers[k - 1] @ Ad

    Phi = np.zeros((N * nx, nx))
    for k in range(N):
        Phi[k * nx : (k + 1) * nx, :] = Ad_powers[k]

    Gamma = np.zeros((N * nx, N * nu))
    Psi = np.zeros(N * nx)
    for k in range(N):
        for j in range(k + 1):
            row = slice(k * nx, (k + 1) * nx)
            col = slice(j * nu, (j + 1) * nu)
            if k == j:
                Gamma[row, col] = Bd
            else:
                Gamma[row, col] = Ad_powers[k - j - 1] @ Bd
        psi_k = fd.copy()
        for j in range(1, k):
            psi_k = psi_k + Ad_powers[j] @ fd
        if k > 0:
            psi_k = psi_k + Ad_powers[0] @ fd if k == 1 else psi_k
        Psi[k * nx : (k + 1) * nx] = psi_k if k > 0 else fd

    # Cost matrices
    Qbar = np.zeros((N * nx, N * nx))
    for k in range(N - 1):
        Qbar[k * nx : (k + 1) * nx, k * nx : (k + 1) * nx] = p.Q_mpc
    Qbar[(N - 1) * nx : N * nx, (N - 1) * nx : N * nx] = p.Q_term

    Rbar = np.kron(np.eye(N), p.R_mpc)

    # Rate cost
    T = np.eye(N * nu)
    for k in range(1, N):
        T[k * nu : (k + 1) * nu, (k - 1) * nu : k * nu] = -np.eye(nu)
    T0 = np.zeros((N * nu, nu))
    T0[:nu, :] = np.eye(nu)
    dRbar = np.kron(np.eye(N), p.dR_mpc)

    Zref = z_ref[:, :N].reshape(-1, order="F")

    e0 = Phi @ z0 + Psi - Zref
    H = Gamma.T @ Qbar @ Gamma + Rbar + T.T @ dRbar @ T
    H = (H + H.T) / 2
    f_qp = Gamma.T @ Qbar @ e0 - T.T @ dRbar @ T0 @ u_prev

    lb = np.tile([p.V_min, p.V_min], N)
    ub = np.tile([p.V_max, p.V_max], N)

    solve_fn = _solve_qp_osqp if _HAS_OSQP else _solve_qp_fallback
    U_opt, exitflag = solve_fn(H, f_qp, lb, ub)

    V_L = max(min(float(U_opt[0]), p.V_max), p.V_min)
    V_R = max(min(float(U_opt[1]), p.V_max), p.V_min)

    Z_pred = Phi @ z0 + Gamma @ U_opt + Psi
    info = {
        "Z_pred": Z_pred.reshape(nx, N, order="F"),
        "U_opt": U_opt.reshape(nu, N, order="F"),
        "cost": 0.5 * U_opt @ H @ U_opt + f_qp @ U_opt,
        "exitflag": exitflag,
        "z0": z0,
    }
    return V_L, V_R, info
