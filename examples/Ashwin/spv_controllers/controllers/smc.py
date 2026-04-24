"""Sliding mode controller — direct port of results/smc_controller.m."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..params import RoverParams


def angle_wrap(e: float) -> float:
    return (e + math.pi) % (2 * math.pi) - math.pi


def sat(x: float) -> float:
    return max(min(x, 1.0), -1.0)


@dataclass
class SMCState:
    int_v: float = 0.0
    int_h: float = 0.0


def smc_controller(
    s_plant: np.ndarray,
    z_ref: np.ndarray,
    u_prev: np.ndarray,
    p: RoverParams,
    smc_state: SMCState,
) -> tuple[float, float, dict]:
    """Sliding mode controller with boundary layer."""
    theta = s_plant[2]
    dx, dy, dtheta = s_plant[3], s_plant[4], s_plant[5]
    v_fwd = dx * math.cos(theta) + dy * math.sin(theta)

    theta_ref = z_ref[2]
    v_ref = z_ref[3]
    omega_ref = z_ref[4]

    e_v = v_fwd - v_ref
    e_h = angle_wrap(theta - theta_ref)

    smc_state.int_v += e_v * p.dt_ctrl
    smc_state.int_h += e_h * p.dt_ctrl

    sigma_v = e_v + p.lambda_v * smc_state.int_v
    sigma_h = e_h + p.lambda_h * smc_state.int_h

    # Equivalent control (feedforward)
    eta = p.eta_0 if p.eta_0 > 0 else 0.252
    V_avg_eq = v_ref / (eta * p.r_drive * p.cos_alpha * p.omega_per_V) + p.V_offset

    if p.eta_diff_0 > 0:
        dV_eq = -omega_ref * p.d_drives / (p.eta_diff_0 * p.r_drive * p.cos_alpha * p.omega_per_V * 2)
    else:
        dV_eq = 0.0

    u_sw_v = -p.eta_smc_v * sat(sigma_v / p.phi_v)
    u_sw_h = -p.eta_smc_h * sat(sigma_h / p.phi_h)

    V_base = V_avg_eq + u_sw_v
    dV = dV_eq - u_sw_h

    V_L = max(min(V_base + dV, p.V_max), p.V_min)
    V_R = max(min(V_base - dV, p.V_max), p.V_min)

    info = {
        "smc_state": smc_state,
        "sigma_v": sigma_v,
        "sigma_h": sigma_h,
        "e_v": e_v,
        "e_h": e_h,
    }
    return V_L, V_R, info
