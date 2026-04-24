"""PID controller — direct port of results/pid_controller.m."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ..params import RoverParams


def angle_wrap(e: float) -> float:
    return (e + math.pi) % (2 * math.pi) - math.pi


@dataclass
class PIDState:
    int_v: float = 0.0
    int_h: float = 0.0
    prev_e_v: float = 0.0
    prev_e_h: float = 0.0


def pid_controller(
    s_plant: np.ndarray,
    z_ref: np.ndarray,
    u_prev: np.ndarray,
    p: RoverParams,
    pid_state: PIDState,
) -> tuple[float, float, dict]:
    """Two-loop PID: speed loop -> V_base, heading loop -> dV."""
    theta = s_plant[2]
    dx, dy = s_plant[3], s_plant[4]
    v_fwd = dx * math.cos(theta) + dy * math.sin(theta)

    theta_ref = z_ref[2]
    v_ref = z_ref[3]

    # Speed loop
    e_v = v_ref - v_fwd
    pid_state.int_v += e_v * p.dt_ctrl
    pid_state.int_v = max(min(pid_state.int_v, p.V_max / max(p.Ki_v, 0.01)),
                          -p.V_max / max(p.Ki_v, 0.01))
    de_v = (e_v - pid_state.prev_e_v) / p.dt_ctrl
    pid_state.prev_e_v = e_v

    V_base = p.Kp_v * e_v + p.Ki_v * pid_state.int_v + p.Kd_v * de_v

    # Speed feedforward
    eta = p.eta_0 if p.eta_0 > 0 else 0.252
    V_ff = v_ref / (eta * p.r_drive * p.cos_alpha * p.omega_per_V) + p.V_offset
    V_base += V_ff

    # Heading loop
    e_h = angle_wrap(theta_ref - theta)
    pid_state.int_h += e_h * p.dt_ctrl
    pid_state.int_h = max(min(pid_state.int_h, math.pi / max(p.Ki_h, 0.01)),
                          -math.pi / max(p.Ki_h, 0.01))
    de_h = (e_h - pid_state.prev_e_h) / p.dt_ctrl
    pid_state.prev_e_h = e_h

    dV = -(p.Kp_h * e_h + p.Ki_h * pid_state.int_h + p.Kd_h * de_h)

    # Heading feedforward
    if p.eta_diff_0 > 0:
        omega_ref = z_ref[4]
        dV_ff = -omega_ref * p.d_drives / (p.eta_diff_0 * p.r_drive * p.cos_alpha * p.omega_per_V * 2)
        dV += dV_ff

    V_L = max(min(V_base + dV, p.V_max), p.V_min)
    V_R = max(min(V_base - dV, p.V_max), p.V_min)

    info = {
        "pid_state": pid_state,
        "e_v": e_v,
        "e_h": e_h,
        "V_base": V_base,
        "dV": dV,
        "V_ff": V_ff,
    }
    return V_L, V_R, info
