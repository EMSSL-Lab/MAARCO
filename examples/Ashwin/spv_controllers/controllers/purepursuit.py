"""Pure pursuit controller — direct port of results/purepursuit_controller.m."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..params import RoverParams


def angle_wrap(e: float) -> float:
    return (e + math.pi) % (2 * math.pi) - math.pi


@dataclass
class PurePursuitState:
    int_v: float = 0.0
    last_idx: int = 0


def purepursuit_controller(
    s_plant: np.ndarray,
    z_ref_full: np.ndarray,
    u_prev: np.ndarray,
    p: RoverParams,
    pp_state: PurePursuitState,
) -> tuple[float, float, dict]:
    """Pure pursuit path following + speed PI.

    z_ref_full: (5, N) reference trajectory or (5,) single point.
    """
    x = s_plant[0]
    y_pos = s_plant[1]
    theta = s_plant[2]
    dx, dy = s_plant[3], s_plant[4]
    v_fwd = dx * math.cos(theta) + dy * math.sin(theta)

    # Adaptive lookahead distance
    L_d = max(p.L_d_min, p.L_d_gain * abs(v_fwd))

    if z_ref_full.ndim == 2 and z_ref_full.shape[1] > 1:
        ref_x = z_ref_full[0, :]
        ref_y = z_ref_full[1, :]
        N_ref = len(ref_x)

        # Find nearest point (search forward from last index)
        start_idx = max(pp_state.last_idx, 0)
        search_end = min(start_idx + 500, N_ref)
        search_range = np.arange(start_idx, search_end)
        if len(search_range) == 0:
            search_range = np.array([min(start_idx, N_ref - 1)])
        dist_search = np.sqrt((ref_x[search_range] - x) ** 2 + (ref_y[search_range] - y_pos) ** 2)
        rel_idx = np.argmin(dist_search)
        nearest_idx = int(search_range[rel_idx])
        nearest_idx = max(nearest_idx, pp_state.last_idx)
        pp_state.last_idx = nearest_idx

        # Lookahead by arc length
        arc_len = 0.0
        la_idx = nearest_idx
        for k in range(nearest_idx, N_ref - 1):
            arc_len += math.sqrt((ref_x[k + 1] - ref_x[k]) ** 2 + (ref_y[k + 1] - ref_y[k]) ** 2)
            if arc_len >= L_d:
                la_idx = k + 1
                break
        if la_idx <= nearest_idx:
            la_idx = min(nearest_idx + 10, N_ref - 1)

        la_x = ref_x[la_idx]
        la_y = ref_y[la_idx]
        v_ref = z_ref_full[3, min(la_idx, N_ref - 1)]
    else:
        z = z_ref_full.ravel()
        la_x = z[0]
        la_y = z[1]
        v_ref = z[3]

    # Steering
    dx_la = la_x - x
    dy_la = la_y - y_pos
    angle_to_la = math.atan2(dy_la, dx_la)
    alpha = angle_wrap(angle_to_la - theta)

    dist_to_la = math.sqrt(dx_la ** 2 + dy_la ** 2)
    kappa = 2 * math.sin(alpha) / dist_to_la if dist_to_la > 0.01 else 0.0
    omega_cmd = kappa * max(v_fwd, 0.01)

    # Curvature to differential voltage
    eta_d = p.eta_diff_0 if p.eta_diff_0 > 0 else 0.20
    dV = -omega_cmd * p.d_drives / (eta_d * p.r_drive * p.cos_alpha * p.omega_per_V * 2)

    # Speed PI loop
    e_v = v_ref - v_fwd
    pp_state.int_v += e_v * p.dt_ctrl
    pp_state.int_v = max(min(pp_state.int_v, p.V_max / max(p.Ki_pp, 0.01)),
                         -p.V_max / max(p.Ki_pp, 0.01))

    V_base = p.Kp_pp * e_v + p.Ki_pp * pp_state.int_v

    eta = p.eta_0 if p.eta_0 > 0 else 0.252
    V_ff = v_ref / (eta * p.r_drive * p.cos_alpha * p.omega_per_V) + p.V_offset
    V_base += V_ff

    V_L = max(min(V_base + dV, p.V_max), p.V_min)
    V_R = max(min(V_base - dV, p.V_max), p.V_min)

    info = {
        "pp_state": pp_state,
        "kappa": kappa,
        "alpha": alpha,
        "L_d": L_d,
        "la_idx": la_idx if z_ref_full.ndim == 2 else 0,
        "e_v": e_v,
        "omega_cmd": omega_cmd,
    }
    return V_L, V_R, info
