"""16-state plant ODE — direct port of scripts/plant_ode.m.

Propulsive force: weight-proportional Coulomb friction (F = mu_p * N_drive),
directed along the slip vector. Rate-independent granular friction model
identified from 617 experimental runs.

State vector (0-indexed):
  s[0:6]   = [x, y, theta, dx, dy, dtheta]     CM kinematics
  s[6:10]  = [phi_L, omega_L, phi_R, omega_R]   drive rotation
  s[10:12] = [psi, dpsi]                         pitch
  s[12:14] = [i_L, i_R]                          motor currents
  s[14:16] = [delta_L, delta_R]                  sinkage
"""

from __future__ import annotations

import math

import numpy as np

from .params import RoverParams


def plant_ode(t: float, s: np.ndarray, V_L: float, V_R: float, p: RoverParams) -> np.ndarray:
    """Compute ds/dt for the 16-state helical-drive rover plant."""
    # Unpack states
    theta = s[2]
    dx = s[3]
    dy = s[4]
    dtheta = s[5]
    omega_L = s[7]
    omega_R = s[9]
    psi = s[10]
    dpsi = s[11]
    i_L = s[12]
    i_R = s[13]
    delta_L = max(s[14], 0.0)
    delta_R = max(s[15], 0.0)

    # Motor electrical dynamics
    i_L_eff = max(min(i_L, p.i_max), -p.i_max)
    i_R_eff = max(min(i_R, p.i_max), -p.i_max)

    di_L = (V_L - p.R_motor * i_L - p.K_e * omega_L) / p.L_motor
    di_R = (V_R - p.R_motor * i_R - p.K_e * omega_R) / p.L_motor

    # Motor torques
    tau_L = p.K_t * i_L_eff
    tau_R = p.K_t * i_R_eff

    # Shorthand
    cth = math.cos(theta)
    sth = math.sin(theta)
    ca = p.cos_alpha
    sa = p.sin_alpha
    d2 = p.d_drives / 2
    rd = p.r_drive

    # Contact-point velocities (world frame)
    r_Lx = -d2 * sth
    r_Ly = d2 * cth
    r_Rx = d2 * sth
    r_Ry = -d2 * cth

    v_Lx = dx - dtheta * r_Ly
    v_Ly = dy + dtheta * r_Lx
    v_Rx = dx - dtheta * r_Ry
    v_Ry = dy + dtheta * r_Rx

    # Project into body frame
    v_L_fwd = v_Lx * cth + v_Ly * sth
    v_L_lat = -v_Lx * sth + v_Ly * cth
    v_R_fwd = v_Rx * cth + v_Ry * sth
    v_R_lat = -v_Rx * sth + v_Ry * cth

    # Blade surface velocities (body frame)
    v_blade_L = rd * omega_L
    v_blade_R = rd * omega_R

    v_bL_fwd = v_blade_L * ca
    v_bL_lat = v_blade_L * sa
    v_bR_fwd = v_blade_R * ca
    v_bR_lat = -v_blade_R * sa

    # Slip velocities
    slip_L_fwd = v_bL_fwd - v_L_fwd
    slip_L_lat = v_bL_lat - v_L_lat
    slip_R_fwd = v_bR_fwd - v_R_fwd
    slip_R_lat = v_bR_lat - v_R_lat

    # Propulsive forces (weight-proportional, rate-independent Coulomb)
    F_prop = p.mu_p * p.N_drive  # force magnitude per drive [N]

    slip_L_mag = math.sqrt(slip_L_fwd**2 + slip_L_lat**2 + p.eps_slip**2)
    slip_R_mag = math.sqrt(slip_R_fwd**2 + slip_R_lat**2 + p.eps_slip**2)

    F_slip_L_fwd = F_prop * slip_L_fwd / slip_L_mag
    F_slip_L_lat = F_prop * slip_L_lat / slip_L_mag
    F_slip_R_fwd = F_prop * slip_R_fwd / slip_R_mag
    F_slip_R_lat = F_prop * slip_R_lat / slip_R_mag

    # Residual drag (per drive, opposes contact velocity)
    speed_L = math.sqrt(v_L_fwd**2 + v_L_lat**2 + p.eps_fric**2)
    speed_R = math.sqrt(v_R_fwd**2 + v_R_lat**2 + p.eps_fric**2)

    F_drag_L_fwd = -(p.c_drag_fwd * v_L_fwd + p.F_coulomb * v_L_fwd / speed_L)
    F_drag_L_lat = -(p.c_drag_lat * v_L_lat + p.F_coulomb * v_L_lat / speed_L)
    F_drag_R_fwd = -(p.c_drag_fwd * v_R_fwd + p.F_coulomb * v_R_fwd / speed_R)
    F_drag_R_lat = -(p.c_drag_lat * v_R_lat + p.F_coulomb * v_R_lat / speed_R)

    # Total force per drive (body frame)
    F_L_fwd = F_slip_L_fwd + F_drag_L_fwd
    F_L_lat = F_slip_L_lat + F_drag_L_lat
    F_R_fwd = F_slip_R_fwd + F_drag_R_fwd
    F_R_lat = F_slip_R_lat + F_drag_R_lat

    # Transform to world frame
    Fpx_L = F_L_fwd * cth - F_L_lat * sth
    Fpy_L = F_L_fwd * sth + F_L_lat * cth
    Fpx_R = F_R_fwd * cth - F_R_lat * sth
    Fpy_R = F_R_fwd * sth + F_R_lat * cth

    # Translational dynamics
    ddx = (Fpx_L + Fpx_R) / p.m
    ddy = (Fpy_L + Fpy_R) / p.m

    # Yaw dynamics
    tau_L_yaw = r_Lx * Fpy_L - r_Ly * Fpx_L
    tau_R_yaw = r_Rx * Fpy_R - r_Ry * Fpx_R
    tau_yaw_damp = -p.c_yaw * dtheta
    ddtheta = (tau_L_yaw + tau_R_yaw + tau_yaw_damp) / p.I_yaw

    # Drive rotational dynamics
    tau_terrain_L = rd * (F_slip_L_fwd * ca + abs(F_slip_L_lat) * sa)
    tau_terrain_R = rd * (F_slip_R_fwd * ca + abs(F_slip_R_lat) * sa)

    sign_L = 1.0 if omega_L > 0 else (-1.0 if omega_L < 0 else 0.0)
    sign_R = 1.0 if omega_R > 0 else (-1.0 if omega_R < 0 else 0.0)

    domega_L = (tau_L - p.b_drag * omega_L - p.tau_c_drive * sign_L - tau_terrain_L) / p.I_drive
    domega_R = (tau_R - p.b_drag * omega_R - p.tau_c_drive * sign_R - tau_terrain_R) / p.I_drive

    # Pitch dynamics
    F_fwd_total = F_L_fwd + F_R_fwd
    tau_pitch = (F_fwd_total * p.h_cm * math.cos(psi)
                 - p.m * p.g * p.h_cm * math.sin(psi)
                 - p.k_pitch * psi
                 - p.c_pitch * dpsi)
    ddpsi = tau_pitch / p.I_pitch

    # Sinkage dynamics
    delta_eq_L = p.N_drive / (p.k_substrate * p.A_contact)
    delta_eq_R = p.N_drive / (p.k_substrate * p.A_contact)
    ddelta_L = (delta_eq_L - s[14]) / p.tau_sink
    ddelta_R = (delta_eq_R - s[15]) / p.tau_sink

    # Assemble
    ds = np.zeros(16)
    ds[0] = dx
    ds[1] = dy
    ds[2] = dtheta
    ds[3] = ddx
    ds[4] = ddy
    ds[5] = ddtheta
    ds[6] = omega_L
    ds[7] = domega_L
    ds[8] = omega_R
    ds[9] = domega_R
    ds[10] = dpsi
    ds[11] = ddpsi
    ds[12] = di_L
    ds[13] = di_R
    ds[14] = ddelta_L
    ds[15] = ddelta_R
    return ds
