"""Rover parameters — direct port of scripts/rover_params.m.

All physical and controller parameters for the helical-drive rover simulation.

16-State plant model:
  s = [x, y, theta, dx, dy, dtheta,       (0-5)
       phi_L, omega_L, phi_R, omega_R,     (6-9)
       psi, dpsi,                          (10-11)
       i_L, i_R,                           (12-13)
       delta_L, delta_R]                   (14-15)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class RoverParams:
    """All physical and controller parameters for the SPV simulation."""

    # ---- Mass properties (from CAD) ----
    m_chassis: float = 5.63
    m_drive: float = 1.01

    @property
    def m(self) -> float:
        return self.m_chassis + 2 * self.m_drive  # 7.65 kg

    g: float = 9.81

    # ---- Chassis geometry ----
    L_chassis: float = 0.2794
    W_chassis: float = 0.272
    H_chassis: float = 0.10

    # ---- Helical drive geometry ----
    r_inner: float = 0.04445
    r_outer: float = 0.04445
    l_drive: float = 0.2921
    d_drives: float = 2 * 0.136
    helix_angle: float = math.radians(11.8)
    n_blades: int = 1

    @property
    def r_drive(self) -> float:
        return self.r_outer

    # ---- Moments of inertia (from CAD) ----
    I_chassis_tensor: np.ndarray = field(default_factory=lambda: np.array([
        [0.07632, 0.00009, -0.00021],
        [0.00009, 0.08490, -0.00110],
        [-0.00021, -0.00110, 0.02689],
    ]))
    I_drive_tensor: np.ndarray = field(default_factory=lambda: np.array([
        [0.01245, 0.00000, -0.00001],
        [0.00000, 0.01245, -0.00002],
        [-0.00001, -0.00002, 0.00099],
    ]))
    I_rover_tensor: np.ndarray = field(default_factory=lambda: np.array([
        [0.12307, 0.00021, -0.00026],
        [0.00021, 0.13376, 0.00806],
        [-0.00026, 0.00806, 0.05297],
    ]))
    I_drive: float = 0.000987
    I_yaw: float = 0.052968
    I_pitch: float = 0.133755

    # ---- Propulsive force model (weight-proportional, rate-independent) ----
    mu_p: float = 0.370       # propulsive friction coefficient [-]
    eps_slip: float = 0.0005  # slip regularisation [m/s]

    # ---- Ground drag (body-sand resistance) ----
    c_drag_fwd: float = 0.14
    c_drag_lat: float = 0.21
    mu_coulomb: float = 0.356  # body-sand Coulomb friction [-] (from eta_0=0.253)
    eps_fric: float = 0.0005   # regularisation [m/s]

    @property
    def F_coulomb(self) -> float:
        return self.mu_coulomb * self.m * self.g / 2

    # ---- Sinkage dynamics ----
    k_substrate: float = 500000
    tau_sink: float = 0.5
    delta_max: float = 0.05

    @property
    def N_drive(self) -> float:
        return self.m * self.g / 2

    @property
    def A_contact(self) -> float:
        return self.l_drive * 2 * self.r_drive * 0.3

    # ---- Drive rotational drag (benchtop no-load) ----
    b_drag: float = 0.004578
    tau_c_drive: float = 0.30623

    # ---- Pitch model ----
    @property
    def h_cm(self) -> float:
        return self.r_outer + self.H_chassis / 2

    k_pitch: float = 80.0
    c_pitch: float = 12.0

    # ---- Yaw damping ----
    c_yaw: float = 0.50

    # ---- DC motor ----
    R_motor: float = 1.0
    L_motor: float = 0.005
    K_t: float = 0.3366
    K_e: float = 0.3366

    # ---- Actuator limits ----
    V_max: float = 11.1
    V_min: float = 0.0
    i_max: float = 20.0

    # ---- Derived constants ----
    @property
    def cos_alpha(self) -> float:
        return math.cos(self.helix_angle)

    @property
    def sin_alpha(self) -> float:
        return math.sin(self.helix_angle)

    @property
    def V_offset(self) -> float:
        return self.R_motor * self.tau_c_drive / self.K_t

    @property
    def K_denom(self) -> float:
        return self.R_motor * self.b_drag / self.K_t + self.K_e

    @property
    def omega_per_V(self) -> float:
        return 1.0 / self.K_denom

    @property
    def b_eff(self) -> float:
        return self.K_t * self.K_e / self.R_motor + self.b_drag

    @property
    def v_blade_per_V(self) -> float:
        return self.r_drive * self.omega_per_V

    # ---- Simulation timing ----
    dt_plant: float = 0.001
    dt_ctrl: float = 0.10
    T_sim: float = 150.0

    # ---- MPC parameters ----
    N_horizon: int = 20
    dt_pred: float = 0.50
    Q_mpc: np.ndarray = field(default_factory=lambda: np.diag([500, 500, 100, 5, 2]).astype(float))
    Q_term: np.ndarray = field(default_factory=lambda: np.diag([1500, 1500, 300, 10, 4]).astype(float))
    R_mpc: np.ndarray = field(default_factory=lambda: np.diag([0.0005, 0.0005]))
    dR_mpc: np.ndarray = field(default_factory=lambda: np.diag([0.05, 0.05]))

    # ---- PID gains ----
    Kp_v: float = 3.0
    Ki_v: float = 1.0
    Kd_v: float = 0.1
    Kp_h: float = 5.0
    Ki_h: float = 1.5
    Kd_h: float = 0.3

    # ---- LQR weights ----
    Q_lqr: np.ndarray = field(default_factory=lambda: np.diag([200, 200, 80, 5, 2]).astype(float))
    R_lqr: np.ndarray = field(default_factory=lambda: np.diag([0.01, 0.01]))

    # ---- SMC parameters ----
    lambda_v: float = 2.0
    lambda_h: float = 3.0
    eta_smc_v: float = 3.0
    eta_smc_h: float = 2.0
    phi_v: float = 0.02
    phi_h: float = 0.15

    # ---- Pure Pursuit ----
    L_d_min: float = 0.10
    L_d_gain: float = 2.0
    Kp_pp: float = 15.0
    Ki_pp: float = 3.0

    # ---- Propulsive efficiency (from papers) ----
    eta_0: float = 0.252
    eta_diff_0: float = 0.200
    eta_diff_km: float = 0.004

    # ---- Noise ----
    process_noise_std: np.ndarray = field(default_factory=lambda: np.zeros(16))
    sensor_noise_std: np.ndarray = field(default_factory=lambda: np.array([0.005, 0.005, 0.002, 0.01, 0.01]))


def rover_params() -> RoverParams:
    """Return default rover parameters (matches scripts/rover_params.m)."""
    return RoverParams()


def controller_comparison_params() -> RoverParams:
    """Return params with overrides from results/run_controller_comparison.m lines 24-40."""
    p = RoverParams()
    p.eta_0 = 0.252
    p.eta_diff_0 = 0.200
    p.eta_diff_km = 0.004
    p.mu_coulomb = 0.0
    p.c_drag_fwd = 22.0
    p.c_drag_lat = 33.0
    p.c_yaw = 2.0
    return p
