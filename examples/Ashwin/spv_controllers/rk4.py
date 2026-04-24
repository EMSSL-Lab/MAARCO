"""RK4 integrator — direct port of results/run_controller_comparison.m rk4_step."""

from __future__ import annotations

import numpy as np

from .params import RoverParams
from .plant_model import plant_ode


def rk4_step(s: np.ndarray, V_L: float, V_R: float, dt: float, p: RoverParams) -> np.ndarray:
    """Single RK4 integration step using plant_ode."""
    k1 = plant_ode(0, s, V_L, V_R, p)
    k2 = plant_ode(0, s + dt / 2 * k1, V_L, V_R, p)
    k3 = plant_ode(0, s + dt / 2 * k2, V_L, V_R, p)
    k4 = plant_ode(0, s + dt * k3, V_L, V_R, p)
    return s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
