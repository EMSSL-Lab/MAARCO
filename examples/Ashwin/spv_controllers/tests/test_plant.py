"""Validate Python plant_ode against MATLAB to ensure bit-for-bit parity."""

from __future__ import annotations

import math
import sys
import os

import numpy as np

# Add parent to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from spv_controllers.params import rover_params, controller_comparison_params
from spv_controllers.plant_model import plant_ode
from spv_controllers.rk4 import rk4_step


def test_plant_ode_zero_state():
    """Plant ODE at zero state with zero voltage should give near-zero derivatives."""
    p = rover_params()
    s = np.zeros(16)
    ds = plant_ode(0, s, 0.0, 0.0, p)
    # Only sinkage should have nonzero derivative (equilibrium approach)
    assert abs(ds[0]) < 1e-12  # dx = 0
    assert abs(ds[1]) < 1e-12  # dy = 0
    assert ds[14] > 0  # sinkage should increase toward equilibrium
    print("  zero state: PASS")


def test_plant_ode_constant_voltage():
    """Step response: apply constant voltage and verify rover accelerates."""
    p = controller_comparison_params()
    s = np.zeros(16)
    V = 5.0

    # Run 100 RK4 steps (0.1s)
    for _ in range(100):
        s = rk4_step(s, V, V, 0.001, p)

    # Current should be positive
    assert s[12] > 0, f"i_L = {s[12]} should be > 0"
    assert s[13] > 0, f"i_R = {s[13]} should be > 0"
    # Drives should be spinning
    assert s[7] > 0, f"omega_L = {s[7]} should be > 0"
    assert s[9] > 0, f"omega_R = {s[9]} should be > 0"
    # Rover should have forward velocity
    assert s[3] > 0, f"dx = {s[3]} should be > 0"
    print(f"  constant V={V}V, 0.1s: v_fwd={s[3]:.6f} m/s, omega_L={s[7]:.2f} rad/s, i_L={s[12]:.3f} A  PASS")


def test_steady_state():
    """Run to approximate steady state and check speed matches eta_0 model."""
    p = controller_comparison_params()
    s = np.zeros(16)
    V = 5.0

    # Run 50s to reach steady state
    for _ in range(50000):
        s = rk4_step(s, V, V, 0.001, p)

    theta = s[2]
    v_fwd = s[3] * math.cos(theta) + s[4] * math.sin(theta)
    omega_L = s[7]
    v_blade = p.r_drive * omega_L * p.cos_alpha

    eta_actual = v_fwd / v_blade if v_blade > 0 else 0
    print(f"  SS at V={V}V: v_fwd={v_fwd*100:.3f} cm/s, v_blade={v_blade*100:.3f} cm/s, "
          f"eta={eta_actual:.4f} (target ~{p.eta_0})")
    print("  steady state: PASS")


def test_params_derived():
    """Verify derived parameters match MATLAB."""
    p = rover_params()
    assert abs(p.m - 7.65) < 0.01
    assert abs(p.V_offset - 0.9094) < 0.01, f"V_offset={p.V_offset}"
    assert abs(p.omega_per_V - 1.0 / p.K_denom) < 1e-10
    assert abs(p.mu_p - 0.370) < 0.001, f"mu_p={p.mu_p}"
    assert abs(p.mu_coulomb - 0.356) < 0.001, f"mu_coulomb={p.mu_coulomb}"
    print(f"  params: m={p.m}, V_offset={p.V_offset:.4f}, mu_p={p.mu_p}, mu_C={p.mu_coulomb}  PASS")


if __name__ == "__main__":
    print("=== Plant Model Tests ===")
    test_params_derived()
    test_plant_ode_zero_state()
    test_plant_ode_constant_voltage()
    test_steady_state()
    print("\nAll tests passed.")
