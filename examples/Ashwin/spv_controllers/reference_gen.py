"""Reference trajectory generation — direct port of scripts/generate_reference.m."""

from __future__ import annotations

import math

import numpy as np

from .params import RoverParams


def _smooth_ramp(t: np.ndarray, delay: float) -> np.ndarray:
    """Smooth Hermite ramp from 0 to 1 over delay seconds."""
    s = np.clip(t / delay, 0, 1)
    return s ** 2 * (3 - 2 * s)


def generate_reference(
    traj_type: str,
    p: RoverParams,
    *,
    radius: float = 3.0,
    speed: float = 0.8,
    max_speed: float = float("inf"),
    start_delay: float = 3.0,
    amplitude: float = 3.0,
    period: float = 20.0,
) -> dict:
    """Generate a smooth reference trajectory.

    Returns dict with keys: t, x, y, theta, v, omega, Z (5xN).
    """
    dt_fine = 0.01
    t = np.arange(0, p.T_sim + dt_fine / 2, dt_fine)

    is_circle = False
    if traj_type in ("circle_right", "circle"):
        ref = _gen_circle_right(t, radius, speed)
        is_circle = True
    elif traj_type == "circle_left":
        ref = _gen_circle_left(t, radius, speed)
        is_circle = True
    elif traj_type == "figure8":
        ref = _gen_figure8(t, amplitude, period)
    elif traj_type == "straight":
        ref = _gen_straight(t, speed)
    else:
        raise ValueError(f"Unknown trajectory type: {traj_type}")

    ramp = _smooth_ramp(t, start_delay)

    if is_circle:
        if np.isfinite(max_speed):
            ramp = ramp * np.clip(max_speed / (np.abs(ref["v_full"]) + 1e-6), 0, 1)
        ref["v"] = ref["v_full"] * ramp
        ref["omega"] = ref["omega_full"] * ramp
        ref["x"] = np.cumsum(ref["v"] * np.cos(ref["theta"])) * dt_fine
        ref["y"] = np.cumsum(ref["v"] * np.sin(ref["theta"])) * dt_fine
        del ref["v_full"], ref["omega_full"]
    else:
        ref["v"] = ref["v"] * ramp
        ref["omega"] = ref["omega"] * ramp
        if np.isfinite(max_speed):
            scale = np.clip(max_speed / (ref["v"] + 1e-6), 0, 1)
            ref["v"] *= scale
            ref["omega"] *= scale
        ref["x"] = np.cumsum(ref["v"] * np.cos(ref["theta"])) * dt_fine
        ref["y"] = np.cumsum(ref["v"] * np.sin(ref["theta"])) * dt_fine
        dx_dt = ref["v"] * np.cos(ref["theta"])
        dy_dt = ref["v"] * np.sin(ref["theta"])
        ref["theta"] = np.unwrap(np.arctan2(dy_dt, dx_dt))

    ref["Z"] = np.vstack([ref["x"], ref["y"], ref["theta"], ref["v"], ref["omega"]])
    ref["t"] = t
    return ref


def _gen_circle_right(t: np.ndarray, R: float, v0: float) -> dict:
    w0 = v0 / R
    phi = w0 * t
    return {
        "x": R * np.sin(phi),
        "y": -R * (1 - np.cos(phi)),
        "theta": np.unwrap(-phi),
        "v_full": v0 * np.ones_like(t),
        "omega_full": -w0 * np.ones_like(t),
        "v": v0 * np.ones_like(t),
        "omega": -w0 * np.ones_like(t),
    }


def _gen_circle_left(t: np.ndarray, R: float, v0: float) -> dict:
    w0 = v0 / R
    phi = w0 * t
    return {
        "x": R * np.sin(phi),
        "y": R * (1 - np.cos(phi)),
        "theta": np.unwrap(phi),
        "v_full": v0 * np.ones_like(t),
        "omega_full": w0 * np.ones_like(t),
        "v": v0 * np.ones_like(t),
        "omega": w0 * np.ones_like(t),
    }


def _gen_figure8(t: np.ndarray, A: float, T_period: float) -> dict:
    w = 2 * math.pi / T_period
    dx_dt = A * w * np.cos(w * t)
    dy_dt = A * w * np.cos(2 * w * t)
    return {
        "x": A * np.sin(w * t),
        "y": (A / 2) * np.sin(2 * w * t),
        "theta": np.unwrap(np.arctan2(dy_dt, dx_dt)),
        "v": np.sqrt(dx_dt ** 2 + dy_dt ** 2),
        "omega": np.gradient(np.unwrap(np.arctan2(dy_dt, dx_dt)), t),
    }


def _gen_straight(t: np.ndarray, v0: float) -> dict:
    return {
        "x": v0 * t,
        "y": np.zeros_like(t),
        "theta": np.zeros_like(t),
        "v": v0 * np.ones_like(t),
        "omega": np.zeros_like(t),
    }
