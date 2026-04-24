"""Controller comparison simulation — port of results/run_controller_comparison.m."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np

from ..params import RoverParams, controller_comparison_params
from ..reference_gen import generate_reference
from ..rk4 import rk4_step
from ..controllers.pid import PIDState, pid_controller
from ..controllers.lqr import lqr_controller
from ..controllers.smc import SMCState, smc_controller
from ..controllers.purepursuit import PurePursuitState, purepursuit_controller
from ..controllers.mpc import mpc_controller


def angle_wrap(e: float) -> float:
    return (e + math.pi) % (2 * math.pi) - math.pi


def get_ref_at_time(t: float, ref: dict) -> np.ndarray:
    """Interpolate reference at time t."""
    idx = np.searchsorted(ref["t"], t, side="right") - 1
    idx = max(0, min(idx, len(ref["t"]) - 1))
    return np.array([ref["x"][idx], ref["y"][idx], ref["theta"][idx], ref["v"][idx], ref["omega"][idx]])


def get_ref_window(t: float, ref: dict, p: RoverParams) -> np.ndarray:
    """Extract N-step prediction window for MPC."""
    N = p.N_horizon
    z_win = np.zeros((5, N))
    for k in range(N):
        t_k = t + (k + 1) * p.dt_pred
        idx = np.searchsorted(ref["t"], t_k, side="right") - 1
        idx = max(0, min(idx, len(ref["t"]) - 1))
        z_win[:, k] = [ref["x"][idx], ref["y"][idx], ref["theta"][idx], ref["v"][idx], ref["omega"][idx]]
    return z_win


@dataclass
class SimMetrics:
    rms_pos: float = 0.0
    rms_head: float = 0.0
    max_pos: float = 0.0
    mean_comp: float = 0.0
    max_comp: float = 0.0
    energy: float = 0.0
    n_sat: int = 0


def run_single(ctrl_name: str, ref: dict, p: RoverParams) -> dict:
    """Run one controller on one trajectory. Returns logs and metrics."""
    N_steps = round(p.T_sim / p.dt_plant)
    N_ctrl = round(p.T_sim / p.dt_ctrl)
    ctrl_every = round(p.dt_ctrl / p.dt_plant)

    t_log = np.zeros(N_steps + 1)
    s_log = np.zeros((N_steps + 1, 16))
    u_log = np.zeros((N_ctrl, 2))
    e_pos_log = np.zeros(N_ctrl)
    e_head_log = np.zeros(N_ctrl)
    comp_time = np.zeros(N_ctrl)

    s = np.zeros(16)
    s_log[0, :] = s
    u = np.array([p.V_offset + 0.5, p.V_offset + 0.5])

    # Controller states
    pid_state = PIDState()
    smc_state = SMCState()
    pp_state = PurePursuitState()

    ctrl_idx = -1

    for k in range(1, N_steps + 1):
        t = (k - 1) * p.dt_plant

        if (k - 1) % ctrl_every == 0:
            ctrl_idx += 1
            z_ref = get_ref_at_time(t, ref)

            t0 = time.perf_counter()
            if ctrl_name == "PID":
                V_L, V_R, info = pid_controller(s, z_ref, u, p, pid_state)
                pid_state = info["pid_state"]
            elif ctrl_name == "LQR":
                V_L, V_R, info = lqr_controller(s, z_ref, u, p)
            elif ctrl_name == "SMC":
                V_L, V_R, info = smc_controller(s, z_ref, u, p, smc_state)
                smc_state = info["smc_state"]
            elif ctrl_name == "PurePursuit":
                z_ref_full = ref["Z"]
                V_L, V_R, info = purepursuit_controller(s, z_ref_full, u, p, pp_state)
                pp_state = info["pp_state"]
            elif ctrl_name == "MPC":
                z_win = get_ref_window(t, ref, p)
                V_L, V_R, info = mpc_controller(s, z_win, u, p)
            else:
                raise ValueError(f"Unknown controller: {ctrl_name}")

            comp_time[ctrl_idx] = (time.perf_counter() - t0) * 1000  # ms
            u = np.array([V_L, V_R])

            if ctrl_idx < N_ctrl:
                u_log[ctrl_idx, :] = u

            theta_curr = s[2]
            pos_err = math.sqrt((s[0] - z_ref[0]) ** 2 + (s[1] - z_ref[1]) ** 2)
            head_err = abs(angle_wrap(theta_curr - z_ref[2]))
            if ctrl_idx < N_ctrl:
                e_pos_log[ctrl_idx] = pos_err
                e_head_log[ctrl_idx] = head_err * 180 / math.pi

        # Plant integration
        s = rk4_step(s, u[0], u[1], p.dt_plant, p)
        t_log[k] = t + p.dt_plant
        s_log[k, :] = s

    # Metrics (skip first 20s for settling)
    n_valid = ctrl_idx + 1
    settle_idx = round(20 / p.dt_ctrl)
    valid = slice(settle_idx, n_valid)

    e_pos_v = e_pos_log[valid]
    e_head_v = e_head_log[valid]
    u_v = u_log[:n_valid, :]

    metrics = SimMetrics(
        rms_pos=float(np.sqrt(np.mean(e_pos_v ** 2)) * 100),
        rms_head=float(np.sqrt(np.mean(e_head_v ** 2))),
        max_pos=float(np.max(e_pos_v) * 100),
        mean_comp=float(np.mean(comp_time[:n_valid])),
        max_comp=float(np.max(comp_time[:n_valid])),
        energy=float(np.sum(u_v) * p.dt_ctrl),
        n_sat=int(np.sum(u_v[:, 0] >= p.V_max - 0.01) + np.sum(u_v[:, 1] >= p.V_max - 0.01)
                   + np.sum(u_v[:, 0] <= p.V_min + 0.01) + np.sum(u_v[:, 1] <= p.V_min + 0.01)),
    )

    return {
        "t": t_log,
        "s": s_log,
        "u": u_log[:n_valid],
        "e_pos": e_pos_log[:n_valid],
        "e_head": e_head_log[:n_valid],
        "comp_time": comp_time[:n_valid],
        "metrics": metrics,
    }


def run_comparison(p: RoverParams | None = None) -> dict:
    """Run all 5 controllers on 3 trajectories. Returns nested results dict."""
    if p is None:
        p = controller_comparison_params()

    traj_configs = {
        "circle": {"traj_type": "circle_right", "radius": 1.2, "speed": 0.028, "start_delay": 5},
        "straight": {"traj_type": "straight", "speed": 0.05, "start_delay": 3},
        "scurve": {"traj_type": "figure8", "speed": 0.028, "amplitude": 0.5, "period": 60, "start_delay": 5},
    }

    ctrl_names = ["PID", "LQR", "SMC", "PurePursuit", "MPC"]
    refs = {}

    print("Generating reference trajectories...")
    for tname, cfg in traj_configs.items():
        refs[tname] = generate_reference(cfg.pop("traj_type"), p, **cfg)

    results = {}
    for tname in traj_configs:
        results[tname] = {}
        print(f"\n=== Trajectory: {tname} ===")
        for cname in ctrl_names:
            print(f"  Running {cname}...", end=" ", flush=True)
            res = run_single(cname, refs[tname], p)
            m = res["metrics"]
            print(f"RMS_pos={m.rms_pos:.2f}cm, RMS_head={m.rms_head:.2f}deg, comp={m.mean_comp:.2f}ms")
            results[tname][cname] = res

    # Summary table
    print("\n\n=== SUMMARY TABLE ===")
    print(f"{'Trajectory':<12} {'Controller':<12} {'RMS_pos':>8} {'RMS_hd':>8} {'Max_pos':>8} {'Comp_ms':>8} {'Energy':>8}")
    print("-" * 75)
    for tname in traj_configs:
        for cname in ctrl_names:
            m = results[tname][cname]["metrics"]
            print(f"{tname:<12} {cname:<12} {m.rms_pos:7.2f} {m.rms_head:7.2f} "
                  f"{m.max_pos:7.2f} {m.mean_comp:7.2f} {m.energy:7.1f}")

    results["ctrl_names"] = ctrl_names
    results["traj_names"] = list(traj_configs.keys())
    results["refs"] = refs
    results["params"] = p
    return results


if __name__ == "__main__":
    run_comparison()
