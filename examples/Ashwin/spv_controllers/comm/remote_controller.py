"""Remote controller for the SPV rover.

Receives sensor telemetry from Raspberry Pi via UDP, runs SPV controllers
on the laptop, converts voltage commands to PWM, and sends motor commands
back to the Pi.

Architecture:
    [Pi: main_remote.rs] --UDP:5008--> [Laptop: this script] --UDP:5007--> [Pi]
         sensor bridge                  SPV controllers                  motor PWM

Supported controllers:
    PID          -- Two-loop PID (speed + heading) with feedforward
    SMC          -- Sliding mode with boundary layer
    Pure Pursuit -- Geometric path following + speed PI
    LQR          -- Time-varying linear quadratic regulator
    MPC          -- Model predictive control (requires osqp)

Usage:
    python -m spv_controllers.comm.remote_controller --controller pid --speed 0.03
    python remote_controller.py --controller smc --pi-ip 172.20.10.6

Keys (in GUI window):
    Click       -- Add waypoint
    1           -- Start mission
    2           -- Clear waypoints
    3           -- Emergency stop
    4           -- Set GPS origin + calibrate heading
    5           -- Switch to PID controller
    6           -- Switch to SMC controller
    7           -- Switch to Pure Pursuit controller
    8           -- Switch to LQR controller
    9           -- Switch to MPC controller
    +/-         -- Increase/decrease cruise speed
"""

from __future__ import annotations

import argparse
import math
import socket
import sys
import os
import time
import turtle
from dataclasses import dataclass, field
from typing import Optional, List, Tuple

import numpy as np

# ── Path setup for standalone execution ───────────────────────────────────
# Add the parent of spv_controllers to sys.path so imports work when
# running this file directly (python remote_controller.py).
_this_dir = os.path.dirname(os.path.abspath(__file__))
_pkg_root = os.path.dirname(os.path.dirname(_this_dir))
if _pkg_root not in sys.path:
    sys.path.insert(0, _pkg_root)

from spv_controllers.params import RoverParams, rover_params
from spv_controllers.controllers.pid import pid_controller, PIDState
from spv_controllers.controllers.smc import smc_controller, SMCState
from spv_controllers.controllers.purepursuit import purepursuit_controller, PurePursuitState
from spv_controllers.controllers.lqr import lqr_controller
from spv_controllers.controllers.mpc import mpc_controller


# ══════════════════════════════════════════════════════════════════════════
#  TELEMETRY PARSING
# ══════════════════════════════════════════════════════════════════════════

@dataclass
class TelemetryPacket:
    """Parsed telemetry from Pi's main_remote.rs.

    Field indices (after splitting by comma, [0] = 'TEL'):
        [1]  gps_x          meters east from origin
        [2]  gps_y          meters north from origin
        [3]  heading_deg    corrected yaw (degrees)
        [4]  gps_speed_kmh  GPS ground speed
        [5]  gps_lat        decimal degrees
        [6]  gps_lon        decimal degrees
        [7]  fix_quality    NMEA fix code ("0"-"8" or "N/A")
        [8]  euler_x        raw BNO055 X (degrees)
        [9]  euler_y        raw BNO055 Y (degrees)
        [10] euler_z        raw BNO055 Z (degrees)
        [11] acc_lin_x      m/s^2
        [12] acc_lin_y      m/s^2
        [13] acc_lin_z      m/s^2
        [14] motor_current_L  amperes
        [15] motor_current_R  amperes
        [16] rpm_left
        [17] rpm_right
        [18] rotations_left   cumulative
        [19] rotations_right  cumulative
        [20] voltage_left   volts
        [21] voltage_right  volts
        [22] current_left_mA
        [23] current_right_mA
        [24] sonar_mm
        [25] tof_mm
    """
    recv_time: float = 0.0
    gps_x: float = 0.0
    gps_y: float = 0.0
    heading_deg: float = 0.0
    gps_speed_kmh: float = 0.0
    gps_lat: float = 0.0
    gps_lon: float = 0.0
    fix_quality: str = "N/A"
    euler_x: float = float('nan')
    euler_y: float = float('nan')
    euler_z: float = float('nan')
    acc_lin_x: float = float('nan')
    acc_lin_y: float = float('nan')
    acc_lin_z: float = float('nan')
    motor_current_L: float = float('nan')
    motor_current_R: float = float('nan')
    rpm_left: float = float('nan')
    rpm_right: float = float('nan')
    rotations_left: float = float('nan')
    rotations_right: float = float('nan')
    voltage_left: float = float('nan')
    voltage_right: float = float('nan')
    current_left_mA: float = float('nan')
    current_right_mA: float = float('nan')
    sonar_mm: float = float('nan')
    tof_mm: float = float('nan')


def _safe_float(s: str) -> float:
    """Parse string to float, returning NaN for 'nan' or invalid values."""
    try:
        return float(s)
    except (ValueError, TypeError):
        return float('nan')


def parse_telemetry(msg: str) -> Optional[TelemetryPacket]:
    """Parse a TEL,... message into a TelemetryPacket.

    Returns None if the message is malformed or not a TEL packet.
    """
    parts = msg.strip().split(',')
    if len(parts) < 26 or parts[0] != "TEL":
        return None

    pkt = TelemetryPacket()
    pkt.recv_time = time.time()
    pkt.gps_x = _safe_float(parts[1])
    pkt.gps_y = _safe_float(parts[2])
    pkt.heading_deg = _safe_float(parts[3])
    pkt.gps_speed_kmh = _safe_float(parts[4])
    pkt.gps_lat = _safe_float(parts[5])
    pkt.gps_lon = _safe_float(parts[6])
    pkt.fix_quality = parts[7]
    pkt.euler_x = _safe_float(parts[8])
    pkt.euler_y = _safe_float(parts[9])
    pkt.euler_z = _safe_float(parts[10])
    pkt.acc_lin_x = _safe_float(parts[11])
    pkt.acc_lin_y = _safe_float(parts[12])
    pkt.acc_lin_z = _safe_float(parts[13])
    pkt.motor_current_L = _safe_float(parts[14])
    pkt.motor_current_R = _safe_float(parts[15])
    pkt.rpm_left = _safe_float(parts[16])
    pkt.rpm_right = _safe_float(parts[17])
    pkt.rotations_left = _safe_float(parts[18])
    pkt.rotations_right = _safe_float(parts[19])
    pkt.voltage_left = _safe_float(parts[20])
    pkt.voltage_right = _safe_float(parts[21])
    pkt.current_left_mA = _safe_float(parts[22])
    pkt.current_right_mA = _safe_float(parts[23])
    pkt.sonar_mm = _safe_float(parts[24])
    pkt.tof_mm = _safe_float(parts[25])
    return pkt


# ══════════════════════════════════════════════════════════════════════════
#  STATE ESTIMATION
# ══════════════════════════════════════════════════════════════════════════

def _nan_or(val: float, default: float = 0.0) -> float:
    """Return val if finite, else default."""
    return val if math.isfinite(val) else default


def telemetry_to_state(
    pkt: TelemetryPacket,
    prev_theta: float,
    dt: float,
    p: RoverParams,
) -> np.ndarray:
    """Convert a telemetry packet to the 16-state plant vector.

    The controllers only use states [0:6] (position + velocity) plus
    optionally [7,9] (drive angular velocity) and [12,13] (motor current).
    States [6,8] (drive angle) and [14,15] (sinkage) are not measured
    and are left at zero.

    Velocity is estimated from GPS ground speed + IMU heading. When
    GPS speed is very low, we fall back to a wheel-odometry estimate
    using RPM and the eta_0 kinematic invariant.
    """
    s = np.zeros(16)

    # -- Position and heading --
    theta = math.radians(pkt.heading_deg)
    s[0] = pkt.gps_x
    s[1] = pkt.gps_y
    s[2] = theta

    # -- Forward velocity --
    # GPS-based (best when moving)
    v_gps = pkt.gps_speed_kmh / 3.6  # km/h -> m/s

    # RPM-based (fallback for low GPS speed)
    rpm_l = _nan_or(pkt.rpm_left)
    rpm_r = _nan_or(pkt.rpm_right)
    omega_avg = (rpm_l + rpm_r) / 2.0 * (2.0 * math.pi / 60.0)
    v_wheel = p.eta_0 * p.r_drive * omega_avg * p.cos_alpha

    # Use GPS if speed > 0.1 km/h, otherwise use wheel-based estimate
    v_fwd = v_gps if v_gps > 0.03 else v_wheel

    s[3] = v_fwd * math.cos(theta)  # dx
    s[4] = v_fwd * math.sin(theta)  # dy

    # -- Yaw rate (numerical derivative of heading) --
    dtheta = theta - prev_theta
    # Wrap to [-pi, pi]
    if dtheta > math.pi:
        dtheta -= 2.0 * math.pi
    elif dtheta < -math.pi:
        dtheta += 2.0 * math.pi
    s[5] = dtheta / dt if dt > 0 else 0.0

    # -- Drive angular velocities from RPM --
    s[7] = rpm_l * (2.0 * math.pi / 60.0)  # omega_L
    s[9] = rpm_r * (2.0 * math.pi / 60.0)  # omega_R

    # -- Pitch from euler_y (if available) --
    s[10] = math.radians(_nan_or(pkt.euler_y))

    # -- Motor currents --
    s[12] = _nan_or(pkt.motor_current_L)  # i_L (A)
    s[13] = _nan_or(pkt.motor_current_R)  # i_R (A)

    return s


# ══════════════════════════════════════════════════════════════════════════
#  VOLTAGE <-> PWM CONVERSION
# ══════════════════════════════════════════════════════════════════════════

def voltage_to_pwm(V: float, V_max: float = 11.1) -> int:
    """Convert motor voltage [0, V_max] to PWM pulse width [1500, 2000] us.

    PWM convention:
        1500 us = stopped (0V)
        2000 us = full forward (V_max = 11.1V battery)
        1000 us = full reverse (not typically used for SPV)

    Only forward voltages are mapped; negative voltages are clamped to 0.
    """
    V = max(0.0, min(V, V_max))
    return int(round(1500 + (V / V_max) * 500))


def pwm_to_voltage(pwm: int, V_max: float = 11.1) -> float:
    """Convert PWM pulse width [1500, 2000] us to motor voltage [0, V_max]."""
    pwm = max(1500, min(pwm, 2000))
    return (pwm - 1500) / 500.0 * V_max


# ══════════════════════════════════════════════════════════════════════════
#  WAYPOINT -> REFERENCE CONVERSION
# ══════════════════════════════════════════════════════════════════════════

def waypoint_to_reference(
    x: float, y: float, theta: float,
    wp_x: float, wp_y: float,
    cruise_speed: float,
) -> np.ndarray:
    """Compute a 5-state reference [x, y, theta, v, omega] aimed at a waypoint.

    For PID/SMC/LQR controllers that take a single reference point.
    """
    dx = wp_x - x
    dy = wp_y - y
    theta_ref = math.atan2(dy, dx)
    return np.array([wp_x, wp_y, theta_ref, cruise_speed, 0.0])


def waypoint_to_horizon(
    x: float, y: float, theta: float,
    wp_x: float, wp_y: float,
    cruise_speed: float,
    N: int,
    dt_pred: float,
) -> np.ndarray:
    """Generate a (5, N) reference window for MPC prediction horizon.

    Projects a straight-line trajectory from current position toward
    the waypoint at the cruise speed. Stops at the waypoint.
    """
    theta_ref = math.atan2(wp_y - y, wp_x - x)
    dist_to_wp = math.sqrt((wp_x - x) ** 2 + (wp_y - y) ** 2)

    Z = np.zeros((5, N))
    for k in range(N):
        t = (k + 1) * dt_pred
        d = min(cruise_speed * t, dist_to_wp)
        Z[0, k] = x + d * math.cos(theta_ref)
        Z[1, k] = y + d * math.sin(theta_ref)
        Z[2, k] = theta_ref
        Z[3, k] = cruise_speed if d < dist_to_wp else 0.0
        Z[4, k] = 0.0
    return Z


def waypoints_to_trajectory(
    waypoints: List[Tuple[float, float]],
    speed: float = 0.03,
    spacing: float = 0.02,
) -> np.ndarray:
    """Interpolate a dense (5, N) reference trajectory through waypoints.

    For the Pure Pursuit controller which needs a full path.

    Parameters
    ----------
    waypoints : list of (x, y) tuples in meters
    speed : desired cruise speed (m/s)
    spacing : interpolation spacing (meters)

    Returns
    -------
    Z : np.ndarray of shape (5, N)
        [x, y, theta, v, omega] at each point along the path.
    """
    if len(waypoints) == 0:
        return np.zeros((5, 1))

    if len(waypoints) == 1:
        return np.array([[waypoints[0][0]], [waypoints[0][1]], [0.0], [speed], [0.0]])

    # Interpolate a dense path between consecutive waypoints
    xs, ys = [waypoints[0][0]], [waypoints[0][1]]
    for i in range(len(waypoints) - 1):
        x0, y0 = waypoints[i]
        x1, y1 = waypoints[i + 1]
        seg_dist = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
        n_pts = max(int(seg_dist / spacing), 1)
        for j in range(1, n_pts + 1):
            t = j / n_pts
            xs.append(x0 + t * (x1 - x0))
            ys.append(y0 + t * (y1 - y0))

    xs = np.array(xs)
    ys = np.array(ys)
    N = len(xs)

    # Compute heading at each point
    thetas = np.zeros(N)
    for i in range(N - 1):
        thetas[i] = math.atan2(ys[i + 1] - ys[i], xs[i + 1] - xs[i])
    thetas[-1] = thetas[-2] if N > 1 else 0.0

    # Speed: cruise everywhere, decelerate at the end
    vs = np.full(N, speed)
    vs[-1] = 0.0  # Stop at final waypoint

    # Angular velocity (omega = dtheta/dt = dtheta/ds * v)
    omegas = np.zeros(N)
    for i in range(1, N):
        dth = thetas[i] - thetas[i - 1]
        if dth > math.pi:
            dth -= 2 * math.pi
        if dth < -math.pi:
            dth += 2 * math.pi
        ds = math.sqrt((xs[i] - xs[i - 1]) ** 2 + (ys[i] - ys[i - 1]) ** 2)
        omegas[i] = dth * speed / ds if ds > 1e-6 else 0.0

    return np.vstack([xs, ys, thetas, vs, omegas])


# ══════════════════════════════════════════════════════════════════════════
#  REMOTE CONTROLLER GUI + CONTROL LOOP
# ══════════════════════════════════════════════════════════════════════════

class RemoteController:
    """Turtle-based ground control station with integrated SPV controllers.

    Receives telemetry from the Pi, runs the selected controller, and sends
    motor commands back. Uses Turtle graphics for waypoint visualization.
    """

    # Controller display names
    CONTROLLERS = {
        "pid": "PID (speed + heading)",
        "smc": "SMC (sliding mode)",
        "purepursuit": "Pure Pursuit (geometric)",
        "lqr": "LQR (linear quadratic)",
        "mpc": "MPC (model predictive)",
    }

    def __init__(self, args: argparse.Namespace):
        # ── Network ───────────────────────────────────────────────────
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.bind(("0.0.0.0", args.listen_port))
        self.recv_sock.setblocking(False)

        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pi_addr = (args.pi_ip, args.send_port)

        # ── Rover parameters ──────────────────────────────────────────
        self.p = rover_params()
        self.p.dt_ctrl = 0.10      # 10 Hz control
        self.p.V_max = 11.1
        self.p.V_min = 0.0

        # ── Controller state ──────────────────────────────────────────
        self.controller_name = args.controller
        self.pid_state = PIDState()
        self.smc_state = SMCState()
        self.pp_state = PurePursuitState()

        # ── Estimation state ──────────────────────────────────────────
        self.prev_theta = 0.0
        self.u_prev = np.array([0.0, 0.0])
        self.latest_telem: Optional[TelemetryPacket] = None

        # ── Mission state ─────────────────────────────────────────────
        self.waypoints: List[Tuple[float, float]] = []
        self.current_wp_idx = 0
        self.is_running = False
        self.cruise_speed = args.speed
        self.wp_threshold = 0.10  # meters — switch to next WP when this close

        # ── Telemetry counters ────────────────────────────────────────
        self.pkt_count = 0
        self.last_pkt_time = 0.0
        self.ctrl_count = 0

        # ── GUI ───────────────────────────────────────────────────────
        self._setup_gui()

        # ── Start control loop ────────────────────────────────────────
        print(f"Controller: {self.CONTROLLERS.get(self.controller_name, self.controller_name)}")
        print(f"Cruise speed: {self.cruise_speed*100:.1f} cm/s")
        print(f"Listening on UDP port {args.listen_port}")
        print(f"Sending to {self.pi_addr}")
        self.screen.ontimer(self._control_loop, 100)

    # ── GUI Setup ─────────────────────────────────────────────────────

    def _setup_gui(self):
        self.screen = turtle.Screen()
        self.screen.setup(width=0.9, height=0.9)
        self.screen.bgcolor("#2c3e50")
        self.screen.title(
            "SPV Remote Controller  |  "
            "[1] Start [2] Clear [3] STOP [4] Origin  |  "
            "[5] PID [6] SMC [7] PP [8] LQR [9] MPC  |  [+/-] Speed"
        )

        self.view_size = 5  # meters (±5m view)
        self.screen.setworldcoordinates(
            -self.view_size, -self.view_size,
            self.view_size, self.view_size,
        )
        self.screen.tracer(0)

        # Grid
        self.grid_pen = turtle.Turtle(visible=False)
        self._draw_grid()

        # Waypoint path drawer
        self.wp_pen = turtle.Turtle()
        self.wp_pen.pencolor("#e74c3c")
        self.wp_pen.penup()

        # Rover triangle
        self.rover_turtle = turtle.Turtle(shape="triangle")
        self.rover_turtle.shapesize(1.2, 1.2)
        self.rover_turtle.color("#2ecc71")
        self.rover_turtle.penup()

        # Trail
        self.trail_pen = turtle.Turtle(visible=False)
        self.trail_pen.pencolor("#3498db")
        self.trail_pen.pensize(2)
        self.trail_pen.penup()
        self.trail_started = False

        # Status text
        self.status_pen = turtle.Turtle(visible=False)
        self.status_pen.penup()
        self.status_pen.color("white")

        # Telemetry info text
        self.info_pen = turtle.Turtle(visible=False)
        self.info_pen.penup()
        self.info_pen.color("#95a5a6")

        # Heartbeat indicator
        self.heartbeat_pen = turtle.Turtle(visible=False)
        self.heartbeat_pen.penup()
        self.heartbeat_pen.goto(self.view_size - 0.5, self.view_size - 0.5)

        # Key bindings
        self.screen.listen()
        self.screen.onclick(self._handle_click)
        self.screen.onkey(self._start_mission, "1")
        self.screen.onkey(self._clear_mission, "2")
        self.screen.onkey(self._emergency_stop, "3")
        self.screen.onkey(self._set_origin, "4")
        self.screen.onkey(lambda: self._switch_controller("pid"), "5")
        self.screen.onkey(lambda: self._switch_controller("smc"), "6")
        self.screen.onkey(lambda: self._switch_controller("purepursuit"), "7")
        self.screen.onkey(lambda: self._switch_controller("lqr"), "8")
        self.screen.onkey(lambda: self._switch_controller("mpc"), "9")
        self.screen.onkey(self._speed_up, "plus")
        self.screen.onkey(self._speed_up, "equal")  # = key (unshifted +)
        self.screen.onkey(self._speed_down, "minus")

        self._update_status("READY: Click to add waypoints")

    def _draw_grid(self):
        """Draw 1-meter grid lines."""
        self.grid_pen.clear()
        self.grid_pen.pencolor("#34495e")
        self.grid_pen.pensize(1)
        v = self.view_size
        for i in range(-int(v), int(v) + 1):
            # Vertical
            self.grid_pen.penup()
            self.grid_pen.goto(i, -v)
            self.grid_pen.pendown()
            self.grid_pen.goto(i, v)
            # Horizontal
            self.grid_pen.penup()
            self.grid_pen.goto(-v, i)
            self.grid_pen.pendown()
            self.grid_pen.goto(v, i)
        # Origin marker
        self.grid_pen.penup()
        self.grid_pen.goto(0, 0)
        self.grid_pen.dot(8, "#7f8c8d")
        self.screen.update()

    def _update_status(self, text: str):
        self.status_pen.clear()
        v = self.view_size
        self.status_pen.goto(-v + 0.2, v - 0.6)
        self.status_pen.write(
            f"STATUS: {text}",
            font=("Consolas", 12, "bold"),
        )
        ctrl_name = self.CONTROLLERS.get(self.controller_name, self.controller_name)
        self.status_pen.goto(-v + 0.2, v - 1.1)
        self.status_pen.write(
            f"CTRL: {ctrl_name}  |  SPEED: {self.cruise_speed*100:.1f} cm/s  |  "
            f"WP: {self.current_wp_idx}/{len(self.waypoints)}",
            font=("Consolas", 10, "normal"),
        )

    def _update_info(self, telem: TelemetryPacket, V_L: float, V_R: float,
                     pwm_l: int, pwm_r: int):
        """Update telemetry info display at bottom of screen."""
        self.info_pen.clear()
        v = self.view_size

        v_fwd = telem.gps_speed_kmh / 3.6
        rpm_l = _nan_or(telem.rpm_left)
        rpm_r = _nan_or(telem.rpm_right)

        self.info_pen.goto(-v + 0.2, -v + 1.4)
        self.info_pen.write(
            f"Pos: ({telem.gps_x:.2f}, {telem.gps_y:.2f}) m  |  "
            f"Hdg: {telem.heading_deg:.1f} deg  |  "
            f"v: {v_fwd*100:.1f} cm/s  |  "
            f"Fix: {telem.fix_quality}",
            font=("Consolas", 10, "normal"),
        )
        self.info_pen.goto(-v + 0.2, -v + 0.8)
        self.info_pen.write(
            f"RPM: L={rpm_l:.0f}  R={rpm_r:.0f}  |  "
            f"V_cmd: L={V_L:.2f}V  R={V_R:.2f}V  |  "
            f"PWM: L={pwm_l}  R={pwm_r}",
            font=("Consolas", 10, "normal"),
        )
        self.info_pen.goto(-v + 0.2, -v + 0.2)
        self.info_pen.write(
            f"Packets: {self.pkt_count}  |  Ctrl cycles: {self.ctrl_count}",
            font=("Consolas", 10, "normal"),
        )

    # ── Key Handlers ──────────────────────────────────────────────────

    def _handle_click(self, x: float, y: float):
        dist = math.sqrt(x ** 2 + y ** 2)
        if dist < 0.05:
            print("Too close to origin, ignoring")
            return
        self.waypoints.append((x, y))
        self.wp_pen.goto(x, y)
        self.wp_pen.dot(8, "#e74c3c")
        self.wp_pen.pendown()
        print(f"  WP {len(self.waypoints)}: ({x:.2f}, {y:.2f})")
        self._update_status(f"{len(self.waypoints)} waypoint(s)")

    def _start_mission(self):
        if len(self.waypoints) == 0:
            print("No waypoints set!")
            return
        self.is_running = True
        self.current_wp_idx = 0
        self.pp_state = PurePursuitState()  # Reset lookahead
        self._update_status("MISSION RUNNING")
        print(f"Mission started: {len(self.waypoints)} waypoints, "
              f"controller={self.controller_name}")

    def _clear_mission(self):
        self.is_running = False
        self.waypoints.clear()
        self.current_wp_idx = 0
        self.wp_pen.clear()
        self.wp_pen.penup()
        self.trail_pen.clear()
        self.trail_pen.penup()
        self.trail_started = False
        self._update_status("CLEARED: Click to add waypoints")
        # Send stop command immediately
        self.send_sock.sendto(b"CMD,1500,1500", self.pi_addr)
        print("Mission cleared")

    def _emergency_stop(self):
        self.is_running = False
        self.send_sock.sendto(b"STOP", self.pi_addr)
        self._update_status("EMERGENCY STOP")
        print("EMERGENCY STOP sent")

    def _set_origin(self):
        self.send_sock.sendto(b"SET_ORIGIN", self.pi_addr)
        self.prev_theta = 0.0
        self.trail_pen.clear()
        self.trail_pen.penup()
        self.trail_started = False
        self._update_status("ORIGIN SET: Rover at (0,0)")
        print("SET_ORIGIN sent")

    def _switch_controller(self, name: str):
        self.controller_name = name
        # Reset controller states
        self.pid_state = PIDState()
        self.smc_state = SMCState()
        self.pp_state = PurePursuitState()
        self.u_prev = np.array([0.0, 0.0])
        ctrl_display = self.CONTROLLERS.get(name, name)
        self._update_status(f"Controller: {ctrl_display}")
        print(f"Switched to: {ctrl_display}")

    def _speed_up(self):
        self.cruise_speed = min(self.cruise_speed + 0.005, 0.10)
        self._update_status(f"Speed: {self.cruise_speed*100:.1f} cm/s")
        print(f"Cruise speed: {self.cruise_speed*100:.1f} cm/s")

    def _speed_down(self):
        self.cruise_speed = max(self.cruise_speed - 0.005, 0.005)
        self._update_status(f"Speed: {self.cruise_speed*100:.1f} cm/s")
        print(f"Cruise speed: {self.cruise_speed*100:.1f} cm/s")

    # ── Control Loop ──────────────────────────────────────────────────

    def _control_loop(self):
        """Main control loop — called every 100 ms by Turtle timer.

        1. Drain all pending telemetry from the Pi
        2. Estimate plant state from latest telemetry
        3. Compute reference from waypoint mission
        4. Run selected controller to get V_L, V_R
        5. Convert to PWM and send to Pi
        6. Update GUI
        """
        # ── 1. Receive latest telemetry ───────────────────────────────
        telem = None
        while True:
            try:
                data, _ = self.recv_sock.recvfrom(4096)
                msg = data.decode('utf-8', errors='replace')
                pkt = parse_telemetry(msg)
                if pkt is not None:
                    telem = pkt
                    self.pkt_count += 1
            except BlockingIOError:
                break
            except Exception:
                break

        if telem is None:
            # No new telemetry — send heartbeat to keep watchdog alive
            try:
                self.send_sock.sendto(b"CMD,1500,1500", self.pi_addr)
            except OSError:
                pass
            self.heartbeat_pen.clear()
            self.heartbeat_pen.dot(12, "#e74c3c")  # Red = no data
            self.screen.update()
            self.screen.ontimer(self._control_loop, 100)
            return

        self.latest_telem = telem
        self.heartbeat_pen.clear()
        self.heartbeat_pen.dot(12, "#2ecc71")  # Green = receiving

        # ── 2. Estimate state ─────────────────────────────────────────
        dt = self.p.dt_ctrl
        s = telemetry_to_state(telem, self.prev_theta, dt, self.p)
        self.prev_theta = s[2]  # Store theta for next dtheta computation

        x, y, theta = s[0], s[1], s[2]

        # ── 3. Compute reference & run controller ─────────────────────
        V_L, V_R = 0.0, 0.0

        if self.is_running and self.current_wp_idx < len(self.waypoints):
            wp = self.waypoints[self.current_wp_idx]
            dx = wp[0] - x
            dy = wp[1] - y
            dist = math.sqrt(dx ** 2 + dy ** 2)

            # Check if waypoint reached
            if dist < self.wp_threshold:
                self.current_wp_idx += 1
                if self.current_wp_idx >= len(self.waypoints):
                    print("Mission complete!")
                    self.is_running = False
                    self._update_status("MISSION COMPLETE")
                else:
                    print(f"WP {self.current_wp_idx} reached, "
                          f"{len(self.waypoints) - self.current_wp_idx} remaining")

            # Run controller if mission still active
            if self.is_running and self.current_wp_idx < len(self.waypoints):
                V_L, V_R = self._run_controller(s)

        self.u_prev = np.array([V_L, V_R])
        self.ctrl_count += 1

        # ── 4. Convert to PWM and send ────────────────────────────────
        pwm_l = voltage_to_pwm(V_L, self.p.V_max)
        pwm_r = voltage_to_pwm(V_R, self.p.V_max)

        cmd = f"CMD,{pwm_l},{pwm_r}"
        try:
            self.send_sock.sendto(cmd.encode(), self.pi_addr)
        except OSError as e:
            print(f"Send error: {e}")

        # ── 5. Update GUI ─────────────────────────────────────────────
        self.rover_turtle.goto(telem.gps_x, telem.gps_y)
        self.rover_turtle.setheading(telem.heading_deg)

        # Trail
        if self.trail_started:
            self.trail_pen.goto(telem.gps_x, telem.gps_y)
        else:
            self.trail_pen.goto(telem.gps_x, telem.gps_y)
            self.trail_pen.pendown()
            self.trail_started = True

        self._update_info(telem, V_L, V_R, pwm_l, pwm_r)
        self._update_status(
            "RUNNING" if self.is_running else "STOPPED"
        )
        self.screen.update()

        # ── Schedule next ─────────────────────────────────────────────
        self.screen.ontimer(self._control_loop, 100)

    def _run_controller(self, s: np.ndarray) -> Tuple[float, float]:
        """Dispatch to the selected controller and return (V_L, V_R)."""
        wp = self.waypoints[self.current_wp_idx]

        if self.controller_name == "pid":
            z_ref = waypoint_to_reference(
                s[0], s[1], s[2], wp[0], wp[1], self.cruise_speed,
            )
            V_L, V_R, _ = pid_controller(
                s, z_ref, self.u_prev, self.p, self.pid_state,
            )

        elif self.controller_name == "smc":
            z_ref = waypoint_to_reference(
                s[0], s[1], s[2], wp[0], wp[1], self.cruise_speed,
            )
            V_L, V_R, _ = smc_controller(
                s, z_ref, self.u_prev, self.p, self.smc_state,
            )

        elif self.controller_name == "purepursuit":
            # Build dense trajectory through remaining waypoints
            remaining_wps = self.waypoints[self.current_wp_idx:]
            if len(remaining_wps) < 2:
                remaining_wps = [(s[0], s[1])] + remaining_wps
            z_ref_full = waypoints_to_trajectory(
                remaining_wps, self.cruise_speed,
            )
            V_L, V_R, _ = purepursuit_controller(
                s, z_ref_full, self.u_prev, self.p, self.pp_state,
            )

        elif self.controller_name == "lqr":
            z_ref = waypoint_to_reference(
                s[0], s[1], s[2], wp[0], wp[1], self.cruise_speed,
            )
            V_L, V_R, _ = lqr_controller(
                s, z_ref, self.u_prev, self.p,
            )

        elif self.controller_name == "mpc":
            z_ref_horizon = waypoint_to_horizon(
                s[0], s[1], s[2], wp[0], wp[1], self.cruise_speed,
                self.p.N_horizon, self.p.dt_pred,
            )
            V_L, V_R, _ = mpc_controller(
                s, z_ref_horizon, self.u_prev, self.p,
            )

        else:
            print(f"Unknown controller: {self.controller_name}")
            V_L, V_R = 0.0, 0.0

        return V_L, V_R


# ══════════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="SPV Remote Controller — laptop-side control loop + GCS GUI",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--controller", "-c",
        choices=["pid", "smc", "purepursuit", "lqr", "mpc"],
        default="pid",
        help="Controller to use (default: pid)",
    )
    parser.add_argument(
        "--speed", "-s",
        type=float,
        default=0.03,
        help="Cruise speed in m/s (default: 0.03 = 3 cm/s)",
    )
    parser.add_argument(
        "--pi-ip",
        default="172.20.10.6",
        help="Raspberry Pi IP address (default: 172.20.10.6)",
    )
    parser.add_argument(
        "--send-port",
        type=int,
        default=5007,
        help="UDP port to send commands to Pi (default: 5007)",
    )
    parser.add_argument(
        "--listen-port",
        type=int,
        default=5008,
        help="UDP port to listen for telemetry (default: 5008)",
    )
    args = parser.parse_args()

    print("═" * 50)
    print("  SPV Remote Controller")
    print(f"  Controller : {args.controller}")
    print(f"  Speed      : {args.speed*100:.1f} cm/s")
    print(f"  Pi address : {args.pi_ip}:{args.send_port}")
    print(f"  Listen on  : 0.0.0.0:{args.listen_port}")
    print("═" * 50)

    ctrl = RemoteController(args)
    turtle.done()


if __name__ == "__main__":
    main()
