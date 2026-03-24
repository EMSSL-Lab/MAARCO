import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- 1. PHYSICAL PARAMETERS ---
MASS = 5.0           # kg
INERTIA = 0.08       # kg*m^2
TRACK_WIDTH = 0.2    # meters
D = TRACK_WIDTH / 2
WHEEL_RADIUS = 0.0127 # 0.5 inch in meters

# --- 2. MOTOR CONSTANTS (RB-Dfr-444) ---
V_BUS = 12.0
R_OHMS = 1.71
KT = 0.265
KE = 0.456
V_FR = 0.6           # Friction compensation voltage

class RobotController:
    def __init__(self):
        # PD Gains for the 5kg chassis
        self.Kp_v, self.Kd_v = 30.0, 10.0
        self.Kp_t, self.Kd_t = 12.0, 2.5

    def get_efforts(self, v_target, theta_target, v_curr, theta_curr, omega_curr):
        error_v = v_target - v_curr
        # Normalize theta error to [-pi, pi]
        error_theta = (theta_target - theta_curr + np.pi) % (2 * np.pi) - np.pi
        
        F_total = MASS * ((self.Kp_v * error_v) - (self.Kd_v * v_curr))
        tau_total = INERTIA * ((self.Kp_t * error_theta) - (self.Kd_t * omega_curr))
        
        return F_total, tau_total

def calculate_motor_logic(F_total, tau_total, v_curr, omega_curr, V_measured):
    # Mixing
    F_r = (F_total / 2.0) + (tau_total / (2.0 * D))
    F_l = (F_total / 2.0) - (tau_total / (2.0 * D))
    
    # Kinematics to find wheel speeds
    omega_w_r = (v_curr + (omega_curr * D)) / WHEEL_RADIUS
    omega_w_l = (v_curr - (omega_curr * D)) / WHEEL_RADIUS
    
    def get_v_des(F_w, omega_w):
        if abs(F_w) < 0.001 and abs(omega_w) < 0.001: return 0.0
        sgn = np.sign(F_w) if F_w != 0 else np.sign(omega_w)
        v_torque = (F_w * WHEEL_RADIUS * R_OHMS) / KT
        v_back_emf = KE * omega_w
        return (sgn * V_FR) + v_torque + v_back_emf

    Vr = np.clip(get_v_des(F_r, omega_w_r), -V_measured, V_measured)
    Vl = np.clip(get_v_des(F_l, omega_w_l), -V_measured, V_measured)
    
    pwm_r = int((abs(Vr) / V_measured) * 255)
    pwm_l = int((abs(Vl) / V_measured) * 255)
    
    return Vr, Vl, pwm_r, pwm_l

# --- 3. PRE-CALCULATE PHYSICS ---
dt = 0.02
time = np.arange(0, 24, dt)
state = np.zeros(5) # [x, y, theta, v, omega]
history = []
pwm_history = []
bot = RobotController()

targets = [(0.4, 0), (0, np.pi/2), (0.4, np.pi/2), (0, np.pi), 
           (0.4, np.pi), (0, 3*np.pi/2), (0.4, 3*np.pi/2), (0, 2*np.pi)]

for t in time:
    v_target, theta_target = targets[int(t/3) % len(targets)]
    x, y, theta, v, omega = state
    
    F, T = bot.get_efforts(v_target, theta_target, v, theta, omega)
    Vr, Vl, pr, pl = calculate_motor_logic(F, T, v, omega, 12.0)
    
    # Physics Integration (Back-calculating Force from Voltage)
    ir = (Vr - KE * ((v + omega*D)/WHEEL_RADIUS)) / R_OHMS
    il = (Vl - KE * ((v - omega*D)/WHEEL_RADIUS)) / R_OHMS
    fr = (KT * ir - (np.sign(ir)*V_FR*KT/R_OHMS)) / WHEEL_RADIUS
    fl = (KT * il - (np.sign(il)*V_FR*KT/R_OHMS)) / WHEEL_RADIUS
    
    v += ((fr + fl) / MASS) * dt
    omega += (((fr - fl) * D) / INERTIA) * dt
    theta += omega * dt
    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt
    
    state = [x, y, theta, v, omega]
    history.append(state)
    pwm_history.append([pr, pl])

history = np.array(history)
pwm_history = np.array(pwm_history)

# --- 4. ANIMATION SETUP ---
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

# Plot 1: The Turtle Bot
ax1.set_xlim(-0.2, 1.0); ax1.set_ylim(-0.2, 1.0); ax1.set_aspect('equal')
ax1.grid(True, linestyle='--')
path_line, = ax1.plot([], [], 'b-', alpha=0.6, label="Path")
robot_body, = ax1.plot([], [], 'ro', markersize=10, label="Turtle")
orient_line, = ax1.plot([], [], 'r-', lw=2)

# Plot 2: PWM Signals
ax2.set_xlim(0, 24); ax2.set_ylim(0, 260)
ax2.set_title("Real-time PWM Output")
ax2.set_xlabel("Time (s)"); ax2.set_ylabel("PWM (0-255)")
pwm_r_line, = ax2.plot([], [], 'r-', label="Right Motor")
pwm_l_line, = ax2.plot([], [], 'b-', label="Left Motor")
ax2.legend(loc='upper right')

def update(frame):
    # Update Robot Position
    path_line.set_data(history[:frame, 0], history[:frame, 1])
    rx, ry, rt = history[frame, 0], history[frame, 1], history[frame, 2]
    robot_body.set_data([rx], [ry])
    orient_line.set_data([rx, rx + 0.06*np.cos(rt)], [ry, ry + 0.06*np.sin(rt)])
    
    # Update PWM Plot
    pwm_r_line.set_data(time[:frame], pwm_history[:frame, 0])
    pwm_l_line.set_data(time[:frame], pwm_history[:frame, 1])
    
    return path_line, robot_body, orient_line, pwm_r_line, pwm_l_line

ani = FuncAnimation(fig, update, frames=range(0, len(time), 5), blit=True, interval=20)
plt.tight_layout()
plt.show()