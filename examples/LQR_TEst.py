import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_continuous_are

# 1. Physical Parameters
m = 1.0    # Mass (kg)
b = 0.5    # Friction coefficient (N*s/m)

# 2. State-Space Matrices (A, B)
# States x = [position, velocity]
A = np.array([[0, 1],
              [0, -b/m]])
B = np.array([[0],
              [1/m]])

# 3. LQR Weights (Q, R)
# Penalize position error (100) and velocity error (10)
Q = np.diag([100, 10]) 
# Penalize control effort (force)
R = np.array([[0.01]])

# 4. (Algebraic Riccati Equation) to find optimal P and then K
P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P  # Optimal Gain Vector [k1, k2]

# 5. Simulation Setup
dt = 0.01
time = np.linspace(0, 10, 1000)
x = np.array([[0.0], [0.0]]) # Initial state [pos, vel]
history = []

# CHOOSE MODE: 'position', 'velocity', or 'dash' (2m/s to 10m)
mode_1 = 'velocity' 

for t in time:
    # 6. Define the Moving Reference r(t)
    if mode_1 == 'position':
        r = np.array([[10.0], [0.0]]) # Just go to 10m and stop
    elif mode_1 == 'velocity':
        r = np.array([[x[0,0]], [2.0]])  # Just hit 2m/s, don't care where
    elif mode_1 == 'dash':
        # Go 2m/s until 10m, then stop
        target_p = min(2.0 * t, 10.0)
        target_v = 2.0 if target_p < 10.0 else 0.0
        r = np.array([[target_p], [target_v]])

    # 7. Control Law: u = -K(x - r) + FeedForward
    error = x - r
    u_feedback = -K @ error
    
    # Feed-forward to fight friction at the target velocity
    u_ff = b * r[1,0] 
    
    u = u_feedback + u_ff
    
    # 8. Physics Update (Euler Integration)
    x_dot = A @ x + B @ u
    x = x + x_dot * dt
    history.append(x.flatten())

# 9. Plotting
history = np.array(history)
plt.figure(figsize=(10, 5))
plt.subplot(2,1,1)
plt.plot(time, history[:,0], label='Actual Position')
plt.ylabel('Position (m)')
plt.legend()
plt.subplot(2,1,2)
plt.plot(time, history[:,1], label='Actual Velocity', color='orange')
plt.ylabel('Velocity (m/s)')
plt.xlabel('Time (s)')
plt.show()

# ==================== 2D LQR Point-to-Point Example ====================

# 1. Robot Parameters
m = 1.0     # Mass (kg)
I = 0.1     # Moment of Inertia (kg*m^2)
b_v = 0.5   # Linear Friction
b_w = 0.1   # Angular Friction
v_nom = 2.0 # Nominal velocity for linearization

# 2. 2D State-Space (x, y, theta, v, omega)
# We assume the robot is mostly pointing along the X-axis for the ARE
A = np.array([
    [0, 0, 0,     1, 0],     # dx = v * cos(theta) -> approx v
    [0, 0, v_nom, 0, 0],     # dy = v * sin(theta) -> approx v_nom * theta
    [0, 0, 0,     0, 1],     # dtheta = omega
    [0, 0, 0, -b_v/m, 0],    # dv = Force/m - drag
    [0, 0, 0, 0, -b_w/I]     # domega = Torque/I - drag
])

B = np.array([
    [0, 0],
    [0, 0],
    [0, 0],
    [1/m, 0],   # Input 1: Force (Thrust)
    [0, 1/I]    # Input 2: Torque (Steering)
])

# 3. Weights (Q and R)
# Penalize X error, Y error, Heading error, then velocities
Q = np.diag([50, 50, 10, 1, 1]) 
# R is 2x2: [Thrust effort, Steering effort]
R = np.diag([0.01, 0.01])

# 4. Solve the 2D "P Business"
P = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ P

# 5. Simulation
dt = 0.01
t_final = 10
time = np.arange(0, t_final, dt)

traj = []
mode_2 = 'dash' # 'position', 'velocity', or 'dash'

# State: [x, y, theta, v, omega]
state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

for t in time:
    
    # 6. Define the Moving Reference r(t)
    if mode_2 == 'position':
        # Target: [x, y, theta, v, omega]
        target = np.array([10.0, 10.0, 0.785, 0.0, 0.0]) # 0.785 rad = 45 deg
    elif mode_2 == 'velocity':
        # Target: Stay at current x,y but maintain 2m/s at 45 degrees
        target = np.array([state[0], state[1], 0.785, 2.0, 0.0])    
    elif mode_2 == 'dash':
        # Calculate distance to (10,10)
        dist_to_go = np.sqrt((10 - state[0])**2 + (10 - state[1])**2)
        if dist_to_go > 0.1:
            # We are still moving: Set ghost car position and speed
            # To go 2m/s total, x and y move at 1.41m/s
            v_ref = 2.0
            p_ref = min(1.41 * t, 10.0) 
            target = np.array([p_ref, p_ref, 0.785, v_ref, 0.0])
        else:
            # We arrived: Stop the ghost car
            target = np.array([10.0, 10.0, 0.785, 0.0, 0.0])

    # 7. Error vector (Now the subtraction works perfectly)
    err = state - target
    
    # Control Law (u is [Force, Torque])
    u = -K @ err
    
    # Physics Update (Using nonlinear trig for the actual movement!)
    dstatedt = np.array([
        state[3] * np.cos(state[2]), # Real dx
        state[3] * np.sin(state[2]), # Real dy
        state[4],                    # dtheta
        (u[0] - b_v*state[3])/m,     # dv
        (u[1] - b_w*state[4])/I      # domega
    ])
    
    state += dstatedt * dt
    traj.append(state.copy())

# 6. Plotting the XY Path
traj = np.array(traj)
plt.figure(figsize=(6,6))
plt.plot(traj[:,0], traj[:,1], label='Robot Path')
plt.scatter([0, 10], [0, 10], color='red', label='Start/End')
plt.title("2D LQR Point-to-Point")
plt.xlabel("X Position (m)"); plt.ylabel("Y Position (m)")
plt.legend(); plt.grid(True)
plt.show()