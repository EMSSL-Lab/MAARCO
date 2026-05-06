import gymnasium as gym
from gymnasium import spaces
import numpy as np
import pygame
import math

class RoverPIDEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": 50}

    def __init__(self, render_mode=None):
        super(RoverPIDEnv, self).__init__()
        
        self.dt = 0.1
        self.velocity = 0.2  # Constant forward speed
        self.target_pos = np.array([0.0, 0.0]) # Initialize with a default
        self.start_pos = np.array([300.0, 300.0])
        self.yaw = 0.0
        self.rover_pos = np.array([300.0, 300.0]) #
        self.target_yaw = 0
        # RL Spaces
        # Observation: [dist_to_target, heading_error, integral_error, prev_error]
        self.observation_space = spaces.Box(
            low=np.array([0, -np.pi, -10.0, -np.pi]), 
            high=np.array([20.0, np.pi, 10.0, np.pi]), 
            dtype=np.float32
        )
        
        # Action: PID Gains [Kp, Ki, Kd]
        self.action_space = spaces.Box(low=0.0, high=10.0, shape=(3,), dtype=np.float32)
        # self.action_space = spaces.Box(low=0.0, high=1.0, shape=(3,), dtype=np.float32)

        self.render_mode = render_mode
        # self.screen_size = 600
        # Use class variables to ensure consistency
        self.width = 800
        self.height = 600
        self.screen = None
        self.clock = None

        self.is_out_of_bounds = False

    def _get_obs(self):
        dx = self.target_pos[0] - self.rover_pos[0]
        dy = self.target_pos[1] - self.rover_pos[1]
        dist = np.sqrt(dx**2 + dy**2)
        
        target_angle = np.arctan2(dy, dx)
        # Normalize heading error to [-pi, pi]
        # self.error = (target_angle - self.yaw + np.pi) % (2 * np.pi) - np.pi
        
        # In _get_obs()
        self.error = (self.target_yaw - self.yaw + np.pi) % (2 * np.pi) - np.pi

        return np.array([dist, self.error, self.integral, self.prev_error], dtype=np.float32)

    def step(self, action):
        if self.is_out_of_bounds:
            # If out of bounds, force the robot to stay still and return 0 reward
            # This forces the model to wait for the human "F" flip
            return self._get_obs(), 0.0, False, False, {"status": "WAITING_FOR_FLIP"}
        
        self.steps += 1
        kp, ki, kd = action

        # 1. PID Logic (Calculating the correction for the Right Motor)
        self.integral = np.clip(self.integral + self.error * self.dt, -5.0, 5.0)
        derivative = (self.error - self.prev_error) / self.dt
        
        # The PID output now defines the velocity of the right motor
        # base_velocity is your "constant" for the left motor
        base_vel = 0.2 
        
        v_left = base_vel * (1 + self.motor_drift)
        # Right motor is the correcting variable
        # We add steering because if error is positive, we want to turn left
        # (which requires a faster right motor)
        v_right = base_vel + (kp * self.error + ki * self.integral + kd * derivative)
        
        v_left = np.clip(v_left, -1.0, 1.0)
        v_right = np.clip(v_right, -1.0, 1.0)

        # 2. Differential Drive Physics
        # Wheelbase width (distance between wheels)
        L = 0.5 
        
        # Linear velocity (average of both motors)
        v_linear = (v_left + v_right) / 2.0
        # Angular velocity (difference divided by wheelbase)
        v_angular = (v_right - v_left) / L
        
        # Update State
        self.yaw += v_angular * self.dt
        self.rover_pos[0] += v_linear * 50 * np.cos(self.yaw) * self.dt
        self.rover_pos[1] += v_linear * 50 * np.sin(self.yaw) * self.dt
        
        self.prev_error = self.error
        obs = self._get_obs()
        dist_to_target = obs[0]


        # Convert 20 degrees to radians
        overshoot_threshold = np.radians(20)
        # 3. Reward Function
        # Reward for pointing at target + huge bonus for reaching it
        reward = np.cos(self.error) - 0.1 # Base reward for orientation
        
        terminated = False
        if dist_to_target < 15:
            reward += 100.0
            terminated = True
        
        # 2. Threshold Penalty
        # If the absolute error exceeds 20 degrees, apply a significant penalty
        if abs(self.error) > overshoot_threshold:
            # You can use a flat penalty or a scaling one
            # reward -= 2.0  # Flat penalty for being out of "acceptable" range
            
            # Optional: Scaling penalty (the further past 20 deg, the worse it gets)
            reward -= 5.0 * (abs(self.error) - overshoot_threshold)

        # Out of bounds
        if not (0 < self.rover_pos[0] < self.width and 0 < self.rover_pos[1] < self.height):
            reward -= 50.0
            terminated = True

        truncated = self.steps >= 500

        if self.render_mode == "human":
            self.render()

        # Check bounds (adjust numbers for your screen size)
        if not (0 < self.rover_pos[0] < 800 and 0 < self.rover_pos[1] < 600):
            self.is_out_of_bounds = True
            print("\n!!! OUT OF BOUNDS !!! Please press [F] to flip and continue.")
        
        # In Rover.py step()
        self.current_steps += 1
        if self.current_steps > 1000:
            truncated = True  # This will force the logger to print a summary
            self.current_steps = 0
            
        # We never set terminated = True for bounds anymore
        terminated = False 
        truncated = False

        return obs, reward, terminated, truncated, {}

    def render(self):
        if self.screen is None:
            pygame.init()
            self.screen = pygame.display.set_mode((self.width, self.height))
            self.clock = pygame.time.Clock()

        self.screen.fill((44, 62, 80)) # Dark Blue background
        
        # Draw Target
        # pygame.draw.circle(self.screen, (231, 76, 60), self.target_pos.astype(int), 10)

        # --- FIXED TARGET YAW LINE ---
        # The line starts at self.start_pos and stays there
        line_len = 150 
        target_end_x = self.start_pos[0] + line_len * np.cos(self.target_yaw)
        target_end_y = self.start_pos[1] + line_len * np.sin(self.target_yaw)
        
        # Draw the target orientation line in Red
        pygame.draw.line(self.screen, (231, 76, 60), self.start_pos, (target_end_x, target_end_y), 3)
        
        # Draw a small circle at the origin point for clarity
        pygame.draw.circle(self.screen, (231, 76, 60), self.start_pos.astype(int), 4)

        # Draw Rover (Triangle)
        points = [
            (15, 0), (-10, -8), (-10, 8) # Triangle vertices
        ]
        rotated_points = []
        for x, y in points:
            rx = x * np.cos(self.yaw) - y * np.sin(self.yaw) + self.rover_pos[0]
            ry = x * np.sin(self.yaw) + y * np.cos(self.yaw) + self.rover_pos[1]
            rotated_points.append((rx, ry))
        
        pygame.draw.polygon(self.screen, (52, 152, 219), rotated_points)
        
        pygame.display.flip()
        self.clock.tick(self.metadata["render_fps"])

    # def reset(self, seed=None, options=None):
    #         super().reset(seed=seed)
            
    #         # Start rover in center, target at a random distance
    #         self.rover_pos = np.array([300.0, 300.0])
    #         # Capture the starting position to anchor the line
    #         self.start_pos = self.rover_pos.copy()

    #         self.yaw = 0.0
            
    #         # Random target within 200px radius
    #         angle = np.random.uniform(0, 2 * np.pi)
    #         dist = np.random.uniform(100, 250)
    #         self.target_pos = self.rover_pos + np.array([np.cos(angle)*dist, np.sin(angle)*dist])

    #         self.integral = 0.0
    #         self.prev_error = 0.0
    #         self.steps = 0
            
    #         self.motor_drift = 0.15  # 15% bias towards one side

    #         # self.target_yaw = np.random.uniform(-np.pi, np.pi)  # Random target orientation (not used in this example but can be extended)
    #         self.target_yaw = 0  # Random target orientation (not used in this example but can be extended)


    #         return self._get_obs(), {}
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        # Only teleport to center on a HARD reset (start of training)
        self.rover_pos = np.array([400.0, 300.0])
        self.yaw = 0.0
        self.target_yaw = 0.0
        self.is_out_of_bounds = False
        self.integral = 0.0
        self.prev_error = 0.0
        self.steps = 0
        self.motor_drift = 0.15  # 15% bias towards one side
        return self._get_obs(), {}