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
        self.next_reset_is_flip = False
        # RL Spaces
        # Observation: [dist_to_target, heading_error, integral_error, prev_error]
        self.observation_space = spaces.Box(
            low=np.array([0, -np.pi, -10.0, -np.pi]), 
            high=np.array([20.0, np.pi, 10.0, np.pi]), 
            dtype=np.float32
        )
        self.trigger_new_episode = False  # Initialize the flag
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
        # 1. Check for Flip signal FIRST
        if getattr(self, 'trigger_new_episode', False):
            self.trigger_new_episode = False
            self.is_out_of_bounds = False
            print(f" [TRAIN] Episode Terminated: Flip Triggered at step {self.steps}")
            return self._get_obs(), 0.0, True, False, {"status": "FLIP_RESET"}

        # 2. LOCK STATE: If waiting for flip, do nothing but return observations
        if self.is_out_of_bounds:
            return self._get_obs(), -1.0, False, False, {"status": "WAITING_FOR_FLIP"}
        
        # 3. PID & Physics Logic
        self.steps += 1
        kp, ki, kd = action

        # PID Math
        self.integral = np.clip(self.integral + self.error * self.dt, -5.0, 5.0)
        derivative = (self.error - self.prev_error) / self.dt
        
        base_vel = 0.2 
        v_left = base_vel * (1 + self.motor_drift)
        v_right = base_vel + (kp * self.error + ki * self.integral + kd * derivative)
        
        v_left = np.clip(v_left, -1.0, 1.0)
        v_right = np.clip(v_right, -1.0, 1.0)

        # Differential Drive Update
        L = 0.5 
        v_linear = (v_left + v_right) / 2.0
        v_angular = (v_right - v_left) / L
        
        self.yaw += v_angular * self.dt
        self.rover_pos[0] += v_linear * 50 * np.cos(self.yaw) * self.dt
        self.rover_pos[1] += v_linear * 50 * np.sin(self.yaw) * self.dt
        
        self.prev_error = self.error
        obs = self._get_obs()
        dist_to_target = obs[0]

        # 3. Reward Calculation
        overshoot_threshold = np.radians(20)
        reward = np.cos(self.error) - 0.1 
        
        if abs(self.error) > overshoot_threshold:
            reward -= 5.0 * (abs(self.error) - overshoot_threshold)

        # 4. State Management (Bounds & Episodes)
        terminated = False
        truncated = False

        # CONDITION A: Flip detected (Signal sent from Callback)
        if getattr(self, 'trigger_new_episode', False):
            terminated = True
            self.trigger_new_episode = False 
            print(f" [TRAIN] Episode Terminated: Flip Completed at step {self.steps}")

        # CONDITION B: Step Limit (Scaling for the volleyball court)
        # 1000 steps at 0.1s dt is 100 seconds of driving.
        if self.steps >= 1000:
            truncated = True
            self.next_reset_is_flip = True
            print(f" [TRAIN] Episode Truncated: Step limit reached.")

        # 3. Boundary Check (Does not end episode, just pauses for Flip)
        if not (0 < self.rover_pos[0] < self.width and 0 < self.rover_pos[1] < self.height):
            if not self.is_out_of_bounds:
                self.is_out_of_bounds = True
                print("\n!!! OUT OF BOUNDS !!! Press [F] to pivot back.")
            
            # Safety clip
            self.rover_pos[0] = np.clip(self.rover_pos[0], 0, self.width)
            self.rover_pos[1] = np.clip(self.rover_pos[1], 0, self.height)
        
        if self.render_mode == "human":
            self.render()
        
        return self._get_obs(), reward, terminated, truncated, {}

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

    def reset(self, seed=None, options=None):
            super().reset(seed=seed)
            
            # Check if this is a "Flip" or a "Fresh Start"
            # If options is None or doesn't have 'is_flip', it defaults to False
            is_flip = False
            if options and "is_flip" in options:
                is_flip = options["is_flip"]
            elif getattr(self, "next_reset_is_flip", False):
                is_flip = True

            if not is_flip:
                # HARD RESET: Move back to center (only for initial load)
                self.rover_pos = np.array([400.0, 300.0])
                self.yaw = 0.0
                self.target_yaw = 0.0
            
            # PERSISTENT STATE: Keep current rover_pos and yaw if is_flip is True
            # We just reset the "housekeeping" variables
            self.start_pos = self.rover_pos.copy() # Move the line origin to where we are now
            self.next_reset_is_flip = False
            self.is_out_of_bounds = False
            self.integral = 0.0
            self.prev_error = 0.0
            self.steps = 0
            self.motor_drift = 0.15 
            
            return self._get_obs(), {}