import gymnasium as gym
from gymnasium import spaces
import numpy as np
import pygame
import sys

class BallAndBeamPIDEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": 50}

    def __init__(self, render_mode=None):
        super(BallAndBeamPIDEnv, self).__init__()
        
        self.training = True  # Flag to indicate training mode for disturbance injection

        # Physical Constants (Based on the Study)
        self.L = 0.3          # 30cm beam length
        self.dt = 0.02         # 20ms sampling rate
        self.g = 9.81         # Gravity
        self.max_angle = np.radians(25) # Beam limit +/- 25 degrees
        
        # RL Spaces
        # Observation: [beam_angle, ball_pos, ball_vel, setpoint]
        self.observation_space = spaces.Box(
            low=np.array([-self.max_angle, -self.L/2, -2.0, -self.L/2]), 
            high=np.array([self.max_angle, self.L/2, 2.0, self.L/2]), 
            dtype=np.float32
        )
        # Action: PID Gains [Kp, Ki, Kd] limited to 0.1 as per paper
        self.action_space = spaces.Box(low=0.0, high=0.1, shape=(3,), dtype=np.float32)

        # Rendering setup
        self.render_mode = render_mode
        self.screen_width = 600
        self.screen_height = 400
        self.screen = None
        self.clock = None

    def _get_obs(self):
        return np.array([
            self.beam_angle, 
            self.ball_pos, 
            self.ball_vel, 
            self.setpoint
        ], dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        # Initial Disturbance (Paper starts with ball at edge or tilted beam)
        if self.training:
            # Random start position within the beam during training
            self.ball_pos = np.random.uniform(-0.12, 0.12)
        else:
            self.ball_pos = -0.12  # Consistent start for testing
        self.ball_vel = 0.0
        self.beam_angle = 0.0
        self.setpoint = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.steps = 0
        
        self.g = np.random.uniform(9.7, 9.9) 
        self.ball_mass = np.random.uniform(0.02, 0.04)

        if self.render_mode == "human":
            self.render()
            
        return self._get_obs(), {}

    def step(self, action):
        self.steps += 1
        kp, ki, kd = action

        # 1. PID Logic
        error = self.setpoint - self.ball_pos
        self.integral += error * self.dt
        # Anti-windup (prevent integral from growing infinitely)
        self.integral = np.clip(self.integral, -1.0, 1.0)
        
        derivative = (error - self.prev_error) / self.dt
        
        # PID control sets the target beam angle
        u = (kp * error) + (ki * self.integral) + (kd * derivative)
        self.beam_angle = np.clip(u, -self.max_angle, self.max_angle)
        
        # 2. Physics Simulation (Ball on Beam dynamics)
        # Acceleration = (5/7) * g * sin(theta)
        accel = (5.0 / 7.0) * self.g * np.sin(self.beam_angle)
        self.ball_vel += accel * self.dt
        self.ball_pos += self.ball_vel * self.dt
        
        flick_occurred = False
        # Add a 1% chance of a random disturbance every step
        if self.training and np.random.random() < 0.01:
            # Random velocity spike between -0.5 and 0.5 m/s
            flick = np.random.uniform(-0.5, 0.5)
            self.ball_vel += flick
            flick_occurred = True

        # Add the flag to the info dict
        info = {"flick": flick_occurred}
        # 3. Reward Function (From Equation 7 in the paper)
        # r = 1 - |error/L|
        reward = 1.0 - abs(error / (self.L / 2))
        
        # 4. Terminated / Truncated
        # Terminate if ball falls off the 30cm beam
        terminated = bool(abs(self.ball_pos) > self.L / 2)
        # Optional: Truncate after 500 steps (10 seconds)
        truncated = bool(self.steps >= 500)
        
        self.prev_error = error

        if self.render_mode == "human":
            self.render()

        return self._get_obs(), reward, terminated, truncated, info

    def render(self):
        if self.screen is None:
            pygame.init()
            pygame.display.init()
            self.screen = pygame.display.set_mode((self.screen_width, self.screen_height))
            pygame.display.set_caption("Ball and Beam RL Simulation")
            self.clock = pygame.time.Clock()

        # Handle Pygame events to prevent freezing
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.close()
                sys.exit()

        self.screen.fill((255, 255, 255)) # White Background
        
        cx, cy = self.screen_width // 2, self.screen_height // 2
        beam_length_px = 400
        
        # Draw Beam
        x1 = cx - (beam_length_px // 2) * np.cos(self.beam_angle)
        y1 = cy - (beam_length_px // 2) * np.sin(self.beam_angle)
        x2 = cx + (beam_length_px // 2) * np.cos(self.beam_angle)
        y2 = cy + (beam_length_px // 2) * np.sin(self.beam_angle)
        
        pygame.draw.line(self.screen, (50, 50, 50), (x1, y1), (x2, y2), 10)
        
        # Draw Pivot Point
        pygame.draw.circle(self.screen, (0, 0, 0), (cx, cy), 15)

        # Draw Ball
        # Position scaling: ball_pos is +/- 0.15m, beam_length_px is 400px
        ball_offset_px = (self.ball_pos / self.L) * beam_length_px
        bx = cx + ball_offset_px * np.cos(self.beam_angle)
        by = cy + ball_offset_px * np.sin(self.beam_angle) - 15 # Offset above beam
        
        pygame.draw.circle(self.screen, (255, 100, 0), (int(bx), int(by)), 12)

        pygame.display.flip()
        self.clock.tick(self.metadata["render_fps"])

    def close(self):
        if self.screen is not None:
            pygame.display.quit()
            pygame.quit()
            self.screen = None