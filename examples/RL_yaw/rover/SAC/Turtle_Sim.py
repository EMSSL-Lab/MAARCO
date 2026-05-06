# import sys
# import os
# import time
# import numpy as np
# import pygame
# from stable_baselines3 import SAC
# sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
# from Rover import RoverPIDEnv 

# def handle_clicks(env):
#     """Checks for mouse clicks and updates the target position."""
#     for event in pygame.event.get():
#         if event.type == pygame.MOUSEBUTTONDOWN:
#             # Get mouse position in pixels
#             mouse_x, mouse_y = pygame.mouse.get_pos()
#             # Update the environment's target
#             env.unwrapped.target_pos = np.array([float(mouse_x), float(mouse_y)])
#             print(f"\n[NEW TARGET] Set to: {env.unwrapped.target_pos}")
#         elif event.type == pygame.QUIT:
#             pygame.quit()
#             sys.exit()

# def test_sac(apply_disturbance=True):
#     pygame.init()
#     pygame.display.set_mode((600, 600)) # Match your Env screen size

#     env = RoverPIDEnv(render_mode="human")
#     model = SAC.load("examples/RL_yaw/rover/SAC/sac_ball_beam_model.zip")

#     obs, _ = env.reset()

#     # NEW: Logic to prevent movement until you click
#     first_click_received = False
#     print("READY: The rover is waiting. CLICK somewhere to set the first target!")

#     step = 0
#     while True:
#         # 1. Check for clicks
#         for event in pygame.event.get():
#             if event.type == pygame.MOUSEBUTTONDOWN:
#                 mouse_x, mouse_y = pygame.mouse.get_pos()
#                 env.unwrapped.target_pos = np.array([float(mouse_x), float(mouse_y)])
                
#                 # IMPORTANT: Recalculate observation immediately after click
#                 obs = env.unwrapped._get_obs() 
                
#                 first_click_received = True
#                 print(f"\n[TARGET SET] {env.unwrapped.target_pos}")
            
#             if event.type == pygame.QUIT:
#                 pygame.quit()
#                 return

#         if not first_click_received:
#             env.render()
#             continue

#         # 2. SAC Prediction and Step (Only runs AFTER first click)
#         action, _ = model.predict(obs, deterministic=True)
#         kp, ki, kd = action

#         #3 env setup
#         obs, reward, terminated, truncated, info = env.step(action)
        
#         # 4. Extract data for printing
#         dist_to_target = obs[0]
#         heading_error_deg = np.degrees(obs[1])

#         # 5. Print Gains every 10 steps
#         if step % 10 == 0:
#             print(f"Step: {step:<4} | Kp: {kp:.4f} | Ki: {ki:.4f} | Kd: {kd:.4f} | Dist: {dist_to_target:.2f} | Err: {heading_error_deg:.1f}°")

#         if terminated:
#             print("\n--- TARGET REACHED! Waiting for next click ---")
#             first_click_received = False 
#             step = 0 # Reset step counter for the next run
#             continue

#         step += 1

# if __name__ == "__main__":
#     test_sac(apply_disturbance=True)

import sys
import os
import time
import numpy as np
import pygame
import math
from stable_baselines3 import SAC

# Ensure this points to your new Rover class
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Rover import RoverPIDEnv 

class SACInteractiveRunner:
    def __init__(self, model_path):
        # 1. CRITICAL FIX: Initialize Pygame and Video system first
        pygame.init()
        self.screen_size = 600
        self.screen = pygame.display.set_mode((self.screen_size, self.screen_size))
        pygame.display.set_caption("SAC Rover: Click Trajectory")
        
        self.env = RoverPIDEnv(render_mode="human")
        self.env.reset()

        # Ensure target_pos exists so the first render doesn't crash
        if not hasattr(self.env.unwrapped, 'target_pos'):
            self.env.unwrapped.target_pos = np.array([300.0, 300.0])
        # --------------------

        self.model = SAC.load(model_path)
        
        self.click_points = []
        self.waypoints = []
        self.current_wp_idx = 0
        self.is_running = False
        self.step_count = 0
        
        # Internal state
        self.dt = 0.1

    def get_angle_between(self, p1, p2, p3):
        a = math.atan2(p1[1]-p2[1], p1[0]-p2[0])
        b = math.atan2(p3[1]-p2[1], p3[0]-p2[0])
        angle = math.degrees(abs(a - b))
        if angle > 180: angle = 360 - angle
        return angle

    def generate_smoothed_path(self):
        if len(self.click_points) < 2:
            self.waypoints = self.click_points[:]
            return

        self.waypoints = [self.click_points[0]]
        for i in range(1, len(self.click_points) - 1):
            p_prev, p_curr, p_next = self.click_points[i-1:i+2]
            turn_angle = 180 - self.get_angle_between(p_prev, p_curr, p_next)

            if turn_angle > 70:
                steps = 10
                m1 = ((p_prev[0]+p_curr[0])/2, (p_prev[1]+p_curr[1])/2)
                m2 = ((p_curr[0]+p_next[0])/2, (p_curr[1]+p_next[1])/2)
                for j in range(steps + 1):
                    t = j / steps
                    x = (1-t)**2 * m1[0] + 2*(1-t)*t * p_curr[0] + t**2 * m2[0]
                    y = (1-t)**2 * m1[1] + 2*(1-t)*t * p_curr[1] + t**2 * m2[1]
                    self.waypoints.append((float(x), float(y)))
            else:
                self.waypoints.append(p_curr)
        
        self.waypoints.append(self.click_points[-1])

    def run(self):
        print("\n--- CONTROLS ---")
        print("LEFT CLICK: Add point")
        print("SPACE:      Start SAC Rover")
        print("R:          Reset Path")
        
        running = True
        while running:
            # 1. Event Handling
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                
                if event.type == pygame.MOUSEBUTTONDOWN and not self.is_running:
                    pos = pygame.mouse.get_pos()
                    self.click_points.append((float(pos[0]), float(pos[1])))
                    self.generate_smoothed_path()
                
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE and self.waypoints:
                        self.is_running = True
                        self.current_wp_idx = 0
                        # Set initial position
                        self.env.unwrapped.rover_pos = np.array(self.waypoints[0], dtype=np.float32)
                        print("\n[STARTING] SAC following trajectory...")
                    
                    if event.key == pygame.K_r:
                        self.is_running = False
                        self.click_points = []
                        self.waypoints = []
                        self.step_count = 0
                        self.env.reset()
                        print("[RESET] Path cleared.")

            # 2. Physics & SAC Logic
            if self.is_running and self.current_wp_idx < len(self.waypoints):
                target = np.array(self.waypoints[self.current_wp_idx])
                self.env.unwrapped.target_pos = target
                
                obs = self.env.unwrapped._get_obs()
                dist_to_wp = obs[0]

                if dist_to_wp < 15:
                    self.current_wp_idx += 1
                else:
                    action, _ = self.model.predict(obs, deterministic=True)
                    kp, ki, kd = action
                    obs, _, _, _, _ = self.env.step(action)
                    
                    if self.step_count % 20 == 0:
                        print(f"WP: {self.current_wp_idx} | Kp: {kp:.3f} | Ki: {ki:.3f} | Kd: {kd:.3f} | Dist: {dist_to_wp:.1f}")
                    self.step_count += 1
            elif self.is_running:
                print("\n--- TRAJECTORY COMPLETE ---")
                self.is_running = False

            # 3. Rendering
            self.render_custom()

        pygame.quit()

    def render_custom(self):
        # Update the environment state visual
        self.env.render()
        
        # Draw our planning lines on top of the environment's surface
        screen = pygame.display.get_surface()
        
        if len(self.waypoints) > 1:
            pygame.draw.lines(screen, (200, 200, 200), False, self.waypoints, 2)
        
        for pt in self.click_points:
            pygame.draw.circle(screen, (231, 76, 60), (int(pt[0]), int(pt[1])), 5)
            
        # pygame.display.flip()

if __name__ == "__main__":
    MODEL_PATH = "examples/RL_yaw/rover/SAC/sac_rover_yaw_model.zip"
    runner = SACInteractiveRunner(MODEL_PATH)
    runner.run()