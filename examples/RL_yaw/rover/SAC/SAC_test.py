import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import time
import numpy as np
from stable_baselines3 import SAC
# Ensure this points to your new Rover class
from Rover_2 import RoverPIDEnv 
import pygame

def test_sac(apply_disturbance = True):
    env = RoverPIDEnv(render_mode="human")
    env.render()

    # Update path to your actual trained Rover model
    model_path = "examples/RL_yaw/rover/SAC/sac_rover_yaw_HITL_model.zip"
    model = SAC.load(model_path)

    obs, _ = env.reset()
    print(f"{'Step':<6} | {'Kp':<8} | {'Ki':<8} | {'Kd':<8} | {'Dist':<8} | {'Err (deg)':<8}")
    print("-" * 70)
    paused = False  
    step = 0

    while True:
        # --- HANDLE KEYBOARD INPUT ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
            
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_p:
                    paused = not paused
                    print("PAUSED" if paused else "RESUMED")
                
                if event.key == pygame.K_f:
                    # Access the internal env to flip the yaw
                    env.unwrapped.yaw += np.pi
                    print("FLIPPED 180 DEGREES")
                    new_goal = (env.unwrapped.yaw + np.pi) % (2 * np.pi) - np.pi
                    env.unwrapped.target_yaw = new_goal
                    
                    # 3. Reset PID internals so it doesn't "remember" the old error
                    env.unwrapped.integral = 0.0
                    env.unwrapped.prev_error = 0.0
                    
                    print(f"Flipped! New goal: {np.degrees(new_goal):.1f}°")

        if not paused:
            # Predict actions using the SAC model
            action, _states = model.predict(obs, deterministic=True)
            kp, ki, kd = action
            
            # 1. Step the environment
            obs, reward, terminated, truncated, info = env.step(action)
            
            # 2. THE FIX: If we hit 1000 steps, reset internals but STAY PUT
            if truncated:
                print(f"\n--- 1000 STEPS REACHED: Clearing PID memory & continuing ---")
                # This calls your custom reset that keeps the rover's position
                obs, _ = env.reset(options={"is_flip": True})
                step = 0 # Reset your local print counter
                continue 

            # 3. Handle actual crashes/boundaries
            if terminated:
                print(f"\n--- OUT OF BOUNDS: Teleporting to Center ---")
                time.sleep(1.0) 
                obs, _ = env.reset() # Standard reset (teleports to start)
                step = 0
                continue
                
            step += 1
    env.close()

if __name__ == "__main__":
    test_sac(apply_disturbance = True)