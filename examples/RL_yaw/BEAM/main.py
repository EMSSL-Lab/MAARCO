import numpy as np
from beam import BallAndBeamPIDEnv

def run_visual_test():
    # Set render_mode to "human" to open the window
    env = BallAndBeamPIDEnv(render_mode="human")
    obs, _ = env.reset()
    
    # Try the values found to be stable in the paper (PPO/TD3 style)
    # Gains are limited to 0.1 in the study [cite: 89]
    manual_gains = np.array([0.07, 0.01, 0.04]) 

    for step in range(1000):
        obs, reward, terminated, truncated, _ = env.step(manual_gains)

        # obs[1] is the ball position [cite: 88]
        if step % 20 == 0:
            print(f"Step {step}: Ball Position = {obs[1]:.4f}m, Reward = {reward:.4f}")

        # This opens the window and draws the current state
        env.render()
        
        if terminated or truncated:
            print("Resetting environment...")
            obs, _ = env.reset()
            
    env.close()

if __name__ == "__main__":
    run_visual_test()