import numpy as np
from beam import BallAndBeamPIDEnv

def run_visual_test(apply_disturbance = True):  # Set to True to test disturbance handling
    # Set render_mode to "human" to open the window
    env = BallAndBeamPIDEnv(render_mode="human")
    obs, _ = env.reset()
    env.training = False
    # Try the values found to be stable in the paper (PPO/TD3 style)
    # Gains are limited to 0.1 in the study [cite: 89]
    manual_gains = np.array([0.07, 0.01, 0.04]) 

    step = 0
    while step < 1500:
        if apply_disturbance and step == 500:
            print("\n!!! DISTURBANCE INJECTED: Flicking the ball !!!")
            # We bypass the API and modify the physics state directly
            # Adding a sudden velocity of 0.5 m/s to the right
            env.unwrapped.ball_vel += 0.03 
            # Or shift position: env.unwrapped.ball_pos += 0.05

        obs, reward, terminated, truncated, _ = env.step(manual_gains)
        # obs[1] is the ball position [cite: 88]
        if step % 20 == 0:
            print(f"Step {step}: Ball Position = {obs[1]:.4f}m, Reward = {reward:.4f}")

        # This opens the window and draws the current state
        env.render()
        
        if terminated:
            print("Resetting environment...")
            obs, _ = env.reset()
        step += 1
    env.close()

if __name__ == "__main__":
    run_visual_test(apply_disturbance=True)