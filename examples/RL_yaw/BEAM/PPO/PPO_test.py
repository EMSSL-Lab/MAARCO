import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from beam import BallAndBeamPIDEnv  # Assuming your class is in beam.py

import time
from stable_baselines3 import PPO  # Changed from SAC

def test_ppo(apply_disturbance=True):
    # Load environment with rendering enabled
    env = BallAndBeamPIDEnv(render_mode="human")
    
    # Load the trained model
    model = PPO.load("examples/RL_yaw/BEAM/PPO/ppo_ball_beam_model")

    env.training = False
    
    obs, _ = env.reset()
    print(f"{'Step':<6} | {'Kp':<8} | {'Ki':<8} | {'Kd':<8} | {'Ball Pos':<8}")
    print("-" * 50)

    step = 0
    while step < 1500:
        # 1. Inject Disturbance at 10 seconds (Step 500)
        if apply_disturbance and step == 500:
            print("\n!!! DISTURBANCE INJECTED: Flicking the ball !!!")
            # We bypass the API and modify the physics state directly
            # Adding a sudden velocity of 0.5 m/s to the right
            env.unwrapped.ball_vel += 0.15 
            # Or shift position: env.unwrapped.ball_pos += 0.05

        # The action IS the set of PID gains
        action, _states = model.predict(obs, deterministic=True)
        kp, ki, kd = action
        
        obs, reward, terminated, truncated, info = env.step(action)
        
        # obs[1] is the ball position from our _get_obs method
        ball_pos = obs[1]

        # Print the gains every 5 steps so it's readable
        if step % 5 == 0:
            print(f"{step:<6} | {kp:.4f} | {ki:.4f} | {kd:.4f} | {ball_pos:.4f}m")
        
        if terminated or truncated:
            # Render one last time and pause so we can see the failure
            env.render()
            print(f"Fail at step {step}. Ball Pos: {ball_pos:.4f}m")
            time.sleep(1.0) 
            print("\n--- Resetting Environment ---")
            obs, _ = env.reset()
            step = 0  # Reset step count after episode ends
            continue

        step += 1
    env.close()

if __name__ == "__main__":
    # Uncomment to train first, then test
    # train_ppo()
    test_ppo(apply_disturbance=True)