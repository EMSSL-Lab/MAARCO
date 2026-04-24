import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from beam import BallAndBeamPIDEnv  # Assuming your class is in beam.py

from stable_baselines3 import SAC

def test_sac():
    # Load environment with rendering enabled
    env = BallAndBeamPIDEnv(render_mode="human")
    
    # Load the trained model
    model = SAC.load("examples/RL_yaw/BEAM/SAC/sac_ball_beam_model")

    obs, _ = env.reset()
    print(f"{'Step':<6} | {'Kp':<8} | {'Ki':<8} | {'Kd':<8} | {'Ball Pos':<8}")
    print("-" * 50)

    for step in range(1000):
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
            print("\n--- Resetting Environment ---")
            obs, _ = env.reset()

    env.close()

if __name__ == "__main__":
    # Uncomment to train first, then test
    # train_sac()
    test_sac()