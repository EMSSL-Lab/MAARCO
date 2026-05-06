import gymnasium as gym
from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from beam import BallAndBeamPIDEnv  # Assuming your class is in beam.py
import numpy as np
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.callbacks import BaseCallback
import signal

class StopTrainingOnSignal(BaseCallback):
    def __init__(self, verbose=0):
        super(StopTrainingOnSignal, self).__init__(verbose)
        self.stop_training = False
        # Catch the Ctrl+C signal (SIGINT)
        signal.signal(signal.SIGINT, self._signal_handler)
        print("\n>>> Press Ctrl+C ONCE to stop training and save. Press twice to force quit. <<<\n")

    def _signal_handler(self, sig, frame):
        if not self.stop_training:
            print("\nStopping training gracefully... please wait for the current step to finish.")
            self.stop_training = True
        else:
            print("Force quitting...")
            sys.exit(0)

    def _on_step(self) -> bool:
        # If signal was caught, returning False stops model.learn()
        return not self.stop_training

def make_env(rank, seed=0):
    def _init():
        env = BallAndBeamPIDEnv(render_mode=None)  # No rendering during training
        return env
    set_random_seed(seed)
    return _init

def train_td3():
    num_cpu = 4  # Use your 4 cores
    env = SubprocVecEnv([make_env(i) for i in range(num_cpu)])

    # TD3 Exploration Noise
    n_actions = env.action_space.shape[-1]
    
    # Scale noise to 10% of the gain range (0.1 * 0.1 = 0.01)
    action_noise = NormalActionNoise(
        mean=np.zeros(n_actions), 
        sigma=0.01 * np.ones(n_actions)
    )

    model = TD3(
        "MlpPolicy", 
        env, 
        action_noise=action_noise,
        batch_size=256,         # Slightly smaller batch often helps TD3 stability
        learning_starts=1000,   # Collect 1000 steps of random data first
        tau=0.005,              # Soft update coefficient
        train_freq=1,           # Update every step
        gradient_steps=1,
        verbose=1,
        tensorboard_log="./td3_beam_logs/"
    )
    
    stop_callback = StopTrainingOnSignal()
    
    try:
        model.learn(total_timesteps=100000, callback=stop_callback)
    except KeyboardInterrupt:
        pass
    finally:
        save_path = "examples/RL_yaw/BEAM/TD3/td3_ball_beam_model"
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        model.save(save_path)
        env.close()
        print(f"Model saved to {save_path}")


if __name__ == "__main__":
    train_td3()
