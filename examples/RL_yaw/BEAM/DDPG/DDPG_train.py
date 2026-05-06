import gymnasium as gym
from stable_baselines3 import DDPG  # Changed from SAC

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from beam import BallAndBeamPIDEnv 

from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.noise import NormalActionNoise
import signal
import numpy as np
class StopTrainingOnSignal(BaseCallback):
    def __init__(self, verbose=0):
        super(StopTrainingOnSignal, self).__init__(verbose)
        self.stop_training = False
        signal.signal(signal.SIGINT, self._signal_handler)
        print("\n>>> DDPG TRAINING STARTED. Press Ctrl+C ONCE to stop and save. <<<\n")

    def _signal_handler(self, sig, frame):
        if not self.stop_training:
            print("\nStopping training gracefully...")
            self.stop_training = True
        else:
            print("Force quitting...")
            sys.exit(0)

    def _on_step(self) -> bool:
        # 'infos' is a list of dictionaries (one for each CPU core)
        for i, info_dict in enumerate(self.locals['infos']):
            if info_dict.get("flick"):
                # Use total_timesteps to know when it happened
                print(f"Step {self.num_timesteps} | Core {i}: Disturbance detected!")
        return not self.stop_training

def make_env(rank, seed=0):
    def _init():
        # Important: Some PPO versions prefer a reset seed
        env = BallAndBeamPIDEnv(render_mode=None)
        return env
    set_random_seed(seed)
    return _init

def train_DDPG():
    num_cpu = 4 
    env = SubprocVecEnv([make_env(i) for i in range(num_cpu)])

    # 1. DDPG MUST have noise to explore
    n_actions = env.action_space.shape[-1]
    action_noise = NormalActionNoise(
        mean=np.zeros(n_actions), 
        sigma=0.1 * np.ones(n_actions)
    )

    model = DDPG(
        "MlpPolicy", 
        env, 
        action_noise=action_noise, # Crucial for DDPG
        batch_size=256, 
        tau=0.005,                 # Soft update coefficient
        learning_rate=1e-3,
        verbose=1,
        tensorboard_log="./ddpg_beam_tensorboard/"
    )
    
    stop_callback = StopTrainingOnSignal()
    
    try:
        model.learn(total_timesteps=200000, callback=stop_callback)
    except KeyboardInterrupt:
        pass 
    finally:
        save_path = "examples/RL_yaw/BEAM/DDPG/DDPG_ball_beam_model"
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        model.save(save_path)
        env.close()
        print(f"Model saved as {save_path}.zip")

if __name__ == "__main__":
    train_DDPG()