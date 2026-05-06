import gymnasium as gym
from stable_baselines3 import A2C

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from beam import BallAndBeamPIDEnv  # Assuming your class is in beam.py

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

def train_a2c():
    num_cpu = 4  # Use your 4 cores
    env = SubprocVecEnv([make_env(i) for i in range(num_cpu)])

    model = A2C(
        "MlpPolicy", 
        env, 
        n_steps=128,      # Steps per env before update
        gamma=0.99, 
        learning_rate=7e-4, 
        verbose=1,
        tensorboard_log="./a2c_beam_logs/"
    )
    stop_callback = StopTrainingOnSignal()
    try:
        model.learn(total_timesteps=100000, callback=stop_callback)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected! Stopping training and saving model...")
    finally:
            save_path = "examples/RL_yaw/BEAM/A2C/a2c_ball_beam_model"
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            model.save(save_path)
            print(f"Model saved as {save_path}.zip")


if __name__ == "__main__":
    train_a2c()
