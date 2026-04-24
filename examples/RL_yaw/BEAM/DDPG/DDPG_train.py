import gymnasium as gym
from stable_baselines3 import SAC

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

def train_sac():
    num_cpu = 4  # Use your 4 cores
    env = SubprocVecEnv([make_env(i) for i in range(num_cpu)])

    model = SAC("MlpPolicy", env, batch_size= 512, verbose=1)
    stop_callback = StopTrainingOnSignal()
    try:
        model.learn(total_timesteps=100000, callback=stop_callback)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected! Stopping training and saving model...")
    finally:
        model.save("examples/RL_yaw/BEAM/SAC/sac_ball_beam_model")
        print("Model saved as sac_ball_beam_model.zip")


if __name__ == "__main__":
    train_sac()
