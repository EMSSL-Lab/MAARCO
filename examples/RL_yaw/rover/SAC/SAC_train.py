import gymnasium as gym
from stable_baselines3 import SAC
from HITL import HumanInTheLoopCallback
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Rover_2 import RoverPIDEnv  # Assuming your class is in beam.py
from stable_baselines3.common.vec_env import DummyVecEnv
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
        # env = RoverPIDEnv(render_mode=None)  # No rendering during training
        env = RoverPIDEnv(render_mode="human")  # No rendering during training
        env.render()
        return env
    set_random_seed(seed)
    return _init

def train_sac():

    # num_cpu = 1 
    # env = SubprocVecEnv([make_env(i) for i in range(num_cpu)])
    # env = DummyVecEnv([lambda: RoverPIDEnv(render_mode="human")])
    
    model_path = "examples/RL_yaw/rover/SAC/sac_rover_yaw_HITL_model"
    env = DummyVecEnv([lambda: RoverPIDEnv(render_mode="human")])
    env.render()

    # Check if a saved model exists to continue training
    if os.path.exists(model_path + ".zip"):
        print(f"Loading existing model from {model_path}...")
        # Load the model and connect it to the current environment
        model = SAC.load(model_path, env=env)
    else:
        print("No existing model found. Starting training from scratch.")
        model = SAC(
            "MlpPolicy", 
            env, 
            batch_size=512, 
            verbose=1
        )

    # stop_callback = StopTrainingOnSignal()
    hitl_callback = HumanInTheLoopCallback()
    print("\n>>> HITL ACTIVE: Press [P] to Pause, [F] to Flip during training <<<\n")
    
    try:
        # This will now pick up where the loaded model left off
        model.learn(total_timesteps=100000, callback=hitl_callback, log_interval=1,reset_num_timesteps=False)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected! Saving...")
    finally:
        model.save(model_path)
        print(f"Model saved to {model_path}.zip")


if __name__ == "__main__":
    train_sac()
