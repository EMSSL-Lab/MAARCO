import pygame
import signal
import sys
import numpy as np
from stable_baselines3.common.callbacks import BaseCallback

class HumanInTheLoopCallback(BaseCallback):
    def __init__(self, verbose=0):
        super(HumanInTheLoopCallback, self).__init__(verbose)
        self.paused = False
        self.stop_training_flag = False
        
        # Ensure Pygame video system is up so we can use event.get()
        if not pygame.display.get_init():
            pygame.display.init()
            # If the window isn't open yet, event.get() will fail on some systems.
            # We don't necessarily need to set_mode here if the Env does it, 
            # but the video system MUST be initialized.

        # 1. Setup Signal Handler for Ctrl+C
        signal.signal(signal.SIGINT, self._signal_handler)
        
        print("\n" + "="*50)
        print(">>> HUMAN-IN-THE-LOOP ACTIVE <<<")
        print(" [P] Pause/Resume Training")
        print(" [F] Flip Robot 180° (Step Response Test)")
        print(" [Ctrl+C] Graceful Stop & Save")
        print("="*50 + "\n")
    
    def _on_training_start(self) -> None:
        """
        This method is called by the model before the first step of training.
        """
        print("\n" + "="*50)
        print(">>> TRAINING INITIALIZED <<<")
        print(f"Algorithm: SAC")
        print(f"Device: {self.model.device}")
        print(f"Buffer size: {self.model.buffer_size}")
        print(f"Learning starts after: {self.model.learning_starts} steps")
        print(f"Initial Entropy Coefficient: {self.model.ent_coef}")
        print("="*50 + "\n")
        
    def _signal_handler(self, sig, frame):
        if not self.stop_training_flag:
            print("\n[SIGNAL] Graceful stop triggered. Finishing current step...")
            self.stop_training_flag = True
        else:
            print("\n[SIGNAL] Force quitting...")
            pygame.quit()
            sys.exit(0)

    # def _on_step(self) -> bool:
    #     # 2. Check for Pygame Keyboard Events
    #     for event in pygame.event.get():
    #         if event.type == pygame.KEYDOWN:
    #             if event.key == pygame.K_p:
    #                 self.paused = not self.paused
    #                 status = "PAUSED" if self.paused else "RESUMED"
    #                 print(f" [TRAIN] {status}")
                
    #             if event.key == pygame.K_f:
    #                 # Accessing the environments to flip them
    #                 # Works with DummyVecEnv
    #                 for i in range(self.training_env.num_envs):
    #                     raw_env = self.training_env.envs[i].unwrapped
    #                     raw_env.yaw += np.pi
    #                     raw_env.target_yaw = raw_env.yaw # Anchor new straight line
    #                     raw_env.integral = 0.0           # Clear I-term for fresh start
    #                 print(" [TRAIN] FLIPPED 180°")

    #     # 3. Handle Pause State
    #     while self.paused and not self.stop_training_flag:
    #         # We still need to check for P or Ctrl+C while paused
    #         for event in pygame.event.get():
    #             if event.type == pygame.KEYDOWN and event.key == pygame.K_p:
    #                 self.paused = False
    #                 print(" [TRAIN] RESUMED")
            
    #         # Keep the render window active so it doesn't freeze
    #         self.training_env.render()
    #         pygame.time.wait(100)

    #     # 4. Return False to stop model.learn() if Ctrl+C was pressed
    #     return not self.stop_training_flag

    def _on_step(self) -> bool:
        # 1. Check for events during normal training
        self.process_events()

        # 2. Handle the Pause Loop
        while self.paused and not self.stop_training_flag:
            # We must process events INSIDE the while loop too!
            self.process_events()
            
            # Keep the window from "Not Responding"
            self.training_env.render()
            pygame.time.wait(100)

        return not self.stop_training_flag

    def process_events(self):
        """Helper to handle keys in both active and paused states"""
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                # --- PAUSE LOGIC ---
                if event.key == pygame.K_p:
                    self.paused = not self.paused
                    print(" [TRAIN] PAUSED" if self.paused else " [TRAIN] RESUMED")
                
                # --- FLIP LOGIC (Now works while paused!) ---
                if event.key == pygame.K_f:
                    for i in range(self.training_env.num_envs):
                        raw_env = self.training_env.envs[i].unwrapped
                        raw_env.yaw += np.pi
                        raw_env.target_yaw = raw_env.yaw
                        raw_env.integral = 0.0
                        # 2. NUDGE back into bounds so the check passes
                        # Move it 10 pixels toward the center
                        if raw_env.rover_pos[0] <= 0: raw_env.rover_pos[0] = 10
                        if raw_env.rover_pos[0] >= 800: raw_env.rover_pos[0] = 790
                        if raw_env.rover_pos[1] <= 0: raw_env.rover_pos[1] = 10
                        if raw_env.rover_pos[1] >= 600: raw_env.rover_pos[1] = 590
                        
                        # 3. Signal the environment to terminate and reset as a flip
                        raw_env.trigger_new_episode = True
                        raw_env.next_reset_is_flip = True
                    
                    print(" [TRAIN] FLIP command queued for next step.")