import numpy as np
from Rover import RoverPIDEnv # Assuming you saved the class in rover_env.py

def run_rover_test(apply_disturbance=True):
    # Initialize the environment with human rendering
    env = RoverPIDEnv(render_mode="human")
    obs, _ = env.reset()
    
    # Manual Gains: 
    # Kp: How hard it turns toward the target
    # Ki: How much it corrects for persistent offset
    # Kd: How much it slows the turn to prevent overshooting
    manual_gains = np.array([0.6, 0.02, 0.15]) 

    print(f"Starting Rover Test. Target Position: {env.target_pos}")

    step = 0
    while step < 1000:
        # Inject disturbances at specific intervals
        if apply_disturbance:
            if step == 300:
                print("\n!!! DISTURBANCE: Sudden side-wind (Heading Shift) !!!")
                env.unwrapped.yaw += np.radians(45) # Instant 45-degree rotation
            
            if step == 600:
                print("\n!!! DISTURBANCE: Magnet drag (Position Shift) !!!")
                env.unwrapped.rover_pos += np.array([40.0, -40.0]) # Instant jump

        # Step the environment
        obs, reward, terminated, truncated, _ = env.step(manual_gains)
        
        # Observation index 0 is distance, index 1 is heading error
        if step % 50 == 0:
            print(f"Step {step:03d} | Dist to Target: {obs[0]:.2f}px | Heading Error: {np.degrees(obs[1]):.2f}°")

        if terminated:
            print("Target Reached or Out of Bounds! Resetting...")
            obs, _ = env.reset()
            # If we reached the target, we might want to stop the test
            if obs[0] < 20: 
                break

        step += 1
        
    env.close()

if __name__ == "__main__":
    run_rover_test(apply_disturbance=True)