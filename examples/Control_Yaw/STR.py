import numpy as np

class STRController:
    def __init__(self):
        # --- Parameter Estimation (RLS) State ---
        self.theta = np.array([0.0, 0.0]) # Estimates of [a, b]
        self.P = np.eye(2) * 10.0         # Covariance (uncertainty)
        self.forgetting_factor = 0.95     # How fast to forget old data (0.9-0.99)
        
        # --- Controller State ---
        self.last_output = 0.0
        self.last_input = 0.0
        self.desired_pole = 0.5           # Faster = 0.1, More stable = 0.8
        
    def compute(self, error):
        """
        In STR, we aren't just reacting to error; we are trying to force
        the robot to follow a specific 'Reference Model'.
        """
        if error is None: return 0
        
        # 1. Update the Model (Estimation Step)
        # Current 'behavior' observation
        y = error  # Using error as the output proxy
        phi = np.array([self.last_output, self.last_input])
        
        # RLS Math: Updating our guess of 'a' and 'b'
        # Gain vector K
        denom = self.forgetting_factor + phi.T @ self.P @ phi
        K = (self.P @ phi) / denom
        
        # Update theta (our internal model of physics)
        innovation = y - (phi.T @ self.theta)
        self.theta = self.theta + K * innovation
        
        # Update Covariance matrix
        self.P = (self.P - np.outer(K, phi.T @ self.P)) / self.forgetting_factor
        
        # 2. Design the Controller (The 'Self-Tuning' Step)
        # We want to solve for 'u' such that the next error follows our desired pole
        a_est = self.theta[0]
        b_est = self.theta[1]
        
        # Avoid division by zero if the model hasn't learned yet
        if abs(b_est) < 0.001:
            u = 0.1 * error # Fallback to a tiny P-control
        else:
            # Control Law: u = (Desired_Pole - a_est) * y / b_est
            # This forces the closed-loop system to have the pole we want.
            u = (self.desired_pole - a_est) * y / b_est

        # 3. Save states for next iteration
        self.last_output = y
        self.last_input = u
        
        # Clamp output to prevent 'explosion' during learning
        return max(min(u, 10.0), -10.0)