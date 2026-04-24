class PDController:
    def __init__(self, kp=0.6, kd=0.3):
        self.kp = kp
        self.kd = kd
        self.last_error = 0.0

    def compute(self, error):
        if error is None: 
            return 0
        
        # Calculate the rate of change (derivative)
        delta_error = error - self.last_error
        
        # PD Formula: Output = (Proportional * Error) + (Derivative * Delta)
        output = (self.kp * error) + (self.kd * delta_error)
        
        # Store error for the next time step
        self.last_error = error
        
        return output