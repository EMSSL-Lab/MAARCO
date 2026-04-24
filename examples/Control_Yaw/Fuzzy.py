class FuzzyController:
    def __init__(self):
        self.last_error = 0

    def compute(self, error):
        if error is None: return 0
        
        delta_error = error - self.last_error
        self.last_error = error
        
        # MOCK FUZZY LOGIC (Simplified Membership)
        # In a real setup, we'd use 'skfuzzy' or define triangle functions
        if abs(error) > 20:
            output = 5.0 if error > 0 else -5.0 # "Hard Turn"
        elif abs(error) > 5:
            output = 2.0 if error > 0 else -2.0 # "Soft Turn"
        else:
            output = 0.5 if error > 0 else -0.5 # "Correcting"
            
        return output
