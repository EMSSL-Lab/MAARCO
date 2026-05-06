from Rover import BaseRover
from Fuzzy import FuzzyController
from PD import PDController
from STR import STRController
import turtle    

def run_sim():
    rover = BaseRover()
    tuner = FuzzyController()
    tuned = PDController(kp=0.5, kd=0.2)
    str= STRController()
    def step():
        error = rover.get_error()
        if error is not None:
            steering = tuner.compute(error)
            steering2 = tuned.compute(error)
            steer = str.compute(error)

            
            rover.move(steering2)
            turtle.ontimer(step, 20)
        else:
            print("Path Complete.")

    rover.screen.onkey(step, "space")
    rover.screen.listen()
    turtle.done()

if __name__ == "__main__":
    run_sim()