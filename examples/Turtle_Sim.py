# ====================================================================
# Robot Hard Coded Line with PD Control and Drift Simulation
# ====================================================================

# import turtle
# import time
# import random

# class RobotSimulator:
#     def __init__(self):
#         self.screen = turtle.Screen()
#         self.screen.setup(width=800, height=800, startx=100, starty=100)
#         self.screen.title("Drift Correction Sim")
#         self.screen.bgcolor("#2c3e50")
        
#         self.robot = turtle.Turtle()
#         self.robot.shape("triangle")
#         self.robot.color("#3498db") # Blue robot
        
#         # --- NEW STARTING POSITION ---
#         self.robot.penup()           # Lift pen so it doesn't draw while moving
#         self.robot.setx(-300)      # Move to x = -3000
#         self.robot.sety(0)          # Ensure y is at 0
#         self.robot.pendown()         # Put pen down to start the simulation trail
#         # -----------------------------
#         # State
#         self.yaw = 0.0          # Current heading
#         self.target_yaw = 0.0   # Where we WANT to go (Straight line)
#         # PID Gains
#         self.kp = 0.15   # Reaction strength
#         self.kd = 0.09   # Damping strength (The "Shock Absorber")
        
#         self.last_error = 0.0

#     def apply_drift(self):
#         """Simulates external forces like wind or uneven terrain."""
#         # Randomly nudge the yaw by -2 to +2 degrees
#         drift = random.uniform(-2.0, 2.0)
#         self.yaw += drift

#     def get_pd_correction(self):
#         """Calculates PD Control output."""
#         error = self.target_yaw - self.yaw
#         derivative = error - self.last_error
#         correction = (self.kp * error) + (self.kd * derivative)
#         self.last_error = error
#         return error, correction

#     def update(self, velocity):
#         # 1. Simulate environmental drift
#         self.apply_drift()
        
#         # 2. Calculate and apply correction
#         error, correction = self.get_pd_correction()
#         self.yaw += correction
        
#         # 3. Move the turtle
#         self.robot.setheading(self.yaw)
#         self.robot.forward(velocity)
#         pos = self.robot.pos()
#         # Using \r and end="" to keep the terminal output on one line
#         print(f"Pos: ({pos[0]:>7.2f}, {pos[1]:>7.2f}) | Vel: {velocity} | Yaw Error: {error:>6.2f}°", end="\r")

# def run_simulation():
#     sim = RobotSimulator()
#     print("Driving straight with drift correction enabled... 🌪️ -> ✅")
    
#     # Draw a guide line so we can see the drift
#     guide = turtle.Turtle()
#     guide.hideturtle()
#     guide.pencolor("#ffffff")
#     guide.penup()
#     guide.goto(-300, 0)
#     guide.pendown()
#     guide.goto(300, 0)

#     for _ in range(200):
#         sim.update(velocity=3)
#         time.sleep(0.03)

#     sim.screen.exitonclick()

# if __name__ == "__main__":
#     run_simulation()

# ====================================================================
# Robot Hard Coded Box with PD Control and Drift Simulation
# ====================================================================

# import turtle
# import time
# import random

# class RobotSimulator:
#     def __init__(self):
#         self.screen = turtle.Screen()
#         self.screen.setup(width=800, height=800, startx=100, starty=100)
#         self.screen.title("Box Pattern: PD Drift Correction")
#         self.screen.bgcolor("#2c3e50")
        
#         self.start_x, self.start_y = 300, 0
#         self.waypoints = [
#             (300, 300),    # Corner 1
#             (-100, 300),   # Corner 2
#             (-100, -100),  # Corner 3
#             (300, -100),   # Corner 4
#             (300, 0)       # Back to Home
#         ]
        
#         self.draw_path()

#         self.robot = turtle.Turtle()
#         self.robot.shape("triangle")
#         self.robot.color("#3498db")
#         self.robot.penup()
#         self.robot.goto(self.start_x, self.start_y)
#         self.robot.pendown()
#         self.robot.pensize(2)
        
#         self.current_wp = 0
#         self.yaw = 90.0          
#         self.target_yaw = 90.0   
#         self.kp = 0.3            
#         self.kd = 0.1           
#         self.last_error = 0.0

#     def draw_path(self):
#         path_drawer = turtle.Turtle()
#         path_drawer.hideturtle()
#         path_drawer.speed(0)
#         path_drawer.penup()
#         path_drawer.goto(self.start_x, self.start_y)
#         path_drawer.pendown()
#         path_drawer.pencolor("#7f8c8d")
#         path_drawer.pensize(1)
#         for wp in self.waypoints:
#             path_drawer.goto(wp)

#     def apply_drift(self):
#         drift = random.uniform(-2.5, 2.5)
#         self.yaw += drift

#     def get_pd_correction(self):
#         error = self.target_yaw - self.yaw
#         derivative = error - self.last_error
#         correction = (self.kp * error) + (self.kd * derivative)
#         self.last_error = error
#         return error, correction

#     def update(self, velocity):
#         target_pos = self.waypoints[self.current_wp]
        
#         # Waypoint switching logic
#         if self.robot.distance(target_pos) < 15:
#             if self.current_wp < len(self.waypoints) - 1:
#                 self.current_wp += 1
#                 self.target_yaw += 90.0 
#             else:
#                 # FINAL STOP LOGIC: Check if we are at the very last waypoint
#                 print(f"\nPos: {self.robot.pos()} | Reached final waypoint. Stopping.")
#                 return True 

#         self.apply_drift()
#         error, correction = self.get_pd_correction()
#         self.yaw += correction
        
#         self.robot.setheading(self.yaw)
#         self.robot.forward(velocity)

#         pos = self.robot.pos()
#         print(f"Pos: ({pos[0]:>6.1f}, {pos[1]:>6.1f}) | Vel: {velocity} | Error: {error:>6.2f}°", end="\r")
#         return False

# def run_simulation():
#     sim = RobotSimulator()
    
#     for _ in range(800):
#         # We check if update returns True (the stop signal)
#         finished = sim.update(velocity=4)
#         if finished:
#             break
#         time.sleep(0.02)

#     print("Simulation ended.")
#     sim.screen.exitonclick()

# if __name__ == "__main__":
#     run_simulation()

# ====================================================================
# Robot Hard Coded Trajectory with PD Control and Drift Simulation
# ====================================================================

# import turtle
# import time
# import random
# import math

# class RobotSimulator:
#     def __init__(self, path_points):
#         self.screen = turtle.Screen()
#         self.screen.setup(width=800, height=800, startx=100, starty=100)
#         self.screen.title("Path Following PD Control")
#         self.screen.bgcolor("#2c3e50")
        
#         self.waypoints = path_points
#         self.start_x, self.start_y = self.waypoints[0]
        
#         # 1. Draw the desired trajectory first
#         self.draw_path()

#         # 2. Setup the Robot
#         self.robot = turtle.Turtle()
#         self.robot.shape("triangle")
#         self.robot.color("#3498db")
#         self.robot.penup()
#         self.robot.goto(self.start_x, self.start_y)
#         self.robot.pendown()
#         self.robot.pensize(2)
        
#         # State
#         self.current_wp_idx = 1 # Start chasing the second point
#         self.yaw = 0.0          
#         self.target_yaw = 0.0   
#         self.kp = 0.4           # Higher gain for better path following
#         self.kd = 0.1           
#         self.last_error = 0.0

#     def draw_path(self):
#         path_drawer = turtle.Turtle()
#         path_drawer.hideturtle()
#         path_drawer.speed(0)
#         path_drawer.penup()
#         path_drawer.goto(self.start_x, self.start_y)
#         path_drawer.pendown()
#         path_drawer.pencolor("#7f8c8d")
#         for wp in self.waypoints:
#             path_drawer.goto(wp)

#     def apply_drift(self):
#         # Increased drift to show off the PD correction
#         self.yaw += random.uniform(-3.0, 3.0)

#     def get_pd_correction(self):
#         # Handle the "wrap around" problem (e.g., difference between 350 and 10 degrees)
#         error = (self.target_yaw - self.yaw + 180) % 360 - 180
        
#         derivative = error - self.last_error
#         correction = (self.kp * error) + (self.kd * derivative)
#         self.last_error = error
#         return error, correction

#     def update(self, velocity):
#         if self.current_wp_idx >= len(self.waypoints):
#             return True # Path complete

#         target_pos = self.waypoints[self.current_wp_idx]
        
#         # 1. CALCULATE TARGET YAW (pointing at the next waypoint)
#         dx = target_pos[0] - self.robot.xcor()
#         dy = target_pos[1] - self.robot.ycor()
#         self.target_yaw = math.degrees(math.atan2(dy, dx))

#         # 2. Waypoint switching
#         if self.robot.distance(target_pos) < 15:
#             self.current_wp_idx += 1

#         # 3. Physics & Control
#         self.apply_drift()
#         error, correction = self.get_pd_correction()
#         self.yaw += correction
        
#         self.robot.setheading(self.yaw)
#         self.robot.forward(velocity)

#         # Telemetry
#         print(f"Pos: ({self.robot.xcor():>6.1f}, {self.robot.ycor():>6.1f}) | Target WP: {self.current_wp_idx} | Error: {error:>6.2f}°", end="\r")
#         return False

# def run_simulation():
#     # Define ANY trajectory here (Custom Path)
#     my_trajectory = [
#         (300, 0), (300, 300), (0, 300), (-200, 100), 
#         (-200, -200), (100, -300), (300, 0)
#     ]
    
#     sim = RobotSimulator(my_trajectory)
    
#     # Run until path is finished
#     while True:
#         finished = sim.update(velocity=5)
#         if finished:
#             break
#         time.sleep(0.02)

#     print("\nTrajectory tracking complete.")
#     sim.screen.exitonclick()

# if __name__ == "__main__":
#     run_simulation()

# import turtle
# import time
# import random
# import math

# class RobotSimulator:
#     def __init__(self):
#         self.screen = turtle.Screen()
#         self.screen.setup(width=800, height=800, startx=100, starty=100)
#         self.screen.title("Click to Draw Path - Press SPACE to Start")
#         self.screen.bgcolor("#2c3e50")
        
#         # Lists and State
#         self.waypoints = []
#         self.current_wp_idx = 0
#         self.is_running = False
        
#         # Setup Path Drawer (Hidden)
#         self.path_drawer = turtle.Turtle()
#         self.path_drawer.hideturtle()
#         self.path_drawer.pencolor("#7f8c8d")
#         self.path_drawer.speed(0)
        
#         # Setup Robot
#         self.robot = turtle.Turtle()
#         self.robot.shape("triangle")
#         self.robot.color("#3498db")
#         self.robot.penup()
        
#         # PD Control State
#         self.yaw = 0.0
#         self.target_yaw = 0.0
#         self.kp = 0.4
#         self.kd = 0.1
#         self.last_error = 0.0
#         self.vel = 10
#         # Bind Input Events
#         self.screen.onclick(self.add_waypoint)
#         self.screen.onkey(self.start_sim, "space")
#         self.screen.listen()
        
#         print("INSTRUCTIONS:\n1. Click on the screen to set waypoints.\n2. Press SPACE to start the robot.")

#     def add_waypoint(self, x, y):
#         """Adds a point when the user clicks the screen."""
#         if not self.is_running:
#             if not self.waypoints:
#                 self.path_drawer.penup()
#                 self.path_drawer.goto(x, y)
#                 self.path_drawer.pendown()
#                 self.robot.goto(x, y) # Start robot at first click
            
#             self.waypoints.append((x, y))
#             self.path_drawer.goto(x, y)
#             self.path_drawer.dot(5, "#e74c3c")
#             print(f"Added waypoint: ({x:.1f}, {y:.1f})")

#     def start_sim(self):
#         if self.waypoints and not self.is_running:
#             print("\nStarting Mission! 🚀")
#             self.is_running = True
#             self.robot.pendown()
#             self.run_loop()

#     def apply_drift(self):
#         # Increased drift to show off the PD correction
#         self.yaw += random.uniform(-8.0, 8.0)

#     def get_pd_correction(self):
#         # Shortest path angle math
#         error = (self.target_yaw - self.yaw + 180) % 360 - 180
#         derivative = error - self.last_error
#         correction = (self.kp * error) + (self.kd * derivative)
#         self.last_error = error
#         return error, correction

#     def run_loop(self):
#         """The main simulation loop."""
#         if self.current_wp_idx >= len(self.waypoints):
#             print("\nMission Complete!")
#             return

#         target_pos = self.waypoints[self.current_wp_idx]
        
#         # 1. Point at the waypoint
#         dx = target_pos[0] - self.robot.xcor()
#         dy = target_pos[1] - self.robot.ycor()
#         self.target_yaw = math.degrees(math.atan2(dy, dx))

#         # 2. Check if reached
#         if self.robot.distance(target_pos) < 10:
#             self.current_wp_idx += 1

#         # 3. Control & Movement
#         # self.yaw += random.uniform(-2.5, 2.5) # Drift
#         self.apply_drift()
#         error, correction = self.get_pd_correction()
#         self.yaw += correction
        
#         self.robot.setheading(self.yaw)
#         self.robot.forward(self.vel)

#         # Telemetry
#         print(f"Target Vel: {self.vel} | Target Yaw: {self.target_yaw:>6.1f}° | Pos: ({self.robot.xcor():>6.1f}, {self.robot.ycor():>6.1f}) | Error: {error:>6.2f}°", end="\r")

#         # Schedule the next frame
#         self.screen.ontimer(self.run_loop, 20)

# # Main Execution
# if __name__ == "__main__":
#     sim = RobotSimulator()
#     turtle.done()

import turtle
import time
import random
import math

class RobotSimulator:
    def __init__(self):

        # ---- WORLD PARAMETERS ----
        self.world_x = 18.0   # meters
        self.world_y = 9.0   # meters

        self.window_px = 800
        self.scale = self.window_px / self.world_x   # pixels per meter

        # Robot dimensions
        self.robot_width = 0.4064 * self.scale   # 16 inches
        self.robot_length = 0.2032 * self.scale   # 8 inches

        # ---- SCREEN ----
        self.screen = turtle.Screen()
        self.screen.setup(width=800, height=800, startx=100, starty=100)
        self.screen.title("Click to Draw Path - Press SPACE to Start")
        self.screen.bgcolor("#2c3e50")

        # ---- CUSTOM ROBOT SHAPE ----
        robot_shape = (
            ( self.robot_length/2,  self.robot_width/2),
            ( self.robot_length/2, -self.robot_width/2),
            (-self.robot_length/2, -self.robot_width/2),
            (-self.robot_length/2,  self.robot_width/2)
        )
        self.screen.register_shape("robot_rect", robot_shape)

        # Lists and State
        self.waypoints = []
        self.current_wp_idx = 0
        self.is_running = False

        # ---- PATH DRAWER ----
        self.path_drawer = turtle.Turtle()
        self.path_drawer.hideturtle()
        self.path_drawer.pencolor("#7f8c8d")
        self.path_drawer.speed(0)

        # ---- ROBOT ----
        self.robot = turtle.Turtle()
        self.robot.shape("robot_rect")
        self.robot.color("#3498db")
        self.robot.penup()

        # PD Control State
        self.yaw = 0.0
        self.target_yaw = 0.0
        self.kp = 0.4
        self.kd = 0.1
        self.last_error = 0.0
        self.vel = 0.1 * self.scale   # 0.1 m per step

        # Input
        self.screen.onclick(self.add_waypoint)
        self.screen.onkey(self.start_sim, "space")
        self.screen.listen()

        print("INSTRUCTIONS:")
        print("1. Click to add waypoints.")
        print("2. Press SPACE to start.")

    def add_waypoint(self, x, y):

        if not self.is_running:

            if not self.waypoints:
                self.path_drawer.penup()
                self.path_drawer.goto(x, y)
                self.path_drawer.pendown()
                self.robot.goto(x, y)

            self.waypoints.append((x, y))

            self.path_drawer.goto(x, y)
            self.path_drawer.dot(5, "#e74c3c")

            print(f"Added waypoint: ({x/self.scale:.2f} m, {y/self.scale:.2f} m)")

    def start_sim(self):

        if self.waypoints and not self.is_running:
            print("\nStarting Mission 🚀")
            self.is_running = True
            self.robot.pendown()
            self.run_loop()

    def apply_drift(self):
        self.yaw += random.uniform(-5.0, 5.0)

    def get_pd_correction(self):

        error = (self.target_yaw - self.yaw + 180) % 360 - 180
        derivative = error - self.last_error

        correction = (self.kp * error) + (self.kd * derivative)

        self.last_error = error

        return error, correction

    def run_loop(self):

        if self.current_wp_idx >= len(self.waypoints):
            print("\nMission Complete!")
            return

        target_pos = self.waypoints[self.current_wp_idx]

        dx = target_pos[0] - self.robot.xcor()
        dy = target_pos[1] - self.robot.ycor()

        self.target_yaw = math.degrees(math.atan2(dy, dx))

        if self.robot.distance(target_pos) < 10:
            self.current_wp_idx += 1

        # Drift + PD correction
        self.apply_drift()
        error, correction = self.get_pd_correction()

        self.yaw += correction

        self.robot.setheading(self.yaw)
        self.robot.forward(self.vel)

        print(
            f"Yaw Target: {self.target_yaw:6.1f} | Pos: ({self.robot.xcor()/self.scale:5.2f} m, {self.robot.ycor()/self.scale:5.2f} m) | Error: {error:6.2f}",
            end="\r"
        )

        self.screen.ontimer(self.run_loop, 20)


if __name__ == "__main__":
    sim = RobotSimulator()
    turtle.done()