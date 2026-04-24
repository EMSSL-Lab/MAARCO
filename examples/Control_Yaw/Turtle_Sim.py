# # ====================================================================
# # Robot Hard Coded Line with PD Control and Drift Simulation
# # ====================================================================

# # import turtle
# # import time
# # import random

# # class RobotSimulator:
# #     def __init__(self):
# #         self.screen = turtle.Screen()
# #         self.screen.setup(width=800, height=800, startx=100, starty=100)
# #         self.screen.title("Drift Correction Sim")
# #         self.screen.bgcolor("#2c3e50")
        
# #         self.robot = turtle.Turtle()
# #         self.robot.shape("triangle")
# #         self.robot.color("#3498db") # Blue robot
        
# #         # --- NEW STARTING POSITION ---
# #         self.robot.penup()           # Lift pen so it doesn't draw while moving
# #         self.robot.setx(-300)      # Move to x = -3000
# #         self.robot.sety(0)          # Ensure y is at 0
# #         self.robot.pendown()         # Put pen down to start the simulation trail
# #         # -----------------------------
# #         # State
# #         self.yaw = 0.0          # Current heading
# #         self.target_yaw = 0.0   # Where we WANT to go (Straight line)
# #         # PID Gains
# #         self.kp = 0.15   # Reaction strength
# #         self.kd = 0.09   # Damping strength (The "Shock Absorber")
        
# #         self.last_error = 0.0

# #     def apply_drift(self):
# #         """Simulates external forces like wind or uneven terrain."""
# #         # Randomly nudge the yaw by -2 to +2 degrees
# #         drift = random.uniform(-2.0, 2.0)
# #         self.yaw += drift

# #     def get_pd_correction(self):
# #         """Calculates PD Control output."""
# #         error = self.target_yaw - self.yaw
# #         derivative = error - self.last_error
# #         correction = (self.kp * error) + (self.kd * derivative)
# #         self.last_error = error
# #         return error, correction

# #     def update(self, velocity):
# #         # 1. Simulate environmental drift
# #         self.apply_drift()
        
# #         # 2. Calculate and apply correction
# #         error, correction = self.get_pd_correction()
# #         self.yaw += correction
        
# #         # 3. Move the turtle
# #         self.robot.setheading(self.yaw)
# #         self.robot.forward(velocity)
# #         pos = self.robot.pos()
# #         # Using \r and end="" to keep the terminal output on one line
# #         print(f"Pos: ({pos[0]:>7.2f}, {pos[1]:>7.2f}) | Vel: {velocity} | Yaw Error: {error:>6.2f}°", end="\r")

# # def run_simulation():
# #     sim = RobotSimulator()
# #     print("Driving straight with drift correction enabled... 🌪️ -> ✅")
    
# #     # Draw a guide line so we can see the drift
# #     guide = turtle.Turtle()
# #     guide.hideturtle()
# #     guide.pencolor("#ffffff")
# #     guide.penup()
# #     guide.goto(-300, 0)
# #     guide.pendown()
# #     guide.goto(300, 0)

# #     for _ in range(200):
# #         sim.update(velocity=3)
# #         time.sleep(0.03)

# #     sim.screen.exitonclick()

# # if __name__ == "__main__":
# #     run_simulation()

# # ====================================================================
# # Robot Hard Coded Box with PD Control and Drift Simulation
# # ====================================================================

# # import turtle
# # import time
# # import random

# # class RobotSimulator:
# #     def __init__(self):
# #         self.screen = turtle.Screen()
# #         self.screen.setup(width=800, height=800, startx=100, starty=100)
# #         self.screen.title("Box Pattern: PD Drift Correction")
# #         self.screen.bgcolor("#2c3e50")
        
# #         self.start_x, self.start_y = 300, 0
# #         self.waypoints = [
# #             (300, 300),    # Corner 1
# #             (-100, 300),   # Corner 2
# #             (-100, -100),  # Corner 3
# #             (300, -100),   # Corner 4
# #             (300, 0)       # Back to Home
# #         ]
        
# #         self.draw_path()

# #         self.robot = turtle.Turtle()
# #         self.robot.shape("triangle")
# #         self.robot.color("#3498db")
# #         self.robot.penup()
# #         self.robot.goto(self.start_x, self.start_y)
# #         self.robot.pendown()
# #         self.robot.pensize(2)
        
# #         self.current_wp = 0
# #         self.yaw = 90.0          
# #         self.target_yaw = 90.0   
# #         self.kp = 0.3            
# #         self.kd = 0.1           
# #         self.last_error = 0.0

# #     def draw_path(self):
# #         path_drawer = turtle.Turtle()
# #         path_drawer.hideturtle()
# #         path_drawer.speed(0)
# #         path_drawer.penup()
# #         path_drawer.goto(self.start_x, self.start_y)
# #         path_drawer.pendown()
# #         path_drawer.pencolor("#7f8c8d")
# #         path_drawer.pensize(1)
# #         for wp in self.waypoints:
# #             path_drawer.goto(wp)

# #     def apply_drift(self):
# #         drift = random.uniform(-2.5, 2.5)
# #         self.yaw += drift

# #     def get_pd_correction(self):
# #         error = self.target_yaw - self.yaw
# #         derivative = error - self.last_error
# #         correction = (self.kp * error) + (self.kd * derivative)
# #         self.last_error = error
# #         return error, correction

# #     def update(self, velocity):
# #         target_pos = self.waypoints[self.current_wp]
        
# #         # Waypoint switching logic
# #         if self.robot.distance(target_pos) < 15:
# #             if self.current_wp < len(self.waypoints) - 1:
# #                 self.current_wp += 1
# #                 self.target_yaw += 90.0 
# #             else:
# #                 # FINAL STOP LOGIC: Check if we are at the very last waypoint
# #                 print(f"\nPos: {self.robot.pos()} | Reached final waypoint. Stopping.")
# #                 return True 

# #         self.apply_drift()
# #         error, correction = self.get_pd_correction()
# #         self.yaw += correction
        
# #         self.robot.setheading(self.yaw)
# #         self.robot.forward(velocity)

# #         pos = self.robot.pos()
# #         print(f"Pos: ({pos[0]:>6.1f}, {pos[1]:>6.1f}) | Vel: {velocity} | Error: {error:>6.2f}°", end="\r")
# #         return False

# # def run_simulation():
# #     sim = RobotSimulator()
    
# #     for _ in range(800):
# #         # We check if update returns True (the stop signal)
# #         finished = sim.update(velocity=4)
# #         if finished:
# #             break
# #         time.sleep(0.02)

# #     print("Simulation ended.")
# #     sim.screen.exitonclick()

# # if __name__ == "__main__":
# #     run_simulation()

# # ====================================================================
# # Robot Hard Coded Trajectory with PD Control and Drift Simulation
# # ====================================================================

# # import turtle
# # import time
# # import random
# # import math

# # class RobotSimulator:
# #     def __init__(self, path_points):
# #         self.screen = turtle.Screen()
# #         self.screen.setup(width=800, height=800, startx=100, starty=100)
# #         self.screen.title("Path Following PD Control")
# #         self.screen.bgcolor("#2c3e50")
        
# #         self.waypoints = path_points
# #         self.start_x, self.start_y = self.waypoints[0]
        
# #         # 1. Draw the desired trajectory first
# #         self.draw_path()

# #         # 2. Setup the Robot
# #         self.robot = turtle.Turtle()
# #         self.robot.shape("triangle")
# #         self.robot.color("#3498db")
# #         self.robot.penup()
# #         self.robot.goto(self.start_x, self.start_y)
# #         self.robot.pendown()
# #         self.robot.pensize(2)
        
# #         # State
# #         self.current_wp_idx = 1 # Start chasing the second point
# #         self.yaw = 0.0          
# #         self.target_yaw = 0.0   
# #         self.kp = 0.4           # Higher gain for better path following
# #         self.kd = 0.1           
# #         self.last_error = 0.0

# #     def draw_path(self):
# #         path_drawer = turtle.Turtle()
# #         path_drawer.hideturtle()
# #         path_drawer.speed(0)
# #         path_drawer.penup()
# #         path_drawer.goto(self.start_x, self.start_y)
# #         path_drawer.pendown()
# #         path_drawer.pencolor("#7f8c8d")
# #         for wp in self.waypoints:
# #             path_drawer.goto(wp)

# #     def apply_drift(self):
# #         # Increased drift to show off the PD correction
# #         self.yaw += random.uniform(-3.0, 3.0)

# #     def get_pd_correction(self):
# #         # Handle the "wrap around" problem (e.g., difference between 350 and 10 degrees)
# #         error = (self.target_yaw - self.yaw + 180) % 360 - 180
        
# #         derivative = error - self.last_error
# #         correction = (self.kp * error) + (self.kd * derivative)
# #         self.last_error = error
# #         return error, correction

# #     def update(self, velocity):
# #         if self.current_wp_idx >= len(self.waypoints):
# #             return True # Path complete

# #         target_pos = self.waypoints[self.current_wp_idx]
        
# #         # 1. CALCULATE TARGET YAW (pointing at the next waypoint)
# #         dx = target_pos[0] - self.robot.xcor()
# #         dy = target_pos[1] - self.robot.ycor()
# #         self.target_yaw = math.degrees(math.atan2(dy, dx))

# #         # 2. Waypoint switching
# #         if self.robot.distance(target_pos) < 15:
# #             self.current_wp_idx += 1

# #         # 3. Physics & Control
# #         self.apply_drift()
# #         error, correction = self.get_pd_correction()
# #         self.yaw += correction
        
# #         self.robot.setheading(self.yaw)
# #         self.robot.forward(velocity)

# #         # Telemetry
# #         print(f"Pos: ({self.robot.xcor():>6.1f}, {self.robot.ycor():>6.1f}) | Target WP: {self.current_wp_idx} | Error: {error:>6.2f}°", end="\r")
# #         return False

# # def run_simulation():
# #     # Define ANY trajectory here (Custom Path)
# #     my_trajectory = [
# #         (300, 0), (300, 300), (0, 300), (-200, 100), 
# #         (-200, -200), (100, -300), (300, 0)
# #     ]
    
# #     sim = RobotSimulator(my_trajectory)
    
# #     # Run until path is finished
# #     while True:
# #         finished = sim.update(velocity=5)
# #         if finished:
# #             break
# #         time.sleep(0.02)

# #     print("\nTrajectory tracking complete.")
# #     sim.screen.exitonclick()

# # if __name__ == "__main__":
# #     run_simulation()

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


# import turtle
# import time
# import random
# import math

# class RobotSimulator:
#     def __init__(self):

#         # ---- WORLD PARAMETERS ----
#         self.world_x = 18.0   # meters
#         self.world_y = 9.0   # meters

#         self.window_px = 800
#         self.scale = self.window_px / self.world_x   # pixels per meter

#         # Robot dimensions
#         self.robot_width = 0.4064 * self.scale   # 16 inches
#         self.robot_length = 0.2032 * self.scale   # 8 inches

#         # ---- SCREEN ----
#         self.screen = turtle.Screen()
#         self.screen.setup(width=800, height=800, startx=100, starty=100)
#         self.screen.title("Click to Draw Path - Press SPACE to Start")
#         self.screen.bgcolor("#2c3e50")

#         # ---- CUSTOM ROBOT SHAPE ----
#         robot_shape = (
#             ( self.robot_length/2,  self.robot_width/2),
#             ( self.robot_length/2, -self.robot_width/2),
#             (-self.robot_length/2, -self.robot_width/2),
#             (-self.robot_length/2,  self.robot_width/2)
#         )
#         self.screen.register_shape("robot_rect", robot_shape)

#         # Lists and State
#         self.waypoints = []
#         self.current_wp_idx = 0
#         self.is_running = False

#         # ---- PATH DRAWER ----
#         self.path_drawer = turtle.Turtle()
#         self.path_drawer.hideturtle()
#         self.path_drawer.pencolor("#7f8c8d")
#         self.path_drawer.speed(0)

#         # ---- ROBOT ----
#         self.robot = turtle.Turtle()
#         self.robot.shape("robot_rect")
#         self.robot.color("#3498db")
#         self.robot.penup()

#         # PD Control State
#         self.yaw = 0.0
#         self.target_yaw = 0.0
#         self.kp = 0.4
#         self.kd = 0.1
#         self.last_error = 0.0
#         self.vel = 0.1 * self.scale   # 0.1 m per step

#         # Input
#         self.screen.onclick(self.add_waypoint)
#         self.screen.onkey(self.start_sim, "space")
#         self.screen.listen()

#         print("INSTRUCTIONS:")
#         print("1. Click to add waypoints.")
#         print("2. Press SPACE to start.")

#     def add_waypoint(self, x, y):

#         if not self.is_running:

#             if not self.waypoints:
#                 self.path_drawer.penup()
#                 self.path_drawer.goto(x, y)
#                 self.path_drawer.pendown()
#                 self.robot.goto(x, y)

#             self.waypoints.append((x, y))

#             self.path_drawer.goto(x, y)
#             self.path_drawer.dot(5, "#e74c3c")

#             print(f"Added waypoint: ({x/self.scale:.2f} m, {y/self.scale:.2f} m)")

#     def start_sim(self):

#         if self.waypoints and not self.is_running:
#             print("\nStarting Mission 🚀")
#             self.is_running = True
#             self.robot.pendown()
#             self.run_loop()

#     def apply_drift(self):
#         self.yaw += random.uniform(-5.0, 5.0)

#     def get_pd_correction(self):

#         error = (self.target_yaw - self.yaw + 180) % 360 - 180
#         derivative = error - self.last_error

#         correction = (self.kp * error) + (self.kd * derivative)

#         self.last_error = error

#         return error, correction

#     def run_loop(self):

#         if self.current_wp_idx >= len(self.waypoints):
#             print("\nMission Complete!")
#             return

#         target_pos = self.waypoints[self.current_wp_idx]

#         dx = target_pos[0] - self.robot.xcor()
#         dy = target_pos[1] - self.robot.ycor()

#         self.target_yaw = math.degrees(math.atan2(dy, dx))

#         if self.robot.distance(target_pos) < 10:
#             self.current_wp_idx += 1

#         # Drift + PD correction
#         self.apply_drift()
#         error, correction = self.get_pd_correction()

#         self.yaw += correction

#         self.robot.setheading(self.yaw)
#         self.robot.forward(self.vel)

#         print(
#             f"Yaw Target: {self.target_yaw:6.1f} | Pos: ({self.robot.xcor()/self.scale:5.2f} m, {self.robot.ycor()/self.scale:5.2f} m) | Error: {error:6.2f}",
#             end="\r"
#         )

#         self.screen.ontimer(self.run_loop, 20)


# if __name__ == "__main__":
#     sim = RobotSimulator()
#     turtle.done()


# import turtle
# import time
# import random
# import math

# class RobotSimulator:
#     def __init__(self):
#         # ---- WORLD PARAMETERS ----
#         self.world_size = 10.0  # 20m total width/height
#         self.window_px = 800
        
#         # Set coordinates so (-10, -10) is bottom-left and (10, 10) is top-right
#         self.screen = turtle.Screen()
#         self.screen.setup(width=self.window_px, height=self.window_px)
#         self.screen.setworldcoordinates(-self.world_size/2, -self.world_size/2, 
#                                          self.world_size/2, self.world_size/2)
#         self.screen.title("Robot PD Sim: 20x20m Grid")
#         self.screen.bgcolor("#2c3e50")
#         self.screen.tracer(0) # Turn off animation for setup

#         # ---- DRAW GRID ----
#         self.grid_tool = turtle.Turtle()
#         self.grid_tool.hideturtle()
#         self.draw_grid()

#         # Robot dimensions in meters
#         self.robot_width = 0.4064  # 16 inches
#         self.robot_length = 0.2032 # 8 inches



#         # ---- CUSTOM ROBOT SHAPE ----
#         # Since we use setworldcoordinates, we define the shape in meters
#         robot_shape = (
#             ( self.robot_length/2,  self.robot_width/2),
#             ( self.robot_length/2, -self.robot_width/2),
#             (-self.robot_length/2, -self.robot_width/2),
#             (-self.robot_length/2,  self.robot_width/2)
#         )
#         self.screen.register_shape("robot_rect", robot_shape)

#         # Lists and State
#         self.waypoints = []
#         self.current_wp_idx = 0
#         self.is_running = False

#         # ---- PATH DRAWER ----
#         self.path_drawer = turtle.Turtle()
#         self.path_drawer.hideturtle()
#         self.path_drawer.pencolor("#e74c3c")
#         self.path_drawer.width(2)

#         # ---- ROBOT ----
#         self.robot = turtle.Turtle()
#         self.robot.shape("robot_rect")
#         self.robot.color("#3498db")
#         self.robot.penup()

#         # PD Control State (Gains adjusted for meter-scale)
#         self.yaw = 0.0
#         self.target_yaw = 0.0
#         self.kp = 0.4
#         self.kd = 0.1
#         self.last_error = 0.0
#         self.vel = 0.1 # 0.1 meters per step

#         self.screen.update()
#         self.screen.tracer(1) # Turn animation back on

#         # Input
#         self.screen.onclick(self.add_waypoint)
#         self.screen.onkey(self.start_sim, "space")
#         self.screen.listen()

#     def draw_grid(self):
#         self.grid_tool.penup()
#         self.grid_tool.pencolor("#34495e")
        
#         # Draw vertical and horizontal lines every 1 meter
#         for i in range(int(-self.world_size/2), int(self.world_size/2) + 1):
#             # Vertical
#             self.grid_tool.goto(i, -self.world_size/2)
#             self.grid_tool.pendown()
#             self.grid_tool.goto(i, self.world_size/2)
#             self.grid_tool.penup()
#             # Horizontal
#             self.grid_tool.goto(-self.world_size/2, i)
#             self.grid_tool.pendown()
#             self.grid_tool.goto(self.world_size/2, i)
#             self.grid_tool.penup()

#         # Draw axis labels every 5 meters
#         self.grid_tool.pencolor("#95a5a6")
#         for i in range(int(-self.world_size/2), int(self.world_size/2) + 1, 5):
#             self.grid_tool.goto(i, 0.2)
#             self.grid_tool.write(f"{i}m", align="center", font=("Arial", 10, "bold"))
#             self.grid_tool.goto(0.2, i)
#             self.grid_tool.write(f"{i}m", align="left", font=("Arial", 10, "bold"))

#     def add_waypoint(self, x, y):
#         if not self.is_running:
#             if not self.waypoints:
#                 self.path_drawer.penup()
#                 self.path_drawer.goto(x, y)
#                 self.path_drawer.pendown()
#                 self.robot.goto(x, y)

#             self.waypoints.append((x, y))
#             self.path_drawer.goto(x, y)
#             self.path_drawer.dot(6, "#e74c3c")
#             print(f"Added waypoint: ({x:.2f} m, {y:.2f} m)")

#     def start_sim(self):
#         if self.waypoints and not self.is_running:
#             print("\nStarting Mission 🚀")
#             self.is_running = True
#             self.robot.pendown()
#             self.run_loop()

#     def apply_drift(self):
#         self.yaw += random.uniform(-4.0, 4.0)

#     def get_pd_correction(self):
#         error = (self.target_yaw - self.yaw + 180) % 360 - 180
#         derivative = error - self.last_error
#         correction = (self.kp * error) + (self.kd * derivative)
#         self.last_error = error
#         return error, correction

#     def run_loop(self):
#         if self.current_wp_idx >= len(self.waypoints):
#             print("\nMission Complete!")
#             return

#         target_pos = self.waypoints[self.current_wp_idx]
#         dx = target_pos[0] - self.robot.xcor()
#         dy = target_pos[1] - self.robot.ycor()

#         self.target_yaw = math.degrees(math.atan2(dy, dx))

#         # Check distance in meters
#         if self.robot.distance(target_pos) < 0.05:
#             self.current_wp_idx += 1

#         self.apply_drift()
#         error, correction = self.get_pd_correction()
#         self.yaw += correction

#         self.robot.setheading(self.yaw)
#         self.robot.forward(self.vel)

#         print(f"Target Yaw: {self.target_yaw:6.1f}° | Pos: ({self.robot.xcor():5.2f}m, {self.robot.ycor():5.2f}m) | Error: {error:6.2f}", end="\r")
#         self.screen.ontimer(self.run_loop, 20)

# if __name__ == "__main__":
#     sim = RobotSimulator()
#     turtle.done()

# import turtle
# import time
# import random
# import math

# class RobotSimulator:
#     def __init__(self):
#         # ---- WORLD PARAMETERS ----
#         self.world_size = 10.0
#         self.window_px = 800
        
#         self.screen = turtle.Screen()
#         self.screen.setup(width=self.window_px, height=self.window_px)
#         self.screen.setworldcoordinates(-self.world_size/2, -self.world_size/2, 
#                                          self.world_size/2, self.world_size/2)
#         self.screen.title("Arc Pathing: Min Turning Radius & Look-Ahead")
#         self.screen.bgcolor("#2c3e50")
#         self.screen.tracer(0)

#         # ---- DRAW GRID ----
#         self.grid_tool = turtle.Turtle()
#         self.grid_tool.hideturtle()
#         self.draw_grid()

#         # Robot Dimensions
#         self.robot_width = 0.4064 
#         self.robot_length = 0.2032 

#         # ---- CONSTRAINTS ----
#         # self.min_turning_radius = 1.0  # Meters (Approx)
#         # self.max_turn_per_step = 5.0   # Degrees per frame (enforces radius)
#         # self.waypoint_threshold = 0.8  # "Min distance" to trigger next arc

#         self.min_turning_radius = 1.0  # Meters (Approx)
#         self.max_turn_per_step = 5.0   # Degrees per frame (enforces radius)
#         self.waypoint_threshold = 0.8  # "Min distance" to trigger next arc


#         # Robot Shape
#         robot_shape = (
#             (self.robot_length/2, self.robot_width/2),
#             (self.robot_length/2, -self.robot_width/2),
#             (-self.robot_length/2, -self.robot_width/2),
#             (-self.robot_length/2, self.robot_width/2)
#         )
#         self.screen.register_shape("robot_rect", robot_shape)

#         # State
#         self.waypoints = []
#         self.current_wp_idx = 0
#         self.is_running = False

#         self.path_drawer = turtle.Turtle()
#         self.path_drawer.hideturtle()
#         self.path_drawer.pencolor("#e74c3c")
#         self.path_drawer.width(2)

#         self.robot = turtle.Turtle()
#         self.robot.shape("robot_rect")
#         self.robot.color("#3498db")
#         self.robot.penup()

#         # PD Control State
#         self.yaw = 0.0
#         self.target_yaw = 0.0
#         self.kp = 0.35 
#         self.kd = 0.15
#         self.last_error = 0.0
#         self.vel = 0.08 # Speed in meters per frame

#         self.screen.update()
#         self.screen.tracer(1)

#         self.screen.onclick(self.add_waypoint)
#         self.screen.onkey(self.start_sim, "space")
#         self.screen.listen()

#     def draw_grid(self):
#         self.grid_tool.penup()
#         self.grid_tool.pencolor("#34495e")
#         for i in range(int(-self.world_size/2), int(self.world_size/2) + 1):
#             self.grid_tool.goto(i, -5); self.grid_tool.pendown(); self.grid_tool.goto(i, 5); self.grid_tool.penup()
#             self.grid_tool.goto(-5, i); self.grid_tool.pendown(); self.grid_tool.goto(5, i); self.grid_tool.penup()

#     def add_waypoint(self, x, y):
#         if not self.is_running:
#             if not self.waypoints:
#                 self.robot.goto(x, y)
#                 self.path_drawer.penup(); self.path_drawer.goto(x, y); self.path_drawer.pendown()
#             self.waypoints.append((x, y))
#             self.path_drawer.goto(x, y)
#             self.path_drawer.dot(6, "#e74c3c")

#     def start_sim(self):
#         if self.waypoints and not self.is_running:
#             self.is_running = True
#             self.run_loop()

#     def run_loop(self):
#         if self.current_wp_idx >= len(self.waypoints):
#             print("\nMission Complete!")
#             return

#         target_pos = self.waypoints[self.current_wp_idx]
#         dist = self.robot.distance(target_pos)
#         is_last_point = (self.current_wp_idx == len(self.waypoints) - 1)

#         # 1. ARC LOGIC: Switch waypoint BEFORE reaching it (except for the last one)
#         threshold = 0.1 if is_last_point else self.waypoint_threshold
#         if dist < threshold:
#             self.current_wp_idx += 1
#             return self.screen.ontimer(self.run_loop, 10)

#         # 2. TARGET HEADING
#         dx = target_pos[0] - self.robot.xcor()
#         dy = target_pos[1] - self.robot.ycor()
#         self.target_yaw = math.degrees(math.atan2(dy, dx))

#         # 3. PD CORRECTION
#         error = (self.target_yaw - self.yaw + 180) % 360 - 180
#         correction = (self.kp * error) + (self.kd * (error - self.last_error))
#         self.last_error = error

#         # 4. MIN TURNING RADIUS: Limit the max degrees the robot can turn per frame
#         if correction > self.max_turn_per_step: correction = self.max_turn_per_step
#         if correction < -self.max_turn_per_step: correction = -self.max_turn_per_step

#         self.yaw += correction
#         self.robot.setheading(self.yaw)

#         # 5. VELOCITY CONTROL: Slow down for the final point only
#         move_vel = self.vel
#         if is_last_point and dist < 1.0:
#             move_vel *= (dist + 0.1)

#         self.robot.forward(move_vel)
        
#         print(f"WP: {self.current_wp_idx} | Dist: {dist:.2f}m | Turn: {correction:.2f}°", end="\r")
#         self.screen.ontimer(self.run_loop, 25)

# if __name__ == "__main__":
#     sim = RobotSimulator()
#     turtle.done()

# import turtle
# import random
# import math

# class RobotSimulator:
#     def __init__(self):
#         # ---- WORLD PARAMETERS (In Meters) ----
#         self.world_size = 20.0  # 20m x 20m total area
#         self.window_px = 800
        
#         self.screen = turtle.Screen()
#         self.screen.setup(width=self.window_px, height=self.window_px)
#         # Coordinates: Bottom-left (-10, -10) to Top-right (10, 10)
#         self.screen.setworldcoordinates(-self.world_size/2, -self.world_size/2, 
#                                          self.world_size/2, self.world_size/2)
#         self.screen.title("Robot PD Sim: 20x20m Grid")
#         self.screen.bgcolor("#2c3e50")
#         self.screen.tracer(0) 

#         # Robot physical dimensions (approximate for display)
#         self.robot_width = 0.4
#         self.robot_length = 0.6

#         # ---- DRAW GRID ----
#         self.grid_tool = turtle.Turtle()
#         self.grid_tool.hideturtle()
#         self.draw_grid()

#         # Lists and State
#         self.waypoints = []
#         self.current_wp_idx = 0
#         self.is_running = False

#         # ---- PATH DRAWER ----
#         self.path_drawer = turtle.Turtle()
#         self.path_drawer.hideturtle()
#         self.path_drawer.pencolor("#7f8c8d")
#         self.path_drawer.width(2)

#         # ---- ROBOT ----
#         self.robot = turtle.Turtle()
#         self.robot.shape("triangle") # Simple shape for scaling
#         self.robot.shapesize(stretch_wid=0.5, stretch_len=1) 
#         self.robot.color("#3498db")
#         self.robot.penup()

#         # PD Control State
#         self.yaw = 0.0
#         self.kp = 0.6  # Proportional
#         self.kd = 0.2  # Derivative
#         self.last_error = 0.0
#         self.vel = 0.05  # 5cm per frame (much more stable)

#         # Input
#         self.screen.onclick(self.add_waypoint)
#         self.screen.onkey(self.start_sim, "space")
#         self.screen.listen()

#         self.screen.update()
#         print("1. Click to add waypoints | 2. SPACE to start.")

#     def draw_grid(self):
#         self.grid_tool.pencolor("#34495e")
#         for i in range(int(-self.world_size/2), int(self.world_size/2) + 1):
#             # Vertical
#             self.grid_tool.penup(); self.grid_tool.goto(i, -self.world_size/2); self.grid_tool.pendown()
#             self.grid_tool.goto(i, self.world_size/2)
#             # Horizontal
#             self.grid_tool.penup(); self.grid_tool.goto(-self.world_size/2, i); self.grid_tool.pendown()
#             self.grid_tool.goto(self.world_size/2, i)

#     def add_waypoint(self, x, y):
#         if not self.is_running:
#             if not self.waypoints:
#                 self.robot.goto(x, y)
#                 self.path_drawer.penup()
#                 self.path_drawer.goto(x, y)
#                 self.path_drawer.pendown()
            
#             self.waypoints.append((x, y))
#             self.path_drawer.goto(x, y)
#             self.path_drawer.dot(8, "#e74c3c")
#             self.screen.update()
#             print(f"Added WP: ({x:.2f}m, {y:.2f}m)")

#     def start_sim(self):
#         if self.waypoints and not self.is_running:
#             self.is_running = True
#             self.robot.pendown()
#             self.run_loop()

#     def run_loop(self):
#         if self.current_wp_idx >= len(self.waypoints):
#             print("\nMission Complete!")
#             return

#         target_pos = self.waypoints[self.current_wp_idx]

#         # 1. Calculate Target Heading
#         dx = target_pos[0] - self.robot.xcor()
#         dy = target_pos[1] - self.robot.ycor()
#         target_yaw = math.degrees(math.atan2(dy, dx))

#         # 2. Check for Arrival (Threshold should be small, like 0.2m)
#         if self.robot.distance(target_pos) < 0.2:
#             self.current_wp_idx += 1
#             self.screen.ontimer(self.run_loop, 10)
#             return

#         # 3. Apply Noise/Drift
#         self.yaw += random.uniform(-3.0, 5.0)

#         # 4. PD Correction
#         error = (target_yaw - self.yaw + 180) % 360 - 180
#         derivative = error - self.last_error
#         correction = (self.kp * error) + (self.kd * derivative)
#         self.last_error = error

#         self.yaw += correction
#         self.robot.setheading(self.yaw)
#         self.robot.forward(self.vel)

#         self.screen.update()
#         self.screen.ontimer(self.run_loop, 20)

# if __name__ == "__main__":
#     sim = RobotSimulator()
#     turtle.done()

import turtle
import random
import math

class RobotSimulator:
    def __init__(self):
        # ---- WORLD PARAMETERS ----
        self.world_size = 20.0
        self.window_px = 800
        
        self.screen = turtle.Screen()
        self.screen.setup(width=self.window_px, height=self.window_px)
        self.screen.setworldcoordinates(-self.world_size/2, -self.world_size/2, 
                                         self.world_size/2, self.world_size/2)
        self.screen.title("Robot PD: Sharp Turn Smoothing (>70°)")
        self.screen.bgcolor("#2c3e50")
        self.screen.tracer(0) 

        self.grid_tool = turtle.Turtle()
        self.grid_tool.hideturtle()
        self.draw_grid()

        self.click_points = []  
        self.waypoints = []     
        self.current_wp_idx = 0
        self.is_running = False

        # Visual layers
        self.ui_drawer = turtle.Turtle()    # For red dots and dashed lines
        self.ui_drawer.hideturtle()
        self.path_drawer = turtle.Turtle()  # For the actual smoothed trajectory
        self.path_drawer.hideturtle()

        # Robot
        self.robot = turtle.Turtle()
        self.robot.shape("triangle") 
        self.robot.shapesize(stretch_wid=0.5, stretch_len=1) 
        self.robot.color("#3498db")
        self.robot.penup()

        # PD Control State
        self.yaw = 0.0
        self.kp = 0.7 
        self.kd = 0.3 
        self.last_error = 0.0
        self.vel = 0.08 

        self.screen.onclick(self.add_waypoint)
        self.screen.onkey(self.start_sim, "space")
        self.screen.listen()

        self.screen.update()
        print("Click to add points. Dashed = Input, Solid = Smoothed Robot Path. SPACE to start.")

    def draw_grid(self):
        self.grid_tool.pencolor("#34495e")
        for i in range(int(-self.world_size/2), int(self.world_size/2) + 1):
            self.grid_tool.penup(); self.grid_tool.goto(i, -self.world_size/2); self.grid_tool.pendown()
            self.grid_tool.goto(i, self.world_size/2)
            self.grid_tool.penup(); self.grid_tool.goto(-self.world_size/2, i); self.grid_tool.pendown()
            self.grid_tool.goto(self.world_size/2, i)

    def draw_dashed_line(self, p1, p2):
        """ Draws a dashed line between two points """
        self.ui_drawer.penup()
        self.ui_drawer.goto(p1)
        self.ui_drawer.pendown()
        
        dist = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
        dash_length = 0.2
        num_dashes = int(dist / (dash_length * 2))
        
        angle = math.atan2(p2[1]-p1[1], p2[0]-p1[0])
        curr_pos = list(p1)
        
        for _ in range(num_dashes):
            self.ui_drawer.pendown()
            curr_pos[0] += math.cos(angle) * dash_length
            curr_pos[1] += math.sin(angle) * dash_length
            self.ui_drawer.goto(curr_pos[0], curr_pos[1])
            self.ui_drawer.penup()
            curr_pos[0] += math.cos(angle) * dash_length
            curr_pos[1] += math.sin(angle) * dash_length
            self.ui_drawer.goto(curr_pos[0], curr_pos[1])
        
        self.ui_drawer.goto(p2) # Ensure we reach the exact end

    def get_angle_between(self, p1, p2, p3):
        a = math.atan2(p1[1]-p2[1], p1[0]-p2[0])
        b = math.atan2(p3[1]-p2[1], p3[0]-p2[0])
        angle = math.degrees(abs(a - b))
        if angle > 180: angle = 360 - angle
        return angle

    def update_visuals(self):
        self.ui_drawer.clear()
        self.path_drawer.clear()
        self.waypoints = []

        if not self.click_points: return

        # 1. Draw Clicks & Dashed Intent
        self.ui_drawer.pencolor("#95a5a6")
        for i in range(len(self.click_points)):
            p = self.click_points[i]
            if i > 0:
                self.draw_dashed_line(self.click_points[i-1], p)
            self.ui_drawer.penup()
            self.ui_drawer.goto(p)
            self.ui_drawer.dot(8, "#e74c3c") # Persistent Red Dots

        # 2. Generate and Draw Smoothed Path
        if len(self.click_points) < 2: return
        
        self.waypoints = [self.click_points[0]]
        for i in range(1, len(self.click_points) - 1):
            p_prev, p_curr, p_next = self.click_points[i-1:i+2]
            turn_angle = 180 - self.get_angle_between(p_prev, p_curr, p_next)

            if turn_angle > 70:
                steps = 15
                m1 = ((p_prev[0]+p_curr[0])/2, (p_prev[1]+p_curr[1])/2)
                m2 = ((p_curr[0]+p_next[0])/2, (p_curr[1]+p_next[1])/2)
                for j in range(steps + 1):
                    t = j / steps
                    x = (1-t)**2 * m1[0] + 2*(1-t)*t * p_curr[0] + t**2 * m2[0]
                    y = (1-t)**2 * m1[1] + 2*(1-t)*t * p_curr[1] + t**2 * m2[1]
                    self.waypoints.append((x, y))
            else:
                self.waypoints.append(p_curr)
        
        self.waypoints.append(self.click_points[-1])

        # Draw the solid smoothed path
        self.path_drawer.pencolor("#bdc3c7")
        self.path_drawer.width(1)
        self.path_drawer.penup()
        self.path_drawer.goto(self.waypoints[0])
        self.path_drawer.pendown()
        for pt in self.waypoints:
            self.path_drawer.goto(pt)

        self.screen.update()

    def add_waypoint(self, x, y):
        if self.is_running: return
        self.click_points.append((x, y))
        self.update_visuals()

    def start_sim(self):
        if self.waypoints and not self.is_running:
            self.is_running = True
            self.robot.goto(self.waypoints[0])
            self.robot.pendown()
            self.robot.pencolor("#2ecc71")
            self.run_loop()

    def run_loop(self):
        if self.current_wp_idx >= len(self.waypoints):
            print("Mission Complete!")
            return

        target = self.waypoints[self.current_wp_idx]
        dx, dy = target[0] - self.robot.xcor(), target[1] - self.robot.ycor()
        target_yaw = math.degrees(math.atan2(dy, dx))

        if self.robot.distance(target) < 0.15:
            self.current_wp_idx += 1
            self.screen.ontimer(self.run_loop, 10)
            return

        self.yaw += random.uniform(-6.5, 8.5) # Drift
        error = (target_yaw - self.yaw + 180) % 360 - 180
        correction = (self.kp * error) + (self.kd * (error - self.last_error))
        self.last_error = error

        self.yaw += correction
        self.robot.setheading(self.yaw)
        self.robot.forward(self.vel)

        self.screen.update()
        self.screen.ontimer(self.run_loop, 20)

if __name__ == "__main__":
    sim = RobotSimulator()
    turtle.done()