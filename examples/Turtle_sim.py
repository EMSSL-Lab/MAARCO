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


        # Velocity PD (Longitudinal - forward/backward)
        self.kp_v = 0.5       # Proportional gain for speed
        self.kd_v = 0.05      # Derivative gain for speed
        self.last_dist_error = 0.0
        
        self.max_vel = 0.15 * self.scale
        self.min_safe_vel = 0.03 * self.scale 
        self.vel = self.max_vel
        
        self.is_pivoting = False
        # ... (keep your input listeners and turtles) ...

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
    
    def get_velocity_pd(self, distance_error):
        # Calculate how much the distance has changed since last frame
        derivative = distance_error - self.last_dist_error
        
        # Calculate PD Output
        output = (self.kp_v * distance_error) + (self.kd_v * derivative)
        self.last_dist_error = distance_error
        
        # Clamp the output so the bot never goes too fast or stalls
        return max(self.min_safe_vel, min(self.max_vel, output))

    # def run_loop(self):

    #     if self.current_wp_idx >= len(self.waypoints):
    #         print("\nMission Complete!")
    #         return

    #     target_pos = self.waypoints[self.current_wp_idx]

    #     dx = target_pos[0] - self.robot.xcor()
    #     dy = target_pos[1] - self.robot.ycor()

    #     self.target_yaw = math.degrees(math.atan2(dy, dx))

    #     if self.robot.distance(target_pos) < 10:
    #         self.current_wp_idx += 1

    #     # Drift + PD correction
    #     self.apply_drift()
    #     error, correction = self.get_pd_correction()

    #     self.yaw += correction

    #     self.robot.setheading(self.yaw)
    #     self.robot.forward(self.vel)

    #     print(
    #         f"Yaw Target: {self.target_yaw:6.1f} | Pos: ({self.robot.xcor()/self.scale:5.2f} m, {self.robot.ycor()/self.scale:5.2f} m) | Error: {error:6.2f}",
    #         end="\r"
    #     )

    #     self.screen.ontimer(self.run_loop, 20)

    def run_loop(self):
        if self.current_wp_idx >= len(self.waypoints):
            print("\nMission Complete!")
            return

        target_pos = self.waypoints[self.current_wp_idx]
        dist_to_wp = self.robot.distance(target_pos)
        
        # 1. Update Target Heading
        dx = target_pos[0] - self.robot.xcor()
        dy = target_pos[1] - self.robot.ycor()
        self.target_yaw = math.degrees(math.atan2(dy, dx))

        # 2. Get Steering Correction (Lateral)
        self.apply_drift()
        angle_error, steer_corr = self.get_pd_correction()

        # 3. Get Velocity Correction (Longitudinal)
        # We pass distance in meters (pixels / scale)
        target_speed = self.get_velocity_pd(dist_to_wp / self.scale)

        # 4. Movement Logic
        if self.is_pivoting:
            # STOP and Pivot in place
            self.yaw += steer_corr
            if abs(angle_error) < 5: # Threshold to finish pivot
                self.is_pivoting = False
            status = "PIVOTING"
        else:
            # DRIVE and Steer
            self.yaw += steer_corr
            self.robot.forward(target_speed)
            status = "DRIVING "

        self.robot.setheading(self.yaw)

        # 5. Waypoint Arrival & Decision
        if dist_to_wp < 5:
            self.current_wp_idx += 1
            if self.current_wp_idx < len(self.waypoints):
                # Calculate turn to NEXT point
                next_wp = self.waypoints[self.current_wp_idx]
                next_yaw = math.degrees(math.atan2(next_wp[1]-target_pos[1], next_wp[0]-target_pos[0]))
                turn_diff = (next_yaw - self.yaw + 180) % 360 - 180
                
                # If next turn is sharp, trigger pivot for the NEXT leg
                if abs(turn_diff) > 100:
                    self.is_pivoting = True

        print(f"Status: {status:9} | Speed: {target_speed/self.scale:4.2f}m/s | Angle Err: {angle_error:6.1f}", end="\r")
        self.screen.ontimer(self.run_loop, 20)


if __name__ == "__main__":
    sim = RobotSimulator()
    turtle.done()