import turtle
import time

class RoverGCS:
    def __init__(self):
        self.screen = turtle.Screen()
        
        # 1. Force Full Screen
        self.screen.setup(width=1.0, height=1.0)
        self.screen.bgcolor("#1a1a1a")
        self.screen.title("Screw-Propelled Rover GCS: PINN vs EKF Analysis")
        
        # Turn off animation for real-time performance
        self.screen.tracer(0)

        # 2. Dynamic Aspect Ratio Calculation
        # We use your 1920x1280 specs
        self.screen_w = 1920 
        self.screen_h = 1280
        self.aspect_ratio = self.screen_w / self.screen_h # 1.5

        # Define how many meters you want to see vertically
        self.view_meters_y = 20 
        self.view_meters_x = self.view_meters_y * self.aspect_ratio # 30 meters

        # 3. Set World Coordinates (Center is 0,0)
        # X: -15 to 15, Y: -10 to 10
        self.screen.setworldcoordinates(
            -self.view_meters_x/2, 
            -self.view_meters_y/2, 
            self.view_meters_x/2, 
            self.view_meters_y/2
        )

        # 4. Initialize Turtles for the 4 Models
        self.create_turtles()
        
        # Draw the static environment
        self.draw_grid()
        self.screen.update()

    def create_turtles(self):
        # We create a different colored turtle for each of your 4 comparison models
        self.models = {
            "raw": self.make_turtle("#ff4444"),      # Red
            "kinematic": self.make_turtle("#44ff44"), # Green
            "dynamic": self.make_turtle("#4444ff"),   # Blue
            "pinn": self.make_turtle("#ff00ff")       # Magenta (The Star)
        }

    def make_turtle(self, color):
        t = turtle.Turtle()
        t.shape("triangle")
        t.shapesize(0.5, 0.5)
        t.color(color)
        t.penup()
        t.speed(0)
        return t

    def draw_grid(self):
        writer = turtle.Turtle()
        writer.hideturtle()
        writer.pencolor("#2a2a2a") # Subtle grid color
        writer.speed(0)

        # Vertical Lines (Meters)
        for x in range(int(-self.view_meters_x/2), int(self.view_meters_x/2) + 1):
            writer.penup()
            writer.goto(x, -self.view_meters_y/2)
            writer.pendown()
            writer.goto(x, self.view_meters_y/2)

        # Horizontal Lines (Meters)
        for y in range(int(-self.view_meters_y/2), int(self.view_meters_y/2) + 1):
            writer.penup()
            writer.goto(-self.view_meters_x/2, y)
            writer.pendown()
            writer.goto(self.view_meters_x/2, y)
            
        # Draw Origin Axes
        writer.pensize(2)
        writer.pencolor("#444444")
        writer.penup(); writer.goto(0, -10); writer.pendown(); writer.goto(0, 10)
        writer.penup(); writer.goto(-15, 0); writer.pendown(); writer.goto(15, 0)

    def update_position(self, model_name, x, y, yaw):
        """Call this function when you get data from Rust"""
        if model_name in self.models:
            t = self.models[model_name]
            t.goto(x, y)
            t.setheading(yaw)
            # t.pendown() # Uncomment if you want to leave a trail/path
            self.screen.update()

    def run_test_walk(self):
        """Simulates a 5-meter walk to verify grid scaling"""
        print("Starting 5-meter scale test...")
        for i in range(51):
            dist = i / 10.0
            # Move all turtles together to see alignment
            for name in self.models:
                self.update_position(name, dist, 0, 0)
            time.sleep(0.05)
        print("Test Complete. Red triangle should be on the 5th grid line.")

if __name__ == "__main__":
    gcs = RoverGCS()
    gcs.run_test_walk()
    gcs.screen.mainloop()