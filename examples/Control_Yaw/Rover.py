# import turtle
# import random
# import math

# class BaseRover:
#     def __init__(self, world_size=20.0):
#         self.world_size = world_size
#         self.screen = turtle.Screen()
#         self.screen.setup(800, 800)
#         self.screen.setworldcoordinates(-10, -10, 10, 10)
#         self.screen.bgcolor("#2c3e50")
#         self.screen.tracer(0)
        
#         # Robot Physical State
#         self.robot = turtle.Turtle(shape="triangle")
#         self.robot.color("#3498db")
#         self.robot.penup()
#         self.yaw = 0.0
#         self.vel = 0.08
        
#         # Drawing Layers
#         self.ui_layer = turtle.Turtle(visible=False)
#         self.path_layer = turtle.Turtle(visible=False)
#         self.path_layer.pencolor("#bdc3c7")
        
#         self.click_points = []
#         self.waypoints = []
#         self.current_wp_idx = 0
        
#         self.screen.onclick(self.add_point)

#     def add_point(self, x, y):
#         self.click_points.append((x, y))
#         self.ui_layer.penup()
#         self.ui_layer.goto(x, y)
#         self.ui_layer.dot(8, "#e74c3c")
#         self._generate_smoothed_path()
#         self.screen.update()

#     def _generate_smoothed_path(self):
#         """Logic for dash-lines and arc-smoothing (>70 deg)"""
#         self.path_layer.clear()
#         if len(self.click_points) < 2: return
        
#         # (Internal logic for smoothing as defined previously)
#         self.waypoints = [self.click_points[0]]
#         # ... logic to fill self.waypoints ...
#         # (Simplified for brevity, assuming standard path generation)
#         self.waypoints = self.click_points # Placeholder for full smoothing logic
        
#         self.path_layer.penup()
#         self.path_layer.goto(self.waypoints[0])
#         self.path_layer.pendown()
#         for pt in self.waypoints:
#             self.path_layer.goto(pt)

#     def get_state(self):
#         """Returns the current error to the controller"""
#         if self.current_wp_idx >= len(self.waypoints):
#             return None
            
#         target = self.waypoints[self.current_wp_idx]
#         dx = target[0] - self.robot.xcor()
#         dy = target[1] - self.robot.ycor()
#         target_yaw = math.degrees(math.atan2(dy, dx))
        
#         # Distance Check
#         if self.robot.distance(target) < 0.15:
#             self.current_wp_idx += 1
            
#         error = (target_yaw - self.yaw + 180) % 360 - 180
#         return error

#     def apply_action(self, steering_correction):
#         """Updates physics with steering and noise"""
#         self.yaw += random.uniform(-2.5, 2.5) # The 'Drift'
#         self.yaw += steering_correction
#         self.robot.setheading(self.yaw)
#         self.robot.forward(self.vel)
#         self.screen.update()

import turtle
import random
import math

class BaseRover:
    def __init__(self, world_size=20.0):
        self.world_size = world_size
        self.screen = turtle.Screen()
        self.screen.setup(800, 800)
        self.screen.setworldcoordinates(-10, -10, 10, 10)
        self.screen.bgcolor("#2c3e50")
        self.screen.tracer(0)
        
        self.robot = turtle.Turtle(shape="triangle")
        self.robot.color("#3498db")
        self.robot.penup()
        
        self.ui_layer = turtle.Turtle(visible=False)
        self.path_layer = turtle.Turtle(visible=False)
        
        self.yaw = 0.0
        self.vel = 0.08
        self.click_points = []
        self.waypoints = []
        self.current_wp_idx = 0
        
        self.screen.onclick(self.add_point)

    def draw_dashed_line(self, p1, p2):
        self.ui_layer.penup()
        self.ui_layer.goto(p1)
        dist = math.dist(p1, p2)
        angle = math.atan2(p2[1]-p1[1], p2[0]-p1[0])
        for _ in range(int(dist/0.4)):
            self.ui_layer.pendown()
            self.ui_layer.setheading(math.degrees(angle))
            self.ui_layer.forward(0.2)
            self.ui_layer.penup()
            self.ui_layer.forward(0.2)

    def add_point(self, x, y):
        self.click_points.append((x, y))
        self._update_pathing()
        self.screen.update()

    def _update_pathing(self):
        self.ui_layer.clear()
        self.path_layer.clear()
        self.waypoints = []
        
        if not self.click_points: return
        
        # Draw Red Dots and Dashes
        for i, p in enumerate(self.click_points):
            self.ui_layer.penup()
            self.ui_layer.goto(p)
            self.ui_layer.dot(8, "#e74c3c")
            if i > 0: self.draw_dashed_line(self.click_points[i-1], p)

        # Generate Smoothed Waypoints
        if len(self.click_points) < 2: 
            self.waypoints = self.click_points
            return

        self.waypoints.append(self.click_points[0])
        for i in range(1, len(self.click_points)-1):
            p1, p2, p3 = self.click_points[i-1:i+2]
            # Angle check for smoothing
            a = math.atan2(p1[1]-p2[1], p1[0]-p2[0])
            b = math.atan2(p3[1]-p2[1], p3[0]-p2[0])
            angle = math.degrees(abs(a-b))
            if angle > 180: angle = 360 - angle
            
            if (180 - angle) > 70: # Sharp turn
                m1 = ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2)
                m2 = ((p2[0]+p3[0])/2, (p2[1]+p3[1])/2)
                for t in [i/10 for i in range(11)]:
                    x = (1-t)**2 * m1[0] + 2*(1-t)*t * p2[0] + t**2 * m2[0]
                    y = (1-t)**2 * m1[1] + 2*(1-t)*t * p2[1] + t**2 * m2[1]
                    self.waypoints.append((x, y))
            else:
                self.waypoints.append(p2)
        self.waypoints.append(self.click_points[-1])

        # Draw actual path
        self.path_layer.pencolor("#bdc3c7")
        self.path_layer.penup()
        self.path_layer.goto(self.waypoints[0])
        self.path_layer.pendown()
        for wp in self.waypoints: self.path_layer.goto(wp)

    def get_error(self):
        if self.current_wp_idx >= len(self.waypoints): return None
        target = self.waypoints[self.current_wp_idx]
        if self.robot.distance(target) < 0.2:
            self.current_wp_idx += 1
            return self.get_error()
        
        angle_to_target = math.degrees(math.atan2(target[1]-self.robot.ycor(), target[0]-self.robot.xcor()))
        return (angle_to_target - self.yaw + 180) % 360 - 180

    def move(self, steering):
        self.yaw += random.uniform(-10.0, 2.0) # Noise
        self.yaw += steering
        self.robot.setheading(self.yaw)
        self.robot.forward(self.vel)
        self.screen.update()