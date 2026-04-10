# import turtle
# import socket

# # Setup UDP Socket
# # RUST_ADDR = ("127.0.0.1", 5007)

# PYTHON_LISTEN_ADDR = ("0.0.0.0", 5008)  # Receiving POS from Rust
# RUST_ADDR = ("172.20.10.6", 5007)

# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# class WaypointGenerator:
#     def __init__(self):
#         # ---- SCREEN SETUP FIRST ----
#         self.screen = turtle.Screen()
#         # Use 1.0 to fill the phone screen entirely
#         self.screen.setup(width=1.0, height=1.0)
#         self.screen.bgcolor("#2c3e50")
        
#         # ---- WORLD PARAMETERS ----
#         # self.world_x = 18.0   # meters
#         # # Now get the actual width after setup
#         # self.width = self.screen.window_width()
#         # self.scale = self.width / self.world_x 
#         # Instead of manual scaling, let Turtle do the math:

#         # This sets the screen to be 20m wide, with (0,0) in the center.
#         self.world_size = 20 
#         self.screen.setworldcoordinates(-self.world_size, -self.world_size, 
#         self.world_size, self.world_size)

#         # ---- STATE ----
#         self.origin_x = None
#         self.origin_y = None

#         # ---- DRAWING TOOLS ----
#         self.drawer = turtle.Turtle()
#         self.drawer.hideturtle()
#         self.drawer.speed(0)
#         self.drawer.pencolor("#ecf0f1")
#         self.drawer.penup()
#         # Text tool for on-screen info
#         self.writer = turtle.Turtle()
#         self.writer.hideturtle()
#         self.writer.penup()
#         self.writer.pencolor("white")
#         self.update_ui_text("TAP TO SET ORIGIN")

#         # UI Bindings
#         self.screen.onclick(self.handle_click)
#         self.screen.listen()

#     def update_ui_text(self, text):
#         self.writer.clear()
#         # Position text at the top of the screen
#         self.writer.goto(0, (self.screen.window_height()/2) - 40)
#         self.writer.write(text, align="center", font=("Arial", 16, "bold"))

#     def handle_click(self, x, y):
#         # Check if user clicked the "START ZONE" (Top 10% of screen)
#         if y > (self.screen.window_height() / 2) - 60:
#             if self.origin_x is not None:
#                 self.send_start_signal()
#             return

#         # Set origin on the very first click
#         if self.origin_x is None:
#             self.origin_x, self.origin_y = x, y
#             self.drawer.goto(x, y)
#             self.drawer.dot(20, "#2ecc71") # Larger dot for thumbs
#             self.drawer.pendown()
#             sock.sendto(b"SET_ORIGIN", RUST_ADDR)
#             self.update_ui_text("ORIGIN SET. TAP WAYPOINTS, THEN TAP HERE TO START")
#             return

#         # Calculate meters relative to origin
#         rel_x_m = (x - self.origin_x) / self.scale
#         rel_y_m = (y - self.origin_y) / self.scale
        
#         # Send to Rust
#         message = f"WP,{rel_x_m:.2f},{rel_y_m:.2f}"
#         sock.sendto(message.encode(), RUST_ADDR)
        
#         # Draw path on UI
#         self.drawer.goto(x, y)
#         self.drawer.dot(15, "#e74c3c") # Larger dots for phone screen
#         print(f"Sent: {rel_x_m:.2f}m E, {rel_y_m:.2f}m N")

#     def send_start_signal(self):
#         sock.sendto(b"START_MISSION", RUST_ADDR)
#         self.update_ui_text("MISSION RUNNING!")
#         print("🚀 START_MISSION sent")

# if __name__ == "__main__":
#     gen = WaypointGenerator()
#     turtle.done()

# import turtle
# import socket
# import select

# RUST_SEND_ADDR = ("172.20.10.6", 5007) # Sending WPs to Rust
# PYTHON_LISTEN_ADDR = ("0.0.0.0", 5008)  # Receiving POS from Rust

# class MissionControl:
#     def __init__(self):
#         self.screen = turtle.Screen()
#         self.screen.setup(width=0.9, height=0.9)
#         self.screen.bgcolor("#2c3e50")
        
#         # 1. SCALE: Set world to meters. (0,0) is center.
#         self.view_size = 15 # meters from center
#         self.screen.setworldcoordinates(-self.view_size, -self.view_size, 
#                                          self.view_size, self.view_size)

#         # Drawer for Waypoints (Red)
#         self.drawer = turtle.Turtle()
#         self.drawer.pencolor("#e74c3c")
#         self.drawer.penup()
        
#         # Rover Turtle (Green) - This shows the live bot
#         self.rover = turtle.Turtle(shape="triangle")
#         self.rover.color("#2ecc71")
#         self.rover.penup()
#         self.rover.speed(0)

#         # Socket Setup
#         self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.sock.bind(PYTHON_LISTEN_ADDR)
#         self.sock.setblocking(False)

#         self.screen.onclick(self.handle_click)
#         self.screen.ontimer(self.update_rover, 100) # Check for data every 100ms
#         self.screen.listen()

#     def handle_click(self, x, y):
#         # Coordinates are now automatically in meters!
#         message = f"WP,{x:.2f},{y:.2f}"
#         self.sock.sendto(message.encode(), RUST_SEND_ADDR)
        
#         self.drawer.goto(x, y)
#         self.drawer.dot(10)
#         self.drawer.pendown()
#         print(f"Sent Waypoint: {x:.2f}m, {y:.2f}m")

#     def update_rover(self):
#         # Similar to your ML live loop but non-blocking
#         try:
#             data, _ = self.sock.recvfrom(1024)
#             msg = data.decode().split(',')
#             if msg[0] == "POS":
#                 # msg format: ["POS", "x", "y", "yaw"]
#                 rx, ry = float(msg[1]), float(msg[2])
#                 ryaw = float(msg[3])
#                 self.rover.goto(rx, ry)
#                 self.rover.setheading(ryaw) # Triangle points where bot points
#         except BlockingIOError:
#             pass
        
#         self.screen.ontimer(self.update_rover, 100)

# if __name__ == "__main__":
#     gui = MissionControl()
#     turtle.done()

# import turtle
# import socket

# # UPDATE THESE TO YOUR ACTUAL IPS
# RUST_SEND_ADDR = ("172.20.10.6", 5007) # Sending WPs to Rust
# PYTHON_LISTEN_ADDR = ("0.0.0.0", 5008)  # Receiving POS from Rust

# class MissionControl:
#     def __init__(self):
#         self.screen = turtle.Screen()
#         self.screen.setup(width=0.9, height=0.9)
#         self.screen.bgcolor("#2c3e50")
#         self.screen.title("Rover Mission Control - Local Coordinates (Meters)")
        
#         # 1. SCALE: Set world to meters. (0,0) is center.
#         self.view_size = 15 # meters from center (Total 30m x 30m view)
#         self.screen.setworldcoordinates(-self.view_size, -self.view_size, 
#                                          self.view_size, self.view_size)
#         self.screen.tracer(0) # Turn off animation for instant grid drawing

#         # 2. Draw Grid & Axes
#         self.grid_tool = turtle.Turtle()
#         self.grid_tool.hideturtle()
#         self.grid_tool.speed(0)
#         self.draw_grid()

#         # Drawer for Waypoints (Red)
#         self.drawer = turtle.Turtle()
#         self.drawer.pencolor("#e74c3c")
#         self.drawer.width(2)
#         self.drawer.penup()
        
#         # Rover Turtle (Green) - This shows the live bot
#         self.rover = turtle.Turtle(shape="triangle")
#         self.rover.shapesize(1.5, 1.5)
#         self.rover.color("#2ecc71")
#         self.rover.penup()
#         self.rover.setundobuffer(100) # Keep a small trail if you use pen down

#         self.screen.update() # Render everything drawn so far
#         self.screen.tracer(1) # Turn animation back on for the rover

#         # Socket Setup
#         self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.sock.bind(PYTHON_LISTEN_ADDR)
#         self.sock.setblocking(False)

#         self.screen.onclick(self.handle_click)
#         self.update_rover() # Start loop
#         self.screen.listen()

#     def draw_grid(self):
#         self.grid_tool.clear()
#         # Draw soft grid lines every 1 meter
#         self.grid_tool.pencolor("#34495e")
#         for i in range(-self.view_size, self.view_size + 1):
#             # Vertical lines
#             self.grid_tool.penup()
#             self.grid_tool.goto(i, -self.view_size)
#             self.grid_tool.pendown()
#             self.grid_tool.goto(i, self.view_size)
#             # Horizontal lines
#             self.grid_tool.penup()
#             self.grid_tool.goto(-self.view_size, i)
#             self.grid_tool.pendown()
#             self.grid_tool.goto(self.view_size, i)

#         # Draw main Axes (X/Y)
#         self.grid_tool.pencolor("#95a5a6")
#         self.grid_tool.width(2)
#         # X-Axis
#         self.grid_tool.penup(); self.grid_tool.goto(-self.view_size, 0); self.grid_tool.pendown(); self.grid_tool.goto(self.view_size, 0)
#         # Y-Axis
#         self.grid_tool.penup(); self.grid_tool.goto(0, -self.view_size); self.grid_tool.pendown(); self.grid_tool.goto(0, self.view_size)

#         # Add Units/Labels every 5 meters
#         self.grid_tool.penup()
#         for i in range(-self.view_size, self.view_size + 1, 5):
#             if i == 0: continue
#             # X labels
#             self.grid_tool.goto(i, -0.7)
#             self.grid_tool.write(f"{i}m", align="center", font=("Arial", 10, "bold"))
#             # Y labels
#             self.grid_tool.goto(0.2, i - 0.3)
#             self.grid_tool.write(f"{i}m", align="left", font=("Arial", 10, "bold"))

#     def handle_click(self, x, y):
#         message = f"WP,{x:.2f},{y:.2f}"
#         self.sock.sendto(message.encode(), RUST_SEND_ADDR)
        
#         self.drawer.goto(x, y)
#         self.drawer.dot(8, "#e74c3c")
#         self.drawer.pendown()
#         print(f"Sent Waypoint: {x:.2f}m, {y:.2f}m")

#     def update_rover(self):
#         try:
#             data, _ = self.sock.recvfrom(1024)
#             msg = data.decode().split(',')
#             if msg[0] == "POS":
#                 # Expecting: POS,x,y,yaw
#                 rx, ry = float(msg[1]), float(msg[2])
#                 self.rover.goto(rx, ry)
                
#                 # Update heading only if yaw was sent
#                 if len(msg) > 3:
#                     ryaw = float(msg[3])
#                     self.rover.setheading(ryaw)
                    
#         except (BlockingIOError, IndexError, ValueError):
#             pass
        
#         self.screen.ontimer(self.update_rover, 50) # Faster 50ms refresh for smoother motion

# if __name__ == "__main__":
#     gui = MissionControl()
#     turtle.done()
