# import turtle
# import socket

# # UPDATE THESE TO YOUR ACTUAL IPs
# RUST_SEND_ADDR = ("172.20.10.6", 5007) 
# PYTHON_LISTEN_ADDR = ("0.0.0.0", 5008)  

# class CenteredGCS:
#     def __init__(self):
#         self.screen = turtle.Screen()
#         self.screen.setup(width=1100, height=600)
#         self.screen.bgcolor("#1a1a1a")
#         self.screen.title("Rover GCS: EKF + Target Tracking")
#         self.screen.tracer(0)

#         self.screen.setworldcoordinates(-10, -10, 20, 10)

#         self.writer = turtle.Turtle(visible=False)
#         self.writer.penup()
#         self.writer.color("white")
        
#         self.grid_tool = turtle.Turtle(visible=False)
        
#         # --- NEW: Target Marker Turtle ---
#         self.target_marker = turtle.Turtle(shape="circle")
#         self.target_marker.shapesize(0.5, 0.5)
#         self.target_marker.color("#e74c3c") # Red
#         self.target_marker.penup()
#         # Start it off-screen until a waypoint is set
#         self.target_marker.goto(100, 100) 

#         self.setup_layout()

#         self.rover = turtle.Turtle(shape="triangle")
#         self.rover.shapesize(1.0, 1.0)
#         self.rover.color("#2ecc71") # Green
#         self.rover.penup()

#         self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.sock.bind(PYTHON_LISTEN_ADDR)
#         self.sock.setblocking(False)

#         self.screen.listen()
#         self.screen.onkey(self.input_wp, "i")
#         self.screen.onkey(self.set_origin, "o")
#         self.screen.onkey(self.emergency_stop, "s")
#         self.screen.onkey(self.clear_map, "c")

#         self.update_loop()

#     def setup_layout(self):
#         self.grid_tool.clear()
#         # UI Background
#         self.grid_tool.penup()
#         self.grid_tool.goto(10, -10)
#         self.grid_tool.color("#2c3e50")
#         self.grid_tool.begin_fill()
#         for x, y in [(20, -10), (20, 10), (10, 10), (10, -10)]:
#             self.grid_tool.goto(x, y)
#         self.grid_tool.end_fill()

#         # Divider and Grid
#         self.grid_tool.pensize(3); self.grid_tool.color("white")
#         self.grid_tool.goto(10, 10); self.grid_tool.goto(10, -10)
#         self.grid_tool.pensize(1); self.grid_tool.color("#34495e")
#         for i in range(-10, 11):
#             self.grid_tool.penup(); self.grid_tool.goto(i, -10); self.grid_tool.pendown(); self.grid_tool.goto(i, 10)
#             self.grid_tool.penup(); self.grid_tool.goto(-10, i); self.grid_tool.pendown(); self.grid_tool.goto(10, i)

#         self.writer.goto(11, 5)
#         self.writer.write("CONTROLS", font=("Arial", 14, "bold"))
#         self.writer.goto(11, 1)
#         self.writer.write("[ I ] New Waypoint\n[ O ] Set Origin\n[ S ] STOP\n[ C ] Clear Trail", font=("Courier", 10, "normal"))

#     def input_wp(self):
#         x_val = self.screen.numinput("Waypoint", "Target X:", default=0)
#         y_val = self.screen.numinput("Waypoint", "Target Y:", default=0)
#         if x_val is not None and y_val is not None:
#             # Move the red dot immediately for visual feedback
#             self.target_marker.goto(x_val, y_val)
#             msg = f"WP,{x_val:.2f},{y_val:.2f}"
#             self.sock.sendto(msg.encode(), RUST_SEND_ADDR)
#             self.sock.sendto(b"START_MISSION", RUST_SEND_ADDR)

#     def set_origin(self):
#         self.sock.sendto(b"SET_ORIGIN", RUST_SEND_ADDR)
#         self.rover.penup()
#         self.rover.clear() # Clear the trail on origin reset
#         self.rover.goto(0, 0)
#         self.target_marker.goto(100, 100) # Hide target
#         print("Origin Reset.")

#     def emergency_stop(self):
#         self.sock.sendto(b"STOP", RUST_SEND_ADDR)
#         self.target_marker.goto(100, 100) # Hide target dot on stop

#     def clear_map(self):
#         self.rover.clear()

#     def update_loop(self):
#         try:
#             data, addr = self.sock.recvfrom(1024)
#             raw_data = data.decode()
#             print(f"RAW FROM RUST: {raw_data}")  # <--- ADD THIS LINE
#             msg = data.decode().split(',')
            
#             # Now parsing the 6 values: POS, X, Y, Head, TargetX, TargetY
#             if msg[0] == "POS" and len(msg) >= 6:
#                 rx, ry = float(msg[1]), float(msg[2])
#                 r_head = float(msg[3])
#                 tx, ty = float(msg[4]), float(msg[5])

#                 # Update Rover Position
#                 self.rover.goto(rx, ry)
#                 self.rover.setheading(r_head)
#                 self.rover.pendown() 

#                 # Update Target Dot Position from Rust's "True" target
#                 # (This ensures the dot stays where the RUST mission thinks it is)
#                 if abs(tx - rx) < 0.01 and abs(ty - ry) < 0.01:
#                     self.target_marker.goto(100, 100) # Hide if we are basically there
#                 else:
#                     self.target_marker.goto(tx, ty)

#         except BlockingIOError:
#             pass
#         except Exception as e:
#             print(f"Data error: {e}")

#         self.screen.update()
#         self.screen.ontimer(self.update_loop, 50)

# if __name__ == "__main__":
#     gui = CenteredGCS()
#     turtle.done()
import turtle
import socket

# UPDATE THESE TO YOUR ACTUAL IPs
RUST_SEND_ADDR = ("172.20.10.6", 5007) 
PYTHON_LISTEN_ADDR = ("0.0.0.0", 5008)  

class CenteredGCS:
    def __init__(self):
        self.screen = turtle.Screen()
        
        # 1. SETUP FULLSCREEN (1920x1280)
        self.screen.setup(width=1.0, height=1.0)
        self.screen.bgcolor("#1a1a1a")
        self.screen.title("Rover GCS: 1:1 Scale PINN Monitoring")
        self.screen.tracer(0)

        # 2. ASPECT RATIO MATH
        # This ensures 1 unit in Rust = a perfect square on your screen
        screen_w = 1920
        screen_h = 1280
        aspect_ratio = screen_w / screen_h # 1.5
        
        # We want to see 20 meters vertically (-10 to 10)
        # Therefore we see 30 meters horizontally (-15 to 15)
        self.view_h = 20
        self.view_w = self.view_h * aspect_ratio
        self.screen.setworldcoordinates(-self.view_w/2, -self.view_h/2, self.view_w/2, self.view_h/2)

        # 3. TURTLES
        self.writer = turtle.Turtle(visible=False)
        self.writer.penup()
        self.writer.color("white")
        
        self.grid_tool = turtle.Turtle(visible=False)
        self.grid_tool.speed(0)

        self.target_marker = turtle.Turtle(shape="circle")
        self.target_marker.shapesize(0.5, 0.5)
        self.target_marker.color("#e74c3c") # Red
        self.target_marker.penup()
        self.target_marker.goto(100, 100) # Hide initially

        self.rover = turtle.Turtle(shape="triangle")
        self.rover.shapesize(1.2, 1.2) # Made slightly bigger for 1920p
        self.rover.color("#2ecc71") # Green
        self.rover.penup()

        # 4. INITIALIZE
        self.setup_layout()

        # Networking
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(PYTHON_LISTEN_ADDR)
        self.sock.setblocking(False)

        # Key Bindings
        self.screen.listen()
        self.screen.onkey(self.input_wp, "i")
        self.screen.onkey(self.set_origin, "o")
        self.screen.onkey(self.emergency_stop, "s")
        self.screen.onkey(self.clear_map, "c")

        self.update_loop()

    def setup_layout(self):
        self.grid_tool.clear()
        
        # UI Sidebar Background (The right 1/3rd of the screen)
        # In our coords, X starts at 5 and goes to 15
        self.grid_tool.penup()
        self.grid_tool.goto(5, -10)
        self.grid_tool.color("#2c3e50")
        self.grid_tool.begin_fill()
        for x, y in [(15, -10), (15, 10), (5, 10), (5, -10)]:
            self.grid_tool.goto(x, y)
        self.grid_tool.end_fill()

        # Sidebar Divider
        self.grid_tool.pensize(3)
        self.grid_tool.color("white")
        self.grid_tool.goto(5, 10)
        self.grid_tool.goto(5, -10)

        # DRAW 1-METER GRID (The left map area)
        self.grid_tool.pensize(1)
        self.grid_tool.color("#34495e")
        # Vertical lines from -15 to 5 (Map zone)
        for i in range(-15, 6):
            self.grid_tool.penup(); self.grid_tool.goto(i, -10); self.grid_tool.pendown(); self.grid_tool.goto(i, 10)
        # Horizontal lines from -10 to 10
        for i in range(-10, 11):
            self.grid_tool.penup(); self.grid_tool.goto(-15, i); self.grid_tool.pendown(); self.grid_tool.goto(5, i)

        # TEXT LABELS
        self.writer.goto(6, 8)
        self.writer.write("ROVER TELEMETRY", font=("Arial", 16, "bold"))
        self.writer.goto(6, 4)
        self.writer.write("CONTROLS:\n[ I ] New Waypoint\n[ O ] Set Origin\n[ S ] STOP\n[ C ] Clear Trail", font=("Courier", 12, "normal"))

    def input_wp(self):
        x_val = self.screen.numinput("Waypoint", "Target X (-15 to 5):", default=0)
        y_val = self.screen.numinput("Waypoint", "Target Y (-10 to 10):", default=0)
        if x_val is not None and y_val is not None:
            self.target_marker.goto(x_val, y_val)
            msg = f"WP,{x_val:.2f},{y_val:.2f}"
            self.sock.sendto(msg.encode(), RUST_SEND_ADDR)
            self.sock.sendto(b"START_MISSION", RUST_SEND_ADDR)

    def set_origin(self):
        self.sock.sendto(b"SET_ORIGIN", RUST_SEND_ADDR)
        self.rover.penup()
        self.rover.clear()
        self.rover.goto(0, 0)
        self.target_marker.goto(100, 100)
        print("Origin Reset.")

    def emergency_stop(self):
        self.sock.sendto(b"STOP", RUST_SEND_ADDR)
        self.target_marker.goto(100, 100)

    def clear_map(self):
        self.rover.clear()

    def update_loop(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            msg = data.decode().split(',')
            
            # Parsing: POS, X, Y, Heading, TargetX, TargetY
            if msg[0] == "POS" and len(msg) >= 6:
                rx, ry = float(msg[1]), float(msg[2])
                r_head = float(msg[3])
                tx, ty = float(msg[4]), float(msg[5])

                # The Turtle heading in Python is 0=East, 90=North.
                # If your Rust heading is different, you might need (r_head + 90)
                self.rover.goto(rx, ry)
                self.rover.setheading(r_head)
                self.rover.pendown() 

                # Hide target if arrived
                if abs(tx - rx) < 0.1 and abs(ty - ry) < 0.1:
                    self.target_marker.goto(100, 100)
                else:
                    self.target_marker.goto(tx, ty)

        except BlockingIOError:
            pass
        except Exception as e:
            pass # Silent catch for malformed packets

        self.screen.update()
        self.screen.ontimer(self.update_loop, 30) # 30ms for smoother tracking

if __name__ == "__main__":
    gui = CenteredGCS()
    turtle.done()