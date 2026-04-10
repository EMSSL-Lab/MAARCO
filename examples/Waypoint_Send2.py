# import turtle
# import socket

# # UPDATE THESE TO YOUR ACTUAL IPs
# RUST_SEND_ADDR = ("172.20.10.6", 5007) 
# PYTHON_LISTEN_ADDR = ("0.0.0.0", 5008)  

# class DebugControl:
#     def __init__(self):
#         self.screen = turtle.Screen()
#         self.screen.setup(width=1000, height=600)
#         self.screen.bgcolor("#1a1a1a")
#         self.screen.title("Rover Debug: Manual X,Y Entry")
#         self.screen.tracer(0)

#         # Map on Left, Controls on Right
#         self.view_size = 15
#         self.screen.setworldcoordinates(-self.view_size, -self.view_size, 
#                                          self.view_size + 10, self.view_size)

#         # UI Pen
#         self.ui_pen = turtle.Turtle(visible=False)
#         self.ui_pen.penup()
#         self.ui_pen.color("white")
        
#         self.grid_tool = turtle.Turtle(visible=False)
#         self.draw_static_ui()

#         # Rover Visuals
#         self.rover = turtle.Turtle(shape="triangle")
#         self.rover.shapesize(1.2, 1.2)
#         self.rover.color("#2ecc71")
#         self.rover.penup()
#         self.rover.speed(0)

#         # Socket Setup
#         self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.sock.bind(PYTHON_LISTEN_ADDR)
#         self.sock.setblocking(False)

#         # Key Bindings (No Clicks!)
#         self.screen.listen()
#         self.screen.onkey(self.manual_coords, "i")   # [I]nput X,Y
#         self.screen.onkey(self.set_origin, "o")      # [O]rigin (Zero out)
#         self.screen.onkey(self.emergency_stop, "s")  # [S]top / Cancel
#         self.screen.onkey(self.clear_trail, "c")     # [C]lear Screen

#         self.update_loop()

#     def draw_static_ui(self):
#         self.grid_tool.clear()
#         # Divider Line
#         self.grid_tool.pencolor("#444444")
#         self.grid_tool.penup(); self.grid_tool.goto(0, -self.view_size); self.grid_tool.pendown(); self.grid_tool.goto(0, self.view_size)
        
#         # Grid (Left Side Only)
#         self.grid_tool.pencolor("#222222")
#         for i in range(-self.view_size, 1):
#             self.grid_tool.penup(); self.grid_tool.goto(i, -self.view_size); self.grid_tool.pendown(); self.grid_tool.goto(i, self.view_size)
#             self.grid_tool.penup(); self.grid_tool.goto(-self.view_size, i); self.grid_tool.pendown(); self.grid_tool.goto(0, i)

#         # Instructions on Right
#         self.ui_pen.goto(self.view_size/2 + 2, 5)
#         self.ui_pen.write("DEBUG COMMANDS\n\n[ I ]  Input X, Y Location\n[ O ]  Set Current as Origin (0,0)\n[ S ]  EMERGENCY STOP\n[ C ]  Clear Path Trail", 
#                           align="center", font=("Courier", 12, "bold"))
#         self.screen.update()

#     def manual_coords(self):
#         """Right side logic: Prompt for specific X and Y."""
#         try:
#             target_x = self.screen.numinput("Target", "Enter X coordinate:", default=0)
#             target_y = self.screen.numinput("Target", "Enter Y coordinate:", default=0)
            
#             if target_x is not None and target_y is not None:
#                 # Matches your original WP format
#                 msg = f"WP,{target_x:.2f},{target_y:.2f}"
#                 self.sock.sendto(msg.encode(), RUST_SEND_ADDR)
#                 print(f"Sent Waypoint: {target_x}, {target_y}")
#                 # Optional: trigger the start immediately if your Rust logic requires it
#                 self.sock.sendto(b"START_MISSION", RUST_SEND_ADDR)
#         except:
#             pass

#     def set_origin(self):
#         """Resets the rover's internal coordinate system to 0,0."""
#         self.sock.sendto(b"SET_ORIGIN", RUST_SEND_ADDR)
#         self.rover.penup()
#         self.rover.goto(0,0) # Reset visual
#         print("Command Sent: SET_ORIGIN")

#     def emergency_stop(self):
#         """Cancel current movement."""
#         self.sock.sendto(b"STOP", RUST_SEND_ADDR)
#         print("Command Sent: STOP")

#     def clear_trail(self):
#         self.rover.clear()

#     def update_loop(self):
#         try:
#             data, addr = self.sock.recvfrom(1024)
#             msg = data.decode().split(',')
#             if msg[0] == "POS" and len(msg) >= 4:
#                 rx, ry, r_head = float(msg[1]), float(msg[2]), float(msg[3])
                
#                 self.rover.goto(rx, ry)
#                 self.rover.setheading(r_head)
#                 self.rover.pendown() # Shows the path for debugging
                
#         except BlockingIOError:
#             pass
#         except Exception as e:
#             print(f"UDP Error: {e}")

#         self.screen.update()
#         self.screen.ontimer(self.update_loop, 50)

# if __name__ == "__main__":
#     gui = DebugControl()
#     turtle.done()

import turtle
import socket

# UPDATE THESE TO YOUR ACTUAL IPs
RUST_SEND_ADDR = ("172.20.10.6", 5007) 
PYTHON_LISTEN_ADDR = ("0.0.0.0", 5008)  

class CenteredGCS:
    def __init__(self):
        self.screen = turtle.Screen()
        self.screen.setup(width=1100, height=600)
        self.screen.bgcolor("#1a1a1a")
        self.screen.title("Rover GCS: Centered Origin")
        self.screen.tracer(0)

        # COORDINATE SYSTEM:
        # Left side (Map) is -10 to +10. 
        # Right side (UI) is +10 to +20.
        # Total X span: 30 units. Y span: 20 units.
        self.screen.setworldcoordinates(-10, -10, 20, 10)

        self.writer = turtle.Turtle(visible=False)
        self.writer.penup()
        self.writer.color("white")
        
        self.grid_tool = turtle.Turtle(visible=False)
        self.setup_layout()

        # The Rover - now starts at 0,0 (Center of the left half)
        self.rover = turtle.Turtle(shape="triangle")
        self.rover.shapesize(1.0, 1.0)
        self.rover.color("#2ecc71")
        self.rover.penup()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(PYTHON_LISTEN_ADDR)
        self.sock.setblocking(False)

        self.screen.listen()
        self.screen.onkey(self.input_wp, "i")
        self.screen.onkey(self.set_origin, "o")
        self.screen.onkey(self.emergency_stop, "s")
        self.screen.onkey(self.clear_map, "c")

        self.update_loop()

    def setup_layout(self):
        self.grid_tool.clear()
        
        # 1. Draw UI Background (Right Side: X from 10 to 20)
        self.grid_tool.penup()
        self.grid_tool.goto(10, -10)
        self.grid_tool.color("#2c3e50")
        self.grid_tool.begin_fill()
        for x, y in [(20, -10), (20, 10), (10, 10), (10, -10)]:
            self.grid_tool.goto(x, y)
        self.grid_tool.end_fill()

        # 2. Divider Line (At X = 10)
        self.grid_tool.pensize(3)
        self.grid_tool.color("white")
        self.grid_tool.goto(10, 10)
        self.grid_tool.goto(10, -10)

        # 3. Grid (Left Side: -10 to 10)
        self.grid_tool.pensize(1)
        self.grid_tool.color("#34495e")
        for i in range(-10, 11):
            # Vertical lines
            self.grid_tool.penup(); self.grid_tool.goto(i, -10); self.grid_tool.pendown(); self.grid_tool.goto(i, 10)
            # Horizontal lines
            self.grid_tool.penup(); self.grid_tool.goto(-10, i); self.grid_tool.pendown(); self.grid_tool.goto(10, i)

        # 4. Labels
        self.writer.goto(-9, 8)
        self.writer.write("MONITOR (0,0 is CENTER)", font=("Arial", 12, "bold"))
        
        self.writer.goto(11, 5)
        self.writer.write("CONTROLS", font=("Arial(Bold)", 14, "bold"))
        self.writer.goto(11, 1)
        self.writer.write("[ I ] New Waypoint\n[ O ] Set Origin\n[ S ] STOP\n[ C ] Clear Trail", 
                          font=("Courier", 10, "normal"))
        self.screen.update()

    def input_wp(self):
        x_val = self.screen.numinput("Waypoint", "Target X:", default=0)
        y_val = self.screen.numinput("Waypoint", "Target Y:", default=0)
        
        if x_val is not None and y_val is not None:
            # Check bounds: prevent driving into the UI area visually
            if x_val > 9.5:
                print("Warning: Waypoint is inside the UI panel!")
                
            msg = f"WP,{x_val:.2f},{y_val:.2f}"
            self.sock.sendto(msg.encode(), RUST_SEND_ADDR)
            self.sock.sendto(b"START_MISSION", RUST_SEND_ADDR)

    def set_origin(self):
        self.sock.sendto(b"SET_ORIGIN", RUST_SEND_ADDR)
        self.rover.penup()
        self.rover.goto(0, 0) # Snaps to center of map
        print("Origin Reset to Center.")

    def emergency_stop(self):
        self.sock.sendto(b"STOP", RUST_SEND_ADDR)

    def clear_map(self):
        self.rover.clear()

    def update_loop(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            msg = data.decode().split(',')
            if msg[0] == "POS" and len(msg) >= 4:
                rx, ry, r_head = float(msg[1]), float(msg[2]), float(msg[3])
                self.rover.goto(rx, ry)
                self.rover.setheading(r_head)
                self.rover.pendown() 
        except BlockingIOError:
            pass
        except Exception as e:
            print(f"Data error: {e}")

        self.screen.update()
        self.screen.ontimer(self.update_loop, 50)

if __name__ == "__main__":
    gui = CenteredGCS()
    turtle.done()