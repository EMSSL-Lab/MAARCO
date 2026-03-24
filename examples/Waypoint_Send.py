import turtle
import socket
import math
# UPDATE THESE TO YOUR ACTUAL IPs
RUST_SEND_ADDR = ("172.20.10.6", 5007) 
PYTHON_LISTEN_ADDR = ("0.0.0.0", 5008)  

class MissionControl:
    def __init__(self):
        self.screen = turtle.Screen()
        self.screen.setup(width=0.9, height=0.9)
        self.screen.bgcolor("#2c3e50")
        self.screen.title("Rover GCS - [1] Start | [2] Clear | [3] STOP | [4] Set Origin")
        
        # Scale: 1 unit = 1 meter
        self.view_size = 15 
        self.screen.setworldcoordinates(-self.view_size, -self.view_size, 
                                         self.view_size, self.view_size)
        self.screen.tracer(0)

        # UI Text Drawer
        self.ui_pen = turtle.Turtle()
        self.ui_pen.hideturtle()
        self.ui_pen.penup()
        self.ui_pen.color("white")

        # Grid and Rover setup
        self.grid_tool = turtle.Turtle(visible=False)
        self.draw_grid()
        
        self.drawer = turtle.Turtle() # Waypoint drawer
        self.drawer.pencolor("#e74c3c")
        self.drawer.penup()

        self.rover = turtle.Turtle(shape="triangle")
        self.rover.shapesize(1.2, 1.2)
        self.rover.color("#2ecc71")
        self.rover.penup()

        # 1. Initialize variables FIRST
        self.last_heartbeat = 0
        
        # 2. Setup the Heartbeat Turtle BEFORE calling update_rover
        self.heartbeat_turtle = turtle.Turtle(visible=False)
        self.heartbeat_turtle.penup()
        # Position it in the top right corner
        self.heartbeat_turtle.goto(self.view_size - 1.5, self.view_size - 1.5)

        # Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(PYTHON_LISTEN_ADDR)
        self.sock.setblocking(False)

        # Keyboard Bindings
        self.screen.listen()
        self.screen.onclick(self.handle_click)
        self.screen.onkey(self.start_mission, "1")
        self.screen.onkey(self.clear_mission, "2")
        self.screen.onkey(self.emergency_stop, "3")
        self.screen.onkey(self.set_origin, "4")

        self.update_status("READY: Click to set points")
        self.update_rover()

    def draw_nav_info(self, dist, bearing):
        # Use a dedicated pen to avoid flickering the whole screen
        self.ui_pen.undo() # Optional: if you want to overwrite specific lines
        # Better yet, just clear a specific area:
        self.ui_pen.goto(-self.view_size + 1, -self.view_size + 2)
        self.ui_pen.color("yellow")
        info = f"DISTANCE TO WP: {dist:.2f}m | BEARING: {bearing:.1f}°"
        self.ui_pen.write(info, font=("Verdana", 12, "bold"))
        
    # def draw_grid(self):
    #     self.grid_tool.clear()
    #     self.grid_tool.pencolor("#34495e")
    #     for i in range(-self.view_size, self.view_size + 1):
    #         self.grid_tool.penup(); self.grid_tool.goto(i, -self.view_size); self.grid_tool.pendown(); self.grid_tool.goto(i, self.view_size)
    #         self.grid_tool.penup(); self.grid_tool.goto(-self.view_size, i); self.grid_tool.pendown(); self.grid_tool.goto(self.view_size, i)
    #     self.screen.update()
    def draw_grid(self):
        self.grid_tool.clear()
        self.grid_tool.pencolor("#34495e")
        
        # Draw a line every 0.1 units (10cm)
        # We use a while loop because range() only works with integers
        start = -int(self.view_size)
        end = int(self.view_size)
        
        current = -self.view_size
        while current <= self.view_size:
            # Vertical lines
            self.grid_tool.penup()
            self.grid_tool.goto(current, -self.view_size)
            self.grid_tool.pendown()
            self.grid_tool.goto(current, self.view_size)
            
            # Horizontal lines
            self.grid_tool.penup()
            self.grid_tool.goto(-self.view_size, current)
            self.grid_tool.pendown()
            self.grid_tool.goto(self.view_size, current)
            
            current += 1 # Move 10cm
            
        self.screen.update()

    def update_status(self, text):
        self.ui_pen.clear()
        self.ui_pen.goto(-self.view_size + 1, self.view_size - 1.5)
        self.ui_pen.write(f"STATUS: {text}", font=("Verdana", 14, "bold"))
        self.ui_pen.goto(-self.view_size + 1, self.view_size - 2.5)
        self.ui_pen.write("[1] START | [2] CLEAR | [3] STOP", font=("Verdana", 10, "normal"))

    def handle_click(self, x, y):
        # If the click is too close to the center, ignore it
        dist_from_center = math.sqrt(x**2 + y**2)
        if dist_from_center < 1.0: 
            print("Point too close to origin! Click further away.")
            return
            # Only add WPs, don't send START yet
        msg = f"WP,{x:.2f},{y:.2f}"
        self.sock.sendto(msg.encode(), RUST_SEND_ADDR)
        self.drawer.goto(x, y)
        self.drawer.dot(8, "#e74c3c")
        self.drawer.pendown()
        print(f"Added WP: {x:.2f}, {y:.2f}")

    def start_mission(self):
        self.sock.sendto(b"START_MISSION", RUST_SEND_ADDR)
        self.update_status("MISSION RUNNING...")
        print("Sent: START_MISSION")

    def clear_mission(self):
        # Rust doesn't have a formal CLEAR yet, so we just clear the UI
        self.drawer.clear()
        self.drawer.penup()
        self.update_status("CLEARED: Ready for new points")
        print("UI Cleared locally")

    def emergency_stop(self):
        # We can send a command that triggers (1500, 1500) in Rust
        self.sock.sendto(b"STOP", RUST_SEND_ADDR)
        self.update_status("EMERGENCY STOP SENT")
        print("Sent: STOP")

    # Add this new method to the class:
    def set_origin(self):
        self.sock.sendto(b"SET_ORIGIN", RUST_SEND_ADDR)
        self.update_status("ORIGIN SET: Rover at (0,0)")
        print("Sent: SET_ORIGIN")
        
    # Add this new method:
    def draw_heartbeat(self, active):
        self.heartbeat_turtle.clear()
        color = "#2ecc71" if active else "#e74c3c"
        self.heartbeat_turtle.dot(15, color)
            
    def update_rover(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            self.draw_heartbeat(True)
            
            msg = data.decode().split(',')
            if msg[0] == "POS" and len(msg) >= 6:
                # Parse current pos
                rx, ry, r_head = float(msg[1]), float(msg[2]), float(msg[3])
                # Parse target pos
                tx, ty = float(msg[4]), float(msg[5])

                # 1. Update Rover position/heading
                self.rover.goto(rx, ry)
                self.rover.setheading(r_head)

                # 2. Calculate Distance and Direction
                dx = tx - rx
                dy = ty - ry
                distance = math.sqrt(dx**2 + dy**2)
                # Calculate angle (atan2 gives radians, we convert to degrees)
                target_angle = math.degrees(math.atan2(dy, dx))

                # 3. Print to UI
                self.draw_nav_info(distance, target_angle)

        except BlockingIOError:
            pass
        except Exception as e:
            print(f"Nav Error: {e}")

        self.screen.update()
        self.screen.ontimer(self.update_rover, 50)

if __name__ == "__main__":
    gui = MissionControl()
    turtle.done()