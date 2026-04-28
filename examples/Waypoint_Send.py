import turtle
import socket
import math
import time
# UPDATE THESE TO YOUR ACTUAL IPs
RUST_SEND_ADDR = ("172.20.10.4", 5007) 
PYTHON_LISTEN_ADDR = ("0.0.0.0", 5008)  

class MissionControl:
    def __init__(self):
        self.screen = turtle.Screen()
        
        self.screen.setup(width=0.5, height=0.75)
        self.screen.bgcolor("#2c3e50")
        self.screen.title("Rover GCS - [1] Start | [2] Clear | [3] STOP | [4] Set Origin | [5] Undo | [6] RPM+ | [7] RPM- | Scroll to Zoom")
        
        # Scale: 1 unit = 1 meter
        self.view_size = 5.0 
        self.screen.setworldcoordinates(-self.view_size, -self.view_size, 
                                         self.view_size, self.view_size)
        self.screen.tracer(0)

        # UI Text Drawer
        self.nav_pen = turtle.Turtle()
        self.nav_pen.hideturtle()
        self.nav_pen.penup()
        self.nav_pen.color("white")
        self.ui_pen = turtle.Turtle()
        self.ui_pen.hideturtle()
        self.ui_pen.penup()
        self.ui_pen.color("white")
        self.scale_pen = turtle.Turtle()
        self.scale_pen.hideturtle()
        self.scale_pen.penup()
        self.scale_pen.color("white")

        # Navigation Status Pen (Actively Navigating vs Idle)
        self.state_pen = turtle.Turtle()
        self.state_pen.hideturtle()
        self.state_pen.penup()
        
        # Keep track of the last drawn state so we don't cause flicker
        self.last_drawn_state = None

        # Grid and Rover setup
        self.grid_tool = turtle.Turtle(visible=False)
        self.draw_grid()
        self.draw_origin()
        self.draw_scale_bar()
        
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
        self.screen.onkey(self.undo_waypoint, "5")
        self.screen.onkey(self.increase_rpm, "6")
        self.screen.onkey(self.decrease_rpm, "7")
        self.screen.cv.bind("<MouseWheel>", self.zoom)
        # --- NEW NAVIGATION STATE VARIABLES ---
        self.target_rpm = 30.0 # Default starting RPM
        self.waypoint_queue = []     # List to hold (x, y) tuples
        self.is_navigating = False   # Lock to prevent spamming commands
        self.active_waypoint = None
        self.current_rover_x = 0.0   # Absolute X position from Rust
        self.current_rover_y = 0.0   # Absolute Y position from Rust
        self.distance = 0.0          # Distance to next waypoint
        self.target_angle = 0.0      # Bearing to next waypoint
        # --------------------------------------

        self.last_telemetry_time = 0.0
        self.update_status("READY: Click to set points")
        self.update_rover()
        
        

    def draw_nav_info(self, dist, bearing):
        self.nav_pen.clear()
        x_pos = -self.view_size + 0.5 
        y_pos = self.view_size - 0.8
        self.nav_pen.goto(x_pos, y_pos)
        
        # Added RPM to the display string
        info_str = f"DIST: {dist:.2f}m\nBRNG: {bearing:.1f}°\nRPM: {self.target_rpm:.1f}"
        
        self.nav_pen.write(info_str, align="left", font=("Arial", 12, "bold"))
    
    def draw_nav_state(self):
        # Only redraw if the state has changed to prevent screen flickering
        if self.last_drawn_state == self.is_navigating:
            return
            
        self.state_pen.clear()
        # Place it at the very top center of the screen
        self.state_pen.goto(0, self.view_size - 0.2) 
        
        if self.is_navigating:
            self.state_pen.color("#12f35d") # Warning Orange
            self.state_pen.write("STATUS: ACTIVELY NAVIGATING", align="center", font=("Arial", 14, "bold"))
        else:
            self.state_pen.color("#ac7715") # Idle Grey
            self.state_pen.write("STATUS: IDLE / STANDBY", align="center", font=("Arial", 14, "bold"))
            
        self.last_drawn_state = self.is_navigating

    def draw_scale_bar(self):
        length = self.view_size / 5
        x = self.view_size - length - 0.5
        y = -self.view_size + 1.0
        self.scale_pen.clear()
        self.scale_pen.goto(x, y)
        self.scale_pen.pendown()
        self.scale_pen.goto(x + length, y)
        self.scale_pen.penup()
        self.scale_pen.goto(x + length / 2, y - 0.2)
        if length >= 0.1:
            self.scale_pen.write(f"{length:.1f}m", align="center", font=("Verdana", 8, "normal"))
        else:
            self.scale_pen.write(f"{int(length * 100)}cm", align="center", font=("Verdana", 8, "normal"))
        self.screen.update()
        
    # def draw_grid(self):
    #     self.grid_tool.clear()
    #     self.grid_tool.pencolor("#34495e")
    #     for i in range(-self.view_size, self.view_size + 1):
    #         self.grid_tool.penup(); self.grid_tool.goto(i, -self.view_size); self.grid_tool.pendown(); self.grid_tool.goto(i, self.view_size)
    #         self.grid_tool.penup(); self.grid_tool.goto(-self.view_size, i); self.grid_tool.pendown(); self.grid_tool.goto(self.view_size, i)
    #     self.screen.update()
    def draw_grid(self):
        self.grid_tool.clear()
        current = -self.view_size
        while current <= self.view_size:
            # --- 1. Draw the Grid Lines ---
            # Highlight the origin axes (X=0 and Y=0)
            if current == 0:
                self.grid_tool.pencolor("#7f8c8d") # Lighter grey for center axis
                self.grid_tool.pensize(2)
            else:
                self.grid_tool.pencolor("#34495e") # Dark grey for regular grid
                self.grid_tool.pensize(1)

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

            # --- 2. Draw the Unit Labels ---
            self.grid_tool.pencolor("#bdc3c7") # Light whitish-grey for text
            self.grid_tool.penup()
            
            # X-axis labels (drawn along the bottom edge)
            if current != 0 and current != -self.view_size: 
                self.grid_tool.goto(current, -self.view_size + 0.2)
                self.grid_tool.write(f"{int(current)}", align="center", font=("Verdana", 8, "normal"))
            
            # Y-axis labels (drawn along the left edge)
            if current != 0 and current != -self.view_size:
                self.grid_tool.goto(-self.view_size + 0.2, current - 0.2)
                self.grid_tool.write(f"{int(current)}", align="left", font=("Verdana", 8, "normal"))

            # Label the origin (0,0) exactly in the center
            if current == 0:
                self.grid_tool.goto(0.2, 0.2)
                self.grid_tool.write("0", align="left", font=("Verdana", 8, "bold"))

            current += 1 # Move 1 unit (1 meter)

        # --- 3. Draw Cardinal Direction Labels ---
        self.grid_tool.pencolor("#11100F") # Orange color for directions
        # North (+Y)
        self.grid_tool.goto(0, self.view_size - 0.5)
        self.grid_tool.write("NORTH", align="center", font=("Verdana", 10, "bold"))
        # South (-Y)
        self.grid_tool.goto(0, -self.view_size + 0.1)
        self.grid_tool.write("SOUTH", align="center", font=("Verdana", 10, "bold"))
        # East (+X)
        self.grid_tool.goto(self.view_size - 0.1, 0.1)
        self.grid_tool.write("EAST", align="right", font=("Verdana", 10, "bold"))
        # West (-X)
        self.grid_tool.goto(-self.view_size + 0.1, 0.1)
        self.grid_tool.write("WEST", align="left", font=("Verdana", 10, "bold"))

        self.screen.update()

    def draw_origin(self):
        # Create a dedicated turtle just for the origin marker
        origin_pen = turtle.Turtle(visible=False)
        origin_pen.speed(0)
        origin_pen.penup()
        
        # Go to (0,0) and stamp a grey dot
        origin_pen.goto(0, 0)
        origin_pen.dot(12, "grey")
        
        # Add a small text label underneath it
        origin_pen.sety(-0.3) # Shift down slightly based on your scale
        origin_pen.color("grey")
        origin_pen.write("Origin (Start)", align="center", font=("Arial", 10, "bold"))

    def update_status(self, text):
        self.ui_pen.clear()
        self.ui_pen.goto(-self.view_size + 1, self.view_size - 1.5)
        self.ui_pen.write(f"STATUS: {text}", font=("Verdana", 14, "bold"))
        self.ui_pen.goto(-self.view_size + 1, self.view_size - 2.5)
        self.ui_pen.write("[1] START | [2] CLEAR | [3] STOP | [4] SET ORIGIN | [5] UNDO", font=("Verdana", 10, "normal"))
        self.screen.update()

    def draw_path(self):
        self.drawer.clear()
        self.drawer.penup()
        
        curr_x, curr_y = self.current_rover_x, self.current_rover_y
        
        # 1. DRAW THE ACTIVE LEG
        if self.active_waypoint:
            self.drawer.goto(curr_x, curr_y)
            
            # Switch color based on navigation state
            if self.is_navigating:
                self.drawer.color("#2ecc71") # Green
            else:
                self.drawer.color("#f39c12") # Orange (Paused)
                
            self.drawer.pensize(3)
            self.drawer.pendown()
            self.drawer.goto(self.active_waypoint[0], self.active_waypoint[1])
            self.drawer.penup()
            self.drawer.dot(10) 
            
            start_x, start_y = self.active_waypoint[0], self.active_waypoint[1]
        else:
            start_x, start_y = curr_x, curr_y

        # 2. DRAW THE QUEUED LEGS (RED)
        self.drawer.color("#e74c3c")
        self.drawer.pensize(1)
        for wp in self.waypoint_queue:
            self.drawer.goto(start_x, start_y)
            self.drawer.pendown()
            self.drawer.goto(wp[0], wp[1])
            self.drawer.penup()
            self.drawer.dot(8, "#e74c3c")
            start_x, start_y = wp[0], wp[1]

    def handle_click(self, x, y):
        # If the click is too close to the center, ignore it
        dist_from_center = math.sqrt(x**2 + y**2)
        if dist_from_center < 1.0: 
            print("Point too close to origin! Click further away.")
            return

        # 1. Add to Queue
        self.waypoint_queue.append((x, y))
        
        # 2. Redraw the path
        self.draw_path()
        
        print(f"Queued WP: ({x:.2f}, {y:.2f}) | Queue size: {len(self.waypoint_queue)}")
        self.screen.update()

        # --- UPDATE UI TEXT IMMEDIATELY ON CLICK ---
        if not self.is_navigating and len(self.waypoint_queue) == 1:
            dx = x - self.current_rover_x
            dy = y - self.current_rover_y
            
            dist = math.sqrt(dx**2 + dy**2)
            target_angle = math.degrees(math.atan2(dx, dy))
            if target_angle < 0:
                target_angle += 360
                
            self.draw_nav_info(dist, target_angle)

    def start_mission(self):
        if self.is_navigating:
            print("Already navigating!")
            return

        # Case A: Resume existing waypoint
        if self.active_waypoint:
            print("Resuming mission...")
            self.is_navigating = True
            self.send_nav_command(self.active_waypoint[0], self.active_waypoint[1])
        
        # Case B: Grab from queue
        elif self.waypoint_queue:
            print("Starting mission from queue...")
            self.is_navigating = True
            self.active_waypoint = self.waypoint_queue.pop(0)
            self.send_nav_command(self.active_waypoint[0], self.active_waypoint[1])
        

        self.draw_path() # Force refresh to turn line Green

    def emergency_stop(self):
        self.sock.sendto(b"STOP", RUST_SEND_ADDR)
        self.is_navigating = False 
        self.draw_path() # Force refresh to turn line Orange
        self.update_status("EMERGENCY STOP SENT")

    def clear_mission(self):
        # Rust doesn't have a formal CLEAR yet, so we just clear the UI
        self.drawer.clear()
        self.drawer.penup()
        self.waypoint_queue.clear() # Empty queue
        self.update_status("CLEARED: Ready for new points")
        self.distance = 0.0
        self.target_angle = 0.0
        self.active_waypoint = None
        self.is_navigating = False  
        self.draw_path()
        self.screen.update()
        print("Mission cleared!, Navigation reset.")
        

    def emergency_stop(self):
        # We can send a command that triggers (1500, 1500) in Rust
        self.sock.sendto(b"STOP", RUST_SEND_ADDR)
        
        self.is_navigating = False 
        
        self.draw_path() 
        self.update_status("EMERGENCY STOP SENT")
        print("Sent: STOP Command to Rover!")

    # Add this new method to the class:
    def set_origin(self):
        self.sock.sendto(b"SET_ORIGIN", RUST_SEND_ADDR)
        self.update_status("ORIGIN SET: Rover at (0,0)")
        print("Sent: SET_ORIGIN")
        
    def undo_waypoint(self):
        if self.waypoint_queue:
            self.waypoint_queue.pop()
            
            # Redraw the path instantly
            self.draw_path()
            self.screen.update()
            
            print(f"Undid last waypoint. Queue size: {len(self.waypoint_queue)}")
        else:
            print("No waypoints to undo.")
            
    def zoom(self, event):
        delta = event.delta
        factor = 1.1 if delta > 0 else 0.9
        self.view_size *= factor
        self.view_size = max(0.5, min(20.0, self.view_size))
        self.screen.setworldcoordinates(-self.view_size, -self.view_size, self.view_size, self.view_size)
        self.draw_grid()
        self.draw_scale_bar()
        self.screen.update()
        
    # Add this new method:
    def draw_heartbeat(self, active):
        self.ui_pen.clear()
        # Position the pen in the top left or top right
        self.ui_pen.goto(-self.view_size + 0.5, self.view_size - 1)
        
        if active:
            self.ui_pen.color("#2ecc71") # Emerald Green
            self.ui_pen.write("● SYSTEM ACTIVE", font=("Arial", 12, "bold"))
        else:
            self.ui_pen.color("#e74c3c") # Alizarin Red
            self.ui_pen.write("● SYSTEM OFFLINE", font=("Arial", 12, "bold"))

    def increase_rpm(self):
        self.target_rpm += 5.0
        self.send_rpm_to_rust()

    def decrease_rpm(self):
        self.target_rpm = max(0, self.target_rpm - 5.0) # Prevent negative RPM
        self.send_rpm_to_rust()

    def send_rpm_to_rust(self):
        # Use the existing socket to send the new message type
        rpm_msg = f"RPM,{self.target_rpm:.1f}"
        self.sock.sendto(rpm_msg.encode(), RUST_SEND_ADDR)
        
        # Refresh the UI text immediately
        # Using a dummy distance/bearing if not navigating
        self.draw_nav_info(self.distance, self.target_angle)
        print(f"Sent RPM Update: {self.target_rpm}")

    def draw_rpm_ui(self):
   
        self.ui_pen.color("white")
        self.ui_pen.goto(self.view_size - 1.5, self.view_size - 0.8) # Positioned below status
        # Clear a small area first if needed, or just overwrite since UI pen clears often
        self.ui_pen.write(f"Target RPM: {self.target_rpm:.1f}", font=("Arial", 11, "normal"))

    def send_nav_command(self, tx, ty):
        """Calculates distance/angle and sends the NAV packet to Rust."""
        # Calculate distance and angle relative to current position
        dx = tx - self.current_rover_x
        dy = ty - self.current_rover_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Calculate target angle for Rust
        target_angle = math.degrees(math.atan2(dy, dx))
        
        # Send the Navigation instructions to Rust
        nav_msg = f"NAV,{distance:.3f},{target_angle:.3f}"
        self.sock.sendto(nav_msg.encode(), RUST_SEND_ADDR)
        
        # Update UI text
        self.draw_nav_info(distance, target_angle)
        self.update_status(f"NAVIGATING to ({tx:.1f}, {ty:.1f})")
        print(f"Sent NAV: {distance:.2f}m at {target_angle:.1f}°")

    ### NEW METHOD TO SEND THE NEXT WAYPOINT IN THE QUEUE TO RUST
    def send_next_waypoint(self):
        if len(self.waypoint_queue) == 0:
            self.active_waypoint = None
            self.is_navigating = False
            self.draw_path()
            self.update_status("IDLE: All waypoints reached")
            return
        
        tx, ty = self.waypoint_queue.pop(0)
        self.active_waypoint = (tx, ty)
        self.is_navigating = True
        self.send_nav_command(tx, ty)
        self.draw_path()


    def update_rover(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            self.draw_heartbeat(True)
            
            msg = data.decode().split(',')
            
            # --- 1. UPDATE LIVE POSITION ---
            if msg[0] == "TELEM" and len(msg) >= 4:
                self.current_rover_x = float(msg[1])
                self.current_rover_y = float(msg[2])
                rover_yaw = float(msg[3])
                
                # CONVERT RUST HEADING BACK TO PYTHON HEADING FOR UI
                turtle_angle = (90 - rover_yaw) % 360
                
                # Move Turtle
                self.rover.goto(self.current_rover_x, self.current_rover_y)
                self.rover.setheading(turtle_angle)

                # Redraw path so the green line anchors to the moving rover
                self.draw_path()
                
                # --- NEW: CALCULATE REMAINING DISTANCE ---
                # Figure out what point we should be measuring to
                target_wp = None
                
                if self.is_navigating and self.active_waypoint:
                    target_wp = self.active_waypoint          # Driving: Measure to active point
                elif not self.is_navigating and len(self.waypoint_queue) > 0:
                    target_wp = self.waypoint_queue[0]        # Idle: Measure to the first queued point
                    
                # If we have a target, calculate the distance and show it
                # Calculate distance to the active waypoint for the UI text
                if self.active_waypoint:
                    dx = self.active_waypoint[0] - self.current_rover_x
                    dy = self.active_waypoint[1] - self.current_rover_y
                    remaining_dist = math.sqrt(dx**2 + dy**2)
                    self.draw_nav_info(remaining_dist, rover_yaw)
                else:
                    # If there are no waypoints at all, clear the text
                    self.nav_pen.clear()
            
            # --- 2. HANDLE ARRIVAL NOTIFICATION ---
            elif msg[0] == "ARRIVED":
                print("\n[SUCCESS] Rover reached waypoint!")
                self.is_navigating = False  # Unlock the system
                self.send_next_waypoint()   # Instantly grab the next WP in the queue!
            self.last_telemetry_time = time.time()  # Update heartbeat timestamp on any message received

        except BlockingIOError:
            pass
        except Exception as e:
            print(f"Socket Error: {e}")
        if time.time() - self.last_telemetry_time > 2.0:
            self.draw_heartbeat(False)
        self.draw_nav_state()
        
        self.screen.update()
        self.screen.ontimer(self.update_rover, 50)
    

if __name__ == "__main__":
    gui = MissionControl()
    turtle.done()