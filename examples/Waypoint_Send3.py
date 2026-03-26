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
        self.screen.title("Rover GCS: EKF + Target Tracking")
        self.screen.tracer(0)

        self.screen.setworldcoordinates(-10, -10, 20, 10)

        self.writer = turtle.Turtle(visible=False)
        self.writer.penup()
        self.writer.color("white")
        
        self.grid_tool = turtle.Turtle(visible=False)
        
        # --- NEW: Target Marker Turtle ---
        self.target_marker = turtle.Turtle(shape="circle")
        self.target_marker.shapesize(0.5, 0.5)
        self.target_marker.color("#e74c3c") # Red
        self.target_marker.penup()
        # Start it off-screen until a waypoint is set
        self.target_marker.goto(100, 100) 

        self.setup_layout()

        self.rover = turtle.Turtle(shape="triangle")
        self.rover.shapesize(1.0, 1.0)
        self.rover.color("#2ecc71") # Green
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
        # UI Background
        self.grid_tool.penup()
        self.grid_tool.goto(10, -10)
        self.grid_tool.color("#2c3e50")
        self.grid_tool.begin_fill()
        for x, y in [(20, -10), (20, 10), (10, 10), (10, -10)]:
            self.grid_tool.goto(x, y)
        self.grid_tool.end_fill()

        # Divider and Grid
        self.grid_tool.pensize(3); self.grid_tool.color("white")
        self.grid_tool.goto(10, 10); self.grid_tool.goto(10, -10)
        self.grid_tool.pensize(1); self.grid_tool.color("#34495e")
        for i in range(-10, 11):
            self.grid_tool.penup(); self.grid_tool.goto(i, -10); self.grid_tool.pendown(); self.grid_tool.goto(i, 10)
            self.grid_tool.penup(); self.grid_tool.goto(-10, i); self.grid_tool.pendown(); self.grid_tool.goto(10, i)

        self.writer.goto(11, 5)
        self.writer.write("CONTROLS", font=("Arial", 14, "bold"))
        self.writer.goto(11, 1)
        self.writer.write("[ I ] New Waypoint\n[ O ] Set Origin\n[ S ] STOP\n[ C ] Clear Trail", font=("Courier", 10, "normal"))

    def input_wp(self):
        x_val = self.screen.numinput("Waypoint", "Target X:", default=0)
        y_val = self.screen.numinput("Waypoint", "Target Y:", default=0)
        if x_val is not None and y_val is not None:
            # Move the red dot immediately for visual feedback
            self.target_marker.goto(x_val, y_val)
            msg = f"WP,{x_val:.2f},{y_val:.2f}"
            self.sock.sendto(msg.encode(), RUST_SEND_ADDR)
            self.sock.sendto(b"START_MISSION", RUST_SEND_ADDR)

    def set_origin(self):
        self.sock.sendto(b"SET_ORIGIN", RUST_SEND_ADDR)
        self.rover.penup()
        self.rover.clear() # Clear the trail on origin reset
        self.rover.goto(0, 0)
        self.target_marker.goto(100, 100) # Hide target
        print("Origin Reset.")

    def emergency_stop(self):
        self.sock.sendto(b"STOP", RUST_SEND_ADDR)
        self.target_marker.goto(100, 100) # Hide target dot on stop

    def clear_map(self):
        self.rover.clear()

    def update_loop(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            msg = data.decode().split(',')
            
            # Now parsing the 6 values: POS, X, Y, Head, TargetX, TargetY
            if msg[0] == "POS" and len(msg) >= 6:
                rx, ry = float(msg[1]), float(msg[2])
                r_head = float(msg[3])
                tx, ty = float(msg[4]), float(msg[5])

                # Update Rover Position
                self.rover.goto(rx, ry)
                self.rover.setheading(r_head)
                self.rover.pendown() 

                # Update Target Dot Position from Rust's "True" target
                # (This ensures the dot stays where the RUST mission thinks it is)
                if abs(tx - rx) < 0.01 and abs(ty - ry) < 0.01:
                    self.target_marker.goto(100, 100) # Hide if we are basically there
                else:
                    self.target_marker.goto(tx, ty)

        except BlockingIOError:
            pass
        except Exception as e:
            print(f"Data error: {e}")

        self.screen.update()
        self.screen.ontimer(self.update_loop, 50)

if __name__ == "__main__":
    gui = CenteredGCS()
    turtle.done()