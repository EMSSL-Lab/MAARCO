import turtle
import socket
import tkinter as tk
from tkinter import ttk

# --- CONFIGURATION ---
RUST_SEND_ADDR = ("172.20.10.6", 5007) 
PYTHON_LISTEN_ADDR = ("0.0.0.0", 5008)  

class GCSApp:
    def __init__(self, root):
        self.root = root
        self.root.title("MAARCO GCS: EKF Tuner")
        
        # --- DATA STATE ---
        self.current_pos = (0.0, 0.0)
        self.total_dist_rust = 0.0
        self.tot_dist_rust = 0.0
        self.rel_dist_rust = 0.0
        self.error_percent = 0.0
        self.truth_dist = 5.0 # Default goal
        
        # --- UI LAYOUT ---
        # Left Panel: Turtle Map
        self.canvas = tk.Canvas(root, width=800, height=600)
        self.canvas.pack(side=tk.LEFT, padx=10, pady=10)
        
        # Right Panel: Controls
        self.ctrl_frame = ttk.Frame(root, padding="10")
        self.ctrl_frame.pack(side=tk.RIGHT, fill=tk.Y)

        self.setup_turtle()
        self.setup_controls()
        self.setup_network()
        self.update_loop()

    def setup_turtle(self):
        self.screen = turtle.TurtleScreen(self.canvas)
        self.screen.bgcolor("#1a1a1a")
        self.screen.tracer(0)

        dim_x = 10  # Meters in x direction
        dim_y = 10  # Meters in y direction
        self.screen.setworldcoordinates(-dim_x, -dim_y, dim_x, dim_y)
        
        # Grid
        grid = turtle.RawTurtle(self.screen, visible=False)
        grid.pencolor("#34495e")
        for i in range(-int(dim_x), int(dim_x) + 1):
            grid.penup(); grid.goto(i, -int(dim_y)); grid.pendown(); grid.goto(i, int(dim_y))
            grid.penup(); grid.goto(-int(dim_x), i); grid.pendown(); grid.goto(int(dim_x), i)

        # Add unit labels
        label = turtle.RawTurtle(self.screen, visible=False)
        label.pencolor("#ffffff")
        label.penup()
        for i in range(-int(dim_x), int(dim_x) + 1):
            # X-axis labels at bottom
            label.goto(i, -0.2)
            label.write(str(i), align="center", font=("Arial", 8, "normal"))
            # Y-axis labels at left
            label.goto(-0.2, i)
            label.write(str(i), align="right", font=("Arial", 8, "normal"))

        # Axis labels
        label.goto(dim_x - 1, -1)
        label.write("X (m)", align="left", font=("Arial", 10, "bold"))
        label.goto(-1, dim_y - 1)
        label.write("Y (m)", align="center", font=("Arial", 10, "bold"))

        # Compass labels
        label.goto(0.5, dim_y - 1)
        label.write("N", align="center", font=("Arial", 12, "bold"))
        label.goto(dim_x  - 1, 0.5)
        label.write("E", align="center", font=("Arial", 12, "bold"))
        label.goto(0.5, -dim_y + 1)
        label.write("S", align="center", font=("Arial", 12, "bold"))
        label.goto(-dim_x + 1, 0.5)
        label.write("W", align="center", font=("Arial", 12, "bold"))

        self.rover = turtle.RawTurtle(self.screen, shape="triangle")
        self.rover.color("#2ecc71")
        self.rover.penup()

        self.gps_marker = turtle.RawTurtle(self.screen, visible=False)
        self.gps_marker.penup()
        self.gps_marker.color("red")

    def setup_controls(self):
        # --- EKF TUNING SLIDERS ---
        ttk.Label(self.ctrl_frame, text="EKF TUNING", font=('Arial', 12, 'bold')).pack(pady=5)
        
        # Process Noise (Q)
        self.q_pos_s, _ = self.create_slider("Q Position", 0.001, 0.1, 0.01)
        self.q_vel_s, _ = self.create_slider("Q Velocity", 0.01, 0.5, 0.1)
        self.q_ori_s, _ = self.create_slider("Q Orientation", 0.0001, 0.01, 0.001)
        
        self.r_fix_s, _ = self.create_slider("R RTK Fix", 0.00001, 0.001, 0.0001)
        self.r_float_s, _ = self.create_slider("R RTK Float", 0.01, 1.0, 0.25)
        self.cutoff_hz_s, _ = self.create_slider("Cutoff Hz", 1.0, 50.0, 10.0)
        
        ttk.Button(self.ctrl_frame, text="Send Gains", command=self.send_gains).pack(pady=10)
        
        ttk.Separator(self.ctrl_frame).pack(fill=tk.X, pady=10)

        # --- TRUTH TESTER ---
        ttk.Label(self.ctrl_frame, text="DISTANCE TRUTH TEST", font=('Arial', 12, 'bold')).pack(pady=5)
        
        ttk.Label(self.ctrl_frame, text="Planned Walk (m):").pack()
        self.truth_entry = ttk.Entry(self.ctrl_frame)
        self.truth_entry.insert(0, "7.5")
        self.truth_entry.pack()

        self.error_label = ttk.Label(self.ctrl_frame, text="Error: 0.0%", font=('Courier', 12))
        self.error_label.pack(pady=10)

        ttk.Button(self.ctrl_frame, text="Reset Odometer", command=self.reset_odom).pack(pady=5)

    def create_slider(self, name, start, end, default):
        """Creates a slider and a label that shows its current value."""
        frame = ttk.Frame(self.ctrl_frame)
        frame.pack(fill=tk.X, pady=2)
        
        # The Label (e.g., "Q Position: 0.0100")
        label_var = tk.StringVar(value=f"{name}: {default:.4f}")
        lbl = ttk.Label(frame, textvariable=label_var)
        lbl.pack(side=tk.TOP, anchor=tk.W)
        
        var = tk.DoubleVar(value=default)
        
        # Update function called every time the slider moves
        def update_label(val):
            label_var.set(f"{name}: {float(val):.4f}")

        s = ttk.Scale(frame, from_=start, to=end, orient=tk.HORIZONTAL, 
                      variable=var, command=update_label, length=200)
        s.pack(side=tk.TOP, fill=tk.X)
        
        return var, label_var

    def send_gains(self):
        # Matches the Rust "parts.len() == 7" logic
        msg = f"GAIN,{self.q_pos_s.get():.6f},{self.q_vel_s.get():.6f},{self.q_ori_s.get():.6f},{self.r_fix_s.get():.6f},{self.r_float_s.get():.6f},{self.cutoff_hz_s.get():.6f},{self.error_percent:.2f}"
        # print(f"Sent: {msg}")
        try:
            self.sock.sendto(msg.encode(), RUST_SEND_ADDR)
        except OSError as e:
            print(f"Network error sending gains: {e}")

    def reset_odom(self):
        try:
            self.sock.sendto(b"RESET_ODOM", RUST_SEND_ADDR)
        except OSError as e:
            print(f"Network error resetting odom: {e}")
        self.total_dist_rust = 0.0
        print(f"RESET_ODOM")

    def setup_network(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(PYTHON_LISTEN_ADDR)
        self.sock.setblocking(False)

    def update_loop(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            msg = data.decode().split(',')
            if msg[0] == "POS":
                # print(f"Received: {msg}")
                # Expecting: POS, x, y, heading, total_dist
                rx, ry, head, tot_dist,rel_dist,gps_x,gps_y = map(float, msg[1:8])
                self.rover.goto(rx, ry)
                self.rover.setheading(head)
                self.rover.pendown()
                
                self.tot_dist_rust = tot_dist
                self.rel_dist_rust = rel_dist
                self.calculate_error()
                self.gps_marker.goto(gps_x, gps_y)
                self.gps_marker.dot(5) # Leave a small red dot
                                
        except BlockingIOError:
            pass
        
        self.screen.update()
        self.root.after(50, self.update_loop)

    def calculate_error(self):
        try:
            planned = float(self.truth_entry.get())
            if planned > 0:
                planned_dist = float(self.truth_entry.get())
                
                # # We assume the goal is (planned_dist, 0) for a straight line test
                # goal_x = planned_dist
                # goal_y = 0.0 
                
                # # Distance from where we ARE to where we SHOULD BE
                # error_meters = ((self.current_pos[0] - goal_x)**2 + (self.current_pos[1] - goal_y)**2)**0.5
                
                # # Percentage error relative to the total trip length
                # if planned_dist > 0:
                #     error_percent = (error_meters / planned_dist) * 100
                #     self.error_label.config(text=f"Dist Err: {error_meters:.2f}m\nRel Err: {error_percent:.1f}%")    

                # error = abs(self.tot_dist_rust - planned) / planned * 100
                # self.error_label.config(text=f"Rust: {self.tot_dist_rust:.2f}m\nError: {error:.1f}%")

                error = abs(self.rel_dist_rust - planned) / planned * 100
                self.error_label.config(text=f"Rust: {self.rel_dist_rust:.2f}m\nError: {error:.1f}%")

                self.error_percent = error
                self.send_error(error)
        except ValueError:
            pass

    def send_error(self, error: float):
        self.error_percent = error
        msg = f"GAIN,{self.q_pos_s.get():.6f},{self.q_vel_s.get():.6f},{self.q_ori_s.get():.6f},{self.r_fix_s.get():.6f},{self.r_float_s.get():.6f},{self.cutoff_hz_s.get():.6f},{self.error_percent:.2f}"
        try:
            self.sock.sendto(msg.encode(), RUST_SEND_ADDR)
        except OSError as e:
            print(f"Network error sending error: {e}")  # Optional: log the error

if __name__ == "__main__":
    root = tk.Tk()
    app = GCSApp(root)
    root.mainloop()