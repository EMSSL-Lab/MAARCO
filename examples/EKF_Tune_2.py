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
        self.tot_dist_rust = 0.0
        self.rel_dist_rust = 0.0
        self.error_percent = 0.0
        
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

        label = turtle.RawTurtle(self.screen, visible=False)
        label.pencolor("#ffffff")
        label.penup()
        for i in range(-int(dim_x), int(dim_x) + 1):
            label.goto(i, -0.2)
            label.write(str(i), align="center", font=("Arial", 8, "normal"))
            label.goto(-0.2, i)
            label.write(str(i), align="right", font=("Arial", 8, "normal"))

        self.rover = turtle.RawTurtle(self.screen, shape="triangle")
        self.rover.color("#2ecc71")
        self.rover.penup()

        self.gps_marker = turtle.RawTurtle(self.screen, visible=False)
        self.gps_marker.penup()
        self.gps_marker.color("red")

    def setup_controls(self):
        # --- EKF TUNING NUMERICAL INPUTS ---
        ttk.Label(self.ctrl_frame, text="EKF TUNING", font=('Arial', 12, 'bold')).pack(pady=5)
        
        self.params = {}
        # Format: (Label, Key, Default Value)
        tuning_fields = [
            ("Q Position", "q_pos", "0.010000"),
            ("Q Velocity", "q_vel", "0.100000"),
            ("Q Orientation", "q_ori", "0.001000"),
            ("R RTK Fix", "r_fix", "0.000100"),
            ("R RTK Float", "r_float", "0.250000"),
            ("Cutoff Hz", "cutoff_hz", "10.0")
        ]

        for label_text, key, default in tuning_fields:
            self.params[key] = self.create_input(label_text, default)
        
        ttk.Button(self.ctrl_frame, text="Send Gains", command=self.send_gains).pack(pady=15)
        
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

    def create_input(self, name, default):
        frame = ttk.Frame(self.ctrl_frame)
        frame.pack(fill=tk.X, pady=4)
        ttk.Label(frame, text=name).pack(side=tk.TOP, anchor=tk.W)
        
        var = tk.StringVar(value=default)
        entry = ttk.Entry(frame, textvariable=var)
        entry.pack(side=tk.TOP, fill=tk.X)
        return var

    def send_gains(self):
        try:
            msg = self.build_gain_string()
            self.sock.sendto(msg.encode(), RUST_SEND_ADDR)
            print(f"Sent Gains: {msg}")
        except ValueError:
            print("Error: Please enter valid numbers in all fields.")
        except OSError as e:
            print(f"Network error: {e}")

    def build_gain_string(self):
        # Extracts current values from numerical fields
        q_p = float(self.params["q_pos"].get())
        q_v = float(self.params["q_vel"].get())
        q_o = float(self.params["q_ori"].get())
        r_fx = float(self.params["r_fix"].get())
        r_fl = float(self.params["r_float"].get())
        hz = float(self.params["cutoff_hz"].get())
        return f"GAIN,{q_p:.6f},{q_v:.6f},{q_o:.6f},{r_fx:.6f},{r_fl:.6f},{hz:.2f},{self.error_percent:.2f}"

    def reset_odom(self):
        try:
            self.sock.sendto(b"RESET_ODOM", RUST_SEND_ADDR)
            self.tot_dist_rust = 0.0
            print("RESET_ODOM sent")
        except OSError as e:
            print(f"Network error: {e}")

    def setup_network(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(PYTHON_LISTEN_ADDR)
        self.sock.setblocking(False)

    def update_loop(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            msg = data.decode().split(',')
            if msg[0] == "POS" and len(msg) >= 8:
                rx, ry, head, tot_dist, rel_dist, gps_x, gps_y = map(float, msg[1:8])
                
                self.rover.goto(rx, ry)
                self.rover.setheading(head)
                self.rover.pendown()
                
                self.tot_dist_rust = tot_dist
                self.rel_dist_rust = rel_dist
                self.gps_marker.goto(gps_x, gps_y)
                self.gps_marker.dot(5)
                
                self.calculate_error()
                                
        except (BlockingIOError, ValueError):
            pass
        
        self.screen.update()
        self.root.after(50, self.update_loop)

    def calculate_error(self):
        try:
            planned = float(self.truth_entry.get())
            if planned > 0:
                error = abs(self.rel_dist_rust - planned) / planned * 100
                self.error_label.config(text=f"Rust: {self.rel_dist_rust:.2f}m\nError: {error:.1f}%")
                self.error_percent = error
                
                # Auto-sync error percent to Rust
                msg = self.build_gain_string()
                self.sock.sendto(msg.encode(), RUST_SEND_ADDR)
        except (ValueError, OSError):
            pass

if __name__ == "__main__":
    root = tk.Tk()
    app = GCSApp(root)
    root.mainloop()