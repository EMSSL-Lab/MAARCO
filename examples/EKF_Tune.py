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
        self.screen.setworldcoordinates(-10, -10, 10, 10)
        
        # Grid
        grid = turtle.RawTurtle(self.screen, visible=False)
        grid.pencolor("#34495e")
        for i in range(-10, 11):
            grid.penup(); grid.goto(i, -10); grid.pendown(); grid.goto(i, 10)
            grid.penup(); grid.goto(-10, i); grid.pendown(); grid.goto(10, i)

        self.rover = turtle.RawTurtle(self.screen, shape="triangle")
        self.rover.color("#2ecc71")
        self.rover.penup()

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
        self.truth_entry.insert(0, "5.0")
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
        msg = f"GAIN,{self.q_pos_s.get():.6f},{self.q_vel_s.get():.6f},{self.q_ori_s.get():.6f},{self.r_fix_s.get():.6f},{self.r_float_s.get():.6f},{self.cutoff_hz_s.get():.6f}"
        print(f"Sent: {msg}")
        self.sock.sendto(msg.encode(), RUST_SEND_ADDR)

    def reset_odom(self):
        self.sock.sendto(b"RESET_ODOM", RUST_SEND_ADDR)
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
                # Expecting: POS, x, y, heading, total_dist
                rx, ry, head, dist = map(float, msg[1:5])
                self.rover.goto(rx, ry)
                self.rover.setheading(head)
                self.rover.pendown()
                
                self.total_dist_rust = dist
                self.calculate_error()
        except BlockingIOError:
            pass
        
        self.screen.update()
        self.root.after(50, self.update_loop)

    def calculate_error(self):
        try:
            planned = float(self.truth_entry.get())
            if planned > 0:
                error = abs(self.total_dist_rust - planned) / planned * 100
                self.error_label.config(text=f"Rust: {self.total_dist_rust:.2f}m\nError: {error:.1f}%")
        except ValueError:
            pass

if __name__ == "__main__":
    root = tk.Tk()
    app = GCSApp(root)
    root.mainloop()