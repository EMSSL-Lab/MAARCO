import turtle
import socket

# Setup UDP Socket
# RUST_ADDR = ("127.0.0.1", 5007)

RUST_ADDR = ("172.20.10.6", 5007)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

class WaypointGenerator:
    def __init__(self):
        # ---- SCREEN SETUP FIRST ----
        self.screen = turtle.Screen()
        # Use 1.0 to fill the phone screen entirely
        self.screen.setup(width=1.0, height=1.0)
        self.screen.bgcolor("#2c3e50")
        
        # ---- WORLD PARAMETERS ----
        self.world_x = 18.0   # meters
        # Now get the actual width after setup
        self.width = self.screen.window_width()
        self.scale = self.width / self.world_x 

        # ---- STATE ----
        self.origin_x = None
        self.origin_y = None

        # ---- DRAWING TOOLS ----
        self.drawer = turtle.Turtle()
        self.drawer.hideturtle()
        self.drawer.speed(0)
        self.drawer.pencolor("#ecf0f1")
        self.drawer.penup()
        kjlklj
        # Text tool for on-screen info
        self.writer = turtle.Turtle()
        self.writer.hideturtle()
        self.writer.penup()
        self.writer.pencolor("white")
        self.update_ui_text("TAP TO SET ORIGIN")

        # UI Bindings
        self.screen.onclick(self.handle_click)
        self.screen.listen()

    def update_ui_text(self, text):
        self.writer.clear()
        # Position text at the top of the screen
        self.writer.goto(0, (self.screen.window_height()/2) - 40)
        self.writer.write(text, align="center", font=("Arial", 16, "bold"))

    def handle_click(self, x, y):
        # Check if user clicked the "START ZONE" (Top 10% of screen)
        if y > (self.screen.window_height() / 2) - 60:
            if self.origin_x is not None:
                self.send_start_signal()
            return

        # Set origin on the very first click
        if self.origin_x is None:
            self.origin_x, self.origin_y = x, y
            self.drawer.goto(x, y)
            self.drawer.dot(20, "#2ecc71") # Larger dot for thumbs
            self.drawer.pendown()
            sock.sendto(b"SET_ORIGIN", RUST_ADDR)
            self.update_ui_text("ORIGIN SET. TAP WAYPOINTS, THEN TAP HERE TO START")
            return

        # Calculate meters relative to origin
        rel_x_m = (x - self.origin_x) / self.scale
        rel_y_m = (y - self.origin_y) / self.scale
        
        # Send to Rust
        message = f"WP,{rel_x_m:.2f},{rel_y_m:.2f}"
        sock.sendto(message.encode(), RUST_ADDR)
        
        # Draw path on UI
        self.drawer.goto(x, y)
        self.drawer.dot(15, "#e74c3c") # Larger dots for phone screen
        print(f"Sent: {rel_x_m:.2f}m E, {rel_y_m:.2f}m N")

    def send_start_signal(self):
        sock.sendto(b"START_MISSION", RUST_ADDR)
        self.update_ui_text("MISSION RUNNING!")
        print("🚀 START_MISSION sent")

if __name__ == "__main__":
    gen = WaypointGenerator()
    turtle.done()