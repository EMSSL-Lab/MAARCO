import turtle
import socket
import math
import time
import tkinter as tk
import csv
import os
from collections import deque
from datetime import datetime

# UPDATE THESE TO YOUR ACTUAL IPs
RUST_SEND_ADDR = ("172.20.10.4", 5007)
PYTHON_LISTEN_ADDR = ("0.0.0.0", 5008)


class MissionControl:
    def __init__(self):
        self.screen = turtle.Screen()
        self.screen.setup(width=1340, height=800)
        self.screen.bgcolor("#2c3e50")
        self.screen.title("Rover GCS - [1] Start | [2] Clear | [3] STOP | [4] Set Origin | [5] Undo | [▲] RPM+ | [▼] RPM- | Scroll=Zoom | Drag=Pan")

        # ── VIEW STATE ────────────────────────────────────────────────────────
        # view_size: half-width/height of the visible world in meters
        # pan_x/pan_y: world-space center of the current view (origin starts at 0,0)
        self.view_size = 5.0
        self.pan_x     = 0.0
        self.pan_y     = 0.0

        # ── TKINTER SETUP ─────────────────────────────────────────────────────
        self.tk_root   = self.screen._root
        self.tk_canvas = self.screen.getcanvas()
        self.HUD_W    = 270
        self.TELEM_W  = 260

        self.screen.tracer(0)
        self.tk_root.update_idletasks()
        win_w = self.tk_root.winfo_width()
        win_h = self.tk_root.winfo_height()

        self.tk_canvas.config(width=win_w - self.HUD_W - self.TELEM_W, height=win_h)
        self.tk_canvas.grid(row=0, column=0, sticky="nsew")

        # ── HUD PANEL ─────────────────────────────────────────────────────────
        self.hud_frame = tk.Frame(self.tk_root, bg="#1a252f",
                                  width=self.HUD_W, bd=2, relief="sunken")
        self.hud_frame.grid(row=0, column=1, sticky="nsew")
        self.hud_frame.grid_propagate(False)

        self.tk_root.columnconfigure(0, weight=1)
        self.tk_root.columnconfigure(1, weight=0)
        self.tk_root.columnconfigure(2, weight=0)
        self.tk_root.rowconfigure(0, weight=1)

        pad = dict(padx=8, pady=3, anchor="w")

        tk.Label(self.hud_frame, text="══ TELEMETRY DATA ══",
                 bg="#1a252f", fg="#ecf0f1",
                 font=("Arial", 11, "bold")).pack(pady=(10, 4))

        self.lbl_system = tk.Label(self.hud_frame, text="● SYSTEM OFFLINE",
                                   bg="#1a252f", fg="#e74c3c",
                                   font=("Arial", 11, "bold"))
        self.lbl_system.pack(**pad)

        tk.Frame(self.hud_frame, bg="#34495e", height=1).pack(fill="x", padx=6, pady=4)

        self.lbl_nav_state = tk.Label(self.hud_frame, text="STATUS: IDLE / STANDBY",
                                      bg="#1a252f", fg="#ac7715",
                                      font=("Arial", 10, "bold"),
                                      wraplength=self.HUD_W - 16, justify="left")
        self.lbl_nav_state.pack(**pad)

        tk.Frame(self.hud_frame, bg="#34495e", height=1).pack(fill="x", padx=6, pady=4)

        tk.Label(self.hud_frame, text="TELEMETRY",
                 bg="#1a252f", fg="#7f8c8d",
                 font=("Arial", 9, "bold")).pack(**pad)

        self.lbl_dist = tk.Label(self.hud_frame, text="DIST TO TARGET:  —",
                                  bg="#1a252f", fg="#ecf0f1",
                                  font=("Courier", 11, "bold"))
        self.lbl_dist.pack(**pad)

        self.lbl_brng = tk.Label(self.hud_frame, text="BEARING TO TARGET:  —",
                                  bg="#1a252f", fg="#ecf0f1",
                                  font=("Courier", 11, "bold"))
        self.lbl_brng.pack(**pad)

        self.lbl_rpm = tk.Label(self.hud_frame, text="RPM SETPOINT:   30.0",
                                 bg="#1a252f", fg="#ecf0f1",
                                 font=("Courier", 11, "bold"))
        self.lbl_rpm.pack(**pad)

        # GPS fix quality label
        self.lbl_gps = tk.Label(self.hud_frame, text="GPS FIX:  NO FIX",
                                 bg="#1a252f", fg="#e74c3c",
                                 font=("Courier", 11, "bold"))
        self.lbl_gps.pack(**pad)

        # Rover (x,y) coordinates 
        self.lbl_coords = tk.Label(self.hud_frame, text="Position: X: 0.00, Y: 0.00", font=("Courier", 10, "bold"), fg="#f4f8f6", bg="#1a252f")
        self.lbl_coords.pack(anchor="w",padx=15,pady=4)

        tk.Frame(self.hud_frame, bg="#34495e", height=1).pack(fill="x", padx=6, pady=4)

        tk.Label(self.hud_frame, text="STATUS MSG",
                 bg="#1a252f", fg="#7f8c8d",
                 font=("Arial", 9, "bold")).pack(**pad)

        self.lbl_status = tk.Label(self.hud_frame,
                                   text="READY: Click to set points",
                                   bg="#1a252f", fg="#f0e68c",
                                   font=("Arial", 9, "normal"),
                                   wraplength=self.HUD_W - 16, justify="left")
        self.lbl_status.pack(**pad)

        tk.Frame(self.hud_frame, bg="#34495e", height=1).pack(fill="x", padx=6, pady=4)

        tk.Frame(self.hud_frame, bg="#34495e", height=1).pack(fill="x", padx=6, pady=4)

        tk.Label(self.hud_frame, text="DATA LOGGING",
                 bg="#1a252f", fg="#7f8c8d",
                 font=("Arial", 9, "bold")).pack(padx=8, pady=(2, 2), anchor="w")

        self.lbl_logging = tk.Label(self.hud_frame, text="● LOGGING: OFF",
                                    bg="#1a252f", fg="#e74c3c",
                                    font=("Arial", 10, "bold"))
        self.lbl_logging.pack(padx=8, pady=(0, 4), anchor="w")

        btn_frame = tk.Frame(self.hud_frame, bg="#1a252f")
        btn_frame.pack(padx=8, pady=(0, 6), anchor="w")

        self.btn_start_log = tk.Button(
            btn_frame, text="▶ Start Log",
            bg="#1a6b2e", fg="white",
            font=("Arial", 9, "bold"),
            activebackground="#27ae60", activeforeground="white",
            relief="raised", bd=2, padx=6, pady=3,
            command=self.start_logging
        )
        self.btn_start_log.pack(side="left", padx=(0, 6))

        self.btn_stop_log = tk.Button(
            btn_frame, text="■ Stop Log",
            bg="#7f1010", fg="white",
            font=("Arial", 9, "bold"),
            activebackground="#c0392b", activeforeground="white",
            relief="raised", bd=2, padx=6, pady=3,
            command=self.stop_logging,
            state="disabled"
        )
        self.btn_stop_log.pack(side="left")

        ## Motor battery voltage label

        tk.Frame(self.hud_frame, bg="#34495e", height=1).pack(fill="x", padx=6, pady=4)

        tk.Frame(self.hud_frame, bg="#34495e", height=1).pack(fill="x", padx=6, pady=4)

        self.lbl_voltage = tk.Label(
            self.hud_frame, text="Motor Battery Voltage: --.-- V", font=("Courier",12,"bold"), 
            bg="#1a252f", fg="#34bd1c", anchor="w" 
        )
        self.lbl_voltage.pack(fill="x", padx=8, pady=2)

        # Create a queue that remembers the last 150 readings (10 seconds)
        self.voltage_history = deque(maxlen=150)


        tk.Label(self.hud_frame,
                 text="[1] Start  [2] Clear\n[3] STOP   [4] Origin\n[5] Undo   [6/7] RPM±\nScroll=Zoom  Drag=Pan",
                 bg="#1a252f", fg="#7f8c8d",
                 font=("Arial", 8, "normal"),
                 justify="left").pack(side="bottom", padx=8, pady=8, anchor="w")

        # ── LIVE TELEMETRY PANEL (right side) ────────────────────────────────
        BG   = "#0d1b2a"   # dark navy background
        FG   = "#cfd8dc"   # default text colour
        GRN  = "#2ecc71"
        YEL  = "#f1c40f"
        CYN  = "#1abc9c"
        RED  = "#e74c3c"
        DIM  = "#546e7a"
        VAL  = "#ecf0f1"

        self.telem_frame = tk.Frame(self.tk_root, bg=BG,
                                    width=self.TELEM_W, bd=2, relief="sunken")
        self.telem_frame.grid(row=0, column=2, sticky="nsew")
        self.telem_frame.grid_propagate(False)

        def _section(parent, title, color=CYN):
            tk.Label(parent, text=title, bg=BG, fg=color,
                     font=("Arial", 9, "bold")).pack(fill="x", padx=8, pady=(8, 1))
            tk.Frame(parent, bg=color, height=1).pack(fill="x", padx=6, pady=(0, 4))

        def _row(parent, label, init="—", val_color=VAL):
            f = tk.Frame(parent, bg=BG)
            f.pack(fill="x", padx=10, pady=1)
            tk.Label(f, text=label, bg=BG, fg=DIM,
                     font=("Courier", 9, "normal"), width=14, anchor="w").pack(side="left")
            v = tk.Label(f, text=init, bg=BG, fg=val_color,
                         font=("Courier", 9, "bold"), anchor="e")
            v.pack(side="right")
            return v

        tk.Label(self.telem_frame, text="══ LIVE SENSOR DATA ══",
                 bg=BG, fg=FG, font=("Arial", 11, "bold")).pack(pady=(10, 2))

        # ── IMU ───────────────────────────────────────────────────────────────
        _section(self.telem_frame, " ══ IMU  /  ATTITUDE ══", YEL)
        self.td_yaw   = _row(self.telem_frame, "Yaw", val_color=YEL)
        self.td_pitch = _row(self.telem_frame, "Pitch", val_color=YEL)
        self.td_roll  = _row(self.telem_frame, "Roll", val_color=YEL)
        self.td_gyro_x = _row(self.telem_frame, "Gyro X", val_color=YEL)
        self.td_gyro_y = _row(self.telem_frame, "Gyro Y", val_color=YEL)
        self.td_gyro_z = _row(self.telem_frame, "Gyro Z", val_color=YEL)
        tk.Frame(self.telem_frame, bg=BG, height=2).pack()
        self.td_ax    = _row(self.telem_frame, "Accel X", val_color=YEL)
        self.td_ay    = _row(self.telem_frame, "Accel Y", val_color=YEL)
        self.td_az    = _row(self.telem_frame, "Accel Z", val_color=YEL)

        # ── MOTORS ────────────────────────────────────────────────────────────
        _section(self.telem_frame, " ══ MOTOR DATA ══", GRN)

        # RPM sub-header
        tk.Label(self.telem_frame, text=" ---- RPM ----", bg=BG, fg=DIM,
                 font=("Courier", 8, "normal")).pack(anchor="w", padx=10)
        self.td_rpm_r  = _row(self.telem_frame, "  Right", val_color=GRN)
        self.td_rpm_l  = _row(self.telem_frame, "  Left",  val_color=GRN)

        # Voltage sub-header
        tk.Label(self.telem_frame, text=" ---- Voltage (V) ----", bg=BG, fg=DIM,
                 font=("Courier", 8, "normal")).pack(anchor="w", padx=10)
        self.td_volt_r = _row(self.telem_frame, "  Right", val_color=GRN)
        self.td_volt_l = _row(self.telem_frame, "  Left",  val_color=GRN)

        # Current sub-header
        tk.Label(self.telem_frame, text=" ---- Current (A) ----", bg=BG, fg=DIM,
                 font=("Courier", 8, "normal")).pack(anchor="w", padx=10)
        self.td_cur_r  = _row(self.telem_frame, "  Right", val_color=GRN)
        self.td_cur_l  = _row(self.telem_frame, "  Left",  val_color=GRN)

        # Motor current sub-header
        tk.Label(self.telem_frame, text=" ---- Motor Current (A) ----", bg=BG, fg=DIM,
                 font=("Courier", 8, "normal")).pack(anchor="w", padx=10)
        self.td_mcur_r = _row(self.telem_frame, "  Right", val_color=GRN)
        self.td_mcur_l = _row(self.telem_frame, "  Left",  val_color=GRN)

        # Rotations sub-header
        tk.Label(self.telem_frame, text=" ---- Total Revolutions ----", bg=BG, fg=DIM,
                 font=("Courier", 8, "normal")).pack(anchor="w", padx=10)
        self.td_rot_r  = _row(self.telem_frame, "  Right", val_color=GRN)
        self.td_rot_l  = _row(self.telem_frame, "  Left",  val_color=GRN)

        # ── SENSORS ───────────────────────────────────────────────────────────
        _section(self.telem_frame, " ══ Distance Sensors ══", RED)
        self.td_sonar = _row(self.telem_frame, "Sonar",  val_color=RED)
        self.td_tof   = _row(self.telem_frame, "ToF",    val_color=RED)

        # ── WORLD COORDINATES ─────────────────────────────────────────────────
        self._apply_world_coords()

        # ── TURTLE OBJECTS ────────────────────────────────────────────────────
        self.grid_tool = turtle.Turtle(visible=False)
        self.scale_pen = turtle.Turtle()
        self.scale_pen.hideturtle()
        self.scale_pen.penup()
        self.scale_pen.color("white")

        self.trail_turtle = turtle.Turtle()
        self.trail_turtle.hideturtle()
        self.trail_turtle.penup()
        self.trail_turtle.speed(0)
        self.breadcrumbs = []       # List of (x, y) tuples
        self.last_crumb_x = 0.0
        self.last_crumb_y = 0.0
        self.crumb_spacing = 0.25    # Drop a crumb every 0.5 meters

        self.origin_pen = turtle.Turtle(visible=False)
        self.origin_pen.speed(0)
        self.origin_pen.penup()

        self.draw_grid()
        self.draw_origin()
        self.draw_scale_bar()
        

        self.drawer = turtle.Turtle()
        self.drawer.pencolor("#e74c3c")
        self.drawer.penup()

        self.rover = turtle.Turtle(shape="triangle")
        self.rover.shapesize(1.2, 1.2)
        self.rover.color("#2ecc71")
        self.rover.penup()


        # ── STATE VARIABLES ───────────────────────────────────────────────────
        self.last_heartbeat      = 0
        self.last_drawn_state    = None
        self.target_rpm          = 0.0
        self.waypoint_queue      = []
        self.is_navigating       = False
        self.last_yaw_adjust_time = 0.0
        self.active_waypoint     = None
        self.current_rover_x     = 0.0
        self.current_rover_y     = 0.0
        self.distance            = 0.0
        self.target_angle        = 0.0
        self.last_telemetry_time = 0.0
        self.breadcrumbs = []       # List of (x, y) tuples
        self.last_crumb_x = 0.0
        self.last_crumb_y = 0.0
        self.crumb_spacing = 0.5    # Drop a crumb every 0.5 meters

        # Logging state
        self.is_logging      = False
        self.log_file        = None
        self.log_writer      = None

        # Drag/pan state
        self._drag_start_px = (0, 0)
        self._last_drag_px  = (0, 0)
        self._is_dragging   = False
        # Drag threshold in pixels — movement below this is treated as a click
        self._DRAG_THRESHOLD = 5
        self.last_pan_time = time.time() # time delay variable to prevent recursion crash due to rapid mouse panning

        # ── SOCKET ────────────────────────────────────────────────────────────
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(PYTHON_LISTEN_ADDR)
        self.sock.setblocking(False)

        # ── BINDINGS ──────────────────────────────────────────────────────────
        self.screen.listen()
        # Mouse: use raw tkinter bindings so we can separate click from drag.
        # screen.onclick is NOT used — we handle Button-1 ourselves.
        self.tk_canvas.bind("<ButtonPress-1>",   self._on_mouse_press)
        self.tk_canvas.bind("<B1-Motion>",        self._on_mouse_drag)
        self.tk_canvas.bind("<ButtonRelease-1>",  self._on_mouse_release)
        self.tk_canvas.bind("<MouseWheel>",       self.zoom)

        self.screen.onkey(self.start_mission,  "1")
        self.screen.onkey(self.clear_mission,  "2")
        self.screen.onkey(self.emergency_stop, "3")
        self.screen.onkey(self.set_origin,     "4")
        self.screen.onkey(self.undo_waypoint,  "5")
        self.screen.onkey(self.increase_rpm,   "Up")
        self.screen.onkey(self.decrease_rpm,   "Down")

        self.update_status("READY: Click to set points")
        self.update_rover()

    # ── COORDINATE HELPERS ────────────────────────────────────────────────────

    def _apply_world_coords(self):
        """Push the current pan+zoom state into turtle's world coordinate system."""
        self.screen.setworldcoordinates(
            self.pan_x - self.view_size,
            self.pan_y - self.view_size,
            self.pan_x + self.view_size,
            self.pan_y + self.view_size,
        )

    def _canvas_to_world(self, cx, cy):
        """Convert a tkinter canvas pixel (cx, cy) to world (meter) coordinates."""
        cw = self.tk_canvas.winfo_width()
        ch = self.tk_canvas.winfo_height()
        wx = (self.pan_x - self.view_size) + cx * (2.0 * self.view_size / cw)
        wy = (self.pan_y + self.view_size) - cy * (2.0 * self.view_size / ch)
        return wx, wy

    # ── PAN / CLICK MOUSE HANDLERS ────────────────────────────────────────────

    def _on_mouse_press(self, event):
        self._drag_start_px = (event.x, event.y)
        self._last_drag_px  = (event.x, event.y)
        self._is_dragging   = False

    def _on_mouse_drag(self, event):
        # Measure total movement from press start
        total_dx = event.x - self._drag_start_px[0]
        total_dy = event.y - self._drag_start_px[1]
        if math.sqrt(total_dx**2 + total_dy**2) > self._DRAG_THRESHOLD:
            self._is_dragging = True

        if self._is_dragging:
            # Delta from last reported position (incremental pan)
            dx_px = event.x - self._last_drag_px[0]
            dy_px = event.y - self._last_drag_px[1]
            self._last_drag_px = (event.x, event.y)

            cw = self.tk_canvas.winfo_width()
            ch = self.tk_canvas.winfo_height()
            # Dragging right (dx_px > 0) → view centre moves left (pan_x decreases)
            self.pan_x -= dx_px * (2.0 * self.view_size / cw)
            # Dragging down (dy_px > 0) → view centre moves up (pan_y increases)
            self.pan_y += dy_px * (2.0 * self.view_size / ch)

            # Throttle graphics rendering to prevent recursion crash due to rapid mouse panning\
            current_time = time.time()
            if current_time - self.last_pan_time > 0.03:
                self.last_pan_time = current_time
                self._apply_world_coords()
                self.draw_grid()
                self.draw_scale_bar()
                self.screen.update()
                self.last_pan_time = current_time # Reset the timer


    def _on_mouse_release(self, event):
        if not self._is_dragging:
            # Short press with no meaningful movement = waypoint click
            wx, wy = self._canvas_to_world(event.x, event.y)
            self.handle_click(wx, wy)
        self._is_dragging = False

    # ── HUD UPDATE HELPERS ────────────────────────────────────────────────────

    def draw_heartbeat(self, active):
        if active:
            self.lbl_system.config(text="● SYSTEM ACTIVE", fg="#2ecc71")
        else:
            self.lbl_system.config(text="● SYSTEM OFFLINE", fg="#e74c3c")

    def draw_nav_state(self):
        if self.last_drawn_state == self.is_navigating:
            return
        if self.is_navigating:
            self.lbl_nav_state.config(text="STATUS: ACTIVELY NAVIGATING", fg="#12f35d")
        else:
            self.lbl_nav_state.config(text="STATUS: IDLE / STANDBY", fg="#ac7715")
        self.last_drawn_state = self.is_navigating

    def draw_nav_info(self, dist, bearing):
        self.distance     = dist
        self.target_angle = bearing
        self.lbl_dist.config(text=f"DIST TO TARGET:  {dist:.2f} m")
        self.lbl_brng.config(text=f"BEARING TO TARGET:  {bearing:.1f}°")
        self.lbl_rpm.config( text=f"RPM SETPOINT:   {self.target_rpm:.1f}")

    def draw_gps_fix(self, fix_str):
        """Update the GPS fix quality label with colour coding."""
        COLOR_MAP = {
            "RTK_FIX":   ("#2ecc71", "RTK FIX (5/5)"),      # bright green
            "RTK_FLOAT": ("#3498db", "RTK FLOAT (4/5)"),     # blue
            "DGPS":      ("#f1c40f", "DGPS (3/5)"),          # yellow
            "GPS":       ("#f39c12", "GPS (AUTONOMOUS) (2/5)"),  # orange
            "NO_FIX":    ("#e74c3c", "NO FIX"),        # red
        }
        color, label = COLOR_MAP.get(fix_str, ("#e74c3c", fix_str))
        self.lbl_gps.config(text=f"GPS FIX:  {label}", fg=color)

    def update_status(self, text):
        self.lbl_status.config(text=text)
        self.screen.update()

    # ── SCALE BAR ─────────────────────────────────────────────────────────────

    def draw_scale_bar(self):
        """Draw a 1-meter scale bar anchored to the bottom-right of the viewport."""
        self.scale_pen.clear()
        # Always represent exactly 1 meter regardless of zoom
        length = 1.0
        # Calculate viewport-adaptive margins so the position doesn't drift when zooming
        margin_x = 0.2 * self.view_size  # Distance from right edge
        margin_y = 0.2 * self.view_size  # Distance from bottom edge

        # Position in the bottom-right corner of the current viewport
        x = (self.pan_x + self.view_size) - length - margin_x
        y = (self.pan_y - self.view_size) + margin_y
        
        self.scale_pen.goto(x, y)
        self.scale_pen.pendown()
        self.scale_pen.goto(x + length, y)
        self.scale_pen.penup()
        
        # Offset the text proportionally as well so it doesn't overlap the line
        text_offset = 0.04 * self.view_size
        self.scale_pen.goto(x + length / 2, y - text_offset)
        self.scale_pen.write("1 m", align="center", font=("Verdana", 8, "normal"))
        
        self.screen.update()

    # ── GRID ──────────────────────────────────────────────────────────────────

    def draw_grid(self):
        self.grid_tool.clear()

        # Viewport bounds in world (meter) space
        left   = self.pan_x - self.view_size
        right  = self.pan_x + self.view_size
        bottom = self.pan_y - self.view_size
        top    = self.pan_y + self.view_size

        # Integer meter grid lines visible in this viewport
        first_v = int(math.ceil(left))
        last_v  = int(math.floor(right))
        first_h = int(math.ceil(bottom))
        last_h  = int(math.floor(top))

        # ── Vertical lines ────────────────────────────────────────────────────
        for x in range(first_v, last_v + 1):
            if x == 0:
                self.grid_tool.pencolor("black")
                self.grid_tool.pensize(2)
            else:
                self.grid_tool.pencolor("#34495e")
                self.grid_tool.pensize(1)
            self.grid_tool.penup()
            self.grid_tool.goto(x, bottom)
            self.grid_tool.pendown()
            self.grid_tool.goto(x, top)

        # ── Horizontal lines ──────────────────────────────────────────────────
        for y in range(first_h, last_h + 1):
            if y == 0:
                self.grid_tool.pencolor("black")
                self.grid_tool.pensize(2)
            else:
                self.grid_tool.pencolor("#34495e")
                self.grid_tool.pensize(1)
            self.grid_tool.penup()
            self.grid_tool.goto(left, y)
            self.grid_tool.pendown()
            self.grid_tool.goto(right, y)

        # ── Coordinate labels ─────────────────────────────────────────────────
        self.grid_tool.pencolor("#bdc3c7")
        self.grid_tool.pensize(1)
        self.grid_tool.penup()

        for x in range(first_v, last_v + 1):
            if x == 0:
                continue  # draw "0" once below
            self.grid_tool.goto(x, bottom + 0.2)
            self.grid_tool.write(f"{x}", align="center", font=("Verdana", 8, "normal"))

        for y in range(first_h, last_h + 1):
            if y == 0:
                continue
            self.grid_tool.goto(left + 0.2, y - 0.2)
            self.grid_tool.write(f"{y}", align="left", font=("Verdana", 8, "normal"))

        # "0" label only when the origin is in view
        if first_v <= 0 <= last_v and first_h <= 0 <= last_h:
            self.grid_tool.goto(0.15, 0.15)
            self.grid_tool.write("0", align="left", font=("Verdana", 8, "bold"))

        # ── Cardinal labels (centred on current viewport edges) ───────────────
        self.grid_tool.pencolor("#11100F")
        # Calculate viewport-adaptive margins so text doesn't scale weirdly when zooming
        margin_y = 0.08 * self.view_size
        margin_x = 0.04 * self.view_size

        # NORTH (Locked to Top Center of screen)
        self.grid_tool.penup()
        self.grid_tool.goto(self.pan_x, self.pan_y + self.view_size - margin_y)
        self.grid_tool.write("NORTH", align="center", font=("Verdana", 10, "bold"))

        # SOUTH (Locked to Bottom Center of screen)
        self.grid_tool.penup()
        self.grid_tool.goto(self.pan_x, self.pan_y - self.view_size + (margin_y / 2))
        self.grid_tool.write("SOUTH", align="center", font=("Verdana", 10, "bold"))

        # EAST (Locked to Right Center of screen)
        self.grid_tool.penup()
        self.grid_tool.goto(self.pan_x + self.view_size - margin_x, self.pan_y)
        self.grid_tool.write("EAST", align="right", font=("Verdana", 10, "bold"))

        # WEST (Locked to Left Center of screen)
        self.grid_tool.penup()
        self.grid_tool.goto(self.pan_x - self.view_size + margin_x, self.pan_y)
        self.grid_tool.write("WEST", align="left", font=("Verdana", 10, "bold"))

        # Breadcrumbs (dotted trail of past rover positions)
        self.trail_turtle.clear()

        for (crumb_x, crumb_y) in self.breadcrumbs:
            self.trail_turtle.goto(crumb_x, crumb_y)
            self.trail_turtle.dot(4, "#4be15f")  # bright yellow dot
        self.screen.update()

    # ── ORIGIN MARKER ─────────────────────────────────────────────────────────

    def draw_origin(self):
        """Draw the fixed origin marker at world (0, 0) — redrawn on demand."""
        self.origin_pen.clear()
        self.origin_pen.goto(0, 0)
        self.origin_pen.dot(12, "grey")
        self.origin_pen.goto(0, -0.3)
        self.origin_pen.color("grey")
        self.origin_pen.write("Origin (Start)", align="center", font=("Arial", 10, "bold"))

    # ── WAYPOINT PATH ─────────────────────────────────────────────────────────

    def draw_path(self):
        self.drawer.clear()
        self.drawer.penup()
        curr_x, curr_y = self.current_rover_x, self.current_rover_y

        if self.active_waypoint:
            self.drawer.goto(curr_x, curr_y)
            self.drawer.color("#2ecc71" if self.is_navigating else "#f39c12")
            self.drawer.pensize(3)
            self.drawer.pendown()
            self.drawer.goto(self.active_waypoint[0], self.active_waypoint[1])
            self.drawer.penup()
            self.drawer.dot(10)
            start_x, start_y = self.active_waypoint
        else:
            start_x, start_y = curr_x, curr_y

        self.drawer.color("#e74c3c")
        self.drawer.pensize(1)
        for wp in self.waypoint_queue:
            self.drawer.goto(start_x, start_y)
            self.drawer.pendown()
            self.drawer.goto(wp[0], wp[1])
            self.drawer.penup()
            self.drawer.dot(8, "#e74c3c")
            start_x, start_y = wp

    # ── ZOOM ──────────────────────────────────────────────────────────────────

    def zoom(self, event):
        factor = 1.1 if event.delta > 0 else 0.9
        self.view_size *= factor
        # Increased max to 50 to allow zooming out far after panning
        self.view_size = max(0.5, min(50.0, self.view_size))
        self._apply_world_coords()
        self.draw_grid()
        self.draw_scale_bar()
        self.screen.update()

    # ── MISSION CONTROLS ──────────────────────────────────────────────────────

    def handle_click(self, x, y):
        if math.sqrt(x**2 + y**2) < 0.5:
            print("Point too close to origin! Click further away.")
            return
        self.waypoint_queue.append((x, y))
        self.draw_path()
        print(f"Queued WP: ({x:.2f}, {y:.2f}) | Queue size: {len(self.waypoint_queue)}")
        self.screen.update()
        if not self.is_navigating and len(self.waypoint_queue) == 1:
            dx = x - self.current_rover_x
            dy = y - self.current_rover_y
            dist = math.sqrt(dx**2 + dy**2)
            target_angle = math.degrees(math.atan2(dx, dy))
         
            self.draw_nav_info(dist, target_angle)
    def start_mission(self):
        if self.is_navigating:
            print("Already navigating!")
            return
        if self.active_waypoint:
            print("Resuming mission...")
            self.is_navigating = True
            self.send_nav_command(self.active_waypoint[0], self.active_waypoint[1])
        elif self.waypoint_queue:
            print("Starting mission from queue...")
            self.is_navigating = True
            self.active_waypoint = self.waypoint_queue.pop(0)
            self.send_nav_command(self.active_waypoint[0], self.active_waypoint[1])
        self.draw_path()

    def emergency_stop(self):
        self.sock.sendto(b"STOP", RUST_SEND_ADDR)
        self.is_navigating = False
        self.draw_path()
        self.update_status("STOP COMMAND SENT")
        print("Sent: STOP Command to Rover!")

    def clear_mission(self):
        self.drawer.clear()
        self.trail_turtle.clear()
        self.drawer.penup()
        self.waypoint_queue.clear()
        self.update_status("CLEARED: Ready for new points")
        self.distance        = 0.0
        self.target_angle    = 0.0
        self.active_waypoint = None
        self.is_navigating   = False
        self.breadcrumbs = []
        self.lbl_dist.config(text="DIST TO TARGET:  —")
        self.lbl_brng.config(text="BEARING TO TARGET:  —")
        self.lbl_rpm.config( text=f"RPM SETPOINT:   {self.target_rpm:.1f}")
        self.draw_path()
        self.screen.update()
        print("Mission cleared! Navigation reset.")

    def set_origin(self):
        self.sock.sendto(b"SET_ORIGIN", RUST_SEND_ADDR)
        self.update_status("ORIGIN SET: Rover at (0,0)")
        self.breadcrumbs.clear()
        self.last_crumb_x = 0.0
        self.last_crumb_y = 0.0
        print("Sent: SET_ORIGIN")

    def undo_waypoint(self):
        if self.waypoint_queue:
            self.waypoint_queue.pop()
            self.draw_path()
            if len(self.waypoint_queue) == 0 and not self.is_navigating:
                self.distance     = 0.0
                self.target_angle = 0.0
                self.lbl_dist.config(text="DIST TO TARGET:  —")
                self.lbl_brng.config(text="BEARING TO TARGET:  —")
            self.screen.update()
            print(f"Undid last waypoint. Queue size: {len(self.waypoint_queue)}")
        else:
            print("No waypoints to undo.")

    def increase_rpm(self):
        self.target_rpm += 5.0
        self.send_rpm_to_rust()

    def decrease_rpm(self):
        self.target_rpm = max(0, self.target_rpm - 5.0)
        self.send_rpm_to_rust()

    def send_rpm_to_rust(self):
        rpm_msg = f"RPM,{self.target_rpm:.1f}"
        self.sock.sendto(rpm_msg.encode(), RUST_SEND_ADDR)
        self.draw_nav_info(self.distance, self.target_angle)
        print(f"Sent RPM Update: {self.target_rpm}")

    def send_nav_command(self, tx, ty):
        dx = tx - self.current_rover_x
        dy = ty - self.current_rover_y
        distance     = math.sqrt(dx**2 + dy**2)
        target_angle = math.degrees(math.atan2(dx, dy)) ## Convert yaw angle from math heading to compass heading 
        nav_msg = f"NAV,{distance:.3f},{target_angle:.3f}"
        self.sock.sendto(nav_msg.encode(), RUST_SEND_ADDR)
        self.draw_nav_info(distance, target_angle)
        self.update_status(f"NAVIGATING to ({tx:.1f}, {ty:.1f})")
        print(f"Sent NAV: {distance:.2f}m at {target_angle:.1f}°")

    def send_next_waypoint(self):
        if len(self.waypoint_queue) == 0:
            self.active_waypoint = None
            self.is_navigating   = False
            self.draw_path()
            self.update_status("IDLE: All waypoints reached")
            return
        tx, ty = self.waypoint_queue.pop(0)
        self.active_waypoint = (tx, ty)
        self.is_navigating   = True
        self.send_nav_command(tx, ty)
        self.draw_path()

    # ── LIVE TELEMETRY PANEL UPDATE ───────────────────────────────────────────

    def update_live_panel(self, msg):
        """Parse the full TELEM message and refresh every label in the live panel.

        Expected TELEM format (0-based msg index):

        ============================= Arduino Data =============================

          msg[0]  = "TELEM"
          msg[1]  = x           msg[2]  = y
          msg[3]  = fix label         msg[4]  = accel_x
          msg[5]  = accel_y     msg[6]  = accel_z     msg[7]  = yaw
          msg[8]  = pitch       msg[9]  = roll
          msg[10] = voltage_l   msg[11] = voltage_r
          msg[12] = current_l   msg[13] = current_r
          msg[14] = motor_current_l  msg[15] = motor_current_r
          msg[16] = rpm_l       msg[17] = rpm_r
          msg[18] = sonar_mm    msg[19] = tof_mm
          msg[20] = rotations_l msg[21] = rotations_r
          msg [22] = gyro_x     msg[23] = gyro_y     msg[24] = gyro_z

        =============================== GPS DATA ===============================

            msg [25] = timestamp_ns, msg [26] = fix_time,
            msg [27] = fix_date, msg [28] = gga_fix_quality
            msg [29] = avg_snr, msg [30] = latitude, msg [31] = longitude,
            msg [32] = altitude, msg [33] = speed_over_ground,
            msg [34] = true_course, msg [35] = num_of_satellites
            msg [36] = hdop, msg [37] = vdop, msg [38] = pdop,
            msg [39] = geoid_separation
        """

        def _safe(index, fmt=".2f", suffix=""):
            try:
                return f"{float(msg[index]):{fmt}}{suffix}"
            except (IndexError, ValueError):
                return "—"

        # IMU / Attitude
        self.td_yaw.config(  text=_safe(7,  ".1f", "°"))
        self.td_pitch.config( text=_safe(8,  ".1f", "°"))
        self.td_roll.config(  text=_safe(9,  ".1f", "°"))
        self.td_gyro_x.config( text=_safe(22,  ".3f", " °/s"))
        self.td_gyro_y.config( text=_safe(23,  ".3f", " °/s"))
        self.td_gyro_z.config( text=_safe(24,  ".3f"," °/s"))
        self.td_ax.config(    text=_safe(4,  ".3f", " g"))
        self.td_ay.config(    text=_safe(5,  ".3f", " g"))
        self.td_az.config(    text=_safe(6,  ".3f", " g"))

        # Drivetrain
        self.td_rpm_r.config(  text=_safe(17, ".1f", " rpm"))
        self.td_rpm_l.config(  text=_safe(16, ".1f", " rpm"))
        self.td_volt_r.config( text=_safe(11, ".2f", " V"))
        self.td_volt_l.config( text=_safe(10, ".2f", " V"))
        self.td_cur_r.config(  text=_safe(13, ".3f", " A"))
        self.td_cur_l.config(  text=_safe(12, ".3f", " A"))
        self.td_mcur_r.config( text=_safe(15, ".3f", " A"))
        self.td_mcur_l.config( text=_safe(14, ".3f", " A"))
        self.td_rot_r.config(  text=_safe(21, ".1f"))
        self.td_rot_l.config(  text=_safe(20, ".1f"))

        # Sensors
        self.td_sonar.config( text=_safe(18, ".0f", " mm"))
        self.td_tof.config(   text=_safe(19, ".0f", " mm"))

    # ── CSV LOGGING ───────────────────────────────────────────────────────────

    def start_logging(self):
        if self.is_logging:
            return
        desktop = os.path.join(os.path.expanduser("~"), "Desktop")
        os.makedirs(desktop, exist_ok=True)
        timestamp_str = datetime.now().strftime("%m_%d_%Y")
        log_path = os.path.join(desktop, f"gui_log_{timestamp_str}.csv")
        try:
            self.log_file = open(log_path, "w", newline="")
            self.log_writer = csv.writer(self.log_file)
            # Write the header row for the csv file
            self.log_writer.writerow([ "x_meters", "y_meters","fix_label","accel_x","accel_y","accel_z","yaw","pitch","roll","voltage_l","voltage_r","current_l","current_r","motor_current_l","motor_current_r","rpm_l","rpm_r","sonar_mm","tof_mm","rotations_l","rotations_r","gyro_x","gyro_y","gyro_z",
            "timestamp_ns","fix_time","fix_date","gga_fix_quality","avg_snr","latitude","longitude","altitude","speed_over_ground","true_course","num_of_satellites","hdop","vdop","pdop","geoid_separation"                          ])  # Header row
            self.log_file.flush()
            self.is_logging = True
            self.lbl_logging.config(text="● LOGGING: ON", fg="#2ecc71")
            self.btn_start_log.config(state="disabled")
            self.btn_stop_log.config(state="normal")
            self.update_status(f"LOGGING to {os.path.basename(log_path)}")
            print(f"[LOG] Started logging to: {log_path}")
        except Exception as e:
            self.update_status(f"LOG ERROR: {e}")
            print(f"[LOG] Failed to open log file: {e}")

    def stop_logging(self):
        if not self.is_logging:
            return
        self.is_logging = False
        try:
            if self.log_file:
                self.log_file.close()
                self.log_file = None
                self.log_writer = None
        except Exception as e:
            print(f"[LOG] Error closing log file: {e}")
        self.lbl_logging.config(text="● LOGGING: OFF", fg="#e74c3c")
        self.btn_start_log.config(state="normal")
        self.btn_stop_log.config(state="disabled")
        self.update_status("LOGGING STOPPED")
        print("[LOG] Logging stopped.")


    # ── DATA Recieved from RUST ───────────────────────────────────────────────────

    def update_rover(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            self.draw_heartbeat(True)
            msg = data.decode().split(',')

            if msg[0] == "TELEM" and len(msg) >= 4:
                self.current_rover_x = float(msg[1])
                self.current_rover_y = float(msg[2])
                rover_yaw            = float(msg[7])
                self.lbl_coords.config(text=f"Position: X: {self.current_rover_x:.2f}, Y: {self.current_rover_y:.2f}")
                turtle_angle         = (90 - rover_yaw) % 360

            # NEW, bread crumb trail logic
                dist_from_last_crumb = math.hypot(self.current_rover_x - self.last_crumb_x, self.current_rover_y - self.last_crumb_y)
                if dist_from_last_crumb >= self.crumb_spacing:
                    self.breadcrumbs.append((self.current_rover_x, self.current_rover_y))
                    self.last_crumb_x = self.current_rover_x
                    self.last_crumb_y = self.current_rover_y

            # --- NEW BATTERY WARNING LOGIC ---
                if len(msg) > 10:
                    try:
                        voltage_l = float(msg[10]) # Index 10 is voltage_left
                        voltage_r = float(msg[11]) # Index 11 is voltage_right
                       # raw_average_voltage = (voltage_l + voltage_r) / 2.0
                        m = 1 # calibration constant: SCALE (slope)
                        b = 0.5 # [V] Calibration constant: OFFSET (y-intercept)
                        # Right voltage reading seems more accurate than left 
                        # 1. Mathematical model (linear equation) to adjust voltage to represent true battery voltage, taking in factors such as upstream resistance and sensor inaccuracies
                        true_voltage = (voltage_r*m) + b 
                        # 2. Add new voltage reading to history list
                        self.voltage_history.append(true_voltage)

                        # 3. Calculate the average of the last X number of readings
                        avg_voltage = sum(self.voltage_history) / len(self.voltage_history)

                        # Set your low-voltage threshold here! 
                        # This warning threshold should be lower than the actual cutoff threshold due to resistance in the wires
                        warning_threshold = 10.2 

                        # Update voltage label based on average voltage 
                        
                        if avg_voltage < warning_threshold:
                            # Red text + "LOW" warning
                            self.lbl_voltage.config(
                                text=f"BATTERY: {avg_voltage:.2f} V [LOW VOLTAGE!]", 
                                fg="#932013"
                            )
                        elif 10.2 <= avg_voltage <= 10.8:
                            self.lbl_voltage.config(
                                text=f"BATTERY: {avg_voltage:.2f} V ", fg="#f1c40f")
                        elif avg_voltage > 10.8:
                            # Normal green text
                            self.lbl_voltage.config(
                                text=f"BATTERY: {avg_voltage:.2f} V", 
                                fg="#2ecc71"
                            )
                    except ValueError:
                        pass # Ignore temporary parsing glitches

                # Update live data panel with all telemetry fields
                self.update_live_panel(msg)

                # ---- NEW LOGGING LOGIC GOES HERE ----
                if self.is_logging and self.log_file:
                    # Get current time
                    iso_timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                    
                    # msg[1:] grabs ALL 21 variables from Rust. 
                    # join() stitches them together with commas automatically.
                    data_str = ",".join(msg[1:])
                    
                    # Write the timestamp and the 21 variables to the file
                    self.log_file.write(f"{iso_timestamp},{data_str}\n")
                    self.log_file.flush()

                self.rover.goto(self.current_rover_x, self.current_rover_y)
                self.rover.setheading(turtle_angle)
                self.draw_path()

                # Get GPS fix quality msg[3] 
                if len(msg) >= 5:
                    self.draw_gps_fix(msg[3].strip())

                if self.active_waypoint:
                    dx = self.active_waypoint[0] - self.current_rover_x
                    dy = self.active_waypoint[1] - self.current_rover_y
                    remaining_dist = math.sqrt(dx**2 + dy**2)
                    # Recalculate true bearing to target
                    target_angle = math.degrees(math.atan2(dx,dy))
                    self.draw_nav_info(remaining_dist, target_angle)
                    current_time = time.time()
                    if self.is_navigating and (current_time - self.last_yaw_adjust_time > 1.0):
                         adjust_cmd = f"ADJUST_DISTANCE_TRACKER,{target_angle:.2f},{remaining_dist:.2f}"
                         self.sock.sendto(adjust_cmd.encode(), RUST_SEND_ADDR)
                         self.last_yaw_adjust_time = current_time

                else:
                    self.lbl_dist.config(text="DIST TO TARGET:  —")
                    self.lbl_brng.config(text="BEARING TO TARGET:  —")

            elif msg[0] == "ARRIVED":
                print("\n[SUCCESS] Rover reached waypoint!")
                self.is_navigating = False
                self.send_next_waypoint()

            self.last_telemetry_time = time.time()

        except BlockingIOError:
            pass
        except Exception as e:
            print(f"Socket Error: {e}")
            
        try: 
            # send a heatbeat 20 times a second to tell Rust connection is valid
            self.sock.sendto(b"HEARTBEAT", RUST_SEND_ADDR)
        except Exception as e:
            pass # Ignore temporary network sending glitches 
        

        if time.time() - self.last_telemetry_time > 2.0:
            self.draw_heartbeat(False)

        self.draw_nav_state()
        self.screen.update()
        self.screen.ontimer(self.update_rover, 50)


if __name__ == "__main__":
    gui = MissionControl()
    turtle.done()