#!/usr/bin/env python3
"""
plotter_control_combined.py — Runs on your LAPTOP
Full GUI that sends commands to the Pi over MQTT (WiFi).

Install dependencies on laptop:
    pip install paho-mqtt pillow opencv-python

Run:
    python3 plotter_control_combined.py

Make sure pi_agent.py is running on the Pi first.
"""

import math
import time
import json
import queue
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from datetime import datetime
from typing import Optional

import cv2
import paho.mqtt.client as mqtt
import db

from PIL import Image, ImageTk

# ============================================================================
# MQTT settings — change PI_IP to your Raspberry Pi's IP address
# ============================================================================

# PI_IP        = "10.53.180.29"   # ← CHANGE THIS to your Pi's IP (run `hostname -I` on Pi)
PI_IP = "104.39.14.215"
BROKER_PORT  = 1883

TOPIC_COMMANDS = "plotter/commands"
TOPIC_RESPONSE = "plotter/response"
TOPIC_STATUS   = "plotter/status"


# ============================================================================
# LoginWindow
# ============================================================================

class LoginWindow:
    BG       = "#0f1117"
    PANEL    = "#1a1d27"
    ACCENT   = "#4f8ef7"
    TEXT     = "#e8eaf0"
    MUTED    = "#6b7280"
    ERROR    = "#ef4444"
    BORDER   = "#2d3148"
    ENTRY_BG = "#252839"

    def __init__(self):
        self.user = None
        self.root = tk.Tk()
        self.root.title("XY Plotter — Sign In")
        self.root.resizable(False, False)
        self.root.configure(bg=self.BG)
        w, h = 420, 480
        sw = self.root.winfo_screenwidth()
        sh = self.root.winfo_screenheight()
        self.root.geometry(f"{w}x{h}+{(sw-w)//2}+{(sh-h)//2}")
        self._build()

    def _build(self):
        outer = tk.Frame(self.root, bg=self.BG)
        outer.pack(expand=True, fill=tk.BOTH, padx=40, pady=30)

        logo_frame = tk.Frame(outer, bg=self.BG)
        logo_frame.pack(fill=tk.X, pady=(0, 24))
        c = tk.Canvas(logo_frame, width=48, height=48, bg=self.BG, highlightthickness=0)
        c.pack()
        c.create_rectangle(4, 4, 44, 44, fill="#7c3aed", outline="", width=0)
        c.create_line(14, 24, 34, 24, fill="white", width=2)
        c.create_line(24, 14, 24, 34, fill="white", width=2)
        c.create_oval(20, 20, 28, 28, fill="white", outline="")

        tk.Label(logo_frame, text="XY Plotter Control",
                 bg=self.BG, fg=self.TEXT,
                 font=("Courier New", 16, "bold")).pack(pady=(10, 0))
        tk.Label(logo_frame, text="Sign in to continue",
                 bg=self.BG, fg=self.MUTED,
                 font=("Courier New", 10)).pack()

        card = tk.Frame(outer, bg=self.PANEL,
                        highlightbackground=self.BORDER, highlightthickness=1)
        card.pack(fill=tk.BOTH, expand=True)
        inner = tk.Frame(card, bg=self.PANEL)
        inner.pack(fill=tk.BOTH, expand=True, padx=28, pady=28)

        tk.Label(inner, text="USERNAME", bg=self.PANEL, fg=self.MUTED,
                 font=("Courier New", 9, "bold"), anchor="w").pack(fill=tk.X)
        self._username = tk.StringVar()
        self._make_entry(inner, self._username, show=None).pack(fill=tk.X, pady=(4, 14))

        tk.Label(inner, text="PASSWORD", bg=self.PANEL, fg=self.MUTED,
                 font=("Courier New", 9, "bold"), anchor="w").pack(fill=tk.X)
        self._password = tk.StringVar()
        self._make_entry(inner, self._password, show="●").pack(fill=tk.X, pady=(4, 6))

        self._err_label = tk.Label(inner, text="", bg=self.PANEL,
                                   fg=self.ERROR, font=("Courier New", 9))
        self._err_label.pack(fill=tk.X, pady=(0, 14))

        self._login_btn = tk.Button(
            inner, text="SIGN IN",
            bg=self.ACCENT, fg="white",
            activebackground="#3b74e0", activeforeground="white",
            font=("Courier New", 11, "bold"),
            relief=tk.FLAT, cursor="hand2", padx=0, pady=10,
            command=self._attempt_login)
        self._login_btn.pack(fill=tk.X)

        hint = tk.Frame(inner, bg=self.PANEL)
        hint.pack(fill=tk.X, pady=(20, 0))
        tk.Label(hint, text="Demo accounts:", bg=self.PANEL, fg=self.MUTED,
                 font=("Courier New", 8)).pack(anchor="w")
        tk.Label(hint, text="  admin / admin123", bg=self.PANEL, fg=self.MUTED,
                 font=("Courier New", 8)).pack(anchor="w")
        tk.Label(hint, text="  operator / operator456", bg=self.PANEL, fg=self.MUTED,
                 font=("Courier New", 8)).pack(anchor="w")

        self.root.bind("<Return>", lambda _e: self._attempt_login())

    def _make_entry(self, parent, textvariable, show):
        frame = tk.Frame(parent, bg=self.ENTRY_BG,
                         highlightbackground=self.BORDER, highlightthickness=1)
        e = tk.Entry(frame, textvariable=textvariable, show=show or "",
                     bg=self.ENTRY_BG, fg=self.TEXT, insertbackground=self.TEXT,
                     font=("Courier New", 12), relief=tk.FLAT, bd=0)
        e.pack(fill=tk.X, padx=10, pady=8)
        e.bind("<FocusIn>",  lambda _: frame.config(highlightbackground=self.ACCENT))
        e.bind("<FocusOut>", lambda _: frame.config(highlightbackground=self.BORDER))
        return frame

    def _attempt_login(self):
        username = self._username.get().strip()
        password = self._password.get()
        if not username or not password:
            self._err_label.config(text="⚠  Please enter username and password.")
            return
        self._login_btn.config(text="Checking…", state=tk.DISABLED)
        self.root.update_idletasks()
        user = db.authenticate(username, password)
        if user:
            self.user = dict(user)
            self.root.destroy()
        else:
            self._login_btn.config(text="SIGN IN", state=tk.NORMAL)
            self._err_label.config(text="⚠  Invalid username or password.")
            self._password.set("")

    def run(self):
        self.root.mainloop()
        return self.user


# ============================================================================
# PlotterControlGUI
# ============================================================================

class PlotterControlGUI:
    """Main GUI — talks to Pi over MQTT instead of driving hardware directly."""

    CAM_W = 280
    CAM_H = 210
    CAM_FPS_TARGET = 15

    def __init__(self, current_user: dict):
        self.root = tk.Tk()
        self.root.title(f"XY Plotter Control  ·  {current_user['display_name']}")
        self.current_user = current_user

        # Plotter state (mirrored from Pi responses)
        self.plotter_initialized = False
        self.is_homed            = False
        self.x_max_mm            = None
        self.y_max_mm            = None
        self.current_x_mm        = 0.0
        self.current_y_mm        = 0.0

        # Image / points
        self.image          = None
        self.image_tk       = None
        self.image_path     = None
        self.real_width_mm  = None
        self.real_height_mm = None
        self.scale_factor   = 1.0
        self.points         = []
        self.point_markers  = []
        self.current_mode   = tk.StringVar(value="sequence")
        self.is_executing   = False

        # Settings
        self.press_distance = tk.DoubleVar(value=5.0)
        self.offset_x       = tk.DoubleVar(value=0.0)
        self.offset_y       = tk.DoubleVar(value=0.0)

        # Camera state
        self._cam_url         = tk.StringVar(value="")
        self._cam_running     = False
        self._cam_thread      = None
        self._cam_frame_queue = queue.Queue(maxsize=2)
        self._cam_photo       = None
        self._cam_last_pil    = None
        self._cam_after_id    = None
        self._cam_status      = tk.StringVar(value="Disconnected")
        self._cam_fps_counter = 0
        self._cam_fps_display = tk.StringVar(value="— fps")
        self._cam_fps_ts      = time.time()

        # MQTT
        self._mqtt_response_queue = queue.Queue()
        self._mqtt_connected      = False
        self._setup_mqtt()

        self._create_ui()

        # Poll MQTT response queue from the main thread
        self.root.after(100, self._mqtt_poll_responses)

    # =========================================================================
    # MQTT setup
    # =========================================================================

    def _setup_mqtt(self):
        self.mqtt = mqtt.Client(client_id=f"plotter_gui_{int(time.time())}")
        self.mqtt.on_connect    = self._mqtt_on_connect
        self.mqtt.on_disconnect = self._mqtt_on_disconnect
        self.mqtt.on_message    = self._mqtt_on_message

        try:
            self.mqtt.connect(PI_IP, BROKER_PORT, keepalive=60)
            self.mqtt.loop_start()   # background thread
            print(f"[mqtt] Connecting to {PI_IP}:{BROKER_PORT} …")
        except Exception as e:
            print(f"[mqtt] Could not connect: {e}")
            messagebox.showwarning(
                "MQTT",
                f"Could not connect to Pi at {PI_IP}:{BROKER_PORT}\n\n"
                f"Make sure:\n"
                f"1. Pi and laptop are on the same WiFi\n"
                f"2. Mosquitto is running on the Pi\n"
                f"3. pi_agent.py is running on the Pi\n\n"
                f"Error: {e}"
            )

    def _mqtt_on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self._mqtt_connected = True
            client.subscribe(TOPIC_RESPONSE)
            client.subscribe(TOPIC_STATUS)
            print("[mqtt] Connected ✓")
        else:
            print(f"[mqtt] Connection failed rc={rc}")

    def _mqtt_on_disconnect(self, client, userdata, rc):
        self._mqtt_connected = False
        print(f"[mqtt] Disconnected (rc={rc})")

    def _mqtt_on_message(self, client, userdata, msg):
        """Called from MQTT background thread — queue payload for main thread."""
        try:
            data = json.loads(msg.payload.decode())
            self._mqtt_response_queue.put(data)
        except Exception as e:
            print(f"[mqtt] Bad message: {e}")

    def _mqtt_poll_responses(self):
        """Drain MQTT response queue on the main thread (safe for Tkinter)."""
        while not self._mqtt_response_queue.empty():
            try:
                data = self._mqtt_response_queue.get_nowait()
                self._handle_mqtt_response(data)
            except queue.Empty:
                break
        self.root.after(100, self._mqtt_poll_responses)

    def _send_command(self, payload: dict):
        if not self._mqtt_connected:
            messagebox.showerror("Not Connected",
                                 f"Not connected to Pi at {PI_IP}.\n"
                                 "Check WiFi and that pi_agent.py is running.")
            return
        self.mqtt.publish(TOPIC_COMMANDS, json.dumps(payload))
        print(f"[mqtt] → {payload}")

    def _handle_mqtt_response(self, data: dict):
        """Process a response from the Pi (runs on main thread)."""
        action = data.get("action", "")
        status = data.get("status", "")

        # Agent status heartbeat
        if "agent_status" in data:
            s = data["agent_status"]
            color = {"online": "green", "busy": "orange",
                     "stopped": "red", "offline": "red"}.get(s, "gray")
            self._update_mqtt_indicator(s, color)
            return

        if status == "error":
            msg = data.get("message", "Unknown error")
            self._update_status(f"Pi error: {msg}", "red")
            # Log the error
            if action == "move_and_press":
                db.log_button_press(
                    user_id=self.current_user["id"],
                    username=self.current_user["username"],
                    target_x_mm=data.get("x", 0),
                    target_y_mm=data.get("y", 0),
                    press_dist=self.press_distance.get(),
                    status="error", note=msg
                )
            self.is_executing = False
            self.run_button.config(state=tk.NORMAL if self.plotter_initialized else tk.DISABLED)
            return

        if action == "home":
            self.plotter_initialized = True
            self.is_homed  = True
            self.x_max_mm  = data["x_max"]
            self.y_max_mm  = data["y_max"]
            self._update_status(f"Plotter ready ({self.x_max_mm:.1f}×{self.y_max_mm:.1f}mm)", "green")
            self.plotter_status_label.config(
                text=f"Ready  {self.x_max_mm:.1f}×{self.y_max_mm:.1f}mm",
                foreground="green")
            self.stop_button.config(state=tk.NORMAL)
            if self.current_mode.get() == "sequence":
                self.run_button.config(state=tk.NORMAL)

        elif action == "move_and_press":
            x          = data["x"]
            y          = data["y"]
            servo_used = data["servo_used"]
            self.current_x_mm = data.get("current_x", self.current_x_mm)
            self.current_y_mm = data.get("current_y", self.current_y_mm)
            self._update_status(f"Done — servo {servo_used} ✓  logged", "green")
            db.log_button_press(
                user_id=self.current_user["id"],
                username=self.current_user["username"],
                target_x_mm=x, target_y_mm=y,
                servo_used=servo_used,
                press_dist=self.press_distance.get(),
                status="success"
            )
            # Advance the sequence if running
            self._sequence_step_done()

        elif action in ("jog", "move_to", "get_position"):
            self.current_x_mm = data.get("current_x", self.current_x_mm)
            self.current_y_mm = data.get("current_y", self.current_y_mm)
            self._update_jog_labels(data)

        elif action == "set_servo":
            servo = data["servo_num"]
            angle = data["angle"]
            self._update_status(f"Servo {servo} → {angle:.1f}°", "green")

        elif action == "emergency_stop":
            self._update_status("EMERGENCY STOP — motors halted", "red")
            self.is_executing = False
            self.run_button.config(state=tk.DISABLED)

        elif action == "ping":
            self._update_status("Pi is online ✓", "green")

    # =========================================================================
    # Sequence helpers
    # =========================================================================

    def _sequence_step_done(self):
        """Called after each successful move_and_press in sequence mode."""
        if not self.is_executing:
            return
        idx = self._seq_index
        if idx < len(self.points):
            self.canvas.itemconfig(f'point_{idx}', fill='green')
        self._seq_index += 1
        if self._seq_index < len(self._seq_points):
            # Fire next point
            self._seq_fire_next()
        else:
            self._update_status("Sequence complete ✓  all logged", "green")
            self.is_executing = False
            self.run_button.config(state=tk.NORMAL)

    def _seq_fire_next(self):
        idx    = self._seq_index
        mm_x, mm_y = self._seq_points[idx]
        # Highlight on canvas
        if idx < len(self.points):
            self.canvas.itemconfig(f'point_{idx}', fill='red')
            self.points_listbox.selection_clear(0, tk.END)
            self.points_listbox.selection_set(idx)
            self.points_listbox.see(idx)
        self._update_status(f"Point {idx+1}/{len(self._seq_points)}: ({mm_x:.1f}, {mm_y:.1f})", "orange")
        self._send_command({
            "action":     "move_and_press",
            "x":          mm_x,
            "y":          mm_y,
            "press_dist": self.press_distance.get(),
        })

    # =========================================================================
    # UI construction
    # =========================================================================

    def _create_ui(self):
        topbar = tk.Frame(self.root, bg="#1a1d27", height=36)
        topbar.pack(fill=tk.X, side=tk.TOP)
        topbar.pack_propagate(False)
        tk.Label(topbar, text="XY PLOTTER", bg="#1a1d27", fg="#4f8ef7",
                 font=("Courier New", 10, "bold")).pack(side=tk.LEFT, padx=14, pady=8)

        # MQTT indicator
        self._mqtt_status_label = tk.Label(
            topbar, text="● Connecting…", bg="#1a1d27", fg="#f59e0b",
            font=("Courier New", 9))
        self._mqtt_status_label.pack(side=tk.RIGHT, padx=14, pady=8)

        tk.Label(topbar,
                 text=f"👤  {self.current_user['display_name']}  ({self.current_user['username']})",
                 bg="#1a1d27", fg="#9ca3af",
                 font=("Courier New", 9)).pack(side=tk.RIGHT, padx=14, pady=8)

        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        control_tab = ttk.Frame(self.notebook)
        self.notebook.add(control_tab, text="  Plotter Control  ")

        log_tab = ttk.Frame(self.notebook)
        self.notebook.add(log_tab, text="  Button Log  ")

        self._build_control_tab(control_tab)
        self._build_log_tab(log_tab)
        self.notebook.bind("<<NotebookTabChanged>>", self._on_tab_change)

    def _update_mqtt_indicator(self, text, color):
        colors = {"green": "#22c55e", "orange": "#f59e0b",
                  "red": "#ef4444", "gray": "#6b7280"}
        fg = colors.get(color, "#6b7280")
        self._mqtt_status_label.config(text=f"● {text.capitalize()}", fg=fg)

    # ── Tab 1: Plotter Control ────────────────────────────────────────────────

    def _build_control_tab(self, parent):
        main_frame = ttk.Frame(parent)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        ctrl_container = ttk.Frame(main_frame, width=300)
        ctrl_container.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        ctrl_container.pack_propagate(False)

        ctrl_canvas = tk.Canvas(ctrl_container, width=280, highlightthickness=0)
        sb = ttk.Scrollbar(ctrl_container, orient="vertical", command=ctrl_canvas.yview)
        ctrl_frame = ttk.Frame(ctrl_canvas)
        ctrl_frame.bind("<Configure>",
                        lambda e: ctrl_canvas.configure(scrollregion=ctrl_canvas.bbox("all")))
        ctrl_canvas.create_window((0, 0), window=ctrl_frame, anchor="nw")
        ctrl_canvas.configure(yscrollcommand=sb.set)
        ctrl_canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        ctrl_canvas.bind_all("<MouseWheel>",
                             lambda e: ctrl_canvas.yview_scroll(int(-1*(e.delta/120)), "units"))

        canvas_frame = ttk.Frame(main_frame)
        canvas_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        cam_sidebar = ttk.Frame(main_frame, width=300)
        cam_sidebar.pack(side=tk.LEFT, fill=tk.Y, padx=(10, 0))
        cam_sidebar.pack_propagate(False)

        self._create_controls(ctrl_frame)
        self._create_canvas(canvas_frame)
        self._create_camera_sidebar(cam_sidebar)

    def _create_controls(self, parent):
        # ── Connection ────────────────────────────────────────────────────────
        conn_sec = ttk.LabelFrame(parent, text="Pi Connection", padding=10)
        conn_sec.pack(fill=tk.X, pady=(0, 10))
        tk.Label(conn_sec, text=f"Pi IP: {PI_IP}", font=("Courier New", 9),
                 foreground="#6b7280").pack(anchor=tk.W)
        ttk.Button(conn_sec, text="Ping Pi",
                   command=self._ping_pi).pack(fill=tk.X, pady=(4, 0))

        # ── Image ─────────────────────────────────────────────────────────────
        img_sec = ttk.LabelFrame(parent, text="Image", padding=10)
        img_sec.pack(fill=tk.X, pady=(0, 10))
        ttk.Button(img_sec, text="Load Image",
                   command=self._load_image).pack(fill=tk.X, pady=2)
        self.image_info_label = ttk.Label(img_sec, text="No image loaded", wraplength=250)
        self.image_info_label.pack(fill=tk.X, pady=2)

        # ── Plotter ───────────────────────────────────────────────────────────
        plt_sec = ttk.LabelFrame(parent, text="Plotter", padding=10)
        plt_sec.pack(fill=tk.X, pady=(0, 10))
        ttk.Button(plt_sec, text="Initialize & Home",
                   command=self._initialize_plotter).pack(fill=tk.X, pady=2)
        self.plotter_status_label = ttk.Label(plt_sec, text="Not initialized",
                                              foreground="red")
        self.plotter_status_label.pack(fill=tk.X, pady=2)

        # ── Mode ──────────────────────────────────────────────────────────────
        mode_sec = ttk.LabelFrame(parent, text="Mode", padding=10)
        mode_sec.pack(fill=tk.X, pady=(0, 10))
        ttk.Radiobutton(mode_sec, text="Live Mode", variable=self.current_mode,
                        value="live", command=self._on_mode_change).pack(anchor=tk.W)
        ttk.Label(mode_sec, text="  Click → execute immediately",
                  font=('TkDefaultFont', 9), foreground='gray').pack(anchor=tk.W, padx=(20, 0))
        ttk.Radiobutton(mode_sec, text="Sequence Mode", variable=self.current_mode,
                        value="sequence", command=self._on_mode_change).pack(anchor=tk.W, pady=(5, 0))
        ttk.Label(mode_sec, text="  Build list → Run all",
                  font=('TkDefaultFont', 9), foreground='gray').pack(anchor=tk.W, padx=(20, 0))

        # ── Settings ──────────────────────────────────────────────────────────
        set_sec = ttk.LabelFrame(parent, text="Settings", padding=10)
        set_sec.pack(fill=tk.X, pady=(0, 10))
        for label, var in [("Press Distance (mm):", self.press_distance),
                            ("X Offset (mm):", self.offset_x),
                            ("Y Offset (mm):", self.offset_y)]:
            ttk.Label(set_sec, text=label).pack(anchor=tk.W)
            ttk.Entry(set_sec, textvariable=var, width=10).pack(anchor=tk.W, pady=(0, 5))

        # ── Points ────────────────────────────────────────────────────────────
        pts_sec = ttk.LabelFrame(parent, text="Points", padding=10)
        pts_sec.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        self.points_label = ttk.Label(pts_sec, text="Points: 0")
        self.points_label.pack(anchor=tk.W, pady=(0, 5))
        lf = ttk.Frame(pts_sec)
        lf.pack(fill=tk.BOTH, expand=True)
        psb = ttk.Scrollbar(lf)
        psb.pack(side=tk.RIGHT, fill=tk.Y)
        self.points_listbox = tk.Listbox(lf, yscrollcommand=psb.set, height=8)
        self.points_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        psb.config(command=self.points_listbox.yview)
        bf = ttk.Frame(pts_sec)
        bf.pack(fill=tk.X, pady=(5, 0))
        ttk.Button(bf, text="Delete Selected",
                   command=self._delete_selected_point).pack(side=tk.LEFT, fill=tk.X,
                                                             expand=True, padx=(0, 2))
        ttk.Button(bf, text="Clear All",
                   command=self._clear_all_points).pack(side=tk.LEFT, fill=tk.X,
                                                        expand=True, padx=(2, 0))

        # ── Jog ───────────────────────────────────────────────────────────────
        jog_sec = ttk.LabelFrame(parent, text="Manual Jog", padding=10)
        jog_sec.pack(fill=tk.X, pady=(0, 10))
        ttk.Button(jog_sec, text="Jog Control",
                   command=self._open_jog_dialog).pack(fill=tk.X)
        ttk.Label(jog_sec, text="Manual axis movement",
                  font=('TkDefaultFont', 9), foreground='gray').pack(anchor=tk.W)

        # ── Servo setup ───────────────────────────────────────────────────────
        setup_sec = ttk.LabelFrame(parent, text="End Effector Setup", padding=10)
        setup_sec.pack(fill=tk.X, pady=(0, 10))
        ttk.Button(setup_sec, text="Setup Tool",
                   command=self._open_setup_dialog).pack(fill=tk.X)
        ttk.Label(setup_sec, text="Load racks into servo gears",
                  font=('TkDefaultFont', 9), foreground='gray').pack(anchor=tk.W)

        # ── Execution ─────────────────────────────────────────────────────────
        exec_sec = ttk.LabelFrame(parent, text="Execution", padding=10)
        exec_sec.pack(fill=tk.X)
        self.run_button = ttk.Button(exec_sec, text="Run Sequence",
                                     command=self._run_sequence, state=tk.DISABLED)
        self.run_button.pack(fill=tk.X, pady=(0, 5))
        self.stop_button = ttk.Button(exec_sec, text="Emergency Stop",
                                      command=self._emergency_stop, state=tk.DISABLED)
        self.stop_button.pack(fill=tk.X)
        self.status_label = ttk.Label(exec_sec, text="Ready", foreground="green")
        self.status_label.pack(fill=tk.X, pady=(5, 0))

    def _create_canvas(self, parent):
        cc = ttk.Frame(parent)
        cc.pack(fill=tk.BOTH, expand=True)
        h_sb = ttk.Scrollbar(cc, orient=tk.HORIZONTAL)
        h_sb.pack(side=tk.BOTTOM, fill=tk.X)
        v_sb = ttk.Scrollbar(cc, orient=tk.VERTICAL)
        v_sb.pack(side=tk.RIGHT, fill=tk.Y)
        self.canvas = tk.Canvas(cc, bg='gray',
                                xscrollcommand=h_sb.set, yscrollcommand=v_sb.set)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        h_sb.config(command=self.canvas.xview)
        v_sb.config(command=self.canvas.yview)
        self.canvas.bind('<Button-1>', self._on_canvas_click)
        self.canvas.create_text(400, 300, text="Load an image to begin",
                                font=('Arial', 16), fill='white', tags='instructions')

    # =========================================================================
    # IP Camera Sidebar
    # =========================================================================

    def _create_camera_sidebar(self, parent):
        hdr = tk.Frame(parent, bg="#1a1d27", height=32)
        hdr.pack(fill=tk.X)
        hdr.pack_propagate(False)
        tk.Label(hdr, text="IP CAMERA", bg="#1a1d27", fg="#4f8ef7",
                 font=("Courier New", 9, "bold")).pack(side=tk.LEFT, padx=8, pady=7)
        self._cam_dot = tk.Canvas(hdr, width=10, height=10,
                                  bg="#1a1d27", highlightthickness=0)
        self._cam_dot.pack(side=tk.RIGHT, padx=8, pady=11)
        self._cam_dot.create_oval(1, 1, 9, 9, fill="#374151", outline="", tags="dot")

        url_frame = tk.Frame(parent, bg="#252839")
        url_frame.pack(fill=tk.X, padx=6, pady=(6, 0))
        tk.Label(url_frame, text="URL", bg="#252839", fg="#6b7280",
                 font=("Courier New", 8, "bold")).pack(side=tk.LEFT, padx=(6, 4), pady=6)
        url_entry = tk.Entry(url_frame, textvariable=self._cam_url,
                             bg="#1a1d27", fg="#e8eaf0",
                             insertbackground="#e8eaf0",
                             font=("Courier New", 9),
                             relief=tk.FLAT, bd=0)
        url_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4), pady=6)
        self._set_placeholder(url_entry, "http://192.168.1.x/video")

        btn_row = tk.Frame(parent, bg="#0f1117")
        btn_row.pack(fill=tk.X, padx=6, pady=4)
        self._cam_connect_btn = tk.Button(
            btn_row, text="Connect",
            bg="#4f8ef7", fg="white", activebackground="#3b74e0",
            font=("Courier New", 9, "bold"), relief=tk.FLAT, cursor="hand2",
            padx=10, pady=4, command=self._cam_connect)
        self._cam_connect_btn.pack(side=tk.LEFT, padx=(0, 4))
        self._cam_disconnect_btn = tk.Button(
            btn_row, text="Disconnect",
            bg="#374151", fg="#9ca3af", activebackground="#4b5563",
            font=("Courier New", 9), relief=tk.FLAT, cursor="hand2",
            padx=10, pady=4, command=self._cam_disconnect,
            state=tk.DISABLED)
        self._cam_disconnect_btn.pack(side=tk.LEFT)

        vid_outer = tk.Frame(parent, bg="#0a0c12",
                             highlightbackground="#2d3148", highlightthickness=1)
        vid_outer.pack(fill=tk.X, padx=6, pady=6)
        self._cam_canvas = tk.Canvas(vid_outer,
                                     width=self.CAM_W, height=self.CAM_H,
                                     bg="#0a0c12", highlightthickness=0)
        self._cam_canvas.pack()
        self._cam_canvas.create_text(
            self.CAM_W // 2, self.CAM_H // 2,
            text="No feed", fill="#374151",
            font=("Courier New", 13), tags="placeholder")

        status_row = tk.Frame(parent, bg="#0f1117")
        status_row.pack(fill=tk.X, padx=6)
        tk.Label(status_row, textvariable=self._cam_status,
                 bg="#0f1117", fg="#6b7280",
                 font=("Courier New", 8)).pack(side=tk.LEFT)
        tk.Label(status_row, textvariable=self._cam_fps_display,
                 bg="#0f1117", fg="#4f8ef7",
                 font=("Courier New", 8)).pack(side=tk.RIGHT)

        snap_frame = ttk.LabelFrame(parent, text="Snapshot", padding=8)
        snap_frame.pack(fill=tk.X, padx=6, pady=(10, 4))
        ttk.Button(snap_frame, text="Save Frame as PNG",
                   command=self._cam_snapshot).pack(fill=tk.X)
        tk.Label(snap_frame, text="Saves current camera frame to disk",
                 font=("TkDefaultFont", 8), foreground="gray").pack(anchor=tk.W, pady=(4, 0))

        tips = ttk.LabelFrame(parent, text="Supported URL formats", padding=8)
        tips.pack(fill=tk.X, padx=6, pady=(6, 0))
        for tip in ["MJPEG stream:  .../video",
                    "Single JPEG:   .../shot.jpg",
                    "With auth:     http://user:pass@ip/"]:
            tk.Label(tips, text=tip, font=("Courier New", 7),
                     foreground="#6b7280", anchor="w").pack(fill=tk.X)

    def _set_placeholder(self, entry, text):
        entry.insert(0, text)
        entry.config(fg="#4b5563")
        def on_focus_in(e):
            if entry.get() == text:
                entry.delete(0, tk.END)
                entry.config(fg="#e8eaf0")
        def on_focus_out(e):
            if not entry.get():
                entry.insert(0, text)
                entry.config(fg="#4b5563")
        entry.bind("<FocusIn>",  on_focus_in)
        entry.bind("<FocusOut>", on_focus_out)

    # =========================================================================
    # Camera logic
    # =========================================================================

    def _cam_connect(self):
        url = self._cam_url.get().strip()
        if not url or url == "http://192.168.1.x/video":
            messagebox.showwarning("Camera", "Please enter a valid camera URL.")
            return
        if self._cam_running:
            self._cam_stop_thread()
        self._cam_running = True
        self._cam_status.set("Connecting…")
        self._cam_connect_btn.config(state=tk.DISABLED)
        self._cam_disconnect_btn.config(state=tk.NORMAL)
        self._cam_thread = threading.Thread(target=self._cam_rtsp_loop, args=(url,), daemon=True)
        self._cam_thread.start()
        self._cam_ui_poll()

    def _cam_disconnect(self):
        self._cam_stop_thread()
        self._cam_status.set("Disconnected")
        self._cam_fps_display.set("— fps")
        self._cam_connect_btn.config(state=tk.NORMAL)
        self._cam_disconnect_btn.config(state=tk.DISABLED)
        self._cam_dot.itemconfig("dot", fill="#374151")
        self._cam_canvas.delete("all")
        self._cam_canvas.create_text(self.CAM_W // 2, self.CAM_H // 2,
                                     text="No feed", fill="#374151",
                                     font=("Courier New", 13), tags="placeholder")

    def _cam_stop_thread(self):
        self._cam_running = False
        if self._cam_after_id:
            self.root.after_cancel(self._cam_after_id)
            self._cam_after_id = None
        while not self._cam_frame_queue.empty():
            try: self._cam_frame_queue.get_nowait()
            except queue.Empty: break

    def _cam_push_frame_pil(self, img):
        self._cam_last_pil = img.copy()
        img.thumbnail((self.CAM_W, self.CAM_H), Image.Resampling.LANCZOS)
        if self._cam_frame_queue.full():
            try: self._cam_frame_queue.get_nowait()
            except queue.Empty: pass
        self._cam_frame_queue.put_nowait(img)

    def _cam_on_connect_failed(self):
        self._cam_dot.itemconfig("dot", fill="#ef4444")
        self._cam_connect_btn.config(state=tk.NORMAL)
        self._cam_disconnect_btn.config(state=tk.DISABLED)

    def _cam_rtsp_loop(self, url):
        cap = cv2.VideoCapture()
        cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 8000)
        cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000)
        opened = cap.open(url)
        if not opened or not cap.isOpened():
            self.root.after(0, lambda: self._cam_status.set("Error: could not connect"))
            self._cam_running = False
            self.root.after(0, self._cam_on_connect_failed)
            return
        self.root.after(0, lambda: self._cam_status.set("Connected — waiting for frames…"))
        while self._cam_running:
            ret, frame = cap.read()
            if not ret:
                if self._cam_running:
                    self.root.after(0, lambda: self._cam_status.set("Error: stream lost"))
                    self._cam_running = False
                    self.root.after(0, self._cam_on_connect_failed)
                break
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            self._cam_push_frame_pil(img)
        cap.release()

    def _cam_ui_poll(self):
        updated = False
        while not self._cam_frame_queue.empty():
            try: img = self._cam_frame_queue.get_nowait()
            except queue.Empty: break
            photo = ImageTk.PhotoImage(img)
            self._cam_photo = photo
            iw, ih = img.size
            x = (self.CAM_W - iw) // 2
            y = (self.CAM_H - ih) // 2
            self._cam_canvas.delete("all")
            self._cam_canvas.create_image(x, y, anchor=tk.NW, image=photo)
            updated = True
            self._cam_fps_counter += 1
        if updated:
            self._cam_status.set("Live")
            self._cam_dot.itemconfig("dot", fill="#22c55e")
        now = time.time()
        if now - self._cam_fps_ts >= 1.0:
            fps = self._cam_fps_counter / (now - self._cam_fps_ts)
            self._cam_fps_display.set(f"{fps:.1f} fps")
            self._cam_fps_counter = 0
            self._cam_fps_ts = now
        if not self._cam_running and self._cam_thread and not self._cam_thread.is_alive():
            self._cam_dot.itemconfig("dot", fill="#ef4444")
            self._cam_connect_btn.config(state=tk.NORMAL)
            self._cam_disconnect_btn.config(state=tk.DISABLED)
            return
        if self._cam_running:
            self._cam_after_id = self.root.after(33, self._cam_ui_poll)

    def _cam_snapshot(self):
        if self._cam_last_pil is None:
            messagebox.showinfo("Snapshot", "No camera frame available yet.")
            return
        filepath = filedialog.asksaveasfilename(
            title="Save Camera Frame",
            defaultextension=".png",
            filetypes=[("PNG image", "*.png"), ("JPEG image", "*.jpg")],
            initialfile=f"cam_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png",
        )
        if not filepath:
            return
        try:
            self._cam_last_pil.save(filepath)
            messagebox.showinfo("Snapshot", f"Frame saved to:\n{filepath}")
        except Exception as e:
            messagebox.showerror("Snapshot Error", str(e))

    # =========================================================================
    # Tab 2: Button Log
    # =========================================================================

    def _build_log_tab(self, parent):
        top = ttk.Frame(parent)
        top.pack(fill=tk.X, padx=10, pady=(10, 4))
        ttk.Label(top, text="Button Press History",
                  font=('TkDefaultFont', 11, 'bold')).pack(side=tk.LEFT)
        ttk.Button(top, text="⟳  Refresh", command=self._refresh_log).pack(side=tk.RIGHT)

        filter_frame = ttk.Frame(parent)
        filter_frame.pack(fill=tk.X, padx=10, pady=(0, 6))
        ttk.Label(filter_frame, text="Filter user:").pack(side=tk.LEFT)
        self._log_filter = tk.StringVar(value="all")
        fc = ttk.Combobox(filter_frame, textvariable=self._log_filter,
                          values=["all", "admin", "operator"],
                          width=12, state="readonly")
        fc.pack(side=tk.LEFT, padx=6)
        fc.bind("<<ComboboxSelected>>", lambda _: self._refresh_log())

        cols = ("id", "pressed_at", "username", "x_mm", "y_mm",
                "servo", "press_dist", "status", "note")
        col_labels = {
            "id": "#", "pressed_at": "Timestamp", "username": "User",
            "x_mm": "X (mm)", "y_mm": "Y (mm)", "servo": "Servo",
            "press_dist": "Press dist", "status": "Status", "note": "Note",
        }
        col_widths = {
            "id": 40, "pressed_at": 160, "username": 80,
            "x_mm": 70, "y_mm": 70, "servo": 50,
            "press_dist": 80, "status": 70, "note": 140,
        }

        tree_frame = ttk.Frame(parent)
        tree_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))
        tsb_y = ttk.Scrollbar(tree_frame, orient=tk.VERTICAL)
        tsb_y.pack(side=tk.RIGHT, fill=tk.Y)
        tsb_x = ttk.Scrollbar(tree_frame, orient=tk.HORIZONTAL)
        tsb_x.pack(side=tk.BOTTOM, fill=tk.X)
        self.log_tree = ttk.Treeview(tree_frame, columns=cols, show="headings",
                                     yscrollcommand=tsb_y.set, xscrollcommand=tsb_x.set)
        for c in cols:
            self.log_tree.heading(c, text=col_labels[c])
            self.log_tree.column(c, width=col_widths[c], anchor=tk.CENTER)
        self.log_tree.pack(fill=tk.BOTH, expand=True)
        tsb_y.config(command=self.log_tree.yview)
        tsb_x.config(command=self.log_tree.xview)
        self.log_tree.tag_configure("success", background="#f0fdf4")
        self.log_tree.tag_configure("error",   background="#fef2f2")
        self.log_tree.tag_configure("mine",    font=('TkDefaultFont', 9, 'bold'))

        self.log_summary = ttk.Label(parent, text="", foreground="gray")
        self.log_summary.pack(padx=10, pady=(0, 6), anchor=tk.W)

    def _refresh_log(self):
        for row in self.log_tree.get_children():
            self.log_tree.delete(row)
        fu = self._log_filter.get()
        entries = db.get_log(300) if fu == "all" else db.get_log_for_user(fu, 300)
        for e in entries:
            tags = [e["status"]]
            if e["username"] == self.current_user["username"]:
                tags.append("mine")
            self.log_tree.insert("", tk.END, values=(
                e["id"], e["pressed_at"], e["username"],
                f'{e["target_x_mm"]:.2f}', f'{e["target_y_mm"]:.2f}',
                str(e["servo_used"]) if e["servo_used"] else "—",
                f'{e["press_dist"]:.1f}' if e["press_dist"] else "—",
                e["status"], e["note"] or "",
            ), tags=tuple(tags))
        success = sum(1 for e in entries if e["status"] == "success")
        self.log_summary.config(
            text=f"Showing {len(entries)} entries  ·  {success} success  ·  {len(entries)-success} error")

    def _on_tab_change(self, _event):
        if self.notebook.index("current") == 1:
            self._refresh_log()

    # =========================================================================
    # Image loading
    # =========================================================================

    def _load_image(self):
        filepath = filedialog.askopenfilename(
            title="Select Image",
            filetypes=[("Image files", "*.png *.jpg *.jpeg *.bmp *.gif *.tiff"),
                       ("All files", "*.*")])
        if not filepath:
            return
        dialog = tk.Toplevel(self.root)
        dialog.title("Image Dimensions")
        dialog.geometry("300x150")
        dialog.transient(self.root)
        dialog.grab_set()
        ttk.Label(dialog, text="Enter real-world dimensions:").pack(pady=10)
        width_var  = tk.DoubleVar(value=200.0)
        height_var = tk.DoubleVar(value=150.0)
        for label, var in [("Width (mm):", width_var), ("Height (mm):", height_var)]:
            f = ttk.Frame(dialog)
            f.pack(pady=5)
            ttk.Label(f, text=label).pack(side=tk.LEFT)
            ttk.Entry(f, textvariable=var, width=10).pack(side=tk.LEFT)

        def on_ok():
            self.real_width_mm  = width_var.get()
            self.real_height_mm = height_var.get()
            self.image_path = filepath
            dialog.destroy()
            self._display_image()

        ttk.Button(dialog, text="OK", command=on_ok).pack(pady=10)
        dialog.wait_window()

    def _display_image(self):
        try:
            self.image = Image.open(self.image_path)
            iw, ih = self.image.size
            self.scale_factor = min(800/iw, 600/ih, 1.0)
            dw = int(iw * self.scale_factor)
            dh = int(ih * self.scale_factor)
            disp = self.image.resize((dw, dh), Image.Resampling.LANCZOS)
            self.image_tk = ImageTk.PhotoImage(disp)
            self.canvas.delete('all')
            self.canvas.create_image(0, 0, anchor=tk.NW, image=self.image_tk)
            self.canvas.config(scrollregion=(0, 0, dw, dh))
            self.image_info_label.config(
                text=f"Image: {dw}×{dh}px\nReal: {self.real_width_mm}×{self.real_height_mm}mm")
            self._clear_all_points()
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load image: {e}")

    # =========================================================================
    # Plotter init
    # =========================================================================

    def _initialize_plotter(self):
        if not self._mqtt_connected:
            messagebox.showerror("Not Connected",
                                 f"Cannot reach Pi at {PI_IP}.\n"
                                 "Check WiFi and pi_agent.py.")
            return
        if self.plotter_initialized:
            if not messagebox.askyesno("Re-initialize?", "Re-home the plotter?"):
                return
        self._update_status("Sending home command to Pi…", "orange")
        self.plotter_status_label.config(text="Homing…", foreground="orange")
        self._send_command({"action": "home"})

    def _ping_pi(self):
        self._update_status("Pinging Pi…", "orange")
        self._send_command({"action": "ping"})

    # =========================================================================
    # Canvas click & point management
    # =========================================================================

    def _on_canvas_click(self, event):
        if self.image is None:
            messagebox.showwarning("No Image", "Please load an image first")
            return
        if not self.plotter_initialized:
            messagebox.showwarning("Plotter Not Ready", "Please initialize plotter first")
            return
        if self.is_executing:
            return
        canvas_x = self.canvas.canvasx(event.x)
        canvas_y = self.canvas.canvasy(event.y)
        mm_x = (canvas_x / self.scale_factor / self.image.width) * self.real_width_mm \
               + self.offset_x.get()
        mm_y = self.real_height_mm \
               - (canvas_y / self.scale_factor / self.image.height) * self.real_height_mm \
               + self.offset_y.get()
        self._add_point(mm_x, mm_y, canvas_x, canvas_y)
        if self.current_mode.get() == "live":
            self._execute_live_point(len(self.points) - 1)

    def _add_point(self, mm_x, mm_y, canvas_x, canvas_y):
        self.points.append((mm_x, mm_y))
        self.points_listbox.insert(tk.END, f"{len(self.points)}: ({mm_x:.1f}, {mm_y:.1f})")
        color = "yellow" if self.current_mode.get() == "sequence" else "cyan"
        marker = self.canvas.create_oval(
            canvas_x-5, canvas_y-5, canvas_x+5, canvas_y+5,
            fill=color, outline="black", width=2, tags=f'point_{len(self.points)-1}')
        self.point_markers.append(marker)
        self.canvas.create_text(canvas_x, canvas_y-15, text=str(len(self.points)),
                                fill="white", font=('Arial', 10, 'bold'),
                                tags=f'point_{len(self.points)-1}')
        self.points_label.config(text=f"Points: {len(self.points)}")

    def _delete_selected_point(self):
        if self.is_executing: return
        sel = self.points_listbox.curselection()
        if not sel: return
        idx = sel[0]
        self.points.pop(idx)
        self.points_listbox.delete(idx)
        self.canvas.delete(f'point_{idx}')
        self._refresh_points_display()

    def _clear_all_points(self):
        if self.is_executing: return
        self.points.clear()
        self.points_listbox.delete(0, tk.END)
        for m in self.point_markers:
            self.canvas.delete(m)
        self.point_markers.clear()
        self.canvas.delete('point')
        self.points_label.config(text="Points: 0")

    def _refresh_points_display(self):
        self.points_listbox.delete(0, tk.END)
        for i in range(len(self.point_markers) + 10):
            self.canvas.delete(f'point_{i}')
        self.point_markers.clear()
        for i, (mm_x, mm_y) in enumerate(self.points):
            px = (mm_x - self.offset_x.get()) * self.image.width / self.real_width_mm
            py = self.image.height - ((mm_y - self.offset_y.get()) *
                                      self.image.height / self.real_height_mm)
            cx = px * self.scale_factor
            cy = py * self.scale_factor
            self.points_listbox.insert(tk.END, f"{i+1}: ({mm_x:.1f}, {mm_y:.1f})")
            color = "yellow" if self.current_mode.get() == "sequence" else "cyan"
            m = self.canvas.create_oval(cx-5, cy-5, cx+5, cy+5,
                                        fill=color, outline="black", width=2,
                                        tags=f'point_{i}')
            self.point_markers.append(m)
            self.canvas.create_text(cx, cy-15, text=str(i+1),
                                    fill="white", font=('Arial', 10, 'bold'),
                                    tags=f'point_{i}')
        self.points_label.config(text=f"Points: {len(self.points)}")

    # =========================================================================
    # Execution
    # =========================================================================

    def _execute_live_point(self, index: int):
        """Live mode: immediately send the clicked point to Pi."""
        mm_x, mm_y = self.points[index]
        self.canvas.itemconfig(f'point_{index}', fill='red')
        self._update_status(f"Sending ({mm_x:.1f}, {mm_y:.1f}) to Pi…", "orange")
        self.is_executing = True
        self._seq_index = index    # reuse sequence tracking for response handling
        self._seq_points = self.points[:]
        self._single_live = True
        self._send_command({
            "action":     "move_and_press",
            "x":          mm_x,
            "y":          mm_y,
            "press_dist": self.press_distance.get(),
        })

    def _run_sequence(self):
        if not self.points:
            messagebox.showinfo("No Points", "No points to execute")
            return
        if not self.plotter_initialized:
            messagebox.showwarning("Not Ready", "Initialize plotter first")
            return
        self.is_executing  = True
        self._seq_points   = self.points[:]
        self._seq_index    = 0
        self._single_live  = False
        self.run_button.config(state=tk.DISABLED)
        self._seq_fire_next()

    def _emergency_stop(self):
        self.is_executing = False
        self._send_command({"action": "emergency_stop"})
        self._update_status("EMERGENCY STOP sent", "red")

    def _on_mode_change(self):
        mode = self.current_mode.get()
        self.run_button.config(
            state=tk.NORMAL if (mode == "sequence" and self.plotter_initialized)
            else tk.DISABLED)
        self._refresh_points_display()

    def _update_status(self, message, color="black"):
        self.status_label.config(text=message, foreground=color)
        self.root.update_idletasks()

    # =========================================================================
    # Jog dialog
    # =========================================================================

    # def _jog_labels: Optional[dict] = None   # type: ignore

    def _update_jog_labels(self, data: dict):
        """Update position labels in jog dialog if it's open."""
        if not hasattr(self, '_jog_label_refs'):
            return
        refs = self._jog_label_refs
        if 'gantry' in refs:
            cx = data.get("current_x", self.current_x_mm)
            cy = data.get("current_y", self.current_y_mm)
            refs['gantry'].config(text=f"Gantry — X: {cx:.1f}mm, Y: {cy:.1f}mm")
        if 'ef1' in refs:
            refs['ef1'].config(
                text=f"EF1 — X: {data.get('ef1_x', 0):.1f}mm, Y: {data.get('ef1_y', 0):.1f}mm")
        if 'ef2' in refs:
            refs['ef2'].config(
                text=f"EF2 — X: {data.get('ef2_x', 0):.1f}mm, Y: {data.get('ef2_y', 0):.1f}mm")

    def _open_jog_dialog(self):
        if not self.plotter_initialized:
            messagebox.showwarning("Plotter Not Ready", "Please initialize plotter first")
            return
        dialog = tk.Toplevel(self.root)
        dialog.title("Manual Jog Control")
        dialog.geometry("500x580")
        dialog.transient(self.root)

        pos_frame = ttk.LabelFrame(dialog, text="Current Positions", padding=10)
        pos_frame.pack(fill=tk.X, padx=10, pady=10)
        gantry_lbl = ttk.Label(pos_frame,
                               text=f"Gantry — X: {self.current_x_mm:.1f}mm, Y: {self.current_y_mm:.1f}mm",
                               font=('TkDefaultFont', 11))
        gantry_lbl.pack(anchor=tk.W)
        ef1_lbl = ttk.Label(pos_frame, text="EF1 (servo 1) — …")
        ef1_lbl.pack(anchor=tk.W)
        ef2_lbl = ttk.Label(pos_frame, text="EF2 (servo 2) — …")
        ef2_lbl.pack(anchor=tk.W)

        self._jog_label_refs = {'gantry': gantry_lbl, 'ef1': ef1_lbl, 'ef2': ef2_lbl}

        def refresh():
            self._send_command({"action": "get_position"})

        step_frame = ttk.LabelFrame(dialog, text="Step Size", padding=10)
        step_frame.pack(fill=tk.X, padx=10, pady=5)
        step_size = tk.DoubleVar(value=10.0)
        for lbl, val in [("1mm", 1), ("5mm", 5), ("10mm", 10), ("50mm", 50)]:
            ttk.Radiobutton(step_frame, text=lbl, variable=step_size,
                            value=val).pack(side=tk.LEFT, expand=True)

        jog_frame = ttk.LabelFrame(dialog, text="Jog", padding=10)
        jog_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        def jog(dx, dy):
            self._send_command({"action": "jog",
                                "dx": dx * step_size.get(),
                                "dy": dy * step_size.get()})

        grid = ttk.Frame(jog_frame)
        grid.pack(expand=True)
        ttk.Button(grid, text="Y+",     width=8, command=lambda: jog(0,  1)).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(grid, text="X-",     width=8, command=lambda: jog(-1, 0)).grid(row=1, column=0, padx=5, pady=5)
        ttk.Button(grid, text="Home\n(0,0)", width=8,
                   command=lambda: self._send_command({"action": "move_to", "x": 0, "y": 0})
                   ).grid(row=1, column=1, padx=5, pady=5)
        ttk.Button(grid, text="X+",     width=8, command=lambda: jog(1,  0)).grid(row=1, column=2, padx=5, pady=5)
        ttk.Button(grid, text="Y-",     width=8, command=lambda: jog(0, -1)).grid(row=2, column=1, padx=5, pady=5)

        goto_frame = ttk.LabelFrame(dialog, text="Go To Position", padding=10)
        goto_frame.pack(fill=tk.X, padx=10, pady=5)
        gf = ttk.Frame(goto_frame)
        gf.pack(fill=tk.X)
        ttk.Label(gf, text="X:").pack(side=tk.LEFT)
        gx = tk.DoubleVar(value=0.0)
        ttk.Entry(gf, textvariable=gx, width=8).pack(side=tk.LEFT, padx=5)
        ttk.Label(gf, text="Y:").pack(side=tk.LEFT)
        gy = tk.DoubleVar(value=0.0)
        ttk.Entry(gf, textvariable=gy, width=8).pack(side=tk.LEFT, padx=5)
        ttk.Button(gf, text="Go",
                   command=lambda: self._send_command({"action": "move_to",
                                                       "x": gx.get(), "y": gy.get()})
                   ).pack(side=tk.LEFT, padx=5)

        refresh()
        ttk.Button(dialog, text="Refresh", command=refresh).pack(pady=4)
        ttk.Button(dialog, text="Close",
                   command=lambda: [dialog.destroy(),
                                    self.__dict__.pop('_jog_label_refs', None)]
                   ).pack(pady=6)

    # =========================================================================
    # Setup dialog
    # =========================================================================

    def _open_setup_dialog(self):
        if not self.plotter_initialized:
            messagebox.showwarning("Plotter Not Ready", "Please initialize plotter first")
            return
        dialog = tk.Toplevel(self.root)
        dialog.title("End Effector Setup")
        dialog.geometry("400x500")
        dialog.transient(self.root)
        ttk.Label(dialog,
                  text="Load racks into servo gears:\n"
                       "1. Select servo\n2. Push Forward\n3. Back Up until rack engages",
                  justify=tk.LEFT, padding=10).pack(fill=tk.X)
        servo_frame = ttk.LabelFrame(dialog, text="Select Servo", padding=10)
        servo_frame.pack(fill=tk.X, padx=10, pady=5)
        sel_servo = tk.IntVar(value=1)
        ttk.Radiobutton(servo_frame, text="Servo 1", variable=sel_servo, value=1).pack(anchor=tk.W)
        ttk.Radiobutton(servo_frame, text="Servo 2", variable=sel_servo, value=2).pack(anchor=tk.W)

        push = ttk.LabelFrame(dialog, text="Step 1: Push Forward", padding=10)
        push.pack(fill=tk.X, padx=10, pady=5)
        ttk.Button(push, text="▶ Push Forward (Max)",
                   command=lambda: self._send_command({
                       "action": "set_servo", "servo_num": sel_servo.get(), "angle": 180
                   })).pack(fill=tk.X)

        back = ttk.LabelFrame(dialog, text="Step 2: Back Up", padding=10)
        back.pack(fill=tk.X, padx=10, pady=5)
        bf = ttk.Frame(back)
        bf.pack(fill=tk.X)

        angle_var = tk.DoubleVar(value=90.0)

        def change(delta):
            angle_var.set(max(0, min(180, angle_var.get() + delta)))
            self._send_command({"action": "set_servo",
                                "servo_num": sel_servo.get(),
                                "angle": angle_var.get()})

        ttk.Button(bf, text="◀◀ –10°", command=lambda: change(-10)).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        ttk.Button(bf, text="◀ –1°",   command=lambda: change(-1)).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        ttk.Button(bf, text="+1° ▶",   command=lambda: change(1)).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        rst = ttk.LabelFrame(dialog, text="Reset", padding=10)
        rst.pack(fill=tk.X, padx=10, pady=5)
        ttk.Button(rst, text="Reset to Neutral (90°)",
                   command=lambda: [angle_var.set(90),
                                    self._send_command({"action": "set_servo",
                                                        "servo_num": sel_servo.get(),
                                                        "angle": 90})]
                   ).pack(fill=tk.X)
        ttk.Button(dialog, text="Close", command=dialog.destroy).pack(pady=10)

    # =========================================================================
    # Run / cleanup
    # =========================================================================

    def run(self):
        self.root.mainloop()

    def cleanup(self):
        self._cam_stop_thread()
        self.mqtt.loop_stop()
        self.mqtt.disconnect()


# ============================================================================
# Main
# ============================================================================

def main():
    db.init_db()

    login = LoginWindow()
    user  = login.run()
    if user is None:
        print("Login cancelled – exiting.")
        return

    print(f"[auth] Logged in as: {user['username']} ({user['display_name']})")

    app = PlotterControlGUI(current_user=user)
    try:
        app.run()
    finally:
        app.cleanup()


if __name__ == "__main__":
    main()