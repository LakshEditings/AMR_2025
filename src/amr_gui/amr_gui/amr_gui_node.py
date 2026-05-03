#!/usr/bin/env python3
"""
AMR GUI Node - Window 2
======================
Main View  : 2D SLAM map centred on robot with live LiDAR rays
Minimap    : Top-right corner — full accumulated map overview
Side Panel : Remote Control buttons + robot status + speed sliders + MAP UPLOAD DROPDOWN
"""

import math
import threading
import tkinter as tk
from tkinter import font as tkfont
from tkinter import ttk, messagebox
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from slam_toolbox.srv import Clear


# ─── Colour palette ───────────────────────────────────────────────────────────
BG_DARK   = "#0d0d1a"
BG_MID    = "#12122a"
BG_PANEL  = "#16213e"
BG_CANVAS = "#050510"
ACCENT    = "#e94560"
ACCENT2   = "#0f3460"
GREEN     = "#00ff88"
CYAN      = "#00ccff"
WHITE     = "#ffffff"
GREY_DIM  = "#1e2a4a"


def generate_sdf(shape, r=0.5, sx=1, sy=1, sz=1):
    """Generate SDF string for yellow primitives."""
    geom = ""
    if shape == "cylinder":
        geom = f"<cylinder><radius>{r}</radius><length>{sz}</length></cylinder>"
    elif shape == "box":
        geom = f"<box><size>{sx} {sy} {sz}</size></box>"
    
    return f"""<?xml version='1.0'?>
    <sdf version='1.6'>
      <model name='dyn_obs'>
        <static>true</static>
        <link name='link'>
          <visual name='visual'>
            <geometry>{geom}</geometry>
            <material><ambient>1 1 0 1</ambient><diffuse>1 1 0 1</diffuse></material>
          </visual>
          <collision name='collision'><geometry>{geom}</geometry></collision>
        </link>
      </model>
    </sdf>"""


class AMRGuiNode(Node):
    """ROS2 node — 2-D SLAM viewer (with minimap) + remote control + map spawner."""

    def __init__(self):
        super().__init__("amr_gui_node")

        # ── Publishers & Clients ─────────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_cli = self.create_client(DeleteEntity, '/delete_entity')
        self.slam_reset_cli = self.create_client(Clear, '/slam_toolbox/clear')

        # ── QoS for map (latched) ────────────────────────────────────────────
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # ── Subscribers ──────────────────────────────────────────────────────
        self.scan_sub = self.create_subscription(LaserScan, "scan", self._on_scan, 10)
        self.odom_sub = self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, "map", self._on_map, map_qos)

        # ── State ────────────────────────────────────────────────────────────
        self.latest_scan: LaserScan | None = None
        self.robot_x    = 0.0
        self.robot_y    = 0.0
        self.robot_yaw  = 0.0
        self.occ_grid: OccupancyGrid | None = None  
        self.linear_speed  = 0.3
        self.angular_speed = 1.0
        self.current_obstacles = []

        # ── Build GUI ────────────────────────────────────────────────────────
        self.root = tk.Tk()
        self.root.title("AMR Navigator — Live 2D View")
        self.root.configure(bg=BG_DARK)
        self.root.geometry("1200x750")
        self.root.minsize(900, 600)
        self._build_gui()

        # Start render loop
        self.root.after(50, self._render)

    # ─────────────────────────────────────────────────────────────────────────
    # GUI construction
    # ─────────────────────────────────────────────────────────────────────────

    def _build_gui(self):
        title_font  = tkfont.Font(family="Helvetica", size=14, weight="bold")
        btn_font    = tkfont.Font(family="Helvetica", size=11, weight="bold")
        label_font  = tkfont.Font(family="Helvetica", size=9)
        info_font   = tkfont.Font(family="Courier", size=9)

        # Title bar
        title_bar = tk.Frame(self.root, bg="#0a0a18", pady=6)
        title_bar.pack(fill=tk.X)
        tk.Label(title_bar, text="⬡ AMR Navigator — Live 2D View", font=title_font, fg=ACCENT, bg="#0a0a18").pack()

        # Main container
        main = tk.Frame(self.root, bg=BG_DARK)
        main.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        main.columnconfigure(0, weight=4)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        # ── Left: main canvas ───────────────────────────────────────────────
        left = tk.Frame(main, bg=ACCENT2, bd=2, relief=tk.RIDGE)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 3))

        self.canvas = tk.Canvas(left, bg=BG_CANVAS, highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # ── Right: control panel ─────────────────────────────────────────────
        right = tk.Frame(main, bg=BG_PANEL, bd=2, relief=tk.RIDGE)
        right.pack_propagate(False)
        right.grid(row=0, column=1, sticky="nsew")

        # Map Upload Dropdown
        m = tk.Frame(right, bg=BG_PANEL)
        m.pack(fill=tk.X, padx=8, pady=(8, 4))
        tk.Label(m, text="Upload Map", font=label_font, fg=ACCENT, bg=BG_PANEL).pack(anchor="w")
        
        self.map_var = tk.StringVar()
        map_cb = ttk.Combobox(m, textvariable=self.map_var, state="readonly", font=info_font)
        map_cb['values'] = ("Select Map...", "Map 1 (5 Circles)", "Map 2 (Crescent & Circles)", "Map 3 (Shapes)", "REALTIME MAP (SLAM)")
        map_cb.current(0)
        map_cb.pack(fill=tk.X, pady=4)
        map_cb.bind("<<ComboboxSelected>>", self._on_map_select)

        # Separator
        tk.Frame(right, bg=ACCENT, height=2).pack(fill=tk.X, padx=8, pady=4)

        # Status
        s = tk.Frame(right, bg=BG_PANEL)
        s.pack(fill=tk.X, padx=8, pady=4)
        tk.Label(s, text="Robot Status", font=label_font, fg=ACCENT, bg=BG_PANEL).pack(anchor="w")
        self.status_lbl = tk.Label(s, text="X:0.00  Y:0.00  θ:0°", font=info_font, fg=GREEN, bg="#0a0a1a", anchor="w", padx=6, pady=3)
        self.status_lbl.pack(fill=tk.X, pady=1)
        self.scan_lbl = tk.Label(s, text="LiDAR: --", font=info_font, fg=CYAN, bg="#0a0a1a", anchor="w", padx=6, pady=3)
        self.scan_lbl.pack(fill=tk.X, pady=1)

        # Separator
        tk.Frame(right, bg=ACCENT, height=2).pack(fill=tk.X, padx=8, pady=4)

        # Speed sliders
        sp = tk.Frame(right, bg=BG_PANEL)
        sp.pack(fill=tk.X, padx=8, pady=2)
        tk.Label(sp, text="Speed Control", font=label_font, fg=ACCENT, bg=BG_PANEL).pack(anchor="w")

        for (text, attr, frm, to, init) in [("Linear:", "linear_speed",  0.1, 1.5, 0.3), ("Angular:", "angular_speed", 0.5, 3.0, 1.0)]:
            row = tk.Frame(sp, bg=BG_PANEL)
            row.pack(fill=tk.X, pady=2)
            tk.Label(row, text=text, font=label_font, fg="#aaa", bg=BG_PANEL, width=8).pack(side=tk.LEFT)
            sldr = tk.Scale(row, from_=frm, to=to, resolution=0.1, orient=tk.HORIZONTAL, bg=BG_PANEL, fg=WHITE, troughcolor=ACCENT2, highlightthickness=0, command=lambda v, a=attr: setattr(self, a, float(v)))
            sldr.set(init)
            sldr.pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Direction buttons
        tk.Label(right, text="Remote Control", font=label_font, fg=ACCENT, bg=BG_PANEL).pack(pady=(10,0))
        btn_box = tk.Frame(right, bg=BG_PANEL)
        btn_box.pack(expand=True, fill=tk.BOTH, padx=8, pady=4)

        bkw = dict(font=btn_font, width=6, height=2, bd=0, relief=tk.FLAT, cursor="hand2", activeforeground=WHITE)
        self.btn_fwd = tk.Button(btn_box, text="▲\nFwd",   bg=ACCENT2, fg=WHITE, activebackground="#1a5276", **bkw)
        self.btn_lft = tk.Button(btn_box, text="◄\nLeft",  bg=ACCENT2, fg=WHITE, activebackground="#1a5276", **bkw)
        self.btn_stp = tk.Button(btn_box, text="■\nSTOP",  bg="#8b0000", fg=WHITE, activebackground=ACCENT,  **bkw)
        self.btn_rgt = tk.Button(btn_box, text="►\nRight", bg=ACCENT2, fg=WHITE, activebackground="#1a5276", **bkw)
        self.btn_rev = tk.Button(btn_box, text="▼\nRev",   bg=ACCENT2, fg=WHITE, activebackground="#1a5276", **bkw)

        self.btn_fwd.grid(row=0, column=1, padx=2, pady=2, sticky="nsew")
        self.btn_lft.grid(row=1, column=0, padx=2, pady=2, sticky="nsew")
        self.btn_stp.grid(row=1, column=1, padx=2, pady=2, sticky="nsew")
        self.btn_rgt.grid(row=1, column=2, padx=2, pady=2, sticky="nsew")
        self.btn_rev.grid(row=2, column=1, padx=2, pady=2, sticky="nsew")
        for i in range(3): btn_box.columnconfigure(i, weight=1); btn_box.rowconfigure(i, weight=1)

        # Bindings
        self.btn_fwd.bind("<ButtonPress-1>", lambda e: self._vel(1,  0)); self.btn_fwd.bind("<ButtonRelease-1>", lambda e: self._vel(0,  0))
        self.btn_rev.bind("<ButtonPress-1>", lambda e: self._vel(-1, 0)); self.btn_rev.bind("<ButtonRelease-1>", lambda e: self._vel(0,  0))
        self.btn_lft.bind("<ButtonPress-1>", lambda e: self._vel(0,  1)); self.btn_lft.bind("<ButtonRelease-1>", lambda e: self._vel(0,  0))
        self.btn_rgt.bind("<ButtonPress-1>", lambda e: self._vel(0, -1)); self.btn_rgt.bind("<ButtonRelease-1>", lambda e: self._vel(0,  0))
        self.btn_stp.bind("<ButtonPress-1>", lambda e: self._vel(0,  0))

        for key, lin, ang in [("w", 1, 0), ("s", -1, 0), ("a", 0, 1), ("d", 0, -1), ("Up", 1, 0), ("Down", -1, 0), ("Left", 0, 1), ("Right", 0, -1)]:
            self.root.bind(f"<KeyPress-{key}>",   lambda e, l=lin, ag=ang: self._vel(l, ag))
            self.root.bind(f"<KeyRelease-{key}>", lambda e: self._vel(0, 0))
        self.root.bind("<space>", lambda e: self._vel(0, 0))

    # ─────────────────────────────────────────────────────────────────────────
    # Map Spawner Logic
    # ─────────────────────────────────────────────────────────────────────────
    def _on_map_select(self, event):
        val = self.map_var.get()
        if "Select" in val: return
        threading.Thread(target=self._switch_map, args=(val,), daemon=True).start()

    def _switch_map(self, map_name):
        # 1. Clear old obstacles
        for obs in self.current_obstacles:
            req = DeleteEntity.Request()
            req.name = obs
            self.delete_cli.call(req)
        self.current_obstacles.clear()
        
        from std_msgs.msg import String
        import json
        obs_pub = self.create_publisher(String, "obstacles_json", 10)
        obs_pub.publish(String(data="[]"))

        # 2. Reset SLAM map
        if self.slam_reset_cli.wait_for_service(timeout_sec=1.0):
            self.slam_reset_cli.call(Clear.Request())

        if "REALTIME MAP" in map_name:
            self.root.after(0, lambda: messagebox.showinfo("LiDAR Map", "LiDAR Detected!\nCommencing Custom REALTIME Mapping..."))
            return

        # 3. Spawn new obstacles
        new_obs = []
        if "Map 1" in map_name:
            new_obs = [
                {"name": "obs_1", "shape": "cylinder", "r": 0.4, "sx": 1, "sy": 1, "sz": 2.0, "x": -2.0, "y": 1.0, "yaw": 0.0},
                {"name": "obs_2", "shape": "cylinder", "r": 0.6, "sx": 1, "sy": 1, "sz": 2.0, "x": -1.5, "y": -0.8, "yaw": 0.0},
                {"name": "obs_3", "shape": "cylinder", "r": 0.35, "sx": 1, "sy": 1, "sz": 2.0, "x": 0.5, "y": 0.3, "yaw": 0.0},
                {"name": "obs_4", "shape": "cylinder", "r": 0.4, "sx": 1, "sy": 1, "sz": 2.0, "x": 0.6, "y": -1.0, "yaw": 0.0},
                {"name": "obs_5", "shape": "cylinder", "r": 0.25, "sx": 1, "sy": 1, "sz": 2.0, "x": 2.5, "y": 1.0, "yaw": 0.0}
            ]
        elif "Map 2" in map_name:
            new_obs = [
                {"name": "obs_c1", "shape": "cylinder", "r": 0.2, "sx": 1, "sy": 1, "sz": 2.0, "x": 1.0, "y": 0.6, "yaw": 0.0},
                {"name": "obs_c2", "shape": "cylinder", "r": 0.2, "sx": 1, "sy": 1, "sz": 2.0, "x": 1.3, "y": 0.2, "yaw": 0.0},
                {"name": "obs_c3", "shape": "cylinder", "r": 0.2, "sx": 1, "sy": 1, "sz": 2.0, "x": 1.3, "y": -0.2, "yaw": 0.0},
                {"name": "obs_c4", "shape": "cylinder", "r": 0.2, "sx": 1, "sy": 1, "sz": 2.0, "x": 1.0, "y": -0.6, "yaw": 0.0},
                {"name": "obs_o1", "shape": "cylinder", "r": 0.4, "sx": 1, "sy": 1, "sz": 2.0, "x": -1.5, "y": 0.2, "yaw": 0.0},
                {"name": "obs_o2", "shape": "cylinder", "r": 0.25, "sx": 1, "sy": 1, "sz": 2.0, "x": -0.5, "y": -0.8, "yaw": 0.0},
                {"name": "obs_o3", "shape": "cylinder", "r": 0.3, "sx": 1, "sy": 1, "sz": 2.0, "x": 2.5, "y": 0.8, "yaw": 0.0},
            ]
        elif "Map 3" in map_name:
            new_obs = [
                {"name": "obs_sq", "shape": "box", "r": 0, "sx": 1.5, "sy": 0.8, "sz": 2.0, "x": 1.5, "y": -0.5, "yaw": 0.0},
                {"name": "obs_ci", "shape": "cylinder", "r": 0.5, "sx": 1, "sy": 1, "sz": 2.0, "x": 0.0, "y": 0.0, "yaw": 0.0},
                {"name": "obs_tr", "shape": "box", "r": 0, "sx": 0.8, "sy": 0.8, "sz": 2.0, "x": -2.0, "y": 0.5, "yaw": 0.785},
                {"name": "obs_co", "shape": "cylinder", "r": 0.6, "sx": 1, "sy": 1, "sz": 2.0, "x": 3.5, "y": 1.5, "yaw": 0.0},
            ]

        for obs in new_obs:
            req = SpawnEntity.Request()
            req.name = obs["name"]
            req.xml = generate_sdf(obs["shape"], r=obs["r"], sx=obs["sx"], sy=obs["sy"], sz=obs["sz"])
            req.initial_pose.position.x = float(obs["x"])
            req.initial_pose.position.y = float(obs["y"])
            req.initial_pose.position.z = obs["sz"] / 2.0
            
            req.initial_pose.orientation.z = math.sin(obs["yaw"]/2.0)
            req.initial_pose.orientation.w = math.cos(obs["yaw"]/2.0)
            
            self.spawn_cli.call(req)
            self.current_obstacles.append(obs["name"])

        # Broadcast the obstacles to 3D UI
        obs_pub.publish(String(data=json.dumps(new_obs)))


    # ─────────────────────────────────────────────────────────────────────────
    # ROS callbacks
    # ─────────────────────────────────────────────────────────────────────────
    def _on_scan(self, msg: LaserScan): self.latest_scan = msg
    def _on_odom(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    def _on_map(self, msg: OccupancyGrid): self.occ_grid = msg

    def _vel(self, lin: int, ang: int):
        msg = Twist()
        msg.linear.x  = float(lin) * self.linear_speed
        msg.angular.z = float(ang) * self.angular_speed
        self.cmd_vel_pub.publish(msg)

    # ─────────────────────────────────────────────────────────────────────────
    # Rendering
    # ─────────────────────────────────────────────────────────────────────────
    def _render(self):
        self._draw_main_canvas()
        self._update_labels()
        self.root.after(50, self._render)

    def _draw_main_canvas(self):
        c = self.canvas
        c.delete("all")
        w, h = c.winfo_width(), c.winfo_height()
        if w < 10 or h < 10: return
        cx, cy = w / 2.0, h / 2.0
        ppm = min(w, h) / 10.0

        grid = self.occ_grid
        if grid is not None:
            res, gw, gh = grid.info.resolution, grid.info.width, grid.info.height
            ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
            cell_px = max(1, int(res * ppm))
            for row in range(gh):
                for col in range(gw):
                    val = grid.data[row * gw + col]
                    if val == -1: colour = "#0a0a20"
                    elif val == 0: colour = "#1c2a3a"
                    elif val > 50: colour = "#e0e0e8"
                    else: continue
                    wx, wy = ox + (col + 0.5) * res, oy + (row + 0.5) * res
                    sx, sy = cx + (wx - self.robot_x) * ppm, cy - (wy - self.robot_y) * ppm
                    if -cell_px < sx < w + cell_px and -cell_px < sy < h + cell_px:
                        c.create_rectangle(sx, sy, sx + cell_px, sy + cell_px, fill=colour, outline="")

        for ring_m in [1, 2, 3, 4, 5]:
            r_px = ring_m * ppm
            c.create_oval(cx - r_px, cy - r_px, cx + r_px, cy + r_px, outline="#111830", width=1)

        scan = self.latest_scan
        valid_pts = 0
        if scan is not None:
            angle = scan.angle_min
            for r in scan.ranges:
                if scan.range_min < r < scan.range_max:
                    ray_angle = self.robot_yaw + angle
                    ex, ey = cx + r * math.cos(ray_angle) * ppm, cy - r * math.sin(ray_angle) * ppm
                    ratio = min(r / scan.range_max, 1.0)
                    colour = f"#{int(255 * (1.0 - ratio * 0.6)):02x}{int(80 * ratio):02x}{int(255 * ratio):02x}"
                    c.create_line(cx, cy, ex, ey, fill="#1a2a1a", width=1)
                    c.create_oval(ex - 2, ey - 2, ex + 2, ey + 2, fill=colour, outline="")
                    valid_pts += 1
                angle += scan.angle_increment

        sz, yaw = 10, self.robot_yaw
        c.create_polygon([
            (cx + sz * math.cos(yaw), cy - sz * math.sin(yaw)),
            (cx + sz * 0.6 * math.cos(yaw + 2.4), cy - sz * 0.6 * math.sin(yaw + 2.4)),
            (cx + sz * 0.6 * math.cos(yaw - 2.4), cy - sz * 0.6 * math.sin(yaw - 2.4)),
        ], fill=GREEN, outline="#009944", width=2)
        
        self._draw_minimap(c, w, h, valid_pts)

    def _draw_minimap(self, c, W, H, valid_pts):
        MM_W, MM_H, PAD = 220, 150, 10
        mx, my = W - MM_W - PAD, PAD
        c.create_rectangle(mx - 2, my - 2, mx + MM_W + 2, my + MM_H + 2, fill="#050510", outline=ACCENT, width=2)
        c.create_text(mx + MM_W // 2, my + 8, text="Full Map", fill=ACCENT, font=("Courier", 8, "bold"))

        grid = self.occ_grid
        if grid is None or grid.info.width == 0: return
        gw, gh, res = grid.info.width, grid.info.height, grid.info.resolution
        scale_x, scale_y = MM_W / gw, (MM_H - 16) / gh
        cell_px = max(1, int(min(scale_x, scale_y)))
        mm_ox, mm_oy = mx, my + 14

        for row in range(gh):
            for col in range(gw):
                val = grid.data[row * gw + col]
                if val == -1: colour = "#0a0a18"
                elif val == 0: colour = "#203040"
                elif val > 50: colour = "#c8c8d8"
                else: continue
                sx, sy = mm_ox + int(col * scale_x), mm_oy + int((gh - 1 - row) * scale_y)
                if cell_px >= 2: c.create_rectangle(sx, sy, sx + cell_px, sy + cell_px, fill=colour, outline="")
                else: c.create_rectangle(sx, sy, sx + 1, sy + 1, fill=colour, outline="")

        rx, ry = mm_ox + int((self.robot_x - grid.info.origin.position.x) / res * scale_x), mm_oy + int((gh - 1 - (self.robot_y - grid.info.origin.position.y) / res) * scale_y)
        c.create_oval(rx - 4, ry - 4, rx + 4, ry + 4, fill=GREEN, outline="#009944", width=1)

    def _update_labels(self):
        self.status_lbl.config(text=f"X:{self.robot_x:.2f}  Y:{self.robot_y:.2f}  θ:{math.degrees(self.robot_yaw):.1f}°")
        scan = self.latest_scan
        self.scan_lbl.config(text=f"LiDAR: {sum(1 for r in scan.ranges if scan.range_min < r < scan.range_max)}/{len(scan.ranges)} pts" if scan else "LiDAR: Waiting…")

    def run(self):
        threading.Thread(target=rclpy.spin, args=(self,), daemon=True).start()
        try: self.root.mainloop()
        except KeyboardInterrupt: pass
        finally: self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AMRGuiNode()
    node.run()
    if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__": main()
