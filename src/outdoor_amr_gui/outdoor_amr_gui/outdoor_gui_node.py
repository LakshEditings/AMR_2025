#!/usr/bin/env python3
"""
Outdoor AMR GUI Node — Window 4
================================
2D Top-down view centred on robot:
  - GPS coordinates displayed
  - 2D LiDAR rays projected
  - Mini-map in top-right (GPS trail)
  - WASD / Arrow keys + on-screen buttons for outdoor/cmd_vel
"""

import math
import threading
import tkinter as tk
from tkinter import font as tkfont
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, NavSatFix
from nav_msgs.msg import Odometry

# ── Colour palette (same identity as indoor) ───────────────────────────────
BG_DARK   = "#0d0d1a"
BG_PANEL  = "#16213e"
BG_CANVAS = "#0a1a0a"     # dark green tint for outdoor
ACCENT    = "#00e676"     # green accent (outdoor theme)
ACCENT2   = "#0d3b1e"
GREEN     = "#00ff88"
CYAN      = "#00ccff"
WHITE     = "#ffffff"
ORANGE    = "#ff9800"


class OutdoorGuiNode(Node):
    def __init__(self):
        super().__init__('outdoor_gui_node')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'outdoor/cmd_vel', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, 'scan',           self._on_scan, 10)
        self.odom_sub = self.create_subscription(Odometry,  'outdoor/odom',   self._on_odom, 10)
        self.gps_sub  = self.create_subscription(NavSatFix, 'gps/fix',        self._on_gps,  10)

        # State
        self.latest_scan  = None
        self.robot_x      = 0.0
        self.robot_y      = 0.0
        self.robot_yaw    = 0.0
        self.gps_lat      = None
        self.gps_lon      = None
        self.gps_alt      = None
        self.gps_fix      = False
        self.gps_trail    = []          # list of (lat, lon)
        self.linear_speed = 0.5
        self.angular_speed = 1.2

        # GUI
        self.root = tk.Tk()
        self.root.title("Outdoor AMR — GPS Navigation View")
        self.root.configure(bg=BG_DARK)
        self.root.geometry("1200x750")
        self.root.minsize(900, 600)
        self._build_gui()
        self.root.after(50, self._render)

    # ─────────────────────────────────────────────────────────────────────────
    def _build_gui(self):
        title_font = tkfont.Font(family="Helvetica", size=14, weight="bold")
        btn_font   = tkfont.Font(family="Helvetica", size=11, weight="bold")
        lbl_font   = tkfont.Font(family="Helvetica", size=9)
        info_font  = tkfont.Font(family="Courier",   size=9)

        # Title bar
        bar = tk.Frame(self.root, bg="#050f05", pady=6)
        bar.pack(fill=tk.X)
        tk.Label(bar, text="🛰  Outdoor AMR — GPS Navigation View",
                 font=title_font, fg=ACCENT, bg="#050f05").pack()

        main = tk.Frame(self.root, bg=BG_DARK)
        main.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        main.columnconfigure(0, weight=4)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        # Canvas
        left = tk.Frame(main, bg=ACCENT2, bd=2, relief=tk.RIDGE)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 3))
        self.canvas = tk.Canvas(left, bg=BG_CANVAS, highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Side panel
        right = tk.Frame(main, bg=BG_PANEL, bd=2, relief=tk.RIDGE)
        right.pack_propagate(False)
        right.grid(row=0, column=1, sticky="nsew")

        # GPS Status
        g = tk.Frame(right, bg=BG_PANEL)
        g.pack(fill=tk.X, padx=8, pady=(8, 4))
        tk.Label(g, text="GPS Signal", font=lbl_font, fg=ACCENT, bg=BG_PANEL).pack(anchor="w")
        self.gps_fix_lbl = tk.Label(g, text="NO FIX", font=info_font, fg="#ff4444",
                                    bg="#0a0a1a", anchor="w", padx=6, pady=3)
        self.gps_fix_lbl.pack(fill=tk.X, pady=1)
        self.gps_coord_lbl = tk.Label(g, text="Lat: --\nLon: --\nAlt: --",
                                      font=info_font, fg=CYAN, bg="#0a0a1a",
                                      anchor="w", padx=6, pady=3, justify=tk.LEFT)
        self.gps_coord_lbl.pack(fill=tk.X, pady=1)

        tk.Frame(right, bg=ACCENT, height=2).pack(fill=tk.X, padx=8, pady=4)

        # Robot odometry status
        s = tk.Frame(right, bg=BG_PANEL)
        s.pack(fill=tk.X, padx=8, pady=4)
        tk.Label(s, text="Dead-Reckoning", font=lbl_font, fg=ACCENT, bg=BG_PANEL).pack(anchor="w")
        self.status_lbl = tk.Label(s, text="X:0.00  Y:0.00  θ:0°",
                                   font=info_font, fg=GREEN, bg="#0a0a1a",
                                   anchor="w", padx=6, pady=3)
        self.status_lbl.pack(fill=tk.X, pady=1)
        self.scan_lbl = tk.Label(s, text="3D LiDAR: --",
                                 font=info_font, fg=ORANGE, bg="#0a0a1a",
                                 anchor="w", padx=6, pady=3)
        self.scan_lbl.pack(fill=tk.X, pady=1)

        tk.Frame(right, bg=ACCENT, height=2).pack(fill=tk.X, padx=8, pady=4)

        # Speed sliders
        sp = tk.Frame(right, bg=BG_PANEL)
        sp.pack(fill=tk.X, padx=8, pady=2)
        tk.Label(sp, text="Speed Control", font=lbl_font, fg=ACCENT, bg=BG_PANEL).pack(anchor="w")
        for text, attr, frm, to, init in [
            ("Linear:",  "linear_speed",  0.1, 2.0, 0.5),
            ("Angular:", "angular_speed", 0.5, 3.0, 1.2)
        ]:
            row = tk.Frame(sp, bg=BG_PANEL)
            row.pack(fill=tk.X, pady=2)
            tk.Label(row, text=text, font=lbl_font, fg="#aaa", bg=BG_PANEL, width=8).pack(side=tk.LEFT)
            s2 = tk.Scale(row, from_=frm, to=to, resolution=0.1, orient=tk.HORIZONTAL,
                          bg=BG_PANEL, fg=WHITE, troughcolor=ACCENT2, highlightthickness=0,
                          command=lambda v, a=attr: setattr(self, a, float(v)))
            s2.set(init)
            s2.pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Direction buttons
        tk.Label(right, text="Remote Control", font=lbl_font, fg=ACCENT, bg=BG_PANEL).pack(pady=(10, 0))
        btn_box = tk.Frame(right, bg=BG_PANEL)
        btn_box.pack(expand=True, fill=tk.BOTH, padx=8, pady=4)
        bkw = dict(font=btn_font, width=6, height=2, bd=0, relief=tk.FLAT,
                   cursor="hand2", activeforeground=WHITE)
        self.btn_fwd = tk.Button(btn_box, text="▲\nFwd",   bg=ACCENT2, fg=WHITE, activebackground="#1a5276", **bkw)
        self.btn_lft = tk.Button(btn_box, text="◄\nLeft",  bg=ACCENT2, fg=WHITE, activebackground="#1a5276", **bkw)
        self.btn_stp = tk.Button(btn_box, text="■\nSTOP",  bg="#8b0000", fg=WHITE, activebackground=ACCENT,   **bkw)
        self.btn_rgt = tk.Button(btn_box, text="►\nRight", bg=ACCENT2, fg=WHITE, activebackground="#1a5276", **bkw)
        self.btn_rev = tk.Button(btn_box, text="▼\nRev",   bg=ACCENT2, fg=WHITE, activebackground="#1a5276", **bkw)
        self.btn_fwd.grid(row=0, column=1, padx=2, pady=2, sticky="nsew")
        self.btn_lft.grid(row=1, column=0, padx=2, pady=2, sticky="nsew")
        self.btn_stp.grid(row=1, column=1, padx=2, pady=2, sticky="nsew")
        self.btn_rgt.grid(row=1, column=2, padx=2, pady=2, sticky="nsew")
        self.btn_rev.grid(row=2, column=1, padx=2, pady=2, sticky="nsew")
        for i in range(3):
            btn_box.columnconfigure(i, weight=1)
            btn_box.rowconfigure(i, weight=1)

        self.btn_fwd.bind("<ButtonPress-1>",   lambda e: self._vel(1,  0))
        self.btn_fwd.bind("<ButtonRelease-1>", lambda e: self._vel(0,  0))
        self.btn_rev.bind("<ButtonPress-1>",   lambda e: self._vel(-1, 0))
        self.btn_rev.bind("<ButtonRelease-1>", lambda e: self._vel(0,  0))
        self.btn_lft.bind("<ButtonPress-1>",   lambda e: self._vel(0,  1))
        self.btn_lft.bind("<ButtonRelease-1>", lambda e: self._vel(0,  0))
        self.btn_rgt.bind("<ButtonPress-1>",   lambda e: self._vel(0, -1))
        self.btn_rgt.bind("<ButtonRelease-1>", lambda e: self._vel(0,  0))
        self.btn_stp.bind("<ButtonPress-1>",   lambda e: self._vel(0,  0))

        for key, lin, ang in [("w",1,0),("s",-1,0),("a",0,1),("d",0,-1),
                               ("Up",1,0),("Down",-1,0),("Left",0,1),("Right",0,-1)]:
            self.root.bind(f"<KeyPress-{key}>",   lambda e, l=lin, ag=ang: self._vel(l, ag))
            self.root.bind(f"<KeyRelease-{key}>", lambda e: self._vel(0, 0))
        self.root.bind("<space>", lambda e: self._vel(0, 0))

    # ── ROS callbacks ────────────────────────────────────────────────────────
    def _on_scan(self, msg): self.latest_scan = msg
    def _on_odom(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y),
                                    1.0 - 2.0*(q.y*q.y + q.z*q.z))
    def _on_gps(self, msg: NavSatFix):
        self.gps_lat = msg.latitude
        self.gps_lon = msg.longitude
        self.gps_alt = msg.altitude
        self.gps_fix = msg.status.status >= 0
        if self.gps_fix:
            self.gps_trail.append((msg.latitude, msg.longitude))
            if len(self.gps_trail) > 2000:
                self.gps_trail.pop(0)

    def _vel(self, lin, ang):
        msg = Twist()
        msg.linear.x  = float(lin) * self.linear_speed
        msg.angular.z = float(ang) * self.angular_speed
        self.cmd_vel_pub.publish(msg)

    # ── Rendering ────────────────────────────────────────────────────────────
    def _render(self):
        self._draw_canvas()
        self._update_labels()
        self.root.after(50, self._render)

    def _draw_canvas(self):
        c = self.canvas
        c.delete("all")
        w, h = c.winfo_width(), c.winfo_height()
        if w < 10 or h < 10: return
        cx, cy = w/2.0, h/2.0
        ppm = min(w, h) / 40.0      # 40m across at nominal zoom

        # Grid (range rings)
        for ring_m in [5, 10, 15, 20]:
            r_px = ring_m * ppm
            c.create_oval(cx-r_px, cy-r_px, cx+r_px, cy+r_px, outline="#112211", width=1)
            c.create_text(cx+r_px+4, cy, text=f"{ring_m}m", fill="#334433",
                          font=("Courier", 7))

        # Crosshair
        c.create_line(cx-15, cy, cx+15, cy, fill="#1a3a1a", width=1)
        c.create_line(cx, cy-15, cx, cy+15, fill="#1a3a1a", width=1)

        # GPS trail
        if len(self.gps_trail) > 1 and self.gps_lat:
            # Convert GPS deltas to metres (~111320 m/deg lat, ~111320*cos(lat) m/deg lon)
            ref_lat, ref_lon = self.gps_trail[0]
            pts = []
            for lat, lon in self.gps_trail:
                dy = (lat - ref_lat) * 111320.0
                dx = (lon - ref_lon) * 111320.0 * math.cos(math.radians(ref_lat))
                sx = cx + dx * ppm
                sy = cy - dy * ppm
                pts.append((sx, sy))
            for i in range(len(pts)-1):
                c.create_line(pts[i][0], pts[i][1], pts[i+1][0], pts[i+1][1],
                               fill="#00aa44", width=2)

        # LiDAR rays
        scan = self.latest_scan
        valid_pts = 0
        if scan:
            angle = scan.angle_min
            for r in scan.ranges:
                if scan.range_min < r < scan.range_max:
                    ray_angle = self.robot_yaw + angle
                    ex = cx + r * math.cos(ray_angle) * ppm
                    ey = cy - r * math.sin(ray_angle) * ppm
                    ratio = min(r/scan.range_max, 1.0)
                    col = f"#{int(255*(1-ratio*0.5)):02x}{int(150*ratio):02x}{int(80):02x}"
                    c.create_line(cx, cy, ex, ey, fill="#112211", width=1)
                    c.create_oval(ex-2, ey-2, ex+2, ey+2, fill=col, outline="")
                    valid_pts += 1
                angle += scan.angle_increment

        # Robot body
        sz, yaw = 12, self.robot_yaw
        c.create_polygon([
            (cx + sz*math.cos(yaw),         cy - sz*math.sin(yaw)),
            (cx + sz*0.6*math.cos(yaw+2.4), cy - sz*0.6*math.sin(yaw+2.4)),
            (cx + sz*0.6*math.cos(yaw-2.4), cy - sz*0.6*math.sin(yaw-2.4)),
        ], fill=ACCENT, outline="#009944", width=2)

        self._draw_gps_minimap(c, w, h)

    def _draw_gps_minimap(self, c, W, H):
        """Top-right GPS trail overview minimap."""
        MM_W, MM_H, PAD = 220, 150, 10
        mx, my = W-MM_W-PAD, PAD
        c.create_rectangle(mx-2, my-2, mx+MM_W+2, my+MM_H+2,
                            fill="#050f05", outline=ACCENT, width=2)
        c.create_text(mx+MM_W//2, my+9, text="GPS Trail Map",
                      fill=ACCENT, font=("Courier", 8, "bold"))

        if len(self.gps_trail) < 2: return
        lats = [p[0] for p in self.gps_trail]
        lons = [p[1] for p in self.gps_trail]
        lat_min, lat_max = min(lats), max(lats)
        lon_min, lon_max = min(lons), max(lons)
        span = max(lat_max-lat_min, lon_max-lon_min, 1e-5)

        pts = []
        for lat, lon in self.gps_trail:
            px = mx + int((lon-lon_min)/span * (MM_W-10)) + 5
            py = my+MM_H - int((lat-lat_min)/span * (MM_H-20)) - 5
            pts.append((px, py))

        for i in range(len(pts)-1):
            c.create_line(pts[i][0], pts[i][1], pts[i+1][0], pts[i+1][1],
                           fill=ACCENT, width=2)

        # Current position dot
        if pts:
            x, y = pts[-1]
            c.create_oval(x-5, y-5, x+5, y+5, fill=GREEN, outline="#009944")

    def _update_labels(self):
        if self.gps_fix:
            self.gps_fix_lbl.config(text="FIX ACQUIRED ✓", fg=GREEN)
            self.gps_coord_lbl.config(
                text=f"Lat: {self.gps_lat:.6f}°\n"
                     f"Lon: {self.gps_lon:.6f}°\n"
                     f"Alt: {self.gps_alt:.1f} m")
        else:
            self.gps_fix_lbl.config(text="NO FIX — Waiting…", fg="#ff4444")
            self.gps_coord_lbl.config(text="Lat: --\nLon: --\nAlt: --")

        self.status_lbl.config(
            text=f"X:{self.robot_x:.2f}  Y:{self.robot_y:.2f}  "
                 f"θ:{math.degrees(self.robot_yaw):.1f}°")
        scan = self.latest_scan
        if scan:
            pts = sum(1 for r in scan.ranges if scan.range_min < r < scan.range_max)
            self.scan_lbl.config(text=f"3D LiDAR: {pts}/{len(scan.ranges)} pts")
        else:
            self.scan_lbl.config(text="3D LiDAR: Waiting…")

    def run(self):
        threading.Thread(target=rclpy.spin, args=(self,), daemon=True).start()
        try: self.root.mainloop()
        except KeyboardInterrupt: pass
        finally: self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OutdoorGuiNode()
    node.run()
    if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
