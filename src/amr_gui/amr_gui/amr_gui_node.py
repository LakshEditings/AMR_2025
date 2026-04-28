#!/usr/bin/env python3
"""
AMR GUI Node - Window 2
======================
Main View  : 2D SLAM map centred on robot with live LiDAR rays
Minimap    : Top-right corner — full accumulated map overview
Side Panel : Remote Control buttons + robot status + speed sliders
"""

import math
import threading
import tkinter as tk
from tkinter import font as tkfont

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid


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


class AMRGuiNode(Node):
    """ROS2 node — 2-D SLAM viewer (with minimap) + remote control."""

    def __init__(self):
        super().__init__("amr_gui_node")

        # ── Publishers ───────────────────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # ── QoS for map (latched) ────────────────────────────────────────────
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # ── Subscribers ──────────────────────────────────────────────────────
        self.scan_sub = self.create_subscription(
            LaserScan, "scan", self._on_scan, 10)
        self.odom_sub = self.create_subscription(
            Odometry, "odom", self._on_odom, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, "map", self._on_map, map_qos)

        # ── State ────────────────────────────────────────────────────────────
        self.latest_scan: LaserScan | None = None
        self.robot_x    = 0.0
        self.robot_y    = 0.0
        self.robot_yaw  = 0.0
        self.occ_grid: OccupancyGrid | None = None  # latest full map
        self.linear_speed  = 0.3
        self.angular_speed = 1.0

        # Minimap: PIL-free cached photo-image built from OccupancyGrid
        self._map_image: tk.PhotoImage | None = None
        self._map_dirty = False  # flag to rebuild map image

        # ── Build GUI ────────────────────────────────────────────────────────
        self.root = tk.Tk()
        self.root.title("AMR Navigator — 2D Live View")
        self.root.configure(bg=BG_DARK)
        self.root.geometry("1100x700")
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
        tk.Label(
            title_bar, text="⬡ AMR Navigator — Live 2D View",
            font=title_font, fg=ACCENT, bg="#0a0a18"
        ).pack()

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

        # Status
        s = tk.Frame(right, bg=BG_PANEL)
        s.pack(fill=tk.X, padx=8, pady=(8, 4))
        tk.Label(s, text="Robot Status", font=label_font, fg=ACCENT, bg=BG_PANEL).pack(anchor="w")
        self.status_lbl = tk.Label(
            s, text="X:0.00  Y:0.00  θ:0°", font=info_font,
            fg=GREEN, bg="#0a0a1a", anchor="w", padx=6, pady=3
        )
        self.status_lbl.pack(fill=tk.X, pady=1)
        self.scan_lbl = tk.Label(
            s, text="LiDAR: --", font=info_font,
            fg=CYAN, bg="#0a0a1a", anchor="w", padx=6, pady=3
        )
        self.scan_lbl.pack(fill=tk.X, pady=1)

        # Separator
        tk.Frame(right, bg=ACCENT, height=2).pack(fill=tk.X, padx=8, pady=6)

        # Speed sliders
        sp = tk.Frame(right, bg=BG_PANEL)
        sp.pack(fill=tk.X, padx=8, pady=2)
        tk.Label(sp, text="Speed Control", font=label_font, fg=ACCENT, bg=BG_PANEL).pack(anchor="w")

        for (text, attr, frm, to, init) in [
            ("Linear:", "linear_speed",  0.1, 1.5, 0.3),
            ("Angular:", "angular_speed", 0.5, 3.0, 1.0),
        ]:
            row = tk.Frame(sp, bg=BG_PANEL)
            row.pack(fill=tk.X, pady=2)
            tk.Label(row, text=text, font=label_font, fg="#aaa", bg=BG_PANEL, width=8).pack(side=tk.LEFT)
            sldr = tk.Scale(
                row, from_=frm, to=to, resolution=0.1, orient=tk.HORIZONTAL,
                bg=BG_PANEL, fg=WHITE, troughcolor=ACCENT2, highlightthickness=0,
                command=lambda v, a=attr: setattr(self, a, float(v))
            )
            sldr.set(init)
            sldr.pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Separator
        tk.Frame(right, bg=ACCENT, height=2).pack(fill=tk.X, padx=8, pady=6)

        # Direction buttons
        tk.Label(right, text="Remote Control", font=label_font, fg=ACCENT, bg=BG_PANEL).pack()
        btn_box = tk.Frame(right, bg=BG_PANEL)
        btn_box.pack(expand=True, fill=tk.BOTH, padx=8, pady=4)

        bkw = dict(font=btn_font, width=6, height=2, bd=0, relief=tk.FLAT,
                   cursor="hand2", activeforeground=WHITE)

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
        for i in range(3):
            btn_box.columnconfigure(i, weight=1)
            btn_box.rowconfigure(i, weight=1)

        # Button bindings
        self.btn_fwd.bind("<ButtonPress-1>",   lambda e: self._vel(1,  0))
        self.btn_fwd.bind("<ButtonRelease-1>", lambda e: self._vel(0,  0))
        self.btn_rev.bind("<ButtonPress-1>",   lambda e: self._vel(-1, 0))
        self.btn_rev.bind("<ButtonRelease-1>", lambda e: self._vel(0,  0))
        self.btn_lft.bind("<ButtonPress-1>",   lambda e: self._vel(0,  1))
        self.btn_lft.bind("<ButtonRelease-1>", lambda e: self._vel(0,  0))
        self.btn_rgt.bind("<ButtonPress-1>",   lambda e: self._vel(0, -1))
        self.btn_rgt.bind("<ButtonRelease-1>", lambda e: self._vel(0,  0))
        self.btn_stp.bind("<ButtonPress-1>",   lambda e: self._vel(0,  0))

        # Keyboard bindings
        for key, lin, ang in [
            ("w", 1, 0), ("s", -1, 0), ("a", 0, 1), ("d", 0, -1),
            ("Up", 1, 0), ("Down", -1, 0), ("Left", 0, 1), ("Right", 0, -1),
        ]:
            self.root.bind(f"<KeyPress-{key}>",   lambda e, l=lin, ag=ang: self._vel(l, ag))
            self.root.bind(f"<KeyRelease-{key}>", lambda e: self._vel(0, 0))
        self.root.bind("<space>", lambda e: self._vel(0, 0))

    # ─────────────────────────────────────────────────────────────────────────
    # ROS callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _on_scan(self, msg: LaserScan):
        self.latest_scan = msg

    def _on_odom(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def _on_map(self, msg: OccupancyGrid):
        self.occ_grid = msg
        self._map_dirty = True  # rebuild minimap image next frame

    # ─────────────────────────────────────────────────────────────────────────
    # Velocity helper
    # ─────────────────────────────────────────────────────────────────────────

    def _vel(self, lin: int, ang: int):
        msg = Twist()
        msg.linear.x  = float(lin) * self.linear_speed
        msg.angular.z = float(ang) * self.angular_speed
        self.cmd_vel_pub.publish(msg)

    # ─────────────────────────────────────────────────────────────────────────
    # Main render loop (50 ms ≈ 20 fps)
    # ─────────────────────────────────────────────────────────────────────────

    def _render(self):
        self._draw_main_canvas()
        self._update_labels()
        self.root.after(50, self._render)

    # ─────────────────────────────────────────────────────────────────────────
    # Main canvas: SLAM map centred on robot + LiDAR rays
    # ─────────────────────────────────────────────────────────────────────────

    def _draw_main_canvas(self):
        c = self.canvas
        c.delete("all")

        w = c.winfo_width()
        h = c.winfo_height()
        if w < 10 or h < 10:
            return

        cx, cy = w / 2.0, h / 2.0

        # pixels-per-metre for the main view (show ±5m around robot)
        VIEW_RANGE_M = 5.0
        ppm = min(w, h) / (2.0 * VIEW_RANGE_M)

        # ── Draw SLAM occupancy map ──────────────────────────────────────────
        grid = self.occ_grid
        if grid is not None:
            res  = grid.info.resolution          # metres / cell
            gw   = grid.info.width
            gh   = grid.info.height
            ox   = grid.info.origin.position.x  # world coords of cell (0,0)
            oy   = grid.info.origin.position.y

            cell_px = max(1, int(res * ppm))      # cell size in pixels

            for row in range(gh):
                for col in range(gw):
                    val = grid.data[row * gw + col]
                    if val == -1:
                        colour = "#0a0a20"   # unexplored – very dark blue
                    elif val == 0:
                        colour = "#1c2a3a"   # free – dark teal
                    elif val > 50:
                        colour = "#e0e0e8"   # occupied – light (wall)
                    else:
                        continue  # skip mid values

                    # World coords of this cell
                    wx = ox + (col + 0.5) * res
                    wy = oy + (row + 0.5) * res

                    # Canvas coords (robot-centred)
                    sx = cx + (wx - self.robot_x) * ppm
                    sy = cy - (wy - self.robot_y) * ppm

                    if -cell_px < sx < w + cell_px and -cell_px < sy < h + cell_px:
                        c.create_rectangle(
                            sx, sy, sx + cell_px, sy + cell_px,
                            fill=colour, outline=""
                        )

        # ── Range rings ──────────────────────────────────────────────────────
        for ring_m in [1, 2, 3, 4, 5]:
            r_px = ring_m * ppm
            c.create_oval(cx - r_px, cy - r_px, cx + r_px, cy + r_px,
                          outline="#111830", width=1)
            c.create_text(cx + r_px + 2, cy, text=f"{ring_m}m",
                          fill="#1a3050", font=("Courier", 7), anchor="w")

        # ── Crosshairs ───────────────────────────────────────────────────────
        c.create_line(cx, 0, cx, h, fill="#0e1830", width=1)
        c.create_line(0, cy, w, cy, fill="#0e1830", width=1)

        # ── LiDAR rays & points ──────────────────────────────────────────────
        scan = self.latest_scan
        valid_pts = 0
        if scan is not None:
            angle = scan.angle_min
            yaw   = self.robot_yaw
            for r in scan.ranges:
                if scan.range_min < r < scan.range_max:
                    # Ray direction in world frame
                    ray_angle = yaw + angle
                    ex = cx + r * math.cos(ray_angle) * ppm
                    ey = cy - r * math.sin(ray_angle) * ppm

                    # Colour: near=orange, far=cyan
                    ratio  = min(r / scan.range_max, 1.0)
                    red_c  = int(255 * (1.0 - ratio * 0.6))
                    grn_c  = int(80  * ratio)
                    blu_c  = int(255 * ratio)
                    colour = f"#{red_c:02x}{grn_c:02x}{blu_c:02x}"

                    # Ray line (dim)
                    c.create_line(cx, cy, ex, ey, fill="#1a2a1a", width=1)
                    # Hit point dot
                    c.create_oval(ex - 2, ey - 2, ex + 2, ey + 2,
                                  fill=colour, outline="")
                    valid_pts += 1
                angle += scan.angle_increment

        # ── Robot marker (triangle pointing in yaw direction) ────────────────
        sz = 10
        yaw = self.robot_yaw
        pts = [
            (cx + sz * math.cos(yaw),               cy - sz * math.sin(yaw)),
            (cx + sz * 0.6 * math.cos(yaw + 2.4),   cy - sz * 0.6 * math.sin(yaw + 2.4)),
            (cx + sz * 0.6 * math.cos(yaw - 2.4),   cy - sz * 0.6 * math.sin(yaw - 2.4)),
        ]
        c.create_polygon(pts, fill=GREEN, outline="#009944", width=2)

        # ── Minimap (top-right corner) ────────────────────────────────────────
        self._draw_minimap(c, w, h, valid_pts)

    # ─────────────────────────────────────────────────────────────────────────
    # Minimap overlay (top-right)
    # ─────────────────────────────────────────────────────────────────────────

    def _draw_minimap(self, c: tk.Canvas, W: int, H: int, valid_pts: int):
        MM_W, MM_H = 220, 150  # minimap pixel size
        PAD = 10
        mx  = W - MM_W - PAD   # top-left corner of minimap
        my  = PAD

        # Minimap background
        c.create_rectangle(mx - 2, my - 2, mx + MM_W + 2, my + MM_H + 2,
                           fill="#050510", outline=ACCENT, width=2)
        c.create_text(mx + MM_W // 2, my + 8,
                      text="Full Map", fill=ACCENT, font=("Courier", 8, "bold"))

        grid = self.occ_grid
        if grid is None:
            c.create_text(mx + MM_W // 2, my + MM_H // 2,
                          text="Awaiting map…", fill="#334", font=("Courier", 8))
            return

        gw  = grid.info.width
        gh  = grid.info.height
        res = grid.info.resolution
        ox  = grid.info.origin.position.x
        oy  = grid.info.origin.position.y

        if gw == 0 or gh == 0:
            return

        # Scale entire map to fit minimap box (leave 15px top for label)
        usable_h = MM_H - 16
        scale_x  = MM_W / gw
        scale_y  = usable_h / gh
        cell_px  = max(1, int(min(scale_x, scale_y)))

        mm_ox = mx
        mm_oy = my + 14

        # Draw cells
        for row in range(gh):
            for col in range(gw):
                val = grid.data[row * gw + col]
                if val == -1:
                    colour = "#0a0a18"
                elif val == 0:
                    colour = "#203040"
                elif val > 50:
                    colour = "#c8c8d8"
                else:
                    continue

                sx = mm_ox + int(col * scale_x)
                sy = mm_oy + int((gh - 1 - row) * scale_y)  # flip Y

                if cell_px >= 2:
                    c.create_rectangle(sx, sy, sx + cell_px, sy + cell_px,
                                       fill=colour, outline="")
                else:
                    c.create_rectangle(sx, sy, sx + 1, sy + 1,
                                       fill=colour, outline="")

        # Robot dot on minimap
        robot_col = (self.robot_x - ox) / res
        robot_row = (self.robot_y - oy) / res
        rx = mm_ox + int(robot_col * scale_x)
        ry = mm_oy + int((gh - 1 - robot_row) * scale_y)
        c.create_oval(rx - 4, ry - 4, rx + 4, ry + 4,
                      fill=GREEN, outline="#009944", width=1)

    # ─────────────────────────────────────────────────────────────────────────
    # Status labels
    # ─────────────────────────────────────────────────────────────────────────

    def _update_labels(self):
        yaw_deg = math.degrees(self.robot_yaw)
        self.status_lbl.config(
            text=f"X:{self.robot_x:.2f}  Y:{self.robot_y:.2f}  θ:{yaw_deg:.1f}°"
        )
        scan = self.latest_scan
        if scan is not None:
            pts = sum(1 for r in scan.ranges if scan.range_min < r < scan.range_max)
            self.scan_lbl.config(text=f"LiDAR: {pts}/{len(scan.ranges)} pts")
        else:
            self.scan_lbl.config(text="LiDAR: Waiting…")

    # ─────────────────────────────────────────────────────────────────────────
    # Run
    # ─────────────────────────────────────────────────────────────────────────

    def run(self):
        spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        spin_thread.start()
        try:
            self.root.mainloop()
        except KeyboardInterrupt:
            pass
        finally:
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AMRGuiNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
