#!/usr/bin/env python3
"""
Outdoor AMR Web Server Node
============================
Serves the Outdoor 3D HTML UI and bridges ROS2 topics via WebSockets:
  - outdoor/odom  → odom
  - scan          → lidar 2D scan
  - gps/fix       → GPS coordinates
  - outdoor/cmd_vel ← keyboard commands from browser
Uses port 8001 (HTTP) and 9091 (WS) to avoid conflicts with indoor AMR.
"""

import json
import math
import threading
import asyncio
import websockets
import http.server
import socketserver
import subprocess
import webbrowser

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

HTTP_PORT = 8001
WS_PORT   = 9091


class OutdoorWebServerNode(Node):
    def __init__(self):
        super().__init__('outdoor_web_server_node')

        # Subscriptions
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self._on_scan, 10)
        self.odom_sub = self.create_subscription(Odometry, 'outdoor/odom', self._on_odom, 10)
        self.gps_sub  = self.create_subscription(NavSatFix, 'gps/fix', self._on_gps, 10)
        self.obs_sub  = self.create_subscription(String, 'outdoor/obstacles_json', self._on_obs, 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'outdoor/cmd_vel', 10)

        # State
        self.latest_odom = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.latest_scan = {'ranges': [], 'angle_min': 0.0, 'angle_inc': 0.0, 'range_max': 80.0}
        self.latest_gps  = {'lat': 12.9716, 'lon': 77.5946, 'alt': 920.0, 'fix': False}

        self.ws_clients = set()
        self.loop = None

        threading.Thread(target=self._start_ws,   daemon=True).start()
        threading.Thread(target=self._start_http, daemon=True).start()

    # ── ROS callbacks ────────────────────────────────────────────────────────
    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.latest_odom = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': yaw
        }
        self._broadcast({'type': 'odom', 'data': self.latest_odom})

    def _on_scan(self, msg: LaserScan):
        step = 2
        ranges = [float(r) if msg.range_min < r < msg.range_max else 0.0
                  for r in msg.ranges[::step]]
        self.latest_scan = {
            'ranges': ranges,
            'angle_min': msg.angle_min,
            'angle_inc': msg.angle_increment * step,
            'range_max': msg.range_max
        }
        self._broadcast({'type': 'scan', 'data': self.latest_scan})

    def _on_gps(self, msg: NavSatFix):
        from sensor_msgs.msg import NavSatStatus
        self.latest_gps = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude,
            'fix': msg.status.status >= 0
        }
        self._broadcast({'type': 'gps', 'data': self.latest_gps})

    def _on_obs(self, msg: String):
        try:
            self._broadcast({'type': 'obstacles', 'data': json.loads(msg.data)})
        except Exception:
            pass

    # ── WebSocket ────────────────────────────────────────────────────────────
    def _broadcast(self, msg_dict):
        if not self.ws_clients or self.loop is None:
            return
        msg_str = json.dumps(msg_dict)
        for ws in list(self.ws_clients):
            asyncio.run_coroutine_threadsafe(ws.send(msg_str), self.loop)

    async def _ws_handler(self, websocket):
        self.ws_clients.add(websocket)
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    if data.get('type') == 'cmd_vel':
                        twist = Twist()
                        twist.linear.x  = float(data['linear'])
                        twist.angular.z = float(data['angular'])
                        self.cmd_vel_pub.publish(twist)
                except Exception:
                    pass
        finally:
            self.ws_clients.discard(websocket)

    def _start_ws(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        async def run_server():
            async with websockets.serve(self._ws_handler, '0.0.0.0', WS_PORT):
                await asyncio.Future()

        self.loop.run_until_complete(run_server())

    # ── HTTP ─────────────────────────────────────────────────────────────────
    def _start_http(self):
        import os
        from ament_index_python.packages import get_package_share_directory
        web_dir = os.path.join(get_package_share_directory('outdoor_amr_gui'), 'web')
        os.chdir(web_dir)
        Handler = http.server.SimpleHTTPRequestHandler

        try:
            httpd = socketserver.TCPServer(('', HTTP_PORT), Handler)
            port = HTTP_PORT
        except OSError:
            httpd = socketserver.TCPServer(('', 0), Handler)
            port = httpd.server_address[1]

        url = f'http://localhost:{port}'
        self.get_logger().info(f'Outdoor 3D UI at {url}')
        try:
            subprocess.Popen(['google-chrome', f'--app={url}', '--window-size=1280,820'])
        except Exception:
            webbrowser.open(url)
        httpd.serve_forever()


def main(args=None):
    rclpy.init(args=args)
    node = OutdoorWebServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
