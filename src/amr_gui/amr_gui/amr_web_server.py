#!/usr/bin/env python3
"""
Custom 3D UI Web Server Node
============================
Serves the 3D HTML UI and broadcasts ROS2 topics (Odom, Scan, Entities) via WebSockets.
"""

import json
import math
import threading
import asyncio
import websockets
import http.server
import socketserver
import webbrowser

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# To track spawned obstacles, we could subscribe to gazebo states,
# but for simplicity we will just let the GUI broadcast its state
# or hardcode the known static walls.

HTTP_PORT = 8000
WS_PORT = 9090

class WebServerNode(Node):
    def __init__(self):
        super().__init__('amr_web_server_node')
        
        from std_msgs.msg import String
        from geometry_msgs.msg import Twist
        self.scan_sub = self.create_subscription(LaserScan, "scan", self._on_scan, 10)
        self.odom_sub = self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.obs_sub = self.create_subscription(String, "obstacles_json", self._on_obs, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.latest_odom = {"x": 0, "y": 0, "yaw": 0}
        self.latest_scan = {"ranges": [], "angle_min": 0, "angle_inc": 0, "range_max": 10.0}

        self.ws_clients = set()
        
        # Start WS thread
        threading.Thread(target=self._start_ws, daemon=True).start()
        # Start HTTP thread
        threading.Thread(target=self._start_http, daemon=True).start()

    def _on_obs(self, msg):
        try:
            data = json.loads(msg.data)
            self._broadcast({"type": "obstacles", "data": data})
        except Exception:
            pass

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.latest_odom = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "yaw": yaw
        }
        self._broadcast({"type": "odom", "data": self.latest_odom})

    def _on_scan(self, msg: LaserScan):
        # Downsample scan for web performance
        step = 4
        ranges = [float(r) if msg.range_min < r < msg.range_max else 0.0 for r in msg.ranges[::step]]
        self.latest_scan = {
            "ranges": ranges,
            "angle_min": msg.angle_min,
            "angle_inc": msg.angle_increment * step,
            "range_max": msg.range_max
        }
        self._broadcast({"type": "scan", "data": self.latest_scan})

    def _broadcast(self, msg_dict):
        if not self.ws_clients: return
        msg_str = json.dumps(msg_dict)
        for ws in list(self.ws_clients):
            asyncio.run_coroutine_threadsafe(ws.send(msg_str), self.loop)

    async def _ws_handler(self, websocket):
        self.ws_clients.add(websocket)
        try:
            async for message in websocket:
                try:
                    msg_data = json.loads(message)
                    if msg_data.get("type") == "cmd_vel":
                        twist = Twist()
                        twist.linear.x = float(msg_data["linear"])
                        twist.angular.z = float(msg_data["angular"])
                        self.cmd_vel_pub.publish(twist)
                except Exception:
                    pass
        finally:
            self.ws_clients.remove(websocket)

    def _start_ws(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        async def run_server():
            async with websockets.serve(self._ws_handler, "0.0.0.0", WS_PORT):
                await asyncio.Future()  # run forever
                
        self.loop.run_until_complete(run_server())

    def _start_http(self):
        import os
        import subprocess
        from ament_index_python.packages import get_package_share_directory
        web_dir = os.path.join(get_package_share_directory('amr_gui'), 'web')
        os.chdir(web_dir)
        Handler = http.server.SimpleHTTPRequestHandler
        
        # Use port 8000, or a random open port if 8000 is blocked
        try:
            httpd = socketserver.TCPServer(("", HTTP_PORT), Handler)
            port = HTTP_PORT
        except OSError:
            httpd = socketserver.TCPServer(("", 0), Handler)
            port = httpd.server_address[1]

        url = f"http://localhost:{port}"
        print(f"Serving 3D UI at {url}")
        
        # Launch as a Window (App Mode) instead of a Chrome tab
        try:
            subprocess.Popen(['google-chrome', f'--app={url}', '--window-size=1200,800'])
        except Exception:
            webbrowser.open(url)
            
        httpd.serve_forever()

def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
