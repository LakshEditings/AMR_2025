# AMR Navigation Project 2025

This project contains two fully isolated ROS2 simulation environments for an Autonomous Mobile Robot (AMR):
1. **Indoor System**: Warehouse/Indoor navigation using 2D LiDAR SLAM, dynamic obstacles, and a web-based 3D visualizer.
2. **Outdoor System**: Off-road buggy navigation using 3D LiDAR, GPS, and a large grassy environment.

Both systems feature a **2D GUI Control Panel** with manual teleoperation and an autonomous **PICKUP** waypoint navigation system.

---

## 🛠️ Build Instructions

If you haven't built the project yet, run the following:
```bash
cd ~/AMR
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

---

## 🚀 How to Launch

Both systems can be run simultaneously without port conflicts. 

### 🏭 INDOOR SYSTEM

**Terminal 1 — Indoor Physics & 3D Web UI (Chrome):**
```bash
cd ~/AMR
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch amr_gazebo full_system_launch.py
```
*Wait for it to load, then open `http://localhost:8000` in Google Chrome to view the 3D Indoor Environment.*

**Terminal 2 — Indoor 2D GUI & Control Panel:**
```bash
cd ~/AMR
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run amr_gui amr_gui_node
```

### 🌲 OUTDOOR SYSTEM

**Terminal 3 — Outdoor Physics (GPS + 3D LiDAR) & 3D Web UI:**
```bash
cd ~/AMR
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch outdoor_amr_gazebo outdoor_gazebo_launch.py
```
*Wait for it to load, then open `http://localhost:8001` in Google Chrome to view the 3D Outdoor Environment.*

**Terminal 4 — Outdoor 2D GUI (GPS Tracker & Controls):**
```bash
cd ~/AMR
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run outdoor_amr_gui outdoor_gui_node
```

---

## 🎮 How to use the Interactive Waypoints & Autonomous PICKUP

In either the **Indoor** or **Outdoor** 2D GUI Window, you will find a **Waypoints** panel on the right side:

1. **🏠 Home**: Click the `🏠 Home` button (it turns white). Then click anywhere on the 2D canvas map to drop a White Home Box.
2. **🚩 End**: Click the `🚩 End` button (it turns red). Click anywhere on the map to drop a Red End Box.
3. **🏁 Checkpoints**: Click `🏁 ChkPt` (it turns green). Click as many times as you want on the map to drop numbered checkpoints. Click the button again to exit checkpoint mode.
4. **📦 PICKUP**: Once your markers are placed, click `📦 PICKUP`. 
   - The robot will immediately and autonomously drive to the **Home** box.
   - It will then navigate sequentially through all **Checkpoints**.
   - It dynamically avoids obstacles (using live LiDAR data).
   - Finally, it parks at the **End** box.