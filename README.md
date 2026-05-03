# AMR_2025


Terminal 1: Launch Physics + RViz2 (Window 1)

source /opt/ros/humble/setup.bash
source /home/user/AMR/install/setup.bash
ros2 launch amr_gazebo full_system_launch.py


Terminal 2: Launch the 2D Tracker & Remote (Window 2)

source /opt/ros/humble/setup.bash
source /home/user/AMR/install/setup.bash
ros2 run amr_gui amr_gui_node