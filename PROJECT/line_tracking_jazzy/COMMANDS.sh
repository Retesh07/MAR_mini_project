#!/bin/bash
# ============================================================
#  COMPLETE SETUP FOR ROS2 JAZZY - NO ROSDEP NEEDED
#  Run each section one at a time
# ============================================================

# ==============================================================
# SECTION 1: Install all packages (run ONCE)
# ==============================================================

sudo apt update

# ROS2 Jazzy core
sudo apt install -y ros-jazzy-desktop

# Gazebo Harmonic + ROS bridge
sudo apt install -y ros-jazzy-ros-gz

# Robot tools
sudo apt install -y \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-rviz2

# Build tool
sudo apt install -y python3-colcon-common-extensions

# Python deps (--break-system-packages needed on Ubuntu 24.04)
pip3 install opencv-python numpy --break-system-packages


# ==============================================================
# SECTION 2: Build (run once, or after any code change)
# ==============================================================

source /opt/ros/jazzy/setup.bash
cd ~/line_tracking_jazzy
colcon build --symlink-install


# ==============================================================
# SECTION 3: Run the simulation
# ==============================================================

source /opt/ros/jazzy/setup.bash
source ~/line_tracking_jazzy/install/setup.bash
ros2 launch line_tracking_robot simulation.launch.py


# ==============================================================
# SECTION 4: In NEW terminals - monitor the robot
# ==============================================================

# Terminal 2: Watch robot status
source /opt/ros/jazzy/setup.bash
source ~/line_tracking_jazzy/install/setup.bash
ros2 topic echo /robot_status

# Terminal 3: Check all topics are active
ros2 topic list

# Terminal 4: Manually test robot movement (if not moving)
source /opt/ros/jazzy/setup.bash
source ~/line_tracking_jazzy/install/setup.bash
python3 ~/line_tracking_jazzy/src/line_tracking_robot/scripts/test_movement.py

# Terminal 5: Send a single move command to test bridge
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
