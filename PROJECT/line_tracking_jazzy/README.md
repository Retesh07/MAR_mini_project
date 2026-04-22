# Line Tracking Robot with Obstacle Detection & Avoidance
## ROS2 Jazzy + Gazebo Harmonic Simulation

---

## 📋 Project Overview

| Item | Detail |
|------|--------|
| **Topic** | Line Tracking with Obstacle Detection and Avoidance |
| **ROS Version** | ROS2 Jazzy Jalopy |
| **Simulator** | Gazebo Harmonic (gz sim) |
| **OS** | Ubuntu 24.04 LTS |

---

## ⚙️ Prerequisites — Install Everything

### 1. ROS2 Jazzy
```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

### 2. Gazebo Harmonic + ROS2 packages
```bash
sudo apt install -y ros-jazzy-ros-gz
sudo apt install -y \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-rviz2 \
  python3-colcon-common-extensions \
  python3-rosdep
```

### 3. Python dependencies
```bash
pip3 install opencv-python numpy --break-system-packages
```

---

## 🚀 Build & Run — Step by Step

```bash
# Step 1: Source ROS2
source /opt/ros/jazzy/setup.bash

# Step 2: Go to workspace
cd ~/line_tracking_jazzy

# Step 3: Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Step 4: Build
colcon build --symlink-install

# Step 5: Source workspace
source install/setup.bash

# Step 6: Launch!
ros2 launch line_tracking_robot simulation.launch.py
```

---

## 🔑 Key Difference from Humble Version

| Feature | Humble | Jazzy |
|---------|--------|-------|
| Simulator | Gazebo Classic (`gazebo`) | Gazebo Harmonic (`gz sim`) |
| Drive Plugin | `libgazebo_ros_diff_drive.so` | `gz-sim-diff-drive-system` |
| LiDAR Plugin | `libgazebo_ros_ray_sensor.so` | `gz-sim-gpu-lidar-sensor-system` |
| Camera Plugin | `libgazebo_ros_camera.so` | `gz-sim-camera-system` |
| Bridge pkg | `gazebo_ros` | `ros_gz_bridge` |
| Spawn cmd | `gazebo_ros spawn_entity.py` | `ros_gz_sim create` |
| SDF version | 1.7 | 1.9 |

---

## 📡 Topics

| Topic | Direction | Description |
|-------|-----------|-------------|
| `/cmd_vel` | ROS→GZ | Drive commands |
| `/odom` | GZ→ROS | Odometry |
| `/scan` | GZ→ROS | LiDAR scan |
| `/camera/image_raw` | GZ→ROS | Raw camera image |
| `/line_error` | ROS | Lateral line error [-1,1] |
| `/obstacle_distance` | ROS | Closest obstacle (m) |
| `/robot_status` | ROS | Current mode string |

---

## 🔍 Monitor Commands (new terminals)

```bash
source /opt/ros/jazzy/setup.bash
source ~/line_tracking_jazzy/install/setup.bash

# Robot mode
ros2 topic echo /robot_status

# Line error
ros2 topic echo /line_error

# Obstacle distance  
ros2 topic echo /obstacle_distance

# All topics
ros2 topic list
```

---

## 🐛 Troubleshooting

| Problem | Fix |
|---------|-----|
| `gz: command not found` | `sudo apt install ros-jazzy-ros-gz` |
| Bridge errors | Re-run Step 1 (source ROS2) then relaunch |
| Robot not spawning | Wait 5–8 sec, Gazebo Harmonic is slower to init |
| Camera topic missing | Check bridge is running: `ros2 node list` |
| `colcon` not found | `sudo apt install python3-colcon-common-extensions` |
