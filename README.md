# Autonomous Line Tracking and Obstacle Avoidance Robot (ROS 2 Jazzy)

## 1. Introduction

This project implements an autonomous mobile robot capable of following a predefined path (black line) while detecting and avoiding obstacles in real time. The system is developed using ROS 2 Jazzy and simulated in Gazebo.

The robot uses camera-based vision for line tracking and LiDAR for obstacle detection. A control architecture is designed to switch dynamically between line-following behavior and obstacle avoidance.

---

## 2. Objectives

* To implement real-time line tracking using image processing
* To detect obstacles using LiDAR sensor data
* To design a reliable obstacle avoidance mechanism
* To ensure smooth transition between tracking and avoidance
* To simulate the complete system in Gazebo

---

## 3. System Architecture

The system is divided into four main modules:

### 3.1 Camera Processor Node

* Subscribes to camera image topic
* Detects the black line using thresholding and contour extraction
* Computes the deviation of the robot from the line
* Publishes:

  * `/line_error`
  * `/line_detected`

### 3.2 Line Follower Node

* Implements PID control
* Adjusts robot movement based on line error
* Publishes:

  * `/line_follow_cmd`

### 3.3 Obstacle Avoidance Node

* Processes LiDAR scan data
* Detects obstacles in front and side regions
* Uses a state machine:

  * FOLLOW_LINE
  * TURN_AWAY
  * BYPASS
  * RETURN
* Publishes:

  * `/avoid_cmd`
  * `/is_avoiding`
  * `/obstacle_distance`

### 3.4 Robot Controller Node

* Acts as the decision layer
* Prioritizes obstacle avoidance over line following
* Publishes:

  * `/cmd_vel`

---

## 4. Working Principle

### 4.1 Line Tracking

* The camera captures images of the track
* A region of interest is selected
* The black line is extracted using thresholding
* The centroid of the line is computed
* Error is calculated relative to image center
* PID controller adjusts steering accordingly

### 4.2 Obstacle Detection

* LiDAR provides distance measurements
* Data is divided into front, left, and right sectors
* Minimum distance is used to detect obstacles

### 4.3 Obstacle Avoidance

* If obstacle is detected:

  * Robot selects direction with more free space
  * Rotates away from obstacle
  * Moves around obstacle (bypass phase)
  * Searches and re-aligns with the line (return phase)

### 4.4 Control Decision

* If obstacle is present → use avoidance command
* Else → use line following command

---

## 5. Technologies Used

* ROS 2 Jazzy
* Gazebo Harmonic
* Python (rclpy)
* OpenCV
* LiDAR (LaserScan)
* RViz

---

## 6. Project Structure

```text
PROJECT/
 └── line_tracking_jazzy/
     ├── src/
     │   └── line_tracking_robot/
     │       ├── scripts/
     │       ├── launch/
     │       ├── models/
     │       ├── worlds/
     │       ├── config/
     │       ├── CMakeLists.txt
     │       └── package.xml
     ├── build.sh
     ├── COMMANDS.sh
     └── README.md
```

---

## 7. Installation and Setup

### 7.1 Clone Repository

```bash
git clone https://github.com/Retesh07/MAR_mini_project.git
cd MAR_mini_project/PROJECT/line_tracking_jazzy
```

### 7.2 Source ROS 2

```bash
source /opt/ros/jazzy/setup.bash
```

### 7.3 Build Workspace

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

---

## 8. Running the Simulation

```bash
ros2 launch line_tracking_robot simulation.launch.py
```

---

## 9. Monitoring the System

Open a new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /robot_status
```

Additional useful topics:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /obstacle_distance
ros2 topic echo /is_avoiding
```

---

## 10. Results

* The robot successfully follows the line using PID control
* Obstacles are detected in real time using LiDAR
* The robot avoids obstacles and returns to the line
* System demonstrates stable behavior in simulation

---

## 11. Limitations

* Designed for static obstacles
* Performance depends on lighting conditions
* No global mapping or localization

---

## 12. Future Scope

* Integration of SLAM for autonomous navigation
* Deep learning-based object detection
* Real-world hardware implementation
* Dynamic obstacle handling


## 13. License

This project is intended for academic use.
