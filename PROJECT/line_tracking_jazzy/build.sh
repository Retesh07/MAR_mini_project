#!/bin/bash
# Build script for ROS2 Jazzy
set -e

WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "=============================================="
echo "  Line Tracking Robot - ROS2 Jazzy Build"
echo "=============================================="

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "[OK] ROS2 Jazzy sourced"
else
    echo "[ERROR] ROS2 Jazzy not found. Install it first."
    exit 1
fi

cd "$WORKSPACE_DIR"

echo "[1/3] Installing Python deps..."
pip3 install opencv-python numpy --break-system-packages --quiet 2>/dev/null || true

echo "[2/3] Installing ROS deps..."
rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true

echo "[3/3] Building..."
colcon build --symlink-install

echo ""
echo "=============================================="
echo "  Done! Run with:"
echo "    source install/setup.bash"
echo "    ros2 launch line_tracking_robot simulation.launch.py"
echo "=============================================="
