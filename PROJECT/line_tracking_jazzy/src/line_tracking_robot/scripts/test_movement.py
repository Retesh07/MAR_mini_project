#!/usr/bin/env python3
"""
test_movement.py
================
Run this SEPARATELY after launching the simulation to test
if the robot physically moves in Gazebo.

Usage (in a new terminal):
  source /opt/ros/jazzy/setup.bash
  source ~/line_tracking_jazzy/install/setup.bash
  python3 ~/line_tracking_jazzy/src/line_tracking_robot/scripts/test_movement.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class MovementTester(Node):
    def __init__(self):
        super().__init__('movement_tester')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Movement tester started. Publishing to /cmd_vel ...')

    def run_test(self):
        time.sleep(1.0)  # Let publisher set up

        self.get_logger().info('TEST 1: Moving forward...')
        self._send(0.2, 0.0, duration=3.0)

        self.get_logger().info('TEST 2: Turning left...')
        self._send(0.0, 0.5, duration=2.0)

        self.get_logger().info('TEST 3: Moving forward again...')
        self._send(0.2, 0.0, duration=3.0)

        self.get_logger().info('TEST 4: Turning right...')
        self._send(0.0, -0.5, duration=2.0)

        self.get_logger().info('TEST complete. Stopping.')
        self._send(0.0, 0.0, duration=1.0)

    def _send(self, linear, angular, duration):
        cmd = Twist()
        cmd.linear.x  = linear
        cmd.angular.z = angular
        end = time.time() + duration
        while time.time() < end:
            self.pub.publish(cmd)
            time.sleep(0.05)


def main():
    rclpy.init()
    node = MovementTester()
    node.run_test()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
