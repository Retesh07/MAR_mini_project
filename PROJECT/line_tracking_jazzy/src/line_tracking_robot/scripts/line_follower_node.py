
#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')

        self.declare_parameter('kp', 1.20)
        self.declare_parameter('ki', 0.002)
        self.declare_parameter('kd', 0.20)
        self.declare_parameter('base_speed', 0.28)
        self.declare_parameter('max_angular', 2.20)
        self.declare_parameter('search_angular', 0.60)
        self.declare_parameter('search_linear', 0.03)
        self.declare_parameter('search_duration', 300.0)
        self.declare_parameter('min_speed_factor', 0.42)

        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.base_speed = float(self.get_parameter('base_speed').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.search_angular = float(self.get_parameter('search_angular').value)
        self.search_linear = float(self.get_parameter('search_linear').value)
        self.search_dur = float(self.get_parameter('search_duration').value)
        self.min_speed_factor = float(self.get_parameter('min_speed_factor').value)

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

        self.line_error = 0.0
        self.line_detected = False
        self.lost_time = None

        self.cmd_pub = self.create_publisher(Twist, '/line_follow_cmd', 10)
        self.create_subscription(Float32, '/line_error', self.error_callback, 10)
        self.create_subscription(Bool, '/line_detected', self.detect_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Line Follower Node started.')
        self.get_logger().info(
            f'PID kp={self.kp:.3f} ki={self.ki:.3f} kd={self.kd:.3f} base_speed={self.base_speed:.2f}'
        )

    def error_callback(self, msg):
        self.line_error = float(msg.data)

    def detect_callback(self, msg):
        was_detected = self.line_detected
        self.line_detected = bool(msg.data)
        if not self.line_detected and was_detected:
            self.lost_time = time.time()
            self.integral = 0.0
            self.get_logger().warn('Line lost, searching...')
        elif self.line_detected and not was_detected:
            self.lost_time = None
            self.get_logger().info('Line reacquired.')

    def control_loop(self):
        cmd = Twist()
        now = time.time()
        dt = max(1e-3, now - self.prev_time)
        self.prev_time = now

        if self.line_detected:
            error = self.line_error

            self.integral += error * dt
            self.integral = max(-0.8, min(0.8, self.integral))
            derivative = (error - self.prev_error) / dt
            self.prev_error = error

            angular = -(self.kp * error + self.ki * self.integral + self.kd * derivative)
            angular = max(-self.max_angular, min(self.max_angular, angular))

            speed_factor = 1.0 - 0.65 * abs(error)
            speed_factor = max(self.min_speed_factor, min(1.0, speed_factor))

            cmd.linear.x = self.base_speed * speed_factor
            cmd.angular.z = angular
        else:
            if self.lost_time is not None and (now - self.lost_time) < self.search_dur:
                cmd.linear.x = self.search_linear
                cmd.angular.z = self.search_angular if self.prev_error < 0.0 else -self.search_angular
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
