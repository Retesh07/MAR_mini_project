
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, String


class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')

        self.line_follow_cmd = Twist()
        self.avoid_cmd = Twist()
        self.is_avoiding = False
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')
        self.line_detected = False
        self.line_error = 0.0
        self.emergency_stop = False

        self.declare_parameter('emergency_stop_dist', 0.10)
        self.emergency_dist = float(self.get_parameter('emergency_stop_dist').value)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)

        self.create_subscription(Twist, '/line_follow_cmd', lambda m: setattr(self, 'line_follow_cmd', m), 10)
        self.create_subscription(Twist, '/avoid_cmd', lambda m: setattr(self, 'avoid_cmd', m), 10)
        self.create_subscription(Bool, '/is_avoiding', lambda m: setattr(self, 'is_avoiding', bool(m.data)), 10)
        self.create_subscription(Bool, '/obstacle_detected', lambda m: setattr(self, 'obstacle_detected', bool(m.data)), 10)
        self.create_subscription(Float32, '/obstacle_distance', lambda m: setattr(self, 'obstacle_distance', float(m.data)), 10)
        self.create_subscription(Bool, '/line_detected', lambda m: setattr(self, 'line_detected', bool(m.data)), 10)
        self.create_subscription(Float32, '/line_error', lambda m: setattr(self, 'line_error', float(m.data)), 10)

        self.timer = self.create_timer(0.05, self.control_loop)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('Robot Controller started.')

    def control_loop(self):
        cmd = Twist()

        if self.obstacle_distance < self.emergency_dist:
            if not self.emergency_stop:
                self.get_logger().error(f'EMERGENCY STOP obstacle_distance={self.obstacle_distance:.2f}m')
            self.emergency_stop = True
            self.cmd_vel_pub.publish(cmd)
            return

        self.emergency_stop = False
        cmd = self.avoid_cmd if self.is_avoiding else self.line_follow_cmd
        self.cmd_vel_pub.publish(cmd)

    def publish_status(self):
        if self.emergency_stop:
            mode = 'EMERGENCY_STOP'
        elif self.is_avoiding:
            mode = 'OBSTACLE_AVOIDANCE'
        elif self.line_detected:
            mode = 'LINE_FOLLOWING'
        else:
            mode = 'SEARCHING_FOR_LINE'

        msg = String()
        msg.data = (
            f'Mode={mode} | '
            f'Line={"YES" if self.line_detected else "NO"} (err={self.line_error:.3f}) | '
            f'Obstacle={"YES" if self.obstacle_detected else "NO"} ({self.obstacle_distance:.2f}m)'
        )
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_vel_pub.publish(stop)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
