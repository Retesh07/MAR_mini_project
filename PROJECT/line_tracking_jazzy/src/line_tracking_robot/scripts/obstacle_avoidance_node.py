
#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String


class ObstacleAvoidanceNode(Node):
    STATE_FOLLOW_LINE = 'follow_line'
    STATE_TURN_AWAY = 'turn_away'
    STATE_BYPASS = 'bypass'
    STATE_RETURN = 'return'

    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        self.declare_parameter('stop_distance', 0.65)
        self.declare_parameter('warn_distance', 0.95)
        self.declare_parameter('front_angle_deg', 35.0)
        self.declare_parameter('side_angle_deg', 80.0)
        self.declare_parameter('avoid_linear_speed', 0.18)
        self.declare_parameter('avoid_angular_speed', 1.2)
        self.declare_parameter('bypass_turn_gain', 0.30)
        self.declare_parameter('return_linear_speed', 0.14)
        self.declare_parameter('return_angular_speed', 0.95)
        self.declare_parameter('line_reacquire_hold', 0.40)
        self.declare_parameter('min_bypass_time', 1.10)
        self.declare_parameter('max_avoid_time', 8.0)

        self.stop_dist = float(self.get_parameter('stop_distance').value)
        self.warn_dist = float(self.get_parameter('warn_distance').value)
        self.front_angle = math.radians(float(self.get_parameter('front_angle_deg').value))
        self.side_angle = math.radians(float(self.get_parameter('side_angle_deg').value))
        self.avoid_lin = float(self.get_parameter('avoid_linear_speed').value)
        self.avoid_ang = float(self.get_parameter('avoid_angular_speed').value)
        self.bypass_turn_gain = float(self.get_parameter('bypass_turn_gain').value)
        self.return_lin = float(self.get_parameter('return_linear_speed').value)
        self.return_ang = float(self.get_parameter('return_angular_speed').value)
        self.line_reacquire_hold = float(self.get_parameter('line_reacquire_hold').value)
        self.min_bypass_time = float(self.get_parameter('min_bypass_time').value)
        self.max_avoid_time = float(self.get_parameter('max_avoid_time').value)

        self.state = self.STATE_FOLLOW_LINE
        self.last_avoid_dir = 'left'
        self.line_detected = False
        self.line_seen_since = None
        self.state_since = time.time()

        self.front_min = float('inf')
        self.left_min = float('inf')
        self.right_min = float('inf')

        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.direction_pub = self.create_publisher(String, '/obstacle_direction', 10)
        self.distance_pub = self.create_publisher(Float32, '/obstacle_distance', 10)
        self.avoid_cmd_pub = self.create_publisher(Twist, '/avoid_cmd', 10)
        self.avoiding_pub = self.create_publisher(Bool, '/is_avoiding', 10)

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Bool, '/line_detected', self.line_callback, 10)

        self.get_logger().info('Obstacle Avoidance Node started.')
        self.get_logger().info(
            f'stop={self.stop_dist:.2f} warn={self.warn_dist:.2f} avoid_lin={self.avoid_lin:.2f} avoid_ang={self.avoid_ang:.2f}'
        )

    def set_state(self, new_state, reason=''):
        if self.state != new_state:
            if reason:
                self.get_logger().info(f'{self.state} -> {new_state} | {reason}')
            else:
                self.get_logger().info(f'{self.state} -> {new_state}')
            self.state = new_state
            self.state_since = time.time()

    def line_callback(self, msg):
        self.line_detected = bool(msg.data)
        if self.line_detected:
            if self.line_seen_since is None:
                self.line_seen_since = time.time()
        else:
            self.line_seen_since = None

    def get_sector_min(self, ranges, angles, min_a, max_a):
        valid = [
            r for r, a in zip(ranges, angles)
            if min_a <= a <= max_a and not math.isnan(r) and not math.isinf(r) and r > 0.05
        ]
        return min(valid) if valid else float('inf')

    def scan_callback(self, msg):
        ranges = list(msg.ranges)
        angles = [msg.angle_min + i * msg.angle_increment for i in range(len(ranges))]

        self.front_min = self.get_sector_min(ranges, angles, -self.front_angle, self.front_angle)
        self.left_min = self.get_sector_min(ranges, angles, self.front_angle, self.front_angle + self.side_angle)
        self.right_min = self.get_sector_min(ranges, angles, -(self.front_angle + self.side_angle), -self.front_angle)
        closest = min(self.front_min, self.left_min, self.right_min)

        dist_msg = Float32()
        dist_msg.data = float(closest)
        self.distance_pub.publish(dist_msg)

        obstacle_present = closest < self.warn_dist
        obs_msg = Bool()
        obs_msg.data = obstacle_present
        self.obstacle_pub.publish(obs_msg)

        direction = 'none'
        if self.front_min < self.stop_dist:
            direction = 'front'
        elif self.left_min < self.stop_dist:
            direction = 'left'
        elif self.right_min < self.stop_dist:
            direction = 'right'
        dir_msg = String()
        dir_msg.data = direction
        self.direction_pub.publish(dir_msg)

        cmd = Twist()
        is_avoiding = self.state != self.STATE_FOLLOW_LINE
        now = time.time()
        state_elapsed = now - self.state_since

        if self.state == self.STATE_FOLLOW_LINE:
            if self.front_min < self.stop_dist:
                self.last_avoid_dir = 'left' if self.left_min >= self.right_min else 'right'
                self.set_state(
                    self.STATE_TURN_AWAY,
                    f'obstacle ahead front={self.front_min:.2f} left={self.left_min:.2f} right={self.right_min:.2f} choose={self.last_avoid_dir}'
                )
                is_avoiding = True

        elif self.state == self.STATE_TURN_AWAY:
            is_avoiding = True
            turn_sign = 1.0 if self.last_avoid_dir == 'left' else -1.0
            cmd.linear.x = 0.02
            cmd.angular.z = turn_sign * self.avoid_ang

            front_clear = self.front_min > (self.warn_dist + 0.10)
            chosen_side_clear = (self.left_min if self.last_avoid_dir == 'left' else self.right_min) > (self.stop_dist + 0.10)
            if front_clear and chosen_side_clear:
                self.set_state(self.STATE_BYPASS, 'turned enough, start bypass')
            elif state_elapsed > self.max_avoid_time:
                self.set_state(self.STATE_BYPASS, 'timeout in turn, force bypass')

        elif self.state == self.STATE_BYPASS:
            is_avoiding = True
            clearance_target = 0.45
            followed_side = self.left_min if self.last_avoid_dir == 'left' else self.right_min
            side_error = max(-1.0, min(1.0, clearance_target - followed_side))
            turn_sign = -1.0 if self.last_avoid_dir == 'left' else 1.0

            cmd.linear.x = self.avoid_lin
            cmd.angular.z = turn_sign * self.bypass_turn_gain * side_error

            if self.front_min < self.stop_dist:
                cmd.linear.x = 0.03
                cmd.angular.z = (1.0 if self.last_avoid_dir == 'left' else -1.0) * self.avoid_ang

            if state_elapsed > self.min_bypass_time and self.line_seen_since is not None:
                held = now - self.line_seen_since
                if held >= self.line_reacquire_hold:
                    self.set_state(self.STATE_RETURN, 'line visible again, start return')
            elif state_elapsed > self.max_avoid_time:
                self.set_state(self.STATE_RETURN, 'timeout in bypass, start return')

        elif self.state == self.STATE_RETURN:
            is_avoiding = True
            turn_sign = -1.0 if self.last_avoid_dir == 'left' else 1.0
            cmd.linear.x = self.return_lin
            cmd.angular.z = turn_sign * self.return_ang

            if self.line_seen_since is not None and (now - self.line_seen_since) >= self.line_reacquire_hold:
                cmd = Twist()
                self.set_state(self.STATE_FOLLOW_LINE, 'line reacquired stably')
                is_avoiding = False
            elif state_elapsed > 4.0:
                self.set_state(self.STATE_FOLLOW_LINE, 'return timeout, hand over to line follower')
                is_avoiding = False
                cmd = Twist()

        self.avoid_cmd_pub.publish(cmd)
        avoid_msg = Bool()
        avoid_msg.data = bool(is_avoiding)
        self.avoiding_pub.publish(avoid_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
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
