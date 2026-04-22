#!/usr/bin/env python3
"""
Camera Processor Node
=====================
Processes the downward-facing camera image to detect the black line.
Publishes line position error for the line follower controller.

Topics:
  Subscribes: /camera/image_raw (sensor_msgs/Image)
  Publishes:  /line_error (std_msgs/Float32)  - normalized lateral error [-1, 1]
              /line_detected (std_msgs/Bool)
              /camera/processed (sensor_msgs/Image) - debug visualization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraProcessorNode(Node):
    def __init__(self):
        super().__init__('camera_processor_node')

        # Parameters
        self.declare_parameter('line_threshold', 80)       # pixel darkness threshold
        self.declare_parameter('roi_top_ratio', 0.6)       # top of ROI (fraction of height)
        self.declare_parameter('roi_bottom_ratio', 0.95)   # bottom of ROI
        self.declare_parameter('debug_view', True)

        self.threshold = self.get_parameter('line_threshold').value
        self.roi_top = self.get_parameter('roi_top_ratio').value
        self.roi_bottom = self.get_parameter('roi_bottom_ratio').value
        self.debug = self.get_parameter('debug_view').value

        # CV bridge
        self.bridge = CvBridge()

        # Publishers
        self.line_error_pub = self.create_publisher(Float32, '/line_error', 10)
        self.line_detected_pub = self.create_publisher(Bool, '/line_detected', 10)
        self.debug_img_pub = self.create_publisher(Image, '/camera/processed', 10)

        # Subscriber
        self.img_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # State
        self.last_error = 0.0
        self.frame_count = 0

        self.get_logger().info('Camera Processor Node started.')

    def image_callback(self, msg):
        self.frame_count += 1
        try:
            # Convert ROS image to OpenCV
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        height, width = cv_img.shape[:2]

        # ---- Region of Interest ----
        roi_y_start = int(height * self.roi_top)
        roi_y_end = int(height * self.roi_bottom)
        roi = cv_img[roi_y_start:roi_y_end, :]

        # ---- Convert to grayscale and threshold ----
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, self.threshold, 255, cv2.THRESH_BINARY_INV)

        # ---- Morphological cleanup ----
        kernel = np.ones((5, 5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

        # ---- Find contours ----
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        line_detected = False
        error = self.last_error  # default to last known error (smooth recovery)

        if contours:
            # Pick the largest contour (most likely the line)
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 200:  # Minimum area to be a valid line segment
                line_detected = True

                # Compute centroid
                M = cv2.moments(largest)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    # cy = int(M['m01'] / M['m00'])  # not used for lateral control

                    # Normalized error: 0 = center, -1 = full left, +1 = full right
                    error = (cx - width / 2.0) / (width / 2.0)
                    self.last_error = error

                # Debug visualization
                if self.debug:
                    debug_img = cv_img.copy()
                    # Draw ROI box
                    cv2.rectangle(debug_img, (0, roi_y_start), (width, roi_y_end), (0, 255, 0), 2)
                    # Draw contour on ROI
                    shifted_contour = largest + np.array([0, roi_y_start])
                    cv2.drawContours(debug_img, [shifted_contour], -1, (0, 0, 255), 2)
                    # Draw center line
                    cv2.line(debug_img, (width//2, roi_y_start), (width//2, roi_y_end), (255, 0, 0), 1)
                    # Draw centroid marker
                    if line_detected:
                        cx_global = cx
                        cy_global = int((roi_y_start + roi_y_end) / 2)
                        cv2.circle(debug_img, (cx_global, cy_global), 8, (0, 255, 255), -1)
                    # Error text
                    cv2.putText(debug_img, f'Error: {error:.3f}', (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    cv2.putText(debug_img, f'Line: {"YES" if line_detected else "NO"}', (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if line_detected else (0, 0, 255), 2)

                    try:
                        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
                        self.debug_img_pub.publish(debug_msg)
                    except Exception:
                        pass

        # Publish results
        err_msg = Float32()
        err_msg.data = float(error)
        self.line_error_pub.publish(err_msg)

        det_msg = Bool()
        det_msg.data = line_detected
        self.line_detected_pub.publish(det_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessorNode()
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
