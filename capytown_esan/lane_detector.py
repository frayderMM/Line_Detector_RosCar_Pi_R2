#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('lane_width_m', 0.30)
        self.declare_parameter('px_per_meter', 600.0)
        self.declare_parameter('roi_y_start_ratio', 0.55)
        self.declare_parameter('min_area', 250.0)

        self.declare_parameter('white_low', [0, 0, 160])
        self.declare_parameter('white_high', [180, 80, 255])
        self.declare_parameter('yellow_low', [15, 70, 70])
        self.declare_parameter('yellow_high', [40, 255, 255])

        image_topic = self.get_parameter('image_topic').value

        self.bridge = CvBridge()
        self.pub_error = self.create_publisher(Float32, '/lane_error', 10)
        self.pub_debug = self.create_publisher(Image, '/lane/debug_image', 10)

        self.sub = self.create_subscription(Image, image_topic, self.image_callback, 10)

        self.get_logger().info(f'LaneDetector escuchando: {image_topic}')
        self.get_logger().info('Publicando: /lane_error y /lane/debug_image')

    def largest_contour_x(self, mask):
        min_area = float(self.get_parameter('min_area').value)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None

        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)

        if area < min_area:
            return None, None

        M = cv2.moments(c)
        if M['m00'] == 0:
            return None, None

        cx = int(M['m10'] / M['m00'])
        return cx, c

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {e}')
            return

        h, w = frame.shape[:2]

        roi_ratio = float(self.get_parameter('roi_y_start_ratio').value)
        y0 = int(h * roi_ratio)
        roi = frame[y0:h, 0:w]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        white_low = np.array(self.get_parameter('white_low').value, dtype=np.uint8)
        white_high = np.array(self.get_parameter('white_high').value, dtype=np.uint8)
        yellow_low = np.array(self.get_parameter('yellow_low').value, dtype=np.uint8)
        yellow_high = np.array(self.get_parameter('yellow_high').value, dtype=np.uint8)

        mask_white = cv2.inRange(hsv, white_low, white_high)
        mask_yellow = cv2.inRange(hsv, yellow_low, yellow_high)

        kernel = np.ones((5, 5), np.uint8)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, kernel)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)
        mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)

        x_white, c_white = self.largest_contour_x(mask_white)
        x_yellow, c_yellow = self.largest_contour_x(mask_yellow)

        lane_width_m = float(self.get_parameter('lane_width_m').value)
        px_per_meter = float(self.get_parameter('px_per_meter').value)
        lane_width_px = lane_width_m * px_per_meter

        center_px = None

        if x_white is not None and x_yellow is not None:
            center_px = (x_white + x_yellow) / 2.0
        elif x_white is not None:
            center_px = x_white - lane_width_px / 2.0
        elif x_yellow is not None:
            center_px = x_yellow + lane_width_px / 2.0

        if center_px is None:
            error_m = float('nan')
        else:
            error_m = (center_px - (w / 2.0)) / px_per_meter

        error_msg = Float32()
        error_msg.data = float(error_m)
        self.pub_error.publish(error_msg)

        debug = frame.copy()
        cv2.line(debug, (0, y0), (w, y0), (255, 0, 255), 2)
        cv2.line(debug, (w // 2, y0), (w // 2, h), (255, 0, 0), 2)
        cv2.putText(debug, 'CAM CENTER', (w // 2 - 80, max(20, y0 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        if c_white is not None:
            c_white_draw = c_white + np.array([[[0, y0]]])
            cv2.drawContours(debug, [c_white_draw], -1, (255, 255, 255), 2)
            cv2.circle(debug, (int(x_white), y0 + roi.shape[0] // 2), 6, (255, 255, 255), -1)
            cv2.putText(debug, 'WHITE', (int(x_white), y0 + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        if c_yellow is not None:
            c_yellow_draw = c_yellow + np.array([[[0, y0]]])
            cv2.drawContours(debug, [c_yellow_draw], -1, (0, 255, 255), 2)
            cv2.circle(debug, (int(x_yellow), y0 + roi.shape[0] // 2), 6, (0, 255, 255), -1)
            cv2.putText(debug, 'YELLOW', (int(x_yellow), y0 + 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

        if center_px is not None:
            cv2.line(debug, (int(center_px), y0), (int(center_px), h), (0, 255, 0), 2)
            cv2.putText(debug, f'error_m={error_m:.3f}', (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
        else:
            cv2.putText(debug, 'NO LANE DETECTED', (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            debug_msg.header = msg.header
            self.pub_debug.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Error publicando debug: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
