#!/usr/bin/env python3
import sys
import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import JointState


COLOR_RANGES = {
    'red': [
        ((0, 100, 70), (10, 255, 255)),
        ((170, 100, 70), (180, 255, 255)),
    ],
    'green': [
        ((35, 80, 70), (85, 255, 255)),
    ],
    'blue': [
        ((95, 100, 70), (135, 255, 255)),
    ],
    'yellow': [
        ((20, 100, 100), (35, 255, 255)),
    ],
}


class CameraTrackingNode(Node):
    def __init__(self):
        super().__init__('camera_tracking_node')

        self.declare_parameter('video_device', 0)
        self.declare_parameter('target_color', 'red')
        self.declare_parameter('show_window', True)
        self.declare_parameter('smooth_alpha', 0.25)

        self.dev_idx = self.get_parameter('video_device').value
        self.target_color = self.get_parameter('target_color').value
        self.show_window = self.get_parameter('show_window').value
        self.alpha = float(self.get_parameter('smooth_alpha').value)

        self.pose_pub = self.create_publisher(PoseStamped, '/camera_target_pose', 10)

        self.smoothed_x = -0.042
        self.smoothed_y = -0.237
        self.smoothed_z = 0.245
        self.detected = False

        self.get_logger().info(f'Opening Camera device index: {self.dev_idx}...')
        self.cap = cv2.VideoCapture(self.dev_idx)

        if not self.cap.isOpened():
            self.get_logger().warn(
                f'Could not open /dev/video{self.dev_idx}! In WSL2, please use the 3D Web Command Studio '
                f'(http://localhost:8080) for instant plug-and-play webcam access without USB driver setup.'
            )
        else:
            self.get_logger().info(f'Camera {self.dev_idx} opened successfully! Target color: {self.target_color}')
            self.timer = self.create_timer(1.0 / 30.0, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return

        frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        ranges = COLOR_RANGES.get(self.target_color, COLOR_RANGES['red'])

        mask = np.zeros((h, w), dtype=np.uint8)
        for lower, upper in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, np.array(lower), np.array(upper)))

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > 450:
                self.detected = True
                bx, by, bw, bh = cv2.boundingRect(c)
                cx = bx + bw // 2
                cy = by + bh // 2

                # Coordinate Mapping: 2D Pixel + Size -> Robot Workspace (m)
                norm_u = (cx / float(w)) - 0.5
                target_y = -norm_u * 0.36

                norm_v = 1.0 - (cy / float(h))
                target_z = 0.08 + norm_v * 0.20

                box_dim = max(bw, bh)
                frac = min(1.0, max(0.0, (box_dim - 25) / 115.0))
                target_x = -0.14 - (1.0 - frac) * 0.12

                # Exponential Moving Average Filter
                self.smoothed_x = (1.0 - self.alpha) * self.smoothed_x + self.alpha * target_x
                self.smoothed_y = (1.0 - self.alpha) * self.smoothed_y + self.alpha * target_y
                self.smoothed_z = (1.0 - self.alpha) * self.smoothed_z + self.alpha * target_z

                # Visual HUD annotations
                cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (248, 189, 56), 2)
                cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
                cv2.drawMarker(frame, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 20, 2)

                hud_text = f'X:{self.smoothed_x:.3f} Y:{self.smoothed_y:.3f} Z:{self.smoothed_z:.3f}m'
                cv2.putText(frame, hud_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f'Color: {self.target_color.upper()} (Area: {int(area)}px)', (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

                # Publish Target Pose
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                msg.pose.position = Point(x=float(self.smoothed_x), y=float(self.smoothed_y), z=float(self.smoothed_z))
                msg.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
                self.pose_pub.publish(msg)
            else:
                self.detected = False
        else:
            self.detected = False

        if not self.detected:
            cv2.putText(frame, 'Scanning for Cube...', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        if self.show_window:
            cv2.imshow('DJI Osmo Action 5 / Camera Tracking', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rclpy.shutdown()
            elif key == ord('r'):
                self.target_color = 'red'
            elif key == ord('g'):
                self.target_color = 'green'
            elif key == ord('b'):
                self.target_color = 'blue'
            elif key == ord('y'):
                self.target_color = 'yellow'

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
