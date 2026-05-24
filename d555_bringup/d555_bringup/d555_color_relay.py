#!/usr/bin/env python3
# Copyright 2026 RealSense
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
D555 Color Relay Node — converts YUV422/NV12 images from the D555 camera
to RGB8 for downstream ROS2 consumers (e.g. depth_image_proc point cloud).

The D555 publishes Color images in yuv422_yuy2 encoding to save Ethernet
bandwidth (2 bytes/pixel vs 3 for rgb8). This node subscribes with Best
Effort QoS (matching the device), converts via OpenCV, and republishes
as rgb8.

Supported input encodings:
  - yuv422_yuy2 → rgb8
  - nv12        → rgb8
  - bgr8        → rgb8
  - rgb8        → passthrough (no conversion)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class D555ColorRelay(Node):
    def __init__(self):
        super().__init__('d555_color_relay')

        self.declare_parameter('qos_reliability', 'best_effort')

        reliability = ReliabilityPolicy.BEST_EFFORT
        qos_param = self.get_parameter('qos_reliability').value
        if qos_param == 'reliable':
            reliability = ReliabilityPolicy.RELIABLE

        qos = QoSProfile(
            depth=5,
            reliability=reliability,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._bridge = CvBridge()
        self._pub = self.create_publisher(Image, '~/image_raw', 10)
        self._sub = self.create_subscription(
            Image, '~/image_raw_yuv', self._on_color, qos)

        self.get_logger().info(
            f'Color relay started (QoS: {qos_param})')

    def _on_color(self, msg: Image):
        encoding = msg.encoding.lower()

        if encoding == 'rgb8':
            # Already RGB8 — publish as-is (no mutation of input msg)
            self._pub.publish(msg)
            return

        # Convert to RGB8 via OpenCV
        cv_image = self._bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')

        if encoding == 'yuv422_yuy2':
            rgb = cv2.cvtColor(cv_image, cv2.COLOR_YUV2RGB_YUY2)
        elif encoding == 'nv12':
            rgb = cv2.cvtColor(cv_image, cv2.COLOR_YUV2RGB_NV12)
        elif encoding == 'yuyv':
            rgb = cv2.cvtColor(cv_image, cv2.COLOR_YUV2RGB_YUYV)
        elif encoding == 'bgr8':
            rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        else:
            self.get_logger().warn(
                f'Unknown encoding "{msg.encoding}", passing through',
                throttle_duration_sec=5.0)
            self._pub.publish(msg)
            return

        out = self._bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
        out.header = msg.header
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = D555ColorRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
