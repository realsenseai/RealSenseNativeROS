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
D555 Camera 3D Reconstruction Application

Features:
- 3D RGB textured view (rotating RGB point cloud)
- 3D Depth colormap view (rotating depth cloud)
- Multi-panel metadata display
- Data recording & cumulative 3D reconstruction
- All English UI with ASCII characters

Usage:
    python3 d555_3d_reconstruction_v4.py -gui

Controls:
    Q/ESC   - Quit
    R       - Start/Stop Recording
    B       - Start 3D Reconstruction (Build)
    C       - Clear accumulated point cloud
    S       - Save current 3D model to PLY file
    SPACE   - Pause/Resume display
    +/-     - Zoom in/out
    1/2     - Rotation speed

"""

import argparse
import json
import os
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Optional, Tuple
import pickle

import numpy as np

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# ROS2 message types
from sensor_msgs.msg import Image, CameraInfo, Imu
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String

# OpenCV for visualization
try:
    import cv2
    OPENCV_AVAILABLE = True
except ImportError:
    OPENCV_AVAILABLE = False
    print("[ERROR] OpenCV required! Install: pip3 install opencv-python")
    sys.exit(1)

# Open3D for 3D processing (optional)
try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False


@dataclass
class CameraIntrinsics:
    """Camera intrinsic parameters."""
    width: int = 0
    height: int = 0
    fx: float = 0.0
    fy: float = 0.0
    cx: float = 0.0
    cy: float = 0.0
    distortion_model: str = ""
    distortion_coeffs: List[float] = field(default_factory=list)

    def is_valid(self) -> bool:
        return self.width > 0 and self.height > 0 and self.fx > 0 and self.fy > 0


@dataclass
class Transform:
    """3D transform."""
    parent_frame: str = ""
    child_frame: str = ""
    translation: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rotation: np.ndarray = field(
        default_factory=lambda: np.array([0, 0, 0, 1]))


class D555ReconstructionV4(Node):
    """D555 3D Reconstruction Node."""

    def __init__(self, enable_gui: bool = False):
        super().__init__('d555_3d_reconstruction_v4')

        self.enable_gui = enable_gui
        self.running = True

        # Camera intrinsics
        self.depth_intrinsics = CameraIntrinsics()
        self.color_intrinsics = CameraIntrinsics()

        # Transforms
        self.transforms: Dict[str, Transform] = {}

        # Frame buffers (protected by _frame_lock)
        self._frame_lock = threading.Lock()
        self.depth_frame: Optional[np.ndarray] = None
        self.color_frame: Optional[np.ndarray] = None
        self.depth_timestamp = 0.0
        self.color_timestamp = 0.0
        self.depth_frame_counter = 0
        self.color_frame_counter = 0

        # IMU data
        self.imu_accel = np.zeros(3)
        self.imu_gyro = np.zeros(3)
        self.imu_timestamp = 0.0

        # Metadata
        self.metadata: Dict[str, dict] = {}

        # Statistics
        self.stats = {
            'depth_frames': 0, 'color_frames': 0, 'imu_samples': 0,
            'point_clouds': 0, 'start_time': time.time()
        }

        # Recording
        self.is_recording = False
        self.recorded_frames = []
        self.record_start_time = 0.0

        # 3D Reconstruction
        self.is_reconstructing = False
        self.accumulated_points: List[np.ndarray] = []
        self.accumulated_colors: List[np.ndarray] = []
        self.reconstruction_frame_count = 0

        # Point clouds (with colors)
        self.current_points = None
        self.current_rgb_colors = None  # RGB colors from camera
        self.current_depth_colors = None  # Depth-based colors
        self.pointcloud_lock = threading.Lock()

        # QoS
        self.best_effort_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        # Note: tf_static uses BEST_EFFORT for SafeDDS/FastRTPS interoperability
        self.tf_static_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Device
        self.device_serial = None
        self.topic_prefix = None

        self.get_logger().info("D555 3D Reconstruction starting...")
        self._discover_device()

    def _discover_device(self):
        """Discover D555 device."""
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            topics = self.get_topic_names_and_types()

            for topic_name, _ in topics:
                if '/realsense/D555_' in topic_name and '_Depth' in topic_name:
                    parts = topic_name.split('_Depth')[0]
                    self.topic_prefix = parts
                    self.device_serial = parts.split('_')[-1]
                    break

            if self.device_serial:
                break

        if self.device_serial:
            self.get_logger().info(f"Found D555: {self.device_serial}")
            self._setup_subscriptions()
        else:
            self.get_logger().error("No D555 camera found!")

    def _setup_subscriptions(self):
        """Setup all subscriptions."""
        prefix = self.topic_prefix

        self.create_subscription(
            Image, f'{prefix}_Depth', self._depth_cb, self.best_effort_qos)
        self.create_subscription(
            Image, f'{prefix}_Color', self._color_cb, self.best_effort_qos)
        self.create_subscription(
            CameraInfo, f'{prefix}_Depth/camera_info', self._depth_info_cb, self.best_effort_qos)
        self.create_subscription(
            CameraInfo, f'{prefix}_Color/camera_info', self._color_info_cb, self.best_effort_qos)
        self.create_subscription(
            Imu, f'{prefix}_Motion', self._imu_cb, self.best_effort_qos)
        self.create_subscription(
            TFMessage, f'{prefix}/tf_static', self._tf_cb, self.tf_static_qos)
        self.get_logger().info(
            f"Subscribed to {prefix}/tf_static with BEST_EFFORT + TRANSIENT_LOCAL")

        # Per-stream metadata topics publish JSON when the matching stream has
        # an active subscriber.
        for stream in ['Depth', 'Color', 'Infrared_1', 'Infrared_2', 'Motion']:
            self.create_subscription(
                String, f'{prefix}_{stream}/metadata',
                lambda msg, s=stream: self._metadata_cb(msg, s),
                self.best_effort_qos
            )

        self.create_timer(1.0/30.0, self._process_frame)

    def _depth_cb(self, msg: Image):
        try:
            if msg.encoding == '16UC1':
                depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                    (msg.height, msg.width))
            elif msg.encoding == '32FC1':
                depth = (np.frombuffer(msg.data, dtype=np.float32).reshape(
                    (msg.height, msg.width)) * 1000).astype(np.uint16)
            else:
                return

            with self._frame_lock:
                self.depth_frame = depth
                self.depth_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self.depth_frame_counter += 1
                self.stats['depth_frames'] += 1
                color_snap = self.color_frame.copy() if self.color_frame is not None else None

            if self.is_recording:
                self.recorded_frames.append({
                    'ts': self.depth_timestamp,
                    'depth': depth.copy(),
                    'color': color_snap
                })
        except Exception as e:
            self.get_logger().error(f"Depth error: {e}")

    def _color_cb(self, msg: Image):
        try:
            if msg.encoding in ['rgb8', 'bgr8']:
                color = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    (msg.height, msg.width, 3))
                if msg.encoding == 'bgr8':
                    color = color[:, :, ::-1]
            elif msg.encoding in ['yuv422_yuy2', 'yuyv']:
                yuv = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    (msg.height, msg.width, 2))
                y = yuv[:, :, 0].astype(np.float32)
                u = np.repeat(yuv[:, 0::2, 1].astype(np.float32), 2, axis=1)
                v = np.repeat(yuv[:, 1::2, 1].astype(np.float32), 2, axis=1)
                r = np.clip(y + 1.402 * (v - 128), 0, 255)
                g = np.clip(y - 0.344136 * (u - 128) -
                            0.714136 * (v - 128), 0, 255)
                b = np.clip(y + 1.772 * (u - 128), 0, 255)
                color = np.stack([r, g, b], axis=-1).astype(np.uint8)
            else:
                return

            with self._frame_lock:
                self.color_frame = color
                self.color_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self.color_frame_counter += 1
                self.stats['color_frames'] += 1
        except Exception as e:
            self.get_logger().error(f"Color error: {e}")

    def _depth_info_cb(self, msg: CameraInfo):
        self.depth_intrinsics.width = msg.width
        self.depth_intrinsics.height = msg.height
        self.depth_intrinsics.fx = msg.k[0]
        self.depth_intrinsics.fy = msg.k[4]
        self.depth_intrinsics.cx = msg.k[2]
        self.depth_intrinsics.cy = msg.k[5]
        self.depth_intrinsics.distortion_model = msg.distortion_model
        self.depth_intrinsics.distortion_coeffs = list(msg.d)

    def _color_info_cb(self, msg: CameraInfo):
        self.color_intrinsics.width = msg.width
        self.color_intrinsics.height = msg.height
        self.color_intrinsics.fx = msg.k[0]
        self.color_intrinsics.fy = msg.k[4]
        self.color_intrinsics.cx = msg.k[2]
        self.color_intrinsics.cy = msg.k[5]

    def _imu_cb(self, msg: Imu):
        self.imu_accel = np.array(
            [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        self.imu_gyro = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self.imu_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.stats['imu_samples'] += 1

    def _tf_cb(self, msg: TFMessage):
        for tf in msg.transforms:
            t = Transform(
                parent_frame=tf.header.frame_id,
                child_frame=tf.child_frame_id,
                translation=np.array(
                    [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]),
                rotation=np.array([tf.transform.rotation.x, tf.transform.rotation.y,
                                  tf.transform.rotation.z, tf.transform.rotation.w])
            )
            self.transforms[tf.child_frame_id] = t

    def _metadata_cb(self, msg: String, stream: str):
        try:
            data = json.loads(msg.data)
            self.metadata[stream] = data.get('metadata', data)
        except Exception:
            pass

    def _process_frame(self):
        """Generate point cloud with both RGB and depth colors."""
        with self._frame_lock:
            if self.depth_frame is None or not self.depth_intrinsics.is_valid():
                return
            depth = self.depth_frame.copy()
            color_snap = self.color_frame.copy() if self.color_frame is not None else None

        try:
            intrinsics = self.depth_intrinsics

            h, w = depth.shape
            u, v = np.meshgrid(np.arange(w), np.arange(h))
            z = depth.astype(np.float32) / 1000.0

            valid = (z > 0.1) & (z < 10.0)

            x = (u - intrinsics.cx) * z / intrinsics.fx
            y = (v - intrinsics.cy) * z / intrinsics.fy

            points = np.stack([x, y, z], axis=-1)[valid]

            if len(points) < 100:
                return

            # RGB colors from camera
            rgb_colors = None
            if color_snap is not None:
                color_h, color_w = color_snap.shape[:2]
                scale_u, scale_v = color_w / w, color_h / h
                cu = np.clip((u[valid] * scale_u).astype(int), 0, color_w - 1)
                cv = np.clip((v[valid] * scale_v).astype(int), 0, color_h - 1)
                rgb_colors = color_snap[cv, cu].astype(np.float32) / 255.0

            # Depth-based turbo colors
            z_valid = z[valid]
            z_norm = (z_valid - 0.1) / 4.9
            z_norm = np.clip(z_norm, 0, 1)
            z_uint8 = (z_norm * 255).astype(np.uint8)
            depth_colors_bgr = cv2.applyColorMap(
                z_uint8.reshape(-1, 1), cv2.COLORMAP_TURBO).reshape(-1, 3)
            # BGR to RGB
            depth_colors = depth_colors_bgr[:, ::-1].astype(np.float32) / 255.0

            # Subsample
            if len(points) > 100000:
                idx = np.random.choice(len(points), 100000, replace=False)
                points = points[idx]
                if rgb_colors is not None:
                    rgb_colors = rgb_colors[idx]
                depth_colors = depth_colors[idx]

            with self.pointcloud_lock:
                self.current_points = points
                self.current_rgb_colors = rgb_colors
                self.current_depth_colors = depth_colors

            self.stats['point_clouds'] += 1

            if self.is_reconstructing:
                self.accumulated_points.append(points.copy())
                if rgb_colors is not None:
                    self.accumulated_colors.append(rgb_colors.copy())
                self.reconstruction_frame_count += 1

        except Exception as e:
            self.get_logger().error(f"Process error: {e}")

    def start_recording(self):
        self.is_recording = True
        self.recorded_frames = []
        self.record_start_time = time.time()

    def stop_recording(self):
        self.is_recording = False
        if self.recorded_frames:
            filename = f"recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}.pkl"
            with open(filename, 'wb') as f:
                pickle.dump(self.recorded_frames, f)
            self.get_logger().info(
                f"Saved {len(self.recorded_frames)} frames to {filename}")

    def start_reconstruction(self):
        self.is_reconstructing = True
        self.accumulated_points = []
        self.accumulated_colors = []
        self.reconstruction_frame_count = 0

    def stop_reconstruction(self):
        self.is_reconstructing = False

    def clear_reconstruction(self):
        self.accumulated_points = []
        self.accumulated_colors = []
        self.reconstruction_frame_count = 0

    def save_reconstruction(self):
        if not OPEN3D_AVAILABLE or not self.accumulated_points:
            return

        all_points = np.vstack(self.accumulated_points)
        all_colors = np.vstack(
            self.accumulated_colors) if self.accumulated_colors else None

        if len(all_points) > 1000000:
            idx = np.random.choice(len(all_points), 1000000, replace=False)
            all_points = all_points[idx]
            if all_colors is not None:
                all_colors = all_colors[idx]

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_points)
        if all_colors is not None:
            pcd.colors = o3d.utility.Vector3dVector(all_colors)

        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        filename = f"reconstruction_{datetime.now().strftime('%Y%m%d_%H%M%S')}.ply"
        o3d.io.write_point_cloud(filename, pcd)
        self.get_logger().info(f"Saved {len(pcd.points)} points to {filename}")

    def shutdown(self):
        self.running = False


class ProGUI:
    """Professional GUI with 3D RGB and Depth views."""

    def __init__(self, node: D555ReconstructionV4):
        self.node = node
        self.running = True
        self.paused = False

        # Window
        self.window_name = "D555 3D Reconstruction"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1920, 1080)

        # 3D view params
        self.angle_rgb = 0
        self.angle_depth = 180  # Start from different angle
        self.zoom = 1.0
        self.rotation_speed = 0.8

        # FPS
        self.fps = 0
        self.frame_count = 0
        self.last_fps_time = time.time()

        # Colors
        self.BG = (20, 22, 25)
        self.PANEL_BG = (35, 38, 42)
        self.TEXT = (200, 200, 200)
        self.ACCENT = (255, 180, 100)
        self.GREEN = (100, 255, 150)
        self.RED = (100, 100, 255)

    def _render_3d_pointcloud(self, width: int, height: int, points: np.ndarray,
                              colors: np.ndarray, angle: float, title: str,
                              bg_color=(20, 22, 25)) -> np.ndarray:
        """Render 3D point cloud with rotation."""
        img = np.full((height, width, 3), bg_color, dtype=np.uint8)

        if points is None or len(points) < 100:
            cv2.putText(img, "Waiting for data...", (width//4, height//2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.TEXT, 1)
            self._draw_header(img, title, width)
            return img

        # Rotation matrix (Y-axis)
        angle_rad = np.radians(angle)
        cos_a, sin_a = np.cos(angle_rad), np.sin(angle_rad)
        rot = np.array([[cos_a, 0, sin_a], [0, 1, 0], [-sin_a, 0, cos_a]])

        # Center and rotate
        centroid = np.mean(points, axis=0)
        pts = (points - centroid) @ rot.T

        # Perspective projection
        focal = 350 * self.zoom
        z_offset = 2.5 / self.zoom
        z = pts[:, 2] + z_offset
        z = np.clip(z, 0.1, 100)

        u = (pts[:, 0] * focal / z + width / 2).astype(int)
        v = (pts[:, 1] * focal / z + height / 2).astype(int)

        valid = (u >= 0) & (u < width) & (v >= 0) & (v < height)
        u, v, z_draw = u[valid], v[valid], z[valid]

        # Sort by depth (far to near) for proper occlusion
        sort_idx = np.argsort(-z_draw)
        u, v = u[sort_idx], v[sort_idx]

        # Get colors
        if colors is not None and len(colors) == len(points):
            c = (colors[valid][sort_idx] * 255).astype(np.uint8)
        else:
            c = np.full((len(u), 3), 200, dtype=np.uint8)

        # Draw points (vectorized pixel assignment for performance)
        max_points = min(len(u), 100000)
        valid_u = u[:max_points]
        valid_v = v[:max_points]
        img[valid_v, valid_u] = c[:max_points, ::-1]  # RGB→BGR

        # Header
        self._draw_header(img, title, width)

        # Stats
        cv2.putText(img, f"Points: {len(points):,}", (10, height - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1)
        cv2.putText(img, f"Angle: {angle:.0f} deg", (10, height - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1)

        return img

    def _draw_header(self, img: np.ndarray, title: str, width: int):
        """Draw panel header."""
        cv2.rectangle(img, (0, 0), (width, 28), self.ACCENT, -1)
        cv2.putText(img, title, (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)

    def _apply_turbo(self, depth: np.ndarray) -> np.ndarray:
        """Apply turbo colormap."""
        d = np.clip(depth.astype(np.float32) / 1000.0, 0.1, 5.0)
        d_norm = ((d - 0.1) / 4.9 * 255).astype(np.uint8)
        colored = cv2.applyColorMap(d_norm, cv2.COLORMAP_TURBO)
        colored[(depth < 100) | (depth > 5000)] = [0, 0, 0]
        return colored

    def _render_info_panel(self, width: int, height: int) -> np.ndarray:
        """Render camera info panel."""
        img = np.full((height, width, 3), self.PANEL_BG, dtype=np.uint8)
        self._draw_header(img, "[CAM] Camera Intrinsics", width)

        y = 45
        lh = 18

        # Depth
        cv2.putText(img, "DEPTH CAMERA", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, self.ACCENT, 1)
        y += lh + 2

        di = self.node.depth_intrinsics
        lines = [
            f"Resolution: {di.width} x {di.height}",
            f"Focal: fx={di.fx:.1f} fy={di.fy:.1f}",
            f"Center: cx={di.cx:.1f} cy={di.cy:.1f}",
            f"Model: {di.distortion_model or 'N/A'}",
        ]
        for line in lines:
            cv2.putText(img, line, (15, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, self.TEXT, 1)
            y += lh - 2

        y += 8

        # Color
        cv2.putText(img, "COLOR CAMERA", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, self.ACCENT, 1)
        y += lh + 2

        ci = self.node.color_intrinsics
        lines = [
            f"Resolution: {ci.width} x {ci.height}",
            f"Focal: fx={ci.fx:.1f} fy={ci.fy:.1f}",
            f"Center: cx={ci.cx:.1f} cy={ci.cy:.1f}",
        ]
        for line in lines:
            cv2.putText(img, line, (15, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, self.TEXT, 1)
            y += lh - 2

        return img

    def _render_metadata_panel(self, width: int, height: int) -> np.ndarray:
        """Render metadata panel."""
        img = np.full((height, width, 3), self.PANEL_BG, dtype=np.uint8)
        self._draw_header(img, "[META] Stream Metadata", width)

        y = 45
        lh = 16

        for stream, data in list(self.node.metadata.items())[:4]:
            cv2.putText(img, stream.upper(), (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.ACCENT, 1)
            y += lh

            fields = ['Frame Counter', 'Actual Exposure',
                      'Gain Level', 'ASIC Temperature']
            for f in fields:
                val = data.get(f, data.get(f.lower().replace(' ', '_'), 'N/A'))
                if val != 'N/A':
                    unit = 'C' if 'Temp' in f else ('us' if 'Exp' in f else '')
                    cv2.putText(img, f"  {f}: {val}{unit}", (10, y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.32, self.TEXT, 1)
                    y += lh - 3
            y += 5

            if y > height - 30:
                break

        if not self.node.metadata:
            cv2.putText(img, "Waiting for metadata...", (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 80, 80), 1)

        return img

    def _render_tf_panel(self, width: int, height: int) -> np.ndarray:
        """Render TF panel."""
        img = np.full((height, width, 3), self.PANEL_BG, dtype=np.uint8)
        self._draw_header(img, "[TF] Static Transforms", width)

        y = 45
        lh = 18

        for name, tf in list(self.node.transforms.items())[:6]:
            short = name.replace('camera_', '').replace(
                '_optical_frame', '').replace('_frame', '')
            t = tf.translation
            cv2.putText(img, f"{short}:", (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, self.ACCENT, 1)
            cv2.putText(img, f"[{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}]", (90, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, self.TEXT, 1)
            y += lh

        if not self.node.transforms:
            cv2.putText(img, "Waiting for tf_static...", (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 80, 80), 1)

        return img

    def _render_imu_panel(self, width: int, height: int) -> np.ndarray:
        """Render IMU panel."""
        img = np.full((height, width, 3), self.PANEL_BG, dtype=np.uint8)
        self._draw_header(img, "[IMU] Motion Sensor", width)

        y = 45
        lh = 20

        cv2.putText(img, "ACCELEROMETER (m/s2)", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.ACCENT, 1)
        y += lh

        a = self.node.imu_accel
        cv2.putText(img, f"  X: {a[0]:+8.4f}", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, self.TEXT, 1)
        y += lh - 4
        cv2.putText(img, f"  Y: {a[1]:+8.4f}", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, self.TEXT, 1)
        y += lh - 4
        cv2.putText(img, f"  Z: {a[2]:+8.4f}", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, self.TEXT, 1)
        y += lh + 5

        cv2.putText(img, "GYROSCOPE (rad/s)", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.ACCENT, 1)
        y += lh

        g = self.node.imu_gyro
        cv2.putText(img, f"  X: {g[0]:+8.6f}", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, self.TEXT, 1)
        y += lh - 4
        cv2.putText(img, f"  Y: {g[1]:+8.6f}", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, self.TEXT, 1)
        y += lh - 4
        cv2.putText(img, f"  Z: {g[2]:+8.6f}", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, self.TEXT, 1)

        return img

    def _render_stats_panel(self, width: int, height: int) -> np.ndarray:
        """Render stats panel."""
        img = np.full((height, width, 3), self.PANEL_BG, dtype=np.uint8)
        self._draw_header(img, "[STAT] Performance & Controls", width)

        y = 45
        lh = 18

        elapsed = time.time() - self.node.stats['start_time']
        dfps = self.node.stats['depth_frames'] / elapsed if elapsed > 0 else 0
        cfps = self.node.stats['color_frames'] / elapsed if elapsed > 0 else 0
        imu_rate = self.node.stats['imu_samples'] / \
            elapsed if elapsed > 0 else 0

        stats = [
            f"Runtime: {elapsed:.1f}s",
            f"Depth FPS: {dfps:.1f}",
            f"Color FPS: {cfps:.1f}",
            f"IMU Rate: {imu_rate:.1f} Hz",
            f"Display FPS: {self.fps:.1f}",
            "",
            f"Depth TS: {self.node.depth_timestamp:.3f}",
            f"Color TS: {self.node.color_timestamp:.3f}",
            f"Depth Frame#: {self.node.depth_frame_counter}",
            f"Color Frame#: {self.node.color_frame_counter}",
        ]

        for s in stats:
            if s:
                cv2.putText(img, s, (10, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, self.TEXT, 1)
            y += lh - 3

        y += 10

        # Status
        if self.node.is_recording:
            cv2.putText(img, f"* REC ({len(self.node.recorded_frames)} frames)",
                        (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.RED, 1)
        elif self.node.is_reconstructing:
            cv2.putText(img, f"* BUILD ({self.node.reconstruction_frame_count} frames)",
                        (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.GREEN, 1)
        y += lh + 5

        # Controls
        cv2.putText(img, "CONTROLS:", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.ACCENT, 1)
        y += lh

        ctrls = ["R=Record B=Build", "C=Clear S=Save",
                 "SPACE=Pause Q=Quit", "+/-=Zoom 1/2=Speed"]
        for c in ctrls:
            cv2.putText(img, c, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.32, (150, 150, 150), 1)
            y += lh - 5

        return img

    def update(self) -> bool:
        """Update display."""
        self.frame_count += 1

        now = time.time()
        if now - self.last_fps_time > 1.0:
            self.fps = self.frame_count / (now - self.last_fps_time)
            self.frame_count = 0
            self.last_fps_time = now

        if self.paused:
            key = cv2.waitKey(30) & 0xFF
            return self._handle_key(key)

        # Update angles
        self.angle_rgb = (self.angle_rgb + self.rotation_speed) % 360
        self.angle_depth = (self.angle_depth + self.rotation_speed) % 360

        # Layout
        # Row 1: 3D RGB | 3D Depth | 2D RGB | 2D Depth
        # Row 2: CamInfo | Metadata | TF | IMU | Stats

        cell_w, cell_h = 480, 360
        panel_h = 280

        # Get point cloud data
        with self.node.pointcloud_lock:
            points = self.node.current_points
            rgb_colors = self.node.current_rgb_colors
            depth_colors = self.node.current_depth_colors

        # Top row views
        # 3D RGB textured
        view_3d_rgb = self._render_3d_pointcloud(
            cell_w, cell_h, points, rgb_colors, self.angle_rgb,
            "[3D] RGB Point Cloud", (15, 20, 25)
        )

        # 3D Depth colormap
        view_3d_depth = self._render_3d_pointcloud(
            cell_w, cell_h, points, depth_colors, self.angle_depth,
            "[3D] Depth Point Cloud", (20, 15, 25)
        )

        # 2D RGB
        if self.node.color_frame is not None:
            rgb_2d = cv2.resize(self.node.color_frame, (cell_w, cell_h))
            rgb_2d = cv2.cvtColor(rgb_2d, cv2.COLOR_RGB2BGR)
        else:
            rgb_2d = np.full((cell_h, cell_w, 3), self.BG, dtype=np.uint8)
            cv2.putText(rgb_2d, "Waiting for RGB...", (cell_w//6, cell_h//2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.TEXT, 1)
        self._draw_header(rgb_2d, "[2D] RGB Camera", cell_w)

        # 2D Depth
        if self.node.depth_frame is not None:
            depth_2d = self._apply_turbo(self.node.depth_frame)
            depth_2d = cv2.resize(depth_2d, (cell_w, cell_h))
            # Colorbar
            bar = np.linspace(0, 255, cell_h - 30).astype(np.uint8)
            bar = np.tile(bar.reshape(-1, 1), (1, 20))
            bar_c = cv2.applyColorMap(bar, cv2.COLORMAP_TURBO)[::-1]
            depth_2d[30:, -25:-5] = bar_c
        else:
            depth_2d = np.full((cell_h, cell_w, 3), self.BG, dtype=np.uint8)
            cv2.putText(depth_2d, "Waiting for Depth...", (cell_w//6, cell_h//2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.TEXT, 1)
        self._draw_header(depth_2d, "[2D] Depth Colormap", cell_w)

        top_row = np.hstack([view_3d_rgb, view_3d_depth, rgb_2d, depth_2d])

        # Bottom row panels
        pw = cell_w * 4 // 5  # Panel width

        p1 = self._render_info_panel(pw, panel_h)
        p2 = self._render_metadata_panel(pw, panel_h)
        p3 = self._render_tf_panel(pw, panel_h)
        p4 = self._render_imu_panel(pw, panel_h)
        p5 = self._render_stats_panel(pw, panel_h)

        bottom_row = np.hstack([p1, p2, p3, p4, p5])

        # Resize bottom to match top width
        total_w = top_row.shape[1]
        bottom_row = cv2.resize(bottom_row, (total_w, panel_h))

        # Combine
        combined = np.vstack([top_row, bottom_row])

        # Title bar
        cv2.rectangle(combined, (0, 0),
                      (combined.shape[1], 30), (15, 15, 18), -1)
        title = f"D555 3D Reconstruction | Device: {self.node.device_serial or 'N/A'} | FPS: {self.fps:.0f} | Zoom: {self.zoom:.1f}x"
        cv2.putText(combined, title, (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.ACCENT, 1)

        # Status indicators
        sx = combined.shape[1] - 350
        if self.node.is_recording:
            cv2.circle(combined, (sx, 15), 8, (0, 0, 255), -1)
            cv2.putText(combined, "REC", (sx + 15, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        if self.node.is_reconstructing:
            cv2.circle(combined, (sx + 80, 15), 8, (0, 255, 0), -1)
            cv2.putText(combined, "BUILD", (sx + 95, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        if self.paused:
            cv2.putText(combined, "PAUSED", (sx + 180, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

        cv2.imshow(self.window_name, combined)

        key = cv2.waitKey(1) & 0xFF
        return self._handle_key(key)

    def _handle_key(self, key: int) -> bool:
        if key == ord('q') or key == 27:
            return False
        elif key == ord('r'):
            if self.node.is_recording:
                self.node.stop_recording()
            else:
                self.node.start_recording()
        elif key == ord('b'):
            if self.node.is_reconstructing:
                self.node.stop_reconstruction()
            else:
                self.node.start_reconstruction()
        elif key == ord('c'):
            self.node.clear_reconstruction()
        elif key == ord('s'):
            self.node.save_reconstruction()
        elif key == ord(' '):
            self.paused = not self.paused
        elif key == ord('+') or key == ord('='):
            self.zoom = min(3.0, self.zoom + 0.1)
        elif key == ord('-'):
            self.zoom = max(0.3, self.zoom - 0.1)
        elif key == ord('1'):
            self.rotation_speed = max(0.1, self.rotation_speed - 0.2)
        elif key == ord('2'):
            self.rotation_speed = min(3.0, self.rotation_speed + 0.2)
        return True

    def close(self):
        self.running = False
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description='D555 3D Reconstruction')
    parser.add_argument(
        '-gui', '--gui', action='store_true', help='Enable GUI')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = D555ReconstructionV4(enable_gui=args.gui)

    gui = ProGUI(node) if args.gui else None

    print("\n" + "="*70)
    print("   D555 3D Reconstruction")
    print("="*70)
    print(f"   GUI: {'Enabled' if args.gui else 'Disabled'}")
    print(f"   ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', '0')}")
    print("="*70)
    print("\n   R=Record B=Build C=Clear S=Save SPACE=Pause +/-=Zoom 1/2=Speed Q=Quit\n")

    try:
        if gui:
            while rclpy.ok() and node.running:
                rclpy.spin_once(node, timeout_sec=0.01)
                if not gui.update():
                    break
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("\nShutting down...")
    finally:
        node.shutdown()
        if gui:
            gui.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    print("Goodbye!")


if __name__ == '__main__':
    main()
