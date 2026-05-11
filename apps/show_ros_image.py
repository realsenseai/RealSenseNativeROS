#!/usr/bin/env python3
# Copyright 2026 Intel RealSense
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
"""D555 ROS2 stream viewer – GUI or headless (log-only) mode.

SETUP (Linux):
  source /opt/ros/humble/setup.bash
  Or use the wrapper:  ./run_show_ros_image.sh [args...]

Usage examples:
  # --- GUI mode (requires DISPLAY / X11 forwarding) ---
  python3 show_ros_image.py --gui --stream Color
  python3 show_ros_image.py --gui --serial 344522301530 --stream Depth+IR1

  # --- Headless mode (default, for automated testing / CI) ---
  # Prints status lines to stdout and writes a log file.
  python3 show_ros_image.py --serial 344522301530 --stream Color
  python3 show_ros_image.py --serial 344522301530 --stream Color --duration 30

  # --- Debug mode (adds 10s pcap capture at start) ---
  python3 show_ros_image.py --debug --serial 344522301530 --stream Color

  # Multiple streams ('+' or ',' separated)
  python3 show_ros_image.py --gui --stream Depth+IR1+IR2

Stream aliases:
  IR1 / IR2 / IR3  →  Infrared_1/2/3
  CompColor        →  CompressedColor

Output files (headless / debug):
  Log:   ros2-<SN>-<stream>-image-<YYYYMMDD_HHMMSS>.log
  Pcap:  ros2-<SN>-<stream>-image-<YYYYMMDD_HHMMSS>.pcap  (--debug only)
  Files are auto-deleted on success (HW FPS ≥ 29.5 and frames received).

GUI window layout:
  ┌──────────────────────────────────────────────┐
  │  SN:xxx  FW:7.58.xxx           (device row)  │
  │  IP:192.x  MTU:9000  Delay:0  Link:1000M     │
  ├────────────┬────────────┬────────────────────┤
  │ Stream:Dep │ Stream:IR1 │  Stream:Color       │
  │ Fmt.. HW.. │ Fmt.. HW..│  Fmt.. HW..         │
  │  <image>   │  <image>  │   <image>            │
  └────────────┴────────────┴────────────────────┘

Exit: 'q' / window X (GUI), Ctrl-C or --duration timeout (headless).
"""
import sys
import os
import re
import uuid
import argparse
import threading
import signal
import time
import subprocess
import collections
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String

import cv2
import numpy as np
from cv_bridge import CvBridge


# ══════════════════════════════════════════════════════════════════════════════
# Helper Functions
# ══════════════════════════════════════════════════════════════════════════════

def yuy2_to_bgr(frame, width: int, height: int):
    if frame.ndim == 2 and frame.shape[0] == height and frame.shape[1] == width * 2:
        frame = frame.reshape((height, width, 2))
    return cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUY2)


def normalize_to_u8(x: np.ndarray) -> np.ndarray:
    """Normalize any numeric array to uint8 for display."""
    f = x.astype(np.float32)
    mn, mx = float(np.nanmin(f)), float(np.nanmax(f))
    if not np.isfinite(mn) or not np.isfinite(mx) or mx <= mn:
        return np.zeros_like(f, dtype=np.uint8)
    f = (f - mn) * (255.0 / (mx - mn))
    return np.clip(f, 0, 255).astype(np.uint8)


def build_topic(serial: str, stream: str) -> str:
    return f"/realsense/D555_{serial}_{stream}"


def is_compressed_stream(stream: str, topic: str) -> bool:
    s = (stream or "").lower()
    t = (topic or "").lower()
    return ("compressed" in s) or ("compressed" in t)


def detect_serial() -> str:
    """Auto-detect first D555 node serial from ros2 node list."""
    try:
        out = subprocess.check_output(
            ["ros2", "node", "list"], text=True, timeout=5)
        m = re.search(r"/(?:D555)_(\d+)", out)
        if m:
            return m.group(1)
    except Exception:
        pass
    return ""


def detect_camera_nic(camera_ip: str) -> str:
    """Find the local NIC whose subnet contains camera_ip.

    Parses 'ip -br addr show' output to match the camera's IP subnet.
    Returns the NIC name (e.g. 'enp2s0') or empty string if not found.

    Example: camera_ip='192.168.11.55' → matches enp2s0 at 192.168.11.1/24.
    """
    if not camera_ip or camera_ip in ("?", ""):
        return ""
    import ipaddress
    try:
        cam = ipaddress.ip_address(camera_ip)
        out = subprocess.check_output(["ip", "-br", "addr", "show"],
                                      text=True, timeout=5)
        for line in out.splitlines():
            parts = line.split()
            if len(parts) < 3:
                continue
            nic, state = parts[0], parts[1]
            if state.upper() not in ("UP", "UNKNOWN"):
                continue
            for addr_str in parts[2:]:
                try:
                    iface_net = ipaddress.ip_interface(addr_str)
                    if cam in iface_net.network:
                        return nic
                except ValueError:
                    continue
    except Exception:
        pass
    return ""


def fetch_device_info(serial: str) -> dict:
    """Query 'ros2 param describe /D555_<serial> Device.Info' and build a
    dict with banner line strings for the overlay.

    Returns dict with keys 'sn_fw' and 'net':
      {'sn_fw': 'SN:xxx  FW:7.58.xxx',
       'net':   'IP:192.168.x.x  MTU:9000  Delay:0  Link:1000M'}

    On error or old FW (Type not set):
      {'sn_fw': 'SN:xxx  Update FW >=57.7 to show more info',
       'net':   '(device info unavailable)'}
    """
    node_name = f"/D555_{serial}"
    try:
        out = subprocess.check_output(
            ["ros2", "param", "describe", node_name, "Device.Info"],
            text=True, timeout=5, stderr=subprocess.DEVNULL
        )
    except Exception:
        return {
            "sn_fw": f"SN:{serial}  (device info unavailable)",
            "net":   ""
        }

    # Check for "Type: not set" — old FW that doesn't expose Device.Info
    if "not set" in out.lower():
        return {
            "sn_fw": f"SN:{serial}  Update FW >=57.7 to show more info",
            "net":   ""
        }

    # Extract JSON from the Description: line.
    # The output may be line-wrapped, so collapse it first.
    import json
    out_flat = " ".join(out.split())   # collapse all whitespace/newlines
    m = re.search(r"Description:\s*({.*?})\s*Constraints:", out_flat)
    if not m:
        # fallback: first '{' … last '}'
        m = re.search(r"({.*})", out_flat)
    if not m:
        return {
            "sn_fw": f"SN:{serial}  (device info parse error)",
            "net":   ""
        }
    try:
        info = json.loads(m.group(1))
    except json.JSONDecodeError:
        return {
            "sn_fw": f"SN:{serial}  (device info JSON error)",
            "net":   ""
        }

    serial_val = info.get("serial",      serial)
    fw = info.get("fw-version",  "?")
    ip = info.get("actual_ip",   "?")
    mtu = info.get("mtu",         "?")
    trans_delay = info.get("trans_delay", "?")
    link_speed = info.get("link_speed",  "?")
    # Return a dict so we can build multi-line banner later
    return {
        "sn_fw": f"SN:{serial_val}  FW:{fw}",
        "net":   f"IP:{ip}  MTU:{mtu}  Delay:{trans_delay}  Link:{link_speed}M",
    }


def _run_cmd(cmd: list[str], timeout_sec: float = 8.0) -> tuple[bool, str]:
    """Run command and return (ok, combined_output)."""
    try:
        proc = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout_sec,
            check=False,
        )
        out = (proc.stdout or "") + (proc.stderr or "")
        return proc.returncode == 0, out
    except Exception as e:
        return False, str(e)


def _parse_rs_dds_config_net() -> dict:
    """Parse IP/MTU/traffic delay from rs-dds-config output."""
    info = {"camera_ip": None, "device_mtu": None, "traffic_delay_us": None}
    ok, out = _run_cmd(["rs-dds-config"], timeout_sec=20.0)
    if not out:
        return info

    m_ip = re.search(r"configured:\s*([0-9]+(?:\.[0-9]+){3})\s*&", out)
    if m_ip:
        info["camera_ip"] = m_ip.group(1)

    m_mtu = re.search(r"MTU,\s*bytes:\s*(\d+)", out)
    if m_mtu:
        info["device_mtu"] = m_mtu.group(1)

    m_delay = re.search(
        r"transmission\s*delay,\s*us:\s*(-?\d+)", out, re.IGNORECASE)
    if m_delay:
        info["traffic_delay_us"] = m_delay.group(1)

    if not ok:
        print("[WARN] rs-dds-config returned non-zero; parsed values may be partial")
    return info


def _try_read_device_ufo_from_serial() -> str:
    """Best-effort read of UFO state from serial ifconfig output (optional)."""
    ok, acm_out = _run_cmd(
        ["bash", "-lc", "grep -i '^pu port' ~/.minirc.dfl | awk '{print $3}'"], timeout_sec=3.0)
    acm = acm_out.strip() if ok else ""
    if not acm:
        acm = "/dev/ttyACM0"

    cmd = (
        f"echo ifconfig | tee {acm}; "
        f"echo | tee {acm}; "
        "sleep 1; "
        "tail -200 ~/log/minicom_logs/latest.log"
    )
    ok, out = _run_cmd(["bash", "-lc", cmd], timeout_sec=8.0)
    if not ok or not out:
        return "unknown"

    matches = re.findall(
        r"UFO\s*\(UDP Fragmentation Offload\):\s*([A-Za-z]+)", out)
    return matches[-1] if matches else "unknown"


def check_stream_network_preconditions(device_info: dict) -> bool:
    """Dump device MTU/traffic_delay/UFO and verify host MTU matches."""
    camera_ip = None
    device_mtu = None
    traffic_delay_us = None

    net_line = device_info.get("net", "") or ""
    m_ip = re.search(r"IP:(\S+)", net_line)
    if m_ip:
        camera_ip = m_ip.group(1)
    m_mtu = re.search(r"MTU:(\d+)", net_line)
    if m_mtu:
        device_mtu = m_mtu.group(1)
    m_delay = re.search(r"Delay:([-]?\d+)", net_line)
    if m_delay:
        traffic_delay_us = m_delay.group(1)

    # Fallback to rs-dds-config if Device.Info did not include full fields
    rs_cfg = _parse_rs_dds_config_net()
    camera_ip = camera_ip or rs_cfg.get("camera_ip")
    device_mtu = device_mtu or rs_cfg.get("device_mtu")
    traffic_delay_us = traffic_delay_us or rs_cfg.get("traffic_delay_us")

    ufo_state = _try_read_device_ufo_from_serial()  # optional

    print("\n=== Pre-stream Network Check ===")
    print(f"Device IP            : {camera_ip or 'N/A'}")
    print(f"Device MTU           : {device_mtu or 'N/A'}")
    print(f"Device traffic_delay : {traffic_delay_us or 'N/A'} us")
    print(f"Device UFO (optional): {ufo_state}")

    if not camera_ip:
        print("[ERROR] Cannot resolve device IP from Device.Info or rs-dds-config")
        return False

    ok, route_out = _run_cmd(
        ["ip", "route", "get", camera_ip], timeout_sec=5.0)
    if not ok or not route_out:
        print(f"[ERROR] Cannot resolve host route to camera IP {camera_ip}")
        return False

    m_nic = re.search(r"\bdev\s+(\S+)", route_out)
    if not m_nic:
        print(
            f"[ERROR] Cannot parse host NIC from route output: {route_out.strip()}")
        return False
    host_nic = m_nic.group(1)

    ok, link_out = _run_cmd(
        ["ip", "-o", "link", "show", "dev", host_nic], timeout_sec=5.0)
    if not ok or not link_out:
        print(f"[ERROR] Cannot read MTU from host NIC {host_nic}")
        return False

    m_host_mtu = re.search(r"\bmtu\s+(\d+)", link_out)
    host_mtu = m_host_mtu.group(1) if m_host_mtu else None

    print(f"Host NIC             : {host_nic}")
    print(f"Host MTU             : {host_mtu or 'N/A'}")

    if not device_mtu or not host_mtu:
        print("[ERROR] Missing MTU value, cannot validate consistency")
        return False

    if str(device_mtu) != str(host_mtu):
        print(f"[ERROR] MTU mismatch: device={device_mtu}, host={host_mtu}")
        print("        After changing device MTU/UFO, reboot device and rerun stream test")
        return False

    print(f"[OK] MTU matched: {device_mtu}")
    return True


# Stream name aliases  (case-insensitive lookup)
_STREAM_ALIASES: dict[str, str] = {
    "ir1":          "Infrared_1",
    "ir2":          "Infrared_2",
    "ir3":          "Infrared_3",
    "infrared1":    "Infrared_1",
    "infrared2":    "Infrared_2",
    "infrared3":    "Infrared_3",
    "compcolor":    "CompressedColor",
    "compressedcolor": "CompressedColor",
    "color":        "Color",
    "depth":        "Depth",
    "motion":       "Motion",
}


def resolve_stream_name(name: str) -> str:
    """Expand alias → canonical stream name; unknown names pass through as-is."""
    return _STREAM_ALIASES.get(name.lower(), name)


# ══════════════════════════════════════════════════════════════════════════════
# Per-Stream ROS Subscriber (one per stream, all share same rclpy context)
# ══════════════════════════════════════════════════════════════════════════════

# ── Hardware FPS Measurement & Performance Isolation ────────────────────────────
#
# 【Measurement Algorithm】Sliding-window timestamp method:
#   Each frame arrival is recorded as time.time() in the ROS callback, stored in deque.
#   Timestamps older than the window are evicted. FPS is estimated as:
#     hw_fps = (window_frame_count - 1) / (newest_ts - oldest_ts)
#   Window length _HW_FPS_WINDOW = 2.0s contains ~60 samples at 30fps,
#   statistical error < 1%, sufficient to reflect true frame rate.
#
# 【Accuracy Guarantees】
#   • Timestamps captured at callback entry (not imshow), reflecting actual DDS delivery.
#   • Sliding window smooths short-term jitter; 2s window balances response & stability.
#   • No dependency on frame sequence numbers (fid), avoiding reorder/retransmit errors.
#   • Potential error sources: Python GIL + time.time() precision (~1µs on Linux, adequate);
#     if DDS QoS drops frames (BEST_EFFORT), hw_fps naturally decreases, reflecting reality.
#
# 【Decoupled from Display — No RX Performance Impact】
#   Architecture uses three-layer isolation:
#
#   1. Independent threads: Each stream has dedicated spin thread + SingleThreadedExecutor.
#      ROS callbacks execute in spin thread, UI rendering in main thread — fully parallel.
#
#   2. Single-slot buffer: Callback only writes st.latest (lock time < 5µs).
#      Main thread reads it ~10x per second (100ms interval). If UI is slow, callbacks
#      keep arriving and overwrite latest — hw_fps counter is never blocked by rendering.
#
#   3. Frame-rate downsampling: _DISPLAY_INTERVAL = 0.1s throttles imshow frequency.
#      cv2 imshow/waitKey involve X11/Qt system calls (~1-5ms), but downsampling
#      amortizes this cost across the render cycle — callback is unaffected.
#
#   Conclusion: hw_fps is computed in O(1) with timestamp append in callback;
#   display rendering is entirely outside the ROS critical path.
# ─────────────────────────────────────────────────────────────────────────────
# Sliding-window length (seconds); sample count = fps × 2
_HW_FPS_WINDOW = 2.0
# UI render interval (seconds), ~10fps display, independent of HW fps
_DISPLAY_INTERVAL = 0.1
# HW FPS must be >= this value for LIVE (green) status
_HW_FPS_OK_THRESHOLD = 29.5


class StreamState:
    """Thread-safe state shared between ROS callback and the UI thread."""

    __slots__ = ("lock", "latest", "count", "fmt", "meta",
                 "hw_fps", "_fps_buf", "first_cb_time", "last_cb_time",
                 "last_show", "dis_fps", "_dis_count", "_dis_t",
                 "closed", "od_detections", "od_count")

    def __init__(self):
        self.lock = threading.Lock()
        self.latest = None
        self.count = 0          # total callbacks fired (HW frames received)
        self.fmt = ""
        self.meta = ""
        self.hw_fps = 0.0
        self._fps_buf = collections.deque()   # timestamps of received frames
        # wall-clock time of first callback (None = never)
        self.first_cb_time = None
        self.last_cb_time = None   # wall-clock time of most recent callback
        # display-fps bookkeeping
        self.last_show = 0.0
        self.dis_fps = 0.0
        self._dis_count = 0
        self._dis_t = time.time()
        # set True when the window is closed by user (X button)
        self.closed = False
        # Object detection overlay data (populated when --od is used)
        self.od_detections = []  # list of detection dicts from v1 API JSON
        self.od_count = 0       # total OD messages received

    def record_hw_frame(self):
        """Record one arrived frame and update the sliding-window HW FPS.

        Called inside the ROS callback (spin thread), holding self.lock.

        Algorithm — sliding-window timestamp method:
          • Append current wall-clock time to _fps_buf (a deque).
          • Evict entries older than _HW_FPS_WINDOW seconds (O(1) amortised).
          • Estimate FPS as (N-1) / (t_last - t_first) where N = window size.
            Using N-1 intervals rather than N avoids fence-post over-counting.

        Decoupling from display:
          • This method runs in the spin thread; imshow runs in the main thread.
          • st.latest is overwritten every callback; the main thread reads it
            at ~10 fps.  If the UI is slow, callbacks keep arriving and
            overwriting latest — hw_fps is never blocked by rendering.
          • Lock hold time is O(window evictions) ~ a few µs at 30 fps,
            negligible compared to DDS/network inter-frame gap (~33 ms).
        """
        now = time.time()
        if self.first_cb_time is None:
            self.first_cb_time = now   # mark first-ever callback arrival
        self.last_cb_time = now
        self._fps_buf.append(now)
        # Evict samples outside the sliding window
        cutoff = now - _HW_FPS_WINDOW
        while self._fps_buf and self._fps_buf[0] < cutoff:
            self._fps_buf.popleft()
        n = len(self._fps_buf)
        # Need ≥2 timestamps to compute an interval
        if n > 1:
            self.hw_fps = (n - 1) / (self._fps_buf[-1] - self._fps_buf[0])
        else:
            self.hw_fps = 0.0
        self.count += 1

    def record_display_frame(self):
        """Call each time a frame is actually rendered to screen."""
        self._dis_count += 1
        now = time.time()
        dt = now - self._dis_t
        if dt >= 1.0:
            self.dis_fps = self._dis_count / dt
            self._dis_count = 0
            self._dis_t = now


class Viewer(Node):
    def __init__(self, topic: str, compressed: bool, node_name: str, state: StreamState,
                 od_topic: str = None):
        super().__init__(node_name)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE

        self.bridge = CvBridge()
        self.state = state
        self._sub = None
        self._od_sub = None

        if compressed:
            self._sub = self.create_subscription(
                CompressedImage, topic, self._cb_compressed, qos)
            self.get_logger().info(f"[{topic}] CompressedImage BEST_EFFORT")
        else:
            self._sub = self.create_subscription(
                Image, topic, self._cb_image, qos)
            self.get_logger().info(f"[{topic}] Image BEST_EFFORT")

        if od_topic:
            self._od_sub = self.create_subscription(
                String, od_topic, self._cb_od, qos)
            self.get_logger().info(f"[{od_topic}] OD overlay enabled")

    def unsubscribe(self):
        """Cancel the subscription so no more callbacks fire."""
        if self._sub is not None:
            try:
                self.destroy_subscription(self._sub)
            except Exception:
                pass
            self._sub = None
        if self._od_sub is not None:
            try:
                self.destroy_subscription(self._od_sub)
            except Exception:
                pass
            self._od_sub = None
            self.get_logger().info("subscription cancelled")

    # ── image decode ──────────────────────────────────────────────────────────

    @staticmethod
    def _decode(raw: np.ndarray, enc: str, bridge: CvBridge, msg) -> np.ndarray:
        if enc == "yuv422_yuy2":
            return yuy2_to_bgr(raw, msg.width, msg.height)
        if enc == "bgr8":
            return raw
        if enc == "rgb8":
            return cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
        if enc == "mono8":
            return cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
        if enc in ("mono16", "16uc1"):
            return cv2.cvtColor(normalize_to_u8(raw), cv2.COLOR_GRAY2BGR)
        if enc == "32fc1":
            return cv2.cvtColor(normalize_to_u8(raw), cv2.COLOR_GRAY2BGR)
        # fallback
        try:
            return bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            if getattr(raw, "ndim", 0) == 3 and raw.shape[-1] == 2:
                return cv2.cvtColor(raw, cv2.COLOR_YUV2BGR_YUY2)
            if getattr(raw, "ndim", 0) == 2:
                return cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
            return raw

    def _cb_image(self, msg: Image):
        st = self.state
        if st.closed:          # ← stop processing after window closed
            return
        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        enc = (msg.encoding or "").lower()
        disp = self._decode(raw, enc, self.bridge, msg)
        with st.lock:
            st.record_hw_frame()
            st.latest = disp
            st.fmt = enc
            st.meta = f"enc={msg.encoding} {msg.width}x{msg.height}"

    def _cb_compressed(self, msg: CompressedImage):
        st = self.state
        if st.closed:          # ← stop processing after window closed
            return
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().error(
                f"imdecode failed: format='{msg.format}' bytes={len(msg.data)}")
            return
        with st.lock:
            st.record_hw_frame()
            st.latest = img
            st.fmt = msg.format
            st.meta = f"format={msg.format} {img.shape[1]}x{img.shape[0]}"

    def _cb_od(self, msg: String):
        """Object detection topic callback — stores latest detections for overlay."""
        import json
        st = self.state
        if st.closed:
            return
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, ValueError):
            return
        with st.lock:
            st.od_detections = data.get("detections", [])
            st.od_count += 1


# ──────────────────────────────────────────────────────────────────────────────
# Object Detection overlay helpers
# ──────────────────────────────────────────────────────────────────────────────

_OD_CLASS_NAMES = {0: "Person", 1: "Vehicle",
                   2: "Box", 3: "Robot", 4: "Charger", 5: "Ladder"}
_OD_COLORS = [(0, 255, 0), (255, 0, 0), (0, 0, 255),
              (255, 255, 0), (0, 255, 255), (255, 0, 255)]


def _draw_od_overlay(img, detections, scale_x=1.0, scale_y=1.0):
    """Draw bounding boxes from OD detections onto img (in-place).

    scale_x/y: ratio of display size to original image size for coordinate mapping.
    Uses API field names: class_id, confidence, x1/y1/x2/y2, distance (float, meters).
    """
    for det in detections:
        cid = int(det.get("class_id", 0))
        conf = int(det.get("confidence", 0))
        x1 = int(det.get("x1", 0) * scale_x)
        y1 = int(det.get("y1", 0) * scale_y)
        x2 = int(det.get("x2", 0) * scale_x)
        y2 = int(det.get("y2", 0) * scale_y)
        dist = float(det.get("distance", 0.0))
        color = _OD_COLORS[cid % len(_OD_COLORS)]
        name = _OD_CLASS_NAMES.get(cid, f"cls{cid}")

        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        label = f"{name} {conf}%"
        if dist > 0:
            label += f" {dist:.2f}m"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        cv2.rectangle(img, (x1, y1 - th - 6),
                      (x1 + tw + 2, y1), color, cv2.FILLED)
        cv2.putText(img, label, (x1 + 1, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    if detections:
        cv2.putText(img, f"OD: {len(detections)} det", (4, img.shape[0] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)


# ──────────────────────────────────────────────────────────────────────────────
# Canvas / tiling helpers
# ──────────────────────────────────────────────────────────────────────────────

_SUB_W = 640           # default sub-figure width  (resized to this)
_SUB_H = 400           # default sub-figure height (resized to this)
_FONT = cv2.FONT_HERSHEY_DUPLEX   # sharper than SIMPLEX (double-stroke)
_FONT_SCALE = 0.55
_FONT_THICK = 1                         # thin strokes look crisper at this font
_BANNER_PAD = 8        # vertical padding inside each banner bar
_BORDER_W = 2        # white border width between sub-figures (pixels)


def _banner_height() -> int:
    """Height of one banner row in pixels."""
    (_, th), baseline = cv2.getTextSize("Ag", _FONT, _FONT_SCALE, _FONT_THICK)
    return th + baseline + _BANNER_PAD * 2


# Banner background colours (BGR)
_BG_LIVE = (0, 220,   0)   # green  — actively receiving
_BG_STALE = (0, 140, 255)   # orange — stream interrupted
_BG_NO_CB = (0,   0, 220)   # red    — DDS never matched
_BG_DEVICE = (0, 220, 255)   # yellow — device info rows
_TEXT_BLACK = (0, 0, 0)         # all banner text is black


def _draw_banner(img: np.ndarray, text: str, x: int, y: int, w: int,
                 bg=None) -> int:
    """Draw a filled banner bar of width w at position (x, y).

    bg: BGR background colour — defaults to _BG_DEVICE (yellow).
    Text is always black for maximum contrast.
    Returns the bottom y coordinate of the bar.
    """
    if bg is None:
        bg = _BG_DEVICE
    (_, th), baseline = cv2.getTextSize(text, _FONT, _FONT_SCALE, _FONT_THICK)
    bar_h = th + baseline + _BANNER_PAD * 2
    cv2.rectangle(img, (x, y), (x + w, y + bar_h), bg, cv2.FILLED)
    cv2.putText(img, text, (x + 8, y + th + _BANNER_PAD),
                _FONT, _FONT_SCALE, _TEXT_BLACK, _FONT_THICK, cv2.LINE_AA)
    return y + bar_h


def _placeholder(w: int, h: int, label: str, status: str = "WAITING") -> np.ndarray:
    """Grey tile shown while no frame is available.

    status:
      'WAITING' – never received any frame yet (callback never fired)
      'STALE'   – received frames before but none recently (DDS gap)
    """
    # Background: dark grey for WAITING, dark orange-red tint for STALE
    bg = (60, 60, 60) if status == "WAITING" else (30, 30, 80)
    tile = np.full((h, w, 3), bg, dtype=np.uint8)
    line1 = f"[{status}]  {label}"
    line2 = ("No callback received — possible DDS discovery issue"
             if status == "WAITING"
             else "Frame stream interrupted — last seen > 2s ago")
    (tw1, th1), _ = cv2.getTextSize(line1, _FONT, 0.60, 1)
    (tw2, th2), _ = cv2.getTextSize(line2, _FONT, 0.40, 1)
    cy = h // 2
    color1 = (100, 220, 255) if status == "WAITING" else (80, 120, 255)
    cv2.putText(tile, line1, ((w - tw1) // 2, cy - th1 // 2 - 4),
                _FONT, 0.60, color1, 1, cv2.LINE_AA)
    cv2.putText(tile, line2, ((w - tw2) // 2, cy + th2 + 4),
                _FONT, 0.40, (180, 180, 180), 1, cv2.LINE_AA)
    return tile


def build_combined_frame(
    device_info: dict,
    labels: list[str],
    frames: list,           # list[np.ndarray | None]  — None = no frame yet
    states: list,           # list[StreamState]
    sub_w: int,
    sub_h: int,
) -> np.ndarray:
    """Compose all stream sub-figures into one combined image.

    Layout:
      • Top: two device banner bars spanning full width
      • Below: stream sub-figures tiled in one row, separated by white borders
        Each sub-figure = per-stream banner + scaled image + white border frame
    """
    n = len(labels)
    bh = _banner_height()
    bw = _BORDER_W

    # Each sub-figure column occupies (sub_w + bw) pixels; trailing border on right
    col_w = sub_w + bw          # width per column including left border
    total_w = col_w * n + bw      # +bw for the rightmost border

    # Two device banner rows + bottom border + stream-banner row + image rows + bottom border
    dev_rows = 2 if device_info.get("net") else 1
    dev_h = bh * dev_rows + bw  # device banners + separator line
    sub_area = bh + sub_h          # stream banner + image
    canvas_h = dev_h + bw + sub_area + bw   # top border + content + bottom border

    canvas = np.zeros((canvas_h, total_w, 3), dtype=np.uint8)

    # ── device banners (full width) ───────────────────────────────────────────
    y = 0
    y = _draw_banner(canvas, device_info["sn_fw"], 0, y, total_w)
    if device_info.get("net"):
        y = _draw_banner(canvas, device_info["net"], 0, y, total_w)
    # white separator line between device header and sub-figures
    canvas[y:y + bw, :] = 255
    sub_top = y + bw   # y-offset where sub-figures start

    # ── per-stream sub-figures ────────────────────────────────────────────────
    for idx in range(n):
        st = states[idx]
        # left edge of this sub-figure (after left border)
        x0 = bw + idx * col_w

        # White left border for every sub-figure
        canvas[:, x0 - bw:x0] = 255

        # Snapshot state for this sub-figure
        with st.lock:
            hw_fps = st.hw_fps
            dis_fps = st.dis_fps
            fmt = st.fmt
            total_count = st.count
            first_cb = st.first_cb_time
            last_cb = st.last_cb_time
            od_dets = list(st.od_detections)  # snapshot OD detections
            od_cnt = st.od_count

        now = time.time()
        # Determine stream health status:
        #   NO_CB   – callback has never fired (likely DDS discovery not matched)
        #   STALE   – callback fired before but not in last 2s (stream interrupted)
        #   LOW_FPS – receiving frames but HW FPS < 29.5 (degraded)
        #   LIVE    – actively receiving AND HW FPS ≥ 29.5
        if first_cb is None:
            stream_status = "NO_CB"
        elif (now - last_cb) > _HW_FPS_WINDOW:
            stream_status = "STALE"
        elif hw_fps < _HW_FPS_OK_THRESHOLD:
            stream_status = "LOW_FPS"
        else:
            stream_status = "LIVE"

        # Resize frame or placeholder
        if frames[idx] is not None:
            h_orig = frames[idx].shape[0]
            w_orig = frames[idx].shape[1]
            tile = cv2.resize(frames[idx], (sub_w, sub_h),
                              interpolation=cv2.INTER_LINEAR)
            # Draw OD bounding boxes on the resized tile
            if od_dets:
                sx = sub_w / w_orig if w_orig > 0 else 1.0
                sy = sub_h / h_orig if h_orig > 0 else 1.0
                _draw_od_overlay(tile, od_dets, sx, sy)
        else:
            ph_status = "WAITING" if stream_status == "NO_CB" else "STALE"
            tile = _placeholder(sub_w, sub_h, labels[idx], ph_status)
            h_orig = sub_h
            w_orig = sub_w

        # Banner text — always show count so you can distinguish NO_CB vs LIVE
        stream_txt = (f"Stream:{labels[idx]}  Fmt:{fmt}  "
                      f"{w_orig}x{h_orig}  "
                      f"[{stream_status}] HW:{hw_fps:.1f}fps  "
                      f"Dis:{dis_fps:.1f}fps  cnt:{total_count}")
        if od_cnt > 0:
            stream_txt += f"  OD:{len(od_dets)}det"

        # Banner bg: red=NO_CB, orange=STALE/LOW_FPS, green=LIVE
        if stream_status == "NO_CB":
            banner_bg = _BG_NO_CB
        elif stream_status in ("STALE", "LOW_FPS"):
            banner_bg = _BG_STALE
        else:
            banner_bg = _BG_LIVE

        _draw_banner(canvas, stream_txt, x0, sub_top, sub_w, bg=banner_bg)

        # Paste tile below stream banner
        ty = sub_top + bh
        canvas[ty:ty + sub_h, x0:x0 + sub_w] = tile

    # Right border and bottom border
    canvas[:, -bw:] = 255
    canvas[-bw:, :] = 255

    return canvas


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────

def _make_file_tag(serial: str, stream_str: str) -> str:
    """Build a file-name prefix:  ros2-<SN>-<stream>-image-<ts>"""
    ts = time.strftime("%Y%m%d_%H%M%S")
    s = re.sub(r"[+,]", "_", stream_str)
    return f"ros2-{serial}-{s}-image-{ts}"


def main():
    ap = argparse.ArgumentParser(
        description=("D555 ROS2 stream viewer / headless tester.\n"
                     "Default: headless log-only mode (no GUI).\n"
                     "Add --gui to enable the tiled display window."),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--serial", help="D555 serial(s). Comma-separated for multi-camera single-process mode (recommended). "
                    "Example: --serial 333422301089,338122301551,338122303297")
    ap.add_argument(
        "--stream",
        default="Depth",
        help=("Stream name(s), '+' or ',' separated.  Aliases: "
              "IR1=Infrared_1, IR2=Infrared_2, IR3=Infrared_3, "
              "CompColor=CompressedColor.  e.g. Depth+IR1+IR2"),
    )
    ap.add_argument(
        "--topic", help="Single full topic path (overrides --serial/--stream)")
    ap.add_argument("--domain-id", type=int, help="Override ROS_DOMAIN_ID")
    ap.add_argument("--gui", nargs="?", const="on", default="off",
                    help="GUI mode: 'on' or omitted = enable GUI, 'off' = headless (default: headless).  "
                         "Examples: --gui (enable), --gui on (enable), --gui off (disable).")
    ap.add_argument("--debug", action="store_true",
                    help="Capture a pcap (first 10s) for offline DDS analysis.")
    ap.add_argument("--duration", type=float, default=0,
                    help="Auto-exit after N seconds (0 = run until 'q'/X/Ctrl-C).")
    ap.add_argument("--nic", default="",
                    help="NIC for --debug pcap capture. Auto-detected from camera IP if omitted.")
    ap.add_argument("--sub-width",  type=int, default=_SUB_W,
                    help=f"Width of each stream sub-figure (default {_SUB_W})")
    ap.add_argument("--sub-height", type=int, default=_SUB_H,
                    help=f"Height of each stream sub-figure (default {_SUB_H})")
    ap.add_argument("--od", action="store_true",
                    help="Enable object detection overlay.  Subscribes to the dedicated "
                         "OD topic and draws bounding boxes on Color/CompressedColor streams.")
    args = ap.parse_args()

    if args.domain_id is not None:
        os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)

    gui_mode = args.gui.lower() in ("on", "true", "yes", "1")
    debug_mode = args.debug
    duration = args.duration
    sub_w = args.sub_width
    sub_h = args.sub_height

    # ── resolve serial(s) ───────────────────────────────────────────────────
    # Support multi-camera single-process mode:
    #   --serial 333422301089,338122301551,338122303297
    # All cameras share ONE DDS participant → avoids SPDP/SEDP race.
    serials_raw = args.serial or ""
    serials = [s.strip() for s in re.split(
        r"[,\s]+", serials_raw) if s.strip()]
    if not args.topic and not serials:
        _auto = detect_serial()
        if _auto:
            serials = [_auto]
    if not args.topic and not serials:
        print("ERROR: no D555 node found. Provide --serial or start the camera node.",
              file=sys.stderr)
        return 3
    serial = serials[0]  # primary serial (used for file tag, device info)

    # ── build (topic, label) list ─────────────────────────────────────────────
    if args.topic:
        streams_info = [(args.topic, args.topic.split("/")[-1])]
    else:
        raw_names = re.split(r"[+,]", args.stream.strip())
        stream_names = [resolve_stream_name(
            s.strip()) for s in raw_names if s.strip()]
        # Multi-serial mode: cross-product of serials × streams
        streams_info = []
        for sn in serials:
            for s in stream_names:
                label = f"{sn}_{s}" if len(serials) > 1 else s
                streams_info.append((build_topic(sn, s), label))

    if not streams_info:
        print("ERROR: no streams specified.", file=sys.stderr)
        return 3

    # ── output file paths ─────────────────────────────────────────────────────
    _script_dir = os.path.dirname(os.path.abspath(__file__))
    file_tag = _make_file_tag(serial, args.stream)
    log_path = os.path.join(_script_dir, f"{file_tag}.log")
    pcap_path = os.path.join(
        _script_dir, f"{file_tag}.pcap") if debug_mode else None

    # ── fetch device info (retry up to 2 extra times on transient failure) ────
    # In multi-serial mode, query ALL cameras to pre-warm DDS discovery.
    # This ensures the ros2 daemon discovers all camera participants before
    # we create our subscriber, avoiding SPDP/SEDP race conditions.
    device_info = {"sn_fw": "SN:(unknown)", "net": ""}
    if serials:
        for sn in serials:
            _MAX_RETRIES = 3
            for _attempt in range(1, _MAX_RETRIES + 1):
                print(
                    f"  Querying Device.Info for SN:{sn} … (attempt {_attempt}/{_MAX_RETRIES})")
                _info = fetch_device_info(sn)
                _sn_str = _info["sn_fw"].lower()
                if "unavailable" not in _sn_str and "error" not in _sn_str:
                    break
                if _attempt < _MAX_RETRIES:
                    print(f"  Retrying in 1 s … ({_info['sn_fw']})")
                    time.sleep(1)
            if sn == serial:
                device_info = _info  # use primary serial's info for banner/NIC
            print(f"  Device: {_info['sn_fw']}")

    if not check_stream_network_preconditions(device_info):
        print("ERROR: pre-stream network check failed (MTU mismatch or missing net info)", file=sys.stderr)
        return 4

    # ── optional: start pcap capture (--debug) ────────────────────────────────
    pcap_proc = None
    # Pcap duration = test duration (if set) so we capture the whole window;
    # fallback to 30s (enough to cover a typical discovery stall scenario).
    _PCAP_DURATION = int(duration) if duration > 0 else 30
    if debug_mode and pcap_path:
        # ── auto-detect NIC if not specified ──────────────────────────────────
        _nic = args.nic.strip()
        if not _nic:
            _cam_ip = device_info.get("net", "")
            # extract IP from 'IP:192.168.11.55  MTU:...' format
            _ip_m = re.search(r"IP:(\S+)", _cam_ip)
            _cam_ip_val = _ip_m.group(1) if _ip_m else ""
            _nic = detect_camera_nic(_cam_ip_val)
            if _nic:
                print(
                    f"  [DEBUG] Auto-detected NIC '{_nic}' for camera IP {_cam_ip_val}")
            else:
                print(
                    f"  [DEBUG] ✗ Could not auto-detect NIC for camera IP '{_cam_ip_val}'")
                print(f"           Use --nic <interface> to specify manually.")
                print(f"           Available interfaces: "
                      f"{subprocess.getoutput('ip -br link show | awk NR>1{{print $1}}').split()[:8]}")
                pcap_path = None

        if pcap_path and _nic:
            print(
                f"  [DEBUG] Starting tshark: -i {_nic} -a duration:{_PCAP_DURATION} -w {os.path.basename(pcap_path)}")
            try:
                pcap_proc = subprocess.Popen(
                    ["tshark", "-i", _nic, "-a", f"duration:{_PCAP_DURATION}",
                     "-w", pcap_path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,   # capture stderr for diagnostics
                    cwd=_script_dir,
                )
                # Wait briefly to detect immediate startup failures
                time.sleep(0.5)
                ret = pcap_proc.poll()
                if ret is not None:
                    # Process already exited — capture stderr output
                    err_out = pcap_proc.stderr.read().decode(errors="replace").strip()
                    print(f"  [DEBUG] ✗ tshark exited immediately (rc={ret})")
                    if err_out:
                        for ln in err_out.splitlines():
                            print(f"           {ln}")
                    print(
                        f"           cmd: tshark -i {_nic} -a duration:{_PCAP_DURATION} -w {pcap_path}")
                    pcap_proc = None
                    pcap_path = None
                else:
                    print(f"  [DEBUG] ✓ tshark running (PID {pcap_proc.pid})")
                    print(
                        f"  [DEBUG] ✓ NIC:{_nic}  duration:{_PCAP_DURATION}s  → {pcap_path}")
            except FileNotFoundError:
                print("  [DEBUG] ✗ tshark not found – pcap capture skipped")
                print("           Install with: sudo apt-get install tshark")
                pcap_proc = None
                pcap_path = None

    def sanitize(name: str) -> str:
        clean = re.sub(r"[^A-Za-z0-9_]", "_", name or "")
        clean = clean.strip("_")
        if not clean or not clean[0].isalpha():
            clean = f"n_{clean}" if clean else "node"
        return clean

    # ── init rclpy & create one Viewer node per stream ────────────────────────
    rclpy.init()
    n_streams = len(streams_info)
    states:      list[StreamState] = []
    nodes:       list[Viewer] = []
    labels:      list[str] = []

    for topic, label in streams_info:
        compressed = is_compressed_stream(label, topic)
        node_name = sanitize(f"d555_viewer_{label}_{uuid.uuid4().hex[:6]}")
        st = StreamState()
        # Determine OD topic for this stream (--od flag, Color/CompressedColor only)
        od_topic = None
        if args.od:
            _base = label.split("_")[-1] if "_" in label else label
            if _base.lower() in ("color", "compressedcolor"):
                # Infer serial from the image topic path
                _sn_match = re.search(r"D555_(\d+)", topic)
                _sn = _sn_match.group(1) if _sn_match else serial
                od_topic = f"/realsense/D555_{_sn}_ObjectDetection"
        node = Viewer(topic, compressed, node_name, st, od_topic=od_topic)
        states.append(st)
        nodes.append(node)
        labels.append(label)
        print(f"  → Stream:{label}  topic={topic}  compressed={compressed}"
              f"{'  od=' + od_topic if od_topic else ''}")

    # ── one executor + spin thread per stream ─────────────────────────────────
    executors:    list = []
    spin_threads: list[threading.Thread] = []
    stop_events:  list[threading.Event] = []

    for node in nodes:
        exc = rclpy.executors.SingleThreadedExecutor()
        exc.add_node(node)
        ev = threading.Event()

        def _spin(e=exc, ev=ev):
            while not ev.is_set():
                e.spin_once(timeout_sec=0.005)

        th = threading.Thread(target=_spin, daemon=True)
        th.start()
        executors.append(exc)
        stop_events.append(ev)
        spin_threads.append(th)

    # ── SIGINT handler (same behaviour as 'q' / --duration) ─────────────────
    _sigint_flag = threading.Event()

    def _sigint_handler(sig, frame):  # noqa: ANN001
        _sigint_flag.set()

    signal.signal(signal.SIGINT, _sigint_handler)

    # ── teardown helper ───────────────────────────────────────────────────────
    def teardown_all():
        for i in range(n_streams):
            states[i].closed = True
            nodes[i].unsubscribe()
            stop_events[i].set()
        for i in range(n_streams):
            spin_threads[i].join(timeout=1.0)
        for nd in nodes:
            try:
                nd.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()
        if gui_mode:
            cv2.destroyAllWindows()

    # ── log file handle ───────────────────────────────────────────────────────
    log_fh = open(log_path, "w")
    start_wall = time.time()

    def _log(msg: str):
        """Write a line to both stdout and the log file."""
        elapsed = time.time() - start_wall
        line = f"[{elapsed:8.2f}s] {msg}"
        print(line)
        log_fh.write(line + "\n")
        log_fh.flush()

    _log(f"SN:{serial}  streams:{args.stream}  gui:{gui_mode}  debug:{debug_mode}")
    _log(f"Device: {device_info['sn_fw']}")
    if device_info.get('net'):
        _log(f"Net:    {device_info['net']}")
    _log(f"Log:    {log_path}")
    if pcap_path:
        _log(f"Pcap:   {pcap_path}")

    # ── GUI window (only in gui mode) ─────────────────────────────────────────
    WIN = None
    if gui_mode:
        WIN = f"D555 | {serial} | {' + '.join(labels)}"
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        bh = _banner_height()
        bw = _BORDER_W
        dev_rows = 2 if device_info.get("net") else 1
        dev_h = bh * dev_rows + bw
        sub_area = bh + sub_h
        canvas_h = dev_h + bw + sub_area + bw
        col_w = sub_w + bw
        canvas_w = col_w * n_streams + bw
        cv2.resizeWindow(WIN, canvas_w, canvas_h)
        _log(f"Window: '{WIN}'  size: {canvas_w}x{canvas_h}")
        _log("Press 'q' or close window to exit")
    else:
        _log(f"Headless mode – Ctrl-C or --duration {duration}s to exit")

    # ── main loop ─────────────────────────────────────────────────────────────
    last_render = 0.0
    last_log = 0.0
    _LOG_INTERVAL = 2.0    # print status line every 2s in headless mode
    exit_code = 0

    try:
        while True:
            now = time.time()

            # ── duration timeout ──────────────────────────────────────────────
            if duration > 0 and (now - start_wall) >= duration:
                _log(f"Duration {duration}s reached – exiting.")
                break

            # ── GUI event handling ────────────────────────────────────────────
            if gui_mode:
                key = cv2.waitKey(1) & 0xFF
                try:
                    prop = cv2.getWindowProperty(WIN, cv2.WND_PROP_AUTOSIZE)
                    if prop < 0:
                        _log("Window closed.")
                        break
                except Exception:
                    break
                if key == ord("q"):
                    _log("'q' pressed – closing.")
                    break
            else:
                # Headless: small sleep to avoid busy-wait
                time.sleep(0.05)

            # ── SIGINT (Ctrl-C) check ─────────────────────────────────────────
            if _sigint_flag.is_set():
                _log("SIGINT received – stopping.")
                break

            # ── throttle rendering / logging ──────────────────────────────────
            if gui_mode and (now - last_render) >= _DISPLAY_INTERVAL:
                last_render = now
                frames: list = []
                for i, st in enumerate(states):
                    with st.lock:
                        f = st.latest
                    frames.append(f.copy() if f is not None else None)
                    if f is not None:
                        st.record_display_frame()
                canvas = build_combined_frame(
                    device_info, labels, frames, states, sub_w, sub_h
                )
                cv2.imshow(WIN, canvas)
                try:
                    total_hw = sum(st.hw_fps for st in states)
                    cv2.setWindowTitle(
                        WIN, f"{WIN}  |  total HW: {total_hw:.1f} fps")
                except Exception:
                    pass

            # ── periodic status log (both modes) ─────────────────────────────
            if (now - last_log) >= _LOG_INTERVAL:
                last_log = now
                for i, st in enumerate(states):
                    with st.lock:
                        hw = st.hw_fps
                        cnt = st.count
                        fcb = st.first_cb_time
                        lcb = st.last_cb_time
                    if fcb is None:
                        tag = "NO_CB"
                    elif (now - lcb) > _HW_FPS_WINDOW:
                        tag = "STALE"
                    elif hw < _HW_FPS_OK_THRESHOLD:
                        tag = "LOW_FPS"
                    else:
                        tag = "LIVE"
                    _log(
                        f"  {labels[i]:16s}  [{tag:7s}]  HW:{hw:5.1f}fps  cnt:{cnt}")

    except KeyboardInterrupt:  # fallback for environments that bypass signal handler
        _log("Ctrl-C – stopping.")
    finally:
        teardown_all()
        # ── stop pcap if still running ────────────────────────────────────────
        if pcap_proc and pcap_proc.poll() is None:
            pcap_proc.terminate()
            try:
                pcap_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                pcap_proc.kill()
            _log(f"[DEBUG] ✓ pcap capture stopped (terminated)")

        # Check if pcap file exists and report its size
        if pcap_path and os.path.exists(pcap_path):
            pcap_size_kb = os.path.getsize(pcap_path) / 1024
            _log(
                f"[DEBUG] ✓ pcap file exists: {os.path.basename(pcap_path)} ({pcap_size_kb:.1f} KB)")
        elif pcap_path:
            _log(f"[DEBUG] ✗ pcap file NOT found (tshark may have failed)")

        # ── evaluate pass/fail ────────────────────────────────────────────────
        all_ok = True
        for i, st in enumerate(states):
            ok = (st.count > 0 and st.hw_fps >= _HW_FPS_OK_THRESHOLD)
            status = "PASS" if ok else "FAIL"
            _log(f"  RESULT  {labels[i]:16s}  {status}  "
                 f"HW:{st.hw_fps:.1f}fps  cnt:{st.count}")
            if not ok:
                all_ok = False
                exit_code = 1

        log_fh.close()

        # ── auto-cleanup on success ───────────────────────────────────────────
        if all_ok:
            log_removed = _try_remove(log_path)
            pcap_removed = False
            if pcap_path:
                pcap_removed = _try_remove(pcap_path)
            print(
                f"  All streams PASS – artifacts {'auto-cleaned' if (log_removed or pcap_removed) else 'already cleaned'}.")
            if log_removed:
                print(f"    Removed: {log_path}")
            if pcap_removed:
                print(f"    Removed: {pcap_path}")
        else:
            print(f"  Some streams FAIL – preserving artifacts for debug:")
            print(f"    log:  {log_path}")
            if pcap_path:
                print(f"    pcap: {pcap_path}")

    return exit_code


def _try_remove(path: str) -> bool:
    """Silently remove a file if it exists. Return True if removed, False if not found."""
    try:
        os.remove(path)
        return True
    except OSError:
        return False


if __name__ == "__main__":
    raise SystemExit(main())
