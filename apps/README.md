# D555 ROS2 Application Suite

Sample applications for the Intel RealSense D555 camera's native ROS2 interface.

> **Prerequisites:** ROS2 Humble, Python 3.10+, OpenCV (`pip3 install opencv-python`).
> The D555 must be connected via Ethernet on the same subnet (default IP: `192.168.11.55`).
> Set `ROS_DOMAIN_ID` to match the device (e.g., `export ROS_DOMAIN_ID=2`).

---

## `show_ros_image.py` — Stream Viewer & Tester

A multi-stream ROS2 image viewer with **GUI** (tiled display) and **headless** (log-only) modes.
Designed for both interactive viewing and automated CI/regression testing.

### Features

- **Multi-stream tiling:** View Depth, Color, IR1, IR2, CompressedColor simultaneously.
- **Multi-camera:** Comma-separated serials for single-process multi-camera viewing.
- **HW FPS measurement:** Sliding-window (2 s) real-time frame rate, independent of display rendering.
- **Headless CI mode (default):** Prints status to stdout, writes a log file, auto-cleans on PASS.
- **Pre-stream network check:** Validates device/host MTU match before subscribing.
- **Object detection overlay:** `--od` flag draws bounding boxes from the OD topic.
- **Debug pcap capture:** `--debug` starts a tshark capture for offline DDS analysis.

### Usage

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=2
cd apps/

# GUI mode — single stream
python3 show_ros_image.py --gui --serial 343122300393 --stream Color

# GUI mode — multiple streams
python3 show_ros_image.py --gui --serial 343122300393 --stream Depth+Color+IR1

# Headless mode (default) — automated test, 30 s timeout
python3 show_ros_image.py --serial 343122300393 --stream Depth --duration 30

# Auto-detect serial
python3 show_ros_image.py --gui --stream Color

# Multi-camera single-process mode
python3 show_ros_image.py --gui --serial 343122300393,344522301530 --stream Depth+Color

# Debug mode with pcap capture
python3 show_ros_image.py --debug --serial 343122300393 --stream Color --duration 30
```

### Stream Aliases

| Alias | Canonical Name |
|-------|---------------|
| `IR1` | `Infrared_1` |
| `IR2` | `Infrared_2` |
| `CompColor` | `CompressedColor` |

### Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--serial` | auto-detect | D555 serial number(s), comma-separated for multi-camera |
| `--stream` | `Depth` | Stream name(s), `+` or `,` separated |
| `--gui` | off (headless) | Enable tiled GUI window |
| `--duration` | 0 (infinite) | Auto-exit after N seconds |
| `--debug` | off | Capture pcap (tshark) for DDS analysis |
| `--domain-id` | env `ROS_DOMAIN_ID` | Override ROS domain ID |
| `--od` | off | Enable object detection bounding box overlay |
| `--sub-width` | 640 | Width of each stream sub-figure (pixels) |
| `--sub-height` | 400 | Height of each stream sub-figure (pixels) |

### Pass/Fail Criteria (Headless)

- **PASS:** HW FPS ≥ 29.5 and at least one frame received.
- **FAIL:** Otherwise. Log and pcap (if `--debug`) are preserved for analysis.
- On PASS, log/pcap artifacts are auto-deleted.

### GUI Layout

```
┌──────────────────────────────────────────────┐
│  SN:xxx  FW:7.58.xxx           (device row)  │
│  IP:192.x  MTU:9000  Delay:0  Link:1000M     │
├────────────┬────────────┬────────────────────┤
│ Stream:Dep │ Stream:IR1 │  Stream:Color       │
│ Fmt.. HW.. │ Fmt.. HW..│  Fmt.. HW..         │
│  <image>   │  <image>  │   <image>            │
└────────────┴────────────┴────────────────────┘
```

Exit: `q` or close window (GUI), `Ctrl-C` or `--duration` timeout (headless).

### Dependencies

| Package | Install |
|---------|---------|
| `rclpy` | Included with ROS2 Humble |
| `opencv-python` | `pip3 install opencv-python` |
| `cv_bridge` | `sudo apt install ros-humble-cv-bridge` |
| `numpy` | `pip3 install numpy` |
| `tshark` (optional, `--debug`) | `sudo apt install tshark` |

---

## `d555_3d_reconstruction_v4.py` — 3D Reconstruction Viewer

An interactive 3D point cloud reconstruction application with real-time rotating views,
metadata display, and PLY export.

### Features

- **Dual 3D views:** Rotating RGB-textured point cloud and depth-colormap point cloud.
- **2D panels:** Live RGB camera, depth colormap with turbo scale.
- **Info panels:** Camera intrinsics, per-stream metadata, TF transforms, IMU data, performance stats.
- **Recording:** Save depth+color frames to pickle files for offline analysis.
- **3D Reconstruction:** Accumulate point clouds across frames and export to PLY.
- **Keyboard controls:** Interactive zoom, rotation speed, recording, and reconstruction.

### Usage

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=2
cd apps/

# GUI mode — interactive 3D viewer
python3 d555_3d_reconstruction_v4.py -gui

# Headless mode — subscribe and process without display
python3 d555_3d_reconstruction_v4.py
```

### Keyboard Controls

| Key | Action |
|-----|--------|
| `Q` / `ESC` | Quit |
| `R` | Start/Stop recording (saves depth+color frames) |
| `B` | Start/Stop 3D reconstruction (accumulate point clouds) |
| `C` | Clear accumulated point cloud |
| `S` | Save current 3D model to PLY file |
| `SPACE` | Pause/Resume display |
| `+` / `-` | Zoom in/out |
| `1` / `2` | Decrease/Increase rotation speed |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/realsense/<SN>_Depth` | `sensor_msgs/msg/Image` | 16-bit depth map |
| `/realsense/<SN>_Color` | `sensor_msgs/msg/Image` | RGB/YUV color image |
| `/realsense/<SN>_Depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Depth intrinsics |
| `/realsense/<SN>_Color/camera_info` | `sensor_msgs/msg/CameraInfo` | Color intrinsics |
| `/realsense/<SN>_Motion` | `sensor_msgs/msg/Imu` | IMU accelerometer + gyroscope |
| `/realsense/<SN>/tf_static` | `tf2_msgs/msg/TFMessage` | Static transforms (extrinsics) |
| `/realsense/<SN>_*/metadata` | `std_msgs/msg/String` | Per-stream metadata (JSON) |

### Output Files

| File | Format | Description |
|------|--------|-------------|
| `recording_YYYYMMDD_HHMMSS.pkl` | Python pickle | Raw depth+color frame sequence |
| `reconstruction_YYYYMMDD_HHMMSS.ply` | PLY | Merged 3D point cloud (up to 1M points) |

### Dependencies

| Package | Install |
|---------|---------|
| `rclpy` | Included with ROS2 Humble |
| `opencv-python` | `pip3 install opencv-python` |
| `numpy` | `pip3 install numpy` |
| `open3d` (optional, for PLY export) | `pip3 install open3d` |

---

## `external_sync/` — Frame Sync Verification

See [external_sync/README_external_sync.md](external_sync/README_external_sync.md) for the
multi-camera synchronization verification tool.
