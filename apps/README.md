# D555 ROS2 Application Suite

Sample applications for the Intel RealSense D555 camera's native ROS2 interface.

> **Prerequisites:** ROS2 Humble, or ROS2 Jazzy with Cyclone DDS, Python 3.10+, OpenCV (`pip3 install opencv-python`).
> The D555 must be connected via Ethernet on the same subnet (default IP: `192.168.11.55`).
> Set `ROS_DOMAIN_ID` to match the device (e.g., `export ROS_DOMAIN_ID=2`).
> On Jazzy, also set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.

---

## Stream Viewer & Tester

A multi-stream ROS2 image viewer with **GUI** (tiled display) and **headless** (log-only) modes.
Designed for both interactive viewing and automated CI/regression testing.

### Features

- **Multi-stream tiling:** View Depth, Color, IR1, IR2, Color/compressed, aligned depth, Diagnostics/FirmwareLog JSON, and PointCloud2 smoke-test streams.
- **Multi-camera:** Comma-separated serials for single-process multi-camera viewing.
- **HW FPS measurement:** Sliding-window (2 s) real-time frame rate, independent of display rendering.
- **Headless CI mode (default):** Prints status to stdout, writes a log file, auto-cleans on PASS.
- **Pre-stream network check:** Validates device/host MTU match before subscribing.
- **Object detection overlay:** `--od` flag draws bounding boxes and distance values from the OD topic when ObjectDetection publishes output.
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

# On-device aligned depth (enable the firmware topic first)
ros2 param set /D555_343122300393 Depth.option.Align_Depth 1
python3 show_ros_image.py --serial 343122300393 --stream AlignedDepth --duration 10
ros2 param set /D555_343122300393 Depth.option.Align_Depth 0

# Native PointCloud2 smoke test
ros2 param set /D555_343122300393 Depth.option.Enable_PointCloud 2
python3 show_ros_image.py --serial 343122300393 --stream PointCloud --duration 10
ros2 param set /D555_343122300393 Depth.option.Enable_PointCloud 0

# r58.3 temporal + decimation depth-filter smoke test.
# Configure filters before starting Depth/AlignedDepth/PointCloud subscribers.
ros2 param set /D555_343122300393 Depth.filter.Temporal.Toggle 1
ros2 param set /D555_343122300393 Depth.filter.Decimation.Toggle 1
ros2 param set /D555_343122300393 Depth.filter.Decimation.Magnitude 2
python3 show_ros_image.py --serial 343122300393 --stream Depth --duration 10

# MinZ mode is exposed as Improved_Close_Range_Depth.Enable.
# Use it as a separate pre-stream mode, not together with decimation on r58.3.
ros2 param set /D555_343122300393 Depth.filter.Decimation.Toggle 0
ros2 param set /D555_343122300393 Depth.filter.Improved_Close_Range_Depth.Enable 1
python3 show_ros_image.py --serial 343122300393 --stream Depth --duration 10
ros2 param set /D555_343122300393 Depth.filter.Improved_Close_Range_Depth.Enable 0

# Device diagnostics JSON topic
python3 show_ros_image.py --serial 343122300393 --stream Diagnostics --duration 5
ros2 topic echo /realsense/D555_343122300393/diagnostics \
  --once --qos-reliability best_effort

# Firmware log JSON topic.
# firmware_log publishes only while Device.Log_Enable=true and a subscriber is present.
ros2 param set /D555_343122300393 Device.Log_Level 12
ros2 param set /D555_343122300393 Device.Log_Enable true
python3 show_ros_image.py --serial 343122300393 --stream FirmwareLog --duration 10
ros2 topic echo /realsense/D555_343122300393/firmware_log \
  --qos-reliability best_effort
ros2 param set /D555_343122300393 Device.Log_Enable false

# ObjectDetection overlay on color; detections and distance fields in meters are scene/model dependent
ros2 param set /D555_343122300393 ObjectDetection.option.Object_Distance 1
python3 show_ros_image.py --gui --serial 343122300393 --stream Color --od

# Auto-detect serial
python3 show_ros_image.py --gui --stream Color

# Multi-camera single-process mode
python3 show_ros_image.py --gui --serial 343122300393,344522301530 --stream Depth+Color

# Debug mode with pcap capture
python3 show_ros_image.py --debug --serial 343122300393 --stream Color --duration 30
```

For Jazzy, use Cyclone DDS in the shell before running the same commands:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=2
```

<details>
<summary>Stream viewer technical reference</summary>

### Stream Aliases

| Alias | Canonical Name |
|-------|---------------|
| `IR1` | `Infrared_1` |
| `IR2` | `Infrared_2` |
| `CompColor` | `/realsense/<SN>_Color/compressed` |
| `AlignedDepth` | `Aligned_Depth_To_Color` |
| `Points`, `PointCloud` | `Depth_Color_Points` |
| `Diagnostics`, `Diag` | `/realsense/<SN>/diagnostics` |
| `FirmwareLog`, `FwLog`, `Log` | `/realsense/<SN>/firmware_log` |

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

- **PASS:** Image streams require HW FPS >= 29.5 and at least one frame received.
- **PointCloud2 PASS:** PointCloud2 smoke tests require at least one sample and HW FPS >= 0.1. The app keeps hidden Depth + Color guard subscribers active for PointCloud streams.
- **String Topic PASS:** Diagnostics and FirmwareLog smoke tests require at least one sample and HW FPS >= 0.1.
- **FAIL:** Otherwise. Log and pcap (if `--debug`) are preserved for analysis.
- On PASS, log/pcap artifacts are auto-deleted.
- If `ros2 topic hz <topic>` reports less than 29.5 FPS, the app will fail by design even when callbacks are received.

On the tested D555 r58.3 firmware, `AlignedDepth` passed as
`sensor_msgs/msg/Image`, and `Depth_Color_Points` passed as
`sensor_msgs/msg/PointCloud2` with `x,y,z,rgb` fields after
`Depth.option.Enable_PointCloud=2`. The PointCloud smoke path uses one
PointCloud reader plus hidden Depth + Color guard subscribers. Avoid running
multiple PointCloud readers, such as `ros2 topic hz` plus `ros2 topic echo`, at
the same time. `ObjectDetection.option.Object_Distance=1` may return
`Invalid value` if depth streaming is already active; stop depth streaming before
enabling the depth-cache distance path. The overlay still subscribes to
`/realsense/<SN>_ObjectDetection` and draws detections when messages contain
detection boxes.

For r58.3 depth filters, `Depth.filter.Temporal.*` and
`Depth.filter.Decimation.*` are the tested temporal + decimation path. MinZ is
not exposed as a literal `MinZ` parameter; use
`Depth.filter.Improved_Close_Range_Depth.Enable` and keep it separate from
decimation. Diagnostics are published as `std_msgs/msg/String` JSON on
`/realsense/<SN>/diagnostics` when subscribed.

Firmware logs are published as `std_msgs/msg/String` JSON on
`/realsense/<SN>/firmware_log` after `Device.Log_Enable=true` and a subscriber
is present. On current r58.3 builds, the topic may remain visible in DDS
discovery after being enabled once, but it does not emit samples after
`Device.Log_Enable=false`. The stream is event-driven, so a short `FirmwareLog`
smoke test can fail if no firmware log is emitted during the test window. Keep
`ros2 topic echo` running while changing a parameter or starting a stream when
you need a fresh sample.

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
| `rclpy` | Included with ROS2 Humble/Jazzy |
| `opencv-python` | `pip3 install opencv-python` |
| `cv_bridge` | `sudo apt install ros-humble-cv-bridge` |
| `numpy` | `pip3 install numpy` |
| `tshark` (optional, `--debug`) | `sudo apt install tshark` |

</details>

---

## 3D Reconstruction Viewer

An interactive 3D point cloud reconstruction application with real-time rotating views,
metadata display, and PLY export.

This application reconstructs point clouds on the host from raw Depth + Color
topics. It does not depend on the native `/realsense/<SN>_Depth_Color_Points`
PointCloud2 publisher.

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

<details>
<summary>3D reconstruction technical reference</summary>

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
| `rclpy` | Included with ROS2 Humble/Jazzy |
| `opencv-python` | `pip3 install opencv-python` |
| `numpy` | `pip3 install numpy` |
| `open3d` (optional, for PLY export) | `pip3 install open3d` |

</details>

---

## Frame Sync Verification

See [external_sync/README_external_sync.md](external_sync/README_external_sync.md) for the multi-camera
synchronization verification tool.
