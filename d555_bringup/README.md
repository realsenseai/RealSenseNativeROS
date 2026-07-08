# D555_bringup

Host-side ROS 2 launch package for the **RealSense D555** camera.

The D555 camera boots autonomously and publishes ROS 2 topics over DDS/Ethernet.
This package provides companion nodes that run on the host PC for TF broadcasting,
color format conversion, depth alignment, and point cloud generation.

## Prerequisites

- **ROS 2 Humble** (Ubuntu 22.04) or **ROS 2 Jazzy with Cyclone DDS** (Ubuntu 24.04)
- **D555 camera** connected via Ethernet (default IP `192.168.11.55`)
- **Firmware** >= 7.58 (for Aligned_Depth_To_Color, native PointCloud2 publisher, ObjectDetection, and metadata topics)
- `ROS_DOMAIN_ID` matching the device (e.g., `export ROS_DOMAIN_ID=2`)
- On Jazzy, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

Install dependencies:

```bash
sudo apt install ros-humble-tf2-ros ros-humble-depth-image-proc \
                 ros-humble-cv-bridge ros-humble-image-transport \
                 ros-humble-image-transport-plugins
pip3 install opencv-python numpy
```

For Jazzy, install the same package set with the `ros-jazzy-` prefix and
Cyclone DDS:

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp \
                 ros-jazzy-tf2-ros ros-jazzy-depth-image-proc \
                 ros-jazzy-cv-bridge ros-jazzy-image-transport \
                 ros-jazzy-image-transport-plugins
pip3 install opencv-python numpy
```

## Build

```bash
cd ~/code/RealSenseNativeROS
source /opt/ros/humble/setup.bash
colcon build --packages-select d555_bringup --symlink-install
source install/setup.bash
```

For Jazzy:

```bash
cd ~/code/RealSenseNativeROS
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build --packages-select d555_bringup --symlink-install
source install/setup.bash
```

## Usage

```bash
# Basic — launches TF publisher, color relay, and point cloud
ros2 launch d555_bringup d555_bringup.launch.py serial:=<SERIAL>

# Custom mounting pose (30 cm forward, 15 cm up, 15° pitch down)
ros2 launch d555_bringup d555_bringup.launch.py serial:=343122300393 \
    cam_x:=0.30 cam_z:=0.15 cam_pitch:=-0.2618

# Color relay only (no point cloud)
ros2 launch d555_bringup d555_bringup.launch.py serial:=343122300393 \
    enable_pointcloud:=false

# Depth-only point cloud (no color needed)
ros2 launch d555_bringup d555_bringup.launch.py serial:=343122300393 \
    pointcloud_type:=xyz enable_color_relay:=false

# Use on-device aligned depth instead of host-side alignment
ros2 param set /D555_343122300393 Depth.option.Align_Depth 1
ros2 launch d555_bringup d555_bringup.launch.py serial:=343122300393 \
    use_device_align:=true
```

`use_device_align:=true` changes the host subscription path only. It does not
set firmware parameters; enable `Depth.option.Align_Depth` before launching.

## Nodes

| Node | Package | Description |
|------|---------|-------------|
| `static_transform_publisher` | `tf2_ros` | Publishes `base_link` → `camera_link` static TF with configurable 6-DOF pose |
| `d555_color_relay` | `d555_bringup` | Converts YUV422/NV12 color images to RGB8 (saves Ethernet bandwidth — 2 bytes/pixel vs 3) |
| `register_node` | `depth_image_proc` | Host-side depth-to-color alignment (Mode A only) |
| `point_cloud_xyzrgb_node` | `depth_image_proc` | Generates `PointCloud2` from aligned depth + RGB |

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `serial` | *(required)* | D555 camera serial number |
| `camera_name` | `camera` | ROS namespace and TF prefix |
| `base_frame` | `base_link` | Parent TF frame |
| `camera_frame` | `camera_link` | Camera TF frame |
| `cam_x` / `cam_y` / `cam_z` | `0.0` | Mounting translation in meters |
| `cam_roll` / `cam_pitch` / `cam_yaw` | `0.0` | Mounting rotation in radians |
| `enable_color_relay` | `true` | Launch the YUV → RGB8 color relay |
| `enable_pointcloud` | `true` | Launch the point cloud node |
| `enable_depth_align` | `true` | Launch host-side depth alignment (Mode A only) |
| `pointcloud_type` | `xyzrgb` | Point cloud type: `xyzrgb` (colored) or `xyz` (depth only) |
| `use_device_align` | `false` | Use on-device aligned depth (skip host `register_node`) |

## Depth Alignment Modes

### Mode A — Host-side alignment (default)

```
D555 Depth ──► register_node ──► aligned_depth ──► point_cloud_xyzrgb ──► /camera/points2
D555 Color ──► d555_color_relay ──► /camera/color/image_raw ──┘
```

The host runs `depth_image_proc/register_node` to align the raw depth image
to the color camera's coordinate frame using the camera intrinsics.

### Mode B — Device-side alignment (`use_device_align:=true`)

```
D555 Aligned_Depth_To_Color ──► point_cloud_xyzrgb ──► /camera/points2
D555 Color ──► d555_color_relay ──► /camera/color/image_raw ──┘
```

The D555 performs depth-to-color alignment on-device. The host directly
consumes the pre-aligned depth stream.

Before using this mode:

```bash
ros2 param set /D555_<serial> Depth.option.Align_Depth 1
```

The launch file does not enable the firmware parameter automatically.

### Native PointCloud2 Publisher

r58.3 also exposes `/realsense/D555_<serial>_Depth_Color_Points` after:

```bash
ros2 param set /D555_<serial> Depth.option.Enable_PointCloud 1
```

This is separate from the host `/camera/points2` output generated by
`depth_image_proc`. On the tested D555 r58.3 firmware, the native publisher
appeared but no PointCloud2 sample was received in a 25-30 second echo window,
so `/camera/points2` remains the validated path for point cloud output.

## Topic Mapping

| Host Topic | Source Device Topic |
|------------|---------------------|
| `/camera/color/image_raw` (rgb8) | `/realsense/D555_<serial>_Color` (yuv422) |
| `/camera/aligned_depth/image_raw` | Computed by `register_node` or `/realsense/D555_<serial>_Aligned_Depth_To_Color` when `use_device_align:=true` |
| `/camera/points2` | Computed by `point_cloud_xyzrgb_node` on the host |
| `/realsense/D555_<serial>_Depth_Color_Points` | Native firmware publisher, gated by `Depth.option.Enable_PointCloud` |
| `/tf_static` | `base_link` → `camera_link` from launch args |

## License

Apache-2.0 — see [LICENSE](../LICENSE).
