<p align="center">
<!-- Light mode -->
<img src="https://raw.githubusercontent.com/realsenseai/librealsense/master/doc/img/realsense-logo-light-mode.png#gh-light-mode-only" alt="RealSense logo" width="30%"/>
<!-- Dark mode -->
<img src="https://raw.githubusercontent.com/realsenseai/librealsense/master/doc/img/realsense-logo-dark-mode.png#gh-dark-mode-only" alt="RealSense logo" width="30%"/>
  <br><br>
  <b>Native ROS2 Interface for RealSense D555 Camera</b><br>
  <i>Driverless, Embedded ROS2 — No Host Wrapper Required</i>
  <br><br>
</p>

<hr>

[![humble](https://img.shields.io/badge/-HUMBLE-orange?style=flat-square&logo=ros)](https://docs.ros.org/en/humble/index.html)
[![jazzy](https://img.shields.io/badge/-JAZZY-orange?style=flat-square&logo=ros)](https://docs.ros.org/en/jazzy/index.html)
[![ubuntu22](https://img.shields.io/badge/-UBUNTU%2022.04-blue?style=flat-square&logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/jammy/)
[![ubuntu24](https://img.shields.io/badge/-UBUNTU%2024.04-blue?style=flat-square&logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/noble/)

<hr>

## Table of Contents
  * Overview
  * Key Differences from realsense-ros Wrapper
  * Installation
  * Usage
    * Device Discovery
    * Camera Name and Namespace
    * Parameters
    * ROS2/Robot vs Optical/Camera Coordinate Systems
    * TF from coordinate A to coordinate B
    * Extrinsics from sensor A to sensor B
    * Published Topics
    * Metadata Topic
    * Available Services
  * Troubleshooting
  * Contributing
  * License

<hr>

## Overview

The RealSense D555 camera includes a **native ROS2 interface** implemented directly in the device firmware (FreeRTOS). Unlike the traditional D400-series workflow that requires `librealsense` + the `realsense-ros` wrapper on the host, the D555 communicates directly over DDS via Ethernet — no host-side driver or wrapper node is needed.

**Key Advantages:**
- **Zero Host-Side Dependencies:** No `librealsense`, no `realsense-ros` package to install or maintain.
- **Plug-and-Play:** Connect via Ethernet, set `ROS_DOMAIN_ID`, and start using standard ROS2 CLI tools immediately.
- **Reduced Latency:** Data travels directly from firmware to the DDS network with no intermediate processing layer.
- **Embedded Reliability:** The ROS2 node runs on the device's real-time OS (FreeRTOS), ensuring deterministic behavior.
- **Standard Compliance:** Fully compatible with `ros2 topic`, `ros2 service`, `ros2 param`, and `rqt`.

<hr>

## Key Differences from realsense-ros Wrapper

| Aspect | D4xx (realsense-ros wrapper) | D5XX (Native Firmware) |
|:-------|:-----------------------------|:-----------------------|
| **Connection** | USB 3.x / GMSL / MIPI | Ethernet (Gigabit) |
| **Host dependencies** | `librealsense` + `realsense-ros` | **None** |
| **ROS2 node runs on** | Host PC | Device firmware (FreeRTOS) |
| **Launch mechanism** | `ros2 launch realsense2_camera rs_launch.py` | Automatic on device power-on |
| **Parameter syntax** | `depth_module.emitter_enabled` | `Depth.option.Emitter_Enabled` |
| **Topic namespace** | `/camera/camera/<stream>/...` | `/realsense/<DeviceModel>_<Serial>_<Stream>/...` |
| **Image streams** | Depth, Color, IR1, IR2, IMU | Depth, Color, IR1, IR2, Color/compressed, IMU, ObjectDetection; aligned depth is parameter-gated |
| **Camera Info** | ✅ Published | ✅ Published |
| **TF / Extrinsics** | ✅ Published (`/tf`, `/tf_static`) | ✅ Published (`/tf_static`) |
| **Metadata** | ✅ `realsense2_camera_msgs/msg/Metadata` | ✅ `std_msgs/msg/String` (JSON); pre-7.58 firmware also exposed legacy metadata |
| **Point Cloud** | ✅ Host-generated | ✅ Native PointCloud2 publisher, gated by `Depth.option.Enable_PointCloud` |
| **ROS Version** | Humble, Jazzy, Kilted, Rolling | Humble; Jazzy requires Cyclone DDS |

<hr>

## Installation

### Prerequisites
- **Network:** The D555 device must be on the same network subnet as the host PC (e.g., `192.168.11.x/24`, subnet mask `255.255.255.0`).
- **Multicast:** The network must support UDP multicast for DDS discovery.
- **ROS2 Distribution:** Humble or Jazzy.
- **DDS Middleware:** Jazzy must use Cyclone DDS (`rmw_cyclonedds_cpp`).
- **MTU:** 9000 (this is the factory default and can be changed via device configuration). Jumbo frames are required on both host and device.

### Step 1: Install a ROS2 Distribution

Follow the official installation guide for your platform:

- **Ubuntu 22.04:** ROS2 Humble installation guide at `https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html`
- **Ubuntu 24.04:** ROS2 Jazzy installation guide at `https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html`

For Jazzy hosts, install and select Cyclone DDS before using the D555 native ROS
interface:

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

For Humble hosts:

```bash
source /opt/ros/humble/setup.bash
```

### Step 1.5: Update Device Firmware

For the best experience, ensure your D555 is running the latest firmware:

1. Download the latest firmware from the RealSense D500 firmware releases page: `https://dev.realsenseai.com/docs/firmware-release-d500` (including early-access builds when available).
2. Use `realsense-viewer` or `rs-fw-update` (from `librealsense`) to flash the firmware via USB.
3. After updating, the device will reboot and reconnect automatically.

### Step 2: Connect the D555 Camera

1. Connect the D555 to the host PC (or switch) via Ethernet cable. The D555 requires **Power over Ethernet (PoE)** — use a PoE-capable switch or PoE injector.
2. Ensure both host and device are on the same subnet (e.g., `192.168.11.x`). Default D555 IP is `192.168.11.55`.
3. Set the `ROS_DOMAIN_ID` to match the device's configured domain (default `0`):
   ```bash
   export ROS_DOMAIN_ID=0
   ```
4. On Jazzy, keep Cyclone DDS selected in every shell that runs ROS commands:
   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ```

### Step 3: Verify Device Discovery

No additional packages are needed. Verify the device is visible:

```bash
ros2 node list
# Expected output:
# /D555_343122300393
```

> **Note:** Unlike `realsense-ros`, there is no `ros2 launch` or `ros2 run` command to start the camera node. The node starts automatically when the device boots.

> **DDS Discovery:** If the device does not appear immediately, allow up to 30 seconds for DDS discovery to complete. In some network configurations, you may need to increase the DDS discovery timeout. Refer to your DDS middleware documentation (e.g., FastDDS, CycloneDDS) for details on configuring discovery-related timeouts.

> **Jazzy Note:** The D555 native ROS interface requires Cyclone DDS on Jazzy.
> If `ros2 node list` does not discover the camera on Jazzy, verify
> `echo $RMW_IMPLEMENTATION` prints `rmw_cyclonedds_cpp`.

<hr>

## Usage

### Device Discovery

After connecting the D555 via Ethernet, verify discovery:

```bash
# List ROS2 nodes
ros2 node list
# /D555_343122300393

# List all topics
ros2 topic list

# List all services
ros2 service list | grep D555
```

<hr>

<details>
<summary>Detailed ROS interface reference</summary>

### Camera Name and Namespace

The D555 firmware automatically names its ROS2 node and topics based on the device model and serial number.

**Naming Convention:**
- **Node name:** `/<DeviceModel>_<SerialNumber>` (e.g., `/D555_343122300393`)
- **Topic prefix:** `/realsense/<DeviceModel>_<SerialNumber>_<StreamName>/` (e.g., `/realsense/D555_343122300393_Depth/`)

**Example output:**

```bash
> ros2 node list
/D555_343122300393

> ros2 topic list
/realsense/D555_343122300393_Color
/realsense/D555_343122300393_Color/camera_info
/realsense/D555_343122300393_Color/compressed
/realsense/D555_343122300393_Color/metadata
/realsense/D555_343122300393_Depth
/realsense/D555_343122300393_Depth/camera_info
/realsense/D555_343122300393_Depth/metadata
/realsense/D555_343122300393_Infrared_1
/realsense/D555_343122300393_Infrared_1/camera_info
/realsense/D555_343122300393_Infrared_1/metadata
/realsense/D555_343122300393_Infrared_2
/realsense/D555_343122300393_Infrared_2/camera_info
/realsense/D555_343122300393_Infrared_2/metadata
/realsense/D555_343122300393_Motion
/realsense/D555_343122300393_Motion/metadata
/realsense/D555_343122300393_ObjectDetection
/realsense/D555_343122300393/tf_static

> ros2 service list | grep D555
/D555_343122300393/describe_parameters
/D555_343122300393/get_device_info
/D555_343122300393/get_device_info_std
/D555_343122300393/get_parameter_types
/D555_343122300393/get_parameters
/D555_343122300393/help
/D555_343122300393/hw_reset
/D555_343122300393/list_parameters
/D555_343122300393/set_parameters
/D555_343122300393/set_parameters_atomically
```

> **Mapping to realsense-ros:** In `realsense-ros`, topics appear as `/camera/camera/depth/image_rect_raw`. In D555 native mode, the equivalent topic is `/realsense/D555_<Serial>_Depth` (type `sensor_msgs/msg/Image`).

<hr>

### Parameters

> **Note:** Parameter names and values may change in future firmware updates.

#### Available Parameters

- For the entire list of parameters: `ros2 param list /D555_<Serial>`
- For reading a parameter value: `ros2 param get /D555_<Serial> <parameter_name>`
  - For example: `ros2 param get /D555_343122300393 Depth.option.Emitter_Enabled`
- For setting a new value: `ros2 param set /D555_<Serial> <parameter_name> <value>`
  - For example: `ros2 param set /D555_343122300393 Depth.option.Emitter_Enabled 1`

#### Parameter Naming Convention

Most parameters follow the format: `<Sensor>.<Group>.<Name>`. Depth filters use
`Depth.filter.<FilterName>.<OptionName>`.

| Component | Values | Description |
|:----------|:-------|:------------|
| **Sensor** | `Depth`, `RGB`, `Motion` | Target sensor module |
| **Group** | `option`, `Profile`, `filter` | `option` for controls, `Profile` for stream configuration, `filter` for depth filters |
| **Name** | e.g., `Gain`, `Exposure`, `Emitter_Enabled`, `Temporal.Toggle` | Specific control name |

> **Compatibility Note:** Use the exact names reported by `ros2 param list`.
> On r58.3, underscore-separated group names such as `Depth.option_Gain` are
> invalid and are expected to return `Parameter not set.` Use
> `Depth.option.Gain` only if it appears in `ros2 param list`.

#### Mapping from realsense-ros Parameters

| realsense-ros (D4xx) | D555 Native | Description |
| :--- | :--- | :--- |
| `depth_module.emitter_enabled` | `Depth.option.Emitter_Enabled` | Enable/disable laser emitter |
| `depth_module.gain` | `Depth.option.Gain` | Depth sensor gain |
| `depth_module.exposure` | `Depth.option.Exposure` | Depth sensor exposure (μs) |
| `depth_module.enable_auto_exposure` | `Depth.option.Enable_Auto_Exposure` | Auto exposure on/off |
| `depth_module.laser_power` | `Depth.option.Laser_Power` | Laser power [0–360] |
| `depth_module.depth_profile` | `Depth.Profile` | Stream profile (e.g., `"z16, 896, 504, 30"`) |
| `align_depth.enable` | `Depth.option.Align_Depth` | Enable on-device depth-to-color alignment topic |
| `pointcloud.enable` | `Depth.option.Enable_PointCloud` | Enable on-device PointCloud2 publisher |
| `spatial_filter.*`, `temporal_filter.*`, `decimation_filter.*` | `Depth.filter.*` | On-device depth filters |
| `rgb_camera.color_profile` | `RGB.Profile` | RGB profile (e.g., `"yuv, 896, 504, 30"`) |
| `rgb_camera.gain` | `RGB.option.Gain` | RGB sensor gain |
| `rgb_camera.exposure` | `RGB.option.Exposure` | RGB sensor exposure (μs) |
| `rgb_camera.enable_auto_exposure` | `RGB.option.Enable_Auto_Exposure` | Auto exposure on/off |
| `rgb_camera.brightness` | `RGB.option.Brightness` | Brightness [-64–64] |
| `rgb_camera.contrast` | `RGB.option.Contrast` | Contrast [0–100] |
| `rgb_camera.saturation` | `RGB.option.Saturation` | Saturation [0–100] |
| `rgb_camera.sharpness` | `RGB.option.Sharpness` | Sharpness [0–100] |
| `rgb_camera.white_balance` | `RGB.option.White_Balance` | White balance [2800–6500] K |
| `rgb_camera.enable_auto_white_balance` | `RGB.option.Enable_Auto_White_Balance` | Auto WB on/off |

#### Depth Sensor Parameters

| Parameter | Type | Range | Default | Description |
|:----------|:-----|:------|:--------|:------------|
| `Depth.option.Gain` | Integer | 16–248 | 16 | Sensor gain |
| `Depth.option.Exposure` | Integer | 1–200000 | Auto | Exposure time (μs) |
| `Depth.option.Emitter_Enabled` | Integer | 0–1 | 1 | Laser emitter on/off |
| `Depth.option.Laser_Power` | Integer | 0–360 | 150 | Laser power level |
| `Depth.option.Enable_Auto_Exposure` | Integer | 0–1 | 1 | Auto-exposure on/off |
| `Depth.option.Align_Depth` | Integer | 0–1 | 0 | Publish `/realsense/<SN>_Aligned_Depth_To_Color` |
| `Depth.option.Enable_PointCloud` | Integer | 0–2 | 0 | Publish `/realsense/<SN>_Depth_Color_Points` (`1` = XYZ, `2` = XYZRGB) |
| `Depth.Profile` | String | — | `"z16, 896, 504, 30"` | Profile: `<format>, <W>, <H>, <FPS>` |

#### Depth Filter Parameters

The following names describe the r58.3 native ROS parameter surface. Some
underlying filters existed before r58.3, but this README documents the r58.3
ROS-facing names and tested behavior. Availability is firmware-dependent; use
`ros2 param list /D555_<Serial>` as the source of truth.

| Parameter | Type | Description |
|:----------|:-----|:------------|
| `Depth.filter.Temporal.Toggle` | Integer | Enable/disable temporal filtering |
| `Depth.filter.Temporal.Alpha` | Double | Temporal filter alpha |
| `Depth.filter.Temporal.Delta` | Integer | Temporal filter delta |
| `Depth.filter.Temporal.Persistency` | Integer | Temporal persistency mode |
| `Depth.filter.Decimation.Toggle` | Integer | Enable/disable decimation filtering |
| `Depth.filter.Decimation.Magnitude` | Integer | Decimation magnitude |
| `Depth.filter.Improved_Close_Range_Depth.Enable` | Integer | Enable/disable improved close range depth |

Filter restrictions on r58.3:
- Apply graph-selection filters, such as decimation, before starting Depth,
  AlignedDepth, PointCloud, or ObjectDetection subscribers.
- Decimation + temporal is the validated depth-filter combination.
- ObjectDetection and depth filters are not concurrent workflows; stop OD before
  enabling depth filters, and stop depth-filter streaming before enabling OD
  distance.
- Do not combine `Depth.filter.Improved_Close_Range_Depth.Enable` with
  decimation on r58.3. Configure improved close range depth as a separate mode.
- Unsupported or active-stream combinations may reject `ros2 param set` with
  `Invalid value`; verify the applied state with `ros2 param get`.

#### Object Detection Parameters

| Parameter | Type | Description |
|:----------|:-----|:------------|
| `ObjectDetection.Profile` | Integer | Read-only object detection FPS profile; tested value: `30` |
| `ObjectDetection.option.Object_Distance` | Integer | 0/1 toggle. When enabled and accepted, ObjectDetection JSON may include a per-detection `distance` field in meters |

#### RGB Sensor Parameters

| Parameter | Type | Range | Default | Description |
|:----------|:-----|:------|:--------|:------------|
| `RGB.option.Gain` | Integer | 16–248 | Auto | Sensor gain |
| `RGB.option.Exposure` | Integer | 1–10000 | Auto | Exposure time (μs) |
| `RGB.option.Brightness` | Integer | -64–64 | 0 | Brightness adjustment |
| `RGB.option.Contrast` | Integer | 0–100 | 50 | Contrast level |
| `RGB.option.Saturation` | Integer | 0–100 | 64 | Saturation level |
| `RGB.option.Sharpness` | Integer | 0–100 | 50 | Sharpness level |
| `RGB.option.White_Balance` | Integer | 2800–6500 | Auto | White balance (K) |
| `RGB.option.Enable_Auto_White_Balance` | Integer | 0–1 | 1 | Auto white balance |
| `RGB.option.Enable_Auto_Exposure` | Integer | 0–1 | 1 | Auto-exposure on/off |
| `RGB.Profile` | String | — | `"yuv, 896, 504, 30"` | Profile: `<format>, <W>, <H>, <FPS>` |

#### Validation

When `set_parameters` is called, the firmware validates each parameter:

1. **Existence:** Does the parameter name exist?
2. **Type:** Does the value type match the parameter definition?
3. **Read-Only:** Is the parameter read-only?
4. **Range:** Is the value within `[min, max]`?
5. **Step:** (Optional) Is `(value - min) % step == 0`?

If validation fails, the response includes a descriptive `reason` string:
- `"Out of range"` — Value below min or above max.
- `"Read only"` — Attempted to write a read-only parameter.
- `"Type mismatch"` — Wrong value type.
- `"Unsupported"` — Parameter not supported in current mode.
- `"Internal error"` — Hardware communication failure.

#### Example: Setting Manual Exposure

```bash
# 1. Disable Auto Exposure
ros2 param set /D555_343122300393 Depth.option.Enable_Auto_Exposure 0

# 2. Set Exposure Value (microseconds)
ros2 param set /D555_343122300393 Depth.option.Exposure 5000

# 3. Verify
ros2 param get /D555_343122300393 Depth.option.Exposure
```

#### Example: Enabling r58.3 Features

Keep parameter/service commands rate-limited to about 1-2 calls per second.

```bash
# Enable temporal filtering and verify the setting
ros2 param set /D555_343122300393 Depth.filter.Temporal.Toggle 1
ros2 param get /D555_343122300393 Depth.filter.Temporal.Toggle

# Enable on-device aligned depth
ros2 param set /D555_343122300393 Depth.option.Align_Depth 1
ros2 topic echo /realsense/D555_343122300393_Aligned_Depth_To_Color \
  --once --no-arr --qos-reliability best_effort

# Enable native PointCloud2 publisher.
# Use 1 for XYZ or 2 for XYZRGB.
ros2 param set /D555_343122300393 Depth.option.Enable_PointCloud 2
python3 apps/show_ros_image.py \
  --serial 343122300393 --stream PointCloud --duration 10
ros2 param set /D555_343122300393 Depth.option.Enable_PointCloud 0

# Enable object distance in ObjectDetection output.
# Stop Depth/AlignedDepth subscribers first; Object Distance uses the
# depth-cache pipeline and is rejected while normal depth streaming is active.
ros2 param set /D555_343122300393 ObjectDetection.option.Object_Distance 1
ros2 topic echo /realsense/D555_343122300393_ObjectDetection \
  --once --qos-reliability best_effort
```

On the tested D555 r58.3 firmware, aligned depth produced
`sensor_msgs/msg/Image` samples and native PointCloud2 produced
`sensor_msgs/msg/PointCloud2` samples with `x,y,z,rgb` fields. The
`apps/show_ros_image.py --stream PointCloud` smoke test keeps hidden Depth and
Color subscribers alive because the native XYZRGB PointCloud path is most stable
when the Depth, Color, and PointCloud readers are active together. Avoid running
multiple PointCloud readers, such as `ros2 topic hz` and `ros2 topic echo`, at
the same time.

ObjectDetection output is scene/model dependent. `ObjectDetection.option.Object_Distance=1`
is accepted when the depth-cache pipeline is available, but returns
`Invalid value` if normal depth streaming is already active. Stop Depth and
AlignedDepth subscribers before enabling Object Distance.

<hr>

### ROS2/Robot vs Optical/Camera Coordinate Systems

- **Point of View:** Imagine standing behind the camera, looking forward.
- **ROS2 Coordinate System:** (X: Forward, Y: Left, Z: Up)
- **Camera Optical Coordinate System:** (X: Right, Y: Down, Z: Forward)
- **References:** `REP-0103` and `REP-0105`
- All image/depth data is published in the **optical** coordinate frame.
- The `/tf_static` topic provides transforms between the optical and ROS coordinate frames.

<hr>

### TF from coordinate A to coordinate B

- A TF message expresses a transform from coordinate frame `header.frame_id` (source) to `child_frame_id` (destination). Reference: `http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Transform.html`.
- In RealSense cameras, the origin point (0,0,0) is at the **left IR (`Infrared_1`)** sensor, named `camera_link`.
- Depth, left IR, and `camera_link` coordinates converge together.
- The D555 firmware publishes static TFs to `/tf_static` between each sensor coordinate frame and the camera base (`camera_link`), as well as from each sensor's ROS coordinates to its optical coordinates.

**TF Tree:**
```
camera_link (base)
├── camera_depth_frame
│   └── camera_depth_optical_frame
├── camera_color_frame
│   └── camera_color_optical_frame
├── camera_infra1_frame
│   └── camera_infra1_optical_frame
├── camera_infra2_frame
│   └── camera_infra2_optical_frame
└── camera_imu_frame
    └── camera_imu_optical_frame
```

**View static transforms:**
```bash
ros2 topic echo /realsense/D555_343122300393/tf_static --once
```

<hr>

### Extrinsics from sensor A to sensor B

"Extrinsic from A to B" means: a transform that converts coordinates expressed in frame A into coordinates expressed in frame B.

The D555 factory-calibrated extrinsics are encoded in the static TF transforms published on `/tf_static`.

<hr>

### Published Topics

The published topics differ based on the device configuration and active streams. After device discovery, the following topics are typically available:

| Topic | Type | QoS | Description |
|:------|:-----|:----|:------------|
| `/realsense/<SN>_Depth` | `sensor_msgs/msg/Image` | Best Effort | 16-bit depth map |
| `/realsense/<SN>_Color` | `sensor_msgs/msg/Image` | Best Effort | RGB color image |
| `/realsense/<SN>_Color/compressed` | `sensor_msgs/msg/CompressedImage` | Best Effort | JPEG-compressed color |
| `/realsense/<SN>_Infrared_1` | `sensor_msgs/msg/Image` | Best Effort | Left IR image |
| `/realsense/<SN>_Infrared_2` | `sensor_msgs/msg/Image` | Best Effort | Right IR image |
| `/realsense/<SN>_Motion` | `sensor_msgs/msg/Imu` | Best Effort | IMU (gyro + accel) data |
| `/realsense/<SN>_ObjectDetection` | `std_msgs/msg/String` | Best Effort | Object detection JSON output |
| `/realsense/<SN>_<Stream>/camera_info` | `sensor_msgs/msg/CameraInfo` | Best Effort | Camera intrinsics & calibration |
| `/realsense/<SN>_<Stream>/metadata` | `std_msgs/msg/String` | Best Effort | Per-frame metadata (JSON) |
| `/realsense/<SN>/tf_static` | `tf2_msgs/msg/TFMessage` | Reliable, Transient Local | Static coordinate transforms |

Where `<SN>` = `D555_<SerialNumber>` (e.g., `D555_343122300393`).

Additional r58.3 topics appear only after enabling their parameters:

| Topic | Type | Enable Parameter |
|:------|:-----|:-----------------|
| `/realsense/<SN>_Aligned_Depth_To_Color` | `sensor_msgs/msg/Image` | `Depth.option.Align_Depth=1` |
| `/realsense/<SN>_Aligned_Depth_To_Color/camera_info` | `sensor_msgs/msg/CameraInfo` | `Depth.option.Align_Depth=1` |
| `/realsense/<SN>_Depth_Color_Points` | `sensor_msgs/msg/PointCloud2` | `Depth.option.Enable_PointCloud=1` or `2` |

**Mapping to realsense-ros topics:**

| realsense-ros (D4xx) | D555 Native | Type |
| :--- | :--- | :--- |
| `/camera/camera/depth/image_rect_raw` | `/realsense/D555_<Serial>_Depth` | `sensor_msgs/msg/Image` |
| `/camera/camera/color/image_raw` | `/realsense/D555_<Serial>_Color` | `sensor_msgs/msg/Image` |
| `/camera/camera/color/image_raw/compressed` | `/realsense/D555_<Serial>_Color/compressed` | `sensor_msgs/msg/CompressedImage` |
| `/camera/camera/infra1/image_rect_raw` | `/realsense/D555_<Serial>_Infrared_1` | `sensor_msgs/msg/Image` |
| `/camera/camera/infra2/image_rect_raw` | `/realsense/D555_<Serial>_Infrared_2` | `sensor_msgs/msg/Image` |
| `/camera/camera/depth/camera_info` | `/realsense/D555_<Serial>_Depth/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/camera/camera/color/camera_info` | `/realsense/D555_<Serial>_Color/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/camera/camera/depth/metadata` | `/realsense/D555_<Serial>_Depth/metadata` | `std_msgs/msg/String` |
| `/camera/camera/color/metadata` | `/realsense/D555_<Serial>_Color/metadata` | `std_msgs/msg/String` |
| `/camera/camera/aligned_depth_to_color/image_raw` | `/realsense/D555_<Serial>_Aligned_Depth_To_Color` | `sensor_msgs/msg/Image` |
| `/camera/camera/depth/color/points` | `/realsense/D555_<Serial>_Depth_Color_Points` | `sensor_msgs/msg/PointCloud2` |
| `/camera/camera/imu` | `/realsense/D555_<Serial>_Motion` | `sensor_msgs/msg/Imu` |
| `/tf_static` | `/realsense/D555_<Serial>/tf_static` | `tf2_msgs/msg/TFMessage` |

**Subscribe to a stream:**
```bash
# View depth image info
ros2 topic echo /realsense/D555_343122300393_Depth --no-arr

# View camera info
ros2 topic echo /realsense/D555_343122300393_Depth/camera_info --once
```

<hr>

### Metadata Topic

The D555 publishes per-stream metadata containing hardware-level frame information in JSON format.

> **Firmware behavior change:** Starting with firmware version **7.58 and later**, the device exposes only the ROS2-native metadata topic (`std_msgs/msg/String`). On firmware versions **prior to 7.58**, both the native and legacy metadata topics were published simultaneously.

**Topic variants:**

1. **Standard** (`metadata`): Uses `std_msgs/msg/String` — works with any ROS2 installation. Published on all firmware versions.
2. **Legacy** (`metadata_legacy`): Uses `realsense2_camera_msgs/msg/Metadata` — D455-compatible format with `Header` (timestamp + `frame_id`) + `json_data`. Only available on firmware versions prior to 7.58.

**Available metadata topics per stream:**
```
/realsense/<SN>_Depth/metadata              (std_msgs/msg/String)
/realsense/<SN>_Color/metadata
/realsense/<SN>_Infrared_1/metadata
/realsense/<SN>_Infrared_2/metadata
/realsense/<SN>_Motion/metadata
```

`metadata_legacy` topics are not present by default on tested 7.58 firmware.

**Echo metadata from the command line:**
```bash
ros2 topic echo /realsense/D555_343122300393_Depth/metadata
```

**Metadata JSON fields:**

| Field | Type | Unit | Description |
|:------|:-----|:-----|:------------|
| `Frame Counter` | uint32 | count | Hardware frame counter (drop detection) |
| `Frame Timestamp` | uint64 | μs | Timestamp when frame was sent |
| `Sensor Timestamp` | uint64 | μs | Hardware timestamp from MIPI readout |
| `Actual Exposure` | uint32 | μs | Exact exposure time for this frame |
| `Gain Level` | uint16 | — | Applied gain value |
| `Auto Exposure` | uint8 | — | Auto-exposure mode |
| `Brightness` | int8 | — | Brightness setting (-64 to 64) |
| `Contrast` | uint8 | — | Contrast setting (0–100) |
| `Saturation` | uint8 | — | Saturation setting (0–100) |
| `Sharpness` | uint8 | — | Sharpness setting (0–100) |
| `Gamma` | uint16 | — | Gamma value |
| `Hue` | int8 | — | Hue setting |
| `Manual White Balance` | uint16 | K | White balance temperature |
| `Auto White Balance Temperature` | uint16 | K | AWB result |
| `Laser Power` | uint16 | — | Emitter power setting |
| `Emitter Mode` | uint8 | — | Projector on/off state |
| `ASIC Temperature` | float | °C | Real-time sensor temperature |
| `Input Width` / `Input Height` | uint16 | pixels | Image dimensions |

**Python subscriber example:**
```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import json

class MetadataSubscriber(Node):
    def __init__(self):
        super().__init__('metadata_subscriber')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            String,
            '/realsense/D555_343122300393_Depth/metadata',
            self.callback,
            qos)

    def callback(self, msg):
        data = json.loads(msg.data)
        md = data['metadata']
        print(f"Frame: {md['Frame Counter']}, "
              f"Exposure: {md['Actual Exposure']}μs, "
              f"Gain: {md['Gain Level']}, "
              f"Temp: {md['ASIC Temperature']}°C")

rclpy.init()
node = MetadataSubscriber()
rclpy.spin(node)
```

<hr>

### Available Services

The device exposes the following services under the namespace `/<DeviceModel>_<SerialNumber>`:

#### Hardware Reset
- **Service name:** `hw_reset`
- Reset the device. All streams are stopped.
- **Type:** `std_srvs/srv/Empty`
- Call example:
  ```bash
  ros2 service call /D555_343122300393/hw_reset std_srvs/srv/Empty
  ```
- **Note:** The device will disconnect from DDS and reappear after approximately 5–10 seconds.

#### Device Information
- **Service names:** `get_device_info`, `get_device_info_std`
- Retrieve device information: serial number, firmware version, sensors, and transport.
- **Types:** `realsense2_camera_msgs/srv/DeviceInfo` and `std_srvs/srv/Trigger`
- Typed service example:
  ```bash
  ros2 service call /D555_343122300393/get_device_info \
    realsense2_camera_msgs/srv/DeviceInfo "{}"
  ```
  This returns typed fields such as `device_name`, `serial_number`,
  `firmware_version`, `sensors`, and `physical_port`.
- Standard Trigger service example:
  ```bash
  ros2 service call /D555_343122300393/get_device_info_std std_srvs/srv/Trigger
  ```
- Trigger response: `success` (bool) + `message` (JSON string) with device fields:

  | Field | Example Value |
  |:------|:--------------|
  | `serial` | `"343122300393"` |
  | `product` | `"D555"` |
  | `firmware` | `"7.58.40177.x"` |

- `Device.Info` parameter:
  ```bash
  ros2 param get /D555_343122300393 Device.Info
  ros2 param describe /D555_343122300393 Device.Info
  ```
  `param get` returns compact JSON that fits the ROS parameter string response.
  `param describe` provides the full JSON device description.

#### Help
- **Service name:** `help`
- List all available services and their types.
- **Type:** `std_srvs/srv/Trigger`
- Call example:
  ```bash
  ros2 service call /D555_343122300393/help std_srvs/srv/Trigger
  ```

#### List Parameters
- **Service name:** `list_parameters`
- Returns a list of all available parameters.
- **Type:** `rcl_interfaces/srv/ListParameters`
- Call example:
  ```bash
  ros2 service call /D555_343122300393/list_parameters rcl_interfaces/srv/ListParameters
  ```

#### Get Parameters
- **Service name:** `get_parameters`
- Retrieve current values of specified parameters.
- **Type:** `rcl_interfaces/srv/GetParameters`
- Call example:
  ```bash
  ros2 service call /D555_343122300393/get_parameters \
    rcl_interfaces/srv/GetParameters \
    "{names: ['Depth.option.Gain', 'Depth.option.Exposure']}"
  ```
- Returns `PARAMETER_NOT_SET` for unknown parameters.

#### Set Parameters
- **Service name:** `set_parameters`
- Set parameter values with range validation.
- **Type:** `rcl_interfaces/srv/SetParameters`
- Call example:
  ```bash
  ros2 service call /D555_343122300393/set_parameters \
    rcl_interfaces/srv/SetParameters \
    "{parameters: [{name: 'Depth.option.Gain', value: {type: 2, integer_value: 100}}]}"
  ```
- **Response:** `successful` (bool) + `reason` (string) per parameter.

#### Describe Parameters
- **Service name:** `describe_parameters`
- Returns metadata (type, description, range) for parameters.
- **Type:** `rcl_interfaces/srv/DescribeParameters`
- Call example:
  ```bash
  ros2 service call /D555_343122300393/describe_parameters \
    rcl_interfaces/srv/DescribeParameters \
    "{names: ['Depth.option.Gain']}"
  ```

**Mapping to realsense-ros services:**

| realsense-ros (D4xx) | D555 Native | Type |
| :--- | :--- | :--- |
| `/camera/camera/hw_reset` | `/D555_<Serial>/hw_reset` | `std_srvs/srv/Empty` |
| `/camera/camera/device_info` | `/D555_<Serial>/get_device_info` | `realsense2_camera_msgs/srv/DeviceInfo` |
| `/camera/camera/get_parameters` | `/D555_<Serial>/get_parameters` | `rcl_interfaces/srv/GetParameters` |
| `/camera/camera/set_parameters` | `/D555_<Serial>/set_parameters` | `rcl_interfaces/srv/SetParameters` |
| `/camera/camera/list_parameters` | `/D555_<Serial>/list_parameters` | `rcl_interfaces/srv/ListParameters` |
| `/camera/camera/describe_parameters` | `/D555_<Serial>/describe_parameters` | `rcl_interfaces/srv/DescribeParameters` |
| — | `/D555_<Serial>/get_device_info_std` | `std_srvs/srv/Trigger` |
| — | `/D555_<Serial>/help` | `std_srvs/srv/Trigger` |
| — | `/D555_<Serial>/get_parameter_types` | `rcl_interfaces/srv/GetParameterTypes` |

**Python service client example:**
```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

rclpy.init()
node = Node('param_client')

# Get a parameter
get_client = node.create_client(GetParameters, '/D555_343122300393/get_parameters')
get_client.wait_for_service()
request = GetParameters.Request()
request.names = ['Depth.option.Gain']
future = get_client.call_async(request)
rclpy.spin_until_future_complete(node, future)
print(f"Gain = {future.result().values[0].integer_value}")

# Set a parameter
set_client = node.create_client(SetParameters, '/D555_343122300393/set_parameters')
set_client.wait_for_service()
param = Parameter()
param.name = 'Depth.option.Gain'
param.value = ParameterValue()
param.value.type = ParameterType.PARAMETER_INTEGER
param.value.integer_value = 100
request = SetParameters.Request()
request.parameters = [param]
future = set_client.call_async(request)
rclpy.spin_until_future_complete(node, future)
result = future.result().results[0]
print(f"Success: {result.successful}, Reason: {result.reason}")

node.destroy_node()
rclpy.shutdown()
```

<hr>

</details>

## Performance & Constraints

### Latency
| Operation | Typical Latency |
|:----------|:----------------|
| Service call | 10–50 ms |
| Parameter set | 20–100 ms |
| Stream (internal processing) | < 5 ms + network |

### Throughput
- **Interface:** Gigabit Ethernet
- **Theoretical max (all streams):** ~266 Mbps uncompressed
- **Practical (with JPEG compression):** ~70–110 Mbps

### High-bandwidth DDS pacing

Depth and compressed color profiles may report 30 FPS while ROS subscribers only
receive about 15-23 FPS. This usually means the device is still producing frames
at 30 FPS, but large DDS/UDP samples are being dropped on the receive path. You
can confirm this by checking message timestamps:

```bash
ros2 topic echo --qos-reliability best_effort \
  /realsense/D555_<Serial>_Depth --field header.stamp
```

If the header timestamp increments by about 33.3 ms but `ros2 topic echo`
prints `A message was lost!!!`, the camera cadence is 30 FPS and the loss is in
the DDS/network receive path.

On r58.3, `Device.Transmission_Delay` is exposed as a ROS parameter for the
device DDS traffic-shaping setting. It adds microsecond-level pacing to large
DDS data fragments. This reduces Ethernet/DDS microbursts without changing the
stream profile:

```bash
ros2 param set /D555_<Serial> Device.Transmission_Delay 36
ros2 topic hz /realsense/D555_<Serial>_Depth
ros2 topic hz /realsense/D555_<Serial>_Color/compressed
```

On the tested D555 r58.3 firmware, `Device.Transmission_Delay=36` restored
Depth and Color/compressed from about 15-23 FPS to about 30 FPS with MTU 9000.
This can happen even with a single camera when the host subscribes to multiple
large DDS samples at the same time, such as Depth plus Color/compressed,
AlignedDepth, or PointCloud2. Low-bandwidth single-topic tests usually do not
need this setting. Use `48` for a more conservative stress-test setting if
multiple large streams still report dropped samples.

### Known Limitations

> **Note:** The following limitations apply to firmware version **7.58.x**. Some may be resolved in future firmware releases.

1. **Request Rate Limiting:** Limit service calls to approximately 1–2 per second, or add a 500 ms delay between burst requests. The SafeDDS ACK window is limited.
2. **Batch Parameter Semantics:** Prefer `set_parameters_atomically` when multiple parameters must be applied together; plain `set_parameters` returns one result per parameter.
3. **String Length Limits:** Parameter names and string values use fixed-size firmware buffers. `Device.Info` returns compact JSON from `ros2 param get`; use `ros2 param describe` or `get_device_info` for the full device description.
4. **Native PointCloud2 Usage:** r58.3 exposes `/realsense/<SN>_Depth_Color_Points` after `Depth.option.Enable_PointCloud=1` (XYZ) or `2` (XYZRGB). Use one long-lived PointCloud subscriber and keep Depth + Color readers active for the most stable XYZRGB path. `apps/show_ros_image.py --stream PointCloud` adds those hidden guard subscribers automatically.
5. **ObjectDetection Output:** `/realsense/<SN>_ObjectDetection` may publish JSON with `number_of_detections: 0` in an empty scene. `ObjectDetection.option.Object_Distance=1` can return `Invalid value` if Depth or AlignedDepth streaming is already active; stop those subscribers before enabling the depth-cache distance path.
6. **Depth Filter Combinations:** Apply graph-selection filters before starting streams. Decimation + temporal is the validated depth-filter combination; do not combine improved close range depth with decimation on r58.3. OD distance and depth-filter streaming are separate workflows.
7. **High-bandwidth DDS Loss:** With MTU 9000 and `Device.Transmission_Delay=0`, some hosts may receive Depth at about 15-17 FPS and Color/compressed at about 21-23 FPS even though the profile is 30 FPS. This is most visible when one camera publishes multiple large topics. Set `Device.Transmission_Delay` to `36` us before high-bandwidth tests.
8. **Parameter Name Compatibility:** `Depth.option_Gain` and other underscore-separated legacy aliases are not supported on r58.3. Test scripts should discover parameters with `ros2 param list` and skip options that are absent.

<hr>

## Troubleshooting

### Device not found in `ros2 node list`
- **Cause:** Firewall blocking multicast, different `ROS_DOMAIN_ID`, wrong DDS middleware on Jazzy, or network misconfiguration.
- **Fix:**
  ```bash
  # Check ROS_DOMAIN_ID matches (default 0)
  echo $ROS_DOMAIN_ID

  # Jazzy only: verify Cyclone DDS is selected
  echo $RMW_IMPLEMENTATION
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

  # Disable firewall temporarily to test
  sudo ufw disable

  # Verify network connectivity
  ping <device_ip>
  ```

### Service call times out
- **Cause:** Device busy, network packet loss, or request flooding.
- **Fix:** Increase timeout (`--timeout 10`), add delays between requests, verify network quality.

### "Parameter not set" error
- **Cause:** Parameter name typo or unsupported parameter.
- **Fix:** Run `ros2 param list /D555_<Serial>` to verify exact parameter names. For example, `Depth.option_Gain` is not valid on r58.3. Use `Depth.option.Gain` only when it appears in the parameter list; otherwise skip that control in automated tests.

### Topic data not received / QoS mismatch
- **Cause:** Image and metadata topics use `BEST_EFFORT` reliability; subscribers must match.
- **Fix:** Set subscriber QoS to `BEST_EFFORT`:
  ```python
  from rclpy.qos import QoSProfile, ReliabilityPolicy
  qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
  ```

### Image topic FPS is lower than the 30 FPS profile
- **Cause:** The device may be producing 30 FPS, but large BEST_EFFORT DDS
  samples are dropped when the Ethernet/DDS packets arrive as tight bursts. This
  is more visible on Depth because the raw frame payload is much larger than
  JPEG-compressed color.
- **Fix:** Enable DDS bulk-fragment pacing:
  ```bash
  ros2 param set /D555_<Serial> Device.Transmission_Delay 36
  ros2 topic hz /realsense/D555_<Serial>_Depth
  ros2 topic hz /realsense/D555_<Serial>_Color/compressed
  ```
- **Verification:** If `ros2 topic echo --field header.stamp` shows 33.3 ms
  timestamp intervals but also prints `A message was lost!!!`, this is receive
  path loss, not a camera profile issue.

### CDR deserialization errors
- **Cause:** Potential firmware/middleware version mismatch.
- **Fix:** Update device firmware to the latest version.

### Factory Reset / USB Recovery
If the device becomes unreachable due to a faulty network configuration, you can recover it via USB:
1. Connect the D555 to the host PC using a USB cable.
2. Use `realsense-viewer` or `rs-fw-update` (from `librealsense`) to reset the device to factory settings.
3. This will restore the default network configuration (IP `192.168.11.55`, subnet mask `255.255.255.0`).
4. Reconnect via Ethernet and reconfigure as needed.

> **Note:** USB connectivity is intended for recovery and firmware updates only. Normal operation uses Ethernet.

<hr>

## Contributing

Please refer to the project's contribution guidelines.

<hr>

## License

Copyright © RealSense, Inc. All rights reserved.
