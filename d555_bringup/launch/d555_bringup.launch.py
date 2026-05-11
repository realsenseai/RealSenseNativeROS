#!/usr/bin/env python3
"""
D555 Bringup Launch File

Launches host-side companion nodes for the Intel RealSense D555 camera:
  1. static_transform_publisher — base_link → camera_link mounting TF
  2. d555_color_relay — YUV422/NV12 → RGB8 color conversion
  3. register_node (optional) — host-side depth-to-color alignment
  4. point_cloud_xyzrgb_node (optional) — PointCloud2 generation

The D555 camera itself boots autonomously and publishes ROS2 topics over
DDS/Ethernet. This launch file does NOT start the camera — it only starts
host-side processing nodes.

Usage:
  ros2 launch d555_bringup d555_bringup.launch.py serial:=343122300393
  ros2 launch d555_bringup d555_bringup.launch.py serial:=343122300393 \\
      cam_x:=0.30 cam_z:=0.15 cam_pitch:=-0.2618
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    # Resolve all arguments
    serial = LaunchConfiguration('serial').perform(context)
    camera_name = LaunchConfiguration('camera_name').perform(context)
    base_frame = LaunchConfiguration('base_frame').perform(context)
    camera_frame = LaunchConfiguration('camera_frame').perform(context)
    cam_x = LaunchConfiguration('cam_x').perform(context)
    cam_y = LaunchConfiguration('cam_y').perform(context)
    cam_z = LaunchConfiguration('cam_z').perform(context)
    cam_roll = LaunchConfiguration('cam_roll').perform(context)
    cam_pitch = LaunchConfiguration('cam_pitch').perform(context)
    cam_yaw = LaunchConfiguration('cam_yaw').perform(context)
    use_device_align = LaunchConfiguration('use_device_align').perform(context)
    enable_pointcloud = LaunchConfiguration('enable_pointcloud').perform(context)
    enable_color_relay = LaunchConfiguration('enable_color_relay').perform(context)

    # Device topic prefix: /realsense/D555_<serial>
    device_prefix = f'/realsense/D555_{serial}'

    nodes = []

    # ── 1. Static TF: base_link → camera_link ──
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{camera_name}_tf_publisher',
        arguments=[
            '--x', cam_x,
            '--y', cam_y,
            '--z', cam_z,
            '--roll', cam_roll,
            '--pitch', cam_pitch,
            '--yaw', cam_yaw,
            '--frame-id', base_frame,
            '--child-frame-id', camera_frame,
        ],
    )
    nodes.append(tf_node)

    # ── 2. Color Relay: YUV → RGB8 ──
    if enable_color_relay.lower() == 'true':
        color_relay = Node(
            package='d555_bringup',
            executable='d555_color_relay',
            name='d555_color_relay',
            namespace=camera_name,
            remappings=[
                ('~/image_raw_yuv', f'{device_prefix}_Color'),
                ('~/image_raw', f'/{camera_name}/color/image_raw'),
            ],
            parameters=[{
                'qos_reliability': 'best_effort',
            }],
        )
        nodes.append(color_relay)

    # ── 3 & 4. Depth alignment + Point cloud ──
    if use_device_align.lower() != 'true':
        # Mode A: Host-side alignment via register_node
        if enable_pointcloud.lower() == 'true':
            register = Node(
                package='depth_image_proc',
                executable='register_node',
                name='register_node',
                namespace=camera_name,
                remappings=[
                    ('depth/image_rect', f'{device_prefix}_Depth'),
                    ('depth/camera_info', f'{device_prefix}_Depth/camera_info'),
                    ('rgb/camera_info', f'{device_prefix}_Color/camera_info'),
                    ('depth_registered/image_rect',
                     f'/{camera_name}/aligned_depth/image_raw'),
                ],
                parameters=[{
                    'use_sim_time': False,
                }],
            )
            nodes.append(register)

            pc_node = Node(
                package='depth_image_proc',
                executable='point_cloud_xyzrgb_node',
                name='point_cloud_xyzrgb',
                namespace=camera_name,
                remappings=[
                    ('depth_registered/image_rect',
                     f'/{camera_name}/aligned_depth/image_raw'),
                    ('rgb/image_rect_color',
                     f'/{camera_name}/color/image_raw'),
                    ('rgb/camera_info',
                     f'{device_prefix}_Color/camera_info'),
                    ('points', f'/{camera_name}/points2'),
                ],
                parameters=[{
                    'use_sim_time': False,
                }],
            )
            nodes.append(pc_node)
    else:
        # Mode B: Device provides aligned depth
        if enable_pointcloud.lower() == 'true':
            pc_node = Node(
                package='depth_image_proc',
                executable='point_cloud_xyzrgb_node',
                name='point_cloud_xyzrgb',
                namespace=camera_name,
                remappings=[
                    ('depth_registered/image_rect',
                     f'{device_prefix}_Aligned_Depth_To_Color'),
                    ('rgb/image_rect_color',
                     f'/{camera_name}/color/image_raw'),
                    ('rgb/camera_info',
                     f'{device_prefix}_Aligned_Depth_To_Color/camera_info'),
                    ('points', f'/{camera_name}/points2'),
                ],
                parameters=[{
                    'use_sim_time': False,
                }],
            )
            nodes.append(pc_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        # Required
        DeclareLaunchArgument(
            'serial', description='D555 camera serial number'),

        # Camera naming
        DeclareLaunchArgument(
            'camera_name', default_value='camera',
            description='Camera namespace and TF prefix'),

        # TF mounting transform
        DeclareLaunchArgument(
            'base_frame', default_value='base_link',
            description='Parent frame for the camera'),
        DeclareLaunchArgument(
            'camera_frame', default_value='camera_link',
            description='Camera root frame'),
        DeclareLaunchArgument(
            'cam_x', default_value='0.0',
            description='Mounting X translation (meters)'),
        DeclareLaunchArgument(
            'cam_y', default_value='0.0',
            description='Mounting Y translation (meters)'),
        DeclareLaunchArgument(
            'cam_z', default_value='0.0',
            description='Mounting Z translation (meters)'),
        DeclareLaunchArgument(
            'cam_roll', default_value='0.0',
            description='Mounting roll (radians)'),
        DeclareLaunchArgument(
            'cam_pitch', default_value='0.0',
            description='Mounting pitch (radians)'),
        DeclareLaunchArgument(
            'cam_yaw', default_value='0.0',
            description='Mounting yaw (radians)'),

        # Feature toggles
        DeclareLaunchArgument(
            'use_device_align', default_value='false',
            description='Use on-device aligned depth (skip host register_node)'),
        DeclareLaunchArgument(
            'enable_pointcloud', default_value='true',
            description='Launch point cloud generation node'),
        DeclareLaunchArgument(
            'enable_color_relay', default_value='true',
            description='Launch YUV→RGB8 color relay node'),

        OpaqueFunction(function=_launch_setup),
    ])
