#!/usr/bin/env python3
"""
Launch file for sensor drivers only

This launches:
1. Unitree L2 LiDAR driver
2. Livox Mid-70 LiDAR driver  
3. Berxel P100R RGB-D camera driver
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    
    # Launch arguments
    launch_l2 = LaunchConfiguration('launch_l2')
    launch_mid70 = LaunchConfiguration('launch_mid70')
    launch_berxel = LaunchConfiguration('launch_berxel')
    
    declare_launch_l2 = DeclareLaunchArgument(
        'launch_l2',
        default_value='true',
        description='Launch Unitree L2 driver'
    )
    
    declare_launch_mid70 = DeclareLaunchArgument(
        'launch_mid70',
        default_value='true',
        description='Launch Livox Mid-70 driver'
    )
    
    declare_launch_berxel = DeclareLaunchArgument(
        'launch_berxel',
        default_value='true',
        description='Launch Berxel P100R driver'
    )
    
    # ==========================================================================
    # Unitree L2 LiDAR
    # ==========================================================================
    # Note: Replace with actual Unitree L2 ROS2 driver
    # This is a placeholder - actual driver depends on your SDK setup
    l2_driver_node = Node(
        package='unitree_lidar_ros2',  # Replace with actual package name
        executable='unitree_lidar_node',  # Replace with actual executable
        name='unitree_l2_driver',
        output='screen',
        parameters=[{
            'frame_id': 'unitree_l2_link',
            'topic': '/unitree_l2/pointcloud',
            'scan_frequency': 20.0,
        }],
        condition=IfCondition(launch_l2),
        # Uncomment when actual driver is installed
        # respawn=True,
    )
    
    # ==========================================================================
    # Livox Mid-70 LiDAR
    # ==========================================================================
    # Note: Uses livox_ros2_driver
    # Make sure livox_ros2_driver is installed
    mid70_config_path = '/path/to/livox_lidar_config.json'  # Update path
    
    mid70_driver_node = Node(
        package='livox_ros2_driver',  # Official Livox ROS2 driver
        executable='livox_ros2_driver_node',
        name='livox_mid70_driver',
        output='screen',
        parameters=[{
            'xfer_format': 0,  # 0: Pointcloud2
            'multi_topic': 0,  # Single topic
            'data_src': 0,  # 0: LiDAR data
            'publish_freq': 10.0,
            'output_type': 0,
            'frame_id': 'livox_frame',
            'lvx_file_path': '',
            'user_config_path': mid70_config_path,
        }],
        condition=IfCondition(launch_mid70),
    )
    
    # ==========================================================================
    # Berxel P100R RGB-D Camera
    # ==========================================================================
    # Note: This is a placeholder for Berxel driver
    # Replace with actual Berxel ROS2 driver configuration
    berxel_driver_node = Node(
        package='mapless_nav',  # Or berxel_camera package if available
        executable='berxel_camera_node.py',  # Python wrapper
        name='berxel_camera_driver',
        output='screen',
        parameters=[{
            'color_topic': '/berxel/color/image_raw',
            'depth_topic': '/berxel/depth/image_raw',
            'camera_info_topic': '/berxel/depth/camera_info',
            'frame_id': 'berxel_link',
            'depth_frame_id': 'berxel_depth_optical_frame',
            'fps': 30,
            'color_width': 640,
            'color_height': 480,
            'depth_width': 640,
            'depth_height': 480,
        }],
        condition=IfCondition(launch_berxel),
    )
    
    return LaunchDescription([
        # Arguments
        declare_launch_l2,
        declare_launch_mid70,
        declare_launch_berxel,
        
        # Drivers
        # Note: Uncomment drivers as they are installed and configured
        # l2_driver_node,
        # mid70_driver_node,
        # berxel_driver_node,
    ])
