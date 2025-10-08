from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Launch person detector node with Berxel camera"""
    
    # 声明launch参数
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolov8n.pt',
        description='Path to YOLO model file'
    )
    
    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.55',
        description='YOLO confidence threshold'
    )
    
    iou_threshold_arg = DeclareLaunchArgument(
        'iou_threshold',
        default_value='0.45',
        description='YOLO IOU threshold for NMS'
    )
    
    publish_debug_arg = DeclareLaunchArgument(
        'publish_debug_image',
        default_value='true',
        description='Whether to publish debug visualization images'
    )
    
    use_tracking_arg = DeclareLaunchArgument(
        'use_tracking',
        default_value='false',
        description='Enable multi-object tracking (ByteTrack)'
    )
    
    # Person detector node
    person_detector_node = Node(
        package='person_detector',
        executable='person_detector_node',
        name='person_detector_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'iou_threshold': LaunchConfiguration('iou_threshold'),
            'publish_debug_image': LaunchConfiguration('publish_debug_image'),
            'use_tracking': LaunchConfiguration('use_tracking'),
        }],
        remappings=[
            ('person_detections', '/person_detections'),
            ('detection_debug_image', '/detection_debug_image'),
        ]
    )
    
    return LaunchDescription([
        model_path_arg,
        conf_threshold_arg,
        iou_threshold_arg,
        publish_debug_arg,
        use_tracking_arg,
        person_detector_node,
    ])
