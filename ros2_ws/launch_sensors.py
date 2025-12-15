from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    unitree_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('unitree_lidar'),
                'launch',
                'unitree_lidar_launch.py'
            ])
        ]),
        launch_arguments={'use_sdk': 'true'}.items()
    )

    berxel_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('berxel_camera_ros2'),
                'launch',
                'berxel_camera_iHawk100.py'
            ])
        ])
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('berxel_camera_ros2'), 'rviz', 'berxel_default.rviz'])]
    )

    return LaunchDescription([
        unitree_lidar_launch,
        berxel_camera_launch,
        rviz_node
    ])
