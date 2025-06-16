# launch/cuboid_estimator.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('cuboid_dimension_estimator')
    
    # Declare launch arguments
    size_threshold_arg = DeclareLaunchArgument(
        'size_threshold',
        default_value='0.15',
        description='Size threshold in meters (default: 15cm)'
    )
    
    min_depth_arg = DeclareLaunchArgument(
        'min_depth',
        default_value='0.1',
        description='Minimum valid depth in meters'
    )
    
    max_depth_arg = DeclareLaunchArgument(
        'max_depth',
        default_value='5.0',
        description='Maximum valid depth in meters'
    )
    
    # Cuboid estimator node
    cuboid_estimator_node = Node(
        package='cuboid_dimension_estimator',
        executable='cuboid_estimator',
        name='cuboid_dimension_estimator',
        output='screen',
        parameters=[{
            'size_threshold': LaunchConfiguration('size_threshold'),
            'min_depth': LaunchConfiguration('min_depth'),
            'max_depth': LaunchConfiguration('max_depth')
        }],
        remappings=[
            # Uncomment and modify if you need to remap topics
            # ('/camera/image_raw', '/your_camera/image_raw'),
            # ('/camera/depth/image_raw', '/your_camera/depth/image_raw'),
            # ('/camera/camera_info', '/your_camera/camera_info'),
        ]
    )
    
    return LaunchDescription([
        size_threshold_arg,
        min_depth_arg,
        max_depth_arg,
        cuboid_estimator_node,
    ])
