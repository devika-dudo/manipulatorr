from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='camera/image_raw',
        description='Camera image topic'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='camera/camera_info',
        description='Camera info topic'
    )
    
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_link_optical',
        description='Camera frame ID (will be overridden by camera_info if available)'
    )
    
    # Enhanced parameters for the new detector
    enable_pose_estimation_arg = DeclareLaunchArgument(
        'enable_pose_estimation',
        default_value='false',
        description='Enable pose estimation (requires camera calibration)'
    )
    
    default_marker_size_arg = DeclareLaunchArgument(
        'default_marker_size',
        default_value='0.05',
        description='Default marker size in meters for pose estimation (5cm default)'
    )
    
    adaptive_pose_estimation_arg = DeclareLaunchArgument(
        'adaptive_pose_estimation',
        default_value='true',
        description='Try to estimate marker size from pixels for pose estimation'
    )
    
    show_image_arg = DeclareLaunchArgument(
        'show_image',
        default_value='true',
        description='Show detection image window'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish TF transforms for detected markers'
    )
    
    min_marker_size_pixels_arg = DeclareLaunchArgument(
        'min_marker_size_pixels',
        default_value='10',
        description='Minimum marker size in pixels to consider valid'
    )
    
    max_marker_size_pixels_arg = DeclareLaunchArgument(
        'max_marker_size_pixels',
        default_value='1000',
        description='Maximum marker size in pixels to consider valid'
    )
    
    # Enhanced ArUco detector node
    aruco_detector_node = Node(
        package='aruco_detector',  # Update this to match your package name
        executable='aruco_detector_node',  # Update this to match your executable name
        name='aruco_detector',
        parameters=[{
            'camera_frame': LaunchConfiguration('camera_frame'),
            'show_image': LaunchConfiguration('show_image'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'enable_pose_estimation': LaunchConfiguration('enable_pose_estimation'),
            'default_marker_size': LaunchConfiguration('default_marker_size'),
            'adaptive_pose_estimation': LaunchConfiguration('adaptive_pose_estimation'),
            'min_marker_size_pixels': LaunchConfiguration('min_marker_size_pixels'),
            'max_marker_size_pixels': LaunchConfiguration('max_marker_size_pixels'),
        }],
        remappings=[
            ('camera/image_raw', LaunchConfiguration('camera_topic')),
            ('camera/camera_info', LaunchConfiguration('camera_info_topic'))
        ],
        output='screen'
    )
    
    return LaunchDescription([
        camera_topic_arg,
        camera_info_topic_arg,
        camera_frame_arg,
        enable_pose_estimation_arg,
        default_marker_size_arg,
        adaptive_pose_estimation_arg,
        show_image_arg,
        publish_tf_arg,
        min_marker_size_pixels_arg,
        max_marker_size_pixels_arg,
        aruco_detector_node
    ])

# Alternative: Simple launch file for basic usage
def generate_simple_launch_description():
    """Simplified launch file with sensible defaults - no YAML needed"""
    
    aruco_detector_node = Node(
        package='aruco_detector',  # Update this to match your package name
        executable='enhanced_aruco_detector_node',  # Update this to match your executable name
        name='enhanced_aruco_detector',
        parameters=[{
            # Default parameters - works out of the box with any ArUco marker
            'camera_frame': 'camera_link_optical',
            'show_image': True,
            'publish_tf': True,
            'enable_pose_estimation': False,  # Disabled by default for unknown marker sizes
            'default_marker_size': 0.05,     # 5cm default if pose estimation is enabled
            'adaptive_pose_estimation': True,  # Try to estimate size from pixels
            'min_marker_size_pixels': 10,     # Minimum 10 pixels
            'max_marker_size_pixels': 1000,   # Maximum 1000 pixels
        }],
        remappings=[
            ('camera/image_raw', 'camera/image_raw'),
            ('camera/camera_info', 'camera/camera_info')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        aruco_detector_node
    ])
