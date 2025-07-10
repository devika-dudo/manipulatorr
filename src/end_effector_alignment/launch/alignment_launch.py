#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get package directory
    pkg_dir = get_package_share_directory('end_effector_alignment')
    
    # Declare launch arguments
    aruco_topic_arg = DeclareLaunchArgument(
        'aruco_topic',
        default_value='/aruco/poses',
        description='Topic name for ArUco poses'
    )
    
    target_topic_arg = DeclareLaunchArgument(
        'target_topic', 
        default_value='/end_effector_target',
        description='Topic name for target end effector pose'
    )
    
    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='base_link', 
        description='Target reference frame for end effector'
    )
    
    offset_distance_arg = DeclareLaunchArgument(
        'offset_distance',
        default_value='0.1',
        description='Offset distance from panel in meters'
    )
    
    # Add the missing use_sim_time argument declaration
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'alignment_params.yaml'),
        description='Path to configuration file'
    )
    
    # Create the end effector alignment node
    alignment_node = Node(
        package='end_effector_alignment',
        executable='end_effector_aligner',
        name='end_effector_aligner',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'aruco_topic': LaunchConfiguration('aruco_topic'),
                'target_topic': LaunchConfiguration('target_topic'), 
                'target_frame': LaunchConfiguration('target_frame'),
                'offset_distance': LaunchConfiguration('offset_distance'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        output='both',
        emulate_tty=True
    )
    
    return LaunchDescription([
        aruco_topic_arg,
        target_topic_arg,
        target_frame_arg,
        offset_distance_arg,
        use_sim_time_arg,  # Add this to the LaunchDescription
        config_file_arg,
        alignment_node
    ])
