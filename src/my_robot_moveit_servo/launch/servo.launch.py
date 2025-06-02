import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Get servo parameters file
    servo_params_file = PathJoinSubstitution([
        FindPackageShare('my_robot_moveit_servo'),
        'config',
        'servo_config.yaml'
    ])
    
    # Servo node - simplified since other nodes are already running
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        parameters=[
            servo_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        servo_node
    ])
