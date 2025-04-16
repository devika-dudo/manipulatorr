from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hello_moveit',
            executable='hello_moveit',
            name='hello_moveit',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
