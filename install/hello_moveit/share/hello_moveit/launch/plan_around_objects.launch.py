from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hello_moveit',
            executable='plan_around_objects',
            name='plan_around_objects',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
