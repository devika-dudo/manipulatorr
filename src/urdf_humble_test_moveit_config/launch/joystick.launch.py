from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='urdf_humble_test_moveit_config',
            executable='joystick_cartesian_teleop_node',
            name='joystick_cartesian_teleop_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
    ])
