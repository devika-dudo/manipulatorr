from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    from ament_index_python.packages import get_package_share_directory
    import os

    urdf_file = os.path.join(
        get_package_share_directory('urdf_humble_test'),
        'urdf',
        'model.urdf'
    )

    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        )
    ])

