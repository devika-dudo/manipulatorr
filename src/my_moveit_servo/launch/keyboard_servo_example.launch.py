import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    moveit_servo_dir = get_package_share_directory('my_moveit_servo')
    
    # Include the existing servo launch file
    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(moveit_servo_dir, 'launch', 'servo_example.launch.py')
        ])
    )
    
    # Print instruction to run keyboard node separately
    print("="*60)
    print("IMPORTANT: Run the keyboard node in a separate terminal:")
    print("ros2 run my_moveit_servo keyboard_servo_node")
    print("="*60)
    
    return LaunchDescription([
        servo_launch,
        # keyboard_servo_node removed - run separately
    ])
