from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit,OnProcessStart
from launch_ros.actions import Node
from launch.substituitions import Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    pkg_path = get_package_share_directory('urdf_humble_test')
    urdf_file = os.path.join(pkg_path, 'urdf', 'model.urdf')
    controller_config = os.path.join(pkg_path, 'config', 'my_controllers.yaml')
    # Read URDF
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()
    
    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )
    
   robot_description=Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    # 3. Controller Spawners (minimal version)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description':robot_description},controller_config],
        output='screen',
    )
    delayed_controller_manager=TimerAction(period=3.0,action=[controller_manager])
    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='ros2_control_node',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    
    arm_group_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_group_controller'],
        output='screen',
    )
    
    hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hand_controller'],
        output='screen',
    )
    
    delayed_arm_and_hand_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[arm_group_controller_spawner,hand_controller_spawner]
        )
    )
     
    delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner]
        )
    )
   
    
    return LaunchDescription([
       
        # Robot Description Publisher
        robot_state_publisher_node,
        delayed_controller_manager,
        # Load Controllers After Spawning
        delayed_joint_state_broadcaster_spawner,
        delayed_arm_and_hand_spawner
    ])
