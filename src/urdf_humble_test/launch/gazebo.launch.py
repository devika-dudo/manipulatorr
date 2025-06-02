from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    pkg_path = get_package_share_directory('urdf_humble_test')
    urdf_file = os.path.join(pkg_path, 'urdf', 'model.urdf')
    
    # Read URDF
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()
    
    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}, {'use_sim_time': True}]
    )
    
    # 2. Spawn Entity
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', 'urdf_humble_test',
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '0.2'
        ],
        output='screen'
    )
    
    # 3. Controller Spawners (minimal version)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    
    arm_group_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_group_controller'],
        output='screen',
    )
    
    
    # 4. Event Handlers for Sequential Loading
    load_joint_state_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )
    
    load_arm_and_hand_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_group_controller_spawner]
        )
    )
    
    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ])
        ),
        # Robot Description Publisher
        robot_state_publisher_node,
        # Spawn the Robot
        spawn_entity_node,
        # Load Controllers After Spawning
        load_joint_state_after_spawn,
        load_arm_and_hand_after_jsb
    ])
