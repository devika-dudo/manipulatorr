from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    pkg_path = get_package_share_directory('urdf_humble_test')
    urdf_file = os.path.join(pkg_path, 'urdf', 'model.urdf')
    controller_yaml = os.path.join(pkg_path, 'config', 'controller_manager.yaml')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'robot_config.rviz')  # Optional custom config

    # Read URDF
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content},
                    {'use_sim_time': True}]
    )

    # 2. Spawn Entity
    spawn_entity = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'urdf_humble_test',
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '0.2'
        ],
        output='screen'
    )

    # 3. Controller Loaders
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_group_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_group_controller'],
        output='screen'
    )

    load_hand_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'hand_controller'],
        output='screen'
    )

    # 4. Event Handlers
    load_joint_state_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster]
        )
    )

    load_arm_and_hand_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_arm_group_controller, load_hand_controller]
        )
    )

    # 5. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ])
        ),

        # Publish Robot Description
        robot_state_publisher_node,

        # Spawn in Gazebo
        spawn_entity,

        # Load Controllers in Order
        load_joint_state_after_spawn,
        load_arm_and_hand_after_jsb,

        # Launch RViz2
        rviz_node
    ])

