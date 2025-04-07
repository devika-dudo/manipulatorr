import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument

def generate_launch_description():
    pkg_install_path = get_package_share_directory('urdf_humble_test')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path =  os.environ['GAZEBO_MODEL_PATH'] + ':' + pkg_install_path
    else:
        model_path =  pkg_install_path

    # Define launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    package_name = "urdf_humble_test"
    urdf_file = "model.urdf"

    # Get paths
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        "urdf",
        urdf_file
    )

    # Read URDF file
    with open(urdf_path, "r") as infp:
        robot_desc = infp.read()

    # Include Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )


    # Spawn Entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', urdf_path, '-entity', 'urdf_humble_test'],
        output='screen',
    )

    # Load Controllers
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_group_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_group_controller'],
        output='screen'
    )

    load_hand_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'hand_controller'],
        output='screen'
    )

    return LaunchDescription([
        # Start Gazebo
        gazebo,
        
        # Spawn the robot
        spawn_entity,
        # Publish Robot State
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_desc}]
        ),

        # Load Controllers after spawn
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster]
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_arm_group_controller, load_hand_controller]
            )
        )
    ])

