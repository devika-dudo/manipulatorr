import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("urdf_humble_test")
        .robot_description(file_path="config/urdf_humble_test.urdf.xacro")
        .to_moveit_configs()
    )

    # Load servo config
    servo_yaml = load_yaml("my_moveit_servo", "config/urdf_humble_test_simulated_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # RViz node
    rviz_config_file = os.path.join(
        get_package_share_directory("my_moveit_servo"),
        "config",
        "demo_rviz_config.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # REMOVED: Joint State Publisher - Gazebo will provide joint states
    # Keeping this commented out since Gazebo is handling joint state publishing
    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    #     name="joint_state_publisher",
    #     parameters=[{"use_gui": False}],
    # )

    # REMOVED: ros2_control_node - Let Gazebo handle hardware interface
    # This was causing the GazeboSystem plugin error

    # Composable node container
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[
                    moveit_config.robot_description,
                    {"use_sim_time": True}  # Use simulation time from Gazebo
                ],
            ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "/base_link", "frame_id": "/world"}],
            ),
        ],
        output="screen",
    )

    # Standalone servo node with improved timing
    servo_node = Node(
        package="my_moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},  # Use simulation time from Gazebo
        ],
        output="screen",
    )

    return LaunchDescription([
        rviz_node,
        # joint_state_publisher_node,  # REMOVED - Let Gazebo handle joint states
        servo_node,
        container,
    ])
