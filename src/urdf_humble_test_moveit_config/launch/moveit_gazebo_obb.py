import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    robot_description_path = os.path.join(
        get_package_share_directory('urdf_humble_test'))
    
    arm_robot_sim_path = os.path.join(
        get_package_share_directory('urdf_humble_test_moveit_config'))
    
    urdf_file = os.path.join(robot_description_path,
                              'urdf',
                              'model.urdf')
   
    moveit_config = (
        MoveItConfigsBuilder(robot_name="urdf_humble_test")
        .robot_description_semantic(file_path="config/urdf_humble_test.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(arm_robot_sim_path + "/config/controller_setting.yaml")
        .to_moveit_configs()
    )
    #.joint_limits(file_path="config/joint_limits.yaml")
    #.robot_description_kinematics(file_path="config/kinematics.yaml")

    moveit_py_node = Node(
        name="moveit_py",
        package="urdf_humble_test_moveit_config",
        executable="arm_control_from_UI",
        output="both",
        parameters=[moveit_config.to_dict(),
                    {"use_sim_time": True},
                    ],
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("urdf_humble_test_moveit_config"),
        "config",
        "moveit.rviz",
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
            {"use_sim_time": True},
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
        parameters=[{"use_sim_time": True},],
    )

    return LaunchDescription(
        [
            moveit_py_node,
            rviz_node,
            static_tf,
        ]
    )
