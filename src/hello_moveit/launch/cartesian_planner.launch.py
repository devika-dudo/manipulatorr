from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # Declare launch arguments
    planning_group_arg = DeclareLaunchArgument(
        'planning_group',
        default_value='arm_group',
        description='MoveIt planning group name'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Get launch configuration
    planning_group = LaunchConfiguration('planning_group')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Cartesian planner node
    cartesian_planner_node = Node(
        package='hello_moveit',
        executable='hello_moveit_cartesian',
        name='cartesian_planner',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'planning_group': planning_group}
        ],
        emulate_tty=True,  # For colored output
    )
    
    return LaunchDescription([
        planning_group_arg,
        use_sim_time_arg,
        cartesian_planner_node
    ])
