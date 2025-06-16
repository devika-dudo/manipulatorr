#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([
        # Launch perception vision node
        Node(
            package='perception_vision',
            executable='yolov8_obb_node',
            name='yolov8_obb_node',
            output='screen'
        ),
        
        # Launch pose estimator node with a small delay
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='cylinder_position_estimator',
                    executable='pose_estimator_node',
                    name='pose_estimator_node',
                    output='screen'
                )
            ]
        ),
        
        # Launch autonomous pick and place node with a delay
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='urdf_humble_test',
                    executable='autonomous_pick_and_place.py',
                    name='autonomous_pick_place',
                    output='screen',
                    parameters=[{
                        'pick_height': 0.15,
                        'place_position_x': 0.5,
                        'place_position_y': -0.3,
                        'place_position_z': 0.4,
                        'orientation_x': 0.152,
                        'orientation_y': 1.6,
                        'orientation_z': 1.809
                    }]
                )
            ]
        )
    ])
