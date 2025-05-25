#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
from geometry_msgs.msg import Pose, Point, Quaternion
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Initialize MoveIt2 - REPLACE THESE WITH YOUR ROBOT'S VALUES
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                "joint_1",
                "joint_2", 
                "joint_3",
                "joint_4",
                "joint_5",
            ],
            base_link_name="base_link",
            end_effector_name="link_5", 
            group_name="arm_group"
        )
        self.moveit2.planner_id = "RRTConnect"
        
    def move_to_home(self):
        """Move robot to home position"""
        self.get_logger().info("Moving to home position...")
        home_joints = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.moveit2.move_to_configuration(home_joints)
        self.moveit2.wait_until_executed()
        self.get_logger().info("Reached home position")
    def move_to_pose_example(self):
        """Move to a specific Cartesian pose"""
        self.get_logger().info("Moving to target pose...")
        
        target_pose = Pose()
        target_pose.position = Point(x=0.140, y=-0.424, z=0.403)
        target_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        self.moveit2.move_to_pose(target_pose)
        self.moveit2.wait_until_executed()
        self.get_logger().info("Reached target pose")
        
    def move_relative(self):
        """Move relative to current position"""
        self.get_logger().info("Moving relatively...")
        
        # Move 10cm in X direction
        self.moveit2.move_to_pose(
            position=[0.1, 0.0, 0.0],  # relative movement
            quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            cartesian=True
        )
        self.moveit2.wait_until_executed()
        self.get_logger().info("Completed relative movement")

def main():
    rclpy.init()
    
    # Create robot controller
    robot_controller = RobotController()
    
    # Wait for MoveIt2 to initialize
    robot_controller.get_logger().info("Waiting for MoveIt2 to initialize...")
    time.sleep(3.0)
    
    try:
        # Execute movement sequence
        robot_controller.move_to_home()
        time.sleep(2.0)
        
        robot_controller.move_to_pose_example()
        time.sleep(2.0)
        
        robot_controller.move_relative()
        time.sleep(2.0)
        
        robot_controller.move_to_home()
        
    except Exception as e:
        robot_controller.get_logger().error(f"Error during execution: {e}")
    
    finally:
        robot_controller.get_logger().info("Shutting down...")
        rclpy.shutdown()

if __name__ == '__main__':
    main()
