#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from linkattacher_msgs.srv import AttachLink, DetachLink
import time
import subprocess
import signal
import os

class AutonomousPickPlace(Node):
    def __init__(self):
        super().__init__('autonomous_pick_and_place')
        
        # Publishers
        self.arm_command_pub = self.create_publisher(
            Float64MultiArray, 
            '/arm_xyzrpy_command', 
            10
        )
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PointStamped,
            '/cylinder/pose_base',
            self.pose_callback,
            10
        )
        
        # Action clients
        self.gripper_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/hand_controller/follow_joint_trajectory'
        )
        
        # Service clients
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        
        # State variables
        self.cylinder_pose = None
        self.state = "WAITING_FOR_DETECTION"
        self.pick_height = 0.15  # Height above the cylinder for approach
        self.place_position = [0.5, -0.3, 0.4]  # Target drop location
        self.orientation = [0.152, 1.6, 1.809]  # Fixed orientation
        self.first_pose_received = False  # Flag to track first pose reception
        self.state_timer = 0.0  # Initialize state timer
        
        self.get_logger().info("Autonomous Pick and Place Node Started")
        self.get_logger().info("Waiting for cylinder detection...")
        
        # Timer for main state machine
        self.timer = self.create_timer(0.1, self.state_machine)
        
    def pose_callback(self, msg):
        """Callback for cylinder pose detection"""
        self.cylinder_pose = msg.point
        if self.state == "WAITING_FOR_DETECTION" and not self.first_pose_received:
            self.get_logger().info(f"First cylinder pose received at: x={self.cylinder_pose.x:.3f}, y={self.cylinder_pose.y:.3f}, z={self.cylinder_pose.z:.3f}")
            self.first_pose_received = True
            
            # Stop both perception nodes since we have the required pose
            self.get_logger().info("Stopping perception nodes - pose acquired!")
            self.stop_pose_estimator_node()
            self.stop_object_detection_node()
            
            # Immediately transition to move to approach
            self.state = "MOVE_TO_APPROACH"
    
    def stop_pose_estimator_node(self):
        """Stop the pose estimator node using system command"""
        try:
            # First try graceful shutdown with SIGTERM
            result = subprocess.run(['pkill', '-TERM', '-f', 'pose_estimator_node'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info("Gracefully stopping pose_estimator_node")
                time.sleep(1)  # Give it time to shutdown gracefully
            
            # Then force kill if still running
            result = subprocess.run(['pkill', '-KILL', '-f', 'pose_estimator_node'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info("Successfully stopped pose_estimator_node")
            else:
                self.get_logger().warn("pose_estimator_node may not have been running or already stopped")
        except Exception as e:
            self.get_logger().error(f"Failed to stop pose_estimator_node: {e}")

    def stop_object_detection_node(self):
        """Stop the object detection node using system command"""
        try:
            # First try graceful shutdown with SIGTERM
            result = subprocess.run(['pkill', '-TERM', '-f', 'yolov8_obb_node'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info("Gracefully stopping yolov8_obb_node")
                time.sleep(1)  # Give it time to shutdown gracefully
            
            # Then force kill if still running
            result = subprocess.run(['pkill', '-KILL', '-f', 'yolov8_obb_node'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info("Successfully stopped yolov8_obb_node")
            else:
                self.get_logger().warn("yolov8_obb_node may not have been running or already stopped")
        except Exception as e:
            self.get_logger().error(f"Failed to stop yolov8_obb_node: {e}")
    
    def move_arm(self, x, y, z, rx=None, ry=None, rz=None):
        """Send arm movement command"""
        if rx is None:
            rx, ry, rz = self.orientation
        
        msg = Float64MultiArray()
        msg.data = [float(x), float(y), float(z), float(rx), float(ry), float(rz)]
        self.arm_command_pub.publish(msg)
        self.get_logger().info(f"Moving arm to: [{x:.3f}, {y:.3f}, {z:.3f}, {rx:.3f}, {ry:.3f}, {rz:.3f}]")
    
    def control_gripper(self, position, duration=2.0):
        """Control gripper - position 10.0 = open, 0.0 = closed"""
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper action server not available")
            return False
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint_6']
        
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        goal.trajectory.points.append(point)
        
        future = self.gripper_client.send_goal_async(goal)
        self.get_logger().info(f"{'Opening' if position > 0.5 else 'Closing'} gripper")
        return True
    
    def attach_object(self):
        """Attach cylinder to gripper"""
        if not self.attach_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Attach service not available")
            return False
        
        request = AttachLink.Request()
        request.model1_name = 'urdf_humble_test'
        request.link1_name = 'link_8'
        request.model2_name = 'cylinder'
        request.link2_name = 'link'
        
        future = self.attach_client.call_async(request)
        self.get_logger().info("Attaching cylinder to gripper")
        return True
    
    def detach_object(self):
        """Detach cylinder from gripper"""
        if not self.detach_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Detach service not available")
            return False
        
        request = DetachLink.Request()
        request.model1_name = 'urdf_humble_test'
        request.link1_name = 'link_8'
        request.model2_name = 'cylinder'
        request.link2_name = 'link'
        
        future = self.detach_client.call_async(request)
        self.get_logger().info("Detaching cylinder from gripper")
        return True
    
    def state_machine(self):
        """Main state machine for pick and place operation"""
        
        if self.state == "WAITING_FOR_DETECTION":
            # Wait for cylinder detection
            pass
            
        elif self.state == "MOVE_TO_APPROACH":
            if self.cylinder_pose is not None:
                # Move to approach position (above the cylinder)
                approach_x = self.cylinder_pose.x
                approach_y = self.cylinder_pose.y
                approach_z = self.cylinder_pose.z + self.pick_height
                
                self.move_arm(approach_x, approach_y, approach_z)
                self.state = "OPEN_GRIPPER"
                self.state_timer = time.time()
                self.get_logger().info(f"Moving to approach position: ({approach_x:.3f}, {approach_y:.3f}, {approach_z:.3f})")
        
        elif self.state == "OPEN_GRIPPER":
            # Wait for arm to reach approach position, then open gripper
            if time.time() - self.state_timer > 3.0:  # Wait 3 seconds for movement
                self.control_gripper(10.0)  # Open gripper
                self.state = "MOVE_TO_PICK"
                self.state_timer = time.time()
        
        elif self.state == "MOVE_TO_PICK":
            # Wait for gripper to open, then move down to pick position
            if time.time() - self.state_timer > 2.5:  # Wait for gripper to open
                pick_x = self.cylinder_pose.x
                pick_y = self.cylinder_pose.y
                pick_z = self.cylinder_pose.z + 0.03  # Slight offset above cylinder
                
                self.move_arm(pick_x, pick_y, pick_z)
                self.state = "CLOSE_GRIPPER"
                self.state_timer = time.time()
        
        elif self.state == "CLOSE_GRIPPER":
            # Wait for arm to reach pick position, then close gripper
            if time.time() - self.state_timer > 3.0:
                self.control_gripper(3.0)  # Close gripper
                self.state = "ATTACH_OBJECT"
                self.state_timer = time.time()
        
        elif self.state == "ATTACH_OBJECT":
            # Wait for gripper to close, then attach object
            if time.time() - self.state_timer > 2.5:
                self.attach_object()
                self.state = "LIFT_OBJECT"
                self.state_timer = time.time()
        
        elif self.state == "LIFT_OBJECT":
            # Lift the object up
            if time.time() - self.state_timer > 1.0:
                lift_x = self.cylinder_pose.x
                lift_y = self.cylinder_pose.y
                lift_z = self.cylinder_pose.z + self.pick_height
                
                self.move_arm(lift_x, lift_y, lift_z)
                self.state = "MOVE_TO_PLACE"
                self.state_timer = time.time()
        
        elif self.state == "MOVE_TO_PLACE":
            # Move to place location
            if time.time() - self.state_timer > 3.0:
                place_x, place_y, place_z = self.place_position
                self.move_arm(place_x, place_y, place_z)
                self.state = "OPEN_GRIPPER_FINAL"
                self.state_timer = time.time()
        
        
        elif self.state == "OPEN_GRIPPER_FINAL":
            # Open gripper to release object
            if time.time() - self.state_timer > 5.0:
                self.control_gripper(10.0)  # Open gripper
                self.state = "DETACH_OBJECT"
                self.state_timer = time.time()
                
        elif self.state == "DETACH_OBJECT":
            # Detach object from gripper
            if time.time() - self.state_timer > 2.0:
                self.detach_object()
                self.state = "COMPLETED"
                self.state_timer = time.time()
                
       
        
        elif self.state == "COMPLETED":
            if time.time() - self.state_timer > 3.0:
                self.get_logger().info("Pick and place operation completed successfully!")
                self.get_logger().info("System ready for shutdown or new task...")
                # Don't reset to waiting state since we only want to do this once
                self.state = "FINISHED"
        
        elif self.state == "FINISHED":
            # Stay in finished state
            pass

def main(args=None):
    rclpy.init(args=args)
    
    node = AutonomousPickPlace()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down autonomous pick and place node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
