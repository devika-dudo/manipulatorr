#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from linkattacher_msgs.srv import AttachLink, DetachLink
import time

class PickAndPlaceController(Node):
    def __init__(self):
        super().__init__('pick_and_place_controller')
        
        # Publishers
        self.arm_command_pub = self.create_publisher(
            Float64MultiArray, 
            '/arm_xyzrpy_command', 
            10
        )
        
        # Action clients
        self.hand_action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/hand_controller/follow_joint_trajectory'
        )
        
        # Service clients
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')
        
        # Wait for services to be available
        self.get_logger().info('Waiting for services...')
        self.attach_client.wait_for_service(timeout_sec=10.0)
        self.detach_client.wait_for_service(timeout_sec=10.0)
        self.hand_action_client.wait_for_server(timeout_sec=10.0)
        
        self.get_logger().info('All services and actions are ready!')

    def move_arm_to_pose(self, x, y, z, roll, pitch, yaw):
        """Move arm to specified pose"""
        msg = Float64MultiArray()
        msg.data = [x, y, z, roll, pitch, yaw]
        
        self.get_logger().info(f'Moving arm to pose: [{x}, {y}, {z}, {roll}, {pitch}, {yaw}]')
        self.arm_command_pub.publish(msg)
        time.sleep(3.0)  # Wait for movement to complete

    def control_gripper(self, position, duration_sec=2):
        """Control gripper position (10.0 = open, 3.0 = closed)"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['joint_6']
        
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = duration_sec
        point.time_from_start.nanosec = 0
        
        goal_msg.trajectory.points = [point]
        
        action_name = "OPEN" if position > 5.0 else "CLOSE"
        self.get_logger().info(f'{action_name} gripper to position: {position}')
        
        future = self.hand_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            time.sleep(duration_sec + 0.5)  # Wait for action to complete
        else:
            self.get_logger().error('Gripper action was rejected!')

    def attach_object(self):
        """Attach object to gripper"""
        request = AttachLink.Request()
        request.model1_name = 'urdf_humble_test'
        request.link1_name = 'link_8'
        request.model2_name = 'bc'
        request.link2_name = 'link'
        
        self.get_logger().info('Attaching object to gripper...')
        future = self.attach_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            self.get_logger().info('Object attached successfully!')
            return True
        else:
            self.get_logger().error('Failed to attach object!')
            return False

    def detach_object(self):
        """Detach object from gripper"""
        request = DetachLink.Request()
        request.model1_name = 'urdf_humble_test'
        request.link1_name = 'link_8'
        request.model2_name = 'bc'
        request.link2_name = 'link'
        
        self.get_logger().info('Detaching object from gripper...')
        future = self.detach_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            self.get_logger().info('Object detached successfully!')
            return True
        else:
            self.get_logger().error('Failed to detach object!')
            return False

    def execute_pick_and_place(self):
        """Execute the complete pick and place sequence"""
        try:
            self.get_logger().info('Starting pick and place sequence...')
            
            # Step 1: Move to safe height above object
            self.get_logger().info('Step 1: Moving to safe height above object')
            self.move_arm_to_pose(-0.3, 0.3, 0.2, 0.026, 1.6, 1.767)
            
            # Step 2: Open gripper
            self.get_logger().info('Step 2: Opening gripper')
            self.control_gripper(10.0, 2)
            
            # Step 3: Move down to object
            self.get_logger().info('Step 3: Moving down to object')
            self.move_arm_to_pose(-0.3, 0.3, 0.03, 0.026, 1.6, 1.767)
            
            # Step 4: Close gripper
            self.get_logger().info('Step 4: Closing gripper')
            self.control_gripper(3.0, 2)
            
            # Step 5: Attach object (ensure arm is stationary)
            self.get_logger().info('Step 5: Waiting for arm to be stable...')
            time.sleep(1.0)  # Extra wait to ensure arm is completely stationary
            self.get_logger().info('Step 5: Attaching object')
            if not self.attach_object():
                return False
            
            # Step 6: Move up to safe height first (lift the object)
            self.get_logger().info('Step 6: Lifting object to safe height')
            self.move_arm_to_pose(-0.3, 0.3, 0.2, 0.026, 1.6, 1.767)
            
          
            
            # Step 8: Move to place position (reasonable height)
            self.get_logger().info('Step 7: Moving to place position')
            self.move_arm_to_pose(0.3, 0.3, 0.1, 0.026, 1.6, 1.767)
            
            # Step 9: Ensure arm is completely stationary before detaching (placing object)
            self.get_logger().info('Step 8: Waiting for arm to be stable before placing object...')
            time.sleep(3.0)  # Extra wait to ensure arm is completely stationary
            
            # Step 10: Detach object (place it down)
            self.get_logger().info('Step 9: Placing object (detaching from gripper)')
            if not self.detach_object():
                return False
            
            # Step 11: Open gripper
            self.get_logger().info('Step 10: Opening gripper')
            self.control_gripper(10.0, 2)
            
           
            
            self.get_logger().info('Pick and place sequence completed successfully!')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error during pick and place: {str(e)}')
            return False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = PickAndPlaceController()
        
        # Wait a moment for everything to initialize
        time.sleep(2.0)
        
        # Execute the pick and place sequence
        success = controller.execute_pick_and_place()
        
        if success:
            print("\n✅ Pick and place operation completed successfully!")
        else:
            print("\n❌ Pick and place operation failed!")
            
    except KeyboardInterrupt:
        print("\n🛑 Operation interrupted by user")
    except Exception as e:
        print(f"\n💥 Unexpected error: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
