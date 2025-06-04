#!/usr/bin/env python3
"""
Fake Joint State Publisher Node
Ensures continuous publishing of fake_joint states to prevent erratic RViz behavior
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time

class FakeJointPublisher(Node):
    def __init__(self):
        super().__init__('fake_joint_publisher')
        
        # Create publisher for joint states
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        # Set publishing rate (50Hz)
        self.timer = self.create_timer(0.02, self.publish_fake_joint)
        
        # Initialize joint position (you can modify this if needed)
        self.fake_joint_position = 0.0
        
        self.get_logger().info('Fake Joint Publisher started - publishing at 50Hz')
        
    def publish_fake_joint(self):
        """Publish fake_joint state continuously"""
        msg = JointState()
        
        # Set timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        # Joint information
        msg.name = ['fake_joint']
        msg.position = [self.fake_joint_position]
        msg.velocity = [0.0]
        msg.effort = [0.0]
        
        # Publish the message
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = FakeJointPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
