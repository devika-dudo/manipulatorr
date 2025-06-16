#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np

class DepthReader(Node):
    def __init__(self):
        super().__init__('depth_reader')
        self.bridge = CvBridge()
        self.latest_depth_image = None
        
        # Subscribers
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.create_subscription(Float32MultiArray, '/yolo/centre', self.detection_callback, 10)
        
        self.get_logger().info('DepthReader node started')

    def depth_callback(self, msg):
        """Store the latest depth image"""
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')

    def detection_callback(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warn('Invalid detection data')
            return
        
        if self.latest_depth_image is None:
            self.get_logger().warn('No depth image available')
            return
            
        cx, cy = int(msg.data[0]), int(msg.data[1])
        
        # Check bounds
        h, w = self.latest_depth_image.shape
        if cx < 0 or cx >= w or cy < 0 or cy >= h:
            self.get_logger().warn(f'Detection out of bounds: ({cx}, {cy})')
            return
        
        # Get depth at detection center
        depth_value = self.latest_depth_image[cy, cx]
        
        # Check if valid
        if np.isfinite(depth_value) and depth_value > 0:
            self.get_logger().info(f'Object at ({cx}, {cy}) - Depth: {depth_value:.3f}m')
        else:
            self.get_logger().warn(f'Invalid depth at ({cx}, {cy}): {depth_value}')

def main(args=None):
    rclpy.init(args=args)
    node = DepthReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
