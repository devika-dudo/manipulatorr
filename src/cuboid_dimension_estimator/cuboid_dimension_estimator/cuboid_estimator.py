#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float32MultiArray
from vision_msgs.msg import Detection2DArray, Detection2D
import cv2
from cv_bridge import CvBridge
import numpy as np
import message_filters
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import threading
import time


class CuboidDimensionEstimator(Node):
    def __init__(self):
        super().__init__('cuboid_dimension_estimator')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame = None
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers
        self.dimension_pub = self.create_publisher(Point, '/cuboid/dimensions', 10)
        self.oversized_pub = self.create_publisher(Bool, '/cuboid/oversized', 10)
        self.debug_image_pub = self.create_publisher(Image, '/cuboid/debug_image', 10)
        
        # Thread-safe storage for latest messages
        self.lock = threading.Lock()
        self.latest_image = None
        self.latest_depth = None
        self.latest_detections = None
        self.latest_image_time = None
        self.latest_depth_time = None
        self.latest_detections_time = None
        
        # Debug counters
        self.image_count = 0
        self.depth_count = 0
        self.detection_count = 0
        self.processed_count = 0
        
        # Camera info subscriber
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10
        )
        
        # Individual subscribers - no synchronization
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10
        )
        
        # Try to subscribe to detections with error handling
        try:
            self.detection_sub = self.create_subscription(
                Float32MultiArray, '/yolo/centre', self.detection_callback, 10
            )
            self.get_logger().info('Successfully subscribed to /yolo/centre')
        except Exception as e:
            self.get_logger().error(f'Failed to subscribe to /yolo/centre: {e}')
            self.detection_sub = None
        
        # Parameters
        self.size_threshold = 0.15  # 15cm in meters
        self.min_depth = 0.01  # Minimum valid depth (1cm)
        self.max_depth = 5.0   # Maximum valid depth (5m)
        self.max_time_diff = 2.0  # Maximum time difference between messages (seconds)
        
        # Processing timer - try to process every 500ms
        self.process_timer = self.create_timer(0.5, self.try_process_messages)
        
        # Create timer for debug info
        self.debug_timer = self.create_timer(5.0, self.print_debug_info)
        
        self.get_logger().info('Cuboid Dimension Estimator initialized with async processing')

    def image_callback(self, msg):
        with self.lock:
            self.latest_image = msg
            self.latest_image_time = self.get_clock().now()
            self.image_count += 1

    def depth_callback(self, msg):
        with self.lock:
            self.latest_depth = msg
            self.latest_depth_time = self.get_clock().now()
            self.depth_count += 1

    def detection_callback(self, msg):
        with self.lock:
            self.latest_detections = msg
            self.latest_detections_time = self.get_clock().now()
            self.detection_count += 1
            
        # Parse the detection data: [center_x, center_y, width, height]
        if len(msg.data) >= 4:
            self.get_logger().info(f'Received detection: center=({msg.data[0]:.1f}, {msg.data[1]:.1f}), size=({msg.data[2]:.1f}, {msg.data[3]:.1f})')
        else:
            self.get_logger().warn(f'Unexpected detection data length: {len(msg.data)}')

    def try_process_messages(self):
        """Try to process available messages"""
        with self.lock:
            # Check if we have all required messages
            if (self.latest_image is None or 
                self.latest_depth is None or 
                self.latest_detections is None):
                return
            
            # Check if messages are reasonably recent
            current_time = self.get_clock().now()
            
            def time_diff_seconds(msg_time):
                if msg_time is None:
                    return float('inf')
                return abs((current_time - msg_time).nanoseconds / 1e9)
            
            image_age = time_diff_seconds(self.latest_image_time)
            depth_age = time_diff_seconds(self.latest_depth_time)
            detection_age = time_diff_seconds(self.latest_detections_time)
            
            # Check if any message is too old
            if (image_age > self.max_time_diff or 
                depth_age > self.max_time_diff or 
                detection_age > self.max_time_diff):
                
                ages_str = f"Image: {image_age:.1f}s, Depth: {depth_age:.1f}s, Detections: {detection_age:.1f}s"
                self.get_logger().debug(f'Messages too old: {ages_str}')
                return
            
            # Copy messages for processing (release lock quickly)
            image_msg = self.latest_image
            depth_msg = self.latest_depth
            detection_msg = self.latest_detections
        
        # Process outside of lock
        self.process_messages(image_msg, depth_msg, detection_msg)

    def process_messages(self, image_msg, depth_msg, detection_msg):
        """Process the messages"""
        self.processed_count += 1
        self.get_logger().info(f'Processing messages! Count: {self.processed_count}')
        
        if self.camera_matrix is None:
            self.get_logger().warn('Camera parameters not yet received')
            return
        
        # Check if we have any detections
        if len(detection_msg.data) < 4:
            self.get_logger().info('No valid detections in this frame')
            return
        
        try:
            # Convert ROS images to OpenCV
            color_image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            
            # Handle different depth image formats
            if depth_msg.encoding == '16UC1':
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
                depth_image = depth_image.astype(np.float32) / 1000.0  # Convert to meters
            elif depth_msg.encoding == '32FC1':
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
            else:
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
                if depth_image.dtype == np.uint16:
                    depth_image = depth_image.astype(np.float32) / 1000.0
            
            self.get_logger().info(f'Processing detection with data: {detection_msg.data}')
            self.get_logger().info(f'Image shape: {color_image.shape}, Depth shape: {depth_image.shape}')
            self.get_logger().info(f'Depth range: {np.nanmin(depth_image):.3f} - {np.nanmax(depth_image):.3f}m')
            
            # Process detection (assuming single detection per message)
            debug_image = color_image.copy()
            
            # Parse detection data: [center_x, center_y, width, height]
            if len(detection_msg.data) >= 4:
                center_x = detection_msg.data[0]
                center_y = detection_msg.data[1]
                width = detection_msg.data[2]
                height = detection_msg.data[3]
                
                # Create a simple detection object for processing
                detection_data = {
                    'center_x': center_x,
                    'center_y': center_y,
                    'width': width,
                    'height': height
                }
                
                self.get_logger().info(f'Processing detection: center=({center_x:.1f},{center_y:.1f}), size=({width:.1f},{height:.1f})')
                
                dimensions = self.estimate_cuboid_dimensions(
                    detection_data, depth_image, debug_image
                )
                
                if dimensions is not None:
                    # Publish dimensions - FIXED: Convert to native Python floats
                    dim_msg = Point()
                    dim_msg.x = float(dimensions[0])  # width
                    dim_msg.y = float(dimensions[1])  # height  
                    dim_msg.z = float(dimensions[2])  # depth
                    self.dimension_pub.publish(dim_msg)
                    
                    # Check if any dimension exceeds threshold
                    oversized = any(dim > self.size_threshold for dim in dimensions)
                    oversized_msg = Bool()
                    oversized_msg.data = oversized
                    self.oversized_pub.publish(oversized_msg)
                    
                    self.get_logger().info(
                        f'Cuboid dimensions: W={dimensions[0]:.3f}m, '
                        f'H={dimensions[1]:.3f}m, D={dimensions[2]:.3f}m, '
                        f'Oversized: {oversized}'
                    )
                else:
                    self.get_logger().warn('Could not estimate dimensions for detection')
            else:
                self.get_logger().warn(f'Invalid detection data: {detection_msg.data}')
            
            # Publish debug image
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
                debug_msg.header = image_msg.header
                self.debug_image_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f'Error publishing debug image: {e}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing images: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def print_debug_info(self):
        """Print debug information about message counts"""
        self.get_logger().info(
            f'Message counts - Image: {self.image_count}, '
            f'Depth: {self.depth_count}, '
            f'Detections: {self.detection_count}, '
            f'Processed: {self.processed_count}'
        )
        
        # Check if topics are publishing
        if self.image_count == 0:
            self.get_logger().warn('No RGB images received! Check /camera/image_raw topic')
        if self.depth_count == 0:
            self.get_logger().warn('No depth images received! Check /camera/depth/image_raw topic')
        if self.detection_count == 0:
            self.get_logger().warn('No detections received! Check /yolo/centre topic')
        
        # Reset counters for next interval
        self.image_count = 0
        self.depth_count = 0
        self.detection_count = 0

    def camera_info_callback(self, msg):
        """Store camera calibration parameters"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.camera_frame = msg.header.frame_id
        
        # Extract focal lengths and principal point
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]
        
        self.get_logger().info(f'Camera parameters updated: fx={self.fx:.2f}, fy={self.fy:.2f}')

    def estimate_cuboid_dimensions(self, detection_data, depth_image, debug_image):
        """Estimate 3D dimensions of a cuboid using back projection"""
        try:
            # Extract bounding box from detection data
            center_x = int(detection_data['center_x'])
            center_y = int(detection_data['center_y'])
            width = int(detection_data['width'])
            height = int(detection_data['height'])
            
            self.get_logger().info(f'BBox: center=({center_x},{center_y}), size=({width},{height})')
            
            # Validate bounding box
            if (center_x < 0 or center_y < 0 or 
                center_x >= depth_image.shape[1] or center_y >= depth_image.shape[0] or
                width <= 0 or height <= 0):
                self.get_logger().warn('Invalid bounding box coordinates')
                return None
            
            # Define bounding box corners with bounds checking
            x_min = max(0, center_x - width // 2)
            x_max = min(depth_image.shape[1], center_x + width // 2)
            y_min = max(0, center_y - height // 2)
            y_max = min(depth_image.shape[0], center_y + height // 2)
            
            if x_min >= x_max or y_min >= y_max:
                self.get_logger().warn('Invalid bounding box after bounds checking')
                return None
            
            # Extract depth region
            depth_roi = depth_image[y_min:y_max, x_min:x_max]
            
            # Filter valid depths
            valid_mask = (
                (depth_roi > self.min_depth) & 
                (depth_roi < self.max_depth) & 
                (~np.isnan(depth_roi)) &
                (~np.isinf(depth_roi)) &
                (depth_roi > 0)
            )
            
            valid_depths = depth_roi[valid_mask]
            
            self.get_logger().info(f'Valid depth points: {len(valid_depths)} / {depth_roi.size}')
            
            if len(valid_depths) < 10:  # Need sufficient depth points
                self.get_logger().warn('Insufficient valid depth points')
                return None
            
            # Use median depth for robustness
            median_depth = np.median(valid_depths)
            self.get_logger().info(f'Median depth: {median_depth:.3f}m')
            
            # Get corner points in image coordinates
            corners_2d = np.array([
                [x_min, y_min],
                [x_max, y_min],
                [x_max, y_max],
                [x_min, y_max]
            ], dtype=np.float32)
            
            # Back-project to 3D using the median depth
            corners_3d = []
            for corner in corners_2d:
                x_3d = (corner[0] - self.cx) * median_depth / self.fx
                y_3d = (corner[1] - self.cy) * median_depth / self.fy
                z_3d = median_depth
                corners_3d.append([x_3d, y_3d, z_3d])
            
            corners_3d = np.array(corners_3d)
            
            # Calculate dimensions
            # Width: distance between left and right edges
            width_3d = abs(corners_3d[1][0] - corners_3d[0][0])
            
            # Height: distance between top and bottom edges
            height_3d = abs(corners_3d[2][1] - corners_3d[1][1])
            
            # Depth estimation using depth variation
            depth_variation = np.std(valid_depths)
            depth_3d = max(depth_variation * 2, 0.02)  # Minimum 2cm depth
            
            # Alternative depth estimation using edge analysis
            depth_3d_alt = self.estimate_depth_from_edges(
                depth_roi, median_depth
            )
            if depth_3d_alt > 0:
                depth_3d = max(depth_3d, depth_3d_alt)
            
            # FIXED: Ensure all dimensions are native Python floats
            dimensions = [float(width_3d), float(height_3d), float(depth_3d)]
            
            # Draw debug information
            self.draw_debug_info(
                debug_image, corners_2d, dimensions, median_depth,
                x_min, y_min, x_max - x_min, y_max - y_min
            )
            
            return dimensions
            
        except Exception as e:
            self.get_logger().error(f'Error estimating dimensions: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return None

    def estimate_depth_from_edges(self, depth_roi, median_depth):
        """Estimate depth using edge analysis in the depth image"""
        try:
            # Apply Gaussian blur to reduce noise
            blurred = cv2.GaussianBlur(depth_roi.astype(np.float32), (5, 5), 0)
            
            # Calculate gradients
            grad_x = cv2.Sobel(blurred, cv2.CV_64F, 1, 0, ksize=3)
            grad_y = cv2.Sobel(blurred, cv2.CV_64F, 0, 1, ksize=3)
            
            # Calculate gradient magnitude
            gradient_magnitude = np.sqrt(grad_x**2 + grad_y**2)
            
            # Find significant depth changes
            threshold = np.std(gradient_magnitude) * 2
            significant_edges = gradient_magnitude > threshold
            
            if np.sum(significant_edges) > 0:
                edge_depths = depth_roi[significant_edges]
                valid_edge_depths = edge_depths[(edge_depths > 0) & (~np.isnan(edge_depths))]
                if len(valid_edge_depths) > 0:
                    depth_range = np.max(valid_edge_depths) - np.min(valid_edge_depths)
                    return float(depth_range)  # FIXED: Convert to Python float
            
            return 0.0  # FIXED: Return Python float
            
        except Exception as e:
            self.get_logger().error(f'Error in edge depth estimation: {e}')
            return 0.0  # FIXED: Return Python float

    def draw_debug_info(self, image, corners, dimensions, depth, x, y, w, h):
        """Draw debug information on the image"""
        try:
            # Draw bounding box
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Draw corner points
            for corner in corners.astype(int):
                cv2.circle(image, tuple(corner), 3, (255, 0, 0), -1)
            
            # Draw dimension text
            text_lines = [
                f'W: {dimensions[0]:.3f}m',
                f'H: {dimensions[1]:.3f}m', 
                f'D: {dimensions[2]:.3f}m',
                f'Depth: {depth:.3f}m'
            ]
            
            for i, line in enumerate(text_lines):
                cv2.putText(
                    image, line, (x, y - 10 - i * 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2
                )
            
            # Highlight if oversized
            if any(dim > self.size_threshold for dim in dimensions):
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 3)
                cv2.putText(
                    image, 'OVERSIZED', (x, y + h + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
                )
        except Exception as e:
            self.get_logger().error(f'Error drawing debug info: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = CuboidDimensionEstimator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
