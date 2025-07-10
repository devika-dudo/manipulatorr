#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # ArUco dictionaries to try
        self.dictionaries = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
            'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
            'DICT_5X5_1000': cv2.aruco.DICT_5X5_1000,
            'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
            'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
            'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
            'DICT_6X6_1000': cv2.aruco.DICT_6X6_1000,
            'DICT_7X7_50': cv2.aruco.DICT_7X7_50,
            'DICT_7X7_100': cv2.aruco.DICT_7X7_100,
            'DICT_7X7_250': cv2.aruco.DICT_7X7_250,
            'DICT_7X7_1000': cv2.aruco.DICT_7X7_1000,
            'DICT_ARUCO_ORIGINAL': cv2.aruco.DICT_ARUCO_ORIGINAL,
        }
        
        # Detection parameters
        self.detector_params = cv2.aruco.DetectorParameters()
        self.setup_detection_parameters()
        
        # Camera parameters - will be loaded from camera_info
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False
        
        # Declare parameters
        self.declare_parameter('marker_size', 0.05)  # -1 means unknown/auto-detect
        self.declare_parameter('camera_frame', 'camera_link_optical')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('show_image', True)
        self.declare_parameter('estimate_pose', True)  # whether to estimate pose at all
        
        # Get parameters
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.show_image = self.get_parameter('show_image').get_parameter_value().bool_value
        self.estimate_pose = self.get_parameter('estimate_pose').get_parameter_value().bool_value
        
        # Marker size detection
        self.marker_size_unknown = self.marker_size <= 0
        if self.marker_size_unknown:
            self.get_logger().info('Marker size unknown - will detect markers but skip pose estimation')
            self.estimate_pose = False
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            'camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publishers
        self.detection_image_pub = self.create_publisher(
            Image,
            'aruco/detection_image',
            10
        )
        
        self.poses_pub = self.create_publisher(
            PoseArray,
            'aruco/poses',
            10
        )
        
        # Publisher for detection info (when pose estimation is disabled)
        from std_msgs.msg import String
        self.detection_info_pub = self.create_publisher(
            String,
            'aruco/detection_info',
            10
        )
        
        # Log initialization
        self.get_logger().info('ArUco Detector Node initialized')
        self.get_logger().info(f'Subscribing to: camera/image_raw, camera/camera_info')
        self.get_logger().info(f'Publishing to: aruco/detection_image, aruco/poses')
        self.get_logger().info(f'Camera frame: {self.camera_frame}')
        if self.marker_size_unknown:
            self.get_logger().info('Marker size: UNKNOWN - pose estimation disabled')
        else:
            self.get_logger().info(f'Marker size: {self.marker_size}m')
        self.get_logger().info('Waiting for camera_info...')
        
    def camera_info_callback(self, msg):
        """Process camera info to get camera parameters"""
        if not self.camera_info_received:
            # Extract camera matrix
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            
            # Extract distortion coefficients
            self.dist_coeffs = np.array(msg.d)
            
            # Update camera frame from camera_info if not set
            if self.camera_frame == 'camera_link_optical' and msg.header.frame_id:
                self.camera_frame = msg.header.frame_id
            
            self.camera_info_received = True
            
            self.get_logger().info('Camera parameters received:')
            self.get_logger().info(f'  Camera matrix: {self.camera_matrix.flatten()}')
            self.get_logger().info(f'  Distortion coeffs: {self.dist_coeffs}')
            self.get_logger().info(f'  Camera frame: {self.camera_frame}')
            self.get_logger().info('Ready to detect ArUco markers!')
        
    def setup_detection_parameters(self):
        """Setup ArUco detection parameters for better detection"""
        self.detector_params.adaptiveThreshWinSizeMin = 3
        self.detector_params.adaptiveThreshWinSizeMax = 23
        self.detector_params.adaptiveThreshWinSizeStep = 10
        self.detector_params.adaptiveThreshConstant = 7
        self.detector_params.minMarkerPerimeterRate = 0.03
        self.detector_params.maxMarkerPerimeterRate = 4.0
        self.detector_params.polygonalApproxAccuracyRate = 0.03
        self.detector_params.minCornerDistanceRate = 0.05
        self.detector_params.minDistanceToBorder = 3
        self.detector_params.minMarkerDistanceRate = 0.05
        
    def image_callback(self, msg):
        """Process incoming camera images"""
        # Check if we have camera parameters
        if not self.camera_info_received:
            self.get_logger().warn('No camera_info received yet, skipping detection', throttle_duration_sec=2.0)
            return
            
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect ArUco markers
            detections = self.detect_markers_all_dicts(cv_image)
            
            if detections:
                # Draw detections on image
                detection_image = self.draw_detections(cv_image, detections)
                
                # Get poses only if marker size is known
                poses = []
                if self.estimate_pose and not self.marker_size_unknown:
                    poses = self.get_marker_poses(detections, msg.header.stamp)
                    
                    # Publish poses
                    if poses:
                        self.publish_poses(poses, msg.header.stamp)
                        
                        # Publish TF transforms
                        if self.publish_tf:
                            self.publish_transforms(poses, msg.header.stamp)
                else:
                    # Just publish detection info without poses
                    self.publish_detection_info(detections, msg.header.stamp)
                
                # Publish detection image
                detection_msg = self.bridge.cv2_to_imgmsg(detection_image, "bgr8")
                detection_msg.header = msg.header
                self.detection_image_pub.publish(detection_msg)
                
                # Show image if enabled
                if self.show_image:
                    cv2.imshow('ArUco Detection', detection_image)
                    cv2.waitKey(1)
            else:
                # No detections, publish original image
                self.detection_image_pub.publish(msg)
                
                if self.show_image:
                    cv2.imshow('ArUco Detection', cv_image)
                    cv2.waitKey(1)
                    
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def detect_markers_all_dicts(self, frame):
        """Try to detect ArUco markers using all common dictionaries"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        all_detections = []
        
        for dict_name, dict_type in self.dictionaries.items():
            aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
            detector = cv2.aruco.ArucoDetector(aruco_dict, self.detector_params)
            
            corners, ids, rejected = detector.detectMarkers(gray)
            
            if ids is not None and len(ids) > 0:
                detection_info = {
                    'dictionary': dict_name,
                    'corners': corners,
                    'ids': ids.flatten(),
                    'rejected': rejected
                }
                all_detections.append(detection_info)
                
                # Log detection with pixel size estimation
                marker_ids = ', '.join([str(id) for id in ids.flatten()])
                
                # Estimate marker size in pixels for reference
                if len(corners) > 0:
                    pixel_sizes = []
                    for corner in corners:
                        # Calculate approximate marker size in pixels
                        corner_points = corner[0]
                        width = np.linalg.norm(corner_points[1] - corner_points[0])
                        height = np.linalg.norm(corner_points[3] - corner_points[0])
                        avg_size = (width + height) / 2
                        pixel_sizes.append(int(avg_size))
                    
                    pixel_size_str = ', '.join([f'{size}px' for size in pixel_sizes])
                    self.get_logger().info(f'Found markers [{marker_ids}] with {dict_name} (sizes: {pixel_size_str})')
                else:
                    self.get_logger().info(f'Found markers [{marker_ids}] with {dict_name}')
        
        return all_detections
    
    def draw_detections(self, frame, detections):
        """Draw detected markers on the frame"""
        output_frame = frame.copy()
        colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255)]
        
        for i, detection in enumerate(detections):
            color = colors[i % len(colors)]
            dict_name = detection['dictionary']
            corners = detection['corners']
            ids = detection['ids']
            
            # Draw markers
            cv2.aruco.drawDetectedMarkers(output_frame, corners, ids, color)
            
            # Add text with dictionary name, IDs, and pixel size
            for j, (corner, marker_id) in enumerate(zip(corners, ids)):
                # Get center of marker
                center = np.mean(corner[0], axis=0).astype(int)
                
                # Calculate marker size in pixels
                corner_points = corner[0]
                width = np.linalg.norm(corner_points[1] - corner_points[0])
                height = np.linalg.norm(corner_points[3] - corner_points[0])
                avg_size = int((width + height) / 2)
                
                # Draw text
                if self.marker_size_unknown:
                    text = f"{dict_name}: ID {marker_id} ({avg_size}px)"
                else:
                    text = f"{dict_name}: ID {marker_id}"
                
                cv2.putText(output_frame, text, 
                           (center[0] - 60, center[1] - 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        return output_frame
    
    def estimatePoseSingleMarkers(self, corners, marker_size, camera_matrix, dist_coeffs):
        """
        Drop-in replacement for the deprecated cv2.aruco.estimatePoseSingleMarkers
        This function works with OpenCV 4.7+ where estimatePoseSingleMarkers was removed
        """
        # Define the 3D points of a marker in its own coordinate system
        marker_points = np.array([
            [-marker_size/2, marker_size/2, 0],
            [marker_size/2, marker_size/2, 0], 
            [marker_size/2, -marker_size/2, 0],
            [-marker_size/2, -marker_size/2, 0]
        ], dtype=np.float32)
        
        rvecs = []
        tvecs = []
        
        for corner in corners:
            # Solve PnP for each marker
            success, rvec, tvec = cv2.solvePnP(
                marker_points, 
                corner, 
                camera_matrix, 
                dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            
            if success:
                rvecs.append(rvec)
                tvecs.append(tvec)
            else:
                # If IPPE_SQUARE fails, try regular ITERATIVE method
                success, rvec, tvec = cv2.solvePnP(
                    marker_points, 
                    corner, 
                    camera_matrix, 
                    dist_coeffs,
                    flags=cv2.SOLVEPNP_ITERATIVE
                )
                if success:
                    rvecs.append(rvec)
                    tvecs.append(tvec)
                else:
                    # If both fail, append None values
                    rvecs.append(None)
                    tvecs.append(None)
        
        return np.array(rvecs), np.array(tvecs), None
    
    def get_marker_poses(self, detections, timestamp):
        """Get pose estimation for detected markers"""
        poses = []
        
        for detection in detections:
            corners = detection['corners']
            ids = detection['ids']
            dict_name = detection['dictionary']
            
            if len(corners) > 0:
                # Estimate pose for each marker using the new function
                rvecs, tvecs, _ = self.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
                
                for i, (rvec, tvec, marker_id) in enumerate(zip(rvecs, tvecs, ids)):
                    # Skip if pose estimation failed
                    if rvec is None or tvec is None:
                        self.get_logger().warn(f'Failed to estimate pose for marker {marker_id}')
                        continue
                        
                    pose_info = {
                        'id': marker_id,
                        'dictionary': dict_name,
                        'rotation_vector': rvec.flatten(),
                        'translation_vector': tvec.flatten(),
                        'corners': corners[i],
                        'timestamp': timestamp
                    }
                    poses.append(pose_info)
        
        return poses
    
    def rodrigues_to_quaternion(self, rvec):
        """Convert rotation vector to quaternion using scipy"""
        rotation = R.from_rotvec(rvec)
        return rotation.as_quat()  # Returns [x, y, z, w]
    
    def publish_poses(self, poses, timestamp):
        """Publish detected marker poses as PoseArray"""
        pose_array = PoseArray()
        pose_array.header.stamp = timestamp
        pose_array.header.frame_id = self.camera_frame
        
        for pose in poses:
            pose_stamped = PoseStamped()
            
            # Set position
            pose_stamped.pose.position.x = float(pose['translation_vector'][0])
            pose_stamped.pose.position.y = float(pose['translation_vector'][1])
            pose_stamped.pose.position.z = float(pose['translation_vector'][2])
            
            # Convert rotation vector to quaternion using scipy
            quat = self.rodrigues_to_quaternion(pose['rotation_vector'])
            
            pose_stamped.pose.orientation.x = quat[0]
            pose_stamped.pose.orientation.y = quat[1]
            pose_stamped.pose.orientation.z = quat[2]
            pose_stamped.pose.orientation.w = quat[3]
            
            pose_array.poses.append(pose_stamped.pose)
        
        self.poses_pub.publish(pose_array)
    
    def publish_transforms(self, poses, timestamp):
        """Publish TF transforms for detected markers"""
        for pose in poses:
            t = TransformStamped()
            
            # Header
            t.header.stamp = timestamp
            t.header.frame_id = self.camera_frame
            t.child_frame_id = f"aruco_{pose['dictionary']}_{pose['id']}"
            
            # Translation
            t.transform.translation.x = float(pose['translation_vector'][0])
            t.transform.translation.y = float(pose['translation_vector'][1])
            t.transform.translation.z = float(pose['translation_vector'][2])
            
            # Rotation - convert rotation vector to quaternion using scipy
            quat = self.rodrigues_to_quaternion(pose['rotation_vector'])
            
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            
            # Send transform
            self.tf_broadcaster.sendTransform(t)
    
    def publish_detection_info(self, detections, timestamp):
        """Publish detection info when pose estimation is not available"""
        from std_msgs.msg import String
        
        detection_info = {
            'timestamp': timestamp.sec + timestamp.nanosec * 1e-9,
            'detections': []
        }
        
        for detection in detections:
            corners = detection['corners']
            ids = detection['ids']
            dict_name = detection['dictionary']
            
            for i, (corner, marker_id) in enumerate(zip(corners, ids)):
                # Calculate pixel size
                corner_points = corner[0]
                width = np.linalg.norm(corner_points[1] - corner_points[0])
                height = np.linalg.norm(corner_points[3] - corner_points[0])
                avg_pixel_size = (width + height) / 2
                
                # Calculate center in image coordinates
                center = np.mean(corner_points, axis=0)
                
                marker_info = {
                    'id': int(marker_id),
                    'dictionary': dict_name,
                    'pixel_size': float(avg_pixel_size),
                    'center_x': float(center[0]),
                    'center_y': float(center[1]),
                    'corners': corner_points.tolist()
                }
                detection_info['detections'].append(marker_info)
        
        # Publish as JSON string
        import json
        info_msg = String()
        info_msg.data = json.dumps(detection_info, indent=2)
        self.detection_info_pub.publish(info_msg)
    
    def estimate_marker_size_from_distance(self, pixel_size, distance_estimate=1.0):
        """
        Estimate real marker size from pixel size and estimated distance
        This is a rough estimation - actual size depends on camera parameters and distance
        """
        if self.camera_matrix is not None:
            # Rough estimation: real_size = pixel_size * distance / focal_length
            focal_length = (self.camera_matrix[0, 0] + self.camera_matrix[1, 1]) / 2
            estimated_size = pixel_size * distance_estimate / focal_length
            return estimated_size
        return None

def main(args=None):
    rclpy.init(args=args)
    
    node = ArucoDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_image:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
