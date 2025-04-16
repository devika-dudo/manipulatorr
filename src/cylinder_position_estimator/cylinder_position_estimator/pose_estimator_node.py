import rclpy
import tf2_ros
import tf2_geometry_msgs
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class PoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('pose_estimator_node')
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.transformed_pose_pub = self.create_publisher(PointStamped, '/cylinder/pose_base', 10)

        # Cambutera info
        self.camera_matrix = None
        self.dist_coeffs = None

        # Subscribers
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Replace with your actual detection topic if needed
        self.create_subscription(Float32MultiArray, '/yolo/centre', self.detection_callback, 10)

        # Publisher: 3D position of the cylinder
        self.pose_pub = self.create_publisher(PointStamped, '/cylinder/pose', 10)

        self.get_logger().info('PoseEstimatorNode is running.')

    def camera_info_callback(self, msg):
        k = np.array(msg.k).reshape(3, 3)
        self.camera_matrix = k
        self.dist_coeffs = np.array(msg.d).reshape(-1, 1)
        self.get_logger().info('Camera parameters received.')

    def detection_callback(self, msg):
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warn('Waiting for camera intrinsics...')
            return
        if len(msg.data) < 2:
            self.get_logger().warn('Invalid detection data received.')
            return

        cx, cy = msg.data[0], msg.data[1]
        self.get_logger().info(f'Received detection: x={cx}, y={cy}')
        box_width = msg.data[2]
        box_height = msg.data[3]

        w = box_width / 2
        h = box_height / 2
        # Use a fixed model of the cylinder
        object_points = np.array([
            [-0.025, 0.0, -0.025],  # bottom-left
            [ 0.025, 0.0, -0.025],  # bottom-right
            [ 0.025, 0.0,  0.025],  # top-right
            [-0.025, 0.0,  0.025],  # top-left
        ], dtype=np.float32)
        
        # Generate fake corners around the center (for testing)
        
        image_points = np.array([
            [cx - w, cy - h],
            [cx + w, cy - h],
            [cx + w, cy + h],
            [cx - w, cy + h]
        ], dtype=np.float32)

        success, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs)

        if success:
            position_msg = PointStamped()
            position_msg.header.stamp = self.get_clock().now().to_msg()
            position_msg.header.frame_id = 'camera_link_optical'
            position_msg.point.x = float(tvec[0])
            position_msg.point.y = float(tvec[1])
            position_msg.point.z = float(tvec[2])
            self.pose_pub.publish(position_msg)
            self.get_logger().info(f'Cylinder position: {tvec.ravel()}')
            
            try:
                # Transform to 'world' frame
                transform = self.tf_buffer.lookup_transform(
    'world',
    position_msg.header.frame_id,
    rclpy.time.Time(),  # use latest available transform
    timeout=rclpy.duration.Duration(seconds=1.0)
)

                transformed_pose = tf2_geometry_msgs.do_transform_point(position_msg, transform)
                self.transformed_pose_pub.publish(transformed_pose)
                self.get_logger().info(f'Cylinder position (world): ({transformed_pose.point.x}, {transformed_pose.point.y}, {transformed_pose.point.z})')

            except Exception as e:
                self.get_logger().warn(f'Could not transform pose to world: {e}')

        else:
            self.get_logger().warn('solvePnP failed')

    def image_callback(self, msg):
        # Optional: You can visualize things here later using cv2
        pass

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


