#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped, Quaternion
from scipy.spatial.transform import Rotation
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
import math

class EndEffectorAligner(Node):
    def __init__(self):
        super().__init__('end_effector_aligner')

        # Declare parameters
        self.declare_parameter('offset_distance', 0.1)  # 10cm offset from panel
        self.declare_parameter('target_frame', 'base_link')  # Target reference frame
        self.declare_parameter('aruco_topic', '/aruco/poses')
        self.declare_parameter('target_topic', '/end_effector_target')

        # Get parameters
        self.offset_distance = self.get_parameter('offset_distance').value
        self.target_frame = self.get_parameter('target_frame').value
        aruco_topic = self.get_parameter('aruco_topic').value
        target_topic = self.get_parameter('target_topic').value

        # Subscribe to ArUco poses
        self.aruco_sub = self.create_subscription(
            PoseArray,
            aruco_topic,
            self.aruco_callback,
            10
        )

        # Publisher for end effector target pose
        self.target_pub = self.create_publisher(
            PoseStamped,
            target_topic,
            10
        )

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(f'End Effector Aligner started')
        self.get_logger().info(f'Listening to: {aruco_topic}')
        self.get_logger().info(f'Publishing to: {target_topic}')
        self.get_logger().info(f'Target frame: {self.target_frame}')
        self.get_logger().info(f'Offset distance: {self.offset_distance}m')

    def find_hypotenuse_center(self, poses):
        """
        Find the center of the hypotenuse (longest side) of triangle formed by 3 points
        Args:
            poses: list of geometry_msgs/Pose objects
        Returns:
            numpy array: center point of hypotenuse
        """
        # Extract positions
        p1 = np.array([poses[0].position.x, poses[0].position.y, poses[0].position.z])
        p2 = np.array([poses[1].position.x, poses[1].position.y, poses[1].position.z])
        p3 = np.array([poses[2].position.x, poses[2].position.y, poses[2].position.z])
        
        # Calculate distances between all pairs
        d12 = np.linalg.norm(p2 - p1)
        d23 = np.linalg.norm(p3 - p2)
        d13 = np.linalg.norm(p3 - p1)
        
        # Find the longest side (hypotenuse)
        distances = [d12, d23, d13]
        max_dist_idx = np.argmax(distances)
        
        if max_dist_idx == 0:  # p1-p2 is longest
            hypotenuse_center = (p1 + p2) / 2
            self.get_logger().debug(f"Hypotenuse: p1-p2, length: {d12:.3f}")
        elif max_dist_idx == 1:  # p2-p3 is longest
            hypotenuse_center = (p2 + p3) / 2
            self.get_logger().debug(f"Hypotenuse: p2-p3, length: {d23:.3f}")
        else:  # p1-p3 is longest
            hypotenuse_center = (p1 + p3) / 2
            self.get_logger().debug(f"Hypotenuse: p1-p3, length: {d13:.3f}")
            
        return hypotenuse_center

    def calculate_panel_normal(self, poses):
        """
        Calculate normal vector from 3 ArUco marker positions
        Args:
            poses: list of geometry_msgs/Pose objects
        Returns:
            tuple: (normal_vector, hypotenuse_center)
        """
        if len(poses) < 3:
            raise ValueError("Need at least 3 poses to define a plane")

        # Extract positions
        p1 = np.array([poses[0].position.x, poses[0].position.y, poses[0].position.z])
        p2 = np.array([poses[1].position.x, poses[1].position.y, poses[1].position.z])
        p3 = np.array([poses[2].position.x, poses[2].position.y, poses[2].position.z])

        # Create two vectors in the plane
        v1 = p2 - p1
        v2 = p3 - p1

        # Normal vector via cross product
        normal = np.cross(v1, v2)
        normal_magnitude = np.linalg.norm(normal)

        if normal_magnitude < 1e-6:
            raise ValueError("Points are collinear, cannot define a plane")

        normal = normal / normal_magnitude  # Normalize

        # Get center of hypotenuse instead of centroid
        hypotenuse_center = self.find_hypotenuse_center(poses)

        return normal, hypotenuse_center

    def get_aruco_axes(self, pose):
        """
        Extract X, Y, Z axes from ArUco marker orientation
        Args:
            pose: geometry_msgs/Pose object
        Returns:
            tuple: (x_axis, y_axis, z_axis) as numpy arrays
        """
        # Convert quaternion to rotation matrix
        q = pose.orientation
        quat_array = [q.x, q.y, q.z, q.w]
        r = Rotation.from_quat(quat_array)
        rotation_matrix = r.as_matrix()
        
        # Extract axes from rotation matrix columns
        x_axis = rotation_matrix[:, 0]  # First column
        y_axis = rotation_matrix[:, 1]  # Second column  
        z_axis = rotation_matrix[:, 2]  # Third column
        
        return x_axis, y_axis, z_axis

    def create_end_effector_orientation(self, aruco_pose):
        """
        Create end effector orientation based on ArUco marker axes:
        - End Effector X-axis = ArUco Z-axis
        - End Effector Y-axis = ArUco X-axis  
        - End Effector Z-axis = ArUco Y-axis
        Args:
            aruco_pose: geometry_msgs/Pose object
        Returns:
            geometry_msgs/Quaternion
        """
        # Get ArUco marker axes
        aruco_x, aruco_y, aruco_z = self.get_aruco_axes(aruco_pose)
        
        # Map ArUco axes to End Effector axes
        ee_x_axis = aruco_z  # ArUco Z -> EE X
        ee_y_axis = aruco_x  # ArUco X -> EE Y
        ee_z_axis = aruco_y  # ArUco Y -> EE Z
        
        # Ensure all vectors are normalized
        ee_x_axis = ee_x_axis / np.linalg.norm(ee_x_axis)
        ee_y_axis = ee_y_axis / np.linalg.norm(ee_y_axis)
        ee_z_axis = ee_z_axis / np.linalg.norm(ee_z_axis)
        
        # Verify orthogonality and right-handedness
        # Check if Y = Z × X (right-hand rule)
        expected_y = np.cross(ee_z_axis, ee_x_axis)
        dot_product = np.dot(ee_y_axis, expected_y)
        
        if dot_product < 0:
            # If not right-handed, flip one axis
            ee_y_axis = -ee_y_axis
            self.get_logger().info("Flipped Y-axis to maintain right-handed coordinate system")
        
        # Log the axes for debugging
        self.get_logger().info(f"ArUco X-axis: [{aruco_x[0]:.3f}, {aruco_x[1]:.3f}, {aruco_x[2]:.3f}] -> EE Y-axis")
        self.get_logger().info(f"ArUco Y-axis: [{aruco_y[0]:.3f}, {aruco_y[1]:.3f}, {aruco_y[2]:.3f}] -> EE Z-axis")
        self.get_logger().info(f"ArUco Z-axis: [{aruco_z[0]:.3f}, {aruco_z[1]:.3f}, {aruco_z[2]:.3f}] -> EE X-axis")
        self.get_logger().info(f"End effector X-axis: [{ee_x_axis[0]:.3f}, {ee_x_axis[1]:.3f}, {ee_x_axis[2]:.3f}]")
        self.get_logger().info(f"End effector Y-axis: [{ee_y_axis[0]:.3f}, {ee_y_axis[1]:.3f}, {ee_y_axis[2]:.3f}]")
        self.get_logger().info(f"End effector Z-axis: [{ee_z_axis[0]:.3f}, {ee_z_axis[1]:.3f}, {ee_z_axis[2]:.3f}]")
        
        # Create rotation matrix [x_axis, y_axis, z_axis]
        rotation_matrix = np.column_stack([ee_x_axis, ee_y_axis, ee_z_axis])
        
        # Verify the matrix is valid
        det = np.linalg.det(rotation_matrix)
        if abs(det - 1.0) > 1e-6:
            self.get_logger().error(f"Invalid rotation matrix determinant: {det}")
        
        # Convert to quaternion
        r = Rotation.from_matrix(rotation_matrix)
        quat = r.as_quat()  # Returns [x, y, z, w]
        
        return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

    def aruco_callback(self, msg):
        """
        Callback for ArUco poses
        """
        if len(msg.poses) < 3:
            self.get_logger().warn(f"Need at least 3 ArUco markers, got {len(msg.poses)}")
            return

        try:
            # Calculate panel normal and hypotenuse center from first 3 markers
            normal, hypotenuse_center = self.calculate_panel_normal(msg.poses[:3])

            self.get_logger().debug(f"Panel normal: {normal}")
            self.get_logger().debug(f"Hypotenuse center: {hypotenuse_center}")

            # Create target pose
            target_pose = PoseStamped()
            target_pose.header.frame_id = msg.header.frame_id
            target_pose.header.stamp = msg.header.stamp

            # Position: hypotenuse center + offset along normal (away from panel)
            target_pose.pose.position.x = hypotenuse_center[0] + normal[0] * self.offset_distance
            target_pose.pose.position.y = hypotenuse_center[1] + normal[1] * self.offset_distance
            target_pose.pose.position.z = hypotenuse_center[2] + normal[2] * self.offset_distance

            # Orientation: Map ArUco axes to End Effector axes
            target_pose.pose.orientation = self.create_end_effector_orientation(msg.poses[0])

            # Transform to target frame if needed
            if msg.header.frame_id != self.target_frame:
                try:
                    # Wait for transform to be available
                    if not self.tf_buffer.can_transform(
                        self.target_frame,
                        msg.header.frame_id,
                        msg.header.stamp,
                        timeout=rclpy.duration.Duration(seconds=1.0)
                    ):
                        self.get_logger().error(f'Transform from {msg.header.frame_id} to {self.target_frame} not available')
                        return

                    # Transform the pose
                    target_pose = self.tf_buffer.transform(target_pose, self.target_frame)

                except TransformException as ex:
                    self.get_logger().error(f'Could not transform pose: {ex}')
                    return
                except Exception as ex:
                    self.get_logger().error(f'Transform error: {ex}')
                    return

            # Publish target pose
            self.target_pub.publish(target_pose)
            
            # Log detailed information
            q = target_pose.pose.orientation
            quat_array = [q.x, q.y, q.z, q.w]
            r = Rotation.from_quat(quat_array)
            rpy = r.as_euler('xyz')

            self.get_logger().info(f'Published target pose at hypotenuse center: '
                                   f'pos=({target_pose.pose.position.x:.3f}, '
                                   f'{target_pose.pose.position.y:.3f}, '
                                   f'{target_pose.pose.position.z:.3f})')
            self.get_logger().info(f'  Orientation (quaternion): x={q.x:.3f}, y={q.y:.3f}, z={q.z:.3f}, w={q.w:.3f}')
            self.get_logger().info(f'  Orientation (RPY rad): roll={rpy[0]:.3f}, pitch={rpy[1]:.3f}, yaw={rpy[2]:.3f}')
            self.get_logger().info(f'  Axis mapping: ArUco Z->EE X, ArUco X->EE Y, ArUco Y->EE Z')
            
        except Exception as e:
            self.get_logger().error(f'Error processing ArUco poses: {e}')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = EndEffectorAligner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
