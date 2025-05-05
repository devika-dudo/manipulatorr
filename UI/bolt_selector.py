#! /usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import math
import numpy as np
import cv2
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.Qt import *
from bolt_selector_window import Ui_Form

# ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from yolov8_msgs.msg import Yolov8Inference
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

bridge = CvBridge()

img = np.zeros([480, 640, 3])

class GraphicsScene(QGraphicsScene):
    def __init__(self, parent=None):
        QGraphicsScene.__init__(self, parent)
        self.mouse_x = 0
        self.mouse_y = 0
        self.click_mouse = False

    def mouseMoveEvent(self,event):
        self.mouse_x = event.scenePos().x()
        self.mouse_y = event.scenePos().y()

    def mousePressEvent(self, event):
        self.click_mouse = True


class GUI(QDialog):

    def __init__(self,parent=None):
        super(GUI, self).__init__(parent)
        self.ui = Ui_Form()
        self.ui.setupUi(self)

        self.scene = GraphicsScene(self.ui.graphicsView)
        self.ui.graphicsView.setScene(self.scene)
        self.ui.graphicsView.setMouseTracking(True)

        rclpy.init(args=None)
        
        # Create nodes
        self.camera_subscriber = Node('image_subscriber')
        self.yolo_subscriber = Node('yolo_subscriber')
        self.pub_node = Node('pub_path')
        
        # Set up tf2 for proper transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.pub_node)
        
        # Create subscriptions
        self.camera_sub = self.camera_subscriber.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        
        self.yolo_sub = self.yolo_subscriber.create_subscription(
            Yolov8Inference, '/Yolov8_Inference', self.yolo_callback, 10)
        
        self.pub = self.pub_node.create_publisher(
            Float64MultiArray, '/target_point', 10)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(10)

        self.brush = QBrush(QColor(255,255,255,255)) 
        self.target_point = [0, 0, 0]
        
        # Camera intrinsic parameters
        self.fx = 528.433756558705
        self.fy = 528.433756558705
        self.cx = 320.5
        self.cy = 240.5
        
        # Camera position relative to robot base (from tf2_echo)
        self.init_x = -0.159  # X position of the camera wrt robot base
        self.init_y = -0.121  # Y position of the camera wrt robot base
        self.z = 0.263       # Z position of the camera wrt robot base
        
        # Cylinder dimensions - adjust these to match your actual cylinder size
        # These values are in meters
        self.cylinder_radius = 0.025  # 2.5cm radius
        self.cylinder_height = 0.05   # 5cm height

        self.get_logger().info('GUI initialized')

    def get_logger(self):
        return self.pub_node.get_logger()

    def camera_callback(self, data):
        global img
        img = bridge.imgmsg_to_cv2(data, "bgr8")

    def yolo_callback(self, data):
        global img

        self.scene.clear()
        rgb_image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        height, width, channel = rgb_image.shape
        q_image = QImage(rgb_image.data, width, height, 3 * width, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_image)
        pixmap_item = QGraphicsPixmapItem(pixmap)
        self.scene.addItem(pixmap_item)

        for r in data.yolov8_inference:
            points = np.array(r.coordinates).astype(np.int32).reshape([4, 2])
            middle_point = np.sum(points, 0)/4
            dist = math.sqrt((self.scene.mouse_x - middle_point[0])**2 + (self.scene.mouse_y - middle_point[1])**2)
            qpoly = QPolygonF([QPointF(p[0], p[1]) for p in points])

            if dist < 15:
                self.scene.addPolygon(qpoly, QPen(QColor(255,0,0,255)), QBrush(QColor(255,0,0,100)))   

                if self.scene.click_mouse:
                    self.get_logger().info(f"Processing object at pixel position: {middle_point}")
                    
                    # Calculate width and height of bounding box
                    min_x = min(points[:, 0])
                    max_x = max(points[:, 0])
                    min_y = min(points[:, 1])
                    max_y = max(points[:, 1])
                    box_width = max_x - min_x
                    box_height = max_y - min_y
                    
                    
                    
                    s = 0.025  # half side length of cube
                    object_points = np.array([
			    [-s, -s,  s],  # bottom-left (on front face)
			    [ s, -s,  s],  # bottom-right
			    [ s,  s,  s],  # top-right
			    [-s,  s,  s],  # top-left
			], dtype=np.float32)

                    # Generate image points from the detection
                    w = box_width / 2
                    h = box_height / 2
                    image_points = np.array([
                        [middle_point[0] - w, middle_point[1] - h],
                        [middle_point[0] + w, middle_point[1] - h],
                        [middle_point[0] + w, middle_point[1] + h],
                        [middle_point[0] - w, middle_point[1] + h]
                    ], dtype=np.float32)
                    
                    # Camera matrix from intrinsics
                    camera_matrix = np.array([
                        [self.fx, 0, self.cx],
                        [0, self.fy, self.cy],
                        [0, 0, 1]
                    ])
                    
                    # Assuming minimal or no distortion
                    dist_coeffs = np.zeros((4,1))
                    
                    try:
                        # Use solvePnP to get position
                        success, rvec, tvec = cv2.solvePnP(
                            object_points, image_points, camera_matrix, dist_coeffs)
                        
                        if success:
                            # Create a point in camera optical frame
                            cam_point = PointStamped()
                            cam_point.header.stamp = self.pub_node.get_clock().now().to_msg()
                            cam_point.header.frame_id = "camera_link_optical"
                            cam_point.point.x = float(tvec[0])
                            cam_point.point.y = float(tvec[1])
                            cam_point.point.z = float(tvec[2])
                            
                            self.get_logger().info(f"Object in camera frame: [{cam_point.point.x}, {cam_point.point.y}, {cam_point.point.z}]")
                            
                            try:
                                # Transform to base_link frame using tf2
                                transform = self.tf_buffer.lookup_transform(
                                    "base_link",
                                    cam_point.header.frame_id,
                                    rclpy.time.Time(),
                                    timeout=rclpy.duration.Duration(seconds=1.0)
                                )
                                
                                # Apply transformation
                                base_point = tf2_geometry_msgs.do_transform_point(cam_point, transform)
                                
                                self.get_logger().info(f"Object in base frame: [{base_point.point.x}, {base_point.point.y}, {base_point.point.z}]")
                                
                                # Set target point
                                self.target_point[0] = base_point.point.x
                                self.target_point[1] = base_point.point.y
                                
                                # Calculate orientation angle
                                dist1 = math.sqrt((points[0][0] - points[1][0])**2 + (points[0][1] - points[1][1])**2)
                                dist2 = math.sqrt((points[1][0] - points[2][0])**2 + (points[1][1] - points[2][1])**2)
                                
                                if(dist1 > dist2):
                                    denominator = points[0][0] - points[1][0]
                                    if denominator == 0:
                                        angle = math.pi/2
                                    else:
                                        angle = math.atan2(points[0][1] - points[1][1], denominator)
                                else:
                                    denominator = points[1][0] - points[2][0]
                                    if denominator == 0:
                                        angle = math.pi/2
                                    else:
                                        angle = math.atan2(points[1][1] - points[2][1], denominator)
                                
                                self.target_point[2] = math.pi/2 - angle
                                
                                # Publish the target point
                                target_point_pub = Float64MultiArray(data=self.target_point)  
                                self.pub.publish(target_point_pub)
                                self.get_logger().info(f"Published target point: {self.target_point}")
                                
                            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                                    tf2_ros.ExtrapolationException) as e:
                                self.get_logger().error(f"TF2 Error: {e}")
                                
                                # Fallback to manual transformation if tf2 fails
                                self.get_logger().info("Using manual transformation as fallback")
                                cam_x, cam_y, cam_z = tvec.ravel()
                                
                                # Apply transformation matrix from tf2_echo
                                base_x = 0.002 * cam_x + 0.642 * cam_y - 0.766 * cam_z - 0.159
                                base_y = 1.000 * cam_x - 0.001 * cam_y + 0.002 * cam_z - 0.121
                                base_z = 0.000 * cam_x - 0.766 * cam_y - 0.642 * cam_z + 0.263
                                
                                self.target_point[0] = base_x
                                self.target_point[1] = base_y
                                
                                # Publish with manual transformation
                                target_point_pub = Float64MultiArray(data=self.target_point)  
                                self.pub.publish(target_point_pub)
                                self.get_logger().info(f"Published target point (manual transform): {self.target_point}")
                        else:
                            self.get_logger().warn("solvePnP failed")
                    except Exception as e:
                        self.get_logger().error(f"Error during pose estimation: {e}")
                    
                    self.scene.click_mouse = False
            else:
                self.scene.addPolygon(qpoly, QPen(QColor(0,0,255,255)), QBrush(QColor(0,0,255,100)))  

            self.scene.addEllipse(middle_point[0] - 2, middle_point[1] - 2, 4, 4, QPen(Qt.green), QBrush(Qt.green))

    def update(self):
        try:
            rclpy.spin_once(self.camera_subscriber, timeout_sec=0.01)
            rclpy.spin_once(self.yolo_subscriber, timeout_sec=0.01)
            rclpy.spin_once(self.pub_node, timeout_sec=0.01)
        except Exception as e:
            self.get_logger().error(f"Error in update: {e}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = GUI()
    window.show()
    sys.exit(app.exec_())
