import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolov8_obb_node')
        self.bridge = CvBridge()

        # Subscriber: raw camera image
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher: annotated image
        self.image_pub = self.create_publisher(Image, '/yolo/detections', 10)

        # Publisher: center coordinates of bounding boxes
        self.center_pub = self.create_publisher(Float32MultiArray, '/yolo/centre', 10)

        # Load YOLOv8 model (rotated bounding box or standard depending on what you trained)
        self.model = YOLO('/home/devika/yolov8obb_training/best.pt')
        self.get_logger().info('✅ YOLOv8 OBB node is running.')

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Inference
        results = self.model(frame)[0]

        # Debug: Print the structure of the results
        self.get_logger().info(f"Results structure: {results}")

        # Check if there are any detections in the OBB (rotated bounding boxes)
        if results.obb is None or len(results.obb) == 0:
            self.get_logger().info("⚠️ No objects detected!")
        else:
            self.get_logger().info(f"✅ Detected {len(results.obb)} objects.")

        # Process detections from OBB
        centers = []

        if results.obb is not None:
            for obb in results.obb:
                # Extract center coordinates (cx, cy) from the OBB results
                xc, yc, w, h, angle = obb.xywhr[0].tolist()

                confidence = obb.conf[0].item()  # Confidence score of the detection

                # Confidence threshold to filter out low-confidence detections
                confidence_threshold = 0.3  # Example threshold (adjust as necessary)

                if confidence >= confidence_threshold:
                    centers.extend([xc, yc, w, h])

                    self.get_logger().info(f'📍 Center: ({xc:.2f}, {yc:.2f}), Box width: {w:.2f}, Box height: {h:.2f}, Angle: {angle:.2f}, Confidence: {confidence:.2f}')
                else:
                    self.get_logger().info(f"⚠️ Detected object with low confidence ({confidence:.2f})")

        # Publish centers as Float32MultiArray
        if centers:
            centre_msg = Float32MultiArray()
            centre_msg.data = centers
            self.center_pub.publish(centre_msg)

        # Publish annotated image
        annotated_frame = results.plot()
        output_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        self.image_pub.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

