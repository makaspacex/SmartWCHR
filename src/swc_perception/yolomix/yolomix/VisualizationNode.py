import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from yolomix_msgs.msg import YoloPerson, YoloPersons
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import numpy as np

class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Create a subscriber for the YoloPersons messages
        self.subscription = self.create_subscription(
            YoloPersons,
            '/yolo_persons',
            self.listener_callback,
            10
        )
        self.subscription

        # Create a publisher for the annotated image
        self.publisher = self.create_publisher(Image, '/annotated_image', 10)

    def listener_callback(self, msg):
        # Convert the ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='bgr8')

        # Process each YoloPerson message
        for person in msg.persons:
            # Extract bounding box coordinates and convert to int
            x_center = int(person.center.x)
            y_center = int(person.center.y)
            width = int(person.width)
            height = int(person.height)
            x_min = int(x_center - width / 2)
            y_min = int(y_center - height / 2)
            x_max = int(x_center + width / 2)
            y_max = int(y_center + height / 2)

            # Draw bounding box
            cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

            # Draw ID
            text = f"ID: {person.id}"
            cv2.putText(cv_image, text, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


            # Draw keypoints
            draw_points = [6, 10]
            for idx, kp in enumerate(person.keypoints):
                if idx in draw_points:
                    x, y = int(kp.x), int(kp.y)
                    cv2.circle(cv_image, (x, y), 3, (0, 0, 255), -1)

        # Convert the OpenCV image back to ROS Image message
        annotated_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')

        # Create a new header for the annotated image
        annotated_image_msg.header = Header()
        annotated_image_msg.header.stamp = self.get_clock().now().to_msg()
        annotated_image_msg.header.frame_id = msg.header.frame_id

        # Publish the annotated image
        self.publisher.publish(annotated_image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
