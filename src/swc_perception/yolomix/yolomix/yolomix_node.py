# pose_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from yolomix_msgs.msg import YoloPerson, YoloPersons
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
from ament_index_python.packages import get_package_share_directory
import os
from geometry_msgs.msg import Point
from std_msgs.msg import Header



class YolomixNode(Node):
    def __init__(self):
        super().__init__('yolomix_node')

        # Load model
        package_share_dir = get_package_share_directory('yolomix')
        model_path = os.path.join(package_share_dir,"weights/yolov8s-pose.pt")
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        print(f"Using device: {device}")
        self.model = YOLO(model_path).to(device)

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Create a publisher for YoloPersons messages
        self.persons_publisher = self.create_publisher(YoloPersons, '/yolo_persons', 10)

        # Create a subscriber for the image topic
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):

        yolo_persons_msg = YoloPersons()
        yolo_persons_msg.header = Header()
        yolo_persons_msg.header.stamp = self.get_clock().now().to_msg()
        yolo_persons_msg.header.frame_id = "yolomix"
        yolo_persons_msg.image = msg

        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        results = self.model.track(cv_image)

        if results[0].boxes.is_track:
            boxes = results[0].boxes.xywh.tolist()
            track_ids = results[0].boxes.id.int().tolist()
            keypoints = results[0].keypoints.data.tolist()

            
             # 遍历所有检测到的对象结果
            for box, track_id, keypoint in zip(boxes, track_ids, keypoints):
                x_center, y_center, width, height = box

                # 创建 YoloPerson 消息
                yolo_person_msg = YoloPerson()
                yolo_person_msg.id = track_id
                yolo_person_msg.center = Point(x=x_center, y=y_center, z=0.0)
                yolo_person_msg.width = width
                yolo_person_msg.height = height

                # 添加关键点到 YoloPerson 消息中
                yolo_person_msg.keypoints = [Point(x=kp[0], y=kp[1], z=0.0) for kp in keypoint]

                # 将 YoloPerson 消息添加到 YoloPersons 消息中
                yolo_persons_msg.persons.append(yolo_person_msg)
        
            self.persons_publisher.publish(yolo_persons_msg)
            self.get_logger().info('Publish a detected pose image')


def main(args=None):
    rclpy.init(args=args)
    node = YolomixNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
