import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from person_tracking_msgs.msg import PersonInfo, PersonsInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import message_filters
import numpy as np

class PersonsVisualizerNode(Node):
    def __init__(self):
        super().__init__('persons_visualizer_node')

        # qos_profile = QoSProfile(depth=10)
        
        # 创建两个订阅者，一个订阅图像，一个订阅Persons信息
        self.image_subscriber = message_filters.Subscriber(self, Image, '/image')
        self.persons_subscriber = message_filters.Subscriber(self, PersonsInfo, '/persons_info')
        
        # 同步这两个订阅者
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_subscriber, self.persons_subscriber], 10, 0.2)
        self.ts.registerCallback(self.callback)
        
        # 创建发布者，发布处理后的图像
        self.image_publisher = self.create_publisher(Image, '/persons_visualize_image', 10)

        self.bridge = CvBridge()



    def draw_keypoints(self, image, key_landmark):
        h, w, _ = image.shape
        x = int(key_landmark.x)
        y = int(key_landmark.y)

        # x = int(key_landmark.x * w)
        # y = int(key_landmark.y * h)
        cv2.circle(image, (x, y), 10, (0, 255, 0), -1)  # 绘制小圈

        # self.get_logger().info(f"key_point_x : {x}    key_point_y : {y}")

        return image
        # 输出关键点信息
        # print(f'Keypoint {idx} at ({x}, {y}), visibility: {landmark.visibility}')


    def callback(self, image_msg, persons_msg):
        self.get_logger().info("receive a image and person info.......")


        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        
        # 在图像上绘制Persons信息
        for person in persons_msg.persons:
            x = int(person.center.x - person.width / 2)
            y = int(person.center.y - person.height / 2)
            w = int(person.width)
            h = int(person.height)
            id = person.id

            # 绘制边框和ID
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(cv_image, f"ID: {id}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            for idx, key_point in enumerate(person.keypoints):
                # 11 15 12 16
                draw_points = [12, 16]
                if idx in draw_points:
                    if key_point.x > 0 and key_point.y > 0:
                        cv_image = self.draw_keypoints(cv_image, key_point)
                    # self.get_logger().info("draw key point。。。。。。。。。。。。")
                    
        try:
            # 将处理后的图像转换为ROS图像消息并发布
            processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_publisher.publish(processed_image_msg)
            # self.get_logger().info("publish a person image。。。。。。。。。。。。")

        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert processed image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PersonsVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
