import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from my_msgs.msg import HandsInfo
from cv_bridge import CvBridge
import cv2

class HandsInfoVisualizerNode(Node):

    def __init__(self):
        super().__init__('hands_info_visualizer_node')
        self.subscription_image = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10)
        self.subscription_hands_info = self.create_subscription(
            HandsInfo,
            '/hands_info',
            self.hands_info_callback,
            10)
        self.publisher = self.create_publisher(Image, '/visualized_hands_info_image', 10)
        self.bridge = CvBridge()
        self.current_image = None
        self.current_hands_info = None
        self.get_logger().info('HandsInfo visualizer node has been started.')

    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_and_publish_image(msg.header.stamp)

    def hands_info_callback(self, msg):
        self.current_hands_info = msg
        self.process_and_publish_image(msg.header.stamp)

    def process_and_publish_image(self, timestamp):
        if self.current_image is not None and self.current_hands_info is not None:
            image = self.current_image.copy()
            for hand_info in self.current_hands_info.hands:
                bbox = (int(hand_info.center.x - hand_info.bbox_width / 2),
                        int(hand_info.center.y - hand_info.bbox_height / 2),
                        int(hand_info.bbox_width),
                        int(hand_info.bbox_height))
                cv2.rectangle(image, (bbox[0], bbox[1]), 
                              (bbox[0] + bbox[2], bbox[1] + bbox[3]), 
                              (0, 255, 0), 2)
                cv2.putText(image, f'ID: {hand_info.id} Gesture: {hand_info.gesture}', 
                            (bbox[0], bbox[1] - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 
                            0.5, (0, 255, 0), 2)
                
                self.get_logger().info(f'ID:{hand_info.id}')
                
            visualized_image_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            visualized_image_msg.header.stamp = timestamp

            


            self.publisher.publish(visualized_image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HandsInfoVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()