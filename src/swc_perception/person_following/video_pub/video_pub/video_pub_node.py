import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
from ament_index_python.packages import get_package_share_directory

class VideoPublisherNode(Node):
    def __init__(self):
        super().__init__('video_publisher_node')

        # 创建发布者，发布图像消息
        self.publisher_ = self.create_publisher(Image, '/image', 10)
        self.bridge = CvBridge()

        # 设置视频文件路径
        video_path = 'test_videos/kun_test.mp4'
        package_share_directory = get_package_share_directory('video_pub')
        self.video_path = os.path.join(package_share_directory, video_path)

        # self.video_path = os.path.join(self.get_package_share_directory('video_pub'), video_path)  # 这里设置你的视频文件路径
        self.cap = cv2.VideoCapture(self.video_path)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video file: {self.video_path}")
            return
        
        # 创建一个定时器，每隔一段时间发布一帧图像
        timer_period = 0.03  # 设置为每秒发布10帧（根据视频帧率调整）
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("End of video file reached")
            self.cap.release()
            self.timer.cancel()
            return
        
        try:
            # 将OpenCV图像转换为ROS图像消息
            image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(image_msg)
            self.get_logger().info("publish a image")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
