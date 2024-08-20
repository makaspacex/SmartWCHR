import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        
        # Declare and get the 'frequency' parameter
        self.declare_parameter('frequency', 10.0)  # 默认帧率为 10.0 Hz
        frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.timer_period = 1.0 / frequency  # 根据帧率计算定时器周期
        
        self.publisher_ = self.create_publisher(Image, '/image', 10)
        self.cap = cv2.VideoCapture(0)  # 打开默认摄像头 /dev/video0
        self.bridge = CvBridge()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing image frame')
            height, width, _ = frame.shape
            self.get_logger().info(f'height:{height}, width:{width}')


    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
