import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        self.declare_parameter('frequency', 20.0)  # 默认帧率为 20.0 Hz
        frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.timer_period = 1.0 / frequency  # 根据帧率计算定时器周期
        
        self.publisher_ = self.create_publisher(Image, '/image', 10)
        self.cap = cv2.VideoCapture(0)  # 打开默认摄像头 /dev/video0
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*list("MJPG")))
        
        self.get_logger().info(f"CAP_PROP_FPS: {self.cap.get(cv2.CAP_PROP_FPS)}")
        # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*list("YUYV")))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.bridge = CvBridge()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.last_print, self.last_frame_n, self.frame_n = time.time(),0,0

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        self.frame_n += 1
        _cur_time = time.time()
        
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(msg)
        height, width, _ = frame.shape
        if _cur_time - self.last_print >1:
            rate = (self.frame_n - self.last_frame_n)/(_cur_time - self.last_print)
            self.last_print = _cur_time
            self.last_frame_n = self.frame_n
            self.get_logger().info(f'Publishing image frame height:{height}, width:{width} {rate:.2f}fps')
            
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
