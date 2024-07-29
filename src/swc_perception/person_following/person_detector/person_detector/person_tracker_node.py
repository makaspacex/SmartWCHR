import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from person_tracking_msgs.msg import PersonInfo, PersonsInfo
import cv2
import mediapipe as mp
import numpy as np
from .sort import Sort
from cv_bridge import CvBridge

class PersonTrackerNode(Node):
    def __init__(self):
        super().__init__('person_tracker_node')
        
        # 初始化 MediaPipe 和 SORT
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        self.tracker = Sort()
        
        # 初始化 CvBridge
        self.bridge = CvBridge()
        
        # 创建订阅者和发布者
        self.image_subscriber = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )
        
        self.persons_publisher = self.create_publisher(PersonsInfo, '/persons_info', 10)

    def image_callback(self, msg):
        # 将 ROS 图像消息转换为 OpenCV 图像
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 将 BGR 图像转换为 RGB 图像
        rgb_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        
        # 处理图像并获取姿态估计结果
        results = self.pose.process(rgb_frame)
        
        detections = []
        keypoints_list = []
        
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark
            h, w, _ = cv_image.shape

            min_x = min_y = float('inf')
            max_x = max_y = float('-inf')

            # 初始化关节坐标列表
            keypoints = []

            for landmark in landmarks:
                x, y = int(landmark.x * w), int(landmark.y * h)
                min_x = min(min_x, x)
                min_y = min(min_y, y)
                max_x = max(max_x, x)
                max_y = max(max_y, y)
                keypoints.append(Point(x=float(x), y=float(y), z=0.0))

            detections.append([min_x, min_y, max_x, max_y])
            keypoints_list.append(keypoints)

        detections = np.array(detections)
        trackers = self.tracker.update(detections)

        # 创建 PersonsInfo 消息
        persons_info_msg = PersonsInfo()
        persons_info_msg.header = msg.header

        for i, track in enumerate(trackers):
            x1, y1, x2, y2, track_id = track
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            width = x2 - x1
            height = y2 - y1

            person_info = PersonInfo()
            person_info.id = int(track_id)
            person_info.center = Point(x=center_x, y=center_y, z=0.0)
            person_info.width = float(width)
            person_info.height = float(height)
            
            # 添加关节坐标到 PersonInfo
            if i < len(keypoints_list):
                person_info.keypoints = keypoints_list[i]

            persons_info_msg.persons.append(person_info)
        
        # 发布 PersonsInfo 消息
        self.persons_publisher.publish(persons_info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
