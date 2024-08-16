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
from torchreid import models
import torchreid
from  matplotlib  import pyplot as plt
import cv2
import torch
from torchreid.utils import FeatureExtractor
import numpy as np

'''
from torchreid.utils import FeatureExtractor
device = "cuda" if torch.cuda.is_available() else "cpu"
model_name = "osnet_x0_75"
# # reid-specific models
# 'mudeep': MuDeep,
# 'resnet50mid': resnet50mid,
# 'hacnn': HACNN,
# 'pcb_p6': pcb_p6,
# 'pcb_p4': pcb_p4,
# 'mlfn': mlfn,
# 'osnet_x1_0': osnet_x1_0,
# 'osnet_x0_75': osnet_x0_75,
# 'osnet_x0_5': osnet_x0_5,
# 'osnet_x0_25': osnet_x0_25,
# 'osnet_ibn_x1_0': osnet_ibn_x1_0,
# 'osnet_ain_x1_0': osnet_ain_x1_0,
# 'osnet_ain_x0_75': osnet_ain_x0_75,
# 'osnet_ain_x0_5': osnet_ain_x0_5,
# 'osnet_ain_x0_25': osnet_ain_x0_25
reid_fe_extractor = FeatureExtractor(model_name=model_name,device=device)


img_path = "src/swc_perception/person_following/ccf_person_identification/data/test/p05.jpg"
img = cv2.imread(img_path)


fe = reid_fe_extractor([img, img,img,img,img,img,img])
fe.shape
'''


class YolomixNode(Node):
    def __init__(self):
        super().__init__('yolomix_node')

        # Load pose estimation model
        package_share_dir = get_package_share_directory('yolomix')
        model_path = os.path.join(package_share_dir,"weights/yolov8s-pose.pt")
        # device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"Using device: {device}")
        self.model = YOLO(model_path).to(device)

        # Load reID model
        model_name = "osnet_x0_75"
        self.reid_fe_extractor = FeatureExtractor(model_name=model_name,device=device)
        
        # 当前存在的yolo追踪id集合
        self.yolo_ids = set()
        # id特征库，第0维是id号，1~512是特征
        self.id_lib = None
        # yolo追踪的Id号与本地id特征库的映射关系
        self.id_map = None
        

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

    def reid(self, croped_persons, yolo_ids):
        
        pass

    def listener_callback(self, msg):
        yolo_persons_msg = YoloPersons()
        yolo_persons_msg.header = Header()
        yolo_persons_msg.header.stamp = self.get_clock().now().to_msg()
        yolo_persons_msg.header.frame_id = "yolomix"
        yolo_persons_msg.image = msg

        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        results = self.model.track(cv_image)

        # 判断yolo模型是否跟踪到，分配id
        if results[0].boxes.is_track:
            boxes = results[0].boxes.xywh.tolist()
            track_ids = results[0].boxes.id.int().tolist()
            keypoints = results[0].keypoints.data.tolist()

            
            # 本次yolo追踪生成的id
            # this_ids = set()
            # cropped_persons = None
             # 遍历所有检测到的对象结果
            for box, track_id, keypoint in zip(boxes, track_ids, keypoints):
                x_center, y_center, width, height = box
                # this_ids.add(track_id)


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


            # yolo跟踪的id集合出现了变动，需要进行reid
            # if this_ids != self.yolo_ids:
            #     self.yolo_ids = this_ids

            #     self.reid(cropped_persons, self.yolo_ids)  # reid的实际操作是改变映射表

        # 没有跟踪到或者没有检测到人的情况下，也发布消息，只是没有person
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
