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
from torchreid.utils import FeatureExtractor
import numpy as np
import time
from ultralytics.engine.results import Results
from typing import List, Union
from ultralytics.trackers.basetrack import BaseTrack
from ultralytics.trackers.byte_tracker import STrack


# from sklearn.metrics.pairwise import cosine_similarity

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
        
        # self.tracker_config_file = os.path.join(package_share_dir,"config/custom_botsort.yaml")
        self.tracker_config_file = os.path.join(package_share_dir,"config/custom_bytetrack.yaml")
        
        
        # 当前存在的yolo追踪id集合
        self.yolo_ids = set()
        # id特征库，第0维是id号，1~512是特征
        self.id_lib = None
        # yolo追踪的Id号与本地id特征库的映射关系
        self.id_map = None

        # 初始化特征库，初始时为空（0行，513列）
        self.feature_library = np.empty((0, 513))
        self.next_id = 1  # 初始化ID从1开始
        

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Create a publisher for YoloPersons messages
        self.persons_publisher = self.create_publisher(YoloPersons, '/yolo_persons', 10)
        
        self.annotated_image_publisher = self.create_publisher(Image, '/annotated_image', 10)
        self.bridge = CvBridge()
        
        # Create a subscriber for the image topic
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.listener_callback,
            10
        )
        self.subscription

        self.last_save_time = time.time()
        self.save_id = 0

    def reid(self, croped_persons, yolo_ids):
        
        pass


    def get_roi(self, image, box):
        x_center, y_center, width, height = box
        # 计算左上角和右下角的坐标
        x1 = int(x_center - width / 2)
        y1 = int(y_center - height / 2)
        x2 = int(x_center + width / 2)
        y2 = int(y_center + height / 2)

        # 确保坐标在图像边界内
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(image.shape[1], x2)  # image.shape[1] 是图像的宽度
        y2 = min(image.shape[0], y2)  # image.shape[0] 是图像的高度

        # 提取ROI
        roi = image[y1:y2, x1:x2]
        return roi
    

    def cosine_similarity(self, vector_a, vector_b):
        dot_product = np.dot(vector_a, vector_b)
        norm_a = np.linalg.norm(vector_a)
        norm_b = np.linalg.norm(vector_b)
        return dot_product / (norm_a * norm_b)
    
    def match_and_assign_id(self, detected_features, threshold=0.6):
        assigned_ids = []
        
        for feature in detected_features:
            best_match_id = None
            max_similarity = 0

            for stored_feature in self.feature_library:
                stored_id = stored_feature[0]
                stored_vector = stored_feature[1:]

                similarity = self.cosine_similarity(feature, stored_vector)
                self.get_logger().info(f'To {stored_id}    similarity = {similarity}')

                if similarity > max_similarity and similarity > threshold:
                    max_similarity = similarity
                    best_match_id = stored_id

            if best_match_id is not None:
                assigned_ids.append(best_match_id)
            else:
                # 分配新的ID
                new_id = self.next_id
                self.next_id += 1

                # 将新特征添加到特征库
                new_entry = np.concatenate(([new_id], feature))
                self.feature_library = np.vstack([self.feature_library, new_entry])

                assigned_ids.append(new_id)

        return assigned_ids


    def listener_callback(self, msg):

        # now = time.time()
        # save_path = ""
        # if now - self.last_save_time > 2:
        # 
        #     torch.save(self.feature_library, f"/home/jetson/Desktop/SmartWCHR/src/swc_perception/yolomix/yolomix/feature_librarys/feature_library{self.save_id}.pth")
        #     self.save_id += 1

        yolo_persons_msg = YoloPersons()
        yolo_persons_msg.header = Header()
        yolo_persons_msg.header.stamp = self.get_clock().now().to_msg()
        yolo_persons_msg.header.frame_id = "yolomix"
        yolo_persons_msg.image = msg

        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        results:List[Results] = self.model.track(cv_image, persist=True, conf=0.65,  tracker=self.tracker_config_file)
        
        annotated_frame = results[0].plot()
        
        # Convert the OpenCV image back to ROS Image message
        annotated_image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        # Create a new header for the annotated image
        annotated_image_msg.header = Header()
        annotated_image_msg.header.stamp = self.get_clock().now().to_msg()
        annotated_image_msg.header.frame_id = msg.header.frame_id

        # Publish the annotated image
        self.annotated_image_publisher.publish(annotated_image_msg)
        
        if len(results[0].boxes) == 0:
            if hasattr(self.model.predictor, "trackers"):
                count = BaseTrack._count
                self.model.predictor.trackers[0].reset()
                BaseTrack._count = count
        # 判断yolo模型是否跟踪到，分配id
        if results[0].boxes.is_track:
            boxes = results[0].boxes.xywh.tolist()
            track_ids = results[0].boxes.id.int().tolist()
            keypoints = results[0].keypoints.data.tolist()
            
            roi_images = []
             # 遍历所有检测到的对象结果
            for box, track_id, keypoint in zip(boxes, track_ids, keypoints):
                x_center, y_center, width, height = box

                roi_image = self.get_roi(cv_image, box)
                roi_images.append(roi_image)

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

            # if len(roi_images) > 0:
            #     detected_features = self.reid_fe_extractor(roi_images).cpu()
            #     assign_ids = self.match_and_assign_id(detected_features)
            #     for idx, person in enumerate(yolo_persons_msg.persons):
            #         person.id = int(assign_ids[idx])

        # 没有跟踪到或者没有检测到人的情况下，也发布消息，只是没有person
        self.persons_publisher.publish(yolo_persons_msg)
        # self.get_logger().info('Publish a detected pose image')


def main(args=None):
    rclpy.init(args=args)
    node = YolomixNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
