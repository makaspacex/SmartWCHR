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
import time
# from sklearn.metrics.pairwise import cosine_similarity
# from .sort import Sort
from ultralytics.engine.results import Results
from typing import List, Union
from ultralytics.trackers.basetrack import BaseTrack
from ultralytics.trackers.byte_tracker import STrack
from std_msgs.msg import String



class YolomixNode(Node):
    def __init__(self):
        super().__init__('yolomix_reid_node')

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
        # Create a CvBridge object
        self.cv_bridge = CvBridge()
        # self.sort_tracker = Sort()  # 初始化 SORT 跟踪器

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

        self.tracker_config_file = os.path.join(package_share_dir,"config/custom_bytetrack.yaml")


        self.target_id = None
        self.target_feature = None
        self.alpha = 0.9
        self.similarity_threshold = 0.8
        self.yolo_persons = None


        self.visual_publisher = self.create_publisher(Image, '/reid_visualization', 10)
        self.log_publisher = self.create_publisher(String, 'reid_log', 10)



    def is_target(self, keypoint):
        left_shoulder = keypoint[5]  # left shoulder is index 5
        left_wrist = keypoint[9]    # left wrist is index 9

        if left_shoulder[0] > 0 and left_wrist[0] > 0:  
            # Compare y-coordinates to check if wrist is higher than shoulder
            if left_wrist[1] < left_shoulder[1]:
                return True
        return False


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
    

    def log(self, log_data):
        log_msg = String()
        log_msg.data = log_data
        self.log_publisher.publish(log_msg)
    

    def listener_callback(self, msg):
        self.Visualizatin()

        yolo_persons_msg = YoloPersons()
        yolo_persons_msg.header = Header()
        yolo_persons_msg.header.stamp = self.get_clock().now().to_msg()
        yolo_persons_msg.header.frame_id = "yolomix"
        yolo_persons_msg.image = msg

        # Convert ROS Image message to OpenCV image
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        

        results:List[Results] = self.model.track(cv_image, persist=True, conf=0.65,  tracker=self.tracker_config_file)


        if len(results[0].boxes) == 0:
            if hasattr(self.model.predictor, "trackers"):
                count = BaseTrack._count
                self.model.predictor.trackers[0].reset()
                BaseTrack._count = count
        


        boxes = results[0].boxes.xywh.tolist()
        # if len(boxes) == 0:
            # self.get_logger().info('update a null list')
        #     self.sort_tracker.update(np.array(np.array([])))
        

        # 判断yolo模型是否跟踪到，分配id
        if not results[0].boxes.is_track:
            self.persons_publisher.publish(yolo_persons_msg)
            self.yolo_persons = yolo_persons_msg
            # self.get_logger().info('self.yolo_persons = yolo_persons_msg')
            return


        boxes = results[0].boxes.xywh.tolist()
        track_ids = results[0].boxes.id.int().tolist()
        keypoints = results[0].keypoints.data.tolist()
        
        roi_images = []
        have_find = False  # 本次是否找到target


        # 遍历所有检测到的对象结果
        for box, track_id, keypoint in zip(boxes, track_ids, keypoints):
        # for box, keypoint, tracked_object in zip(boxes, keypoints):
        # for box, keypoint in zip(boxes, keypoints):
            x_center, y_center, width, height = box
            # track_id = int(tracked_object[4])
            
            
            # 获取行人的 ROI 图片
            roi_image = self.get_roi(cv_image, box)
            roi_images.append(roi_image)


            if self.target_id is None:  # 第一次通过 keypoint 确定 target_id
                if self.is_target(keypoint):
                    self.target_id = track_id
                    target_feature = self.reid_fe_extractor([roi_image])[0].cpu()
                    self.target_feature = target_feature  # 初始化 target_feature
                    have_find = True

            elif track_id == self.target_id:  # 如果检测到的 track_id 与 target_id 一致
                target_feature = self.reid_fe_extractor([roi_image])[0].cpu()
                # 更新目标行人的特征向量
                self.target_feature = self.alpha * target_feature + (1 - self.alpha) * self.target_feature
                have_find = True


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


        # 之前已经检测到目标，现在丢失了
        # if self.target_feature and not have_find:
        #     pass

        
        if not have_find and (not (self.target_feature is None)) and len(roi_images) > 0:
            detected_features = self.reid_fe_extractor(roi_images).cpu()
            similarities = [self.cosine_similarity(f, self.target_feature) for f in detected_features]

            for idx, similarite in enumerate(similarities):
                this_id = results[0].boxes.id.int().tolist()[idx]
                log_data = f'similarite from {this_id} to target_id {self.target_id} is {similarite}'
                self.log(log_data)
            

            # 获取相似度最高且超过阈值的候选者
            max_similarity = max(similarities)
            self.get_logger().info('max_similarity:       ' + str(max_similarity))
            if max_similarity > self.similarity_threshold:
                best_match_index = similarities.index(max_similarity)
                # self.target_id = tracked_objects[best_match_index][4]  # 更新target_id
                last_target_id = self.target_id

                self.target_id = results[0].boxes.id.int().tolist()[best_match_index]  # 更新target_id



                log_data = f'target_id changed, from {last_target_id} to {self.target_id}, the similarity: {max_similarity}'
                self.log(log_data)
                

                self.get_logger().info('update target:       ' + str(results[0].boxes.id.int()[best_match_index]))


                # 滑动平均更新 target 特征
                new_feature = detected_features[best_match_index]
                self.target_feature = self.alpha * new_feature + (1 - self.alpha) * self.target_feature


        self.persons_publisher.publish(yolo_persons_msg)
        self.yolo_persons = yolo_persons_msg



    def Visualizatin(self):
        # self.get_logger().info('Visualization function is called')
        if self.yolo_persons is None:
            self.get_logger().info('yolo_persons is none')
            return
        
        # 将ROS图像消息转换为OpenCV图像
        cv_image = self.cv_bridge.imgmsg_to_cv2(self.yolo_persons.image, desired_encoding='bgr8')

        for person in self.yolo_persons.persons:
            # 获取边界框坐标和中心点
            center_x, center_y = int(person.center.x), int(person.center.y)
            bbox_width, bbox_height = int(person.width), int(person.height)

            # 计算边界框的坐标
            x_min = int(center_x - bbox_width / 2)
            y_min = int(center_y - bbox_height / 2)
            x_max = int(center_x + bbox_width / 2)
            y_max = int(center_y + bbox_height / 2)

            # 根据ID选择框的颜色
            if person.id == self.target_id:
                color = (0, 0, 255)  # 红色
            else:
                color = (255, 0, 0)  # 蓝色

            # 绘制边界框
            cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), color, 1)
            
            # Draw ID
            text = f"ID: {person.id}"
            cv2.putText(cv_image, text, (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 将OpenCV图像转换回ROS图像消息
        output_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

        # 发布修改后的图像
        self.visual_publisher.publish(output_msg)
        # self.get_logger().info('publish a reid visualization image')

    


def main(args=None):
    rclpy.init(args=args)
    node = YolomixNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
