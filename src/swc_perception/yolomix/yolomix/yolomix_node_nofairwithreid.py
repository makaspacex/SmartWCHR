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

from norfair import draw_points, draw_tracked_objects, get_cutout
from norfair.filter import OptimizedKalmanFilterFactory
from norfair import Tracker as NofairTrakerWithReid
from norfair import Detection as NFDetection
import norfair


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

def get_hist(image):
    hist = cv2.calcHist(
        [cv2.cvtColor(image, cv2.COLOR_BGR2Lab)],
        [0, 1],
        None,
        [128, 128],
        [0, 256, 0, 256],
    )
    return cv2.normalize(hist, hist).flatten()


def embedding_distance(matched_not_init_trackers, unmatched_trackers):
    snd_embedding = unmatched_trackers.last_detection.embedding

    if snd_embedding is None:
        for detection in reversed(unmatched_trackers.past_detections):
            if detection.embedding is not None:
                snd_embedding = detection.embedding
                break
        else:
            return 1

    for detection_fst in matched_not_init_trackers.past_detections:
        if detection_fst.embedding is None:
            continue

        distance = 1 - cv2.compareHist(
            snd_embedding, detection_fst.embedding, cv2.HISTCMP_CORREL
        )
        if distance < 0.5:
            return distance
    return 1


def embedding_distance_v2(matched_not_init_trackers, unmatched_trackers):
    snd_embedding = unmatched_trackers.last_detection.embedding

    if snd_embedding is None:
        for detection in reversed(unmatched_trackers.past_detections):
            if detection.embedding is not None:
                snd_embedding = detection.embedding
                break
        else:
            return 1

    for detection_fst in matched_not_init_trackers.past_detections:
        if detection_fst.embedding is None:
            continue

        distance = 1 - cv2.compareHist(snd_embedding, detection_fst.embedding, cv2.HISTCMP_CORREL)
        
        if distance < 0.5:
            return distance
    return 1


def yolo_detections_to_norfair_detections(yolo_detections: Results, track_points: str = "centroid"  # bbox or centroid
) -> List[NFDetection]:
    """convert detections_as_xywh to norfair detections"""
    norfair_detections: List[NFDetection] = []
    if track_points == "centroid":
        detections_as_xywh = yolo_detections.boxes.xywh
        confs = yolo_detections.boxes.conf
        for detection_as_xywh, conf in zip(detections_as_xywh,confs):
            centroid = np.array(
                [detection_as_xywh[0].item(), detection_as_xywh[1].item()]
            )
            scores = np.array([conf.item()])
            norfair_detections.append(
                NFDetection(
                    points=centroid,
                    scores=scores,
                    label=int(detection_as_xywh[-1].item()),
                )
            )
    elif track_points == "bbox":
        detections_as_xyxy = yolo_detections.boxes.xyxy
        confs = yolo_detections.boxes.conf
        for detection_as_xyxy, conf in zip(detections_as_xyxy,confs):
            bbox = np.array(
                [
                    [detection_as_xyxy[0].item(), detection_as_xyxy[1].item()],
                    [detection_as_xyxy[2].item(), detection_as_xyxy[3].item()],
                ]
            )
            scores = np.array(
                [conf.item(), conf.item()]
            )
            norfair_detections.append(
                NFDetection(
                    points=bbox, scores=scores, label=int(detection_as_xyxy[-1].item())
                )
            )

    return norfair_detections


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
        
        # 跟踪
        self.track_points = "centroid"
        DISTANCE_THRESHOLD_BBOX: float = 0.7
        DISTANCE_THRESHOLD_CENTROID: int = 50

        distance_function = "iou" if self.track_points == "bbox" else "euclidean"
        distance_threshold = ( DISTANCE_THRESHOLD_BBOX
        if self.track_points == "bbox"
        else DISTANCE_THRESHOLD_CENTROID
        )
        
        self.tracker_withreid = NofairTrakerWithReid(
            initialization_delay=1,
            distance_function=distance_function,
            hit_counter_max=10,
            filter_factory=OptimizedKalmanFilterFactory(),
            distance_threshold=distance_threshold,
            past_detections_length=5,
            reid_distance_function=embedding_distance,
            reid_distance_threshold=0.5,
            reid_hit_counter_max=500,
        )
        
        # Create a CvBridge object
        self.bridge = CvBridge()

        # Create a publisher for YoloPersons messages
        self.persons_publisher = self.create_publisher(YoloPersons, '/yolo_persons', 10)
        
        self.annotated_image_publisher = self.create_publisher(Image, '/annotated_image_reid', 10)
        self.bridge = CvBridge()
        
        
        device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"Using device: {device}")
        
        # Load reID model
        model_name = "osnet_x0_75"
        self.reid_fe_extractor = FeatureExtractor(model_name=model_name,device=device)
        
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
        self.get_logger().info(f"yolomix node is ready.")

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

    def listener_callback(self, msg):

        yolo_persons_msg = YoloPersons()
        yolo_persons_msg.header = Header()
        yolo_persons_msg.header.stamp = self.get_clock().now().to_msg()
        yolo_persons_msg.header.frame_id = "yolomix"
        yolo_persons_msg.image = msg

        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 得到yolo的检测结果
        results:List[Results] = self.model(cv_image)
        
        # 绘制yolo的检测结果
        # annotated_frame = results[0].plot()
        annotated_frame = cv_image

        # 将yolo的检测结果转换为nofair的检测结果格式
        detections = yolo_detections_to_norfair_detections(
            results[0], track_points=self.track_points
        )
        
        for detection in detections:
            cut = get_cutout(detection.points, cv_image)
            if cut.shape[0] > 0 and cut.shape[1] > 0:
                detection.embedding = self.reid_fe_extractor(cut).cpu().detach().numpy()[0]
            else:
                detection.embedding = None
        
        # 带有reid的跟踪
        tracked_objects = self.tracker_withreid.update(detections=detections)
        
        # 绘制跟踪的结果
        if self.track_points == "centroid":
            norfair.draw_points(annotated_frame, detections)
            norfair.draw_tracked_objects(annotated_frame, tracked_objects)
        elif self.track_points == "bbox":
            norfair.draw_boxes(annotated_frame, detections)
            norfair.draw_tracked_boxes(annotated_frame, tracked_objects)
        # 发布可视化的图
        # Convert the OpenCV image back to ROS Image message
        annotated_image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        # Create a new header for the annotated image
        annotated_image_msg.header = Header()
        annotated_image_msg.header.stamp = self.get_clock().now().to_msg()
        annotated_image_msg.header.frame_id = msg.header.frame_id

        # Publish the annotated image
        self.annotated_image_publisher.publish(annotated_image_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YolomixNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
