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
from sklearn.metrics.pairwise import cosine_similarity as sk_cosine_similarity
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
import threading
from collections import deque
import copy
from ultralytics.utils.plotting import Annotator, colors, save_one_box


def get_roi( image, box):
    x_center, y_center, width, height = box[:4]
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

def cosine_sim_cal(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    """ 计算 a 与 b 的余弦相似度
    Args:
        a (torch.Tensor): 输入张量 a，形状为 (m, n)
        b (torch.Tensor): 输入张量 b，形状为 (k, n)
    
    Returns:
        torch.Tensor: 余弦相似度矩阵，形状为 (m, k)
    """
    if a.shape[1] != b.shape[1]:
        raise Exception("a.shape[1] != b.shape[1]")

    # 计算 a 和 b 的点积
    a_dot_b = torch.matmul(a, b.t())

    # 计算 a 和 b 的范数
    a_norm = torch.norm(a, p=2, dim=1, keepdim=True)
    b_norm = torch.norm(b, p=2, dim=1, keepdim=True)

    # 计算范数的广播
    abmo = torch.matmul(a_norm, b_norm.t())

    # 计算余弦相似度
    cosine_sim = a_dot_b / abmo

    return cosine_sim

class CostTimer(object):
    def __init__(self):
        self.eles = {}
        self._last_new = None
        
    def new(self, name):
        self._last_new = name
        self.eles[name] = [time.time()]
    
    def _formate(self, name, info) -> str:
        return f"{name}:{int(info[1]*1000):3d}ms"
    
    def end(self, name=None, is_print=False):
        if name:
            _info = self.eles[name]
        elif self._last_new :
             _info = self.eles[self._last_new]
        else:
            name, _info = self.eles.popitem()
        
        _info.append(time.time() - _info[0])
        self.eles[name] = _info
        
        _formate_str = self._formate(name, _info)
        if is_print:
            print(_formate_str)
        return _formate_str
    
    def clear(self):
        self.eles = {}
        
    def __str__(self):
        _sss = ""
        for name, info in self.eles.items():
            if len(info)!=2 or name is None:
                continue
            _sss += f"{self._formate(name, info)} "
        if len(_sss) > 0:
            _sss = _sss[:-1]
        return _sss

class YolomixNode(Node):
    def __init__(self):
        super().__init__('yolomix_node')

        # Load pose estimation model
        package_share_dir = get_package_share_directory('yolomix')
        model_path = os.path.join(package_share_dir,"weights/yolov8n-pose.pt")
        # device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"Using device: {self.device}")
        self.model = YOLO(model_path).to(self.device)
        self.model.model.eval()
        
        # Load reID model
        model_name = "osnet_x0_75"
        model_name = "osnet_x0_5"
        model_name = "osnet_x0_25"
        self.reid_fe_extractor = FeatureExtractor(model_name=model_name,device=self.device)
        
        # 注册跟踪器以及reid的encoder
        # self.tracker_config_file = os.path.join(package_share_dir,"config/custom_botsort.yaml")
        self.tracker_config_file = os.path.join(package_share_dir,"config/custom_bytetrack.yaml")
        
        # id特征库，第0维是id号，第1维是时间，2~513维是特征
        self.feature_library = torch.empty(size=(0,514), device=self.device)
        self.next_id = 1  # 初始化ID从1开始
        
        # Create a CvBridge object
        self.cv_bridge = CvBridge()

        # Create a publisher for YoloPersons messages
        self.persons_publisher = self.create_publisher(YoloPersons, '/yolo_persons', 10)
        
        qos_profile_sensor_data_depth1 = qos_profile_sensor_data
        qos_profile_sensor_data_depth1.depth = 1
        
        # Create a subscriber for the image topic
        self.subscription = self.create_subscription(Image,'/image',self.listener_callback, qos_profile_sensor_data_depth1 )

        self.timer = self.create_timer(10.0, self.timer_callback)

        self.max_tracked_id = 0
        self.mapping_table = {}
        
        self.last_process_time = time.time()

        self.clean_subscription = self.create_subscription(String,'/ctrl_cmd',self.clean_feature_library, qos_profile_sensor_data_depth1)
        self.clean_subscription  # 防止变量被垃圾回收
        
        # 超线程流水线加速处理，提高响应速度
        self.detect_deque = deque(maxlen=1)
        self.super_thread = threading.Thread(target=self.reid, daemon=True)
        self.super_thread.start()
        
        # 开启可视化线程
        self.annotated_image_publisher = self.create_publisher(Image, '/annotated_image', 5)
        self.render_thread = threading.Thread(target=self.render, daemon=True)
        self.render_queen = deque(maxlen=1)
        self.render_thread.start()
        
        # 订阅跟踪控制器的反馈信息
        self.follow_info_sub = self.create_subscription(String, '/following_id', self.follow_info_call_back, qos_profile_sensor_data_depth1)
        self.tart_id = -1
        
    def follow_info_call_back(self,msg:String):
        self.tart_id = int(msg.data)
    
    def render(self):
        while True:
            if not self.render_queen:
                time.sleep(0.001)
                continue
            try:
                detection:Results = self.render_queen.popleft()
                
                # 默认是原图
                annotated_image = detection.orig_img
                
                # 如果没有被跟踪，就直接输出原图
                if not detection.boxes.is_track:
                    continue
                
                # reids = []
                # for _id in detection.boxes.id.int().tolist():
                #     reids.append(self.mapping_table[_id])
                # data = copy.copy(detection.boxes.data)
                # data[:, -3]= torch.tensor(reids, dtype=torch.float32)
                # detection.boxes.data = data
                
                # ==========================================================================================================
                color_mode = "class"
                img = None
                names = detection.names
                conf=True
                labels=True
                is_obb = detection.obb is not None
                pred_boxes, show_boxes = detection.obb if is_obb else detection.boxes, True
                pred_probs, show_probs = detection.probs, True
                annotator = Annotator(
                    copy.deepcopy(detection.orig_img if img is None else img),
                    line_width=1,
                    font_size=8,
                    pil= False or (pred_probs is not None and show_probs),  # Classify tasks default to pil=True
                    example=names,
                )

                # Plot Detect results
                if pred_boxes is not None and show_boxes:
                    for i, d in enumerate(reversed(pred_boxes)):
                        c, conf, track_id = int(d.cls), float(d.conf) if conf else None, None if d.id is None else int(d.id.item())
                        
                        reid = track_id
                        if track_id:
                            reid = self.mapping_table[reid]
                        
                        # 默认是蓝色
                        color = colors(i if color_mode == "instance" else c, True)
                        # 如果是被reid的任务，颜色改为橙色
                        if reid != track_id:
                            color = (72,229,227)
                        
                        # 被跟踪的任务，优先级最高
                        if reid == self.tart_id:
                            color = (0, 0, 255)
                        
                        # name = ("" if reid is None else f"reid:{reid} tid:{track_id}") + names[c]
                        name = ("" if reid is None else f"reid:{reid} tid:{track_id}")
                        label = (f"{name} {conf:.2f}" if conf else name) if labels else None
                        box = d.xyxyxyxy.reshape(-1, 4, 2).squeeze() if is_obb else d.xyxy.squeeze()
                        annotator.box_label(
                            box,
                            label,
                            color=color,
                            rotated=is_obb,
                        )

                # Plot Classify results
                if pred_probs is not None and show_probs:
                    text = ",\n".join(f"{names[j] if names else j} {pred_probs.data[j]:.2f}" for j in pred_probs.top5)
                    x = round(detection.orig_shape[0] * 0.03)
                    annotator.text([x, x], text, txt_color=(255, 255, 255))  # TODO: allow setting colors

                # Plot Pose results
                if detection.keypoints is not None:
                    for i, k in enumerate(reversed(detection.keypoints.data)):
                        annotator.kpts(
                            k,
                            detection.orig_shape,
                            radius=3,
                            kpt_line=True,
                            kpt_color=colors(i, True) if color_mode == "instance" else None,
                        )
                
                # ==========================================================================================================
                annotated_image =  annotator.result()
                
            except Exception as e:
                self.get_logger().error(f"{e}")
            finally:
                self.annotated_image_publisher.publish(self.cv_bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8'))
        
    def reid(self):
        cost_timer = CostTimer()
        class NoPersonError(Exception):
            pass
        
        while True:
            if not self.detect_deque:
                time.sleep(0.001)
                continue
            try:
                cost_timer.new("self.reid")
                # 获取tracker的结果
                detection:Results = self.detect_deque.popleft()
                cv_image = detection.orig_img
                yolo_persons_msg = YoloPersons()
                yolo_persons_msg.header = Header()
                yolo_persons_msg.header.stamp = self.get_clock().now().to_msg()
                yolo_persons_msg.header.frame_id = "yolomix"
                yolo_persons_msg.image = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                
                # 没有跟踪到或者没有检测到人的情况下，也发布消息，只是没有person
                if not detection.boxes.is_track:
                    raise NoPersonError('no person detected or tracked')
                
                # 存在被跟踪的对象
                boxes = detection.boxes.xywh.tolist()
                track_ids = detection.boxes.id.int().tolist()
                keypoints = detection.keypoints.data.tolist()
                
                roi_images = []
                yolo_ids = []  # yolo_track分配的id
                
                # 遍历所有检测到的对象结果
                for box, track_id, keypoint in zip(boxes, track_ids, keypoints):

                    x_center, y_center, width, height = box

                    yolo_ids.append(track_id)
                    roi_image = get_roi(cv_image, box)
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
        
                if len(roi_images) == 0:
                    raise Exception("no valid person")
                
                cost_timer.new("self.reid_fe_extractor")
                reid_features = self.reid_fe_extractor(roi_images)
                cost_timer.end()
                
                cost_timer.new("self.match_and_assign_id")
                self.match_and_assign_id(reid_features, yolo_ids)
                cost_timer.end()
                
                self.max_tracked_id = max(yolo_ids)   # 更新上一帧中最大的tracked_id

                for person in yolo_persons_msg.persons:
                    person.id = self.mapping_table[person.id]
            except NoPersonError:
                pass
            except Exception as e:
                self.get_logger().error(f"{e}")
                raise e
            finally:
                # 交给渲染函数渲染可视化结果
                self.render_queen.append(detection)
                # 发布检测的消息
                self.persons_publisher.publish(yolo_persons_msg)
                cost_timer.end("self.reid")
                # self.get_logger().info(f'{cost_timer}')
            
    def timer_callback(self):
         # 获取当前时间
        current_time = time.time()
        # 找到时间差超过30秒的条目索引
        time_diff = current_time - self.feature_library[:, 1]
        valid_indices = torch.where(time_diff < 10)[0]
        # 过滤掉时间差超过30秒的条目
        self.feature_library = self.feature_library[valid_indices]
    
    def clean_feature_library(self, msg):
        if msg.data != 'clean_reid_fetures_lib':
            return
        self.feature_library = torch.empty(size=(0,514), device=self.device)
        self.get_logger().warn('clean the feature library')

    def cosine_similarity(self, vector_a, vector_b):
        dot_product = np.dot(vector_a, vector_b)
        norm_a = np.linalg.norm(vector_a)
        norm_b = np.linalg.norm(vector_b)
        return dot_product / (norm_a * norm_b)

    
    def match_and_assign_id_new(self, reid_features, yolo_ids, threshold=0.75):
        lib_ids = self.feature_library[:, 0].cpu().numpy().astype(int)
        cos_sim = cosine_sim_cal(reid_features, self.feature_library[:, 2:])   # n×N  
        best_match_idx = torch.argmax(cos_sim, dim=1).cpu().numpy().astype(int).tolist() if cos_sim.shape[1] !=0 else None   # n×1  当前特征跟库里的第几个最匹配
        # 当前track_id最匹配库中的track_id
        best_match_ids = lib_ids[best_match_idx].tolist() if best_match_idx is not None else None
        
        current_time = time.time()
        _new_reids = [k for k, v in self.mapping_table.items() if k!=v]
        for i, track_id, feature in zip(range(len(reid_features)),yolo_ids , reid_features):
            
            # 默认值是track_id
            reid = self.mapping_table.get(track_id, track_id)
            # 库中最匹配的track_id以及相似度
            best_match_lib_track_id = best_match_ids[i] if best_match_ids is not None else None
            best_match_cos_sim = cos_sim[i,best_match_idx[i]] if best_match_idx is not None else None
            
            # 1、这个id 没有被reid过，而且不在库中
            cond1 = track_id not in _new_reids and track_id not in lib_ids
            # 2、当前库中存在匹配的id
            cond2 = best_match_lib_track_id is not None
            # 3、相似度大于阈值
            cond3 = best_match_cos_sim and best_match_cos_sim > threshold
            # 4、这个匹配的id，没有被别人reid，也不在当前帧的id列表中，防止重复
            cond4 = best_match_lib_track_id not in _new_reids and best_match_lib_track_id not in yolo_ids
            # 满足所有条件，那么进行reid映射
            if cond1 and cond2 and cond3 and cond4:
                reid = best_match_lib_track_id
                _new_reids.append(reid)
            
            if cond1:
                self.get_logger().info(f"new {track_id} best_matched_id:{best_match_lib_track_id} cos_sim:{best_match_cos_sim}" )
            
            self.mapping_table[track_id] = reid
            new_entry = torch.cat((torch.tensor([reid, current_time], device=self.device), feature))
            self.feature_library = torch.cat((self.feature_library, torch.unsqueeze(new_entry, dim=0)), dim=0)
        
    
    def match_and_assign_id(self, reid_features, yolo_ids, threshold=0.75):
        '''
        看当前对象是否已经被track，如果已经被track，不重复reid
        每个对象只进行一次reid
        看当前已经出现的id有哪些，即用一个变量记录上一帧中的max_tracked_id
        如果给当前对象分配的track_id < max_tracked_id，则说明当前对象已经被track，不用被reid
        建立一个映射表，从track_id到reid_library中的id
        '''

        # 检查库是否为空
        if self.feature_library.shape[0] == 0:
            for idx, feature in enumerate(reid_features):
                stored_id = yolo_ids[idx]
                self.mapping_table[yolo_ids[idx]] = stored_id
                # stored_id就是tracker分配的id

                self.get_logger().info(f'the feature library is empty, assign the tracker id: {stored_id}')

                # 获取当前时间戳作为存入时间
                current_time = time.time()
                # 将ID、时间和特征组合为一行
                new_entry = torch.cat((torch.tensor([stored_id, current_time], device=self.device), feature))
                # 将新的条目添加到 feature_library
                self.feature_library = torch.cat((self.feature_library, torch.unsqueeze(new_entry, dim=0)), dim=0)
            return

        # reid_features  n×512      stored_lib    N×512     T:512×N
        stored_ids = self.feature_library[:, 0].cpu().numpy().astype(int).tolist()      # 取出库中已存在的id
        cos_sim = cosine_sim_cal(reid_features, self.feature_library[:, 2:])   # n×N  
        best_match_idx = torch.argmax(cos_sim, dim=1).cpu().numpy().tolist()              # n×1  当前特征跟库里的第几个最匹配

        # 当前帧已经被分配的id
        assigned_id = set()
        for idx, feature in enumerate(reid_features):
            if yolo_ids[idx] <= self.max_tracked_id:  # 当前对象已经被track，不用进行reid，直接压进特征库即可
                current_time = time.time()


                # stored_id = self.mapping_table[yolo_ids[idx]]
                stored_id = self.mapping_table.get(yolo_ids[idx], -1)
                if stored_id == -1:
                    self.get_logger().error("key Error with stored_id = self.mapping_table.get(yolo_ids[idx], -1)")
                    continue

                assigned_id.add(stored_id)
                new_entry = torch.cat((torch.tensor([stored_id, current_time], device=self.device), feature))

                # 将新的条目添加到 feature_library
                self.feature_library = torch.cat((self.feature_library, torch.unsqueeze(new_entry, dim=0)), dim=0)
            else:                                           # 当前是新的没有被track的对象，第一次需要reid
                best_match = best_match_idx[idx]
                # stored_id = stored_ids[best_match]          # 与当前特征最匹配的库里的特征对应的id
                best_match_id_library = stored_ids[best_match]          # 与当前特征最匹配的库里的特征对应的id
                similarity = cos_sim[idx, best_match]  # 当前特征与最匹配的库里的特征的相似度

                stored_id = yolo_ids[idx]
                self.get_logger().info(f'a new person appear, the best_match_id is {best_match_id_library}   similarity is {similarity}')
                if similarity < threshold or best_match_id_library in assigned_id:
                    if best_match_id_library in assigned_id:
                        self.get_logger().info(f'The id {best_match_id_library} already exists in this frame, so assign the tracker id: {stored_id}')
                    else:
                        self.get_logger().info(f'similarity is too low, so assign the tracker id: {stored_id}')
                else:
                    stored_id = best_match_id_library
                    self.get_logger().info(f'reid---  assign the person the best_match_id_library  {best_match_id_library}')

                assigned_id.add(stored_id)

                self.mapping_table[yolo_ids[idx]] = stored_id

                # 获取当前时间戳作为存入时间
                current_time = time.time()
                # 将ID、时间和特征组合为一行
                new_entry = torch.cat((torch.tensor([stored_id, current_time], device=self.device), feature))
                # 将新的条目添加到 feature_library
                self.feature_library = torch.cat((self.feature_library, torch.unsqueeze(new_entry, dim=0)), dim=0)
    
    def listener_callback(self, msg:Image):
        
        # Convert ROS Image message to OpenCV image
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results:List[Results] = self.model.track(cv_image, persist=True, conf=0.65,  tracker=self.tracker_config_file, verbose=False)
        detect_result = results[0]
        
        # 如果没有任何对象，就清空跟踪器
        if len(detect_result.boxes) == 0 and hasattr(self.model.predictor, "trackers"):
            count = BaseTrack._count
            self.model.predictor.trackers[0].reset()
            BaseTrack._count = count
        
        self.detect_deque.append(detect_result)
        

def main(args=None):
    rclpy.init(args=args)
    node = YolomixNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
