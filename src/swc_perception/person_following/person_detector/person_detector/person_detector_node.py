import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from person_tracking_msgs.msg import PersonInfo, PersonsInfo
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from .sort import Sort
from ament_index_python.packages import get_package_share_directory
import os

class PersonDetectorNode(Node):
    def __init__(self, weights_path, cfg_path, names_path):
        super().__init__('person_detector_node')
        self.publisher_ = self.create_publisher(PersonsInfo, 'persons_info', 10)
        self.subscription = self.create_subscription(Image, '/image', self.image_callback, 10)
        self.bridge = CvBridge()

        # 加载预训练的YOLOv3模型和配置文件
        self.net = cv2.dnn.readNet(weights_path, cfg_path)
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]

        # 加载类名
        with open(names_path, "r") as f:
            self.classes = [line.strip() for line in f.readlines()]

        # 初始化SORT跟踪器
        self.tracker = Sort()

    def detect(self, image):
        height, width = image.shape[:2]

        # 将图片转换为blob
        blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        boxes = []
        confidences = []
        class_ids = []

        # 处理检测到的物体
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5 and self.classes[class_id] == "person":  # 只检测行人
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # 使用NMS（非极大值抑制）删除冗余的框
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        detections = []
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                detections.append([x, y, x + w, y + h, confidences[i]])

        return detections

    def image_callback(self, msg):
        # self.get_logger().info("receive a image")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        detections = self.detect(cv_image)
        tracks = self.tracker.update(np.array(detections))

        persons_msg = PersonsInfo()
        persons_list = []

        for track in tracks:
            # x1, y1, x2, y2, track_id = track.astype(int)
            x1, y1, x2, y2, track_id = [int(value) for value in track]
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            width = x2 - x1
            height = y2 - y1

            person_msg = PersonInfo()
            # self.get_logger().info("The type of track_id:" + str(track_id.type()))
            
            person_msg.id = track_id
            center=Point(x=float(center_x), y=float(center_y), z=0.0)
            person_msg.center = center
            person_msg.width = float(width)
            person_msg.height = float(height)

            persons_list.append(person_msg)

        

        persons_msg.persons = persons_list


        self.publisher_.publish(persons_msg)
        if len(persons_list) != 0:
            self.get_logger().info("There is person and publish the info of the person")

def main(args=None):
    rclpy.init(args=args)
    package_share_directory = get_package_share_directory('person_detector')


    weights_path = "data/yolov3.weights"
    cfg_path = "data/yolov3.cfg"
    names_path = "data/coco.names"

    weights_path = os.path.join(package_share_directory, weights_path)
    cfg_path = os.path.join(package_share_directory, cfg_path)
    names_path = os.path.join(package_share_directory, names_path)



    person_detector_node = PersonDetectorNode(weights_path, cfg_path, names_path)
    rclpy.spin(person_detector_node)
    person_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
