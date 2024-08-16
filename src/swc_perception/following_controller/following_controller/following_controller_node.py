import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from yolomix_msgs.msg import YoloPerson, YoloPersons
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
import math
from cv_bridge import CvBridge
import cv2


class DetectionNode(Node):
    def __init__(self):
        super().__init__('following_controller_node')

        # Camera intrinsics
        # self.fx = 235.67877
        # self.fy = 236.79534
        # self.cx = 186.26218
        # self.cy = 111.24331
        self.fov_x = 1.19   # 摄像头的水平视场角
        # self.fov_y = 0.94


        self.state = 'initing'          # 当前状态，包括  initing  following  lost
        self.track_id = -1              # 当前跟随对象的id

        self.latest_laser_scan = None
        self.latest_yolo_persons = None
        # Initialize CvBridge
        self.cv_bridge = CvBridge()

        # Create subscribers for the scan and yolo_persons topics
        self.scan_subscriber = Subscriber(self, LaserScan, '/scan_s2_raw', qos_profile=QoSProfile(depth=10))
        self.yolo_subscriber = Subscriber(self, YoloPersons, '/yolo_persons', qos_profile=QoSProfile(depth=10))
        # Publisher for cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.visual_publisher = self.create_publisher(Image, '/following_visualization', 10)

        # Synchronize the subscribers
        self.ts = ApproximateTimeSynchronizer(
            [self.scan_subscriber, self.yolo_subscriber], 
            queue_size=10, 
            slop=0.2
        )
        self.ts.registerCallback(self.sync_callback)

        # Create a timer to process the data
        self.timer = self.create_timer(0.1, self.timer_callback)  # Timer interval is 1 second

    def sync_callback(self, scan_msg: LaserScan, yolo_persons_msg: YoloPersons):
        # Save the latest messages
        self.latest_laser_scan = scan_msg
        self.latest_yolo_persons = yolo_persons_msg

    # 这种方式存在bug，如果画面中只有一个人，这个人走出画面，不会有新的yoloperson消息发布，不更新了
    # 解决方案：让yolomix_node在没有检测到人的情况下也发布yolo_persons消息
    def timer_callback(self):

        self.Visualization()
        if self.state == 'initing':
            self.find_target()
        elif self.state == 'following':
            self.follow_target()
        elif self.state == 'lost':
            self.find_lost_target()
            


    # 将举左手的人作为跟随目标
    def find_target(self):
        if self.latest_yolo_persons is None or self.latest_laser_scan is None:
            return
        
        # Process the data to check if left wrist is higher than left shoulder
        for person in self.latest_yolo_persons.persons:
            keypoints = person.keypoints
            left_shoulder = keypoints[5]  # left shoulder is index 5
            left_wrist = keypoints[9]    # left wrist is index 9

            if left_shoulder.x > 0 and left_wrist.x > 0:  # Ensure that keypoints have at least left shoulder and wrist
                # Compare y-coordinates to check if wrist is higher than shoulder
                if left_wrist.y < left_shoulder.y:
                    self.track_id = person.id
                    self.state = 'following'
                    self.get_logger().info(f"Person ID {person.id}: Left wrist is higher than left shoulder.")

    # 将举左手的人作为跟随目标
    def find_lost_target(self):
        if self.latest_yolo_persons is None or self.latest_laser_scan is None:
            return
        
        for person in self.latest_yolo_persons.persons:
            if person.id == person.id == self.track_id:
                self.state = 'following'


    def follow_target(self):
        if self.latest_yolo_persons is None or self.latest_laser_scan is None:
            return
        
        lost = True
        target_person = YoloPerson()

        if self.latest_laser_scan and self.latest_yolo_persons:
            for person in self.latest_yolo_persons.persons:
                if person.id == self.track_id:
                    lost = False
                    target_person = person

        if lost: 
            self.state = 'lost'
            self.Stop()
            self.get_logger().info("Lost target while following...")
            return
        
        # 判断目标是否举起右手，取消跟随
        keypoints = target_person.keypoints
        right_shoulder = keypoints[6]  # right shoulder is index 5
        right_wrist = keypoints[10]    # right wrist is index 10
        if right_wrist.y < right_shoulder.y:
            self.track_id = -1
            self.state = 'initing'
            self.get_logger().info("Cancel following .......")
            self.Stop()
            return
        
        self.CaculateTwist(target_person)
        

    def CaculateTwist(self, target_person):
        
        center = (target_person.center.x, target_person.center.y)
        bbox_width = target_person.width

        # Get the height and width of the image
        cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_yolo_persons.image)
        image_height, image_width = cv_image.shape[:2]

        angle_per_pixel_x = self.fov_x / image_width

        center_angle_x = -(center[0] - image_width / 2) * angle_per_pixel_x
        half_width_angle = (bbox_width / 2) * angle_per_pixel_x

        angle_min_ = center_angle_x - half_width_angle
        angle_max_ = center_angle_x + half_width_angle

        def normalize_angle(angle):
            while angle < 0.0:
                angle += 2 * math.pi
            while angle >= 2 * math.pi:
                angle -= 2 * math.pi
            return angle

        angle_min_ = normalize_angle(angle_min_)
        angle_max_ = normalize_angle(angle_max_)

        def in_range(angle):
            if angle_min_ > angle_max_:
                return angle >= angle_min_ or angle <= angle_max_
            else:
                return angle_min_ <= angle <= angle_max_

        # Process LaserScan data
        distances = self.latest_laser_scan.ranges
        angles = [self.latest_laser_scan.angle_min + i * self.latest_laser_scan.angle_increment for i in range(len(distances))]

        relevant_distances = [distances[i] for i in range(len(distances)) if in_range(angles[i]) and distances[i] > 0.0]

        if not relevant_distances:
            self.get_logger().warn("No data in the specified angle range")
            return

        depth = min(relevant_distances)
        # self.get_logger().info("depth:  %.2f"%depth)

        # Compute the angle theta
        theta = center_angle_x - (20 / 180 * math.pi)

        # self.get_logger().info("theta:  %.2f"%(theta / math.pi * 180))

        max_vx = 0.8
        min_vx = 0.2
        max_va = 0.3
        gain_vx = 0.8
        gain_va = 0.3
        distance = 2
        angle_threshold = math.pi / 4

        # target_x = Z_camera
        

        va = min(max_va, max(-max_va, theta * gain_va))
        vx = 0.0

        if abs(theta) < angle_threshold:
            vx = (depth - distance) * gain_vx
            if vx < 0:
                vx = 0.0
            else:
                vx = min(max_vx, vx if vx >= min_vx else min_vx)
            # vx = max(0, min(max_vx, vx if vx >= min_vx else min_vx))
            # vx = max(min_vx, min(max_vx, max(0, vx)))
        else:
            self.get_logger().info("Rotation too big, not moving forward")

        # Create and publish velocity command
        twist = Twist()
        twist.linear.x = vx
        twist.angular.z = va
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f"Publish twist vx = {vx},   va = {va}")
    

    def Stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Stop.............")

    def Visualization(self):
        if self.latest_yolo_persons is None:
            return

        # 将ROS图像消息转换为OpenCV图像
        cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_yolo_persons.image, desired_encoding='bgr8')

        for person in self.latest_yolo_persons.persons:
            # 获取边界框坐标和中心点
            center_x, center_y = int(person.center.x), int(person.center.y)
            bbox_width, bbox_height = int(person.width), int(person.height)

            # 计算边界框的坐标
            x_min = int(center_x - bbox_width / 2)
            y_min = int(center_y - bbox_height / 2)
            x_max = int(center_x + bbox_width / 2)
            y_max = int(center_y + bbox_height / 2)

            # 根据ID选择框的颜色
            if person.id == self.track_id:
                color = (0, 0, 255)  # 红色
            else:
                color = (255, 0, 0)  # 蓝色

            # 绘制边界框
            cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), color, 2)

        # 将OpenCV图像转换回ROS图像消息
        output_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

        # 发布修改后的图像
        self.visual_publisher.publish(output_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
