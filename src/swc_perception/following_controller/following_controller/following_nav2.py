import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from yolomix_msgs.msg import YoloPerson, YoloPersons
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
from cv_bridge import CvBridge
import cv2
import time
from geometry_msgs.msg import PointStamped, PoseStamped

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import tf2_ros
import numpy as np
import tf_transformations
import tf2_geometry_msgs
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class DetectionNode(Node):
    def __init__(self):
        super().__init__('following_controller_node')

        '''
          data: [352.04623,   0.     , 312.17749,
           0.     , 351.6016 , 233.29058,
           0.     ,   0.     ,   1.     ]
        '''
        # Camera intrinsics
        self.fx = 352.04623
        self.fy = 351.6016
        self.cx = 312.17749
        self.cy = 233.29058

        # 摄像头的水平视场角
        self.fov_x = 1.48   
        self.fov_y = 1.20


        self.max_vx = 1.3
        self.min_vx = 0.2
        self.max_va = 0.3
        self.gain_vx = 0.8
        self.gain_va = 0.5
        self.person_gap = 1.5                          # 跟随过程中保持的距离
        self.angle_threshold = math.pi / 4         # 小于这个角度，先不前进，只转圈
        self.dleta_theta = 8                       # 摄像头安装的位置到轮椅中心的角度偏差（角度制）


        self.state = 'initing'          # 当前状态，包括  initing  following  lost
        self.track_id = -1              # 当前跟随对象的id

        self.latest_laser_scan = None
        self.latest_yolo_persons = None
        self.last_lost_time = None
        # Initialize CvBridge
        self.cv_bridge = CvBridge()

        # Create subscribers for the scan and yolo_persons topics
        self.scan_subscriber = Subscriber(self, LaserScan, '/scan_s2_raw', qos_profile=QoSProfile(depth=10))
        self.yolo_subscriber = Subscriber(self, YoloPersons, '/yolo_persons', qos_profile=QoSProfile(depth=10))
        # Publisher for cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.visual_publisher = self.create_publisher(Image, '/following_visualization', 10)
        self.following_id_publisher = self.create_publisher(String, 'following_id', 10)
        
        # nav目标发布器
        self.navigator = BasicNavigator()
        self.goal_publisher = self.create_publisher(PoseStamped, 'goal_pose', 1)
        self.goal_publisher = self.create_publisher(PoseStamped, 'move_base/cancel', 1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


        # Synchronize the subscribers
        self.ts = ApproximateTimeSynchronizer(
            [self.scan_subscriber, self.yolo_subscriber], 
            queue_size=10, 
            slop=0.2
        )
        self.ts.registerCallback(self.sync_callback)

    
    def sync_callback(self, scan_msg: LaserScan, yolo_persons_msg: YoloPersons):
        # Save the latest messages
        self.latest_laser_scan = scan_msg
        self.latest_yolo_persons = yolo_persons_msg

        self.publish_following_id()
        # self.Visualization()

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
            left_eye = keypoints[1]  # left eye is index 1
            left_wrist = keypoints[9]    # left wrist is index 9

            if left_eye.x > 0 and left_wrist.x > 0:  # Ensure that keypoints have at least left shoulder and wrist
                # Compare y-coordinates to check if wrist is higher than shoulder
                if left_wrist.y < left_eye.y:
                    self.track_id = person.id
                    self.state = 'following'
                    self.get_logger().info(f"Person ID {person.id}: Left wrist is higher than left shoulder.")

    # 将举左手的人作为跟随目标
    def find_lost_target(self):
        if self.latest_yolo_persons is None or self.latest_laser_scan is None:
            return
        
        # 超过10s没有重新找到跟随对象，重新初始化寻找跟随目标
        if time.time() - self.last_lost_time > 5:
            self.track_id = -1
            self.state = 'initing'
            return
        
        for person in self.latest_yolo_persons.persons:
            if person.id == self.track_id:
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
            self.last_lost_time = time.time()
            self.state = 'lost'
            self.Stop()
            self.get_logger().info("Lost target while following...")
            return
        
        # 判断目标是否举起右手，取消跟随
        keypoints = target_person.keypoints
        right_eye = keypoints[2]  # right eye is index 2
        right_wrist = keypoints[10]    # right wrist is index 10
        if right_wrist.y < right_eye.y:
            self.track_id = -1
            self.state = 'initing'
            self.get_logger().info("Cancel following .......")
            self.Stop()
            return
        
        self.cal_goal(target_person)

        
    def cal_goal(self, target_person):

        lidar_person_dis = self.get_target_depth(target_person)
        if lidar_person_dis < 0:
            return None

        center_x, center_y = (target_person.center.x, target_person.center.y)

        # 计算目标行人相对于相机的角度     center_angle_x
        cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_yolo_persons.image)
        image_height, image_width = cv_image.shape[:2]
        angle_per_pixel_x = self.fov_x / image_width
        # 左为正方向
        center_angle_x = -(center_x - image_width / 2) * angle_per_pixel_x

        # 目标行人中心在相机坐标系下的坐标
        X_camera = (center_x - self.cx) * lidar_person_dis / self.fx   # x是水平方向，往右边为正方向
        Y_camera = (center_y - self.cy) * lidar_person_dis / self.fy   # y是垂直方向，往下为正方向
        Z_camera = lidar_person_dis * np.cos(center_angle_x)           # 相机正前方为z的正方向

        
        # 轮椅与目标行人要保持一定距离，计算x和y的偏移量
        # offset_x = self.person_gap * np.sin(center_angle_x)    # 如果角度为正（目标在左），偏移量为正，实际要减去一定的偏移量

        # 计算目标位置，减去偏移量，使机器人与目标行人保持一定距离
        Z_camera = max(0.0, Z_camera - self.person_gap)
        X_camera = Z_camera * np.tan(center_angle_x)

        # 目标点相对于相机坐标系：
        # 往前Z_camera，往右X_camear，角度为   center_angle_x

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'astra_link'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = Z_camera
        pose_stamped.pose.position.y = X_camera

        # 将旋转角度转换为quaternion，单位为弧度值
        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, center_angle_x)
        
        # 填充四元数数据
        pose_stamped.pose.orientation.x = quaternion[0]
        pose_stamped.pose.orientation.y = quaternion[1]
        pose_stamped.pose.orientation.z = quaternion[2]
        pose_stamped.pose.orientation.w = quaternion[3]

        try:
            # 等待变换变得可用
            self.tf_buffer.can_transform('map', 'astra_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            transform = self.tf_buffer.lookup_transform(
                'map',  # 目标 frame
                'astra_link',  # 源 frame
                rclpy.time.Time()  # 获取最新的变换
            )

            # 将 pose_stamped 从 base_link 变换到 map
            transformed_pose_stamped = tf2_geometry_msgs.do_transform_pose_stamped(pose_stamped, transform)

            # 发布变换后的 PoseStamped
            # self.pose_publisher.publish(transformed_pose)
            self.navigator.goToPose(transformed_pose_stamped)

            self.get_logger().info(f'Published transformed PoseStamped to map frame.x:{Z_camera} y:{X_camera}')
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'Failed to transform pose: {str(e)}')


    # 获取目标行人的距离
    def get_target_depth(self, target_person):
                
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
            self.get_logger().warn("No depth data in the specified angle range")
            return -1

        depth = min(relevant_distances)
        return depth


    def Stop(self):
        self.navigator.cancelTask()

    def publish_following_id(self):
        msg = String()
        msg.data = str(self.track_id)
        self.following_id_publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
