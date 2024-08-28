import rclpy
import rclpy.clock
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import rclpy.time
import rclpy.timer
from dataclasses import dataclass
from copy import deepcopy
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PoseStamped, PointStamped
from geometry_msgs.msg import Pose
from tf2_geometry_msgs import do_transform_pose
import tf2_geometry_msgs
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster
import tf2_ros
import tf_transformations

class FastlioOdomcovert(Node):
    def __init__(self):
        super().__init__(node_name='fastlio_odomcovert')
        self.odom_publisher = self.create_publisher(Odometry, '/covert_odomtry', 10)
        self.odom_subscription = self.create_subscription(Odometry, '/fastlio_odomtry', self.listener_callback, 10)
        
        # 创建Buffer和TransformListener对象
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = TransformBroadcaster(self)

    
    def listener_callback(self, msg:Odometry): 
        try:
            # 初始状态时，livox_frame和camera_init坐标系重合
            # 所以base_footprint坐标系相对于livox_frame坐标系的坐标就是相对于camera_init坐标系的坐标
            
            # 第一步，求初始状态时，base_footprint在camera_init坐标系下的pose
            # 求base_footprint在livox_frame坐标系下的坐标
            # 先求base_footprint在base_footprint坐标系下的坐标（原点）
            # 再通过tf变换，转化成base_footprint在livox_frame坐标系下的坐标

            trans1: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame = "livox_frame",
                source_frame = "base_footprint",
                time = rclpy.time.Time())
            basefootprint_originpose = self.create_origin_pose() 
            basefootprint_at_livox_pose = tf2_geometry_msgs.do_transform_pose(basefootprint_originpose, trans1)
            base_footprint_init_pose = basefootprint_at_livox_pose


            # 第二步，求当前base_footprint在camera_init坐标系下的pose
            # 从body(livox_frame)到camrea_init的变换
            # body_id          ‘livox_frame’

            camera_init_id = msg.header.frame_id
            body_id = msg.child_frame_id

            # 从source_frame到target_frame的变换
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame = camera_init_id,
                source_frame = body_id,
                time=rclpy.time.Time())
            # 这里得到的是，当前的base_footprint在camear_init坐标系下的坐标
            base_footprint_current_pose = tf2_geometry_msgs.do_transform_pose(basefootprint_at_livox_pose, trans)
            
            odom_msg = self.get_pose_diff_odom_msg(pose1=base_footprint_init_pose,pose2=base_footprint_current_pose, frame_id="odom", child_frame_id="base_footprint")
            # 发布变换后的Odometry消息
            self.odom_publisher.publish(odom_msg)
            self.publish_tf_from_odom(odom_msg)
            
        except Exception as e:
            self.get_logger().error(f"Could not get transform: {e}")
    
    
    # 返回原点pose
    def create_origin_pose(self):
        # 创建一个 Pose 对象
        origin_pose = Pose()
        
        # 设置位置为原点 (0, 0, 0)
        origin_pose.position.x = 0.0
        origin_pose.position.y = 0.0
        origin_pose.position.z = 0.0
        
        # 设置方向为单位四元数 (0, 0, 0, 1)
        origin_pose.orientation.x = 0.0
        origin_pose.orientation.y = 0.0
        origin_pose.orientation.z = 0.0
        origin_pose.orientation.w = 1.0
        
        return origin_pose

    def get_pose_diff_odom_msg(self, pose1: Pose, pose2: Pose, frame_id:str, child_frame_id:str) -> Odometry:
        # 计算平移量
        delta_x = pose2.position.x - pose1.position.x
        delta_y = pose2.position.y - pose1.position.y
        delta_z = pose2.position.z - pose1.position.z

        # 计算旋转量（四元数 -> 欧拉角 -> 差值 -> 四元数）
        orientation1 = euler_from_quaternion([
            pose1.orientation.x,
            pose1.orientation.y,
            pose1.orientation.z,
            pose1.orientation.w
        ])
        orientation2 = euler_from_quaternion([
            pose2.orientation.x,
            pose2.orientation.y,
            pose2.orientation.z,
            pose2.orientation.w
        ])
        delta_yaw = orientation2[2] - orientation1[2]  # 计算偏航角的差值

        delta_orientation = quaternion_from_euler(0, 0, delta_yaw)

        # 构造Odometry消息
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = frame_id
        odom.child_frame_id = child_frame_id

        # 设置变换后的位置和朝向
        odom.pose.pose.position.x = delta_x
        odom.pose.pose.position.y = delta_y
        odom.pose.pose.position.z = delta_z
        odom.pose.pose.orientation = Quaternion(
            x=delta_orientation[0],
            y=delta_orientation[1],
            z=delta_orientation[2],
            w=delta_orientation[3]
        )

        # 速度可以根据需求计算，这里设为零
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0

        return odom
        # 发布Odometry消息
    

    def publish_tf_from_odom(self, odom: Odometry):
        try:
            # 创建TransformStamped消息
            transform = TransformStamped()

            # 填充TransformStamped的头信息
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = odom.header.frame_id  # 通常是"odom"
            transform.child_frame_id = odom.child_frame_id  # 通常是"base_link"或其他子帧

            # 填充平移和旋转信息
            transform.transform.translation.x = odom.pose.pose.position.x
            transform.transform.translation.y = odom.pose.pose.position.y
            transform.transform.translation.z = odom.pose.pose.position.z

            transform.transform.rotation = odom.pose.pose.orientation

            # 发布TF变换
            self.tf_broadcaster.sendTransform(transform)
        except Exception as e:
            self.get_logger().error(f"Error {e}")

def main(args=None):
    rclpy.init(args=args)
    fastlio_odomcovert = FastlioOdomcovert()
    rclpy.spin(fastlio_odomcovert)
    fastlio_odomcovert.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
