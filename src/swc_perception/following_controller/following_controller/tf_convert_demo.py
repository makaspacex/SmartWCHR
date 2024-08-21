import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs  # 注意：需要安装 tf2_geometry_msgs 包
import tf_transformations
import math

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.pose_publisher = self.create_publisher(PoseStamped, 'target_pose', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.publish_transformed_pose)

    def publish_transformed_pose(self):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'astra_link'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = 5.0  # x 增加 5 米
        pose_stamped.pose.position.y = 5.0  # y 增加 5 米

        # 将旋转角度转换为quaternion，单位为弧度值
        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, math.pi)
        
        # 填充四元数数据
        pose_stamped.pose.orientation.x = quaternion[0]
        pose_stamped.pose.orientation.y = quaternion[1]
        pose_stamped.pose.orientation.z = quaternion[2]
        pose_stamped.pose.orientation.w = quaternion[3]

        try:
            # 等待变换变得可用
            self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            transform = self.tf_buffer.lookup_transform(
                'map',  # 目标 frame
                'astra_link',  # 源 frame
                rclpy.time.Time()  # 获取最新的变换
            )

            # 将 pose_stamped 从 base_link 变换到 map
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped.pose, transform)

            # 发布变换后的 PoseStamped
            self.pose_publisher.publish(transformed_pose)
            self.get_logger().info('Published transformed PoseStamped to map frame.')
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'Failed to transform pose: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()
    try:
        rclpy.spin(pose_publisher)
    except KeyboardInterrupt:
        pose_publisher.get_logger().info('Node terminated.')
    finally:
        pose_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()