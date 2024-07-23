import rclpy
from rclpy.node import Node
from nav2_msgs.msg import ParticleCloud
from sensor_msgs.msg import PointCloud2, PointField
import struct
from rclpy.qos import QoSProfile, ReliabilityPolicy

class ParticleToPoint(Node):
    def __init__(self):
        super().__init__('particle_to_point')
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        self.subscription = self.create_subscription(
            ParticleCloud,
            '/particle_cloud',
            self.particle_cloud_callback,
            qos_profile)
        self.publisher = self.create_publisher(PointCloud2, '/to_point_cloud', 10)

    def particle_cloud_callback(self, msg):
        point_cloud = PointCloud2()
        point_cloud.header = msg.header
        point_cloud.height = 1
        point_cloud.width = len(msg.particles)
        point_cloud.is_dense = True
        point_cloud.is_bigendian = False
        point_cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_cloud.point_step = 12
        point_cloud.row_step = point_cloud.point_step * point_cloud.width

        data = []
        for particle in msg.particles:
            data.append(struct.pack('fff', particle.pose.position.x, particle.pose.position.y, particle.pose.position.z))
        point_cloud.data = b''.join(data)

        self.publisher.publish(point_cloud)

def main(args=None):
    rclpy.init(args=args)
    node = ParticleToPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
