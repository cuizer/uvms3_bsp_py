import rclpy
from rclpy.node import Node

from uvms_msg_pkg.msg import HalInertialnavi


class HalInertialnaviNode(Node):
    def __init__(self):
        super().__init__('hal_inertialnavi_node')

        # 创建发布器
        self.publisher_ = self.create_publisher(
            HalInertialnavi,
            '/hal/inertialnavi',
            10
        )

        # 50 Hz 发布
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info('hal_inertialnavi_node started.')

        # 示例数据计数器
        self.timestamp_ms = 0

    def timer_callback(self):
        msg = HalInertialnavi()

        # 这里先填测试数据，后面再替换成真实惯导数据
        self.timestamp_ms += 100
        msg.timestamp_ms = self.timestamp_ms

        msg.yaw = 10.5
        msg.pitch = 1.2
        msg.roll = -0.8

        msg.latitude = 31.2304
        msg.longitude = 121.4737

        msg.velocity_east = 0.5
        msg.velocity_north = 1.1
        msg.velocity_up = -0.1

        self.publisher_.publish(msg)

        self.get_logger().info(
            f'Published: ts={msg.timestamp_ms}, '
            f'yaw={msg.yaw:.2f}, pitch={msg.pitch:.2f}, roll={msg.roll:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = HalInertialnaviNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
