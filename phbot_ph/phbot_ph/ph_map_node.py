import rclpy
from rclpy.node import Node


class PHMapNode(Node):
    def __init__(self):
        super().__init__('ph_map_node')
        self.get_logger().info('Node started')


def main(args=None):
    rclpy.init(args=args)

    node = PHMapNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()