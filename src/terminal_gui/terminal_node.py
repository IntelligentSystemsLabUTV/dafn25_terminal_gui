import rclpy
from rclpy.node import Node

class TerminalNode(Node):
    def __init__(self):
        super().__init__('terminal_node')
        self.get_logger().info('Nodo terminal_node avviato!')

def main(args=None):
    rclpy.init(args=args)
    node = TerminalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()