import rclpy

from rclpy.node import Node

from example_interfaces.srv import Trigger  # esempio service
 
class DummyServices(Node):

    def __init__(self):

        super().__init__('dummy_services')

        # service per abilitare componente

        self.enable_srv = self.create_service(Trigger, 'enable_component', self.enable_callback)

        # service per resettare componente

        self.reset_srv = self.create_service(Trigger, 'reset_component', self.reset_callback)

        self.get_logger().info('Dummy Services avviati!')
 
    def enable_callback(self, request, response):

        self.get_logger().info('enable_component chiamato')

        response.success = True

        response.message = 'Componente abilitato'

        return response
 
    def reset_callback(self, request, response):

        self.get_logger().info('reset_component chiamato')

        response.success = True

        response.message = 'Componente resettato'

        return response
 
def main(args=None):

    rclpy.init(args=args)

    node = DummyServices()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
 
if __name__ == '__main__':

    main()
 