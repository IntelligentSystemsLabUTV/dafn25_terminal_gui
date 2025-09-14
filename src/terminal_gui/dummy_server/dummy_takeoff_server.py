import rclpy

from rclpy.node import Node

from rclpy.action import ActionServer

from example_interfaces.action import Fibonacci  # esempio action
 
class DummyTakeoffActionServer(Node):

    def __init__(self):

        super().__init__('dummy_takeoff_action_server')

        self._action_server = ActionServer(

            self,

            Fibonacci,

            'takeoff',

            self.execute_callback

        )

        self.get_logger().info('Dummy Takeoff Action Server avviato!')
 
    def execute_callback(self, goal_handle):

        self.get_logger().info('Goal Takeoff ricevuto!')

        goal_handle.succeed()

        result = Fibonacci.Result()

        result.sequence = [0, 1, 2]

        return result
 
def main(args=None):

    rclpy.init(args=args)

    node = DummyTakeoffActionServer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
 
if __name__ == '__main__':

    main()
 