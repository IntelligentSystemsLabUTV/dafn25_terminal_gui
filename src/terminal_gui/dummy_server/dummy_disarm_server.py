import rclpy

from rclpy.node import Node

from rclpy.action import ActionServer

from example_interfaces.action import Fibonacci  # esempio action
 
class DummyDisarmActionServer(Node):

    def __init__(self):

        super().__init__('dummy_disarm_action_server')

        self._action_server = ActionServer(

            self,

            Fibonacci,  # sostituisci con la tua action

            'disarm',

            self.execute_callback

        )

        self.get_logger().info('Dummy Disarm Action Server avviato!')
 
    def execute_callback(self, goal_handle):

        self.get_logger().info('Goal Disarm ricevuto!')

        goal_handle.succeed()

        result = Fibonacci.Result()

        result.sequence = [0, 1, 2]

        return result
 
def main(args=None):

    rclpy.init(args=args)

    node = DummyDisarmActionServer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
 
if __name__ == '__main__':

    main()

 