import rclpy

from rclpy.node import Node

from rclpy.action import ActionServer

from example_interfaces.action import Fibonacci  # usa la action che vuoi simulare
 
class DummyArmActionServer(Node):

    def __init__(self):

        super().__init__('dummy_arm_action_server')

        self._action_server = ActionServer(

            self,

            Fibonacci,  # sostituisci con la tua action

            'arm',

            self.execute_callback

        )

        self.get_logger().info('Dummy Arm Action Server avviato!')
 
    def execute_callback(self, goal_handle):

        self.get_logger().info('Goal ricevuto!')

        feedback_msg = Fibonacci.Feedback()

        feedback_msg.sequence = [0, 1, 2]  # esempio

        goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        result = Fibonacci.Result()

        result.sequence = [0, 1, 2, 3]  # esempio

        return result
 
def main(args=None):

    rclpy.init(args=args)

    node = DummyArmActionServer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()
 
if __name__ == '__main__':

    main()
 