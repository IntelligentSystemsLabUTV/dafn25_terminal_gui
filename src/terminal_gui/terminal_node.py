import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node

from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.srv import Trigger  # esempio service
from example_interfaces.action import Fibonacci  # esempio action
from ament_index_python.packages import get_package_share_directory
import os
import yaml


# Import delle azioni reali
from dua_interfaces.action import Arm, Disarm, Takeoff, Landing, Navigate

class TerminalNode(Node):
    def __init__(self, enable_logging=True):
        super().__init__('terminal_node')
        self.enable_logging = enable_logging
        self.get_logger().info('Nodo terminal_node avviato!')

        # valori di default per la GUI
        self.slider_defaults = {'throttle': 0.5, 'flight_mode': 'manual'}
        self.checkbox_defaults = {'safety_check': True}
        self.select_defaults = {'mode': 'manual'}

        # Percorso assoluto del file dei parametri (adatta al tuo container)
        params_file = "/home/neo/workspace/src/terminal_gui/params/params.yaml"

        # Carica i parametri YAML
        with open(params_file, 'r') as f:
            self.params = yaml.safe_load(f)

        # --- Dichiarazione parametri base ROS 2 ---

        # Azioni
        self.arm_action_name = self.params['actions']['arm']
        self.disarm_action_name = self.params['actions']['disarm']
        self.takeoff_action_name = self.params['actions']['takeoff']
        self.landing_action_name = self.params['actions']['landing']
        self.navigate_action_name = self.params['actions']['navigate']

        self.declare_parameter('arm_action', self.arm_action_name)
        self.declare_parameter('disarm_action', self.disarm_action_name)
        self.declare_parameter('takeoff_action', self.takeoff_action_name)
        self.declare_parameter('landing_action', self.landing_action_name)
        self.declare_parameter('navigate_action', self.navigate_action_name)

        # Servizi
        self.enable_service_name = self.params['services']['enable_component']
        self.reset_service_name = self.params['services']['reset_component']

        self.declare_parameter('enable_service', self.enable_service_name)
        self.declare_parameter('reset_service', self.reset_service_name)

        # GUI
        self.throttle_default = self.params['gui']['slider_default_values']['throttle']
        self.declare_parameter('throttle_default', self.throttle_default)

        # Creazione dei clients action
        self.arm_action_client = ActionClient(self, Fibonacci, self.arm_action_name)
        self.disarm_action_client = ActionClient(self, Fibonacci, self.disarm_action_name)
        self.takeoff_action_client = ActionClient(self, Fibonacci, self.takeoff_action_name)
        self.landing_action_client = ActionClient(self, Fibonacci, self.landing_action_name)
        self.navigate_action_client = ActionClient(self, Fibonacci, self.navigate_action_name)

        # Creazione dei clients service
        self.enable_service_client = self.create_client(Trigger, self.enable_service_name)
        self.reset_service_client = self.create_client(Trigger, self.reset_service_name)

        # Callback GUI
        self.arm_feedback_gui = None
        self.disarm_feedback_gui = None
        self.takeoff_feedback_gui = None
        self.landing_feedback_gui = None
        self.navigate_feedback_gui = None


        self.get_logger().info('Clients action e service creati!')

    # --- Metodi async per le action ---
    async def send_arm_goal(self, name: str):
        self.arm_client.wait_for_server()
        goal_msg = Arm.Goal()
        goal_msg.name = name
        if self.enable_logging:
            self.get_logger().info(f"[ARM] Invio goal: {name}")
        send_goal_future = self.arm_client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future
        result = await goal_handle.get_result_async()
        if self.arm_feedback_gui:
            await self.arm_feedback_gui(result.result.status)
        return result.result

    async def send_disarm_goal(self, name: str):
        self.disarm_client.wait_for_server()
        goal_msg = Disarm.Goal()
        goal_msg.name = name
        send_goal_future = self.disarm_client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future
        result = await goal_handle.get_result_async()
        if self.disarm_feedback_gui:
            await self.disarm_feedback_gui(result.result.status)
        return result.result

    async def send_takeoff_goal(self, altitude: float):
        self.takeoff_client.wait_for_server()
        goal_msg = Takeoff.Goal()
        goal_msg.altitude = altitude
        send_goal_future = self.takeoff_client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future
        result = await goal_handle.get_result_async()
        if self.takeoff_feedback_gui:
            await self.takeoff_feedback_gui(result.result.success)
        return result.result

    async def send_landing_goal(self):
        self.landing_client.wait_for_server()
        goal_msg = Landing.Goal()
        send_goal_future = self.landing_client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future
        result = await goal_handle.get_result_async()
        if self.landing_feedback_gui:
            await self.landing_feedback_gui(result.result.success)
        return result.result

    async def send_navigate_goal(self, x: float, y: float, z: float):
        self.navigate_client.wait_for_server()
        goal_msg = Navigate.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.z = z
        send_goal_future = self.navigate_client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future
        result = await goal_handle.get_result_async()
        if self.navigate_feedback_gui:
            await self.navigate_feedback_gui((x, y, z))
        return result.result

    # --- Metodi async per i service ---
    async def call_enable_service(self):
        self.enable_service_client.wait_for_service()
        req = Trigger.Request()
        future = self.enable_service_client.call_async(req)
        result = await future
        return result

    async def call_reset_service(self):
        self.reset_service_client.wait_for_service()
        req = Trigger.Request()
        future = self.reset_service_client.call_async(req)
        result = await future
        return result

def main(args=None):
    rclpy.init(args=args)
    node = TerminalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()