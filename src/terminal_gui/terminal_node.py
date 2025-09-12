import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import SetBool, Trigger
import yaml

# Import messaggi action reali
from dua_hardware_interfaces.action import Arm, Disarm
from dua_aircraft_interfaces.action import Takeoff, Landing
from dua_movement_interfaces.action import Navigate

class TerminalNode(Node):
    def __init__(self):
        super().__init__('terminal_node')
        self.get_logger().info('Nodo terminal_node avviato!')

        # Caricamento parametri YAML
        params_file = "/home/neo/workspace/src/terminal_gui/params/params.yaml"
        with open(params_file, 'r') as f:
            self.params = yaml.safe_load(f)

        # Parametri azioni
        self.arm_action_name = self.params['actions']['arm']
        self.disarm_action_name = self.params['actions']['disarm']
        self.takeoff_action_name = self.params['actions']['takeoff']
        self.landing_action_name = self.params['actions']['landing']
        self.navigate_action_name = self.params['actions']['navigate']

        # Parametri servizi
        self.enable_service_name = self.params['services']['enable_component']
        self.reset_service_name = self.params['services']['reset_component']

        # GUI defaults
        self.slider_defaults = self.params['gui']['slider_default_values']
        self.select_defaults = self.params['gui']['select_default_values']
        self.checkbox_defaults = self.params['gui']['checkbox_default_values']

        # --- Creazione clients Action ---
        self.arm_action_client = ActionClient(self, Arm, self.arm_action_name)
        self.disarm_action_client = ActionClient(self, Disarm, self.disarm_action_name)
        self.takeoff_action_client = ActionClient(self, Takeoff, self.takeoff_action_name)
        self.landing_action_client = ActionClient(self, Landing, self.landing_action_name)
        self.navigate_action_client = ActionClient(self, Navigate, self.navigate_action_name)

        # --- Creazione clients Service ---
        self.enable_service_client = self.create_client(SetBool, self.enable_service_name)
        self.reset_service_client = self.create_client(Trigger, self.reset_service_name)

        self.get_logger().info('Clients action e service creati!')

        # Callback GUI per feedback live
        self.arm_feedback_gui = None
        self.disarm_feedback_gui = None
        self.takeoff_feedback_gui = None
        self.landing_feedback_gui = None
        self.navigate_feedback_gui = None

    # --- Azioni con feedback live ---
    async def send_arm_goal(self, name: str):
        await self.arm_action_client.wait_for_server()
        goal_msg = Arm.Goal(name=name)

        def feedback_cb(feedback_msg):
            if self.arm_feedback_gui:
                asyncio.run_coroutine_threadsafe(
                    self.arm_feedback_gui(feedback_msg.current_step),
                    asyncio.get_event_loop()
                )

        send_goal_future = self.arm_action_client.send_goal_async(
            goal_msg, feedback_callback=feedback_cb
        )
        goal_handle = await send_goal_future
        result = await goal_handle.get_result_async()
        return result.result

    async def send_disarm_goal(self, name: str):
        await self.disarm_action_client.wait_for_server()
        goal_msg = Disarm.Goal(name=name)

        def feedback_cb(feedback_msg):
            if self.disarm_feedback_gui:
                asyncio.run_coroutine_threadsafe(
                    self.disarm_feedback_gui(feedback_msg.current_step),
                    asyncio.get_event_loop()
                )

        send_goal_future = self.disarm_action_client.send_goal_async(
            goal_msg, feedback_callback=feedback_cb
        )
        goal_handle = await send_goal_future
        result = await goal_handle.get_result_async()
        return result.result

    async def send_takeoff_goal(self, altitude: float,
                            throttle: float = 0.5,
                            mode: str = "manual",
                            safety_check: bool = True):
        await self.takeoff_action_client.wait_for_server()
        goal_msg = Takeoff.Goal()
        goal_msg.altitude = altitude
        goal_msg.throttle = throttle
        goal_msg.mode = mode
        goal_msg.safety_check = safety_check

        send_goal_future = self.takeoff_action_client.send_goal_async(
            goal_msg, feedback_callback=lambda fb: asyncio.run_coroutine_threadsafe(
                self.takeoff_feedback_gui(fb.current_altitude), asyncio.get_event_loop()
            ) if self.takeoff_feedback_gui else None
        )
        goal_handle = await send_goal_future
        result = await goal_handle.get_result_async()
        return result.result


    async def send_landing_goal(self):
        await self.landing_action_client.wait_for_server()
        goal_msg = Landing.Goal()

        def feedback_cb(feedback_msg):
            if self.landing_feedback_gui:
                asyncio.run_coroutine_threadsafe(
                    self.landing_feedback_gui(feedback_msg.current_altitude),
                    asyncio.get_event_loop()
                )

        send_goal_future = self.landing_action_client.send_goal_async(
            goal_msg, feedback_callback=feedback_cb
        )
        goal_handle = await send_goal_future
        result = await goal_handle.get_result_async()
        return result.result

    async def send_navigate_goal(self, x: float, y: float, z: float,
                             throttle: float = 0.5,
                             mode: str = "manual",
                             safety_check: bool = True):
        await self.navigate_action_client.wait_for_server()
        goal_msg = Navigate.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.z = z
        goal_msg.throttle = throttle
        goal_msg.mode = mode
        goal_msg.safety_check = safety_check

        send_goal_future = self.navigate_action_client.send_goal_async(
            goal_msg, feedback_callback=lambda fb: asyncio.run_coroutine_threadsafe(
                self.navigate_feedback_gui(fb.current_position), asyncio.get_event_loop()
            ) if self.navigate_feedback_gui else None
        )
        goal_handle = await send_goal_future
        result = await goal_handle.get_result_async()
        return result.result


    # --- Servizi ---
    async def call_enable_service(self, enable: bool = True):
        await self.enable_service_client.wait_for_service()
        req = SetBool.Request()
        req.data = enable
        future = self.enable_service_client.call_async(req)
        result = await future
        return result

    async def call_reset_service(self):
        await self.reset_service_client.wait_for_service()
        req = Trigger.Request()
        future = self.reset_service_client.call_async(req)
        result = await future
        return result
