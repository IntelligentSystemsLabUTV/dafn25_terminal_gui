import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default
from std_srvs.srv import SetBool, Trigger
from dua_hardware_interfaces.action import Arm, Disarm
from dua_aircraft_interfaces.action import Takeoff, Landing
from dua_movement_interfaces.action import Navigate
from simple_actionclient_py.simple_actionclient import Client as SimpleActionClient
from simple_serviceclient_py.simple_serviceclient import Client as SimpleServiceClient
import asyncio
from geometry_msgs.msg import PoseStamped, Point
from dua_common_interfaces.msg import CommandResultStamped


class TerminalNode(Node):

    def __init__(self):
        super().__init__('terminal_node')

        # Dichiarazione dei parametri direttamente nel nodo
        self.declare_parameter('actions.arm', '/test_server_node/arm')
        self.declare_parameter('actions.disarm', '/test_server_node/disarm')
        self.declare_parameter('actions.takeoff', '/test_server_node/takeoff')
        self.declare_parameter('actions.landing', '/test_server_node/landing')
        self.declare_parameter('actions.navigate', '/test_server_node/navigate')


        # Lettura dei parametri
        self.arm_action_name = self.get_parameter('actions.arm').get_parameter_value().string_value
        self.disarm_action_name = self.get_parameter('actions.disarm').get_parameter_value().string_value
        self.takeoff_action_name = self.get_parameter('actions.takeoff').get_parameter_value().string_value
        self.landing_action_name = self.get_parameter('actions.landing').get_parameter_value().string_value
        self.navigate_action_name = self.get_parameter('actions.navigate').get_parameter_value().string_value

        # --- Creazione clients action ---
        self.arm_client = SimpleActionClient(self, Arm, self.arm_action_name)
        self.disarm_client = SimpleActionClient(self, Disarm, self.disarm_action_name)
        self.takeoff_client = SimpleActionClient(self, Takeoff, self.takeoff_action_name)
        self.landing_client = SimpleActionClient(self, Landing, self.landing_action_name)
        self.navigate_client = SimpleActionClient(self, Navigate, self.navigate_action_name)

        # Client per Enable/Disable
        self.enable_service_name = "/test_server_node/enable_component"
        self.enable_service_client = SimpleServiceClient(self, SetBool, self.enable_service_name)

        # Client per Reset
        self.reset_service_name = "/test_server_node/reset_component"
        self.reset_service_client = SimpleServiceClient(self, Trigger, self.reset_service_name)

    # --- Action methods ---

    # --- ARM ---
    async def send_arm_goal(self) -> CommandResultStamped:
        goal_msg = Arm.Goal()
        result_msg = CommandResultStamped()
        try:
            # timeout di 5 secondi per la connessione al server
            result_handle = await asyncio.wait_for(self.arm_client.send_goal(goal_msg), timeout=5.0)

            # manteniamo la logica esistente
            result_msg.result = 0 if result_handle.status == 0 else 1
            result_msg.error_msg = "Accepted" if result_handle.status == 0 else "Denied"

        except asyncio.TimeoutError:
            result_msg.result = 1
            result_msg.error_msg = "Arm failed: action server not available"
        except Exception as e:
            result_msg.result = 2
            result_msg.error_msg = f"Arm failed: {e}"

        return result_msg


    # --- DISARM ---
    async def send_disarm_goal(self) -> CommandResultStamped:
        goal_msg = Disarm.Goal()
        result_msg = CommandResultStamped()
        try:
            result_handle = await asyncio.wait_for(self.disarm_client.send_goal(goal_msg), timeout=5.0)

            # manteniamo la logica esistente
            result_msg.result = 0 if result_handle.status == 0 else 1
            result_msg.error_msg = "Accepted" if result_handle.status == 0 else "Denied"

        except asyncio.TimeoutError:
            result_msg.result = 1
            result_msg.error_msg = "Disarm failed: action server not available"
        except Exception as e:
            result_msg.result = 2
            result_msg.error_msg = f"Disarm failed: {e}"

        return result_msg


    # Flight
    async def send_takeoff_goal(self, altitude: float) -> CommandResultStamped:
        result_msg = CommandResultStamped()
        goal_msg = Takeoff.Goal()
        pose = PoseStamped()
        pose.pose.position.z = altitude
        goal_msg.takeoff_pose = pose

        try:
            # timeout di 5 secondi per la connessione al server
            goal_handle = await asyncio.wait_for(
                self.takeoff_client.send_goal(goal_msg),
                timeout=5.0
            )
            result_response = await goal_handle.get_result_async()
            result = getattr(result_response, "result", None)

            result_msg.result = 0
            result_msg.error_msg = getattr(result, "feedback", f"Takeoff to {altitude} m completed")
        except asyncio.TimeoutError:
            result_msg.result = 1
            result_msg.error_msg = "Takeoff failed: action server not available"
        except Exception as e:
            result_msg.result = 2
            result_msg.error_msg = f"Takeoff failed: {e}"

        return result_msg

    async def send_landing_goal(self, descend: bool = True, min_z: float = 0.0) -> CommandResultStamped:
        result_msg = CommandResultStamped()
        goal_msg = Landing.Goal()
        goal_msg.minimums.point = Point(x=0.0, y=0.0, z=min_z)
        goal_msg.descend = descend

        try:
            goal_handle = await asyncio.wait_for(
                self.landing_client.send_goal(goal_msg),
                timeout=5.0
            )
            result_response = await goal_handle.get_result_async()
            result = getattr(result_response, "result", None)

            result_msg.result = 0
            result_msg.error_msg = getattr(result, "feedback", "Landing completed")
        except asyncio.TimeoutError:
            result_msg.result = 1
            result_msg.error_msg = "Landing failed: action server not available"
        except Exception as e:
            result_msg.result = 2
            result_msg.error_msg = f"Landing failed: {e}"

        return result_msg


    async def send_navigate_goal(self, x: float, y: float, z: float) -> CommandResultStamped:
        result_msg = CommandResultStamped()
        goal_msg = Navigate.Goal()
        goal_msg.target_pose = PoseStamped()
        goal_msg.target_pose.pose.position.x = x
        goal_msg.target_pose.pose.position.y = y
        goal_msg.target_pose.pose.position.z = z

        try:
            goal_handle = await asyncio.wait_for(
                self.navigate_client.send_goal(goal_msg),
                timeout=5.0
            )
            result_response = await goal_handle.get_result_async()
            result = getattr(result_response, "result", None)

            result_msg.result = 0
            result_msg.error_msg = getattr(result, "feedback", f"Navigation to ({x},{y},{z}) completed")
        except asyncio.TimeoutError:
            result_msg.result = 1
            result_msg.error_msg = "Navigation failed: action server not available"
        except Exception as e:
            result_msg.result = 2
            result_msg.error_msg = f"Navigation failed: {e}"

        return result_msg

    # --- Service methods con SimpleServiceClient ---
    # --- ENABLE ---
    async def call_enable_service(self, enable: bool = True) -> CommandResultStamped:
        req = SetBool.Request()
        req.data = enable
        result_msg = CommandResultStamped()
        try:
            response = await asyncio.wait_for(self.enable_service_client.call_async(req), timeout=5.0)
            result_msg.result = 0 if response.success else 1
            result_msg.error_msg = response.message or ""
        except asyncio.TimeoutError:
            result_msg.result = 1
            result_msg.error_msg = "Enable service failed: service not available"
        except Exception as e:
            result_msg.result = 2
            result_msg.error_msg = f"Enable service failed: {e}"
        return result_msg


    # --- RESET ---
    async def call_reset_service(self) -> CommandResultStamped:
        req = Trigger.Request()
        result_msg = CommandResultStamped()
        try:
            response = await asyncio.wait_for(self.reset_service_client.call_async(req), timeout=5.0)
            result_msg.result = 0 if response.success else 1
            result_msg.error_msg = response.message or ""
        except asyncio.TimeoutError:
            result_msg.result = 1
            result_msg.error_msg = "Reset service failed: service not available"
        except Exception as e:
            result_msg.result = 2
            result_msg.error_msg = f"Reset service failed: {e}"
        return result_msg