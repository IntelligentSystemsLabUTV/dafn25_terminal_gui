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
import yaml
from geometry_msgs.msg import PoseStamped, Point
from dua_common_interfaces.msg import CommandResultStamped


class TerminalNode(Node):

    def __init__(self):
        super().__init__('terminal_node')

        # --- Caricamento parametri ---
        params_file = "/home/neo/workspace/src/terminal_gui/params/params.yaml"
        with open(params_file, 'r') as f:
            self.params = yaml.safe_load(f)

        # --- Nomi action e service ---
        self.arm_action_name = self.params['actions']['arm']
        self.disarm_action_name = self.params['actions']['disarm']
        self.takeoff_action_name = self.params['actions']['takeoff']
        self.landing_action_name = self.params['actions']['landing']
        self.navigate_action_name = self.params['actions']['navigate']


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
    async def send_arm_goal(self):
        goal_msg = Arm.Goal()
        result = await self.arm_client.send_goal(goal_msg)
        return result

    async def send_disarm_goal(self):
        goal_msg = Disarm.Goal()
        result = await self.disarm_client.send_goal(goal_msg)
        return result


    async def send_takeoff_goal(self, altitude: float):
        goal_msg = Takeoff.Goal()
        pose = PoseStamped()
        pose.pose.position.z = altitude
        goal_msg.takeoff_pose = pose

        goal_handle = await self.takeoff_client.send_goal(goal_msg)
        #result = await goal_handle.get_result_async()
        # Ottieni il risultato dall'azione
        result_response = await goal_handle.get_result_async()
        result: Takeoff.Result = result_response.result

        return result


    async def send_landing_goal(self, descend: bool = True, min_z: float = 0.0):
        goal_msg = Landing.Goal()
        goal_msg.minimums.point = Point(x=0.0, y=0.0, z=min_z)
        goal_msg.descend = descend
        goal_handle = await self.landing_client.send_goal(goal_msg)
        result_response = await goal_handle.get_result_async()
        result: Landing.Result = result_response.result
        return result

    async def send_navigate_goal(self, x: float, y: float, z: float):
        goal_msg = Navigate.Goal()
        goal_msg.target_pose = PoseStamped()
        goal_msg.target_pose.pose.position.x = x
        goal_msg.target_pose.pose.position.y = y
        goal_msg.target_pose.pose.position.z = z
        result = await self.navigate_client.send_goal(goal_msg)
        return result

    # --- Service methods con SimpleServiceClient ---
    async def call_enable_service(self, enable: bool = True) -> CommandResultStamped:
        req = SetBool.Request()
        req.data = enable

        # Usa il client già creato nel __init__
        response = await self.enable_service_client.call_async(req)

        result_msg = CommandResultStamped()
        result_msg.result = 0 if response.success else 1
        result_msg.error_msg = response.message or ""
        return result_msg


    async def call_reset_service(self) -> CommandResultStamped:
        req = Trigger.Request()

        response = await self.reset_service_client.call_async(req)

        result_msg = CommandResultStamped()
        # Trigger ha solo success e message
        result_msg.result = 0 if response.success else 1
        result_msg.error_msg = response.message or ""
        return result_msg