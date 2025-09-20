import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default
from example_interfaces.srv import Trigger
from dua_hardware_interfaces.action import Arm, Disarm
from dua_aircraft_interfaces.action import Takeoff, Landing
from dua_movement_interfaces.action import Navigate
from simple_actionclient_py.simple_actionclient import Client as SimpleActionClient
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

        self.enable_service_name = self.params['services']['enable_component']
        self.reset_service_name = self.params['services']['reset_component']

        # --- Creazione clients action ---
        self.arm_client = SimpleActionClient(self, Arm, self.arm_action_name)
        self.disarm_client = SimpleActionClient(self, Disarm, self.disarm_action_name)
        self.takeoff_client = SimpleActionClient(self, Takeoff, self.takeoff_action_name)
        self.landing_client = SimpleActionClient(self, Landing, self.landing_action_name)
        self.navigate_client = SimpleActionClient(self, Navigate, self.navigate_action_name)

        # --- Creazione clients service ---
        self.enable_service_client = self.create_client(
            Trigger, self.enable_service_name, qos_profile=qos_profile_services_default)
        self.reset_service_client = self.create_client(
            Trigger, self.reset_service_name, qos_profile=qos_profile_services_default)

    # --- Action methods ---

    async def send_arm_goal(self, name: str):
        goal_msg = Arm.Goal()
        goal_msg.name = name
        result = await self.arm_client.send_goal(goal_msg)
        return result

    async def send_disarm_goal(self, name: str):
        goal_msg = Disarm.Goal()
        goal_msg.name = name
        result = await self.disarm_client.send_goal(goal_msg)
        return result


    async def send_takeoff_goal(self, altitude: float):
        goal_msg = Takeoff.Goal()
        # --- correzione: server aspetta takeoff_pose.pose.position.z ---
        pose = PoseStamped()
        pose.pose.position.z = altitude
        goal_msg.takeoff_pose = pose
       
        goal_handle = await self.takeoff_client.send_goal(goal_msg)
        #result = await goal_handle.get_result_async()
        # Ottieni il risultato dall'azione
        result_response = await goal_handle.get_result_async()
        result: Takeoff.Result = result_response.result
 
        # Controllo del risultato effettivo
        if result.result.result == CommandResultStamped.SUCCESS:
            self.get_logger().info("Takeoff completato con successo")
        else:
            self.get_logger().error(f"Errore Takeoff: {result.result.error_msg}")
 
        return result


    async def send_landing_goal(self, descend: bool = True, min_z: float = 0.0):
        goal_msg = Landing.Goal()
        # --- correzione: server aspetta minimums.point.z e descend ---
        goal_msg.minimums.point = Point(x=0.0, y=0.0, z=min_z)
        goal_msg.descend = descend
        goal_handle = await self.landing_client.send_goal(goal_msg)
        result_response = await goal_handle.get_result_async()
        result: Landing.Result = result_response.result

        # Controllo del risultato effettivo
        if result.result.result == CommandResultStamped.SUCCESS:
            self.get_logger().info("Landing completato con successo")
        else:
            self.get_logger().error(f"Errore Landing: {result.result.error_msg}")
 
        return result

    async def send_navigate_goal(self, x: float, y: float, z: float):
        goal_msg = Navigate.Goal()
        goal_msg.target_pose = PoseStamped()
        goal_msg.target_pose.pose.position.x = x
        goal_msg.target_pose.pose.position.y = y
        goal_msg.target_pose.pose.position.z = z
        result = await self.navigate_client.send_goal(goal_msg)
        return result

    # --- Service methods ---
    async def call_enable_service(self):
        if not self.enable_service_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f"Service '{self.enable_service_name}' non disponibile.")
        req = Trigger.Request()
        future = self.enable_service_client.call_async(req)
        result = await asyncio.wrap_future(future)
        return result

    async def call_reset_service(self):
        if not self.reset_service_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f"Service '{self.reset_service_name}' non disponibile.")
        req = Trigger.Request()
        future = self.reset_service_client.call_async(req)
        result = await asyncio.wrap_future(future)
        return result
