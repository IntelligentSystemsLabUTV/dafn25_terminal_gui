import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_services_default
from example_interfaces.srv import Trigger
from dua_interfaces.action import Arm, Disarm, Takeoff, Landing, Navigate
import asyncio
import yaml

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
        self.arm_client = ActionClient(self, Arm, self.arm_action_name)
        self.disarm_client = ActionClient(self, Disarm, self.disarm_action_name)
        self.takeoff_client = ActionClient(self, Takeoff, self.takeoff_action_name)
        self.landing_client = ActionClient(self, Landing, self.landing_action_name)
        self.navigate_client = ActionClient(self, Navigate, self.navigate_action_name)        
        
        # --- Creazione clients service ---
        self.enable_service_client = self.create_client(Trigger, self.enable_service_name, qos_profile=qos_profile_services_default)
        self.reset_service_client = self.create_client(Trigger, self.reset_service_name, qos_profile=qos_profile_services_default)
    
    
    # --- Action methods ---
    async def send_arm_goal(self, name: str):
        if not self.arm_client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError(f"Action server '{self.arm_action_name}' non disponibile.")
        
        goal_msg = Arm.Goal()
        goal_msg.name = name
        send_goal_future = self.arm_client.send_goal_async(goal_msg)
        goal_handle = await asyncio.wrap_future(send_goal_future)

        if not goal_handle.accepted:
            raise RuntimeError("Obiettivo 'Arm' rifiutato.")
        
        result_future = goal_handle.get_result_async()
        result = await asyncio.wrap_future(result_future)
        return result.result

    async def send_disarm_goal(self, name: str):
        if not self.disarm_client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError(f"Action server '{self.disarm_action_name}' non disponibile.")
        
        goal_msg = Disarm.Goal()
        goal_msg.name = name
        send_goal_future = self.disarm_client.send_goal_async(goal_msg)
        goal_handle = await asyncio.wrap_future(send_goal_future)

        if not goal_handle.accepted:
            raise RuntimeError("Obiettivo 'Disarm' rifiutato.")
            
        result_future = goal_handle.get_result_async()
        result = await asyncio.wrap_future(result_future)
        return result.result

    async def send_takeoff_goal(self, altitude: float):
        if not self.takeoff_client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError(f"Action server '{self.takeoff_action_name}' non disponibile.")
        
        goal_msg = Takeoff.Goal()
        goal_msg.altitude = altitude
        send_goal_future = self.takeoff_client.send_goal_async(goal_msg)
        goal_handle = await asyncio.wrap_future(send_goal_future)
        
        if not goal_handle.accepted:
            raise RuntimeError("Obiettivo 'Takeoff' rifiutato.")

        result_future = goal_handle.get_result_async()
        result = await asyncio.wrap_future(result_future)
        return result.result

    async def send_landing_goal(self):
        if not self.landing_client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError(f"Action server '{self.landing_action_name}' non disponibile.")
            
        goal_msg = Landing.Goal()
        send_goal_future = self.landing_client.send_goal_async(goal_msg)
        goal_handle = await asyncio.wrap_future(send_goal_future)

        if not goal_handle.accepted:
            raise RuntimeError("Obiettivo 'Landing' rifiutato.")
            
        result_future = goal_handle.get_result_async()
        result = await asyncio.wrap_future(result_future)
        return result.result

    async def send_navigate_goal(self, x: float, y: float, z: float):
        if not self.navigate_client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError(f"Action server '{self.navigate_action_name}' non disponibile.")
            
        goal_msg = Navigate.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.z = z
        send_goal_future = self.navigate_client.send_goal_async(goal_msg)
        goal_handle = await asyncio.wrap_future(send_goal_future)

        if not goal_handle.accepted:
            raise RuntimeError("Obiettivo 'Navigate' rifiutato.")
            
        result_future = goal_handle.get_result_async()
        result = await asyncio.wrap_future(result_future)
        return result.result

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