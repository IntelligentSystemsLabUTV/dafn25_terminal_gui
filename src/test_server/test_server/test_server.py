#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from dua_hardware_interfaces.action import Arm, Disarm
from dua_aircraft_interfaces.action import Takeoff, Landing
from dua_movement_interfaces.action import Navigate

from std_srvs.srv import SetBool, Trigger

from dua_common_interfaces.msg import CommandResultStamped
from std_msgs.msg import Header


class TestServerNode(Node):
    """
    Test server node that provides action and service servers to test clients.
    All goals are accepted and all operations succeed by default, feel free to change this behavior
    for testing purposes wherever you need.
    Feel also free to change the names of the actions and services to match your needs.
    """
    def __init__(self):
        super().__init__('test_server_node')

        # Use reentrant callback group for concurrent execution
        self.callback_group = ReentrantCallbackGroup()

        # Create action servers with explicit goal callbacks
        self.arm_action_server = ActionServer(
            self,
            Arm,
            '~/arm',
            self.execute_arm,
            goal_callback=self.arm_goal_callback,
            callback_group=self.callback_group
        )

        self.disarm_action_server = ActionServer(
            self,
            Disarm,
            '~/disarm',
            self.execute_disarm,
            goal_callback=self.disarm_goal_callback,
            callback_group=self.callback_group
        )

        self.takeoff_action_server = ActionServer(
            self,
            Takeoff,
            '~/takeoff',
            self.execute_takeoff,
            goal_callback=self.takeoff_goal_callback,
            callback_group=self.callback_group
        )

        self.landing_action_server = ActionServer(
            self,
            Landing,
            '~/landing',
            self.execute_landing,
            goal_callback=self.landing_goal_callback,
            callback_group=self.callback_group
        )

        self.navigate_action_server = ActionServer(
            self,
            Navigate,
            '~/navigate',
            self.execute_navigate,
            goal_callback=self.navigate_goal_callback,
            callback_group=self.callback_group
        )

        # Create service servers
        self.setbool_service = self.create_service(
            SetBool,
            '~/enable_component',
            self.setbool_callback,
            callback_group=self.callback_group
        )

        self.trigger_service = self.create_service(
            Trigger,
            '~/reset_component',
            self.trigger_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('Node initialized')

    def create_success_result(self):
        """Helper to create successful CommandResultStamped"""
        result = CommandResultStamped()
        result.header = Header()
        result.header.stamp = self.get_clock().now().to_msg()
        result.result = CommandResultStamped.SUCCESS  # SUCCESS = 1
        result.error_msg = "No error"
        return result

    # Goal callbacks - control goal acceptance/rejection here
    def arm_goal_callback(self, goal_request):
        """Accept/reject Arm goals"""
        self.get_logger().info('Arm goal ACCEPTED')
        return GoalResponse.ACCEPT
        # return GoalResponse.REJECT

    def disarm_goal_callback(self, goal_request):
        """Accept/reject Disarm goals"""
        self.get_logger().info('Disarm goal ACCEPTED')
        return GoalResponse.ACCEPT
        # return GoalResponse.REJECT

    def takeoff_goal_callback(self, goal_request: Takeoff.Goal):
        """Accept/reject Takeoff goals"""
        # Print relevant data: pose.position.z
        self.get_logger().info(
            f'Takeoff pose.position.z: {goal_request.takeoff_pose.pose.position.z}')
        self.get_logger().info('Takeoff goal ACCEPTED')
        return GoalResponse.ACCEPT
        # return GoalResponse.REJECT

    def landing_goal_callback(self, goal_request: Landing.Goal):
        """Accept/reject Landing goals"""
        # Print relevant data: pose.position.z (from minimums)
        self.get_logger().info(
            f'Landing minimums.point.z: {goal_request.minimums.point.z}')
        self.get_logger().info(f'Landing descend: {goal_request.descend}')
        self.get_logger().info('Landing goal ACCEPTED')
        return GoalResponse.ACCEPT
        # return GoalResponse.REJECT

    def navigate_goal_callback(self, goal_request: Navigate.Goal):
        """Accept/reject Navigate goals"""
        # Print relevant data: pose.position, pose.orientation
        pos = goal_request.target_pose.pose.position
        ori = goal_request.target_pose.pose.orientation
        self.get_logger().info(
            f'Navigate pose.position: x={pos.x}, y={pos.y}, z={pos.z}')
        self.get_logger().info(
            f'Navigate pose.orientation: x={ori.x}, y={ori.y}, z={ori.z}, w={ori.w}')
        self.get_logger().info('Navigate goal ACCEPTED')
        return GoalResponse.ACCEPT
        # return GoalResponse.REJECT

    # Execute callbacks - control execution and result here
    def execute_arm(self, goal_handle):
        """Execute Arm action"""
        self.get_logger().info('Executing Arm action...')

        # Simulate operation duration
        time.sleep(1.0)

        # Create successful result
        result = Arm.Result()
        result.result = self.create_success_result()
        # Change this line to FAILED/ERROR for testing
        result.result.result = CommandResultStamped.SUCCESS

        goal_handle.succeed()
        # goal_handle.abort()
        self.get_logger().info('Arm action completed successfully')
        return result

    def execute_disarm(self, goal_handle):
        """Execute Disarm action"""
        self.get_logger().info('Executing Disarm action...')

        # Simulate operation duration
        time.sleep(1.0)

        # Create successful result
        result = Disarm.Result()
        result.result = self.create_success_result()
        # Change this line to FAILED/ERROR for testing
        result.result.result = CommandResultStamped.SUCCESS

        goal_handle.succeed()
        # goal_handle.abort()
        self.get_logger().info('Disarm action completed successfully')
        return result

    def execute_takeoff(self, goal_handle):
        """Execute Takeoff action"""
        self.get_logger().info('Executing Takeoff action...')

        # Simulate operation duration
        time.sleep(1.0)

        # Create successful result
        result = Takeoff.Result()
        result.result = self.create_success_result()
        # Change this line to FAILED/ERROR for testing
        result.result.result = CommandResultStamped.SUCCESS

        goal_handle.succeed()
        # goal_handle.abort()
        self.get_logger().info('Takeoff action completed successfully')
        return result

    def execute_landing(self, goal_handle):
        """Execute Landing action"""
        self.get_logger().info('Executing Landing action...')

        # Simulate operation duration
        time.sleep(1.0)

        # Create successful result
        result = Landing.Result()
        result.result = self.create_success_result()
        # Change this line to FAILED/ERROR for testing
        result.result.result = CommandResultStamped.SUCCESS

        goal_handle.succeed()
        # goal_handle.abort()
        self.get_logger().info('Landing action completed successfully')
        return result

    def execute_navigate(self, goal_handle):
        """Execute Navigate action"""
        self.get_logger().info('Executing Navigate action...')

        # Simulate operation duration
        time.sleep(1.0)

        # Create successful result
        result = Navigate.Result()
        result.result = self.create_success_result()
        # Change this line to FAILED/ERROR for testing
        result.result.result = CommandResultStamped.SUCCESS

        goal_handle.succeed()
        # goal_handle.abort()
        self.get_logger().info('Navigate action completed successfully')
        return result

    def setbool_callback(self, request, response: SetBool.Response):
        """Handle SetBool service"""
        # Print relevant data: request.data
        self.get_logger().info(f'SetBool request.data: {request.data}')

        # Response always successful
        response.success = True
        response.message = "No error"

        self.get_logger().info('SetBool service completed successfully')
        return response

    def trigger_callback(self, request, response: Trigger.Response):
        """Handle Trigger service"""
        self.get_logger().info('Received Trigger request')

        # Response always successful
        response.success = True
        response.message = "No error"

        self.get_logger().info('Trigger service completed successfully')
        return response


def main(args=None):
    rclpy.init(args=args)

    node = TestServerNode()

    # Use MultiThreadedExecutor for concurrent callback execution
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
