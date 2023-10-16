from tutorial_interfaces.action import MessageTurtleCommands
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import sys
import math


# class MessageTurtleActionServer(Node):
#     def __init__(self):
#         super().__init__('action_turtle_server')
#         self.is_running = False
#         self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
#         self.subscription = self.create_subscription(
#             Pose,
#             '/turtle1/pose',
#             self.pose_callback,
#             10
#         )
#         self._action_server = ActionServer(
#             self,
#             MessageTurtleCommands,
#             'message_turtle',
#             self.execute_callback)
#         self.odometer = 0
# 
#     def pose_callback(self, msg):
#         if msg.linear_velocity == 0 and msg.angular_velocity == 0:
#             self.is_running = False
#         else:
#             self.is_running = True
# 
#     def execute_callback(self, goal_handle):
#         cmd_vel_msg = Twist()
#         result = MessageTurtleCommands.Result()
#         result.result = False
#         feedback_msg = MessageTurtleCommands.Feedback()
# 
#         self.get_logger().info('Executing goal...')
# 
#         if goal_handle.request.command == "forward":
#             cmd_vel_msg.linear.x = float(goal_handle.request.s)
#             self.odometer += goal_handle.request.s
#         elif goal_handle.request.command == "turn_left":
#             cmd_vel_msg.angular.z = math.radians(goal_handle.request.angle)
#         elif goal_handle.request.command == "turn_right":
#             cmd_vel_msg.angular.z = -math.radians(goal_handle.request.angle)
#         else:
#             return result
# 
#         self.get_logger().info('Publishing...')
#         self.publisher.publish(cmd_vel_msg)
#         self.get_logger().info(f'Published cmd_vel: linear.x={cmd_vel_msg.linear.x}, '
#                                f'angular.z={cmd_vel_msg.angular.z}')
#         while self.is_running:
#             pass
# 
#         feedback_msg.odom = self.odometer
#         goal_handle.publish_feedback(feedback_msg)
#         result.result = True
#         goal_handle.succeed()
#         return result
# 
# 
# def main(args=None):
#     rclpy.init(args=args)
#     message_turtle_action_server = MessageTurtleActionServer()
#     rclpy.spin(message_turtle_action_server)
# 
# 
# if __name__ == '__main__':
#     main()


import collections
import threading
import time

from tutorial_interfaces.action import MessageTurtleCommands
from turtlesim.msg import Pose

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class MessageTurtleActionServer(Node):

    def __init__(self):
        super().__init__('message_turtle_action_server')
        self._goal_queue = collections.deque()
        self._goal_queue_lock = threading.Lock()
        self.odometer = 0
        self._current_goal = None
        self.is_running = False

        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'message_turtle',
            handle_accepted_callback=self.handle_accepted_callback,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

    def pose_callback(self, msg):
        if msg.linear_velocity == 0 and msg.angular_velocity == 0:
            self.is_running = False
        else:
            self.is_running = True

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def handle_accepted_callback(self, goal_handle):
        """Start or defer execution of an already accepted goal."""
        with self._goal_queue_lock:
            if self._current_goal is not None:
                # Put incoming goal in the queue
                self._goal_queue.append(goal_handle)
                self.get_logger().info('Goal put in the queue')
            else:
                # Start goal execution right away
                self._current_goal = goal_handle
                self._current_goal.execute()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute a goal."""
        try:
            self.get_logger().info('Executing goal...')

            cmd_vel_msg = Twist()
            result = MessageTurtleCommands.Result()
            result.result = False
            feedback_msg = MessageTurtleCommands.Feedback()

            if goal_handle.request.command == "forward":
                cmd_vel_msg.linear.x = float(goal_handle.request.s)
                self.odometer += goal_handle.request.s
            elif goal_handle.request.command == "turn_left":
                cmd_vel_msg.angular.z = math.radians(goal_handle.request.angle)
            elif goal_handle.request.command == "turn_right":
                cmd_vel_msg.angular.z = -math.radians(goal_handle.request.angle)
            else:
                return result

            self.get_logger().info('Publishing...')
            self.publisher.publish(cmd_vel_msg)
            self.get_logger().info(f'Published cmd_vel: linear.x={cmd_vel_msg.linear.x}, '
                                   f'angular.z={cmd_vel_msg.angular.z}')
            while self.is_running:
                pass

            feedback_msg.odom = self.odometer
            goal_handle.publish_feedback(feedback_msg)
            result.result = True
            goal_handle.succeed()
            self.get_logger().info(
                'Returning result: {0}'.format(result.result))
            return result

        finally:
            with self._goal_queue_lock:
                try:
                    # Start execution of the next goal in the queue.
                    self._current_goal = self._goal_queue.popleft()
                    self.get_logger().info('Next goal pulled from the queue')
                    self._current_goal.execute()
                except IndexError:
                    # No goal in the queue.
                    self._current_goal = None


def main(args=None):
    rclpy.init(args=args)

    message_turtle_action_server = MessageTurtleActionServer()

    executor = MultiThreadedExecutor()

    rclpy.spin(message_turtle_action_server, executor=executor)
    #
    # message_turtle_action_server.destroy()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
