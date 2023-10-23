import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import math
import time
import collections
import threading

from tutorial_interfaces.action import MessageTurtleCommands
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


current_pose = Pose()
current_goal = Pose()


def has_reached_the_goal():
    return (abs(current_pose.x - current_goal.x) < 0.1 and
            abs(current_pose.y - current_goal.y) < 0.1 and
            abs(current_pose.theta - current_goal.theta) < 0.01)


def calculate_goal_point(x_initial, y_initial, theta, d):
    theta_rad = math.radians(theta)

    x_final = x_initial + d * math.cos(theta_rad)
    y_final = y_initial + d * math.sin(theta_rad)

    return x_final, y_final


class MessageTurtleActionServer(Node):

    def __init__(self):
        super().__init__('message_turtle_action_server')
        self.odometer = 0

        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'message_turtle',
            execute_callback=self.execute_callback,
            callback_group=MutuallyExclusiveCallbackGroup())

    def execute_callback(self, goal_handle):
        """Execute a goal."""
        global current_goal
        current_goal = current_pose
        self.get_logger().info('Executing goal...')
        result = MessageTurtleCommands.Result()
        result.result = False
        cmd_vel_msg = Twist()
        if goal_handle.request.command == "forward":
            current_goal.x, current_goal.y = calculate_goal_point(current_pose.x,
                                                                  current_pose.y,
                                                                  current_pose.theta,
                                                                  goal_handle.request.s)
            cmd_vel_msg.linear.x = 1.0
        elif goal_handle.request.command == "turn_left":
            current_goal.theta = current_pose.theta + math.radians(goal_handle.request.angle)
            cmd_vel_msg.angular.z = math.radians(30)
        elif goal_handle.request.command == "turn_right":
            current_goal.theta = current_pose.theta - math.radians(goal_handle.request.angle)
            cmd_vel_msg.angular.z = -math.radians(30)
        else:
            self.get_logger().info(
                'Returning result: {0}'.format(result.result))
            return result
        while not has_reached_the_goal():
            self.publisher.publish(cmd_vel_msg)
        self.publisher.publish(Twist())
        goal_handle.succeed()
        result.result = True
        self.get_logger().info(
            'Returning result: {0}'.format(result.result))
        return result


class PoseTopicSubscriber(Node):
    def __init__(self):
        super().__init__("pose_topic_subscriber")
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        global current_pose
        current_pose = msg


def main(args=None):
    rclpy.init(args=args)
    try:
        message_turtle_action_server = MessageTurtleActionServer()
        pose_topic_subscriber = PoseTopicSubscriber()

        # Set up multithreading
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(message_turtle_action_server)
        executor.add_node(pose_topic_subscriber)

        try:
            # Spin the nodes to execute the callbacks
            executor.spin()
        finally:
            # Shutdown the nodes
            executor.shutdown()
            message_turtle_action_server.destroy_node()
            pose_topic_subscriber.destroy_node()

    finally:
        # Shutdown
        rclpy.shutdown()


if __name__ == '__main__':
    main()
