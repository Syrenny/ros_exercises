import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import math
import time
import collections
import threading
import numpy as np
import sys

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


current_pose = Pose()
current_goal = Pose()
reached_goal = False


def calculate_linear_error():
    return np.sqrt((current_pose.x - current_goal.x)**2 + (current_pose.y - current_goal.y)**2)


def calculate_angular_error():
    return current_goal.theta - current_pose.theta


def calculate_angle(x_dest, y_dest, x_current, y_current, current_theta):
    delta_x = x_dest - x_current
    delta_y = y_dest - y_current
    desired_theta = math.atan2(delta_y, delta_x)
    angle_diff = desired_theta - current_theta
    if angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    elif angle_diff < -math.pi:
        angle_diff += 2 * math.pi
    return angle_diff


class MessageTurtleTopic(Node):
    def __init__(self, x, y, theta):
        super().__init__('message_turtle_topic')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        global current_goal
        current_goal.x = x
        current_goal.y = y
        self.theta = theta

    def timer_callback(self):
        global current_goal
        cmd_vel_msg = Twist()
        linear_error = calculate_linear_error()
        if linear_error > 0.1:
            current_goal.theta = current_pose.theta + calculate_angle(current_goal.x,
                                                                      current_goal.y,
                                                                      current_pose.x,
                                                                      current_pose.y,
                                                                      current_pose.theta)
            cmd_vel_msg.linear.x = linear_error * 1
        else:
            current_goal.theta = self.theta
        angular_error = calculate_angular_error()
        if linear_error < 0.1 and abs(angular_error) < 0.01:
            self.get_logger().info(f"FINISH on: {current_pose}\n")
            global reached_goal
            reached_goal = True
        else:
            cmd_vel_msg.angular.z = angular_error * 3
            self.publisher.publish(cmd_vel_msg)


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
    message_turtle_topic = MessageTurtleTopic(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
    pose_topic_subscriber = PoseTopicSubscriber()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(message_turtle_topic)
    executor.add_node(pose_topic_subscriber)

    while not reached_goal:
        executor.spin_once()
    # executor.shutdown()
    # message_turtle_topic.destroy_node()
    # pose_topic_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
