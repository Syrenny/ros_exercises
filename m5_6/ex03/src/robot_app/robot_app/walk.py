import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import math
import time


class TwistPublisher(Node):
    def __init__(self):
        super().__init__('robot_cmd_vel')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.spiral)
        self.time = 5.0

    def circle(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = 1.5
        cmd_vel_msg.linear.x = 1.0

        self.publisher.publish(cmd_vel_msg)
        self.get_logger().info(f'Published cmd_vel: linear.x={cmd_vel_msg.linear.x}, '
                               f'angular.z={cmd_vel_msg.angular.z}')
        
    def spiral(self):
        twist_msg = Twist()
        twist_msg.linear.x = 2.0
        twist_msg.angular.z = self.time

        self.publisher.publish(twist_msg)
        self.time *= 0.97


def main():
    rclpy.init()
    twist_publisher = TwistPublisher()
    # twist_publisher.circle()
    rclpy.spin(twist_publisher)
    twist_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()