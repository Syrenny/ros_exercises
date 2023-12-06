import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import numpy as np


class BarrierDetector (Node):
    def __init__(self):
        super().__init__('detector')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/robot/scan',
            self.laser_scan_callback,
            10
        )

    def laser_scan_callback(self, msg):
        cmd_vel_msg = Twist()
        view = msg.ranges[164:197]
        view = view * np.cos(np.radians(np.abs(np.abs(np.array(range(164, 197)) - 180))))
        if np.all(view > 1):
            cmd_vel_msg.linear.x = 1.0
        self.publisher.publish(cmd_vel_msg)
        self.get_logger().info(f'Published cmd_vel: linear.x={cmd_vel_msg.linear.x}, '
                               f'angular.z={cmd_vel_msg.angular.z}')


def main():
    rclpy.init()

    barrier_detector = BarrierDetector ()
    rclpy.spin(barrier_detector)

    barrier_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()