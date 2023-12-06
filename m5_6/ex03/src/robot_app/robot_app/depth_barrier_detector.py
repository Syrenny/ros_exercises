import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

import numpy as np


class BarrierDetector (Node):
    def __init__(self):
        super().__init__('detector')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Image,
            '/depth/image',
            self.depth_camera_callback,
            10
        )

    def depth_camera_callback(self, msg):
        cmd_vel_msg = Twist()
        image = np.array(msg.data, dtype="uint8")
        range = image[len(image) // 2 - 30: len(image) // 2 + 30].mean()
        if range > 120.0:
            cmd_vel_msg.linear.x = 1.0
        self.publisher.publish(cmd_vel_msg)
        self.get_logger().info(f'Published cmd_vel: linear.x={cmd_vel_msg.linear.x}, '
                               f'angular.z={cmd_vel_msg.angular.z} range={range}')


def main():
    rclpy.init()

    barrier_detector = BarrierDetector ()
    rclpy.spin(barrier_detector)

    barrier_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()