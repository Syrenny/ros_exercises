import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from custom_interfaces.msg import CustomMessage


class TextToCmdVelNode(Node):
    def __init__(self):
        super().__init__('text_to_cmd_vel')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            CustomMessage,
            'cmd_text',
            self.text_command_callback,
            10
        )

    def text_command_callback(self, msg):
        cmd_vel_msg = Twist()

        if msg.message == 'turn_right':
            cmd_vel_msg.angular.z = -1.5
        elif msg.message == 'turn_left':
            cmd_vel_msg.angular.z = 1.5
        elif msg.message == 'move_forward':
            cmd_vel_msg.linear.x = 1.0
        elif msg.message == 'move_backward':
            cmd_vel_msg.linear.x = -1.0

        self.publisher.publish(cmd_vel_msg)
        self.get_logger().info(f'Published cmd_vel: linear.x={cmd_vel_msg.linear.x}, '
                               f'angular.z={cmd_vel_msg.angular.z}')


def main():
    rclpy.init()

    text_to_cmd_vel_node = TextToCmdVelNode()
    rclpy.spin(text_to_cmd_vel_node)

    text_to_cmd_vel_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
