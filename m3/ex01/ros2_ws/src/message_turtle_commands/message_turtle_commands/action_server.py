from tutorial_interfaces.action import MessageTurtleCommands
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import rclpy
import math
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import time


turtle_is_running = False


class MessageTurtleActionServer(Node):

    def __init__(self):
        super().__init__('message_turtle_action_server')
        self.odometer = 0

        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'message_turtle',
            execute_callback=self.execute_callback)

    def execute_callback(self, goal_handle):
        """Execute a goal."""
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
        time.sleep(0.1)
        self.get_logger().info('Publishing...')
        self.publisher.publish(cmd_vel_msg)
        self.get_logger().info(f'Published cmd_vel: linear.x={cmd_vel_msg.linear.x}, '
                               f'angular.z={cmd_vel_msg.angular.z}')
        while turtle_is_running:
            self.get_logger().info('Turtle is running...')

        goal_handle.succeed()

        feedback_msg.odom = self.odometer
        goal_handle.publish_feedback(feedback_msg)
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
        global turtle_is_running
        if msg.linear_velocity == 0 and msg.angular_velocity == 0:
            turtle_is_running = False
        else:
            turtle_is_running = True


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
