import sys
from tutorial_interfaces.srv import FullNameSumService
import rclpy
from rclpy.node import Node


class FullNameSummClient(Node):
    def __init__(self):
        super().__init__('full_name_summ_client_async')
        self.cli = self.create_client(FullNameSumService, 'full_name_summ_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = FullNameSumService.Request()

    def send_request(self, last_name, name, first_name):
        self.req.last_name = last_name
        self.req.name = name
        self.req.first_name = first_name
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()
    client = FullNameSummClient()
    response = client.send_request(str(sys.argv[1]), str(sys.argv[2]), str(sys.argv[3]))
    client.get_logger().info(
        f'Result of full_name_summ:\n' 
        f'{response.full_name}\n')
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
