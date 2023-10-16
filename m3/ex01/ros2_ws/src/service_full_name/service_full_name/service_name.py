import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import FullNameSumService


class SummFullName(Node):
    def __init__(self):
        super().__init__("full_name_summ_service")
        self.srv = self.create_service(FullNameSumService, "full_name_summ_service", self.summ_full_name_callback)

    def summ_full_name_callback(self, request, response):
        response.full_name = request.first_name + " " + request.name + " " + request.last_name
        self.get_logger().info(f'Incoming request\n'
                               f'name: {request.name} '
                               f'first_name: {request.first_name} '
                               f'last_name: {request.last_name}')
        return response


def main():
    rclpy.init()
    summ_full_name_service = SummFullName()
    rclpy.spin(summ_full_name_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

