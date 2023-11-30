from example_interfaces.srv import AddTwoInts  # type: ignore

import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore


class AddTwoIntsService(Node):
    def __init__(self):
        super().__init__("add_two_ints_service")
        self.srv = self.create_service(
            AddTwoInts, "add_two_ints", self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f"Incoming request: a = {request.a}, b = {request.a}")
        self.get_logger().info(f"Response: {response}")

        return response


def main():
    rclpy.init()
    minimal_service = AddTwoIntsService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
