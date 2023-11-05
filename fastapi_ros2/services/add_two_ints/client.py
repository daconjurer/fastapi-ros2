import sys

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClientAsync(Node):
    def __init__(self):
        super().__init__('add_two_ints_async_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = AddTwoIntsClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        f"Result of add_two_ints: for {int(sys.argv[1])} + {int(sys.argv[2])} = {response.sum}"
    )

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
