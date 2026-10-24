import sys

from loguru import logger

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

from fastapi_ros2.services.call import call_service


class AddTwoIntsClientAsync:
    def __init__(self, node: Node):
        self.client_ = node.create_client(AddTwoInts, "add_two_ints")

    @property
    def name(self):
        return self.client_.srv_name

    async def send_request(self, a: int, b: int) -> AddTwoInts.Response:
        return await call_service(self.client_, AddTwoInts.Request(a=a, b=b))


def main():
    rclpy.init()

    node = rclpy.create_node("add_two_ints_async_client")
    minimal_client = AddTwoIntsClientAsync(node)
    minimal_client.client_.wait_for_service()
    future = minimal_client.client_.call_async(
        AddTwoInts.Request(a=int(sys.argv[1]), b=int(sys.argv[2]))
    )
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    logger.info(
        f"Result of add_two_ints: for {int(sys.argv[1])} + {int(sys.argv[2])} = {response.sum}"
    )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
