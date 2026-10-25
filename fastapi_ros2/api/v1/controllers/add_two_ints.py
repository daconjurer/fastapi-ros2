from typing import final
from example_interfaces.srv import AddTwoInts
from loguru import logger
from rclpy.node import Node

from fastapi_ros2.services.add_two_ints.client import AddTwoIntsClientAsync


@final
class AddTwoIntsController:
    def __init__(self, node: Node):
        self.client = AddTwoIntsClientAsync(node)

    async def add_two_ints(self, a: int, b: int) -> AddTwoInts.Response:
        logger.info(
            f'Sending request to service "{self.client.name}" with params {a} and {b}'
        )
        server_response = await self.client.send_request(a, b)

        logger.info(
            f'Response from service "{self.client.name}" '
            f"to request with params {a} and {b}: {server_response.sum}"
        )
        return server_response
