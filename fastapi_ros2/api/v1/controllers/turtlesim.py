from loguru import logger
from rclpy.node import Node
from turtlesim_msgs.srv import TeleportAbsolute

from fastapi_ros2.services.turtlesim.client import TurtlesimMoveClientAsync


class TurtleSimController:
    def __init__(self, node: Node):
        self.client = TurtlesimMoveClientAsync(node)

    async def move_absolute(
        self, x: float, y: float, theta: float
    ) -> TeleportAbsolute.Response:
        logger.info(
            f'Sending request to service "{self.client.name}" with params {x}, {y} and {theta}'
        )
        server_response = await self.client.send_request(x, y, theta)

        logger.info(
            f'Response from service "{self.client.name}" '
            f"to request with params {x}, {y} and {theta}: {server_response}"
        )
        return server_response
