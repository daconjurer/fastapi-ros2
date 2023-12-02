from loguru import logger

from turtlesim.srv._teleport_absolute import TeleportAbsolute_Response as Response
from fastapi_ros2.services.turtlesim.client import TurtlesimMoveClientAsync


class TurtleSimController:
    def __init__(self):
        self.client = TurtlesimMoveClientAsync()

    async def move_absolute(self, x: float, y: float, theta: float) -> bool:
        logger.info(
            f'Sending request to service "{self.client.name}" with params {x}, {y} and {theta}'
        )
        server_response = self.client.send_request(x, y, theta)
        result_move = True if isinstance(server_response, Response) else False

        logger.info(
            f'Response from service "{self.client.name}" '
            f"to request with params {x}, {y} and {theta}: {result_move}"
        )
        return result_move
