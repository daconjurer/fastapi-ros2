from loguru import logger
from rclpy.node import Node

from fastapi_ros2.topics.print_publisher import PrintPublisher


class PrintController:
    def __init__(self, node: Node):
        self.publisher = PrintPublisher(node)

    async def publish_print(self) -> int:
        logger.info(f'Publishing to topic "{self.publisher.topic}"')
        return self.publisher.publish_print()
