from typing import final

from loguru import logger
from rclpy.node import Node

from fastapi_ros2.topics.publisher import TopicPublisher


@final
class TopicController:
    def __init__(self, node: Node):
        self.publisher = TopicPublisher(node)

    def publish(
        self,
        topic: str,
        message: dict[str, object],
        type_name: str | None,
    ) -> str:
        type_name = self.publisher.publish(topic, message, type_name)
        logger.info(f'Published a {type_name} message to topic "{topic}"')
        return type_name
