from loguru import logger

from fastapi_ros2.topics.print_publisher import PrintPublisher


class PrintController:
    def __init__(self):
        self.publisher = PrintPublisher()

    async def publish_print(self) -> int:
        logger.info(
            f'Publishing from "{self.publisher.get_name()}" to topic {self.publisher.topic}'
        )
        return self.publisher.publish_print()
