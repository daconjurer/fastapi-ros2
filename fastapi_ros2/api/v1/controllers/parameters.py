from typing import final

from loguru import logger
from rclpy.node import Node

from fastapi_ros2.parameters.client import ParametersClientAsync
from fastapi_ros2.parameters.values import PythonValue


@final
class ParameterController:
    def __init__(self, node: Node):
        self.client = ParametersClientAsync(node)

    async def get(self, node: str, name: str) -> PythonValue:
        return await self.client.get(node, name)

    async def set(self, node: str, name: str, value: PythonValue) -> PythonValue:
        logger.info(f'Setting parameter "{name}" of node "{node}" to {value!r}')
        return await self.client.set(node, name, value)
