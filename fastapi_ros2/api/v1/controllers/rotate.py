from collections.abc import AsyncIterator
from typing import final

from loguru import logger
from rclpy.node import Node
from turtlesim_msgs.action import RotateAbsolute

from fastapi_ros2.actions.client import ActionClientAsync, Event


@final
class RotateController:
    def __init__(self, node: Node):
        self.client = ActionClientAsync(
            node,
            RotateAbsolute,
            "/turtle1/rotate_absolute",
        )

    async def rotate(self, theta: float) -> str:
        goal_id = await self.client.send(RotateAbsolute.Goal(theta=theta))
        logger.info(f'Goal "{goal_id}" to rotate the turtle to {theta} accepted')
        return goal_id

    def events(
        self,
        goal_id: str,
    ) -> AsyncIterator[Event[RotateAbsolute.Result, RotateAbsolute.Feedback]]:
        return self.client.events(goal_id)

    async def cancel(self, goal_id: str) -> None:
        logger.info(f'Canceling goal "{goal_id}"')
        await self.client.cancel(goal_id)
