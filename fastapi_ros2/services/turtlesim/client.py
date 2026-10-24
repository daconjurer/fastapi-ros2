import sys

from loguru import logger

import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute

from fastapi_ros2.services.call import call_service


class TurtlesimMoveClientAsync(Node):
    def __init__(self):
        super().__init__("turtlesim_teleport_absolute_async_client")
        self.client_ = self.create_client(
            TeleportAbsolute, "/turtle1/teleport_absolute"
        )

    @property
    def name(self):
        return self.client_.srv_name

    async def send_request(self, x: float, y: float, theta: float):
        return await call_service(
            self.client_, TeleportAbsolute.Request(x=x, y=y, theta=theta)
        )


def main():
    rclpy.init()

    minimal_client = TurtlesimMoveClientAsync()
    minimal_client.client_.wait_for_service()
    future = minimal_client.client_.call_async(
        TeleportAbsolute.Request(
            x=float(sys.argv[1]), y=float(sys.argv[2]), theta=float(sys.argv[3])
        )
    )
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()
    logger.info(
        f"Result of teleport_absolute with params "
        f"x={float(sys.argv[1])}, y={float(sys.argv[2])}, theta={float(sys.argv[2])}: {response}"
    )

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
