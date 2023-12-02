import sys

from loguru import logger

import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute


class TurtlesimMoveClientAsync(Node):
    def __init__(self):
        super().__init__("turtlesim_teleport_absolute_async_client")
        self.client_ = self.create_client(
            TeleportAbsolute, "/turtle1/teleport_absolute"
        )
        while not self.client_.wait_for_service(timeout_sec=1.0):
            logger.info("service not available, waiting...")
        self.req = TeleportAbsolute.Request()

    @property
    def name(self):
        return self.client_.srv_name

    def send_request(self, x: float, y: float, theta: float):
        self.req.x = x
        self.req.y = y
        self.req.theta = theta
        self.future = self.client_.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = TurtlesimMoveClientAsync()
    response = minimal_client.send_request(
        float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    )
    logger.info(
        f"Result of teleport_absolute with params "
        f"x={float(sys.argv[1])}, y={float(sys.argv[2])}, theta={float(sys.argv[2])}: {response}"
    )

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
