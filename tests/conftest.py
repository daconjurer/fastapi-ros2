import os
import random
import threading
import time
from collections.abc import Callable

import pytest
import rclpy
from fastapi.testclient import TestClient
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from turtlesim.srv import TeleportAbsolute

from fastapi_ros2.services.add_two_ints.server import AddTwoIntsService
from main import app

# Keep the tests' ROS graph to this machine and a domain of its own, so it never
# meets a running compose stack (e.g. the real turtlesim). rclpy reads these when
# the app's lifespan initializes it.
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ.setdefault("ROS_DOMAIN_ID", str(random.randint(1, 101)))


def wait_until(condition: Callable[[], object], timeout: float = 2.0) -> None:
    """Wait for the ROS graph to catch up, e.g. for a client to see its service."""
    deadline = time.monotonic() + timeout
    while not condition():
        assert time.monotonic() < deadline, "Timed out waiting for the ROS graph"
        time.sleep(0.001)


def api_client(api: TestClient, srv_type):
    """The API node's client for a service type."""
    return next(c for c in api.app_state["node"].clients if c.srv_type is srv_type)


class Servers:
    """The ROS side the API talks to, run in the test process.

    `add_two_ints` is the project's real server, the teleport server records its
    requests, and a subscription records what is published to `/print`.
    """

    def __init__(self):
        self.add_two_ints = AddTwoIntsService()
        self.node = rclpy.create_node("test_servers")
        self.teleport = self.node.create_service(
            TeleportAbsolute, "/turtle1/teleport_absolute", self._teleport
        )
        self.teleports: list[TeleportAbsolute.Request] = []
        self.prints: list[str] = []
        self.node.create_subscription(
            String, "/print", lambda msg: self.prints.append(msg.data), 10
        )
        # Services on this node receive requests but never answer: it never spins.
        self.unresponsive_node = rclpy.create_node("test_unresponsive_servers")

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.add_two_ints)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def _teleport(self, request, response):
        self.teleports.append(request)
        return response

    def shutdown(self) -> None:
        self.executor.shutdown()
        self.thread.join()
        for node in (self.add_two_ints, self.node, self.unresponsive_node):
            node.destroy_node()


@pytest.fixture
def api():
    """A client for the app, running its real lifespan and ROS node."""
    with TestClient(app) as client:
        yield client


@pytest.fixture
def servers(api):
    servers = Servers()
    node = api.app_state["node"]
    wait_until(lambda: all(client.service_is_ready() for client in node.clients))
    wait_until(lambda: node.count_subscribers("/print") > 0)
    yield servers
    servers.shutdown()
