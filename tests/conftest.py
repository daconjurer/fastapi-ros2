import os
import random
import time
from collections.abc import Callable, Iterator
from typing import Any, cast, final

import pytest
import rclpy
from fastapi.testclient import TestClient
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rcl_interfaces.msg import IntegerRange, ParameterDescriptor
from rclpy.client import Client
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from std_msgs.msg import String
from turtlesim_msgs.action import RotateAbsolute
from turtlesim_msgs.srv import TeleportAbsolute

from fastapi_ros2.nodes import RosState, Spinner
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


def ros_state(api: TestClient) -> RosState:
    """The API's ROS node and controllers, as its lifespan created them."""
    return cast(RosState, api.app_state["ros"])


def api_client(api: TestClient, srv_type: type) -> Client[Any, Any]:  # pyright: ignore[reportExplicitAny]
    """The API node's client for a service type."""
    return next(c for c in ros_state(api).node.clients if c.srv_type is srv_type)


@final
class Servers:
    """The ROS side the API talks to, run in the test process.

    `add_two_ints` is the project's real server, the teleport server records its
    requests, a subscription records what is published to `/print`, the rotate
    action server publishes feedback for `rotate_steps` ticks before it succeeds,
    and the node has parameters like turtlesim's.
    """

    def __init__(self):
        self.add_two_ints = AddTwoIntsService()
        self.node = rclpy.create_node("test_servers")
        self.teleport = self.node.create_service(
            TeleportAbsolute,
            "/turtle1/teleport_absolute",
            self._teleport,
        )
        self.teleports: list[TeleportAbsolute.Request] = []
        self.prints: list[str] = []
        self.node.create_subscription(
            String,
            "/print",
            lambda msg: self.prints.append(msg.data),
            10,
        )
        red = ParameterDescriptor(
            integer_range=[IntegerRange(from_value=0, to_value=255, step=1)],
        )
        self.node.declare_parameter("background_r", 69, red)
        self.node.declare_parameter("scale", 1.0)
        self.node.declare_parameter("names", ["a"])
        self.node.declare_parameter("ids", [1, 2])
        self.node.declare_parameter("weights", [0.5])
        self.rotate_steps = 3
        self.rotating = 0
        self.rotate = ActionServer(
            self.node,
            RotateAbsolute,
            "/turtle1/rotate_absolute",
            self._rotate,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        # Services on this node receive requests but never answer: it never spins.
        self.unresponsive_node = rclpy.create_node("test_unresponsive_servers")

        executor = SingleThreadedExecutor()
        executor.add_node(self.add_two_ints)
        executor.add_node(self.node)
        self.spinner = Spinner(executor)

    def _teleport(
        self,
        request: TeleportAbsolute.Request,
        response: TeleportAbsolute.Response,
    ) -> TeleportAbsolute.Response:
        self.teleports.append(request)
        return response

    async def _rotate(
        self,
        goal: ServerGoalHandle[
            RotateAbsolute.Goal,
            RotateAbsolute.Result,
            RotateAbsolute.Feedback,
            RotateAbsolute.Impl,
        ],
    ) -> RotateAbsolute.Result:
        # A coroutine that waits on timers between steps, so that the node's
        # single-threaded executor serves cancel requests in the meantime.
        self.rotating += 1
        try:
            for step in range(self.rotate_steps):
                if goal.is_cancel_requested:
                    goal.canceled()
                    return RotateAbsolute.Result(delta=float(step))
                remaining = float(self.rotate_steps - step)
                goal.publish_feedback(RotateAbsolute.Feedback(remaining=remaining))
                await self._tick()
            goal.succeed()
            return RotateAbsolute.Result(delta=goal.request.theta)
        finally:
            self.rotating -= 1

    def _tick(self, seconds: float = 0.01) -> Future[None]:
        future: Future[None] = Future()

        def done() -> None:
            timer.cancel()
            future.set_result(None)

        timer = self.node.create_timer(seconds, done)
        return future

    def shutdown(self) -> None:
        # A goal still running when the executor stops would fail mid-step.
        wait_until(lambda: self.rotating == 0)
        self.spinner.stop()
        self.rotate.destroy()
        for node in (self.add_two_ints, self.node, self.unresponsive_node):
            node.destroy_node()


@pytest.fixture
def api() -> Iterator[TestClient]:
    """A client for the app, running its real lifespan and ROS node."""
    with TestClient(app) as client:
        yield client


@pytest.fixture
def servers(api: TestClient) -> Iterator[Servers]:
    servers = Servers()
    node = ros_state(api).node
    wait_until(lambda: all(client.service_is_ready() for client in node.clients))
    wait_until(lambda: node.count_subscribers("/print") > 0)
    # Action clients are not among the node's clients.
    wait_until(ros_state(api).rotate_controller.client.is_ready)
    yield servers
    servers.shutdown()
