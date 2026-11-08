from collections.abc import AsyncGenerator, Callable
from contextlib import asynccontextmanager
from dataclasses import dataclass
from typing import Annotated, cast, final
import os
import signal
import threading

import rclpy
from fastapi import Depends, FastAPI, Request
from loguru import logger
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future

from fastapi_ros2.api.v1.controllers.add_two_ints import AddTwoIntsController
from fastapi_ros2.api.v1.controllers.turtlesim import TurtleSimController
from fastapi_ros2.api.v1.controllers.topics import TopicController
from fastapi_ros2.api.v1.controllers.parameters import ParameterController
from fastapi_ros2.services.call import ServiceClient


@final
class Spinner:
    """Spin an executor on a daemon thread until `stop()`.

    rclpy's `Executor.shutdown()` in Lyrical does not wait for a running callback
    before destroying the executor's guard condition, so the callback fails and
    logs "cannot use Destroyable" (fixed upstream in ros2/rclpy#1666). Stopping
    the spin thread first avoids that: a single-threaded executor runs callbacks
    on that thread, so once it has joined, none is running.
    """

    def __init__(
        self,
        executor: SingleThreadedExecutor,
        on_crash: Callable[[Exception], None] | None = None,
    ):
        self.executor = executor
        # An exception in a callback ends the spin. `on_crash` gets it, if given;
        # otherwise it ends the thread as an unhandled exception.
        self._on_crash = on_crash
        # Completing this future wakes the executor and ends the spin.
        self._stopped: Future[None] = Future()
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self) -> None:
        try:
            self.executor.spin_until_future_complete(self._stopped)
        except ExternalShutdownException:
            # rclpy was shut down from outside, e.g. by its SIGINT/SIGTERM handler.
            pass
        except Exception as error:
            if self._on_crash is None:
                raise
            self._on_crash(error)

    def stop(self) -> None:
        self._stopped.set_result(None)
        self.thread.join()
        self.executor.shutdown()


def _stop_on_crash(error: Exception) -> None:
    """Stop the API when its executor dies, so that Docker restarts it.

    Without the executor no ROS response or message ever arrives again. SIGTERM
    runs the same graceful shutdown as `docker compose stop`.
    """
    logger.opt(exception=error).critical(
        "The ROS executor stopped after a callback failed; stopping the API",
    )
    os.kill(os.getpid(), signal.SIGTERM)


@dataclass(frozen=True)
class RosState:
    """The API's ROS node and what the routes use it through."""

    node: Node
    spin_thread: threading.Thread
    # Shutting rclpy down destroys the node, after which its name can no longer be
    # read, so /health reads it from here.
    node_name: str
    # The services the API depends on, not the parameter services it calls on
    # demand.
    services: list[ServiceClient]
    add_controller: AddTwoIntsController
    turtle_controller: TurtleSimController
    topic_controller: TopicController
    parameter_controller: ParameterController


@asynccontextmanager
async def ros_lifespan(_app: FastAPI) -> AsyncGenerator[dict[str, RosState]]:
    """Run the API's ROS2 node for as long as the app runs.

    All clients and publishers live on this one node. Routes get it, with the
    controllers and the spin thread, by taking a `Ros` parameter.
    """
    rclpy.init()
    node = rclpy.create_node("fastapi_ros2")

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = Spinner(executor, on_crash=_stop_on_crash)

    add_controller = AddTwoIntsController(node)
    turtle_controller = TurtleSimController(node)
    topic_controller = TopicController(node)
    parameter_controller = ParameterController(node)

    try:
        yield {
            "ros": RosState(
                node=node,
                spin_thread=spinner.thread,
                node_name=node.get_fully_qualified_name(),
                services=[add_controller.client, turtle_controller.client],
                add_controller=add_controller,
                turtle_controller=turtle_controller,
                topic_controller=topic_controller,
                parameter_controller=parameter_controller,
            ),
        }
    finally:
        spinner.stop()
        node.destroy_node()
        rclpy.try_shutdown()


def _ros_state(request: Request) -> RosState:
    # Starlette's request state is untyped; the lifespan above sets it.
    return cast(RosState, request.state.ros)


Ros = Annotated[RosState, Depends(_ros_state)]
