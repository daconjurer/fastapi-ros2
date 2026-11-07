from collections.abc import AsyncGenerator
from contextlib import asynccontextmanager
from dataclasses import dataclass
from typing import Annotated, cast, final
import threading

import rclpy
from fastapi import Depends, FastAPI, Request
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future

from fastapi_ros2.api.v1.controllers.add_two_ints import AddTwoIntsController
from fastapi_ros2.api.v1.controllers.turtlesim import TurtleSimController
from fastapi_ros2.api.v1.controllers.print import PrintController


@final
class Spinner:
    """Spin an executor on a daemon thread until `stop()`.

    rclpy's `Executor.shutdown()` in Lyrical does not wait for a running callback
    before destroying the executor's guard condition, so the callback fails and
    logs "cannot use Destroyable" (fixed upstream in ros2/rclpy#1666). Stopping
    the spin thread first avoids that: a single-threaded executor runs callbacks
    on that thread, so once it has joined, none is running.
    """

    def __init__(self, executor: SingleThreadedExecutor):
        self.executor = executor
        # Completing this future wakes the executor and ends the spin.
        self._stopped: Future[None] = Future()
        # An exception in a callback ends the thread, which /health reports.
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self) -> None:
        try:
            self.executor.spin_until_future_complete(self._stopped)
        except ExternalShutdownException:
            # rclpy was shut down from outside, e.g. by its SIGINT/SIGTERM handler.
            pass

    def stop(self) -> None:
        self._stopped.set_result(None)
        self.thread.join()
        self.executor.shutdown()


@dataclass(frozen=True)
class RosState:
    """The API's ROS node and what the routes use it through."""

    node: Node
    spin_thread: threading.Thread
    # Shutting rclpy down destroys the node and its clients, after which their
    # names can no longer be read, so /health reads them from here.
    node_name: str
    service_names: list[str]
    add_controller: AddTwoIntsController
    turtle_controller: TurtleSimController
    print_controller: PrintController


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
    spinner = Spinner(executor)

    add_controller = AddTwoIntsController(node)
    turtle_controller = TurtleSimController(node)
    print_controller = PrintController(node)

    try:
        yield {
            "ros": RosState(
                node=node,
                spin_thread=spinner.thread,
                node_name=node.get_fully_qualified_name(),
                service_names=[client.service_name for client in node.clients],
                add_controller=add_controller,
                turtle_controller=turtle_controller,
                print_controller=print_controller,
            )
        }
    finally:
        spinner.stop()
        node.destroy_node()
        rclpy.try_shutdown()


def _ros_state(request: Request) -> RosState:
    # Starlette's request state is untyped; the lifespan above sets it.
    return cast(RosState, request.state.ros)


Ros = Annotated[RosState, Depends(_ros_state)]
