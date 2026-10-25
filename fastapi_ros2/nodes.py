from collections.abc import AsyncGenerator
from contextlib import asynccontextmanager
from dataclasses import dataclass
from typing import Annotated, cast
import threading

import rclpy
from fastapi import Depends, FastAPI, Request
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from fastapi_ros2.api.v1.controllers.add_two_ints import AddTwoIntsController
from fastapi_ros2.api.v1.controllers.turtlesim import TurtleSimController
from fastapi_ros2.api.v1.controllers.print import PrintController


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

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    add_controller = AddTwoIntsController(node)
    turtle_controller = TurtleSimController(node)
    print_controller = PrintController(node)

    try:
        yield {
            "ros": RosState(
                node=node,
                spin_thread=spin_thread,
                node_name=node.get_fully_qualified_name(),
                service_names=[client.service_name for client in node.clients],
                add_controller=add_controller,
                turtle_controller=turtle_controller,
                print_controller=print_controller,
            )
        }
    finally:
        executor.shutdown()
        spin_thread.join()
        node.destroy_node()
        rclpy.try_shutdown()


def _ros_state(request: Request) -> RosState:
    # Starlette's request state is untyped; the lifespan above sets it.
    return cast(RosState, request.state.ros)


Ros = Annotated[RosState, Depends(_ros_state)]
