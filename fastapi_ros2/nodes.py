from contextlib import asynccontextmanager
import threading

import rclpy
from fastapi import FastAPI
from rclpy.executors import MultiThreadedExecutor

from fastapi_ros2.api.v1.controllers.add_two_ints import AddTwoIntsController
from fastapi_ros2.api.v1.controllers.turtlesim import TurtleSimController
from fastapi_ros2.api.v1.controllers.print import PrintController


@asynccontextmanager
async def ros_lifespan(app: FastAPI):
    """Run the API's ROS2 node for as long as the app runs.

    All clients and publishers live on this one node. The controllers, the node,
    and the spin thread are exposed to the routes as `request.state.<name>`.
    """
    rclpy.init()
    node = rclpy.create_node("fastapi_ros2")

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    controllers = {
        "add_controller": AddTwoIntsController(node),
        "turtle_controller": TurtleSimController(node),
        "print_controller": PrintController(node),
    }

    try:
        yield {
            "node": node,
            "spin_thread": spin_thread,
            # Shutting rclpy down destroys the node and its clients, after which
            # their names can no longer be read, so /health reads them from here.
            "node_name": node.get_fully_qualified_name(),
            "service_names": [client.service_name for client in node.clients],
            **controllers,
        }
    finally:
        executor.shutdown()
        spin_thread.join()
        node.destroy_node()
        rclpy.try_shutdown()
