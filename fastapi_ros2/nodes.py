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
    """Run the ROS2 nodes tied to the API for as long as the app runs.

    The controllers are exposed to the routes as `request.state.<name>`.
    """
    rclpy.init()
    controllers = {
        "add_controller": AddTwoIntsController(),
        "turtle_controller": TurtleSimController(),
        "print_controller": PrintController(),
    }
    nodes = [
        controllers["add_controller"].client,
        controllers["turtle_controller"].client,
        controllers["print_controller"].publisher,
    ]

    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        yield controllers
    finally:
        executor.shutdown()
        spin_thread.join()
        for node in nodes:
            node.destroy_node()
        rclpy.try_shutdown()
