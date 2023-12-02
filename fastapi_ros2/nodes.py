from typing import List
import threading

import rclpy

from fastapi_ros2.api.v1.controllers.add_two_ints import AddTwoIntsController
from fastapi_ros2.api.v1.controllers.turtlesim import TurtleSimController
from fastapi_ros2.api.v1.controllers.print import PrintController


rclpy.init()

# ROS2 nodes tied to the API defined and added to the executor here
add_controller = AddTwoIntsController()
turtle_controller = TurtleSimController()
print_controller = PrintController()

# Define executor
executor = rclpy.executors.MultiThreadedExecutor()

# Add client nodes
executor.add_node(add_controller.client)
executor.add_node(turtle_controller.client)
executor.add_node(print_controller.publisher)

spin_thread = threading.Thread(target=executor.spin, daemon=True)
spin_thread.start()


def get_nodes() -> List:
    return executor.get_nodes()
