import threading
import time
from typing import override

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor

from fastapi_ros2.nodes import Spinner


@pytest.mark.usefixtures("api")  # Initializes rclpy.
def test_stop_shuts_the_executor_down_after_the_running_callback():
    node = rclpy.create_node("test_spinner")
    started, finished = threading.Event(), threading.Event()
    finished_at_shutdown: list[bool] = []

    def slow_callback() -> None:
        started.set()
        time.sleep(0.1)
        finished.set()

    class Executor(SingleThreadedExecutor):
        @override
        def shutdown(self, timeout_sec: float | None = None) -> bool:
            finished_at_shutdown.append(finished.is_set())
            return super().shutdown(timeout_sec)

    timer = node.create_timer(0.001, slow_callback)
    executor = Executor()
    executor.add_node(node)
    spinner = Spinner(executor)
    assert started.wait(2.0)
    timer.cancel()

    spinner.stop()

    # Lyrical's shutdown() destroys the executor without waiting for the callback.
    assert finished_at_shutdown == [True]
    node.destroy_node()
