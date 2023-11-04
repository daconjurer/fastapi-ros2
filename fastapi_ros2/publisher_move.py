import threading

import rclpy
from rclpy.node import Node

import uvicorn
from fastapi import FastAPI
from pydantic import BaseModel

from fastapi_ros2.minimal_client import MinimalClientAsync

app = FastAPI()


class Response(BaseModel):
    msg: int


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self._client = MinimalClientAsync()

        @app.get('/publish', response_model=Response)
        async def publish():
            response = {"msg": -1}
            server_response = self._client.send_request(1, 2)
            response["msg"] = int(server_response.sum)
            return response


def main(args=None):
    rclpy.init()
    minimal_publisher = MinimalPublisher()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(minimal_publisher)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    uvicorn.run(app, port=5000, log_level='warning')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
