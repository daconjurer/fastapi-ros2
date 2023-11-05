import threading

import rclpy
from rclpy.node import Node

import uvicorn
from fastapi import FastAPI
from pydantic import BaseModel

from fastapi_ros2.services.add_two_ints.client import AddTwoIntsClientAsync


app = FastAPI()


class SumResponse(BaseModel):
    sum: int


class SumServer(Node):
    def __init__(self):
        super().__init__('int_sum_service')
        self._client = AddTwoIntsClientAsync()

        @app.get('/services/sum', response_model=SumResponse)
        async def sum_service(a: int, b: int):
            response = SumResponse(sum=-1)
            server_response = self._client.send_request(a, b)
            response.sum = int(server_response.sum)
            return response


def main(args=None):
    rclpy.init()
    sum_server = SumServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(sum_server)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    uvicorn.run(app, port=5000, log_level='warning')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
