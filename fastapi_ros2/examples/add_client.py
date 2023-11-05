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


class SumClient(Node):
    def __init__(self):
        super().__init__('int_sum_service')
        self._client = AddTwoIntsClientAsync()

        @app.get('/services/sum', response_model=SumResponse)
        async def sum_service_client(a: int, b: int):
            response = SumResponse(sum=-1)

            self.get_logger().info(
                f'Sending request to service "{self._client.name}" with params {a} to {b}'
            )
            server_response = self._client.send_request(a, b)
            result_sum = int(server_response.sum)

            self.get_logger().info(
                f'Response from service "{self._client.name}" '
                f'to request with params {a} to {b}: {result_sum}'
            )
            response.sum = result_sum
            return response


def main(args=None):
    rclpy.init()
    sum_server = SumClient()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(sum_server)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    uvicorn.run(app, port=5000, log_level='warning')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
