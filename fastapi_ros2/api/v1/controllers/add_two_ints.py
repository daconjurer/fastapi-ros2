from loguru import logger

from fastapi_ros2.services.add_two_ints.client import AddTwoIntsClientAsync


class AddTwoIntsController:
    def __init__(self):
        self.client = AddTwoIntsClientAsync()

    async def add_two_ints(self, a: int, b: int) -> int:
        logger.info(
            f'Sending request to service "{self.client.name}" with params {a} and {b}'
        )
        server_response = self.client.send_request(a, b)
        result_sum = int(server_response.sum)

        logger.info(
            f'Response from service "{self.client.name}" '
            f"to request with params {a} and {b}: {result_sum}"
        )
        return result_sum
