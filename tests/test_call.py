import asyncio
import time

import pytest
from example_interfaces.srv import AddTwoInts
from fastapi.testclient import TestClient
from rclpy.client import Client
from rclpy.node import Node

from fastapi_ros2.services.call import (
    ServiceTimeoutError,
    ServiceUnavailableError,
    call_service,
)
from tests.conftest import Servers, ros_state, wait_until

INT64_MIN, INT64_MAX = -(2**63), 2**63 - 1


type AddClient = Client[AddTwoInts.Request, AddTwoInts.Response]

pytestmark = pytest.mark.usefixtures("servers")


@pytest.fixture
def node(api: TestClient) -> Node:
    """The API's node, spinning, with the test servers available."""
    return ros_state(api).node


def ready_client(node: Node, srv_name: str) -> AddClient:
    client = node.create_client(AddTwoInts, srv_name)
    wait_until(client.service_is_ready)
    return client


def call(
    client: AddClient, a: int = 2, b: int = 3, timeout_sec: float = 1.0
) -> AddTwoInts.Response:
    request = AddTwoInts.Request(a=a, b=b)
    return asyncio.run(call_service(client, request, timeout_sec))


def test_returns_the_response(node: Node):
    assert call(ready_client(node, "add_two_ints")).sum == 5


def test_server_wraps_an_overflowing_sum(node: Node):
    # The API rejects these inputs, but other ROS callers can send them.
    client = ready_client(node, "add_two_ints")

    assert call(client, INT64_MAX, 1).sum == INT64_MIN
    assert call(client).sum == 5


def test_unavailable_service(node: Node):
    with pytest.raises(ServiceUnavailableError):
        call(node.create_client(AddTwoInts, "nobody_serves_this"))


def test_timeout_removes_the_pending_request(node: Node, servers: Servers):
    servers.unresponsive_node.create_service(AddTwoInts, "stuck", lambda req, res: res)
    client = ready_client(node, "stuck")

    with pytest.raises(ServiceTimeoutError):
        call(client, timeout_sec=0.05)
    # Otherwise a late response would complete a request nobody awaits.
    assert client._pending_requests == {}  # pyright: ignore[reportPrivateUsage]


def test_a_burst_beyond_the_qos_depth_loses_no_calls(node: Node, servers: Servers):
    def slow_add(
        request: AddTwoInts.Request, response: AddTwoInts.Response
    ) -> AddTwoInts.Response:
        time.sleep(0.002)
        response.sum = request.a + request.b
        return response

    servers.node.create_service(AddTwoInts, "slow_add", slow_add)
    client = ready_client(node, "slow_add")
    calls = 5 * client.qos_profile.depth

    async def burst() -> list[AddTwoInts.Response]:
        requests = [AddTwoInts.Request(a=i, b=1) for i in range(calls)]
        return await asyncio.gather(*(call_service(client, r, 3.0) for r in requests))

    # Without the in-flight limit, requests beyond the depth would be dropped
    # by the server's queue and time out.
    assert [r.sum for r in asyncio.run(burst())] == [i + 1 for i in range(calls)]
