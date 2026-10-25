import asyncio

from example_interfaces.srv import AddTwoInts
import pytest

from fastapi_ros2.services.call import (
    ServiceTimeoutError,
    ServiceUnavailableError,
    call_service,
)
from tests.conftest import FakeClient


def call(client: FakeClient, timeout_sec: float = 1.0):
    return asyncio.run(call_service(client, AddTwoInts.Request(), timeout_sec))


def test_returns_the_response():
    client = FakeClient(AddTwoInts, "add_two_ints")
    client.response = AddTwoInts.Response(sum=5)

    assert call(client).sum == 5
    assert client.removed == []


def test_raises_the_call_exception():
    client = FakeClient(AddTwoInts, "add_two_ints")
    client.response = RuntimeError("boom")

    with pytest.raises(RuntimeError, match="boom"):
        call(client)


def test_unavailable_service_is_not_called():
    client = FakeClient(AddTwoInts, "add_two_ints")
    client.ready = False

    with pytest.raises(ServiceUnavailableError):
        call(client)
    assert client.requests == []


def test_timeout_removes_the_pending_request():
    client = FakeClient(AddTwoInts, "add_two_ints")
    client.response = None

    with pytest.raises(ServiceTimeoutError):
        call(client, timeout_sec=0.01)
    assert len(client.removed) == 1


def test_in_flight_calls_are_limited_to_the_qos_depth():
    client = FakeClient(AddTwoInts, "add_two_ints", depth=2)
    client.response = None

    async def burst():
        calls = [call_service(client, AddTwoInts.Request(), 0.05) for _ in range(5)]
        return await asyncio.gather(*calls, return_exceptions=True)

    results = asyncio.run(burst())

    # The other three waited for a free slot until they timed out.
    assert len(client.requests) == 2
    assert all(isinstance(r, ServiceTimeoutError) for r in results)
