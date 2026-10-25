import asyncio
import weakref
from typing import Any

from rclpy.client import Client
from rclpy.task import Future


class ServiceUnavailableError(Exception):
    pass


class ServiceTimeoutError(Exception):
    pass


# A service request or response is an instance of the generated `<Srv>.Request` or
# `<Srv>.Response` class (e.g. `AddTwoInts.Request`). Generated classes share no
# base class and Jazzy's `Client` is not generic, so there is no tighter type.
ServiceRequest = Any
ServiceResponse = Any

# Requests beyond the service QoS history depth are silently dropped by the
# middleware, so each client only gets that many calls in flight; the rest wait.
# ponytail: burst protection for this process only, not added capacity; replicas
# each get their own limit, so raise the server's QoS depth if the API scales out.
_in_flight: weakref.WeakKeyDictionary[Client, asyncio.Semaphore] = (
    weakref.WeakKeyDictionary()
)


async def call_service(
    client: Client,
    request: ServiceRequest,
    timeout_sec: float = 5.0,
) -> ServiceResponse:
    """Await a ROS service call from asyncio, without blocking the event loop.

    The response arrives on the executor thread that spins the client's node.
    The timeout includes time spent waiting for an in-flight slot.
    """
    if not client.service_is_ready():
        raise ServiceUnavailableError(
            f'Service "{client.service_name}" is not available'
        )

    try:
        return await asyncio.wait_for(_call(client, request), timeout_sec)
    except asyncio.TimeoutError:  # Not an alias of TimeoutError before Python 3.11.
        raise ServiceTimeoutError(
            f'Service "{client.service_name}" did not respond within {timeout_sec}s'
        ) from None


async def _call(client: Client, request: ServiceRequest) -> ServiceResponse:
    if client not in _in_flight:
        _in_flight[client] = asyncio.Semaphore(client.qos_profile.depth)

    async with _in_flight[client]:
        loop = asyncio.get_running_loop()
        result: asyncio.Future[ServiceResponse] = loop.create_future()

        ros_future = client.call_async(request)
        ros_future.add_done_callback(
            lambda f: loop.call_soon_threadsafe(_transfer, f, result)
        )
        try:
            return await result
        finally:
            if not ros_future.done():
                client.remove_pending_request(ros_future)


def _transfer(ros_future: Future, result: asyncio.Future[ServiceResponse]) -> None:
    """Copy a completed rclpy future's outcome into an asyncio future.

    Runs on the event loop thread, scheduled from the executor thread with
    `loop.call_soon_threadsafe`, because asyncio futures are not thread-safe.
    Does nothing if `result` is already done, which happens when the call
    timed out and `asyncio.wait_for` cancelled it before the response arrived.
    """
    if result.done():
        return
    if ros_future.exception() is not None:
        result.set_exception(ros_future.exception())
    else:
        result.set_result(ros_future.result())
