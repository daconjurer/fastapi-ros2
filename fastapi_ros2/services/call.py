import asyncio
import weakref
from typing import Any

from rclpy.client import Client
from rclpy.task import Future
from rclpy.type_support import SrvRequestT, SrvResponseT


class ServiceUnavailableError(Exception):
    pass


class ServiceTimeoutError(Exception):
    pass


# Requests beyond the service QoS history depth are silently dropped by the
# middleware, so each client only gets that many calls in flight; the rest wait.
# ponytail: burst protection for this process only, not added capacity; replicas
# each get their own limit, so raise the server's QoS depth if the API scales out.
_in_flight: weakref.WeakKeyDictionary[Client[Any, Any], asyncio.Semaphore] = (
    weakref.WeakKeyDictionary()
)


async def call_service(
    client: Client[SrvRequestT, SrvResponseT],
    request: SrvRequestT,
    timeout_sec: float = 5.0,
) -> SrvResponseT:
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
    except TimeoutError:
        raise ServiceTimeoutError(
            f'Service "{client.service_name}" did not respond within {timeout_sec}s'
        ) from None


async def _call(
    client: Client[SrvRequestT, SrvResponseT], request: SrvRequestT
) -> SrvResponseT:
    if client not in _in_flight:
        _in_flight[client] = asyncio.Semaphore(client.qos_profile.depth)

    async with _in_flight[client]:
        loop = asyncio.get_running_loop()
        result: asyncio.Future[SrvResponseT] = loop.create_future()

        ros_future = client.call_async(request)
        ros_future.add_done_callback(
            lambda f: loop.call_soon_threadsafe(_transfer, f, result)
        )
        try:
            return await result
        finally:
            if not ros_future.done():
                client.remove_pending_request(ros_future)


def _transfer(
    ros_future: Future[SrvResponseT], result: asyncio.Future[SrvResponseT]
) -> None:
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
